#!/usr/bin/env python3
"""
steca_mqtt.py — MQTT bridge for StecaGrid 3600 RS485.

Polls one or more inverters on the same RS485 bus, publishes live metrics per
inverter, and accepts per-inverter power-limit setpoints via MQTT.

Config format
-------------
  inverters:          # list of inverter sections
    - id: 1           # RS485 address (default 1)
      name: ...       # friendly name
      topic: ...      # MQTT base topic for this inverter
      values_of_interest: [...]   # subset of METRICS to publish

Backward-compatible: if no  inverters:  key is present the old flat
  topic / values_of_interest  keys are used with id=1.

Power-limit flow (per inverter)
--------------------------------
  Subscribe : <inv_topic>/setpoint/power_limit_percent   (float 0.0–100.0)
  State     : <inv_topic>/setpoint/power_limit_percent/state
  Metrics   : <inv_topic>/<METRIC_NAME>

  < 100 %: frame re-sent every poll cycle (inverter resets after ~1 min).
  = 100 %: sent once; repeated until the inverter ACKs Ok, then silent.

Home Assistant MQTT autodiscovery
----------------------------------
Enable with  ha_discovery: true.  One HA device per inverter is created
under  <ha_discovery_prefix>/sensor/  and  /number/  (retain=True).
"""

import argparse
import json
import re
import ssl
import threading
import time

import serial           # pip install pyserial
import yaml             # pip install pyyaml
import paho.mqtt.client as mqtt  # pip install paho-mqtt

from StecaGridController import (
    SERIAL_BAUDRATE, SERIAL_BYTES, SERIAL_PARITY, SERIAL_SBIT,
    TOPICS,
    build_request,
    getStecaGridResult,
    read_complete_frame,
    process_steca485,
)
from steca_setpoint import build_setpoint_percent

_DEFAULT_CONFIG = "config.yaml"

# ── Metric definitions ────────────────────────────────────────────────────────
# name → (topic_key in TOPICS dict, result-extractor)
# extractor(val) → (value: float|str, unit: str)
def _float_unit(val):
    if isinstance(val, list) and len(val) >= 2:
        return val[0], val[1]
    return None, None

METRICS = {
    "CURRENT_ELECTRICITY_DELIVERY": ("ac_power",    _float_unit),
    "ELECTRICITY_EXPORTED_TOTAL":   ("total_yield",  _float_unit),
    "CURRENT_DAILY_YIELD":          ("daily_yield",  _float_unit),
    "CURRENT_PANEL_POWER":          ("panel_power",  _float_unit),
    "CURRENT_PANEL_VOLTAGE":        ("panel_voltage",_float_unit),
    "CURRENT_PANEL_CURRENT":        ("panel_current",_float_unit),
    "SG_SERIAL": ("serial", lambda v: (v[0] if isinstance(v, list) else v, "")),
}

# ── Home Assistant autodiscovery metadata ─────────────────────────────────────
# name → (friendly_name, device_class, unit_of_measurement, state_class)
_HA_SENSOR_META = {
    "CURRENT_ELECTRICITY_DELIVERY": ("AC Power",      "power",   "W",  "measurement"),
    "ELECTRICITY_EXPORTED_TOTAL":   ("Total Yield",   "energy",  "Wh", "total_increasing"),
    "CURRENT_DAILY_YIELD":          ("Daily Yield",   "energy",  "Wh", "total_increasing"),
    "CURRENT_PANEL_POWER":          ("Panel Power",   "power",   "W",  "measurement"),
    "CURRENT_PANEL_VOLTAGE":        ("Panel Voltage", "voltage", "V",  "measurement"),
    "CURRENT_PANEL_CURRENT":        ("Panel Current", "current", "A",  "measurement"),
    "SG_SERIAL":                    ("Serial Number", None,      None, None),
}


def _node_id(topic: str) -> str:
    """Derive a safe HA node_id / unique_id prefix from an MQTT topic."""
    return re.sub(r"[^a-zA-Z0-9_-]", "_", topic)


# ── Per-inverter runtime state ────────────────────────────────────────────────
class _InvState:
    def __init__(self, inv_id: int, name: str, topic: str, values: list[str]):
        self.inv_id = inv_id
        self.name   = name
        self.topic  = topic
        self.values = values
        # Setpoint (protected by lock)
        self.limit_pct: float | None = None
        self.limit_confirmed         = False
        self.lock                    = threading.Lock()

    @property
    def setpoint_cmd_topic(self) -> str:
        return f"{self.topic}/setpoint/power_limit_percent"

    @property
    def setpoint_state_topic(self) -> str:
        return f"{self.topic}/setpoint/power_limit_percent/state"


def _load_inverters(cfg: dict) -> list[_InvState]:
    """Parse config into a list of _InvState; supports new and legacy formats."""
    if "inverters" in cfg:
        result = []
        for entry in cfg["inverters"]:
            inv_id = int(entry.get("id", 1))
            result.append(_InvState(
                inv_id = inv_id,
                name   = str(entry.get("name", f"StecaGrid #{inv_id}")),
                topic  = str(entry["topic"]),
                values = list(entry.get("values_of_interest", list(METRICS))),
            ))
        return result

    # Legacy flat format — single inverter, id=1
    return [_InvState(
        inv_id = 1,
        name   = cfg.get("ha_device_name", "StecaGrid 3600"),
        topic  = cfg["topic"],
        values = list(cfg.get("values_of_interest", list(METRICS))),
    )]


# ── Service ───────────────────────────────────────────────────────────────────
class StecaMqttService:
    def __init__(self, config: dict, verbose: bool = False):
        self._cfg      = config
        self._verbose  = verbose
        self._inverters: list[_InvState] = _load_inverters(config)

        # Map setpoint command topic → _InvState for fast lookup in on_message
        self._setpoint_map: dict[str, _InvState] = {
            inv.setpoint_cmd_topic: inv for inv in self._inverters
        }

        self._port: serial.Serial | None = None

        self._mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        if "mqtt_username" in config:
            self._mqtt.username_pw_set(config["mqtt_username"],
                                        password=config.get("mqtt_password", ""))
        self._mqtt.reconnect_delay_set(min_delay=1, max_delay=30)
        self._mqtt.on_connect    = self._on_connect
        self._mqtt.on_message    = self._on_message
        self._mqtt.on_disconnect = self._on_disconnect

    # ── MQTT callbacks ────────────────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code != 0:
            print(f"MQTT connect failed: reason={reason_code}")
            return

        for inv in self._inverters:
            client.subscribe(inv.setpoint_cmd_topic, qos=1)
            if self._verbose:
                print(f"MQTT [{inv.name}] subscribed to {inv.setpoint_cmd_topic}")

        if self._cfg.get("ha_discovery", False):
            self._publish_ha_discovery()

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        if self._verbose:
            print(f"MQTT disconnected (reason={reason_code}), will auto-reconnect")

    def _on_message(self, client, userdata, message):
        inv = self._setpoint_map.get(message.topic)
        if inv is None:
            return
        try:
            pct = float(message.payload.decode("utf-8").strip())
            if not 0.0 <= pct <= 100.0:
                print(f"[{inv.name}] setpoint out of range: {pct}")
                return
            with inv.lock:
                inv.limit_pct       = pct
                inv.limit_confirmed = False
            self._mqtt.publish(inv.setpoint_state_topic,
                               payload=pct, qos=0, retain=True)
            if self._verbose:
                print(f"[{inv.name}] setpoint received: {pct:.1f} %")
        except ValueError:
            print(f"[{inv.name}] setpoint invalid payload: {message.payload!r}")

    # ── Home Assistant autodiscovery ──────────────────────────────────────────
    def _publish_ha_discovery(self):
        prefix = self._cfg.get("ha_discovery_prefix", "homeassistant")

        for inv in self._inverters:
            node   = _node_id(inv.topic)
            device = {
                "identifiers": [node],
                "name":         inv.name,
                "model":        "StecaGrid 3600",
                "manufacturer": "Steca",
            }

            for metric in inv.values:
                if metric not in _HA_SENSOR_META:
                    continue
                fname, dclass, unit, sclass = _HA_SENSOR_META[metric]
                cfg: dict = {
                    "name":        fname,
                    "state_topic": f"{inv.topic}/{metric}",
                    "unique_id":   f"{node}_{metric}",
                    "device":      device,
                }
                if dclass:
                    cfg["device_class"] = dclass
                if unit:
                    cfg["unit_of_measurement"] = unit
                if sclass:
                    cfg["state_class"] = sclass

                disc = f"{prefix}/sensor/{node}/{metric}/config"
                self._mqtt.publish(disc, payload=json.dumps(cfg), qos=1, retain=True)
                if self._verbose:
                    print(f"HA discovery → {disc}")

            # Number entity for the power-limit setpoint
            num_cfg = {
                "name":                "Power Limit",
                "command_topic":       inv.setpoint_cmd_topic,
                "state_topic":         inv.setpoint_state_topic,
                "min":                 0,
                "max":                 100,
                "step":                1,
                "unit_of_measurement": "%",
                "icon":                "mdi:solar-power",
                "unique_id":           f"{node}_power_limit_percent",
                "device":              device,
                "optimistic":          False,
            }
            disc = f"{prefix}/number/{node}/power_limit_percent/config"
            self._mqtt.publish(disc, payload=json.dumps(num_cfg), qos=1, retain=True)
            if self._verbose:
                print(f"HA discovery → {disc}")

    # ── Serial helpers ────────────────────────────────────────────────────────
    def _open_port(self) -> serial.Serial:
        dev = self._cfg.get("serial_device", "/dev/ttyS0")
        return serial.Serial(
            port=dev, baudrate=SERIAL_BAUDRATE, bytesize=SERIAL_BYTES,
            parity=SERIAL_PARITY, stopbits=SERIAL_SBIT,
            timeout=1, xonxoff=0, rtscts=0,
        )

    def _query(self, inv_id: int, topic_key: str):
        topic_byte, cmd_byte = TOPICS[topic_key]
        req = build_request(inv_id, topic_byte, cmd_byte)
        return getStecaGridResult(self._port, req)

    # ── Setpoint logic ────────────────────────────────────────────────────────
    def _send_setpoint(self, inv: _InvState, pct: float) -> bool:
        """Send a setpoint frame addressed to inv; return True on ACK Ok."""
        frame = build_setpoint_percent(pct, to=inv.inv_id)
        self._port.reset_input_buffer()
        self._port.write(frame)
        ack = read_complete_frame(self._port, timeout_s=0.5)
        if ack is None:
            if self._verbose:
                print(f"[{inv.name}] setpoint {pct:.1f} %: no ACK")
            return False
        result = process_steca485(ack)
        if result and len(result) >= 6 and isinstance(result[5], tuple):
            status, sname = result[5]
            ok = (status == 0)
            if self._verbose:
                print(f"[{inv.name}] setpoint {pct:.1f} % ({round(pct*10):d} ‰):"
                      f" ACK {sname} {'✓' if ok else '✗'}")
            return ok
        if self._verbose:
            print(f"[{inv.name}] setpoint {pct:.1f} %: ACK parse failed")
        return False

    def _handle_setpoint(self, inv: _InvState):
        with inv.lock:
            pct       = inv.limit_pct
            confirmed = inv.limit_confirmed

        if pct is None:
            return
        if pct >= 100.0 and confirmed:
            return  # 100 % already ACKed, don't repeat

        ok = self._send_setpoint(inv, pct)
        if ok and pct >= 100.0:
            with inv.lock:
                inv.limit_confirmed = True

    # ── MQTT connect loop ─────────────────────────────────────────────────────
    def _connect_mqtt(self):
        mqtt_default_port = 1883
        if self._cfg.get("mqtt_tls", False) or "mqtt_tls_ca" in self._cfg:
            mqtt_default_port = 8883
            if "mqtt_tls_ca" in self._cfg:
                self._mqtt.tls_set(self._cfg["mqtt_tls_ca"])
            else:
                ssl_context = ssl.create_default_context(purpose=ssl.Purpose.SERVER_AUTH)
                self._mqtt.tls_set_context(ssl_context)
            self._mqtt.tls_insecure_set(False)
        mqtt_broker_port = self._cfg.get("mqtt_broker_port", mqtt_default_port)

        while True:
            try:
                self._mqtt.connect(self._cfg["mqtt_broker_address"], mqtt_broker_port)
                break
            except Exception as e:
                print(f"Can't connect to MQTT broker "
                      f"({self._cfg['mqtt_broker_address']}): {e}")
                time.sleep(5)
        self._mqtt.loop_start()

    # ── Main poll loop ────────────────────────────────────────────────────────
    def run(self):
        self._port = self._open_port()
        if self._verbose:
            names = ", ".join(f"{inv.name} (id={inv.inv_id})"
                              for inv in self._inverters)
            print(f"Serial port opened: {self._port.port}  |  inverters: {names}")

        self._connect_mqtt()

        poll_interval = float(self._cfg.get("poll_interval_s", 5))

        try:
            while True:
                for inv in self._inverters:
                    if self._verbose and len(self._inverters) > 1:
                        print(f"\n── {inv.name} (id={inv.inv_id}) ──")

                    for name in inv.values:
                        if name not in METRICS:
                            if self._verbose:
                                print(f"[{inv.name}] unknown metric: {name}")
                            continue

                        topic_key, extractor = METRICS[name]
                        val = self._query(inv.inv_id, topic_key)

                        if self._verbose:
                            print(f"[{inv.name}] {name}: raw={val!r}")

                        if val is None:
                            continue

                        fval, unit = extractor(val)
                        if fval is None:
                            continue

                        if name == "ELECTRICITY_EXPORTED_TOTAL" and fval == 0:
                            continue

                        try:
                            payload    = fval if isinstance(fval, str) else float(fval)
                            mqtt_topic = f"{inv.topic}/{name}"
                            pub = self._mqtt.publish(mqtt_topic, payload=payload, qos=0)
                            pub.wait_for_publish()
                            if self._verbose:
                                print(f"MQTT ← {mqtt_topic}: {payload} {unit}".rstrip())
                        except Exception as e:
                            print(f"[{inv.name}] MQTT publish failed for {name}: {e}")
                            while not self._mqtt.is_connected():
                                print("MQTT: waiting for reconnect …")
                                time.sleep(5)

                    self._handle_setpoint(inv)

                time.sleep(poll_interval)

        except KeyboardInterrupt:
            print()
        finally:
            self._mqtt.loop_stop()
            self._port.close()


# ── CLI entry point ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="MQTT bridge for StecaGrid 3600 RS485"
    )
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable verbose output")
    parser.add_argument("-c", "--config", default=_DEFAULT_CONFIG,
                        help=f"Config file (default: {_DEFAULT_CONFIG})")
    args = parser.parse_args()

    with open(args.config) as fh:
        cfg = yaml.safe_load(fh)

    if not cfg:
        raise SystemExit(f"Empty or invalid config: {args.config}")

    svc = StecaMqttService(cfg, verbose=args.verbose)
    svc.run()
