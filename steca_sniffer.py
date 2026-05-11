#!/usr/bin/env python3
"""
steca_sniffer.py - Passive RS485 bus monitor for StecaGrid inverters.

Assembles complete Steca frames from a raw byte stream and decodes them,
logging every frame as JSON for later CRC2 analysis.

Frame layout:
  [0]     0x02  start byte
  [1]     ?     (always 0x01 in observed traffic)
  [2:4]   uint16 big-endian total frame length (includes start/end bytes)
  [4]     TO    destination address
  [5]     FROM  source address  (0x7b = SEM/controller, 0x01 = inverter)
  [6]     CRC1  crc over bytes [0:6], poly=0x139 init=0xAA refin=True refout=True
  [7:-3]  payload
  [-3:-1] CRC2  (algorithm unknown, logged for analysis)
  [-1]    0x03  end byte
"""

import sys
import json
import struct
import argparse
import datetime

import serial

try:
    import crcmod
    _crc1_fn = crcmod.mkCrcFun(0x139, initCrc=0xAA, rev=True, xorOut=0x00)
    CRC_AVAILABLE = True
except ImportError:
    _crc1_fn = None
    CRC_AVAILABLE = False

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD_RATE    = 38400
LOG_FILE     = "steca_sniffer.log"

# ── topic byte (frame[11] = payload[4]) ──────────────────────────────────────
TOPIC_NAMES = {
    0x05: "Time",
    0x08: "Mystery One",
    0x09: "Serial Number",
    0x1d: "Nominal Power",
    0x22: "Panel Power",
    0x23: "Panel Voltage",
    0x24: "Panel Current",
    0x29: "AC Power",
    0x3c: "Daily Yield",
    0x51: "ENS",
    0xf1: "Total Yield",
}

# payload_type byte (frame[7])
PAYLOAD_TYPE_NAMES = {
    0x20: "VersionReq",
    0x21: "VersionResp",
    0x40: "RequestA",
    0x41: "ResponseA",
    0x64: "RequestB",
    0x65: "ResponseB",
}


# ── value decoders (verbatim from getStecaGridData.py) ───────────────────────

def decode_stecaFloat_a(ac_bytes):
    unit_map = {0x0B: "W", 0x07: "A", 0x05: "V", 0x0D: "Hz", 0x09: "Wh", 0x00: "NUL"}
    unit = unit_map.get(ac_bytes[0], f"0x{ac_bytes[0]:02x}")
    iacpower = ((ac_bytes[3] << 8 | ac_bytes[1]) << 8 | ac_bytes[2]) << 7
    facpower, = struct.unpack('f', struct.pack('I', iacpower))
    return [facpower, unit]


def decode_TotalYield_a(ba):
    bits = ba[3] << 24 | ba[2] << 16 | ba[1] << 8 | ba[0]
    ieee, = struct.unpack('f', struct.pack('I', bits))
    return [ieee, "Wh"]


# ── helpers ───────────────────────────────────────────────────────────────────

def fmt_hex(b):
    return ' '.join(f'{x:02x}' for x in b)


def check_crc1(frame):
    """Return (computed_byte, ok_bool) or (None, None) if crcmod missing."""
    if not CRC_AVAILABLE:
        return None, None
    computed = _crc1_fn(bytes(frame[0:6]))
    return computed, (computed == frame[6])


def topic_name(byte_val):
    return TOPIC_NAMES.get(byte_val, f"0x{byte_val:02x}")


def payload_type_name(byte_val):
    return PAYLOAD_TYPE_NAMES.get(byte_val, f"0x{byte_val:02x}")


# ── frame decoder ─────────────────────────────────────────────────────────────

def decode_frame(frame):
    """
    Return an info dict for one validated frame.
    'decoded' is a human-readable string or None.
    """
    total_len  = (frame[2] << 8) | frame[3]
    to_addr    = frame[4]
    from_addr  = frame[5]
    crc1_byte  = frame[6]

    if from_addr == 0x7b:
        direction = "REQUEST"
    elif from_addr == 0x01:
        direction = "RESPONSE"
    else:
        direction = f"DIR(from=0x{from_addr:02x})"

    crc1_computed, crc1_ok = check_crc1(frame)

    crc2_word = (frame[-3] << 8) | frame[-2]

    payload     = frame[7:-3]
    payload_hex = fmt_hex(payload)

    ptype      = frame[7]  if len(frame) > 7  else None
    # topic byte is payload[4] = frame[11]; only meaningful when payload >= 5 bytes
    topic_byte = frame[11] if len(payload) >= 5 else None

    ptype_str = payload_type_name(ptype) if ptype is not None else "?"
    topic_str = topic_name(topic_byte)   if topic_byte is not None else "?"

    decoded = _try_decode(frame, ptype, topic_byte)

    return {
        "direction":       direction,
        "to":              to_addr,
        "from":            from_addr,
        "len":             total_len,
        "crc1":            crc1_byte,
        "crc1_computed":   crc1_computed,
        "crc1_ok":         crc1_ok,
        "crc2":            crc2_word,
        "payload_hex":     payload_hex,
        "ptype":           ptype,
        "ptype_str":       ptype_str,
        "topic_byte":      topic_byte,
        "topic_str":       topic_str,
        "decoded":         decoded,
    }


def _try_decode(frame, ptype, topic_byte):
    """Best-effort value decode; returns a string or None."""
    try:
        if ptype == 0x41 and frame[8] == 0x00:          # ResponseA
            if topic_byte == 0x3c:                       # Daily Yield
                if len(frame) >= 17:
                    v = decode_stecaFloat_a(frame[12:17])
                    return f"Daily Yield = {v[0]:.4f} {v[1]}"
            elif topic_byte is not None and topic_byte != 0x51:
                if len(frame) >= 16:
                    label_len   = frame[14]
                    label_start = 15
                    label_end   = label_start + label_len
                    val_end     = label_end + 5
                    if len(frame) >= val_end + 3:
                        label = frame[label_start:label_end].decode("ascii", errors="replace")
                        v     = decode_stecaFloat_a(frame[label_end:val_end])
                        return f"{label} = {v[0]:.4f} {v[1]}"

        elif ptype == 0x65:                              # ResponseB
            if topic_byte == 0xf1:                       # Total Yield
                if len(frame) >= 17:
                    v = decode_TotalYield_a(frame[12:16])
                    return f"Total Yield = {v[0]:.4f} {v[1]}"
            elif topic_byte == 0x05:                     # Time
                if len(frame) >= 19:
                    t  = frame
                    dt = datetime.datetime(
                        2000 + t[12], t[13], t[14], t[15], t[16], t[17]
                    )
                    return f"Time = {dt}"
            elif topic_byte == 0x09:                     # Serial
                if len(frame) > 15:
                    return f"Serial = {frame[12:-4].decode('ascii', errors='replace')}"

    except Exception:
        pass
    return None


# ── output ────────────────────────────────────────────────────────────────────

def print_frame(frame, info, verbose):
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    crc1_tag = ""
    if info["crc1_ok"] is True:
        crc1_tag = " [OK]"
    elif info["crc1_ok"] is False:
        crc1_tag = f" [FAIL expected=0x{info['crc1_computed']:02x}]"
    else:
        crc1_tag = " [crcmod unavailable]"

    topic_part = ""
    if info["topic_byte"] is not None:
        topic_part = f"  TOPIC=0x{info['topic_byte']:02x} ({info['topic_str']})"

    print(f"\n[{ts}] {info['direction']}  "
          f"TO=0x{info['to']:02x}  FROM=0x{info['from']:02x}  LEN={info['len']}")
    print(f"  RAW    : {fmt_hex(frame)}")
    print(f"  CRC1   : 0x{info['crc1']:02x}{crc1_tag}")
    print(f"  CRC2   : 0x{info['crc2']:04x}")
    print(f"  PTYPE  : 0x{info['ptype']:02x} ({info['ptype_str']}){topic_part}")
    print(f"  PAYLOAD: {info['payload_hex']}")
    if info["decoded"]:
        print(f"  DECODED: {info['decoded']}")


def log_frame(logfile, info):
    ts = datetime.datetime.now().isoformat()
    entry = {
        "ts":          ts,
        "direction":   info["direction"],
        "topic":       info["topic_str"],
        "crc1_ok":     info["crc1_ok"],
        "crc1":        f"0x{info['crc1']:02x}",
        "crc2":        f"0x{info['crc2']:04x}",
        "payload_hex": info["payload_hex"],
    }
    logfile.write(json.dumps(entry) + "\n")
    logfile.flush()


# ── frame assembler ───────────────────────────────────────────────────────────

def assemble_frames(buf):
    """
    Yield complete, validated frames from buf (bytearray), consuming them.
    Bytes preceding the first valid frame start are discarded.
    """
    while True:
        start = buf.find(0x02)
        if start == -1:
            buf.clear()
            return
        if start > 0:
            del buf[:start]

        # Need at least 4 bytes to read the length field
        if len(buf) < 4:
            return

        frame_len = (buf[2] << 8) | buf[3]

        # Sanity-check the length before waiting for bytes
        if frame_len < 7 or frame_len > 512:
            del buf[0]          # discard this 0x02 and keep scanning
            continue

        if len(buf) < frame_len:
            return              # wait for more data

        if buf[frame_len - 1] != 0x03:
            del buf[0]          # end byte mismatch; this 0x02 is not a frame start
            continue

        frame = bytes(buf[:frame_len])
        del buf[:frame_len]
        yield frame


# ── main loop ─────────────────────────────────────────────────────────────────

def run_sniffer(port_name, verbose):
    if not CRC_AVAILABLE:
        print("Warning: crcmod not installed – CRC1 verification disabled.")
        print("  pip install crcmod\n")

    print(f"Opening {port_name} at {BAUD_RATE} baud …")
    ser = serial.Serial(
        port=port_name,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )
    print(f"Logging frames to {LOG_FILE}")
    print("Monitoring RS485 bus – press Ctrl+C to stop.\n")

    buf         = bytearray()
    frame_count = 0

    with open(LOG_FILE, "a") as logfile:
        try:
            while True:
                chunk = ser.read(256)
                if not chunk:
                    continue
                buf.extend(chunk)

                if verbose and chunk:
                    print(f"  [rx {len(chunk)} bytes: {fmt_hex(chunk[:16])}"
                          f"{'…' if len(chunk) > 16 else ''}]")

                for frame in assemble_frames(buf):
                    frame_count += 1
                    info = decode_frame(frame)
                    print_frame(frame, info, verbose)
                    log_frame(logfile, info)

        except KeyboardInterrupt:
            print(f"\nStopped. {frame_count} frame(s) captured.")
        finally:
            ser.close()


def main():
    parser = argparse.ArgumentParser(
        description="Passive RS485 sniffer for StecaGrid inverters"
    )
    parser.add_argument(
        "--port", "-p", default=DEFAULT_PORT,
        help=f"Serial port to listen on (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Print raw RX bytes and extra detail",
    )
    args = parser.parse_args()
    run_sniffer(args.port, args.verbose)


if __name__ == "__main__":
    main()
