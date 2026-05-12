#!/usr/bin/env python3
"""
steca_sniffer.py — Passive RS485 bus sniffer for StecaGrid 3600
Threaded UART reader + CRC verification (nibble-table) + event log decoder

CRC1: CRC-8 nibble-table, init=0x55, covers frame[0:6]
CRC2: CRC-16 nibble-table, init=0x5555, covers frame[:-3] + ETX — all frame types ✓

Usage:
  python3 steca_sniffer.py --port /dev/ttyUSB0
  python3 steca_sniffer.py --port /dev/ttyUSB0 --verbose

Install: pip3 install pyserial
"""

import serial
import argparse
import struct
import datetime
import json
import sys
import threading
import queue
import time

from steca_crc import crc1 as _crc1, crc2 as _crc2

SERIAL_BAUDRATE = 38400
SERIAL_TIMEOUT  = 0.02   # short — thread reads continuously
LOG_FILE        = "steca_sniffer.log"
DEBUG_DROPS     = False

SEM_IDS = {0x7b: "SEM-7b", 0xc9: "StecaUser-4.4"}
ID_INVERTER = 0x01

TOPIC_NAMES = {
    0x05: "Time",         0x08: "Mystery_08",   0x09: "Serial",
    0x1d: "NominalPower", 0x22: "PanelPower",   0x23: "PanelVoltage",
    0x24: "PanelCurrent", 0x29: "ACPower",       0x32: "Topic_32",
    0x3c: "DailyYield",   0x51: "GridMeas",      0x52: "ENS_52",
    0x53: "ENS_53",       0x5a: "EventLog_p1",   0x5b: "EventLog_p2",
    0xf1: "TotalYield",
}

# ── CRC helpers ───────────────────────────────────────────────────────────────
def calc_crc1(frame: bytes) -> int:
    return _crc1(frame)

def verify_crc2(frame: bytes):
    """Verify CRC2 for any frame type using the nibble-table CRC-16."""
    expected = _crc2(frame[:-3])
    return expected, "nibble_crc16"

# ── Value decoders ────────────────────────────────────────────────────────────
def _steca_float(b):
    units = {0x0B:"W", 0x07:"A", 0x05:"V", 0x0D:"Hz", 0x09:"Wh", 0x00:"NUL"}
    u = units.get(b[0], f'0x{b[0]:02x}')
    i = (((b[3] << 8 | b[1]) << 8 | b[2]) << 7) & 0xFFFFFFFF
    v, = struct.unpack('f', struct.pack('I', i))
    return v, u

def _total_yield(b):
    bits = b[3]<<24 | b[2]<<16 | b[1]<<8 | b[0]
    v, = struct.unpack('f', struct.pack('I', bits))
    return v, "Wh"

def _try_ts(buf, pos):
    if pos + 6 > len(buf): return None
    b = buf[pos:pos+6]
    try:
        if 0x0d<=b[0]<=0x1a and 1<=b[1]<=12 and 1<=b[2]<=31 and b[3]<=23 and b[4]<=59 and b[5]<=59:
            return datetime.datetime(2000+b[0], b[1], b[2], b[3], b[4], b[5])
    except: pass
    return None

def decode_event_log(payload: bytes):
    """Decode event log response (cmd=0x69, topic 0x5a or 0x5b)."""
    if len(payload) < 7 or payload[0] != 0x69: return 0, []
    data  = payload[6:]
    total = data[0]
    # Collect timestamps, deduplicate overlaps
    raw_ts = [(p, t) for p in range(len(data)-5) if (t := _try_ts(data, p))]
    ts_dedup = []
    for pos, t in raw_ts:
        if ts_dedup and pos < ts_dedup[-1][0] + 6: continue
        ts_dedup.append((pos, t))
    # Collect null-terminated ASCII strings (len >= 4, starts with letter)
    msgs, pos = [], 0
    while pos < len(data):
        if 65 <= data[pos] <= 122:
            end = pos
            while end < len(data) and 32 <= data[end] <= 126: end += 1
            if end - pos >= 4 and end < len(data) and data[end] == 0x00:
                msgs.append((pos, data[pos:end].decode('ascii', errors='replace')))
                pos = end + 1; continue
        pos += 1
    # Pair each message with nearest preceding timestamp
    events = []
    for msg_pos, msg in msgs:
        ts = None
        for ts_pos, t in reversed(ts_dedup):
            if ts_pos < msg_pos: ts = t; break
        events.append((ts, msg))
    return total, events

def fmt_hex(b: bytes) -> str:
    return ' '.join(f'{x:02x}' for x in b)

# ── Frame assembler ────────────────────────────────────────────────────────────
class FrameAssembler:
    def __init__(self): self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf.extend(data)
        frames = []
        while True:
            stx = self._buf.find(0x02)
            if stx == -1: self._buf.clear(); break
            if stx > 0:
                if DEBUG_DROPS:
                    print(f"  [assembler skip {stx}B: {fmt_hex(bytes(self._buf[:min(stx,8)]))}]")
                self._buf = self._buf[stx:]
            if len(self._buf) < 4: break
            tlen = (self._buf[2] << 8) | self._buf[3]
            if tlen < 6 or tlen > 4096:
                if DEBUG_DROPS: print(f"  [assembler reject len={tlen}]")
                self._buf = self._buf[1:]; continue
            if len(self._buf) < tlen: break
            cand = bytes(self._buf[:tlen])
            if cand[-1] != 0x03: self._buf = self._buf[1:]; continue
            frames.append(cand)
            self._buf = self._buf[tlen:]
        return frames

# ── Frame decoder ─────────────────────────────────────────────────────────────
def decode_frame(frame: bytes, verbose: bool) -> dict:
    length  = (frame[2] << 8) | frame[3]
    to_id   = frame[4];  from_id = frame[5]
    crc1_b  = frame[6];  payload = frame[7:-3]
    crc2    = (frame[-3] << 8) | frame[-2]

    crc1_calc = calc_crc1(frame)
    crc1_ok   = crc1_calc == crc1_b
    exp_crc2, crc2_model = verify_crc2(frame)
    crc2_ok = (exp_crc2 == crc2) if exp_crc2 is not None else None

    sem_from = SEM_IDS.get(from_id);  sem_to = SEM_IDS.get(to_id)
    if   sem_from and to_id == ID_INVERTER:      direction = "REQUEST"
    elif sem_to   and from_id == ID_INVERTER:    direction = "RESPONSE"
    elif sem_from and to_id != ID_INVERTER:      direction = f"PING→0x{to_id:02x}"
    else:                                         direction = f"UNKNOWN(from=0x{from_id:02x},to=0x{to_id:02x})"

    cmd        = payload[0] if payload else None
    topic_byte = payload[4] if len(payload) >= 5 else None
    topic_name = TOPIC_NAMES.get(topic_byte, f'0x{topic_byte:02x}') if topic_byte is not None else "?"

    decoded = None
    event_log = None
    try:
        if cmd == 0x20:
            decoded = "ping"
        elif cmd in (0x41,0x65) and topic_byte == 0xf1 and len(payload) >= 9:
            v, u = _total_yield(payload[5:9]);   decoded = f"{v:.2f} {u}"
        elif cmd == 0x41 and topic_byte == 0x3c and len(payload) >= 9:
            v, u = _steca_float(payload[5:9]);   decoded = f"{v:.2f} {u}"
        elif cmd in (0x41,0x65) and topic_byte in (0x29,0x22,0x23,0x24,0x1d) and len(payload) >= 9:
            v, u = _steca_float(payload[5:9]);   decoded = f"{v:.2f} {u}"
        elif cmd == 0x65 and topic_byte == 0x05 and len(payload) >= 11:
            p = payload[5:]
            decoded = str(datetime.datetime(2000+p[0], p[1], p[2], p[3], p[4], p[5]))
        elif cmd == 0x65 and topic_byte == 0x09:
            decoded = payload[5:].rstrip(b'\x00\x9f').decode('latin-1', errors='replace')
        elif cmd == 0x69 and topic_byte in (0x5a, 0x5b):
            total_ev, events = decode_event_log(payload)
            page = "p1" if topic_byte == 0x5a else "p2"
            decoded   = f"event_log({page}): {total_ev} total, {len(events)} entries"
            event_log = (total_ev, events)
    except Exception as e:
        decoded = f"err:{e}"

    return {
        "ts":          datetime.datetime.now().isoformat(),
        "direction":   direction,
        "to":          f"0x{to_id:02x}",
        "from":        f"0x{from_id:02x}",
        "sem":         SEM_IDS.get(from_id, SEM_IDS.get(to_id, "?")),
        "len":         length,
        "crc1":        f"0x{crc1_b:02x}",
        "crc1_calc":   f"0x{crc1_calc:02x}",
        "crc1_ok":     crc1_ok,
        "crc2":        f"0x{crc2:04x}",
        "crc2_exp":    f"0x{exp_crc2:04x}" if exp_crc2 is not None else "?",
        "crc2_ok":     crc2_ok,
        "crc2_model":  crc2_model,
        "topic":       f"0x{topic_byte:02x}" if topic_byte is not None else "?",
        "topic_name":  topic_name,
        "payload_hex": fmt_hex(payload),
        "raw_hex":     fmt_hex(frame),
        "decoded":     decoded,
        "_event_log":  event_log,
    }

# ── Console output ─────────────────────────────────────────────────────────────
def print_frame(info: dict, verbose: bool):
    ts = info["ts"][11:19]
    c1 = "✓" if info["crc1_ok"] else "✗"
    if   info["crc2_ok"] is True:  c2 = "✓"
    elif info["crc2_ok"] is False: c2 = f"✗(exp:{info['crc2_exp']})"
    else:                          c2 = "?"

    print(f"\n[{ts}] {info['direction']}  TO={info['to']} FROM={info['from']}  LEN={info['len']}  {info['sem']}")
    if info["topic"] != "?":
        print(f"  Topic:   {info['topic']} {info['topic_name']}")
    print(f"  CRC1:{info['crc1']}[{c1}]  CRC2:{info['crc2']}[{c2}]  model={info['crc2_model']}")
    if info["decoded"] and info["decoded"] != "ping":
        print(f"  → {info['decoded']}")

    ev = info.get("_event_log")
    if ev:
        total_ev, events = ev
        print(f"  EventLog ({total_ev} total, {len(events)} in this frame):")
        for i, (ts_ev, msg) in enumerate(events, 1):
            ts_str = ts_ev.strftime("%Y-%m-%d %H:%M:%S") if ts_ev else "????-??-?? ??:??:??"
            print(f"    {i:3d}  {ts_str}  {msg}")

    if verbose:
        print(f"  Raw: {info['raw_hex']}")

# ── Stats ──────────────────────────────────────────────────────────────────────
class Stats:
    def __init__(self): self.total = self.c1_ok = self.c2_ok = self.c2_chk = 0
    def update(self, info):
        self.total += 1
        if info["crc1_ok"]: self.c1_ok += 1
        if info["crc2_ok"] is not None:
            self.c2_chk += 1
            if info["crc2_ok"]: self.c2_ok += 1
    def summary(self):
        c2 = f"{self.c2_ok}/{self.c2_chk}" if self.c2_chk else "n/a"
        return f"Frames:{self.total}  CRC1:{self.c1_ok}/{self.total}  CRC2:{c2}"

# ── Serial reader thread ───────────────────────────────────────────────────────
class SerialReader(threading.Thread):
    def __init__(self, ser, out_queue):
        super().__init__(daemon=True)
        self.ser = ser;  self.out_queue = out_queue;  self.running = True

    def run(self):
        while self.running:
            try:
                waiting = self.ser.in_waiting or 0
                n   = max(64, min(waiting, 4096))
                raw = self.ser.read(n)
                if raw:
                    self.out_queue.put(raw)
            except Exception as e:
                print(f"[serial-reader] {e}");  time.sleep(0.1)

    def stop(self): self.running = False

# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Passive RS485 sniffer for StecaGrid 3600')
    parser.add_argument('--port',    default='/dev/ttyUSB0')
    parser.add_argument('--verbose', action='store_true', help='Show raw hex + assembler debug')
    parser.add_argument('--log',     default=LOG_FILE)
    parser.add_argument('--no-log',  action='store_true')
    args = parser.parse_args()

    global DEBUG_DROPS
    DEBUG_DROPS = args.verbose

    print(f"Steca RS485 Sniffer  port={args.port}  baud={SERIAL_BAUDRATE}")
    print(f"Threaded reader  |  CRC1+CRC2: nibble-table (all frame types)  |  EventLog decoder: p1+p2")
    if not args.no_log: print(f"Log → {args.log}")
    print("Ctrl+C to stop.\n")

    try:
        ser = serial.Serial(port=args.port, baudrate=SERIAL_BAUDRATE,
                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE, timeout=SERIAL_TIMEOUT)
    except serial.SerialException as e:
        print(f"ERROR: {e}", file=sys.stderr); sys.exit(1)

    assembler   = FrameAssembler()
    stats       = Stats()
    log_fh      = None if args.no_log else open(args.log, 'a', encoding='utf-8')
    raw_queue   = queue.Queue(maxsize=2000)
    reader      = SerialReader(ser, raw_queue)
    reader.start()
    flush_cnt   = 0

    try:
        while True:
            try:
                raw = raw_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            if args.verbose and raw:
                print(f"  [raw {len(raw)}B]: {fmt_hex(raw[:32])}{'...' if len(raw)>32 else ''}")

            for frame in assembler.feed(raw):
                try:
                    info = decode_frame(frame, args.verbose)
                    stats.update(info)
                    print_frame(info, args.verbose)
                    if log_fh:
                        log_entry = {k: v for k, v in info.items() if not k.startswith('_')}
                        log_fh.write(json.dumps(log_entry) + '\n')
                        flush_cnt += 1
                        if flush_cnt >= 10: log_fh.flush(); flush_cnt = 0
                except Exception as e:
                    print(f"  [err: {e}] raw={fmt_hex(frame)}")

    except KeyboardInterrupt:
        print(f"\n\n{stats.summary()}")
        if log_fh: print(f"Log → {args.log}")
    finally:
        reader.stop()
        ser.close()
        if log_fh: log_fh.flush(); log_fh.close()

if __name__ == '__main__':
    main()
