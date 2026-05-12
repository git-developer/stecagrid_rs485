#!/usr/bin/env python3
"""
steca_sniffer.py — Passive RS485 bus sniffer for StecaGrid 3600
Threaded UART reader + CRC2 verification + event log decoder

CRC1: poly=0x39, init=0xAA, refin=True, refout=True, covers frame[0:6] ✓
CRC2: GF(2) linear model — fully solved:
  - Ping  (cmd=0x20, 12B):             100% verified, SEM=0xc9 + 0x7b
  - Req16 (cmd=0x40/0x64/0x68, 16B):  100% verified, SEM=0xc9 + 0x7b
    · cmd=0x64 base: T_REF=0x05, CRC2_REF=0x8ba1
    · cmd=0x40 offset: XOR 0x572c
    · cmd=0x68 offset: XOR 0xeef5  (derived from 3 captured frames)
    · SEM=0x7b offset: XOR 0xb1e5 (req16) / XOR 0xb6db (ping)

Usage:
  python3 steca_sniffer.py --port /dev/ttyUSB0
  python3 steca_sniffer.py --port /dev/ttyUSB0 --verbose

Install: pip3 install pyserial crcmod
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

# ── CRC1 ─────────────────────────────────────────────────────────────────────
try:
    import crcmod
    _crc1_fn = crcmod.mkCrcFun(0x139, initCrc=0xAA, rev=True, xorOut=0x00)
    def calc_crc1(frame: bytes) -> int:
        return _crc1_fn(frame[0:6])
except ImportError:
    def calc_crc1(frame: bytes) -> int:
        poly, crc = 0x39, 0xAA
        for byte in frame[0:6]:
            byte = int(f'{byte:08b}'[::-1], 2)
            crc ^= byte
            for _ in range(8):
                crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        return int(f'{crc:08b}'[::-1], 2)

# ── CRC2 GF(2) linear model ───────────────────────────────────────────────────
_M_PING      = [0x39b2, 0x7364, 0xe6c8, 0x78cd, 0xf19a, 0x5669, 0xacd2, 0x0000]
_BASE_PING   = 0xf6e5
_OFF_PING_7b = 0xb6db

_T_REF   = 0x05
_C_REF   = 0x8ba1
_M_REQ16 = [
    0x87c7, 0x72a3, 0x2d36, 0x5a6c, 0xb4d8, 0xdced, 0x0c87, 0x190e,
    0x0000, 0x0000, 0xc870, 0x25bd, 0x4b7a, 0x96f4, 0x98b5, 0x8437,
]
_OFF_40 = 0x572c
_OFF_68 = 0xeef5  # verified: 3/3 captured frames (topics 0x09, 0x5a, 0x5b)
_OFF_7b = 0xb1e5

def calc_crc2_ping(to_id: int, sem_id: int):
    crc2 = _BASE_PING
    for bit in range(8):
        if to_id & (1 << bit): crc2 ^= _M_PING[bit]
    if   sem_id == 0x7b: crc2 ^= _OFF_PING_7b
    elif sem_id != 0xc9: return None
    return crc2

def calc_crc2_req16(topic: int, cmd: int, sem_id: int):
    chk_ref = (_T_REF + 0x55) & 0xFF
    chk     = (topic  + 0x55) & 0xFF
    crc2    = _C_REF
    for bit in range(8):
        if ((topic ^ _T_REF) >> bit) & 1: crc2 ^= _M_REQ16[bit]
        if ((chk ^ chk_ref) >> bit) & 1:  crc2 ^= _M_REQ16[8 + bit]
    if cmd == 0x40: crc2 ^= _OFF_40
    if cmd == 0x68: crc2 ^= _OFF_68
    if   sem_id == 0x7b: crc2 ^= _OFF_7b
    elif sem_id != 0xc9: return None
    return crc2

def verify_crc2(frame: bytes):
    length = (frame[2] << 8) | frame[3]
    to_id  = frame[4];  sem_id = frame[5]
    cmd    = frame[7]   if length > 7  else None
    topic  = frame[11]  if length >= 12 else None
    if length == 12 and cmd == 0x20:
        return calc_crc2_ping(to_id, sem_id), "ping"
    if length == 16 and cmd in (0x40, 0x64, 0x68) and to_id == ID_INVERTER and topic is not None:
        return calc_crc2_req16(topic, cmd, sem_id), f"req16/{cmd:02x}"
    return None, "?"

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
    print(f"Threaded reader  |  CRC2 model: ping + req16(0x40/0x64/0x68)  |  EventLog decoder: p1+p2")
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
