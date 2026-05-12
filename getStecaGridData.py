#!/usr/bin/env python3
"""
getStecaGridData.py — Read data via RS485 from StecaGrid 3600

CRC1 (solved): poly=0x139, init=0xAA, rev=True, covers frame[0:6]
CRC2 (solved): GF(2) linear model, verified for ping (12 B) and
               data requests (16 B, cmd=0x40 / 0x64)

Frames are now synthesized for any inverter ID — use --id to target
a specific inverter instead of the default 0x01.
"""

import struct
import serial
import argparse
import datetime
import time

DEBUG = False

SERIAL_DEVICE   = "/dev/ttyS0"
SERIAL_BYTES    = serial.EIGHTBITS
SERIAL_PARITY   = serial.PARITY_NONE
SERIAL_SBIT     = serial.STOPBITS_ONE
SERIAL_BAUDRATE = 38400
SERIAL_TIMEOUT  = 1

SEM_ID = 0x7b

# ── CRC1 ──────────────────────────────────────────────────────────────────────
try:
    import crcmod
    _crc1_fn = crcmod.mkCrcFun(0x139, initCrc=0xAA, rev=True, xorOut=0x00)
    def calc_crc1(b6: bytes) -> int:
        return _crc1_fn(b6)
except ImportError:
    def calc_crc1(b6: bytes) -> int:
        poly, crc = 0x39, 0xAA
        for byte in b6:
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

def calc_crc2_ping(to_id: int, sem_id: int = SEM_ID) -> int:
    crc2 = _BASE_PING
    for bit in range(8):
        if to_id & (1 << bit): crc2 ^= _M_PING[bit]
    if   sem_id == 0x7b: crc2 ^= _OFF_PING_7b
    elif sem_id != 0xc9: raise ValueError(f"unsupported SEM ID 0x{sem_id:02x}")
    return crc2

def calc_crc2_req16(topic: int, cmd: int, sem_id: int = SEM_ID) -> int:
    chk_ref = (_T_REF + 0x55) & 0xFF
    chk     = (topic  + 0x55) & 0xFF
    crc2    = _C_REF
    for bit in range(8):
        if ((topic ^ _T_REF) >> bit) & 1: crc2 ^= _M_REQ16[bit]
        if ((chk ^ chk_ref) >> bit) & 1:  crc2 ^= _M_REQ16[8 + bit]
    if cmd == 0x40: crc2 ^= _OFF_40
    if cmd == 0x68: crc2 ^= _OFF_68
    if   sem_id == 0x7b: crc2 ^= _OFF_7b
    elif sem_id != 0xc9: raise ValueError(f"unsupported SEM ID 0x{sem_id:02x}")
    return crc2

# ── Frame builders ────────────────────────────────────────────────────────────
def build_ping(to_id: int, sem_id: int = SEM_ID) -> bytes:
    """12-byte ping / bus-discovery frame."""
    h    = bytes([0x02, 0x01, 0x00, 0x0c, to_id, sem_id])
    crc1 = calc_crc1(h)
    crc2 = calc_crc2_ping(to_id, sem_id)
    return h + bytes([crc1, 0x20, 0x03, crc2 >> 8, crc2 & 0xFF, 0x03])

def build_request16(to_id: int, topic: int, cmd: int,
                    sem_id: int = SEM_ID) -> bytes:
    """16-byte data-request frame.
    CRC2 solved for cmd=0x40 (RequestA), 0x64 (RequestB), 0x68 (RequestC)."""
    h    = bytes([0x02, 0x01, 0x00, 0x10, to_id, sem_id])
    crc1 = calc_crc1(h)
    crc2 = calc_crc2_req16(topic, cmd, sem_id) if cmd in (0x40, 0x64, 0x68) else 0x0000
    chk  = (topic + 0x55) & 0xFF
    return h + bytes([crc1, cmd, 0x03, 0x00, 0x01, topic, chk,
                      crc2 >> 8, crc2 & 0xFF, 0x03])

# ── Pre-built frames for inverter ID 0x01, SEM 0x7b (synthesized) ─────────────
SG_VERSIONS      = build_ping(0x01)
SG_NOMINAL_POWER = build_request16(0x01, 0x1d, 0x40)
SG_PANEL_POWER   = build_request16(0x01, 0x22, 0x40)
SG_PANEL_VOLTAGE = build_request16(0x01, 0x23, 0x40)
SG_PANEL_CURRENT = build_request16(0x01, 0x24, 0x40)
SG_AC_POWER      = build_request16(0x01, 0x29, 0x40)
SG_DAILY_YIELD   = build_request16(0x01, 0x3c, 0x40)
SG_GRID_MEAS      = build_request16(0x01, 0x51, 0x40)
SG_EVENT_LOG_P1   = build_request16(0x01, 0x5a, 0x68)
SG_EVENT_LOG_P2   = build_request16(0x01, 0x5b, 0x68)
SG_TIME           = build_request16(0x01, 0x05, 0x64)
SG_MYSTERY_ONE   = build_request16(0x01, 0x08, 0x64)
SG_SERIAL        = build_request16(0x01, 0x09, 0x64)
SG_TOTAL_YIELD   = build_request16(0x01, 0xf1, 0x64)

# ── Value decoders ────────────────────────────────────────────────────────────
def decode_stecaFloat_a(ac_bytes):
    unit_map = {0x0B: "W", 0x07: "A", 0x05: "V", 0x0D: "Hz",
                0x09: "Wh", 0x00: "NUL"}
    unit = unit_map.get(ac_bytes[0], f'0x{ac_bytes[0]:02x}')
    iacpower = ((ac_bytes[3] << 8 | ac_bytes[1]) << 8 | ac_bytes[2]) << 7
    facpower, = struct.unpack('f', struct.pack('I', iacpower & 0xFFFFFFFF))
    if DEBUG:
        print("# i: 0x%0X" % iacpower, "=", str(iacpower))
        print("# f:", facpower)
    return [facpower, unit]

def decode_TotalYield_a(ba):
    bits = ba[3] << 24 | ba[2] << 16 | ba[1] << 8 | ba[0]
    ieee, = struct.unpack('f', struct.pack('I', bits))
    return [ieee, "Wh"]

def decode_grid_meas(t):
    """Decode GridMeasurements response (topic=0x51, ResponseA).
    Returns [(label, [val, ...]), (label, [val, ...])] for ENS1 and ENS2."""
    try:
        label_a_len = (t[13] << 8) | t[14]
        label_a     = t[15 : 15 + label_a_len].decode('ascii', errors='replace')
        va          = 15 + label_a_len
        vals_a      = [decode_stecaFloat_a(t[va + i*4 : va + i*4 + 4]) for i in range(4)]
        # separator byte at va+16, label_b length at va+17..18, label_b at va+19
        label_b_len = (t[va + 17] << 8) | t[va + 18]
        vb          = va + 19 + label_b_len
        label_b     = t[va + 19 : vb].decode('ascii', errors='replace')
        vals_b      = [decode_stecaFloat_a(t[vb + i*4 : vb + i*4 + 4]) for i in range(4)]
        return [(label_a, vals_a), (label_b, vals_b)]
    except Exception as e:
        if DEBUG:
            print(f"# decode_grid_meas error: {e}")
        return []

def _try_ts(data, pos):
    """Try to parse a 6-byte YY MM DD HH MM SS timestamp at data[pos]."""
    if pos + 6 > len(data):
        return None
    b = data[pos:pos + 6]
    try:
        if 0x0d <= b[0] <= 0x1a and 1 <= b[1] <= 12 and 1 <= b[2] <= 31 \
                and b[3] <= 23 and b[4] <= 59 and b[5] <= 59:
            return datetime.datetime(2000 + b[0], b[1], b[2], b[3], b[4], b[5])
    except Exception:
        pass
    return None

def decode_event_log(payload: bytes):
    """Decode a ResponseC event log payload (payload[0] must be 0x69).
    Returns (total_events, [(datetime_or_None, message_str), ...])."""
    if len(payload) < 7 or payload[0] != 0x69:
        return 0, []
    data  = payload[6:]
    total = data[0]
    # Collect deduplicated timestamps
    raw_ts = [(p, t) for p in range(len(data) - 5) if (t := _try_ts(data, p))]
    ts_dedup = []
    for pos, t in raw_ts:
        if ts_dedup and pos < ts_dedup[-1][0] + 6:
            continue
        ts_dedup.append((pos, t))
    # Collect null-terminated ASCII strings (len >= 4, starts with a letter)
    msgs, pos = [], 0
    while pos < len(data):
        if 65 <= data[pos] <= 122:
            end = pos
            while end < len(data) and 32 <= data[end] <= 126:
                end += 1
            if end - pos >= 4 and end < len(data) and data[end] == 0x00:
                msgs.append((pos, data[pos:end].decode('ascii', errors='replace')))
                pos = end + 1
                continue
        pos += 1
    # Pair each message with the nearest preceding timestamp
    events = []
    for msg_pos, msg in msgs:
        ts = None
        for ts_pos, t in reversed(ts_dedup):
            if ts_pos < msg_pos:
                ts = t
                break
        events.append((ts, msg))
    return total, events

def decode_version(b):
    o = b'SSXSNSSNSSNSSNSSNSSNSSNSSNSSNSSNSSNSSNSSNSSNSSNSSNSSNSSSSSSSSSSSS'
    so = []
    aos = []
    for i in range(len(b)):
        if o[len(aos)] == 83 and b[i] == 0:
            aos.append(''.join(so))
            so = []
        elif o[len(aos)] == 78 and len(so) > 6:
            aos.append('.'.join(so[2:5]))
            so = []
        elif o[len(aos)] == 88 and len(so) > 1:
            aos.append('')
            so = []
        if o[len(aos)] == 83:
            so.append(chr(b[i]))
        elif o[len(aos)] == 78 or o[len(aos)] == 88:
            so.append(str(b[i]))
    s = ""
    for i in range(len(aos)):
        s += aos[i]
        if i < 3 or (i - 4) % 3 == 1:
            s += '\n'
        else:
            s += '\t'
    return s

# ── Frame helpers ─────────────────────────────────────────────────────────────
def format_hex_bytes(b):
    return ' '.join(f'{byte:02x}' for byte in b)

def format_printable(b):
    return ''.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in b)

def is_one_full_telegram(t):
    if not t or t[0] != 2:
        return False
    if t[-1] != 3:
        return False
    if len(t) != (t[2] << 8 | t[3]):
        return False
    return True

def read_complete_frame(port, timeout_s=2.0):
    """Read bytes from port until a complete, valid Steca frame is assembled.
    Handles partial reads and bus noise; returns bytes or None on timeout."""
    buf      = bytearray()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if chunk:
            buf.extend(chunk)
        # Scan buf for a valid frame regardless of whether chunk was empty
        while True:
            idx = buf.find(0x02)
            if idx == -1:
                buf.clear()
                break
            if idx > 0:
                del buf[:idx]
                continue
            if len(buf) < 4:
                break                        # need more data
            frame_len = (buf[2] << 8) | buf[3]
            if frame_len < 7 or frame_len > 4096:
                del buf[0]
                continue
            if len(buf) < frame_len:
                break                        # need more data
            if buf[frame_len - 1] != 0x03:
                del buf[0]
                continue
            return bytes(buf[:frame_len])   # complete frame
    return None

# ── Frame parser ──────────────────────────────────────────────────────────────
def process_steca485(t):
    """Parse a response telegram. Returns a list: [to, from, cmd, topic, label, value]."""
    if not is_one_full_telegram(t):
        if DEBUG:
            print("# NOT a single full Steca485 Telegram")
        return None

    results = [t[4], t[5], t[7], t[11]]
    total_length = (t[2] << 8 | t[3])

    if DEBUG:
        print("#", format_hex_bytes(t))
        print("# dgram:", end="  ")
        print(f"to:{t[4]}  from:{t[5]}  len:{total_length}", end="  ")
        print(f"crc1:{t[6]:02x}  crc2:{t[-3]:02x}{t[-2]:02x}")
        print("# payload:", format_hex_bytes(t[7:-3]), " ", format_printable(t[7:-3]))

    if t[7] == 0x40:    # RequestA
        if DEBUG:
            topics = {0x1d:"Nominal Power", 0x22:"Panel Power", 0x23:"Panel Voltage",
                      0x24:"Panel Current", 0x29:"ACPower", 0x3c:"Daily Yield",
                      0x51:"Grid Measurements"}
            print(f"# RequestA for 0x{t[11]:02x} ({topics.get(t[11], '?')}) from {t[4]}")

    elif t[7] == 0x41:  # ResponseA
        if t[8] == 0x00:
            dlen = (t[9] << 8 | t[10])
            if DEBUG:
                print(f"# ResponseA for 0x{t[11]:02x} from {t[4]} len={dlen}")
            if t[11] == 0x51:
                groups = decode_grid_meas(t)
                results += ["Grid Measurements", groups]
                if DEBUG:
                    for label, vals in groups:
                        print(f"#  {label}:", ", ".join(f"{v[0]:.2f} {v[1]}" for v in vals))
            elif t[11] == 0x3c:
                val = decode_stecaFloat_a(t[12:16])
                results += ["Daily Yield", val]
                if DEBUG:
                    print(f"# Daily Yield {val[0]} {val[1]}")
            else:
                label = t[15:15 + t[14]].decode("ascii", errors="replace")
                val   = decode_stecaFloat_a(t[15 + t[14] : 15 + t[14] + 5])
                results += [label, val]
                if DEBUG:
                    print(f"# {label} {val[0]} {val[1]}")

    elif t[7] == 0x64:  # RequestB
        if DEBUG:
            topics = {0x05:"Time", 0x08:"Mystery_08", 0x09:"Serial", 0xf1:"Total Yield"}
            print(f"# RequestB for 0x{t[11]:02x} ({topics.get(t[11], '?')}) from {t[4]}")

    elif t[7] == 0x65:  # ResponseB
        if DEBUG:
            print(f"# ResponseB for 0x{t[11]:02x} from {t[4]}")
        if t[11] == 0xf1:
            val = decode_TotalYield_a(t[12:16])
            results += ["Total Yield", val]
            if DEBUG:
                print("#", val)
        elif t[11] == 0x05:
            dt = datetime.datetime(2000 + t[12], t[13], t[14], t[15], t[16], t[17])
            results += ["Time", [dt, ""]]
            if DEBUG:
                print(f"# {dt}")
        elif t[11] == 0x08:
            results += ["???", [format_hex_bytes(t[12:17]), ""]]
            if DEBUG:
                print("#", format_hex_bytes(t[12:17]))
        elif t[11] == 0x09:
            serial_str = t[12:-4].rstrip(b'\x00\x9f').decode("latin-1", errors="replace")
            results += ["Serial Number", [serial_str, ""]]
            if DEBUG:
                print(f"# {serial_str}")
        else:
            results += ["???", [format_hex_bytes(t[12:17]), ""]]

    elif t[7] == 0x69:  # ResponseC (event log)
        if t[11] in (0x5a, 0x5b):
            page = "p1" if t[11] == 0x5a else "p2"
            total_ev, events = decode_event_log(t[7:-3])
            results += [f"EventLog-{page}", (total_ev, events)]
            if DEBUG:
                print(f"# EventLog-{page}: {total_ev} total, {len(events)} entries")

    elif t[7] == 0x21:  # Versions response
        if t[8] == 0x00:
            dlen = (t[9] << 8 | t[10])
            ver  = decode_version(t[11:-3])
            results += ["Versions", [ver, ""]]
            print()
            if DEBUG:
                print(f"# VersionsResponse from {t[4]} len={dlen}")

    return results

# ── Serial I/O ────────────────────────────────────────────────────────────────
def getStecaGridResult(port, req, timeout_s=2.0, retries=3):
    """Send req, read response, return results[5] (the value field).
    Retries on error/busy responses from the inverter (shared bus)."""
    for attempt in range(retries):
        if DEBUG and attempt > 0:
            print(f"# retry {attempt}")
        port.reset_input_buffer()
        port.write(req)
        if DEBUG:
            print("\nserial read:")
        in_data = read_complete_frame(port, timeout_s=timeout_s)
        if in_data is None:
            if DEBUG:
                print("# timeout — no complete frame received")
            break  # no response at all — don't retry (wrong CRC2, device absent, …)
        results = process_steca485(in_data)
        if DEBUG:
            print(results)
        if results and len(results) >= 6:
            val = results[5]
            if isinstance(val, list) and len(val) == 2 and val[1] == "NUL":
                return None
            return val
        # Valid frame received but no usable value (error/busy status byte).
        # The inverter was mid-exchange with the SEM — wait and retry.
        if attempt < retries - 1:
            if DEBUG:
                print("# error response, retrying…")
            time.sleep(0.3)
    return None

# ── Bus discovery ─────────────────────────────────────────────────────────────
def discover_inverters(port, full_scan=False):
    id_range   = range(1, 0x66) if full_scan else range(1, 11)
    scan_label = f"0x{id_range.start:02x}..0x{id_range.stop - 1:02x}"
    print("StecaGrid RS485 Bus Discovery")
    print(f"  Scanning: {len(id_range)} IDs ({scan_label})")

    found       = []
    old_timeout = port.timeout
    # Short port timeout so read_complete_frame loops quickly for silent IDs
    port.timeout = 0.05

    for inv_id in id_range:
        port.reset_input_buffer()
        port.write(build_ping(inv_id))
        # Version response can be several hundred bytes; allow 0.5 s total
        resp_frame = read_complete_frame(port, timeout_s=0.5)
        if resp_frame:
            found.append(inv_id)
            # Query serial number with normal timeout
            port.timeout = old_timeout
            port.reset_input_buffer()
            port.write(build_request16(inv_id, 0x09, 0x64))
            serial_frame = read_complete_frame(port, timeout_s=old_timeout or 2.0)
            serial_str   = ""
            if serial_frame:
                serial_res = process_steca485(serial_frame)
                if serial_res and len(serial_res) >= 6:
                    serial_str = f"  Serial: {serial_res[5][0]}"
            print(f"  0x{inv_id:02x}  ✓ found{serial_str}")
            port.timeout = 0.05

    port.timeout = old_timeout
    print(f"\nResult: {len(found)} inverter(s) on bus.")
    return found

# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Read data via RS485 from StecaGrid3600')
    parser.add_argument('-v', '--verbose',      action='store_true', help='Enable verbose output')
    parser.add_argument('-u', '--unit',          action='store_true', help='Output unit of measurement')
    parser.add_argument('-s', '--serial',        help=f'Serial interface (default {SERIAL_DEVICE})')
    parser.add_argument('--id',                  default='0x01',
                        help='RS485 inverter ID to query, hex or decimal (default 0x01)')
    parser.add_argument('-np', '--nominal_power', action='store_true', help='Request nominal power')
    parser.add_argument('-pp', '--panel_power',   action='store_true', help='Request panel power')
    parser.add_argument('-pv', '--panel_voltage', action='store_true', help='Request panel voltage')
    parser.add_argument('-pc', '--panel_current', action='store_true', help='Request panel current')
    parser.add_argument('-ap', '--ac_power',      action='store_true', help='Request AC power')
    parser.add_argument('-gm', '--grid_meas',     action='store_true', help='Request grid measurements (ENS1+ENS2)')
    parser.add_argument('-el', '--event_log',     action='store_true', help='Request event log (both pages)')
    parser.add_argument('-dy', '--daily_yield',   action='store_true', help='Request daily yield')
    parser.add_argument('-ty', '--total_yield',   action='store_true', help='Request total yield')
    parser.add_argument('-ti', '--time',          action='store_true', help='Request inverter time')
    parser.add_argument('-sn', '--serial_number', action='store_true', help='Request serial number')
    parser.add_argument('-ve', '--versions',      action='store_true', help='Request firmware versions')
    parser.add_argument('-m1', '--mystery_one',   action='store_true', help='Request topic 0x08 (unknown)')
    parser.add_argument('--discover',             action='store_true',
                        help='Scan RS485 bus for inverters (quick: IDs 0x01..0x0a)')
    parser.add_argument('--full-scan',            action='store_true',
                        help='Used with --discover: full scan IDs 0x01..0x65')

    args  = parser.parse_args()
    DEBUG = args.verbose
    uom   = args.unit

    ser_port = args.serial or SERIAL_DEVICE
    inv_id   = int(args.id, 0)

    port = serial.Serial(
        baudrate=SERIAL_BAUDRATE, port=ser_port, timeout=SERIAL_TIMEOUT,
        parity=SERIAL_PARITY, stopbits=SERIAL_SBIT, bytesize=SERIAL_BYTES,
        xonxoff=0, rtscts=0,
    )
    if DEBUG:
        print(port.get_settings())

    if args.discover:
        discover_inverters(port, args.full_scan)
        port.close()
        raise SystemExit(0)

    if args.event_log:
        # Event log: request both pages sequentially (CRC2 unverified for cmd=0x68)
        for topic, page in ((0x5a, "p1"), (0x5b, "p2")):
            req   = build_request16(inv_id, topic, 0x68)
            value = getStecaGridResult(port, req, timeout_s=3.0)
            if value is None:
                print(f"EventLog-{page}: no response")
                continue
            if isinstance(value, tuple):
                total_ev, events = value
                print(f"EventLog-{page} ({total_ev} total, {len(events)} in this frame):")
                for ts_ev, msg in events:
                    ts_str = ts_ev.strftime("%Y-%m-%d %H:%M:%S") if ts_ev else "????-??-?? ??:??:??"
                    print(f"  {ts_str}  {msg}")
            else:
                print(f"EventLog-{page}: unexpected response: {value}")
        port.close()
        raise SystemExit(0)

    # Build the request frame for the selected inverter ID
    if args.nominal_power:   reqval = build_request16(inv_id, 0x1d, 0x40)
    elif args.panel_power:   reqval = build_request16(inv_id, 0x22, 0x40)
    elif args.panel_voltage: reqval = build_request16(inv_id, 0x23, 0x40)
    elif args.panel_current: reqval = build_request16(inv_id, 0x24, 0x40)
    elif args.ac_power:      reqval = build_request16(inv_id, 0x29, 0x40)
    elif args.grid_meas:     reqval = build_request16(inv_id, 0x51, 0x40)
    elif args.daily_yield:   reqval = build_request16(inv_id, 0x3c, 0x40)
    elif args.total_yield:   reqval = build_request16(inv_id, 0xf1, 0x64)
    elif args.time:          reqval = build_request16(inv_id, 0x05, 0x64)
    elif args.serial_number: reqval = build_request16(inv_id, 0x09, 0x64)
    elif args.versions:      reqval = build_ping(inv_id)
    elif args.mystery_one:   reqval = build_request16(inv_id, 0x08, 0x64)
    else:                    reqval = build_request16(inv_id, 0xf1, 0x64)  # default: total yield

    value = getStecaGridResult(port, reqval)

    if value is not None:
        if args.grid_meas and isinstance(value, list) and value and isinstance(value[0], tuple):
            for label, vals in value:
                vals_str = "  ".join(
                    f"{v[0]:.2f} {v[1]}" if uom else f"{v[0]:.2f}"
                    for v in vals
                )
                print(f"{label}: {vals_str}")
        elif isinstance(value, list) and len(value) == 2:
            if uom:
                print(value[0], value[1])
            else:
                print(value[0])
        else:
            print(value)

    port.close()
