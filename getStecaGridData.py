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
    if cmd == 0x40:      crc2 ^= _OFF_40
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
    """16-byte data-request frame (cmd=0x40 RequestA or cmd=0x64 RequestB)."""
    h    = bytes([0x02, 0x01, 0x00, 0x10, to_id, sem_id])
    crc1 = calc_crc1(h)
    crc2 = calc_crc2_req16(topic, cmd, sem_id)
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
SG_GRID_MEAS     = build_request16(0x01, 0x51, 0x40)
SG_TIME          = build_request16(0x01, 0x05, 0x64)
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
def getStecaGridResult(port, req):
    """Send req, read response, return results[5] (the value field)."""
    if DEBUG:
        print("\nserial write:")
        process_steca485(req)
    port.write(req)
    if DEBUG:
        print("\nserial read:")
    in_data = port.read(size=1024)
    results = process_steca485(in_data)
    if DEBUG:
        print(results)
    if results and len(results) >= 6:
        val = results[5]
        if isinstance(val, list) and len(val) == 2 and val[1] == "NUL":
            return None
        return val
    return None

# ── Bus discovery ─────────────────────────────────────────────────────────────
def discover_inverters(port, full_scan=False):
    id_range   = range(1, 0x66) if full_scan else range(1, 11)
    scan_label = f"0x{id_range.start:02x}..0x{id_range.stop - 1:02x}"
    print(f"StecaGrid RS485 Bus Discovery")
    print(f"  Scanning: {len(id_range)} IDs ({scan_label})")

    found        = []
    old_timeout  = port.timeout
    port.timeout = 0.3

    for inv_id in id_range:
        port.write(build_ping(inv_id))
        resp = port.read(256)
        if resp and len(resp) >= 4 and resp[0] == 0x02:
            frame_len = (resp[2] << 8) | resp[3]
            if len(resp) >= frame_len and resp[frame_len - 1] == 0x03:
                found.append(inv_id)
                port.timeout = old_timeout
                port.write(build_request16(inv_id, 0x09, 0x64))
                serial_resp = port.read(1024)
                serial_res  = process_steca485(serial_resp)
                serial_str  = ""
                if serial_res and len(serial_res) >= 6:
                    serial_str = f"  Serial: {serial_res[5][0]}"
                print(f"  0x{inv_id:02x}  ✓ found{serial_str}")
                port.timeout = 0.3

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

    # Build the request frame for the selected inverter ID
    if args.nominal_power: reqval = build_request16(inv_id, 0x1d, 0x40)
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
