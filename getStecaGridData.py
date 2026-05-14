#!/usr/bin/env python3
"""
getStecaGridData.py — Read/write data via RS485 from StecaGrid 3600

CRC1 and CRC2 computed by steca_crc.py (nibble-table algorithms).
Use --discover / --full-scan to find inverters, then query by --id.
"""

import struct
import serial
import argparse
import datetime
import time

from steca_crc import build_frame

DEBUG = False

SERIAL_DEVICE   = "/dev/ttyS0"
SERIAL_BYTES    = serial.EIGHTBITS
SERIAL_PARITY   = serial.PARITY_NONE
SERIAL_SBIT     = serial.STOPBITS_ONE
SERIAL_BAUDRATE = 38400
SERIAL_TIMEOUT  = 1

SEM_ID   = 0x7b   # our RS485 sender address
SEM_ADDR = 0x65   # RS485 address of the StecaGrid SEM energy manager

# ── Topic registry ────────────────────────────────────────────────────────────
# name → (topic_byte, cmd_byte)
TOPICS = {
    # Inverter reads (TO=0x01)
    "nominal_power": (0x1d, 0x40),
    "panel_power":   (0x22, 0x40),
    "panel_voltage": (0x23, 0x40),
    "panel_current": (0x24, 0x40),
    "ac_power":      (0x29, 0x40),
    "daily_yield":   (0x3c, 0x40),
    "grid_meas":     (0x51, 0x40),
    "grid_meas_l2":  (0x52, 0x40),
    "grid_meas_l3":  (0x53, 0x40),
    "event_log_p1":  (0x5a, 0x68),
    "event_log_p2":  (0x5b, 0x68),
    "time":          (0x05, 0x64),
    "bootup_ts":     (0x08, 0x64),
    "serial":        (0x09, 0x64),
    "total_yield":   (0xf1, 0x64),
    # SEM reads (use with to_id=SEM_ADDR)
    "em_config":     (0x0a, 0x64),
    "em_live":       (0x0d, 0x64),
}

# ── Historical yield topic IDs (UploadById cmd=0x64, TO=0x01) ─────────────────
# Index 0 = most recent, index N = N periods ago
_DAY_CURVE_TOPICS   = (0x7b, 0x75, 0x6f, 0x69, 0x63, 0x5d, 0x57,
                       *range(0x93, 0x7b, -1))          # 31: today → −30 days
_DAY_VALUE_TOPICS   = (0xbf, 0xbd, 0xbb, 0xb9, 0xb7, 0xb5, 0xb3,
                       0xb1, 0xaf, 0xad, 0xab, 0xa9, 0xa8)  # 13: this month → −12
_MONTH_VALUE_TOPICS = tuple(range(0xe0, 0xcc, -1))     # 20: this year → −19
_YEAR_VALUE_TOPIC   = 0xef                              # all years as float array

_DAY_CURVE_TOPICS_SET = frozenset(_DAY_CURVE_TOPICS)
_ALL_HIST_TOPICS      = (_DAY_CURVE_TOPICS_SET | frozenset(_DAY_VALUE_TOPICS)
                         | frozenset(_MONTH_VALUE_TOPICS) | {_YEAR_VALUE_TOPIC})

# ── Frame builders ────────────────────────────────────────────────────────────
def build_ping(to_id: int, sem_id: int = SEM_ID) -> bytes:
    """12-byte ping / bus-discovery frame."""
    return build_frame(to_id, sem_id, bytes([0x20, 0x03]))


def build_request(to_id: int, topic: int, cmd: int,
                  sem_id: int = SEM_ID) -> bytes:
    """16-byte read-request frame."""
    chk = (topic + 0x55) & 0xFF
    return build_frame(to_id, sem_id, bytes([cmd, 0x03, 0x00, 0x01, topic, chk]))


def build_write(to_id: int, topic: int, cmd: int,
                data: bytes, sem_id: int = SEM_ID) -> bytes:
    """Variable-length write-request frame (cmd=0x50/0x60).

    DataFrame = [topic] + data; chk = (0x55 + sum(DataFrame)) & 0xFF.
    """
    df  = bytes([topic]) + data
    chk = (0x55 + sum(df)) & 0xFF
    payload = bytes([cmd, 0x03, len(df) >> 8, len(df) & 0xFF]) + df + bytes([chk])
    return build_frame(to_id, sem_id, payload)


# Pre-computed ping frames for all RS485 IDs (avoids per-scan alloc)
PING_FRAMES = {i: build_ping(i) for i in range(1, 0x66)}


def build_set_time(dt: datetime.datetime, inv_id: int = 0x01,
                   sem_id: int = SEM_ID) -> bytes:
    """SetDateTime frame (cmd=0x60 DownloadById, topic=0x05)."""
    return build_write(inv_id, 0x05, 0x60,
                       bytes([dt.year - 2000, dt.month, dt.day,
                               dt.hour, dt.minute, dt.second]),
                       sem_id)


def build_energy_manager_payload(derating_mode: int,
                                  power_limit_w: int,
                                  nominal_power_w: int) -> bytes:
    """Build 87-byte EnergyManager config payload for write to SEM (topic=0x0a).

    derating_mode: 0=Off, 1=RippleControl, 2=PowerLimit, 3=EasyBox
    Unused fields default to zero / disabled.
    """
    buf = bytearray(87)
    # [0]    payload_version = 0 (implicit)
    # [1-2]  S0PulsesPerkWh = 1000
    struct.pack_into('>h', buf, 1, 1000)
    # [3]    DeratingMode
    buf[3] = derating_mode & 0xFF
    # [4-35] DeratingPatterns[16] = -1 (disabled)
    for i in range(16):
        struct.pack_into('>h', buf, 4 + i * 2, -1)
    # [36-39] NominalPowerW
    struct.pack_into('>I', buf, 36, nominal_power_w)
    # [40-43] DeratingPowerLimitW
    struct.pack_into('>I', buf, 40, power_limit_w)
    # [54-55] Limit_Permill = 1000 (= 100.0 %)
    struct.pack_into('>H', buf, 54, 1000)
    return bytes(buf)

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
    """Decode GridMeasurements response (topic=0x51).
    Returns [(label, [val, ...]), (label, [val, ...])] for ENS1 and ENS2."""
    try:
        label_a_len = (t[13] << 8) | t[14]
        label_a     = t[15 : 15 + label_a_len].decode('ascii', errors='replace')
        va          = 15 + label_a_len
        vals_a      = [decode_stecaFloat_a(t[va + i*4 : va + i*4 + 4]) for i in range(4)]
        label_b_len = (t[va + 17] << 8) | t[va + 18]
        vb          = va + 19 + label_b_len
        label_b     = t[va + 19 : vb].decode('ascii', errors='replace')
        vals_b      = [decode_stecaFloat_a(t[vb + i*4 : vb + i*4 + 4]) for i in range(4)]
        return [(label_a, vals_a), (label_b, vals_b)]
    except Exception as e:
        if DEBUG:
            print(f"# decode_grid_meas error: {e}")
        return []


def decode_em_config(raw: bytes) -> dict:
    """Parse 87-byte EnergyManager config payload (assumed big-endian)."""
    if len(raw) < 87:
        return {"error": f"too short ({len(raw)} bytes)"}
    mode_names = {0: "Off", 1: "RippleControl", 2: "PowerLimit", 3: "EasyBox"}
    mode = raw[3]
    patterns = [struct.unpack_from('>h', raw, 4 + i*2)[0] for i in range(16)]
    return {
        "payload_version":      raw[0],
        "S0PulsesPerkWh":       struct.unpack_from('>h', raw, 1)[0],
        "DeratingMode":         f"{mode} ({mode_names.get(mode, '?')})",
        "DeratingPatterns":     patterns,
        "NominalPowerW":        struct.unpack_from('>I', raw, 36)[0],
        "DeratingPowerLimitW":  struct.unpack_from('>I', raw, 40)[0],
        "PID_Kp":               struct.unpack_from('>H', raw, 44)[0],
        "PID_Ki":               struct.unpack_from('>H', raw, 46)[0],
        "PID_Kd":               struct.unpack_from('>H', raw, 48)[0],
        "PeriodeMin_s":         struct.unpack_from('>H', raw, 50)[0],
        "PeriodeMax_s":         struct.unpack_from('>H', raw, 52)[0],
        "Limit_Permill":        struct.unpack_from('>H', raw, 54)[0],
        "RelaisMode":           raw[56],
    }


def _try_ts(data, pos):
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
    """Decode a ResponseC event log payload (payload[0] must be 0x69)."""
    if len(payload) < 7 or payload[0] != 0x69:
        return 0, []
    data  = payload[6:]
    total = data[0]
    raw_ts = [(p, t) for p in range(len(data) - 5) if (t := _try_ts(data, p))]
    ts_dedup = []
    for pos, t in raw_ts:
        if ts_dedup and pos < ts_dedup[-1][0] + 6:
            continue
        ts_dedup.append((pos, t))
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
    """Read bytes from port until a complete, valid Steca frame is assembled."""
    buf      = bytearray()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        chunk = port.read(256)
        if chunk:
            buf.extend(chunk)
        while True:
            idx = buf.find(0x02)
            if idx == -1:
                buf.clear()
                break
            if idx > 0:
                del buf[:idx]
                continue
            if len(buf) < 4:
                break
            frame_len = (buf[2] << 8) | buf[3]
            if frame_len < 7 or frame_len > 4096:
                del buf[0]
                continue
            if len(buf) < frame_len:
                break
            if buf[frame_len - 1] != 0x03:
                del buf[0]
                continue
            return bytes(buf[:frame_len])
    return None

# ── Frame parser ──────────────────────────────────────────────────────────────
def process_steca485(t):
    """Parse a response telegram. Returns [to, from, cmd, topic, label, value]."""
    if not is_one_full_telegram(t):
        if DEBUG:
            print("# NOT a single full Steca485 Telegram")
        return None

    results = [t[4], t[5], t[7], t[11]]
    total_length = (t[2] << 8 | t[3])

    if DEBUG:
        print("#", format_hex_bytes(t))
        print(f"# to:{t[4]}  from:{t[5]}  len:{total_length}  "
              f"crc1:{t[6]:02x}  crc2:{t[-3]:02x}{t[-2]:02x}")
        print("# payload:", format_hex_bytes(t[7:-3]), " ", format_printable(t[7:-3]))

    if t[7] == 0x41:  # ResponseA (ReadDataById)
        if t[8] == 0x00:
            if t[11] == 0x51:
                groups = decode_grid_meas(t)
                results += ["Grid Measurements", groups]
            elif t[11] == 0x3c:
                val = decode_stecaFloat_a(t[12:16])
                results += ["Daily Yield", val]
            else:
                label = t[15:15 + t[14]].decode("ascii", errors="replace")
                val   = decode_stecaFloat_a(t[15 + t[14] : 15 + t[14] + 5])
                results += [label, val]

    elif t[7] == 0x51:  # WriteDataById ACK
        results += ["WriteAck-51", [f"topic=0x{t[11]:02x}", ""]]

    elif t[7] == 0x61:  # DownloadById ACK
        results += ["WriteAck-61", [f"topic=0x{t[11]:02x} status=0x{t[8]:02x}", ""]]

    elif t[7] == 0x65:  # ResponseB (UploadById)
        if t[11] == 0xf1:
            val = decode_TotalYield_a(t[12:16])
            results += ["Total Yield", val]
        elif t[11] == 0x05:
            dt = datetime.datetime(2000 + t[12], t[13], t[14], t[15], t[16], t[17])
            results += ["Time", [dt, ""]]
        elif t[11] == 0x08:
            # Bootup timestamp: BE uint32 milliseconds since last inverter reboot
            if len(t) >= 16:
                ms = struct.unpack('>I', t[12:16])[0]
                boot_time = datetime.datetime.now() - datetime.timedelta(milliseconds=ms)
                results += ["BootupTimestamp", [boot_time, f"{ms} ms uptime"]]
            else:
                results += ["BootupTimestamp", [format_hex_bytes(t[12:16]), ""]]
        elif t[11] == 0x09:
            serial_str = t[12:-4].rstrip(b'\x00\x9f').decode("latin-1", errors="replace")
            results += ["Serial Number", [serial_str, ""]]
        elif t[11] == 0x0a:
            # EnergyManager config (from SEM)
            results += ["EMConfig", t[12:-4]]
        elif t[11] in _ALL_HIST_TOPICS:
            raw = t[12:-4]
            n   = (len(raw) // 4) * 4
            is_curve = t[11] in _DAY_CURVE_TOPICS_SET
            wh_list = []
            for i in range(0, n, 4):
                f, = struct.unpack_from('<f', raw, i)
                wh_list.append(int(round(f * 6 if is_curve else f)))
            results += ["HistYield", (wh_list, "Wh")]
        else:
            results += ["???", [format_hex_bytes(t[12:min(12+16, len(t)-4)]), ""]]

    elif t[7] == 0x69:  # ResponseC (UploadInternById — event log)
        if t[11] in (0x5a, 0x5b):
            page = "p1" if t[11] == 0x5a else "p2"
            total_ev, events = decode_event_log(t[7:-3])
            results += [f"EventLog-{page}", (total_ev, events)]

    elif t[7] == 0x21:  # ReadIdentification response
        if t[8] == 0x00:
            ver = decode_version(t[11:-3])
            results += ["Versions", [ver, ""]]
            print()

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
        in_data = read_complete_frame(port, timeout_s=timeout_s)
        if in_data is None:
            if DEBUG:
                print("# timeout")
            break
        results = process_steca485(in_data)
        if DEBUG:
            print(results)
        if results and len(results) >= 6:
            val = results[5]
            if isinstance(val, list) and len(val) == 2 and val[1] == "NUL":
                return None
            return val
        if attempt < retries - 1:
            time.sleep(0.3)
    return None

def get_inverter_time(port, inv_id: int = 0x01) -> datetime.datetime:
    """Get inverter datetime; fall back to PC time on failure."""
    try:
        val = getStecaGridResult(port, build_request(inv_id, *TOPICS["time"]))
        if isinstance(val, list) and len(val) == 2 and isinstance(val[0], datetime.datetime):
            return val[0]
    except Exception:
        pass
    return datetime.datetime.now()

# ── Historical yield helpers ──────────────────────────────────────────────────
def read_day_curve(port, day_offset: int = 0, inv_id: int = 0x01):
    """Power curve for a day. day_offset=0 → today, 1 → yesterday, …, 30."""
    if not 0 <= day_offset < len(_DAY_CURVE_TOPICS):
        raise ValueError(f"day_offset must be 0..{len(_DAY_CURVE_TOPICS)-1}")
    return getStecaGridResult(port, build_request(inv_id, _DAY_CURVE_TOPICS[day_offset], 0x64))


def read_day_values(port, month_offset: int = 0, inv_id: int = 0x01):
    """Daily yield totals. month_offset=0 → this month, 1 → last month, …, 12."""
    if not 0 <= month_offset < len(_DAY_VALUE_TOPICS):
        raise ValueError(f"month_offset must be 0..{len(_DAY_VALUE_TOPICS)-1}")
    return getStecaGridResult(port, build_request(inv_id, _DAY_VALUE_TOPICS[month_offset], 0x64))


def read_month_values(port, year_offset: int = 0, inv_id: int = 0x01):
    """Monthly yield totals. year_offset=0 → this year, 1 → last year, …, 19."""
    if not 0 <= year_offset < len(_MONTH_VALUE_TOPICS):
        raise ValueError(f"year_offset must be 0..{len(_MONTH_VALUE_TOPICS)-1}")
    return getStecaGridResult(port, build_request(inv_id, _MONTH_VALUE_TOPICS[year_offset], 0x64))


def read_year_values(port, inv_id: int = 0x01):
    """All yearly yield totals as array of floats."""
    return getStecaGridResult(port, build_request(inv_id, _YEAR_VALUE_TOPIC, 0x64))

# ── Yield table formatters ────────────────────────────────────────────────────
def print_10min_history_table(wh_list, ref_date, day_offset: int):
    queried = ref_date - datetime.timedelta(days=day_offset)
    suffix  = "  (today)" if day_offset == 0 else ""
    header  = f"10-min history: {queried}{suffix}"
    print(header)
    print("─" * max(len(header), 30))
    first = next((i for i, v in enumerate(wh_list) if v), None)
    if first is None:
        print("  (no data)")
        return
    last = len(wh_list) - 1 - next(i for i, v in enumerate(reversed(wh_list)) if v)
    total = 0
    for idx in range(first, last + 1):
        wh = wh_list[idx]
        hh, mm = divmod(idx * 10, 60)
        print(f"  {hh:02d}:{mm:02d}  {wh:>8,} Wh")
        total += wh
    print("─" * max(len(header), 30))
    print(f"  Total:  {total:>8,} Wh")


def print_daily_history_table(wh_list, ref_date, month_offset: int):
    y, m = ref_date.year, ref_date.month
    for _ in range(month_offset):
        m -= 1
        if m == 0:
            m, y = 12, y - 1
    month_label = datetime.date(y, m, 1).strftime("%B %Y")
    header = f"Daily history: {month_label}"
    print(header)
    print("─" * max(len(header), 26))
    end = len(wh_list)
    while end > 0 and wh_list[end - 1] == 0:
        end -= 1
    total = 0
    for i in range(end):
        try:
            dt = datetime.date(y, m, i + 1)
        except ValueError:
            break
        print(f"  {dt}  {wh_list[i]:>8,} Wh")
        total += wh_list[i]
    print("─" * max(len(header), 26))
    print(f"  Total:      {total:>8,} Wh")


def print_monthly_history_table(wh_list, ref_date, year_offset: int):
    year = ref_date.year - year_offset
    header = f"Monthly history: {year}"
    print(header)
    print("─" * max(len(header), 24))
    _MONTHS = ["Jan", "Feb", "Mar", "Apr", "May", "Jun",
               "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"]
    end = min(len(wh_list), 12)
    while end > 0 and wh_list[end - 1] == 0:
        end -= 1
    total = 0
    for i in range(end):
        print(f"  {_MONTHS[i]}  {wh_list[i]:>10,} Wh")
        total += wh_list[i]
    print("─" * max(len(header), 24))
    print(f"  Total  {total:>10,} Wh")


def print_yearly_history_table(wh_list, ref_year: int):
    start = 0
    while start < len(wh_list) and wh_list[start] == 0:
        start += 1
    data = wh_list[start:]
    if not data:
        print("  (no data)")
        return
    print("Yearly history")
    print("─" * 22)
    total = 0
    n = len(data)
    for i, wh in enumerate(reversed(data)):
        year = ref_year - (n - 1 - i)
        print(f"  {year}  {wh:>10,} Wh")
        total += wh
    print("─" * 22)
    print(f"  Total  {total:>10,} Wh")


# ── Bus discovery ─────────────────────────────────────────────────────────────
def discover_inverters(port, full_scan=False):
    id_range   = range(1, 0x66) if full_scan else range(1, 11)
    scan_label = f"0x{id_range.start:02x}..0x{id_range.stop - 1:02x}"
    print("StecaGrid RS485 Bus Discovery")
    print(f"  Scanning: {len(id_range)} IDs ({scan_label})")

    found       = []
    old_timeout = port.timeout
    port.timeout = 0.05

    for inv_id in id_range:
        port.reset_input_buffer()
        port.write(PING_FRAMES[inv_id])
        resp_frame = read_complete_frame(port, timeout_s=0.5)
        if resp_frame:
            found.append(inv_id)
            port.timeout = old_timeout
            port.reset_input_buffer()
            port.write(build_request(inv_id, *TOPICS["serial"]))
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
    parser = argparse.ArgumentParser(description='Read/write data via RS485 from StecaGrid 3600')
    parser.add_argument('-v', '--verbose',        action='store_true')
    parser.add_argument('-u', '--unit',            action='store_true', help='Show unit of measurement')
    parser.add_argument('-s', '--serial',          help=f'Serial port (default {SERIAL_DEVICE})')
    parser.add_argument('--id',                    default='0x01',
                        help='Inverter RS485 ID (default 0x01)')
    # Read args
    parser.add_argument('-np', '--nominal_power',  action='store_true')
    parser.add_argument('-pp', '--panel_power',    action='store_true')
    parser.add_argument('-pv', '--panel_voltage',  action='store_true')
    parser.add_argument('-pc', '--panel_current',  action='store_true')
    parser.add_argument('-ap', '--ac_power',       action='store_true')
    parser.add_argument('-gm', '--grid_meas',      action='store_true',
                        help='Grid measurements ENS1+ENS2')
    parser.add_argument('-el', '--event_log',      action='store_true',
                        help='Event log (both pages)')
    parser.add_argument('-dy', '--daily_yield',    action='store_true')
    parser.add_argument('-ty', '--total_yield',    action='store_true')
    parser.add_argument('-ti', '--time',           action='store_true')
    parser.add_argument('-sn', '--serial_number',  action='store_true')
    parser.add_argument('-ve', '--versions',       action='store_true')
    parser.add_argument('--bootup-timestamp',      action='store_true',
                        help='Show inverter boot time (topic 0x08)')
    # Historical yield
    parser.add_argument('--10min-history',   type=int, nargs='?', const=0, metavar='N',
                        dest='hist_10min',
                        help='10-min power history: N days ago (0=today, max 30)')
    parser.add_argument('--daily-history',   type=int, nargs='?', const=0, metavar='N',
                        dest='hist_daily',
                        help='Daily yield history: N months ago (0=this month, max 12)')
    parser.add_argument('--monthly-history', type=int, nargs='?', const=0, metavar='N',
                        dest='hist_monthly',
                        help='Monthly yield history: N years ago (0=this year, max 19)')
    parser.add_argument('--yearly-history',  action='store_true', dest='hist_yearly',
                        help='All yearly yield history')
    # Discovery
    parser.add_argument('--discover',    action='store_true',
                        help='Scan RS485 bus (quick: IDs 0x01..0x0a)')
    parser.add_argument('--full-scan',   action='store_true',
                        help='With --discover: full scan 0x01..0x65')
    # Write / control
    parser.add_argument('--set-time', metavar='DATETIME',
                        help='Set inverter clock, format "YYYY-MM-DD HH:MM:SS". '
                             'The Steca has no DST — always pass standard/winter time.')
    parser.add_argument('--sync-time', action='store_true',
                        help='Sync inverter clock to system time. '
                             'If the system is in summer time (DST), subtracts 1 h '
                             'before writing — the Steca has no DST support.')
    parser.add_argument('--set-power-limit', type=int, metavar='WATTS',
                        help='Set inverter power limit via SEM EnergyManager config '
                             '(reads current config, sets DeratingMode=PowerLimit, writes back)')

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

    # ── Dispatch ─────────────────────────────────────────────────────────────

    if args.discover:
        discover_inverters(port, args.full_scan)
        port.close()
        raise SystemExit(0)

    if args.set_power_limit is not None:
        watts = args.set_power_limit
        print(f"Reading EnergyManager config from SEM (0x{SEM_ADDR:02x})...")
        raw = getStecaGridResult(port, build_request(SEM_ADDR, *TOPICS["em_config"]),
                                 timeout_s=3.0)
        if not isinstance(raw, (bytes, bytearray)) or len(raw) < 87:
            print(f"ERROR: Failed to read EM config (got: {raw!r})")
            port.close()
            raise SystemExit(1)
        config    = bytearray(raw)
        old_mode  = config[3]
        old_limit = struct.unpack_from('>I', config, 40)[0]
        config[3] = 2   # DeratingMode = PowerLimit
        struct.pack_into('>I', config, 40, watts)
        if DEBUG:
            print(f"# EM config: mode {old_mode}→2, limit {old_limit}→{watts} W")
        print(f"Writing power limit {watts} W to SEM...")
        write_req = build_write(SEM_ADDR, 0x0a, 0x60, bytes(config))
        port.reset_input_buffer()
        port.write(write_req)
        resp = read_complete_frame(port, timeout_s=3.0)
        if resp:
            print(f"SEM response: {format_hex_bytes(resp)}")
        else:
            print("WARNING: No response from SEM (may still have worked)")
        port.close()
        raise SystemExit(0)

    if args.set_time:
        try:
            dt = datetime.datetime.strptime(args.set_time, "%Y-%m-%d %H:%M:%S")
        except ValueError:
            print('ERROR: --set-time requires format "YYYY-MM-DD HH:MM:SS"')
            port.close()
            raise SystemExit(1)
        print(f"Setting inverter time to {dt}  (no DST — pass standard/winter time)")
        port.reset_input_buffer()
        port.write(build_set_time(dt, inv_id))
        resp = read_complete_frame(port, timeout_s=3.0)
        print(f"Response: {format_hex_bytes(resp)}" if resp else "WARNING: no response")
        port.close()
        raise SystemExit(0)

    if args.sync_time:
        now = datetime.datetime.now()
        # Steca has no DST: if system is currently in summer time, subtract 1 h
        is_dst = time.localtime().tm_isdst > 0
        if is_dst:
            now -= datetime.timedelta(hours=1)
        dst_note = " (DST active → converted to standard time)" if is_dst else " (no DST correction needed)"
        print(f"Syncing inverter clock to {now.strftime('%Y-%m-%d %H:%M:%S')}{dst_note}")
        port.reset_input_buffer()
        port.write(build_set_time(now, inv_id))
        resp = read_complete_frame(port, timeout_s=3.0)
        print(f"Response: {format_hex_bytes(resp)}" if resp else "WARNING: no response")
        port.close()
        raise SystemExit(0)

    if args.event_log:
        for name in ("event_log_p1", "event_log_p2"):
            page  = name.split("_", 2)[2]
            value = getStecaGridResult(port, build_request(inv_id, *TOPICS[name]),
                                       timeout_s=3.0)
            if value is None:
                print(f"EventLog-{page}: no response")
            elif isinstance(value, tuple):
                total_ev, events = value
                print(f"EventLog-{page} ({total_ev} total, {len(events)} in this frame):")
                for ts_ev, msg in events:
                    ts_str = ts_ev.strftime("%Y-%m-%d %H:%M:%S") if ts_ev else "????-??-?? ??:??:??"
                    print(f"  {ts_str}  {msg}")
            else:
                print(f"EventLog-{page}: unexpected: {value}")
        port.close()
        raise SystemExit(0)

    # Historical yield
    if args.hist_10min is not None or args.hist_daily is not None \
            or args.hist_monthly is not None or args.hist_yearly:
        steca_dt = get_inverter_time(port, inv_id)
        ref_date = steca_dt.date()
        if args.hist_10min is not None:
            result = read_day_curve(port, args.hist_10min, inv_id)
            if result is None:
                print("10-min history: no response")
            else:
                print_10min_history_table(result[0], ref_date, args.hist_10min)
        elif args.hist_daily is not None:
            result = read_day_values(port, args.hist_daily, inv_id)
            if result is None:
                print("Daily history: no response")
            else:
                print_daily_history_table(result[0], ref_date, args.hist_daily)
        elif args.hist_monthly is not None:
            result = read_month_values(port, args.hist_monthly, inv_id)
            if result is None:
                print("Monthly history: no response")
            else:
                print_monthly_history_table(result[0], ref_date, args.hist_monthly)
        elif args.hist_yearly:
            result = read_year_values(port, inv_id)
            if result is None:
                print("Yearly history: no response")
            else:
                print_yearly_history_table(result[0], steca_dt.year)
        port.close()
        raise SystemExit(0)

    # Single-value reads
    if args.nominal_power:     reqval = build_request(inv_id, *TOPICS["nominal_power"])
    elif args.panel_power:     reqval = build_request(inv_id, *TOPICS["panel_power"])
    elif args.panel_voltage:   reqval = build_request(inv_id, *TOPICS["panel_voltage"])
    elif args.panel_current:   reqval = build_request(inv_id, *TOPICS["panel_current"])
    elif args.ac_power:        reqval = build_request(inv_id, *TOPICS["ac_power"])
    elif args.grid_meas:       reqval = build_request(inv_id, *TOPICS["grid_meas"])
    elif args.daily_yield:     reqval = build_request(inv_id, *TOPICS["daily_yield"])
    elif args.total_yield:     reqval = build_request(inv_id, *TOPICS["total_yield"])
    elif args.time:            reqval = build_request(inv_id, *TOPICS["time"])
    elif args.serial_number:   reqval = build_request(inv_id, *TOPICS["serial"])
    elif args.versions:        reqval = build_ping(inv_id)
    elif args.bootup_timestamp: reqval = build_request(inv_id, *TOPICS["bootup_ts"])
    else:                      reqval = build_request(inv_id, *TOPICS["total_yield"])

    value = getStecaGridResult(port, reqval)

    if value is not None:
        if args.grid_meas and isinstance(value, list) and value and isinstance(value[0], tuple):
            for lbl, vals in value:
                vals_str = "  ".join(
                    f"{v[0]:.2f} {v[1]}" if uom else f"{v[0]:.2f}" for v in vals
                )
                print(f"{lbl}: {vals_str}")
        elif args.bootup_timestamp and isinstance(value, list) and len(value) == 2:
            boot_time, uptime_str = value
            print(f"Boot time: {boot_time}  ({uptime_str})")
        elif isinstance(value, list) and len(value) == 2:
            print(f"{value[0]} {value[1]}" if uom else str(value[0]))
        else:
            print(value)

    port.close()
