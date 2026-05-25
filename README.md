# StecaGrid 3600 RS485 Tools
Tools to read out and decode data from a StecaGrid 3600 solar inverter via RS485.
Developed and tested against firmware from 2013 (see [firmware versions](#based-on-versions)).

## The Protocol
A proprietary request/response protocol over RS485, used by the StecaGrid SEM energy manager
to communicate with StecaGrid inverters. Newer inverter models have an XML/HTTP API instead.

### Serial Parameters
| Parameter | Value |
|-----------|-------|
| Baudrate  | **38400** |
| Data bits | 8 |
| Parity    | None |
| Stop bits | 1 |
| Connector | RJ45 (RS485 A/B/GND, **not** Ethernet) |

### RS485 Addresses
| Address | Device |
|---------|--------|
| `0x01`  | Inverter (default) |
| `0x7b`  | SEM sender ID used by this tool |
| `0xc9`  | StecaGrid User 4.4 (SEM software) |
| `0x65`  | StecaGrid SEM energy manager hardware |

### Frame Structure
```
[02] [01] [00] [LEN] [TO] [FROM] [CRC1] [payload...] [CRC2_HI] [CRC2_LO] [03]
  0    1    2    3    4     5      6       7 .. -4       -3         -2      -1

LEN = total frame length including STX (0x02) and ETX (0x03)
```
- **STX** `0x02`, **ETX** `0x03`
- **LEN** big-endian uint16 at bytes [2:4] = total frame length
- **TO / FROM** RS485 device IDs
- **CRC1** covers bytes `[0:6]` — see [CRC section](#crc)
- **CRC2** is the last 2 bytes before ETX — see [CRC section](#crc)
- **Payload** starts at byte 7: `[cmd, auth, dlen_hi, dlen_lo, topic, data..., chk]`
  - `auth` = authorization level byte (`0x03` = Administrator)
  - `dlen` = length of `[topic, data...]` before `chk`
  - `chk` = `(0x55 + sum([topic, data...])) & 0xFF`

### Service Code Table
| Request | Name                  | Response |
|---------|-----------------------|----------|
| `0x11`  | Reset                 | —        |
| `0x20`  | ReadIdentification    | `0x21`   |
| `0x22`  | ReadDiagnosticServices| `0x23`   |
| `0x30`  | ReadErrorBuffer       | `0x31`   |
| `0x32`  | ReadErrorBufferEnvData| `0x33`   |
| `0x34`  | ClearErrorBuffer      | `0x35`   |
| `0x40`  | ReadDataById          | `0x41`   |
| `0x50`  | WriteDataById         | `0x51`   |
| `0x54`  | GetDataById           | `0x55`   |
| `0x60`  | DownloadById          | `0x61`   |
| `0x64`  | UploadById            | `0x65`   |
| `0x68`  | UploadInternById      | `0x69`   |
| `0x70`  | BootloaderConnect     | `0x71`   |

### Authorization Levels
`0`=User, `1`=Service, `2`=Development, `3`=Administrator.
The software operates at Administrator level by default.

---

## Topic Map

### Inverter reads (TO=`0x01`)
| Topic  | Service       | Content                                      |
|--------|---------------|----------------------------------------------|
| `0x05` | Upload (R/W)  | Time (`YY MM DD HH MM SS`, year offset 2000) |
| `0x08` | Upload        | Bootup timestamp (ms since boot, BE uint32)  |
| `0x09` | UploadIntern  | Serial number (ASCII)                        |
| `0x1d` | Read          | Nominal power                                |
| `0x22` | Read          | Panel power (DC)                             |
| `0x23` | Read          | Panel voltage (DC)                           |
| `0x24` | Read          | Panel current (DC)                           |
| `0x29` | Read          | AC power                                     |
| `0x32` | Get           | Country code                                 |
| `0x33` | Get           | Country code list                            |
| `0x3c` | Read          | Daily yield                                  |
| `0x51` | Read          | Grid measurements ENS1+ENS2                 |
| `0x52` | Read          | Grid measurements L2                         |
| `0x53` | Read          | Grid measurements L3                         |
| `0x5a` | UploadIntern  | Event log page 1 (~860 B, up to 20 entries)  |
| `0x5b` | UploadIntern  | Event log page 2 (most recent entries)       |
| `0xef` | Upload        | All yearly yields (float array)              |
| `0xf1` | Upload (R/W)  | Total yield (IEEE 754 LE float, Wh)          |

### Historical yield — all UploadById (`0x64`), TO=`0x01`
Index 0 = most recent period, index N = N periods ago.

| Series       | Count | Topic IDs | Index 0 |
|--------------|-------|-----------|---------|
| DayCurves    | 31    | `0x7b, 0x75, 0x6f, 0x69, 0x63, 0x5d, 0x57,` then `0x93`..`0x7c` | today |
| DayValues    | 13    | `0xbf, 0xbd, 0xbb, 0xb9, 0xb7, 0xb5, 0xb3, 0xb1, 0xaf, 0xad, 0xab, 0xa9, 0xa8` | this month |
| MonthValues  | 20    | `0xe0`..`0xcd` (descending) | this year |
| YearValues   | 1     | `0xef` | all years |

### SEM reads/writes (TO=`0x65`)
| Topic  | Service       | Content                          |
|--------|---------------|----------------------------------|
| `0x0a` | Upload (R/W)  | EnergyManager config (~87 bytes) |
| `0x0b` | Upload        | Relais history                   |
| `0x0d` | Upload        | EnergyManager live measurements  |

---

## Write Operations

### Direct to inverter (TO=`0x01`)
| Cmd    | Topic  | Data                     | Effect                     |
|--------|--------|--------------------------|----------------------------|
| `0x11` | —      | (no payload)             | Reset inverter             |
| `0x50` | `0x01` | `uint32 = 0x55555555`    | Factory reset ⚠️           |
| `0x50` | `0x0b` | `uint32 = countryCode`   | Set country code           |
| `0x50` | `0x0b` | `uint32 = 0xFFFF`        | Delete country code        |
| `0x50` | `0xff` | `uint32 = newAddr`       | Set inverter RS485 address |
| `0x60` | `0x05` | `[YY MM DD HH MM SS]`    | Set time                   |
| `0x60` | `0xf1` | `float32_LE × 1000`      | Set total yield            |

### To SEM (TO=`0x65`)
| Cmd    | Topic  | Data                | Effect                  |
|--------|--------|---------------------|-------------------------|
| `0x60` | `0x0a` | 87-byte EM payload  | Set EnergyManager config |

### EnergyManager payload structure (87 bytes, big-endian)
```
[0]     uint8:  payload_version (= 0)
[1-2]   int16:  S0PulsesPerkWh
[3]     uint8:  DeratingMode  (0=Off, 1=RippleControl, 2=PowerLimit, 3=EasyBox)
[4-35]  16×int16: DeratingPatterns[16]  (-1 = disabled)
[36-39] uint32: NominalPowerW
[40-43] uint32: DeratingPowerLimitW       ← power limit in watts
[44-45] uint16: PID Kp
[46-47] uint16: PID Ki
[48-49] uint16: PID Kd
[50-51] uint16: PeriodeMin_s
[52-53] uint16: PeriodeMax_s
[54-55] uint16: Limit_Permill
[56]    uint8:  RelaisMode  (0=Off, 1=InputPattern, 2=MinPower, 3=MinPower_SmartGrid)
[57-58] uint16: RelaisPatterns bitfield
[59-62] uint32: Activation.ThresholdPower_W
[63-66] uint32: Deactivation.ThresholdPower_W
[67-68] uint16: Activation.ThresholdDerating_Permille
[69-70] uint16: Deactivation.ThresholdDerating_Permille
[71-74] uint32: Activation.Latency_s
[75-78] uint32: Deactivation.Latency_s
[79-82] uint32: Activation.HoldTime_s
[83-86] uint32: Deactivation.HoldTime_s
```

### Active-power setpoint (EMLiveMeas)

Write(0x50) on Topic `0x0d`, TO=`0x01` (inverter), FROM=`0x7b` (SEM sender).
Verified against live hardware across all four relay levels.

**Promille encoding** — 16-bit big-endian, 0..1000 (= 0.0 %..100.0 %, 0.1 % resolution):

| Relay level | Percent | Promille | `<hi> <lo>` |
|-------------|---------|----------|-------------|
| K1          | 0 %     | 0        | `00 00`     |
| K2          | 30 %    | 300      | `01 2C`     |
| K3          | 60 %    | 600      | `02 58`     |
| K4          | 100 %   | 1000     | `03 E8`     |

**Frame body** (between CRC1 and CRC2):
```
50 03 00 05 0d 00 ff <hi> <lo> <chk>
```

**CHK formula** — additive 8-bit, covers body excluding the `0x03` auth byte:
```python
chk = (0x50 + 0x05 + 0x0D + 0x00 + 0xFF + hi + lo) & 0xFF
```

**Usage:**
```python
from steca_setpoint import build_setpoint, build_setpoint_percent, EM_LEVELS

frame = build_setpoint(300)                  # 30 %
frame = build_setpoint_percent(60.0)         # 60 %
frame = build_setpoint(EM_LEVELS["K2"] * 10) # K2 = 30 % → 300 ‰
```

**Operational notes:**
- **Do not send while a physical SEM is connected to address `0x01`** — two-master collision on the RS485 bus; frames will corrupt each other.
- The inverter discards the setpoint after a **timeout** (safe-state fallback, typically a few seconds). The caller must **repeat the frame periodically** to maintain the setpoint. A periodic sender loop is implemented separately.

---

## Data Encoding

**Steca proprietary float** (4 bytes: `[unit, b1, b2, b3]`):
```python
iacpower = ((b3 << 8 | b1) << 8 | b2) << 7
value, = struct.unpack('f', struct.pack('I', iacpower & 0xFFFFFFFF))
```
Unit byte: `0x05`=V, `0x07`=A, `0x09`=Wh, `0x0B`=W, `0x0D`=Hz, `0x00`=NUL

**Total Yield** (4 bytes little-endian IEEE 754 float, Wh):
```python
bits = b[3]<<24 | b[2]<<16 | b[1]<<8 | b[0]
value, = struct.unpack('f', struct.pack('I', bits))
```

**Bootup Timestamp** (topic `0x08`, Upload response):
```python
ms = struct.unpack('>I', payload[5:9])[0]   # BE uint32 milliseconds
boot_time = datetime.now() - timedelta(milliseconds=ms)
```

**Event log entries** contain null-terminated ASCII strings preceded by 6-byte
timestamps (`YY MM DD HH MM SS`, year offset 2000).
The first byte of the payload data is the total event count (ring buffer).

---

## CRC

Both CRC algorithms were fully solved by combining passive RS485 sniffing
(101 ping frames, 68+ data request frames) with cross-referencing the
independent implementation by MichaelOE:
[homeassistant-stecagrid/steca.py](https://github.com/MichaelOE/homeassistant-stecagrid/blob/main/custom_components/stecagrid/steca.py)

Both use a **nibble-based lookup table** approach (not standard CRC polynomials).

```python
CRC8_TABLE  = [0x00, 0x8F, 0x27, 0xA8, 0x4E, 0xC1, 0x69, 0xE6,
               0x9C, 0x13, 0xBB, 0x34, 0xD2, 0x5D, 0xF5, 0x7A]
CRC16_TABLE = [0x0000, 0xACAC, 0xEC05, 0x40A9, 0x6D57, 0xC1FB, 0x8152, 0x2DFE,
               0xDAAE, 0x7602, 0x36AB, 0x9A07, 0xB7F9, 0x1B55, 0x5BFC, 0xF750]

def crc8_nibble(data: bytes, init: int = 0x55) -> int:
    crc = init
    for b in data:
        crc ^= b
        crc = (crc >> 4) ^ CRC8_TABLE[crc & 0x0F]
        crc = (crc >> 4) ^ CRC8_TABLE[crc & 0x0F]
    return crc & 0xFF

def crc16_nibble(data: bytes, init: int = 0x5555) -> int:
    crc = init
    for b in data:
        crc ^= b
        crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x000F]
        crc = (crc >> 4) ^ CRC16_TABLE[crc & 0x000F]
    return crc & 0xFFFF

def build_frame(to: int, frm: int, payload: bytes) -> bytes:
    total_len = len(payload) + 10
    header = bytes([0x02, 0x01, total_len >> 8, total_len & 0xFF, to, frm])
    c1     = crc8_nibble(header)
    body   = header + bytes([c1]) + payload
    c2     = crc16_nibble(body + b'\x03')
    return body + bytes([c2 >> 8, c2 & 0xFF, 0x03])
```

### CRC1 — **Fully solved** ✓
```python
crc1 = crc8_nibble(frame[0:6], init=0x55)
```
Covers frame bytes `[0:6]` (STX through FROM). Verified against all known frames.

### CRC2 — **Fully solved** ✓
```python
crc2 = crc16_nibble(frame[:-3] + b'\x03', init=0x5555)
```
Covers the entire frame **excluding** the two CRC2 bytes, **including** ETX.
Verified against all known frame types: ping, read (`0x40`/`0x64`/`0x68`), write
(`0x34`/`0x50`/`0x60`), and responses.

---

## Captured Reference Frames (SEM=`0x7b`, inverter ID `0x01`)

```python
SG_VERSIONS      = bytes.fromhex("0201000c017bc62003798c03")
SG_NOMINAL_POWER = bytes.fromhex("02010010017bb5400300011d72309503")
SG_PANEL_POWER   = bytes.fromhex("02010010017bb540030001227712ee03")
SG_PANEL_VOLTAGE = bytes.fromhex("02010010017bb540030001237878e403")
SG_PANEL_CURRENT = bytes.fromhex("02010010017bb5400300012479a0b603")
SG_AC_POWER      = bytes.fromhex("02010010017bb540030001297e985b03")
SG_DAILY_YIELD   = bytes.fromhex("02010010017bb5400300013c91e1c903")
SG_TIME          = bytes.fromhex("02010010017bb564030001055a3a4403")
SG_SERIAL        = bytes.fromhex("02010010017bb564030001095e856e03")
SG_TOTAL_YIELD   = bytes.fromhex("02010010017bb564030001f146cc7903")
```

---

## StecaGridController.py
Reads/writes inverter data via RS485. All frames synthesized from `steca_crc.py`.

### Install
```bash
pip3 install pyserial
```

### Usage
```
usage: StecaGridController.py [-h] [-v] [-u] [-s SERIAL] [--id ID]
                           [-np] [-pp] [-pv] [-pc] [-ap] [-gm] [-el]
                           [-dy] [-ty] [-ti] [-sn] [-ve]
                           [--bootup-timestamp]
                           [--10min-history [N]] [--daily-history [N]]
                           [--monthly-history [N]] [--yearly-history]
                           [--discover] [--full-scan]
                           [--set-time DATETIME] [--sync-time] [--DST]
                           [--set-power-limit WATTS]

Read options:
  -ap   AC power (W)
  -dy   Daily yield (Wh)
  -ty   Total yield (Wh)
  -pp   Panel power (W)
  -pv   Panel voltage (V)
  -pc   Panel current (A)
  -np   Nominal power (W)
  -ti   Inverter time
  -sn   Serial number
  -ve   Firmware versions
  -gm   Grid measurements (ENS1 + ENS2)
  -el   Event log (both pages)
  --bootup-timestamp   Inverter boot time (topic 0x08)

Historical yield (UploadById, index 0 = most recent):
  --10min-history [N]    10-minute power curve (0=today, max 30)
  --daily-history [N]    Daily yield totals for month (0=this month, max 12)
  --monthly-history [N]  Monthly yield totals for year (0=this year, max 19)
  --yearly-history       All yearly yield totals

Discovery:
  --discover    Scan RS485 bus (IDs 0x01..0x0a)
  --full-scan   With --discover: scan 0x01..0x65

Clock:
  --set-time DATETIME   Set inverter clock ("YYYY-MM-DD HH:MM:SS").
                        The Steca has no DST — pass standard/winter time by default.
  --sync-time           Sync inverter clock to system time.
                        Subtracts 1 h during DST season (standard time) unless --DST.
  --DST                 Use with --set-time / --sync-time: send summer/DST time
                        instead of converting to standard/winter time.

Write / control:
  --set-power-limit WATTS
      Read EnergyManager config from SEM (0x65), set DeratingMode=PowerLimit
      and DeratingPowerLimitW=WATTS, write back. Requires SEM connected.
  --setpoint PERMILLE
      Send active-power setpoint in permille (0..1000) directly to inverter.
      WARNING: do not use with physical SEM on bus; repeat periodically.
  --setpoint-percent PERCENT
      Like --setpoint but in percent (0.0..100.0, 0.1 % resolution).
```

### Write ACK response codes
All write operations (`0x50`/`0x60`) return a status byte decoded as:

| Code | Name |
|------|------|
| `0x00` | Ok |
| `0x01` | ServiceNotSupported |
| `0x02` | RequestOutOfRange |
| `0x08` | NoCorrectRequest |
| `0x09` | Busy |
| `0x0a` | ReceivedDataInvalid |
| `0x0f` | NoResponse |
| `0x10` | Error |

### Examples
```bash
$ python3 StecaGridController.py -ty -u
52978840.0 Wh

$ python3 StecaGridController.py --bootup-timestamp
Boot time: 2026-05-13 05:30:12  (24048000 ms uptime)

$ python3 StecaGridController.py --sync-time
Syncing inverter clock to 2026-05-14 21:30:00 (no DST correction needed)
OK
Inverter time: 2026-05-14 21:30:01

$ python3 StecaGridController.py --sync-time
Syncing inverter clock to 2026-05-14 21:30:00 (DST active → converted to standard/winter time)
OK
Inverter time: 2026-05-14 21:30:01

$ python3 StecaGridController.py --sync-time --DST
Syncing inverter clock to 2026-05-14 22:30:00 (DST mode — using local/summer time)
OK
Inverter time: 2026-05-14 22:30:01

$ python3 StecaGridController.py --set-time "2026-05-14 21:30:00"
Setting inverter time to 2026-05-14 21:30:00  (standard/winter time)
OK
Inverter time: 2026-05-14 21:30:01

$ python3 StecaGridController.py --10min-history
10-min history: 2026-05-14  (today)
──────────────────────────────────
  06:20         6 Wh
  06:30        12 Wh
  ...
  21:40        18 Wh
──────────────────────────────────
  Total:    8,169 Wh

$ python3 StecaGridController.py --daily-history
Daily history: May 2026
──────────────────────────
  2026-05-01    14,700 Wh
  2026-05-02    26,040 Wh
  ...
──────────────────────────
  Total:       193,270 Wh

$ python3 StecaGridController.py --monthly-history
Monthly history: 2026
─────────────────────────
  Jan      78,610 Wh
  Feb     109,380 Wh
  ...
─────────────────────────
  Total    563,860 Wh

$ python3 StecaGridController.py --yearly-history
Yearly history
──────────────────────
  2014    9,500,000 Wh
  2015   12,300,000 Wh
  ...
  2026       32,300 Wh
──────────────────────
  Total  154,132,300 Wh

$ python3 StecaGridController.py --set-power-limit 2000
Reading EnergyManager config from SEM (0x65)...
Writing power limit 2000 W to SEM...
OK

$ python3 StecaGridController.py --discover --full-scan
StecaGrid RS485 Bus Discovery
  Scanning: 101 IDs (0x01..0x65)
  0x01  ✓ found  Serial: XXXXXXXXXXXXXXXXXXXX
Result: 1 inverter(s) on bus.
```

---

## steca_sniffer.py
Passive RS485 bus sniffer. Monitors all traffic between StecaGrid User software
and the inverter.

Features:
- CRC1 and CRC2 verification for all frame types (nibble-table)
- Decodes all known read responses including GridMeasurements, EventLog, BootupTimestamp
- Decodes write operations: `0x50` WriteDataById, `0x60` DownloadById (SetTime, EMConfig)
- Decodes EnergyManager config reads/writes (SEM address `0x65`)
- JSON log for offline analysis
- Threaded UART reader (no frame loss at 38400 baud)

### Install
```bash
pip3 install pyserial
```

### Usage
```bash
python3 steca_sniffer.py --port /dev/ttyUSB0
python3 steca_sniffer.py --port /dev/ttyUSB0 --verbose
python3 steca_sniffer.py --port /dev/ttyUSB0 --no-log
```

### Example output
```
[00:06:27] RESPONSE  TO=0xc9 FROM=0x01  LEN=860  StecaUser-4.4
  Topic:   0x5a EventLog_p1
  CRC1:0x01[✓]  CRC2:0x0024[✓]  model=nibble_crc16
  → event_log(p1): 74 total, 20 entries

[00:07:01] →SEM  TO=0x65 FROM=0x7b  LEN=103  SEM-7b
  Topic:   0x0a EMConfig
  CRC1:0xe3[✓]  CRC2:0x1a2b[✓]  model=nibble_crc16
  → SetEMConfig mode=PowerLimit(2) limit=2000W nominal=3600W
```

---

## Based on Versions
Tested with the following firmware. Your mileage may vary.
```
StecaGrid 3600  Serial: XXXXXXXXXXXXXXXXXXXX

HMI BFAPI   5.0.0   19.03.2013 14:38:59
HMI FBL     2.0.3   05.04.2013 11:46:20
HMI APP     15.0.0  26.07.2013 13:19:06
HMI PAR     0.0.1   26.07.2013 13:19:06
HMI OEM     0.0.1   11.06.2013 08:11:29
PU BFAPI    5.0.0   19.03.2013_14:38:42
PU FBL      1.0.1   19.12.2012_16:36:04
PU APP      4.0.0   03.05.2013_09:37:55
PU PAR      3.0.0   31.01.2013_13:47:24
ENS1 BFAPI  5.0.0   19.03.2013_14:38:51
ENS1 FBL    1.0.1   19.12.2012_16:34:47
ENS1 APP    39.0.0  11.07.2013_14:39:50
ENS1 PAR    0.0.14  11.07.2013_14:40:03
ENS2 BFAPI  5.0.0   19.03.2013_14:38:51
ENS2 FBL    1.0.1   19.12.2012_16:34:47
ENS2 APP    39.0.0  11.07.2013_14:39:50
ENS2 PAR    0.0.14  11.07.2013_14:40:03
HMI / PU / ENS2 — Net11
```

---

## Open Topics
- **EnergyManager payload endianness**: assumed big-endian; unverified without a live SEM capture.
- **Historical yield float encoding**: verified against live inverter (StecaGrid 3600):
  DayCurve slots `× 6 → Wh`; DayValues / MonthValues / YearValues `round(f) → Wh`.
  Leading all-zero 4-byte groups are padding and are skipped before decoding.
- **SEM live measurements** (topic `0x0d`): structure unknown.
- **`--set-power-limit`**: requires a connected SEM; untested on hardware.

---

## Disclaimer
Ich übernehme keine Garantie oder Gewährleistung für die Nutzung dieser Software.
Verwendung auf eigene Gefahr.
