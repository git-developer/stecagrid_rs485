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

### Frame Structure
```
[02] [01] [00] [LEN] [TO] [FROM] [CRC1] [payload...] [CRC2_HI] [CRC2_LO] [03]
  0    1    2    3    4     5      6       7 .. -4       -3         -2      -1

LEN = total frame length including STX (0x02) and ETX (0x03)
```
- **STX** `0x02`, **ETX** `0x03`
- **LEN** big-endian uint16 at bytes [2:4] = total frame length
- **TO / FROM** RS485 device IDs (inverter = `0x01`, SEM = `0x7b` or `0xc9`)
- **CRC1** covers bytes `[0:6]` — see [CRC section](#crc)
- **CRC2** is the last 2 bytes before ETX — see [CRC section](#crc)
- **Payload** starts at byte 7; first byte = command, byte 5 = topic

### Command Bytes
| Direction | Cmd | Meaning |
|-----------|-----|---------|
| Request  | `0x20` | Ping / bus discovery |
| Request  | `0x40` | Request type A (measurement values) |
| Request  | `0x54` | Request type D |
| Request  | `0x60` | Request type E |
| Request  | `0x64` | Request type B (yield, time, serial) |
| Request  | `0x68` | Request type C (event log, serial detail) |
| Response | `0x41` | Response type A |
| Response | `0x55` | Response type D |
| Response | `0x61` | Response type E |
| Response | `0x65` | Response type B |
| Response | `0x69` | Response type C |
| Response | `0x21` | Versions response |

### Known Topics
| Topic | Name | Cmd | Unit / Format |
|-------|------|-----|---------------|
| `0x05` | Time | `0x64` / `0x65` | `YY MM DD HH MM SS` (year offset 2000) |
| `0x08` | Mystery_08 | `0x64` / `0x65` | Internal counter / state (unknown) |
| `0x09` | Serial | `0x68` / `0x69` | ASCII string + sub-topic manifest |
| `0x1d` | NominalPower | `0x40` / `0x41` | Steca float, W |
| `0x20` | Versions | `0x20` / `0x21` | Firmware version strings |
| `0x22` | PanelPower | `0x40` / `0x41` | Steca float, W |
| `0x23` | PanelVoltage | `0x40` / `0x41` | Steca float, V |
| `0x24` | PanelCurrent | `0x40` / `0x41` | Steca float, A |
| `0x29` | ACPower | `0x40` / `0x41` | Steca float, W |
| `0x3c` | DailyYield | `0x40` / `0x41` | Steca float, Wh |
| `0x51` | GridMeasurements | `0x40` / `0x41` | ENS1+ENS2 label + 4× Steca float |
| `0x5a` | EventLog page 1 | `0x68` / `0x69` | Event entries (large frame, ~860 B) |
| `0x5b` | EventLog page 2 | `0x68` / `0x69` | Event entries (recent / oldest) |
| `0xf1` | TotalYield | `0x64` / `0x65` | IEEE 754 float LE, Wh |

### Data Encoding
**Steca proprietary float** (4 bytes: `[unit] [b1] [b2] [b3]`):
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

**Event log entries** contain null-terminated ASCII strings preceded by two 6-byte
timestamps (`YY MM DD HH MM SS`, year offset 2000).  
The first byte of the payload data is the total event count (ring buffer).  
Topic `0x5a` = bulk history (~20 entries, large frame); topic `0x5b` = most recent entries.

### Bus Discovery / Ping
The SEM scans all 101 RS485 IDs (`0x01`..`0x65`) using 12-byte ping frames (cmd `0x20`).
Only responding devices are queried further. On a single-inverter system, only ID `0x01` responds.

---

## CRC

### CRC1 — **Fully solved** ✓
Covers frame bytes `[0:6]` (STX through FROM byte).
```python
import crcmod
crc1_fn = crcmod.mkCrcFun(0x139, initCrc=0xAA, rev=True, xorOut=0x00)
crc1 = crc1_fn(frame[0:6])
```
| Parameter | Value |
|-----------|-------|
| Polynomial | `0x39` (0x139 with implicit leading 1) |
| Init | `0xAA` |
| Reflected input | Yes |
| Reflected output | Yes |
| XOR out | `0x00` |

### CRC2 — **Fully solved via GF(2) linear model** ✓
CRC2 is a **GF(2) linear function** of the frame bytes — not a standard CRC polynomial.
It was reverse-engineered from 101 ping frames and 68 data request frames captured
by passive RS485 sniffing of StecaGrid User 4.4 (SEM ID `0xc9`).

#### Ping frames (cmd=`0x20`, 12 bytes)
```python
M_COL_PING = [0x39b2, 0x7364, 0xe6c8, 0x78cd,
              0xf19a, 0x5669, 0xacd2, 0x0000]
BASE_PING   = 0xf6e5
OFFSET_7b   = 0xb6db   # XOR to convert SEM=0xc9 → SEM=0x7b

def calc_crc2_ping(to_id: int, sem_id: int = 0xc9) -> int:
    crc2 = BASE_PING
    for bit in range(8):
        if to_id & (1 << bit):
            crc2 ^= M_COL_PING[bit]
    if sem_id == 0x7b:
        crc2 ^= OFFSET_7b
    return crc2
```
Verified: **101/101 frames correct** (all IDs `0x01`..`0x65`).

#### 16-byte data requests (cmd=`0x40` / `0x64`, TO=`0x01`)
```python
T_REF     = 0x05
CRC2_REF  = 0x8ba1   # CRC2 for T=0x05, cmd=0x64, SEM=0xc9
M_COL_64  = [
    0x87c7, 0x72a3, 0x2d36, 0x5a6c,   # topic bits 0-3
    0xb4d8, 0xdced, 0x0c87, 0x190e,   # topic bits 4-7
    0x0000, 0x0000, 0xc870, 0x25bd,   # chk bits 0-3  (chk = topic + 0x55)
    0x4b7a, 0x96f4, 0x98b5, 0x8437,   # chk bits 4-7
]
OFFSET_40 = 0x572c   # XOR for cmd=0x40 vs cmd=0x64
OFFSET_7b = 0xb1e5   # XOR for SEM=0x7b vs SEM=0xc9

def calc_crc2_request16(topic: int, cmd: int = 0x64,
                        sem_id: int = 0xc9) -> int:
    chk_ref = (T_REF + 0x55) & 0xFF
    chk     = (topic + 0x55) & 0xFF
    crc2    = CRC2_REF
    for bit in range(8):
        if ((topic ^ T_REF) >> bit) & 1: crc2 ^= M_COL_64[bit]
        if ((chk ^ chk_ref) >> bit) & 1: crc2 ^= M_COL_64[8 + bit]
    if cmd == 0x40:    crc2 ^= OFFSET_40
    if sem_id == 0x7b: crc2 ^= OFFSET_7b
    return crc2
```
Verified: **68/68 topics correct** for cmd=`0x64`; **9/9 correct** for cmd=`0x40`.

---

## Request Frames
All frames below use SEM ID `0x7b`. CRC2 values are derived from the linear model.

### Synthesized (CRC2 computed, any topic supported)
```python
# Use calc_crc2_request16(topic, cmd, sem_id) from above
# Use calc_crc2_ping(to_id, sem_id) for bus discovery
```

### Captured reference frames (SEM=`0x7b`, inverter ID `0x01`)
```python
SG_VERSIONS      = bytes.fromhex("02010 00c017bc6200379 8c03".replace(" ",""))
SG_NOMINAL_POWER = bytes.fromhex("02010010017bb540030001 1d723095 03".replace(" ",""))
SG_PANEL_POWER   = bytes.fromhex("02010010017bb540030001 227712ee 03".replace(" ",""))
SG_PANEL_VOLTAGE = bytes.fromhex("02010010017bb540030001 237878e4 03".replace(" ",""))
SG_PANEL_CURRENT = bytes.fromhex("02010010017bb540030001 2479a0b6 03".replace(" ",""))
SG_AC_POWER      = bytes.fromhex("02010010017bb540030001 297e985b 03".replace(" ",""))
SG_DAILY_YIELD   = bytes.fromhex("02010010017bb540030001 3c91e1c9 03".replace(" ",""))
SG_TIME          = bytes.fromhex("02010010017bb564030001 055a3a44 03".replace(" ",""))
SG_SERIAL        = bytes.fromhex("02010010017bb564030001 095e856e 03".replace(" ",""))
SG_TOTAL_YIELD   = bytes.fromhex("02010010017bb564030001 f146cc79 03".replace(" ",""))
```

---

## getStecaGridData.py
Reads inverter data via RS485 and prints the result. Supports all known topics.
Includes bus discovery using synthesized ping frames.

### Install
```bash
pip3 install pyserial
```

### Usage
```
usage: getStecaGridData.py [-h] [-v] [-u] [-s SERIAL]
                           [-np] [-pp] [-pv] [-pc] [-ap]
                           [-dy] [-ty] [-ti] [-sn] [-ve]
                           [-gm] [--discover] [--full-scan]

optional arguments:
  -ap  AC power (W)
  -dy  Daily yield (Wh)
  -ty  Total yield (Wh)
  -pp  Panel power (W)
  -pv  Panel voltage (V)
  -pc  Panel current (A)
  -np  Nominal power (W)
  -ti  Inverter time
  -sn  Serial number
  -ve  Firmware versions
  -gm  Grid measurements (ENS1 + ENS2 voltage, frequency)
  -u   Show unit of measurement
  -s   Serial port (default /dev/ttyS0)
  -v   Verbose output
  --discover    Scan RS485 bus for inverters (quick: IDs 0x01..0x0a)
  --full-scan   Used with --discover: full scan IDs 0x01..0x65 (~3 min)
```

### Example
```bash
$ python3 getStecaGridData.py -ty -u
52978840.0 Wh

$ python3 getStecaGridData.py --discover --full-scan
StecaGrid RS485 Bus Discovery
  Scanning: 101 IDs (0x01..0x65)
  0x01  ✓ found  Serial: 748613YI005212850029
  ...
Result: 1 inverter(s) on bus.
```

---

## steca_sniffer.py
Passive RS485 bus sniffer. Monitors all traffic between StecaGrid User software
and the inverter. Uses a dedicated reader thread to avoid losing bytes from large
frames (e.g. event log responses, ~860 bytes).

Features:
- CRC1 and CRC2 verification using the solved models
- Decodes all known topics including GridMeasurements and EventLog (both pages)
- JSON log for offline analysis
- Threaded UART reader (no frame loss at 38400 baud)

### Install
```bash
pip3 install pyserial crcmod
```

### Usage
```bash
python3 steca_sniffer.py --port /dev/ttyUSB0
python3 steca_sniffer.py --port /dev/ttyUSB0 --verbose   # + raw hex + assembler debug
python3 steca_sniffer.py --port /dev/ttyUSB0 --no-log    # suppress JSON log
```

### Example output
```
[00:06:27] RESPONSE  TO=0xc9 FROM=0x01  LEN=860  StecaUser-4.4
  Topic:   0x5a EventLog_p1
  CRC1:0x01[✓]  CRC2:0x0024[?]  model=?
  → event_log(p1): 74 total, 20 entries
  EventLog (74 total, 20 in this frame):
      1  2026-01-09 15:27:20  ENS Grid Voltage too low
      2  2024-11-13 21:26:09  ENS Grid Frequency too low
      3  2024-06-14 09:10:30  ENS Grid Frequency too low
     ...
     20  2013-12-23 11:02:00  ENS Grid Frequency too low
```

---

## Based on Versions
Tested with the following firmware. Your mileage may vary.
```
StecaGrid 3600  Serial: 748613YI005212850029

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
- **CRC2 for cmd=`0x68`** frames (event log requests, serial detail): not yet modelled — too few data points. Captured frames can be replayed as-is.
- **Write / control frames** (power limitation via StecaGrid SEM): not yet captured. Requires running StecaGrid User 4.4 with sniffer while activating feed-in management.
- **SEM ID `0x7b` ping frames** beyond ID `0x01`: CRC2 offset constant (`0xb6db`), so `calc_crc2_ping(to_id, 0x7b)` works for any ID.

---

## Disclaimer
Ich übernehme keine Garantie oder Gewährleistung für die Nutzung dieser Software.
Verwendung auf eigene Gefahr.
