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
| Request  | `0x34` | Unknown (seen before event log requests) |
| Request  | `0x40` | Request type A (measurement values) |
| Request  | `0x54` | Request type D |
| Request  | `0x60` | Request type E |
| Request  | `0x64` | Request type B (yield, time, serial) |
| Request  | `0x68` | Request type C (event log, serial detail) |
| Response | `0x21` | Versions response |
| Response | `0x35` | Unknown response to `0x34` |
| Response | `0x41` | Response type A |
| Response | `0x55` | Response type D |
| Response | `0x61` | Response type E |
| Response | `0x65` | Response type B |
| Response | `0x69` | Response type C |

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
    header = bytes([0x02, 0x01, 0x00, total_len, to, frm])
    c1     = crc8_nibble(header)
    body   = header + bytes([c1]) + payload
    c2     = crc16_nibble(body + b'\x03')
    return body + bytes([c2 >> 8, c2 & 0xFF, 0x03])
```

### CRC1 — **Fully solved** ✓
```python
crc1 = crc8_nibble(frame[0:6], init=0x55)
```
Covers frame bytes `[0:6]` (STX through FROM byte).
Verified against all known frames.

### CRC2 — **Fully solved** ✓
```python
crc2 = crc16_nibble(frame[:-3] + b'\x03', init=0x5555)
```
Covers the entire frame **excluding** the two CRC2 bytes,
**including** the ETX byte `0x03`.

Verified against all known frame types: ping, read (`0x40`/`0x64`/`0x68`), write (`0x34`), responses.

---

## Request Frames

All frames below use SEM ID `0x7b`. CRC values are computed by `steca_crc.py`.

### Captured reference frames (SEM=`0x7b`, inverter ID `0x01`)
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
                           [-gm] [-el] [--power-limit {0,1,2,3}]
                           [--discover] [--full-scan]

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
  -el  Event log (both pages)
  -u   Show unit of measurement
  -s   Serial port (default /dev/ttyS0)
  -v   Verbose output
  --power-limit {0,1,2,3}
       Send power limit frame: 0=100%, 1=60%, 2=30%, 3=0% (experimental)
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
  0x01  ✓ found  Serial: XXXXXXXXXXXXXXXXXXXX
  ...
Result: 1 inverter(s) on bus.
```

---

## steca_sniffer.py
Passive RS485 bus sniffer. Monitors all traffic between StecaGrid User software
and the inverter. Uses a dedicated reader thread to avoid losing bytes from large
frames (e.g. event log responses, ~860 bytes).

Features:
- CRC1 and CRC2 verification for all frame types (nibble-table, no exceptions)
- Decodes all known topics including GridMeasurements and EventLog (both pages)
- JSON log for offline analysis
- Threaded UART reader (no frame loss at 38400 baud)

### Install
```bash
pip3 install pyserial
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
  CRC1:0x01[✓]  CRC2:0x0024[✓]  model=nibble_crc16
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
- **cmd=`0x34`/`0x35`** (12-byte frames seen immediately before event log requests): purpose unknown, CRC2 not yet verified experimentally. `build_power_limit_frame(step)` in `getStecaGridData.py` sends `cmd=0x34` with `sub=step` as a hypothesis for power limiting — unverified.
- **Write / control frames** (power limitation via StecaGrid SEM): not yet captured. Requires running StecaGrid User 4.4 with sniffer while activating feed-in management.

---

## Disclaimer
Ich übernehme keine Garantie oder Gewährleistung für die Nutzung dieser Software.
Verwendung auf eigene Gefahr.
