#!/usr/bin/env python3
"""
steca_crc.py — CRC and frame builder for StecaGrid RS485 protocol.

CRC1 (byte 6): CRC-8  nibble-table, init=0x55,   covers frame[0:6]
CRC2 (last 3): CRC-16 nibble-table, init=0x5555,  covers frame[:-3] + b'\x03'
               (everything before the CRC2 bytes, with ETX appended)
"""

CRC8_TABLE = [
    0x00, 0x8F, 0x27, 0xA8, 0x4E, 0xC1, 0x69, 0xE6,
    0x9C, 0x13, 0xBB, 0x34, 0xD2, 0x5D, 0xF5, 0x7A,
]

CRC16_TABLE = [
    0x0000, 0xACAC, 0xEC05, 0x40A9, 0x6D57, 0xC1FB, 0x8152, 0x2DFE,
    0xDAAE, 0x7602, 0x36AB, 0x9A07, 0xB7F9, 0x1B55, 0x5BFC, 0xF750,
]


def crc1(frame: bytes) -> int:
    """CRC8 nibble-table, init=0x55, covers frame[0:6]"""
    v = 0x55
    for b in frame[0:6]:
        v ^= b
        v = (v >> 4) ^ CRC8_TABLE[v & 0x0F]
        v = (v >> 4) ^ CRC8_TABLE[v & 0x0F]
    return v


def crc2(frame_without_crc2: bytes) -> int:
    """CRC16 nibble-table, init=0x5555.
    Input: all frame bytes before [CRC2_HI, CRC2_LO, ETX]  (i.e. frame[:-3]).
    ETX (0x03) is appended internally."""
    v = 0x5555
    for b in frame_without_crc2 + b'\x03':
        v ^= b
        v = (v >> 4) ^ CRC16_TABLE[v & 0x000F]
        v = (v >> 4) ^ CRC16_TABLE[v & 0x000F]
    return v


def build_frame(to: int, frm: int, payload: bytes) -> bytes:
    """Build a complete valid Steca RS485 frame with correct CRC1 and CRC2.

    payload: bytes between CRC1 and the trailing [CRC2_HI, CRC2_LO, ETX].
    For a ping:      payload = b'\\x20\\x03'
    For a request16: payload = bytes([cmd, 0x03, 0x00, 0x01, topic, chk])
    """
    total = 10 + len(payload)   # 4 hdr + 2 addr + 1 crc1 + payload + 2 crc2 + 1 etx
    hdr   = bytes([0x02, 0x01, total >> 8, total & 0xFF, to, frm])
    c1    = crc1(hdr)
    pre   = hdr + bytes([c1]) + payload
    c2    = crc2(pre)
    return pre + bytes([c2 >> 8, c2 & 0xFF, 0x03])


# ── Self-test ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # All known good frames: (description, raw_hex_no_spaces)
    _KNOWN = [
        # Ping / bus-discovery (SEM=0x7b)
        ("ping SEM=0x7b",
         "0201000c017bc62003798c03"),
        # RequestA (cmd=0x40, SEM=0x7b)
        ("nominal_power 0x40 SEM=0x7b",
         "02010010017bb5400300011d72309503"),
        ("panel_power 0x40 SEM=0x7b",
         "02010010017bb540030001227712ee03"),
        ("panel_voltage 0x40 SEM=0x7b",
         "02010010017bb540030001237878e403"),
        ("panel_current 0x40 SEM=0x7b",
         "02010010017bb5400300012479a0b603"),
        ("ac_power 0x40 SEM=0x7b",
         "02010010017bb540030001297e985b03"),
        ("daily_yield 0x40 SEM=0x7b",
         "02010010017bb5400300013c91e1c903"),
        # RequestB (cmd=0x64, SEM=0x7b)
        ("time 0x64 SEM=0x7b",
         "02010010017bb564030001055a3a4403"),
        ("serial 0x64 SEM=0x7b",
         "02010010017bb564030001095e856e03"),
        ("total_yield 0x64 SEM=0x7b",
         "02010010017bb564030001f146cc7903"),
        # RequestC (cmd=0x68, SEM=0xc9) — from sniffer captures
        ("event_log_p1 0x68 SEM=0xc9",
         "0201001001c96568030001 5aaf564903".replace(" ", "")),
        ("event_log_p2 0x68 SEM=0xc9",
         "0201001001c9656803000 15bb0773903".replace(" ", "")),
        ("serial_detail 0x68 SEM=0xc9",
         "0201001001c965680300 0109 5eda7e03".replace(" ", "")),
    ]

    # Tidy: strip any remaining spaces that crept into the strings above
    KNOWN = [(desc, h.replace(" ", "")) for desc, h in _KNOWN]

    passed = failed = 0
    for desc, hexstr in KNOWN:
        frame = bytes.fromhex(hexstr)
        # Verify CRC1
        got_c1  = crc1(frame)
        want_c1 = frame[6]
        c1_ok   = got_c1 == want_c1

        # Verify CRC2
        got_c2  = crc2(frame[:-3])
        want_c2 = (frame[-3] << 8) | frame[-2]
        c2_ok   = got_c2 == want_c2

        # Verify build_frame reproduces the frame exactly
        to_id   = frame[4]
        frm_id  = frame[5]
        payload = frame[7:-3]
        built   = build_frame(to_id, frm_id, payload)
        build_ok = built == frame

        ok = c1_ok and c2_ok and build_ok
        status = "PASS" if ok else "FAIL"
        if ok:
            passed += 1
        else:
            failed += 1
            details = []
            if not c1_ok:   details.append(f"CRC1 got=0x{got_c1:02x} want=0x{want_c1:02x}")
            if not c2_ok:   details.append(f"CRC2 got=0x{got_c2:04x} want=0x{want_c2:04x}")
            if not build_ok: details.append(f"build mismatch: {built.hex()} vs {frame.hex()}")
            print(f"  [{status}] {desc}: {'; '.join(details)}")

    print(f"\n{'=' * 50}")
    print(f"Results: {passed} passed, {failed} failed out of {passed + failed} known frames")
    if failed:
        raise SystemExit(1)
    print("All frames verified. ✓")
