#!/usr/bin/env python3
"""
steca_setpoint.py — Active-power setpoint for StecaGrid RS485 (EMLiveMeas write).

Write(0x50) on Topic 0x0d, TO=0x01 (inverter), FROM=0x7b (SEM sender).
Payload data: 00 FF <hi> <lo>  where <hi><lo> = setpoint in permille (0..1000, BE uint16).

Frame body between CRC1 and CRC2:
  50 03 00 05 0d 00 ff <hi> <lo> <chk>

CHK = additive 8-bit checksum over body, excluding the 0x03 auth byte:
  chk = (0x50 + 0x05 + 0x0D + 0x00 + 0xFF + hi + lo) & 0xFF

WARNING: Do not send while a physical SEM is connected on address 0x01 —
two-master collision on the RS485 bus.  The inverter discards the setpoint
after a timeout (safe-state fallback); the caller must repeat periodically.
"""

from steca_crc import build_frame

# Relay levels used by the StecaGrid SEM energy manager (values in percent)
EM_LEVELS = {"K1": 0, "K2": 30, "K3": 60, "K4": 100}


def build_setpoint(permille: int, to: int = 0x01, frm: int = 0x7b) -> bytes:
    """Build an active-power setpoint frame.

    permille: 0..1000  (0 = 0 %, 1000 = 100 %, resolution 0.1 %)
    Returns a complete, CRC-verified RS485 frame ready to send.
    """
    if not 0 <= permille <= 1000:
        raise ValueError(f"permille must be 0..1000, got {permille}")
    hi  = (permille >> 8) & 0xFF
    lo  = permille & 0xFF
    chk = (0x50 + 0x05 + 0x0D + 0x00 + 0xFF + hi + lo) & 0xFF
    return build_frame(to, frm,
                       bytes([0x50, 0x03, 0x00, 0x05, 0x0D, 0x00, 0xFF, hi, lo, chk]))


def build_setpoint_percent(percent: float, to: int = 0x01, frm: int = 0x7b) -> bytes:
    """Build a setpoint frame from a percentage value (0.0..100.0, 0.1 % resolution)."""
    return build_setpoint(round(percent * 10), to, frm)


# ── Self-test ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    _KNOWN = [
        (   0, "02010014017b6e500300050d00ff000061fb4203"),
        ( 300, "02010014017b6e500300050d00ff012c8e484f03"),
        ( 600, "02010014017b6e500300050d00ff0258bb5d4603"),
        (1000, "02010014017b6e500300050d00ff03e84c5a0003"),
    ]

    passed = failed = 0
    for permille, hexstr in _KNOWN:
        want  = bytes.fromhex(hexstr)
        got   = build_setpoint(permille)
        ok    = got == want
        label = f"{permille:>4} ‰  ({permille/10:5.1f} %)"
        if ok:
            passed += 1
            print(f"  [PASS] {label}")
        else:
            failed += 1
            print(f"  [FAIL] {label}")
            print(f"         want: {want.hex()}")
            print(f"         got:  {got.hex()}")

    print(f"\n{'=' * 50}")
    print(f"Results: {passed} passed, {failed} failed out of {passed + failed} known frames")
    if failed:
        raise SystemExit(1)
    print("All setpoint frames verified. ✓")
