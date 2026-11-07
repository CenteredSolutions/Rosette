#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
"""Check an i.MX RT1021's FlexRAM layout on the bench, through pyOCD.

Architecture:
    The board's image applies its FlexRAM layout at reset
    (ports/chip/imxrt/flexram_reset_handler.S). This tool resets the board,
    lets the image boot for a moment, halts the core, and checks the layout
    from the debugger, which reaches the TCMs and the OCRAM over the AHB-AP:

    1. Registers: IOMUXC_GPR17.FLEXRAM_BANK_CFG reads back as expected and
       IOMUXC_GPR16.FLEXRAM_BANK_CFG_SEL is 1, so the layout comes from the
       bank configuration and not from the fuses.
    2. Pattern: every 32-bit word of the DTCM and the OCRAM is written with a
       value derived from its own address, then all are read back. A region
       smaller than its window either faults or aliases, and an aliased word
       holds a different address's value, so both show up as a mismatch. The
       test runs twice with the second pattern inverted, so every bit is
       seen as 0 and as 1.
    3. Bounds: the word just past the DTCM's end faults, so the DTCM is no
       larger than expected either.

    The pattern overwrites the running image's RAM, so the tool resets the
    board again at the end; the image boots afresh.

Verification:
    Correct means: exit status 0 exactly when all three checks pass; any
    mismatch is printed with its address, expected, and read values.

References:
    i.MX RT1020 Processor Reference Manual: FlexRAM chapter; IOMUXC_GPR
    GPR16, GPR17; system memory map (DTCM 0x20000000, OCRAM 0x20200000).
    pyOCD Python API: ConnectHelper, MemoryInterface.

Usage:
    .venv/bin/python tools/imxrt_flexram_check.py --dtcm-kib 128 --ocram-kib 128 \\
        --bank-cfg 0xaa55
"""

from __future__ import annotations

import argparse
import sys
import time

from pyocd.core.exceptions import TransferFaultError
from pyocd.core.helpers import ConnectHelper

GPR16 = 0x400AC040
GPR17 = 0x400AC044
FLEXRAM_BANK_CFG_SEL = 0x4
DTCM_BASE = 0x20000000
OCRAM_BASE = 0x20200000
CHUNK_WORDS = 1024


def tag(address: int, invert: bool) -> int:
    """Returns the word stored at an address: the address itself, mixed so neighbours differ in many bits."""
    value = (address * 0x9E3779B1) & 0xFFFFFFFF
    return value ^ 0xFFFFFFFF if invert else value


def pattern_test(target, name: str, base: int, size: int) -> int:
    """Writes then reads back tag() over a region, both polarities; returns the number of mismatches."""
    failures = 0
    for invert in (False, True):
        for offset in range(0, size, CHUNK_WORDS * 4):
            words = [tag(base + offset + 4 * i, invert) for i in range(CHUNK_WORDS)]
            target.write_memory_block32(base + offset, words)

        for offset in range(0, size, CHUNK_WORDS * 4):
            got = target.read_memory_block32(base + offset, CHUNK_WORDS)
            for i, value in enumerate(got):
                address = base + offset + 4 * i
                if value != tag(address, invert):
                    failures += 1
                    if failures <= 8:
                        print(f"  {name} 0x{address:08x}: wrote 0x{tag(address, invert):08x}, read 0x{value:08x}")

    print(f"{name}: {size // 1024} KiB at 0x{base:08x}, {'PASS' if failures == 0 else f'{failures} mismatches'}")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--dtcm-kib", type=int, required=True)
    parser.add_argument("--ocram-kib", type=int, required=True)
    parser.add_argument("--bank-cfg", type=lambda s: int(s, 0), required=True)
    parser.add_argument("--boot-seconds", type=float, default=1.0)
    parser.add_argument("--frequency", type=int, default=1_000_000)
    args = parser.parse_args()

    failures = 0
    with ConnectHelper.session_with_chosen_probe(target_override="mimxrt1020",
                                                 frequency=args.frequency) as session:
        target = session.board.target
        target.reset()
        time.sleep(args.boot_seconds)
        target.halt()

        gpr16 = target.read32(GPR16)
        gpr17 = target.read32(GPR17)
        registers_ok = (gpr17 & 0xFFFF) == args.bank_cfg and (gpr16 & FLEXRAM_BANK_CFG_SEL) != 0
        print(f"GPR16 0x{gpr16:08x}, GPR17 0x{gpr17:08x} (expected BANK_CFG 0x{args.bank_cfg:04x}, "
              f"BANK_CFG_SEL 1): {'PASS' if registers_ok else 'FAIL'}")
        failures += 0 if registers_ok else 1

        failures += pattern_test(target, "DTCM", DTCM_BASE, args.dtcm_kib * 1024)
        failures += pattern_test(target, "OCRAM", OCRAM_BASE, args.ocram_kib * 1024)

        past_end = DTCM_BASE + args.dtcm_kib * 1024
        try:
            target.read32(past_end)
            print(f"DTCM end: 0x{past_end:08x} answered, so the DTCM is larger than expected: FAIL")
            failures += 1
        except TransferFaultError:
            print(f"DTCM end: 0x{past_end:08x} faults: PASS")

        target.reset()

    print("PASS" if failures == 0 else "FAIL")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
