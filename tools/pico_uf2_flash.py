#!/usr/bin/env python3
"""Flash a UF2 onto a Pico over USB without pressing BOOTSEL, then watch its console.

Architecture:
    A running Pico SDK application with USB stdio reboots into the BOOTSEL
    ROM when its CDC port is opened at 1200 baud
    (PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE, on by default). The ROM then
    enumerates as a USB drive labelled RPI-RP2 (RP2040) or RP2350, which the
    desktop session mounts under /media/$USER. Copying a UF2 there flashes it
    and the device reboots into the new image, whose console reappears as a
    CDC port. This needs only membership in the group owning /dev/ttyACM*
    (dialout), not root or a picotool udev rule.

    Steps: 1200-baud touch (skipped when the drive is already present), wait
    for the drive, copy the UF2, wait for the console, then optionally stream
    it until a pattern is seen or a timeout expires.

Verification:
    Correct means: the new image runs (its console output appears) without
    manual BOOTSEL; exit status 0 when --expect matched, 1 otherwise.

References:
    pico-sdk src/rp2_common/pico_stdio_usb/reset_interface.c (1200-baud reset).
    UF2 format: https://github.com/microsoft/uf2

Usage:
    tools/pico_uf2_flash.py build-hw/pico2/rosette_platform_contract.uf2 \\
        --monitor 20 --expect "platform contract: PASS"
"""

from __future__ import annotations

import argparse
import glob
import os
import select
import shutil
import subprocess
import sys
import termios
import time

DRIVE_LABELS = ("RP2350", "RPI-RP2")


def find_drive() -> str | None:
    """Returns the mount point of a Pico BOOTSEL drive, if one is mounted."""
    user = os.environ.get("USER", "")
    for label in DRIVE_LABELS:
        for base in (f"/media/{user}", "/run/media/" + user, "/media"):
            path = os.path.join(base, label)
            if os.path.isfile(os.path.join(path, "INFO_UF2.TXT")):
                return path
    return None


def mount_bootsel_drive() -> str | None:
    """Mounts an unmounted BOOTSEL drive via udisksctl; returns its mount point.

    Desktop sessions automount the drive; headless or SSH sessions do not, and
    udisksctl may then need polkit authorization (run it once interactively).
    """
    for label in DRIVE_LABELS:
        dev = f"/dev/disk/by-label/{label}"
        if not os.path.exists(dev):
            continue
        result = subprocess.run(["udisksctl", "mount", "--no-user-interaction", "-b", dev],
                                capture_output=True, text=True)
        if result.returncode != 0:
            print(f"[flash] could not mount {dev}: {result.stderr.strip()}\n"
                  f"[flash] run: udisksctl mount -b {dev}", file=sys.stderr)
            return None
        return find_drive()
    return None


def touch_1200(port: str) -> None:
    """Opens `port` at 1200 baud and closes it, asking the app to enter BOOTSEL."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B1200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        time.sleep(0.2)
    finally:
        os.close(fd)


def wait_for(predicate, timeout: float, step: float = 0.25):
    """Polls `predicate` until it returns a truthy value or `timeout` passes."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(step)
    return None


def monitor(port: str, seconds: float, expect: str | None) -> bool:
    """Streams the console to stdout; returns True once `expect` is seen."""
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    seen = ""
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            ready, _, _ = select.select([fd], [], [], 0.2)
            if not ready:
                continue
            try:
                chunk = os.read(fd, 4096).decode(errors="replace")
            except (BlockingIOError, OSError):
                continue
            sys.stdout.write(chunk)
            sys.stdout.flush()
            seen = (seen + chunk)[-8192:]
            if expect and expect in seen:
                return True
    finally:
        os.close(fd)
    return expect is None


def main() -> int:
    """Flashes the UF2 and optionally monitors; see the module docstring."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("uf2")
    parser.add_argument("--port", default=None, help="CDC port (default: first /dev/ttyACM*)")
    parser.add_argument("--monitor", type=float, default=0.0, help="seconds to stream the console")
    parser.add_argument("--expect", default=None, help="text that ends monitoring successfully")
    args = parser.parse_args()

    drive = find_drive() or mount_bootsel_drive()
    if drive is None:
        port = args.port or (sorted(glob.glob("/dev/ttyACM*")) or [None])[0]
        if port is None:
            print("no Pico console or BOOTSEL drive found", file=sys.stderr)
            return 1
        touch_1200(port)
        wait_for(lambda: any(os.path.exists(f"/dev/disk/by-label/{l}") for l in DRIVE_LABELS), 10.0)
        drive = find_drive() or mount_bootsel_drive() or wait_for(find_drive, 60.0)
        if drive is None:
            print("BOOTSEL drive not mounted (mount it, then rerun this command)",
                  file=sys.stderr)
            return 1
    print(f"[flash] {args.uf2} -> {drive}", flush=True)
    shutil.copyfile(args.uf2, os.path.join(drive, os.path.basename(args.uf2)))
    os.sync()

    wait_for(lambda: find_drive() is None, 15.0)
    port = wait_for(lambda: args.port if args.port and os.path.exists(args.port)
                    else (sorted(glob.glob("/dev/ttyACM*")) or [None])[0], 15.0)
    if port is None:
        print("[flash] image written; no console appeared", file=sys.stderr)
        return 0 if args.monitor == 0 else 1
    print(f"[flash] running; console on {port}", flush=True)
    if args.monitor > 0:
        return 0 if monitor(port, args.monitor, args.expect) else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
