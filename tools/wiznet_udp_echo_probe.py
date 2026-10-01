#!/usr/bin/env python3
"""Probe a board running examples/wiznet_udp_echo over every address it holds.

Architecture:
    For each board address (IPv4, IPv6 link-local, IPv6 global) the probe
    runs two checks from this host on the same link. The IPv6 addresses are
    asked from the firmware's status port (UDP 8) unless given with --ipv6.

    1. ICMP echo (`ping` / `ping -6`). The WIZnet chip answers these in
       silicon, so a reply proves the address is programmed on the chip and
       neighbor discovery (ARP or IPv6 ND) resolves it.
    2. UDP echo (RFC 862, port 7). Datagrams of several sizes up to the
       1472-octet hardware-UDP limit (1452 for IPv6) go to the firmware, which
       returns each one from the socket of the matching family. A reply proves
       the driver's receive header decoding, send path, and source address
       selection for that family; the round-trip time shows whether the chip's
       INT pin wakes the firmware (well under its 1 s idle wait) or not.

    Link-local IPv6 needs a scope: the interface is taken from --iface, or
    from the host's IPv4 route to the board.

Verification:
    Correct means: every ping and every echo datagram comes back byte-for-byte;
    exit status 0 only then.

References:
    RFC 862 (Echo Protocol); RFC 4291 §2.5.6 (link-local scope);
    W6100 datasheet §SLCR (hardware ICMPv6).

Usage:
    tools/wiznet_udp_echo_probe.py 192.168.32.79
"""

from __future__ import annotations

import argparse
import ipaddress
import os
import socket
import statistics
import subprocess
import sys
import time

ECHO_PORT = 7
STATUS_PORT = 8
SIZES = (1, 64, 512, 1024)
REPEATS = 5


def route_interface(ipv4: str) -> str | None:
    """Returns the interface the host routes `ipv4` through, if any."""
    out = subprocess.run(["ip", "-o", "route", "get", ipv4], capture_output=True, text=True).stdout
    words = out.split()
    return words[words.index("dev") + 1] if "dev" in words else None


def board_status(ipv4: str) -> str:
    """Returns the status text the firmware sends back from its status port."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(2.0)
        s.sendto(b"?", (ipv4, STATUS_PORT))
        return s.recvfrom(1500)[0].decode(errors="replace")


def status_ipv6(status: str) -> list[str]:
    """Extracts the IPv6 addresses the status text reports."""
    found = []
    for line in status.splitlines():
        key, _, value = line.partition(" ")
        if key in ("ipv6-link-local", "ipv6-global") and value.strip() != "none":
            found.append(value.strip())
    return found


def ping(address: str, v6: bool, count: int = 3) -> bool:
    """Returns True when `address` answers ICMP echo."""
    cmd = ["ping", "-6" if v6 else "-4", "-c", str(count), "-W", "1", "-i", "0.2", address]
    result = subprocess.run(cmd, capture_output=True, text=True)
    lines = [l for l in result.stdout.splitlines() if "packet loss" in l or "rtt" in l]
    print(f"    ping {address}: {'; '.join(lines) or result.stderr.strip()}")
    return result.returncode == 0


def udp_echo(address: str, v6: bool, scope: int, max_payload: int) -> bool:
    """Sends datagrams of each size to the echo port and checks every reply."""
    family = socket.AF_INET6 if v6 else socket.AF_INET
    target = (address, ECHO_PORT, 0, scope) if v6 else (address, ECHO_PORT)
    ok = True
    with socket.socket(family, socket.SOCK_DGRAM) as s:
        s.settimeout(2.0)
        rtts = []
        for size in (*SIZES, max_payload):
            for i in range(REPEATS):
                payload = os.urandom(size)
                t0 = time.perf_counter()
                s.sendto(payload, target)
                try:
                    data, peer = s.recvfrom(4096)
                except socket.timeout:
                    print(f"    udp {size:5d} B #{i}: TIMEOUT")
                    ok = False
                    continue
                rtts.append((time.perf_counter() - t0) * 1000.0)
                if data != payload:
                    print(f"    udp {size:5d} B #{i}: MISMATCH ({len(data)} B back from {peer[0]})")
                    ok = False
        if rtts:
            print(f"    udp echo {len(rtts)}/{REPEATS * (len(SIZES) + 1)} back, "
                  f"rtt median {statistics.median(rtts):.2f} ms, max {max(rtts):.2f} ms")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("ipv4", help="board IPv4 address")
    parser.add_argument("--ipv6", action="append", default=[],
                        help="board IPv6 address (repeatable; default: asked from the board)")
    parser.add_argument("--iface", help="interface for link-local scope (default: route to ipv4)")
    args = parser.parse_args()

    iface = args.iface or route_interface(args.ipv4)
    scope = socket.if_nametoindex(iface) if iface else 0

    results = []
    try:
        status = board_status(args.ipv4)
        print("[probe] board status\n    " + status.strip().replace("\n", "\n    "))
    except OSError as err:
        status = ""
        print(f"[probe] no status reply from udp/{STATUS_PORT}: {err}")
    ipv6 = args.ipv6 or status_ipv6(status)

    print(f"[probe] IPv4 {args.ipv4}")
    results.append(("ipv4 ping", ping(args.ipv4, False)))
    results.append(("ipv4 udp", udp_echo(args.ipv4, False, 0, 1472)))

    for text in ipv6:
        addr = ipaddress.IPv6Address(text)
        label = "link-local" if addr.is_link_local else "global"
        ping_target = f"{addr}%{iface}" if addr.is_link_local else str(addr)
        print(f"[probe] IPv6 {label} {addr}")
        results.append((f"ipv6 {label} ping", ping(ping_target, True)))
        results.append((f"ipv6 {label} udp",
                        udp_echo(str(addr), True, scope if addr.is_link_local else 0, 1452)))

    print("[probe] summary")
    for name, ok in results:
        print(f"    {'PASS' if ok else 'FAIL'}  {name}")
    return 0 if all(ok for _, ok in results) else 1


if __name__ == "__main__":
    sys.exit(main())
