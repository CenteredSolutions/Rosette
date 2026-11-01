#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
"""Lease re-match probe: a stock peer must list a node again after losing its lease.

Architecture:
    Both halves of run_rmw_lease_rematch_loopback.sh. Role `board` runs a
    node named `--node` on the RMW under test and only spins. That RMW is
    built with an SPDP resend period longer than its lease, so a stock peer
    expires the participant between announcements and re-discovers it on
    the next one, while the board, which hears the peer's frequent
    announcements, never expires the peer: the one-sided lease loss a board
    sees when its multicast is lost for a while. Role `watch` runs on a
    stock RMW and records, once a second, whether its graph lists the
    board's node and the board's services.

    A stock peer re-learns the board's endpoints from SEDP but re-learns its
    node only from a new `ros_discovery_info` message: Fast DDS discards a
    replay of the message it delivered before the lease loss.

Verification:
    The watcher passes when it saw the board's node, then lost it to lease
    expiry, then listed it again with its services, all within
    `--duration-sec`. A run in which the node is never lost proves nothing
    and fails, naming the build setting it needs.

References:
    - OMG DDSI-RTPS 2.5 §8.5.3.2 (participant lease), §8.4.1.4 (matched_reader_add).
    - Fast DDS 2.6 (Humble) src/cpp/rtps/reader/RTPSReader.cpp, `may_remove_history_record`.
    - rclpy Node.get_node_names_and_namespaces / get_service_names_and_types (Humble).
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy


def parse_args() -> argparse.Namespace:
    """Parses the command line.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--role", choices=("board", "watch"), required=True)
    parser.add_argument("--node", required=True, help="the board's node name")
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument("--expect-rmw", required=True)
    return parser.parse_args()


def watch(node, board: str, duration_sec: float) -> bool:
    """Records the board's visibility each second and judges the run.

    Args:
        node: The watcher's rclpy node.
        board: The board's node name.
        duration_sec: How long to watch.

    Returns:
        True when the board's node was listed, lost, and listed again with
        its services.
    """
    prefix = f"/{board}/"
    phases = []  # Visibility changes, as (seconds, node_listed, services_listed).
    start = time.monotonic()
    while time.monotonic() - start < duration_sec:
        rclpy.spin_once(node, timeout_sec=1.0)
        listed = any(name == board for name, _ in node.get_node_names_and_namespaces())
        services = any(name.startswith(prefix) for name, _ in node.get_service_names_and_types())
        if not phases or phases[-1][1:] != (listed, services):
            phases.append((time.monotonic() - start, listed, services))
            print(f"[watch] t={phases[-1][0]:5.1f}s node={listed} services={services}", flush=True)

    seen = [i for i, phase in enumerate(phases) if phase[1]]
    lost_after_seen = seen and any(not phase[1] for phase in phases[seen[0]:])
    if not lost_after_seen:
        print("[watch] FAIL: the board's lease never expired; build its rmw with "
              "RTPS_SPDP_RESEND_PERIOD_MS above RTPS_SPDP_LEASE_DURATION_MS", flush=True)
        return False

    first_loss = next(i for i in range(seen[0], len(phases)) if not phases[i][1])
    if not any(phase[1] and phase[2] for phase in phases[first_loss:]):
        print("[watch] FAIL: the board's node was not listed again after its lease expired",
              flush=True)
        return False

    print("[watch] PASS: the board's node was listed again after its lease expired", flush=True)
    return True


def main() -> int:
    """Runs one role.

    Returns:
        The process exit status: 0 on pass.
    """
    args = parse_args()
    rclpy.init(domain_id=args.domain)
    rmw = rclpy.get_rmw_implementation_identifier()
    if rmw != args.expect_rmw:
        print(f"[{args.role}] FAIL: loaded {rmw}, expected {args.expect_rmw}", flush=True)
        return 1

    if args.role == "board":
        node = rclpy.create_node(args.node)
        deadline = time.monotonic() + args.duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

        ok = True
    else:
        node = rclpy.create_node(f"{args.node}_watch")
        ok = watch(node, args.node, args.duration_sec)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
