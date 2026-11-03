#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
"""Reliable-burst probe: a subscription keeps every sample its depth covers.

Architecture:
    Two rclpy processes share one DDS domain and differ only in
    `RMW_IMPLEMENTATION`. Role `publisher` (a stock RMW) waits for the
    subscription to match, publishes `--count` samples back to back on a
    RELIABLE, VOLATILE, KEEP_LAST topic, and waits until every matched reader
    has acknowledged them. Role `subscriber` (the RMW under test) creates a
    RELIABLE, VOLATILE, KEEP_LAST subscription of `--depth` (default 10, the
    rclc/rclcpp/rclpy default profile depth), spins for `--discover-sec` so
    discovery completes, then stops spinning for `--idle-sec` and finally
    takes everything that arrived. The publisher waits `--publish-delay-sec`
    after matching, which places the burst inside the subscriber's idle
    window, so the whole burst reaches the reader before the first take.
    Timed windows stand in for a graph query because they need nothing from
    the RMW under test beyond publish/subscribe.

    Against a board, run only the publisher role with `--topic` naming the
    board's subscription (for rclc_chatter, `/rosette/command`); every run
    matches a new writer, and the board's console must print the burst whole.

    This is the shape of a late match on a board: when a reader matches a
    writer after the writer published, the missed samples arrive together as
    one repair burst, ahead of the application's next take. A reader history
    of the requested depth holds the whole burst; one that is shallower
    overwrites the oldest samples after the RTPS reader has already
    acknowledged them, and nothing on the wire reports the loss.

Verification:
    The subscriber passes only when it takes the newest min(count, depth)
    samples of the burst, each exactly once and in order: all of them when the
    burst fits the depth, and only the newest when it does not (a history
    deeper than requested hands the application stale samples). None may be
    taken before its idle
    window (otherwise the burst was not queued and the run proves nothing). The
    publisher passes only when `wait_for_all_acked()` succeeds. Each role
    checks `rclpy.get_rmw_implementation_identifier()` against
    `--expect-rmw` so a misconfigured environment fails loudly.

References:
    - OMG DDS 1.4 §2.2.3.18 (HISTORY: KEEP_LAST depth), §2.2.3.14 (RELIABILITY).
    - OMG DDSI-RTPS 2.5 §8.4.9.2 (reliable StatefulReader behavior).
    - ROS 2 rmw `rmw_qos_profile_default` (depth 10).
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

_LOG_PREFIX = "[reliable-burst-probe]"


def _log(role: str, text: str) -> None:
    """Prints one flushed log line tagged with the probe role.

    Args:
        role: Probe role, `publisher` or `subscriber`.
        text: Message to print.
    """
    print(f"{_LOG_PREFIX} {role}: {text}", flush=True)


def _qos(depth: int) -> QoSProfile:
    """Builds the RELIABLE, VOLATILE, KEEP_LAST profile both roles use.

    Args:
        depth: KEEP_LAST history depth.

    Returns:
        The QoS profile.
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _spin_until(node: Node, predicate, timeout_sec: float) -> bool:
    """Spins `node` until `predicate()` holds or the timeout expires.

    Args:
        node: Node to spin.
        predicate: Zero-argument callable polled between spins.
        timeout_sec: Upper bound on the wait, in seconds.

    Returns:
        True when the predicate held before the deadline.
    """
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


def run_publisher(node: Node, topic: str, args: argparse.Namespace) -> int:
    """Publishes one reliable burst and waits for every reader to acknowledge it.

    Args:
        node: Node that owns the publisher.
        topic: Topic name.
        args: Parsed command line.

    Returns:
        Process exit status: 0 when the burst was acknowledged.
    """
    pub = node.create_publisher(String, topic, _qos(args.depth))
    if not _spin_until(node, lambda: pub.get_subscription_count() > 0, args.match_timeout_sec):
        _log("publisher", "FAIL: no subscription matched")
        return 1
    _spin_until(node, lambda: False, args.publish_delay_sec)
    for index in range(args.count):
        pub.publish(String(data=f"burst-{index}"))
    _log("publisher", f"published {args.count}")
    acked = pub.wait_for_all_acked(Duration(seconds=args.ack_timeout_sec))
    _log("publisher", f"wait_for_all_acked={acked}")
    # Stay up so the subscriber's takes are never raced by a disposal.
    _spin_until(node, lambda: False, args.linger_sec)
    return 0 if acked else 1


def run_subscriber(node: Node, topic: str, args: argparse.Namespace) -> int:
    """Lets a reliable burst queue up untaken, then takes it and checks it.

    Args:
        node: Node that owns the subscription.
        topic: Topic name.
        args: Parsed command line.

    Returns:
        Process exit status: 0 when the newest min(count, depth) samples were
        taken in order.
    """
    received: list[str] = []
    sub = node.create_subscription(String, topic, lambda msg: received.append(msg.data),
                                   _qos(args.depth))
    _log("subscriber", f"requested depth={args.depth}")
    _spin_until(node, lambda: False, args.discover_sec)
    if received:
        _log("subscriber", f"FAIL: took {received} before the idle window; lengthen the windows")
        return 1
    time.sleep(args.idle_sec)
    _spin_until(node, lambda: len(received) >= args.count, args.collect_sec)
    first = max(0, args.count - args.depth)
    expected = [f"burst-{index}" for index in range(first, args.count)]
    _log("subscriber", f"took {len(received)}: {received}")
    if received != expected:
        missing = [data for data in expected if data not in received]
        _log("subscriber", f"FAIL: expected {expected}; missing {missing}")
        return 1
    _log("subscriber", "PASS")
    return 0


def main() -> int:
    """Parses arguments, checks the loaded RMW, and runs one role.

    Returns:
        Process exit status.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--role", choices=("publisher", "subscriber"), required=True)
    parser.add_argument("--session", default="burst", help="Topic namespace shared by both roles.")
    parser.add_argument("--topic", default="", help="Topic to use instead of /<session>/burst.")
    parser.add_argument("--domain", type=int, default=42)
    parser.add_argument("--count", type=int, default=5, help="Samples in the burst.")
    parser.add_argument("--depth", type=int, default=10, help="KEEP_LAST depth of both endpoints.")
    parser.add_argument("--discover-sec", type=float, default=2.0)
    parser.add_argument("--publish-delay-sec", type=float, default=4.0)
    parser.add_argument("--idle-sec", type=float, default=6.0)
    parser.add_argument("--collect-sec", type=float, default=10.0)
    parser.add_argument("--match-timeout-sec", type=float, default=15.0)
    parser.add_argument("--ack-timeout-sec", type=float, default=15.0)
    parser.add_argument("--linger-sec", type=float, default=3.0)
    parser.add_argument("--expect-rmw", default=os.environ.get("RMW_IMPLEMENTATION", ""))
    args = parser.parse_args()

    rclpy.init(domain_id=args.domain)
    try:
        loaded = rclpy.get_rmw_implementation_identifier()
        if args.expect_rmw and loaded != args.expect_rmw:
            _log(args.role, f"FAIL: loaded {loaded}, expected {args.expect_rmw}")
            return 1
        node = Node(f"burst_{args.role}")
        topic = args.topic or f"/{args.session}/burst"
        if args.role == "publisher":
            return run_publisher(node, topic, args)
        return run_subscriber(node, topic, args)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
