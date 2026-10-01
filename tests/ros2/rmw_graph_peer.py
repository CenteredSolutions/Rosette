#!/usr/bin/env python3
"""Graph peer: one publisher and one subscription that come and go on cue.

Architecture:
    The stock-RMW half of run_rmw_graph_loopback.sh. It starts a node,
    waits `--delay-sec` so the observer (rcl_graph_wait_probe, under the RMW
    under test) is already blocked in rcl_wait_for_publishers(), creates one
    publisher and one subscription on `--topic`, holds them `--hold-sec`
    while spinning, then destroys both and keeps the node alive
    `--linger-sec` so the observer sees the withdrawal through SEDP rather
    than through the participant's lease expiring.

Verification:
    Passes when `rclpy.get_rmw_implementation_identifier()` equals
    `--expect-rmw`; the observer carries the graph checks.

References:
    - rclpy Node.create_publisher / create_subscription / destroy_* (Humble).
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def spin_for(node: Node, seconds: float) -> None:
    """Spins `node` for `seconds` so its participant keeps answering discovery.

    Args:
        node: Node to spin.
        seconds: Wall-clock duration.
    """
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    """Runs the peer; see the module docstring.

    Returns:
        0 on success, 1 when the RMW is not the expected one.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--domain", type=int, default=42)
    parser.add_argument("--delay-sec", type=float, default=2.0)
    parser.add_argument("--hold-sec", type=float, default=4.0)
    parser.add_argument("--linger-sec", type=float, default=6.0)
    parser.add_argument("--expect-rmw", default="rmw_fastrtps_cpp")
    args = parser.parse_args()

    rclpy.init(domain_id=args.domain)
    node = Node("rosette_graph_peer")
    rmw = rclpy.get_rmw_implementation_identifier()
    print(f"[rmw-graph-peer] rmw={rmw} topic={args.topic}", flush=True)
    if rmw != args.expect_rmw:
        print(f"[rmw-graph-peer] FAIL: expected {args.expect_rmw}", flush=True)
        return 1

    spin_for(node, args.delay_sec)
    pub = node.create_publisher(String, args.topic, 10)
    sub = node.create_subscription(String, args.topic, lambda _msg: None, 10)
    print("[rmw-graph-peer] endpoints created", flush=True)
    spin_for(node, args.hold_sec)
    node.destroy_publisher(pub)
    node.destroy_subscription(sub)
    print("[rmw-graph-peer] endpoints destroyed", flush=True)
    spin_for(node, args.linger_sec)
    node.destroy_node()
    rclpy.shutdown()
    print("[rmw-graph-peer] PASS", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
