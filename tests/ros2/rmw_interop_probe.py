#!/usr/bin/env python3
"""Bidirectional rclpy probe for proving rmw_rosette interoperability.

Architecture:
    Rosette is exercised through stock ROS 2 client libraries: the same rclpy
    node runs on both sides of a session, and only `RMW_IMPLEMENTATION`
    differs. Role `alpha` publishes on `/<session>/alpha_to_beta` and
    subscribes to `/<session>/beta_to_alpha`; role `beta` mirrors it. A
    session passes when each side receives at least `--expect-rx` samples,
    which requires SPDP discovery, SEDP endpoint matching, and reliable
    DATA/HEARTBEAT/ACKNACK exchange to all work across the two middlewares.

    Run the two roles on one host with `run_rmw_interop_loopback.sh`, or on two
    hosts by hand to put a real network between them.

Verification:
    A probe only counts as evidence when it actually loaded the RMW under
    test. `--expect-rmw` (defaulting to `RMW_IMPLEMENTATION`) is compared with
    `rclpy.get_rmw_implementation_identifier()`, so a misconfigured
    environment fails loudly instead of silently testing a stock RMW against
    itself.

References:
    - ROS 2 design article "ROS 2 middleware interface"
      (https://design.ros2.org/articles/ros_middleware_interface.html).
    - OMG DDSI-RTPS 2.5 §8.5 (discovery), §8.4.7 (reliable writer behavior).
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from rtps_packet_capture import (
    CaptureHandle,
    build_capture_filter,
    choose_capture_iface,
    start_capture,
)

_LOG_PREFIX = "[rmw-interop-probe]"


class InteropProbeNode(Node):
    """Publishes a counter on one topic and counts samples on the other.

    QoS is RELIABLE + VOLATILE with depth 10 on both endpoints, so the session
    exercises the reliable writer/reader handshake without depending on
    durability or history behavior that differs between middlewares.
    """

    def __init__(
        self,
        *,
        node_name: str,
        namespace: str,
        tx_topic: str,
        rx_topic: str,
        publish_period_sec: float,
        role: str,
    ) -> None:
        """Creates the publisher, subscription, and publish timer.

        Args:
            node_name: ROS node name.
            namespace: ROS node namespace; empty for the root namespace.
            tx_topic: Topic this side publishes on.
            rx_topic: Topic this side subscribes to.
            publish_period_sec: Seconds between published samples.
            role: `alpha` or `beta`; embedded in every payload.
        """
        super().__init__(node_name, namespace=namespace)
        self._role = role
        self._host = socket.gethostname()
        self.tx_count = 0
        self.rx_count = 0

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(String, tx_topic, qos)
        self._subscription = self.create_subscription(
            String, rx_topic, self._on_message, qos
        )
        self._timer = self.create_timer(publish_period_sec, self._publish)

    def _publish(self) -> None:
        msg = String()
        msg.data = (
            f"role={self._role} host={self._host} rmw={rclpy.get_rmw_implementation_identifier()} "
            f"tx_seq={self.tx_count}"
        )
        self._publisher.publish(msg)
        self.tx_count += 1
        self.get_logger().info(f'tx seq={self.tx_count} payload="{msg.data}"')

    def _on_message(self, msg: String) -> None:
        self.rx_count += 1
        self.get_logger().info(f'rx seq={self.rx_count} payload="{msg.data}"')


def parse_args() -> argparse.Namespace:
    """Parses command-line options; see `--help` for each flag."""
    parser = argparse.ArgumentParser(
        description="Bidirectional rclpy probe for ROS 2 middleware interop sessions."
    )
    parser.add_argument("--role", choices=("alpha", "beta"), required=True)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--session", default="rosette_probe")
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--publish-period-sec", type=float, default=1.0)
    parser.add_argument("--expect-rx", type=int, default=1)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--node-name", default=None)
    parser.add_argument(
        "--expect-rmw",
        default=os.environ.get("RMW_IMPLEMENTATION"),
        help="RMW identifier this process must load (default: $RMW_IMPLEMENTATION).",
    )
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--capture-iface", default=None)
    parser.add_argument("--pcap-file", default=None)
    return parser.parse_args()


def compute_topics(session: str, role: str) -> tuple[str, str]:
    """Returns `(tx_topic, rx_topic)` for one role of a session.

    Args:
        session: Session name shared by both sides.
        role: `alpha` or `beta`.

    Returns:
        Topic pair such that alpha's tx is beta's rx and vice versa.
    """
    alpha_to_beta = f"/{session}/alpha_to_beta"
    beta_to_alpha = f"/{session}/beta_to_alpha"
    if role == "alpha":
        return alpha_to_beta, beta_to_alpha
    return beta_to_alpha, alpha_to_beta


def main() -> int:
    """Runs one side of an interop session.

    Returns:
        0 when the expected RMW was loaded and at least `--expect-rx` samples
        arrived before the deadline; 1 otherwise.
    """
    args = parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    tx_topic, rx_topic = compute_topics(args.session, args.role)
    node_name = args.node_name or f"{args.session}_{args.role}_probe"

    capture: CaptureHandle | None = None
    if args.capture:
        pcap_file = Path(
            args.pcap_file
            or f"/tmp/{args.session}-{args.role}-domain{args.domain}-{int(time.time())}.pcap"
        )
        try:
            iface = choose_capture_iface(args.capture_iface)
            capture = start_capture(domain=args.domain, iface=iface, pcap_file=pcap_file)
        except RuntimeError as exc:
            print(f"{_LOG_PREFIX} capture setup failed: {exc}", file=sys.stderr)
            return 1
        print(f"{_LOG_PREFIX} capture iface={iface} file={pcap_file} "
              f"filter='{build_capture_filter(args.domain)}'")

    rclpy.init()
    loaded_rmw = rclpy.get_rmw_implementation_identifier()
    if args.expect_rmw and loaded_rmw != args.expect_rmw:
        print(f"{_LOG_PREFIX} loaded rmw '{loaded_rmw}', expected '{args.expect_rmw}'",
              file=sys.stderr)
        rclpy.shutdown()
        if capture is not None:
            capture.stop()
        return 1

    node = InteropProbeNode(
        node_name=node_name,
        namespace=args.namespace,
        tx_topic=tx_topic,
        rx_topic=rx_topic,
        publish_period_sec=args.publish_period_sec,
        role=args.role,
    )
    node.get_logger().info(
        f"role={args.role} rmw={loaded_rmw} domain={args.domain} tx={tx_topic} "
        f"rx={rx_topic} duration_sec={args.duration_sec:.1f} expect_rx={args.expect_rx}"
    )

    deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        tx_count, rx_count = node.tx_count, node.rx_count
        node.get_logger().info(f"summary rmw={loaded_rmw} tx={tx_count} rx={rx_count}")
        node.destroy_node()
        rclpy.shutdown()
        if capture is not None:
            capture.stop()
            print(f"{_LOG_PREFIX} capture saved to {capture.pcap_file}")

    if rx_count < args.expect_rx:
        print(f"{_LOG_PREFIX} expected at least {args.expect_rx} inbound samples, "
              f"got {rx_count}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
