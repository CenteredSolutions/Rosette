#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
"""Echo peer: the stock-ROS 2 side of run_agent_live.sh.

Architecture:
    agent_live_probe (rclc on the micro-ROS agent backend) reaches the DDS
    graph only through a stock micro_ros_agent. This node, on a stock RMW,
    is the graph peer that proves the path: it republishes every message
    from `/<session>/board_to_peer` on `/<session>/peer_to_board`, and calls
    the probe's `/<session>/board_echo` SetBool service with alternating
    values, counting the replies whose `success` equals the request's
    `data`. It runs through the probe's agent outage, so a call that goes
    unanswered is abandoned after `--call-timeout-sec`.

Verification:
    Exits 0 when it ran on `--expect-rmw`, echoed at least `--expect-rx`
    messages, and received at least `--expect-calls` correct replies.

References:
    - rclpy Node.create_subscription / create_client / call_async (Humble).
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class EchoPeer(Node):
    """Echoes the probe's messages and calls its service; see the module docstring."""

    def __init__(self, session: str, call_timeout_sec: float) -> None:
        """Creates the echo publisher, subscription, and service client.

        Args:
            session: Namespace the probe's node runs in.
            call_timeout_sec: How long one service call may stay unanswered.
        """
        super().__init__("agent_echo_peer")
        self.echoed = 0
        self.good_calls = 0
        self._call_timeout_sec = call_timeout_sec
        self._pending = None
        self._pending_since = 0.0
        self._pending_value = False
        self._next_value = True
        self._pub = self.create_publisher(String, f"/{session}/peer_to_board", 10)
        self.create_subscription(String, f"/{session}/board_to_peer", self._on_message, 10)
        self._client = self.create_client(SetBool, f"/{session}/board_echo")
        self.create_timer(0.3, self._call)

    def _on_message(self, msg: String) -> None:
        """Republishes one probe message unchanged."""
        self._pub.publish(msg)
        self.echoed += 1

    def _call(self) -> None:
        """Collects the pending call's reply, or abandons it, then starts the next call."""
        now = time.monotonic()
        if self._pending is not None:
            if self._pending.done():
                reply = self._pending.result()
                if reply is not None and reply.success == self._pending_value:
                    self.good_calls += 1
            elif now - self._pending_since < self._call_timeout_sec:
                return
            self._pending = None

        if not self._client.service_is_ready():
            return

        self._pending_value = self._next_value
        self._next_value = not self._next_value
        self._pending = self._client.call_async(SetBool.Request(data=self._pending_value))
        self._pending_since = now


def main() -> int:
    """Runs the peer; see the module docstring.

    Returns:
        0 on success, 1 otherwise.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--domain", type=int, default=42)
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--expect-rx", type=int, default=5)
    parser.add_argument("--expect-calls", type=int, default=5)
    parser.add_argument("--call-timeout-sec", type=float, default=2.0)
    parser.add_argument("--expect-rmw", default="rmw_fastrtps_cpp")
    args = parser.parse_args()

    rclpy.init(domain_id=args.domain)
    rmw = rclpy.get_rmw_implementation_identifier()
    node = EchoPeer(args.session, args.call_timeout_sec)
    # run_agent_live.sh ends the run early with SIGTERM once the probe is done.
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    deadline = time.monotonic() + args.duration_sec
    while time.monotonic() < deadline and not stop:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"[peer] rmw={rmw} echoed={node.echoed} good_calls={node.good_calls}")
    passed = (
        rmw == args.expect_rmw
        and node.echoed >= args.expect_rx
        and node.good_calls >= args.expect_calls
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
