#!/usr/bin/env python3
"""Bidirectional rclpy probe for proving rmw_rosette interoperability.

Architecture:
    Rosette is exercised through stock ROS 2 client libraries: the same rclpy
    node runs on both sides of a session, and only `RMW_IMPLEMENTATION`
    differs. Role `alpha` publishes on `/<session>/alpha_to_beta` and
    subscribes to `/<session>/beta_to_alpha`; role `beta` mirrors it. Each
    side also serves `std_srvs/SetBool` on `/<session>/<role>_echo` and, like
    every default rclpy node, the parameter services; it calls the peer's
    echo service and the peer's `get_parameters`.

    A session passes when each side receives at least `--expect-rx` samples
    and completes at least `--expect-calls` correlated service calls in each
    direction. That requires SPDP discovery, SEDP endpoint matching, reliable
    DATA/HEARTBEAT/ACKNACK exchange, and request/reply correlation through
    PID_RELATED_SAMPLE_IDENTITY to all work across the two middlewares.

    With `--expect-peer-graph`, the side must also see the peer's node in its
    node list and the peer's echo service listed under that node. Node
    identity travels only in the peer's `ros_discovery_info` messages
    (rmw_dds_common/msg/ParticipantEntitiesInfo), so this checks that the
    peer publishes them and that their GIDs match its SEDP endpoints.

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
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool

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
        session: str,
        peer_node: str,
    ) -> None:
        """Creates the topic endpoints, the echo service, and the peer clients.

        Args:
            node_name: ROS node name.
            namespace: ROS node namespace; empty for the root namespace.
            tx_topic: Topic this side publishes on.
            rx_topic: Topic this side subscribes to.
            publish_period_sec: Seconds between published samples.
            role: `alpha` or `beta`; embedded in every payload.
            session: Session name shared by both sides.
            peer_node: Fully-qualified name of the peer's node.
        """
        super().__init__(node_name, namespace=namespace)
        self._role = role
        self._peer = "beta" if role == "alpha" else "alpha"
        self._host = socket.gethostname()
        self.tx_count = 0
        self.rx_count = 0
        self.echo_ok = 0
        self.param_ok = 0
        self.peer_graph_ok = False
        self._peer_node = peer_node
        self._peer_echo = f"/{session}/{self._peer}_echo"
        self._call_seq = 0
        self._inflight: list = []

        # The peer reads this back through our parameter services.
        self.declare_parameter("probe_rmw", rclpy.get_rmw_implementation_identifier())
        self._echo_service = self.create_service(
            SetBool, f"/{session}/{role}_echo", self._on_echo
        )
        self._echo_client = self.create_client(SetBool, f"/{session}/{self._peer}_echo")
        self._param_client = self.create_client(GetParameters, f"{peer_node}/get_parameters")
        self._call_timer = self.create_timer(publish_period_sec, self._call_peer)

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

    def _on_echo(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Serves the peer: echoes the request so the caller can check correlation."""
        response.success = request.data
        response.message = f"{self._role}:{int(request.data)}"
        return response

    def check_peer_graph(self) -> None:
        """Records whether the graph lists the peer's node and its echo service under it."""
        if self.peer_graph_ok:
            return
        namespace, _, name = self._peer_node.rpartition("/")
        namespace = namespace or "/"
        if (name, namespace) not in self.get_node_names_and_namespaces():
            return
        services = self.get_service_names_and_types_by_node(name, namespace)
        if any(service == self._peer_echo for service, _ in services):
            self.peer_graph_ok = True
            self.get_logger().info(f"peer graph ok: {self._peer_node} serves {self._peer_echo}")

    def _call_peer(self) -> None:
        """Calls the peer's echo and get_parameters services once each, when reachable."""
        self._inflight = [f for f in self._inflight if not f.done()]
        if self._inflight:
            return
        if self._echo_client.service_is_ready():
            self._call_seq += 1
            want = bool(self._call_seq % 2)
            future = self._echo_client.call_async(SetBool.Request(data=want))
            future.add_done_callback(lambda f, w=want: self._on_echo_reply(f, w))
            self._inflight.append(future)
        if self._param_client.service_is_ready():
            future = self._param_client.call_async(GetParameters.Request(names=["probe_rmw"]))
            future.add_done_callback(self._on_param_reply)
            self._inflight.append(future)

    def _on_echo_reply(self, future, sent: bool) -> None:
        reply = future.result()
        expected = f"{self._peer}:{int(sent)}"
        if reply is not None and reply.success == sent and reply.message == expected:
            self.echo_ok += 1
            self.get_logger().info(f'echo ok={self.echo_ok} reply="{reply.message}"')
        else:
            self.get_logger().error(f"echo reply mismatch: sent={sent} got={reply}")

    def _on_param_reply(self, future) -> None:
        reply = future.result()
        values = reply.values if reply is not None else []
        if len(values) == 1 and values[0].type == ParameterType.PARAMETER_STRING:
            self.param_ok += 1
            self.get_logger().info(f"get_parameters ok={self.param_ok} "
                                   f"peer_rmw={values[0].string_value}")
        else:
            self.get_logger().error(f"get_parameters reply unexpected: {reply}")


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
    parser.add_argument("--expect-calls", type=int, default=1,
                        help="Minimum correlated calls per peer service (echo and get_parameters).")
    parser.add_argument("--expect-peer-graph", action="store_true",
                        help="Require the peer's node, and its echo service under it, in the graph.")
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
        0 when the expected RMW was loaded, at least `--expect-rx` samples
        arrived, at least `--expect-calls` calls to each peer service
        returned correctly correlated replies, and (with `--expect-peer-graph`)
        the peer's node and echo service appeared in the graph before the
        deadline; 1 otherwise.
    """
    args = parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    tx_topic, rx_topic = compute_topics(args.session, args.role)
    node_name = args.node_name or f"{args.session}_{args.role}_probe"
    peer_role = "beta" if args.role == "alpha" else "alpha"
    peer_node = f"{args.namespace.rstrip('/')}/{args.session}_{peer_role}_probe"

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
        session=args.session,
        peer_node=peer_node,
    )
    node.get_logger().info(
        f"role={args.role} rmw={loaded_rmw} domain={args.domain} tx={tx_topic} "
        f"rx={rx_topic} duration_sec={args.duration_sec:.1f} expect_rx={args.expect_rx}"
    )

    deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.expect_peer_graph:
                node.check_peer_graph()
    finally:
        tx_count, rx_count = node.tx_count, node.rx_count
        echo_ok, param_ok = node.echo_ok, node.param_ok
        peer_graph_ok = node.peer_graph_ok
        node.get_logger().info(f"summary rmw={loaded_rmw} tx={tx_count} rx={rx_count} "
                               f"echo_calls={echo_ok} param_calls={param_ok}")
        node.destroy_node()
        rclpy.shutdown()
        if capture is not None:
            capture.stop()
            print(f"{_LOG_PREFIX} capture saved to {capture.pcap_file}")

    ok = True
    if rx_count < args.expect_rx:
        print(f"{_LOG_PREFIX} expected at least {args.expect_rx} inbound samples, "
              f"got {rx_count}", file=sys.stderr)
        ok = False
    if min(echo_ok, param_ok) < args.expect_calls:
        print(f"{_LOG_PREFIX} expected at least {args.expect_calls} correlated calls per "
              f"peer service, got echo={echo_ok} get_parameters={param_ok}", file=sys.stderr)
        ok = False
    if args.expect_peer_graph and not peer_graph_ok:
        print(f"{_LOG_PREFIX} peer node {peer_node} or its echo service never appeared in "
              f"the graph", file=sys.stderr)
        ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
