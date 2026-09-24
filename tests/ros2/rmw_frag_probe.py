#!/usr/bin/env python3
"""Large-message probe: fragmentation and reassembly across two RMWs.

Architecture:
    Each role publishes sensor_msgs/Image samples on `/<session>/<role>_frames`
    whose payload sizes cycle through FRAME_SIZES, spanning a single small
    datagram up to many fragments at a 1472-octet datagram limit. Every
    payload is a pattern derived from its sequence number (carried in
    header.frame_id), so the receiver verifies each octet. Each role also
    declares a PAYLOAD_PARAM string parameter of PARAM_CHARS characters and
    reads the peer's through get_parameters, so service replies are fragmented
    too and must still reach the client that asked.

    Each direction is sized independently (--frame-sizes/--param-chars for
    what this role sends, --peer-frame-sizes/--peer-param-chars for what it
    expects back), because a stock peer with its default message size sends
    any sample above the link limit as one IP-fragmented datagram, which a
    link without IP reassembly cannot deliver. Run by
    run_rmw_frag_loopback.sh with rmw_rosette as `alpha` and a stock RMW as
    `beta`.

Verification:
    Correct means: every expected peer frame size arrives at least
    --expect-per-size times with every octet intact, no frame arrives
    corrupted, and at least --expect-calls get_parameters replies return the
    peer's exact string.

References:
    OMG DDSI-RTPS 2.5 §8.4.14 (fragmented data), §9.4.5.4 (DATA_FRAG).
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Image

FRAME_SIZES = (16, 1400, 1500, 4000, 20000, 60000)
PAYLOAD_PARAM = "frag_payload"
PARAM_CHARS = 20000


def frame_pattern(seq: int, size: int) -> bytes:
    """Returns the deterministic payload for frame `seq` of `size` octets."""
    return bytes(((seq * 31) + i) & 0xFF for i in range(size))


def param_text(role: str, chars: int) -> str:
    """Returns the `chars`-long parameter string a role publishes."""
    unit = f"{role}-rosette-fragment-"
    return (unit * (chars // len(unit) + 1))[:chars]


def size_list(text: str) -> tuple[int, ...]:
    """Parses a comma-separated list of frame sizes."""
    return tuple(int(part) for part in text.split(",") if part)


class FragProbeNode(Node):
    """Publishes, verifies, and calls; counts results per frame size."""

    def __init__(self, role: str, session: str, period: float, args) -> None:
        super().__init__(f"{session}_{role}_frag")
        self.role = role
        self.peer = "beta" if role == "alpha" else "alpha"
        self.frame_sizes = size_list(args.frame_sizes)
        self.peer_text = param_text(self.peer, args.peer_param_chars)
        self.tx = 0
        self.ok_by_size = {size: 0 for size in size_list(args.peer_frame_sizes)}
        self.corrupt = 0
        self.param_ok = 0
        self.param_bad = 0
        self._inflight = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.declare_parameter(PAYLOAD_PARAM, param_text(role, args.param_chars))
        self._pub = self.create_publisher(Image, f"/{session}/{role}_frames", qos)
        self.create_subscription(Image, f"/{session}/{self.peer}_frames", self._on_frame, qos)
        self._params = self.create_client(GetParameters,
                                          f"/{session}_{self.peer}_frag/get_parameters")
        self.create_timer(period, self._tick)

    def _tick(self) -> None:
        size = self.frame_sizes[self.tx % len(self.frame_sizes)]
        msg = Image()
        msg.header.frame_id = str(self.tx)
        msg.height, msg.width, msg.step = 1, size, size
        msg.encoding = "mono8"
        msg.data = frame_pattern(self.tx, size)
        self._pub.publish(msg)
        self.tx += 1

        if self._params.service_is_ready() and (self._inflight is None or self._inflight.done()):
            self._inflight = self._params.call_async(GetParameters.Request(names=[PAYLOAD_PARAM]))
            self._inflight.add_done_callback(self._on_param)

    def _on_frame(self, msg: Image) -> None:
        seq = int(msg.header.frame_id)
        if bytes(msg.data) == frame_pattern(seq, msg.width) and msg.width in self.ok_by_size:
            self.ok_by_size[msg.width] += 1
        else:
            self.corrupt += 1
            self.get_logger().error(f"corrupt frame seq={seq} width={msg.width}")

    def _on_param(self, future) -> None:
        reply = future.result()
        values = reply.values if reply is not None else []
        if len(values) == 1 and values[0].string_value == self.peer_text:
            self.param_ok += 1
        else:
            self.param_bad += 1
            self.get_logger().error("get_parameters reply did not carry the peer's payload")


def main() -> int:
    """Runs one role; exits 0 only when every expectation holds."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--role", choices=("alpha", "beta"), required=True)
    parser.add_argument("--session", default="frag")
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--period-sec", type=float, default=0.2)
    parser.add_argument("--expect-per-size", type=int, default=3)
    parser.add_argument("--expect-calls", type=int, default=2)
    default_sizes = ",".join(str(s) for s in FRAME_SIZES)
    parser.add_argument("--frame-sizes", default=default_sizes,
                        help="comma-separated frame sizes this role publishes")
    parser.add_argument("--peer-frame-sizes", default=default_sizes,
                        help="comma-separated frame sizes expected from the peer")
    parser.add_argument("--param-chars", type=int, default=PARAM_CHARS)
    parser.add_argument("--peer-param-chars", type=int, default=PARAM_CHARS)
    args = parser.parse_args()

    rclpy.init()
    rmw = rclpy.get_rmw_implementation_identifier()
    expected_rmw = os.environ.get("RMW_IMPLEMENTATION")
    if expected_rmw and rmw != expected_rmw:
        print(f"[frag-probe] loaded {rmw}, expected {expected_rmw}", file=sys.stderr)
        return 1
    node = FragProbeNode(args.role, args.session, args.period_sec, args)
    deadline = time.monotonic() + args.duration_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    sizes = " ".join(f"{s}:{n}" for s, n in node.ok_by_size.items())
    print(f"[frag-probe] role={args.role} rmw={rmw} tx={node.tx} ok_by_size={sizes} "
          f"corrupt={node.corrupt} param_ok={node.param_ok} param_bad={node.param_bad}",
          flush=True)
    node.destroy_node()
    rclpy.shutdown()

    ok = (node.corrupt == 0 and node.param_bad == 0 and node.param_ok >= args.expect_calls
          and all(n >= args.expect_per_size for n in node.ok_by_size.values()))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
