#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
"""Peer-table probe: a participant that joins after the table fills must still reach the board.

Architecture:
    All three roles of run_rmw_peer_lockout_loopback.sh. Role `board` runs on
    the RMW under test, built with room for few remote participants, and
    serves `std_srvs/SetBool` on `/<session>/board_echo`. Role `filler` runs
    on a stock RMW and hosts a node that uses none of the board's services
    or topics; enough fillers start first to take every one of the board's
    participant slots. Role `late` runs on a stock RMW, starts last, waits for
    the board's service and calls it.

    The board learns a participant from its SPDP announcement and keeps it
    until its lease expires (DDSI-RTPS 2.5 §8.5.3.2). With every slot taken by
    a filler, the board drops the late participant's announcements, so it
    neither reads that participant's SEDP announcements nor sends its own
    (§8.5.4.1): the late participant never sees the board's service, though
    the board would match its client.

Verification:
    The late role passes when the board's service appears within
    `--wait-sec` and `--expect-calls` calls each return the value they sent.
    The board and filler roles pass when they run for `--duration-sec`
    on the expected RMW.

References:
    - OMG DDSI-RTPS 2.5 §8.5.3.2 (SPDP participant data, lease), §8.5.4.1
      (SEDP builtin endpoint matching).
    - rclpy Client.wait_for_service / call_async (Humble).
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from std_msgs.msg import String
from std_srvs.srv import SetBool


def parse_args() -> argparse.Namespace:
    """Parses the command line.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--role", choices=("board", "filler", "late"), required=True)
    parser.add_argument("--session", required=True, help="names this run's nodes and service")
    parser.add_argument("--index", type=int, default=0, help="a filler's number")
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument("--wait-sec", type=float, default=20.0,
                        help="how long the late role waits for the board's service")
    parser.add_argument("--expect-calls", type=int, default=3)
    parser.add_argument("--expect-rmw", required=True)
    return parser.parse_args()


def spin_for(node, duration_sec: float) -> None:
    """Spins `node` for `duration_sec` seconds.

    Args:
        node: The rclpy node to spin.
        duration_sec: How long to spin.
    """
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)


def serve(node, session: str, duration_sec: float) -> bool:
    """Serves the board's echo service until the session ends.

    Args:
        node: The board's rclpy node.
        session: The run's session name.
        duration_sec: How long to serve.

    Returns:
        True; the board's outcome is judged by the late role.
    """
    calls = []

    def echo(request, response):
        calls.append(request.data)
        response.success = request.data
        response.message = f"echo {len(calls)}"
        return response

    node.create_service(SetBool, f"/{session}/board_echo", echo)
    spin_for(node, duration_sec)
    print(f"[board] served {len(calls)} calls", flush=True)
    return True


def fill(node, session: str, duration_sec: float) -> bool:
    """Keeps a participant alive that shares nothing with the board.

    Args:
        node: The filler's rclpy node.
        session: The run's session name.
        duration_sec: How long to stay.

    Returns:
        True.
    """
    publisher = node.create_publisher(String, f"/{session}/{node.get_name()}_only", 1)
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        publisher.publish(String(data=node.get_name()))
        rclpy.spin_once(node, timeout_sec=1.0)

    return True


def call_board(node, session: str, wait_sec: float, expect_calls: int) -> bool:
    """Waits for the board's echo service and calls it.

    Args:
        node: The late participant's rclpy node.
        session: The run's session name.
        wait_sec: How long to wait for the service to appear.
        expect_calls: Calls that must each return the value they sent.

    Returns:
        True when the service appeared and every call was answered correctly.
    """
    client = node.create_client(SetBool, f"/{session}/board_echo")
    start = time.monotonic()
    if not client.wait_for_service(timeout_sec=wait_sec):
        print(f"[late] FAIL: the board's service never appeared in {wait_sec:.0f} s", flush=True)
        return False

    print(f"[late] the board's service appeared after {time.monotonic() - start:.1f} s", flush=True)
    for i in range(expect_calls):
        value = i % 2 == 0
        future = client.call_async(SetBool.Request(data=value))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done() or future.result() is None or future.result().success != value:
            print(f"[late] FAIL: call {i + 1} was not answered with {value}", flush=True)
            return False

    print(f"[late] PASS: {expect_calls} calls answered", flush=True)
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

    name = f"{args.session}_{args.role}" + (f"{args.index}" if args.role == "filler" else "")
    node = rclpy.create_node(name)
    if args.role == "board":
        ok = serve(node, args.session, args.duration_sec)
    elif args.role == "filler":
        ok = fill(node, args.session, args.duration_sec)
    else:
        ok = call_board(node, args.session, args.wait_sec, args.expect_calls)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
