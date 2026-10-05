#!/usr/bin/env python3
"""Calls one service at a fixed rate and reports its round trips.

Architecture:
    One rclpy node waits for the service, then sends a call every period,
    one at a time: the next call goes out when the previous one is answered
    or has timed out. Each round trip is recorded with the time it was sent,
    so a run under load can be compared with a quiet run, and stalls placed
    on the run's timeline. With the device on real hardware and
    discovery_scale_sim.py providing the network around it, the probe
    measures how the network's discovery traffic affects a peer the device
    is matched with.

Verification:
    Prints the number of calls, timeouts, p50/p90/p99/max round trip and
    writes every sample to --report (JSON); exits 1 when the service never
    appeared or a call timed out.

References:
    - rclpy Client.call_async, wait_for_service (Humble).
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time

import rclpy
from rosidl_runtime_py.utilities import get_service


def parse_args() -> argparse.Namespace:
    """Parses the command line."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("service", help="fully qualified service name")
    parser.add_argument("type", help="service type, e.g. std_srvs/srv/Trigger")
    parser.add_argument("--period-ms", type=float, default=100.0)
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--timeout-sec", type=float, default=2.0)
    parser.add_argument("--wait-sec", type=float, default=60.0, help="wait this long for the service")
    parser.add_argument("--report", help="write every sample here (JSON)")
    return parser.parse_args()


def main() -> int:
    """Runs the probe and prints its summary."""
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("service_rtt_probe")
    srv_type = get_service(args.type)
    client = node.create_client(srv_type, args.service)
    t0 = time.monotonic()
    if not client.wait_for_service(timeout_sec=args.wait_sec):
        print(f"[probe] {args.service} did not appear within {args.wait_sec:.0f} s")
        return 1

    print(f"[probe] {args.service} ready after {time.monotonic() - t0:.1f} s", flush=True)
    samples = []
    timeouts = 0
    start = time.monotonic()
    while time.monotonic() - start < args.duration_sec:
        sent = time.monotonic()
        future = client.call_async(srv_type.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout_sec)
        if future.done():
            samples.append((sent - start, (time.monotonic() - sent) * 1000.0))
        else:
            client.remove_pending_request(future)
            timeouts += 1
            samples.append((sent - start, None))

        time.sleep(max(0.0, args.period_ms / 1000.0 - (time.monotonic() - sent)))

    rtts = [r for _t, r in samples if r is not None]
    if len(rtts) > 1:
        q = statistics.quantiles(rtts, n=100, method="inclusive")
        print(f"[probe] {len(samples)} calls, {timeouts} timed out; p50 {q[49]:.1f} ms, "
              f"p90 {q[89]:.1f} ms, p99 {q[98]:.1f} ms, max {max(rtts):.1f} ms")

    if args.report:
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump({"service": args.service, "samples": samples, "timeouts": timeouts}, f)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if rtts and timeouts == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
