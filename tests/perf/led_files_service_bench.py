#!/usr/bin/env python3
"""Service call latency of the examples/led_files board, from a stock rclpy client.

examples/led_files is one rclc source that builds against either ROS 2 backend
Rosette supports: rmw_rosette (the board answers DDS clients directly) or the
micro-ROS agent (a micro_ros_agent on the host answers on the board's behalf).
This script drives its services the same way in both cases, so running it once
per backend compares the two paths end to end: host middleware, network, board
middleware, and the board's executor.

Every call is a synchronous round trip: the next request is sent only after the
previous response arrived (or timed out), which is how a ROS 2 tool or a
supervisory node uses a board service. The phases are:

  led     SetLed, alternating the LED; the smallest request and response.
  put     PutFile of a body of each size under one key (a flash write).
  get     GetFile of that key; the body is checked against what was put.
          Skipped for a size whose puts all failed, since the key then holds
          an earlier body.
  list    ListFiles.
  erase   EraseFile of the key, once, at the end.

A body larger than one 1472-octet datagram travels as RTPS DATA_FRAG
submessages on rmw_rosette and as fragmented XRCE messages on the agent path,
so the size sweep crosses the single-datagram boundary on purpose.

Usage (Humble sourced, plus an overlay holding rosette_msgs):
  tests/perf/led_files_service_bench.py [--count 200] [--sizes 16,1024,4000]

The client's middleware is whatever RMW_IMPLEMENTATION selects; leave it unset
for stock Fast DDS, the interoperability baseline. A stock Fast DDS client
sends a request larger than one datagram as a single IP-fragmented UDP
datagram, which a WIZnet board's hardware UDP cannot reassemble, so puts above
about 1400 octets fail unless the client caps its datagrams (--capped-profile
writes and uses a Fast DDS profile with maxMessageSize 1472, the
configuration tests/ros2/run_rmw_frag_loopback.sh calls "capped").

Verification: every call returns before --timeout, every get returns exactly
the body the preceding put stored, and the summary counts both kinds of failure
so a run that loses or corrupts a call is visible in its output.

References: ROS 2 Humble rclpy Client API (call_async, wait_for_service);
examples/led_files/main.cpp for the services and kBodyMax (4000 octets).
"""

import argparse
import json
import os
import statistics
import string
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rosette_msgs.srv import EraseFile, GetFile, ListFiles, PutFile, SetLed

KEY = "bench"

CAPPED_PROFILE = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_capped</transport_id>
      <type>UDPv4</type>
      <maxMessageSize>1472</maxMessageSize>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="capped" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>udp_capped</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
"""


def use_capped_profile():
    """Points Fast DDS at a profile that keeps every datagram within 1472 octets.

    Fast DDS reads FASTRTPS_DEFAULT_PROFILES_FILE when the participant is
    created, so this must run before rclpy.init(). Other RMWs ignore it.

    References: Fast DDS 2.6 XML profiles, transport_descriptor maxMessageSize.
    """
    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as f:
        f.write(CAPPED_PROFILE)
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = f.name
    os.environ["RMW_FASTRTPS_USE_QOS_FROM_XML"] = "0"


def percentile(sorted_values, fraction):
    """Returns the nearest-rank percentile of an ascending list.

    Args:
        sorted_values: Samples in ascending order; not empty.
        fraction: The percentile as a fraction in [0, 1].

    Returns:
        The sample at rank ceil(fraction * n), clamped to the list.
    """
    index = max(0, min(len(sorted_values) - 1, int(round(fraction * len(sorted_values))) - 1))
    return sorted_values[index]


def summarize(phase, size, rtts_ms, failures, elapsed_s):
    """Builds one result row from a phase's round-trip times.

    Args:
        phase: The phase name (led, put, get, list).
        size: The body size in octets, or 0 for phases without a body.
        rtts_ms: Round-trip times of the calls that succeeded, in milliseconds.
        failures: Calls that timed out, failed, or returned a wrong body.
        elapsed_s: Wall time of the whole phase, in seconds.

    Returns:
        A dict with the count, failures, calls per second, and the latency
        distribution (min, p50, p95, p99, max, mean) in milliseconds.
    """
    row = {"phase": phase, "size": size, "ok": len(rtts_ms), "failed": failures,
           "calls_per_s": (len(rtts_ms) + failures) / elapsed_s if elapsed_s > 0 else 0.0}
    if rtts_ms:
        ordered = sorted(rtts_ms)
        row.update(min=ordered[0], p50=percentile(ordered, 0.50), p95=percentile(ordered, 0.95),
                   p99=percentile(ordered, 0.99), max=ordered[-1], mean=statistics.fmean(ordered))
    return row


class ServiceBench(Node):
    """An rclpy node holding one client per led_files service."""

    def __init__(self, namespace, timeout_s):
        """Creates the five clients under the board's namespace.

        Args:
            namespace: The board node's namespace, e.g. /led_files.
            timeout_s: Seconds a call may take before it counts as failed.
        """
        super().__init__("led_files_service_bench")
        self.timeout_s = timeout_s
        ns = namespace.rstrip("/")
        self.led = self.create_client(SetLed, f"{ns}/led/set")
        self.put = self.create_client(PutFile, f"{ns}/files/put")
        self.get = self.create_client(GetFile, f"{ns}/files/get")
        self.list = self.create_client(ListFiles, f"{ns}/files/list")
        self.erase = self.create_client(EraseFile, f"{ns}/files/erase")

    def wait_for_services(self, timeout_s):
        """Waits until every service is discovered.

        Args:
            timeout_s: Seconds to wait for each service.

        Returns:
            The names of the services that did not appear; empty on success.
        """
        clients = (self.led, self.put, self.get, self.list, self.erase)
        return [c.srv_name for c in clients if not c.wait_for_service(timeout_sec=timeout_s)]

    def call(self, client, request):
        """Sends one request and spins until its response or the timeout.

        Args:
            client: The rclpy client to call.
            request: The request message.

        Returns:
            (response, rtt_ms): the response, or None on timeout, and the
            round-trip time measured on the host's monotonic clock.
        """
        start = time.perf_counter()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_s)
        rtt_ms = (time.perf_counter() - start) * 1e3
        if not future.done():
            client.remove_pending_request(future)
            return None, rtt_ms

        return future.result(), rtt_ms

    def run_phase(self, phase, size, count, make_request, check):
        """Makes `count` sequential calls and summarizes them.

        Args:
            phase: The phase name for the result row.
            size: The body size in octets, or 0.
            count: How many calls to make.
            make_request: Returns (client, request) for call i.
            check: Returns True when a response is correct.

        Returns:
            The summarize() row for this phase.
        """
        rtts, failures = [], 0
        start = time.perf_counter()
        for i in range(count):
            client, request = make_request(i)
            response, rtt_ms = self.call(client, request)
            if response is None or not check(response):
                failures += 1
            else:
                rtts.append(rtt_ms)

        return summarize(phase, size, rtts, failures, time.perf_counter() - start)


def body_of(size):
    """Returns a printable body of `size` octets that differs at every offset mod 62."""
    alphabet = string.ascii_letters + string.digits
    return (alphabet * (size // len(alphabet) + 1))[:size]


def run(bench, args):
    """Runs every phase and returns the result rows.

    Args:
        bench: The ServiceBench node, its services already discovered.
        args: Parsed command-line arguments.

    Returns:
        A list of summarize() rows in phase order.
    """
    rows = []
    for i in range(args.warmup):
        bench.call(bench.led, SetLed.Request(led=0, state=bool(i & 1)))

    rows.append(bench.run_phase(
        "led", 0, args.count,
        lambda i: (bench.led, SetLed.Request(led=0, state=bool(i & 1))),
        lambda r: r.success))
    for size in args.sizes:
        body = body_of(size)
        put_row = bench.run_phase(
            "put", size, args.put_count,
            lambda i, b=body: (bench.put, PutFile.Request(key=KEY, body=b)),
            lambda r: r.success)
        rows.append(put_row)
        # A get checks the body the last put stored, so it measures nothing
        # when no put of this size arrived; it is reported as skipped.
        if put_row["ok"] == 0:
            rows.append({"phase": "get", "size": size, "ok": 0, "failed": 0, "skipped": True,
                         "calls_per_s": 0.0})
            continue

        rows.append(bench.run_phase(
            "get", size, args.count,
            lambda i: (bench.get, GetFile.Request(key=KEY)),
            lambda r, b=body: r.success and r.body == b))

    rows.append(bench.run_phase(
        "list", 0, args.count, lambda i: (bench.list, ListFiles.Request()), lambda r: KEY in r.keys))
    response, _ = bench.call(bench.erase, EraseFile.Request(key=KEY))
    if response is None or not response.success:
        print("erase of the bench key failed", file=sys.stderr)

    return rows


def print_table(rows):
    """Prints the result rows as an aligned table, latencies in milliseconds."""
    print(f"{'phase':<6} {'size':>5} {'ok':>5} {'fail':>4} {'call/s':>7} "
          f"{'min':>7} {'p50':>7} {'p95':>7} {'p99':>7} {'max':>7}")
    for r in rows:
        stats = " ".join(f"{r[k]:7.2f}" for k in ("min", "p50", "p95", "p99", "max")) if r["ok"] else ""
        if r.get("skipped"):
            stats = "skipped: no put of this size arrived"
        print(f"{r['phase']:<6} {r['size']:>5} {r['ok']:>5} {r['failed']:>4} {r['calls_per_s']:7.1f} {stats}")


def parse_args(argv):
    """Parses the command line; see the module docstring for the phases."""
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--namespace", default="/led_files", help="board node namespace")
    p.add_argument("--count", type=int, default=200, help="calls per led/get/list phase")
    p.add_argument("--put-count", type=int, default=20,
                   help="puts per size; each is a flash write, so keep it modest")
    p.add_argument("--sizes", type=lambda s: [int(x) for x in s.split(",")],
                   default=[16, 256, 1024, 1400, 2048, 4000], help="body sizes in octets (max 4000)")
    p.add_argument("--warmup", type=int, default=20, help="unmeasured SetLed calls first")
    p.add_argument("--timeout", type=float, default=1.0, help="seconds before a call counts as failed")
    p.add_argument("--discovery-timeout", type=float, default=10.0, help="seconds to find each service")
    p.add_argument("--capped-profile", action="store_true",
                   help="cap Fast DDS datagrams at 1472 octets so large requests reach the board")
    p.add_argument("--json", help="also write the rows to this file")
    return p.parse_args(argv)


def main(argv=None):
    """Discovers the board's services, runs every phase, and prints the table.

    Returns:
        0 when every call succeeded, 1 when any failed, 2 when a service was
        never discovered.
    """
    args = parse_args(argv)
    if args.capped_profile:
        use_capped_profile()

    rclpy.init()
    bench = ServiceBench(args.namespace, args.timeout)
    try:
        start = time.perf_counter()
        missing = bench.wait_for_services(args.discovery_timeout)
        if missing:
            print(f"services not discovered: {', '.join(missing)}", file=sys.stderr)
            return 2

        print(f"services discovered in {time.perf_counter() - start:.2f} s")
        rows = run(bench, args)
        print_table(rows)
        if args.json:
            with open(args.json, "w", encoding="utf-8") as f:
                json.dump(rows, f, indent=2)

        return 1 if any(r["failed"] for r in rows) else 0
    finally:
        bench.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
