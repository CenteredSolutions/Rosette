#!/usr/bin/env python3
"""Runs a topology's participants in one process, each in its own rclpy context.

Architecture:
    Each participant of the topology (discovery_topology.py) gets its own
    rclpy Context, so it is its own DDS participant exactly as a separate
    process would be, with its nodes, entities and a SingleThreadedExecutor
    spinning in a thread of its own. Ninety stock participants then cost one
    Python process instead of ninety.

    One run plays the network around a device under test. `--exclude` names
    the participants another process provides (typically one controller
    board on the RMW under test, started with `--only`). For every simulated
    participant that shares a match with an excluded one, the run records
    when each of those matching entities became matched: a client or action
    client when its server is ready, a publisher when it has a subscriber, a
    subscription when its first message arrives. `--watch-node` records when
    each participant's graph first lists a node, which is how stock peers
    learn the device's nodes from `ros_discovery_info`. Clients with a
    `probe_period_s` call their service on that period and record round
    trips, which measures the device's latency to a matched peer under the
    load of the whole network's discovery.

Verification:
    The run passes when every simulated participant that matches an excluded
    one matched all of its entities, every participant listed the watched
    node, and every probe completed at least one call; otherwise it reports
    which ones did not and exits 1.

References:
    - rclpy Context, Node, SingleThreadedExecutor, ActionServer and
      ActionClient (Humble).
    - OMG DDSI-RTPS 2.5 §8.5 (discovery), which every context runs on its own.
"""

from __future__ import annotations

import argparse
import json
import resource
import statistics
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_action, get_message, get_service

import discovery_topology as topo


class SimParticipant:
    """One simulated participant: its context, nodes, entities and observations."""

    def __init__(self, spec: topo.Participant, domain: int, facing: set, watch: str | None,
                 t0: float) -> None:
        """Creates the participant's context, nodes and entities.

        Args:
            spec: The participant from the topology.
            domain: DDS domain id.
            facing: (kind, name, type) keys of entities that match the device under test.
            watch: Fully qualified node name whose appearance in the graph is recorded.
            t0: The run's start time (time.monotonic()).
        """
        self.spec = spec
        self.t0 = t0
        self.watch = watch
        self.watch_seen = None
        self.matched_at = {}      # entity name -> seconds after t0
        self.facing = []          # (entity name, kind, handle) still to see matched
        self.probes = {}          # service name -> list of round trips in ms
        self.facing_probes = set()  # probed services the device under test provides
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.nodes = []
        for nspec in spec.nodes:
            node = rclpy.create_node(nspec.name, namespace=nspec.namespace, context=self.context,
                                     start_parameter_services=nspec.stock,
                                     enable_rosout=nspec.stock)
            for ep in nspec.endpoints:
                handle = self._create(node, ep)
                if (ep.kind, ep.name, ep.type) in facing:
                    self.facing.append((ep.name, ep.kind, handle))
                    if ep.probe_period_s > 0:
                        self.facing_probes.add(ep.name)

            self.nodes.append(node)
            self.executor.add_node(node)

        self.nodes[0].create_timer(0.5, self._observe)
        self.thread = threading.Thread(target=self._spin, daemon=True)

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except Exception:  # The context shut down under the executor.
            pass

    def _create(self, node, ep: topo.Endpoint):
        """Creates one entity and, where the topology asks for it, its timer."""
        qos = qos_profile_sensor_data if ep.qos == "best_effort" else QoSProfile(depth=10)
        if ep.kind == "publisher":
            msg_type = get_message(ep.type)
            pub = node.create_publisher(msg_type, ep.name, qos)
            if ep.rate_hz > 0:
                node.create_timer(1.0 / ep.rate_hz, lambda: pub.publish(msg_type()))
            return pub

        if ep.kind == "subscription":
            return node.create_subscription(get_message(ep.type), ep.name,
                                            lambda _msg, n=ep.name: self._received(n), qos)

        if ep.kind == "service":
            return node.create_service(get_service(ep.type), ep.name,
                                       lambda _req, res: res)

        if ep.kind == "client":
            client = node.create_client(get_service(ep.type), ep.name)
            if ep.probe_period_s > 0:
                self.probes[ep.name] = []
                node.create_timer(ep.probe_period_s,
                                  lambda c=client, t=ep.type, n=ep.name: self._probe(c, t, n))
            return client

        if ep.kind == "action_server":
            action_type = get_action(ep.type)

            def execute(goal):
                goal.succeed()
                return action_type.Result()

            return ActionServer(node, action_type, ep.name, execute)

        return ActionClient(node, get_action(ep.type), ep.name)

    def _received(self, name: str) -> None:
        self.matched_at.setdefault(name, time.monotonic() - self.t0)

    def _probe(self, client, type_name: str, name: str) -> None:
        """Calls a probed service once, if it is available, and records the round trip."""
        if not client.service_is_ready():
            return

        start = time.monotonic()
        future = client.call_async(get_service(type_name).Request())
        future.add_done_callback(
            lambda _f: self.probes[name].append((time.monotonic() - start) * 1000.0))

    def _observe(self) -> None:
        """Records entities that became matched and whether the watched node is listed."""
        now = time.monotonic() - self.t0
        for name, kind, handle in self.facing:
            if name in self.matched_at:
                continue

            if ((kind == "client" and handle.service_is_ready())
                    or (kind == "action_client" and handle.server_is_ready())
                    or (kind == "publisher" and handle.get_subscription_count() > 0)):
                self.matched_at[name] = now

        if self.watch and self.watch_seen is None:
            names = {f"{ns.rstrip('/')}/{n}" for n, ns in
                     self.nodes[0].get_node_names_and_namespaces()}
            if self.watch in names:
                self.watch_seen = now

    def pending(self) -> list[str]:
        """Device-facing entities whose match this run can observe and has not seen."""
        observable = ("client", "action_client", "publisher", "subscription")
        return [n for n, kind, _h in self.facing if kind in observable and n not in self.matched_at]

    def report(self) -> dict:
        """This participant's observations, for the JSON report."""
        matched = [self.matched_at[n] for n, _k, _h in self.facing if n in self.matched_at]
        return {
            "participant": self.spec.label,
            "faces_device": bool(self.facing),
            "pending": self.pending(),
            "matched_by_s": max(matched) if matched and not self.pending() else None,
            "watch_seen_s": self.watch_seen,
            "probes": {n: rtts for n, rtts in self.probes.items()},
            "device_probes": sorted(self.facing_probes),
        }

    def shutdown(self) -> None:
        """Shuts the context down, which ends the executor's spin, then destroys the nodes."""
        rclpy.try_shutdown(context=self.context)
        self.thread.join(timeout=5.0)
        self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()


def parse_args() -> argparse.Namespace:
    """Parses the command line."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("topology")
    parser.add_argument("--domain", type=int, default=44)
    parser.add_argument("--exclude", action="append", default=[],
                        help="participant label another process provides (repeatable)")
    parser.add_argument("--only", action="append", default=[],
                        help="run only these participant labels (repeatable)")
    parser.add_argument("--watch-node", help="fully qualified node name to look for in each graph")
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--stagger-ms", type=float, default=0.0,
                        help="pause between starting participants (0: all at once)")
    parser.add_argument("--report", help="write the per-participant observations here (JSON)")
    return parser.parse_args()


def summarize(sims: list[SimParticipant], watch: str | None, startup_s: float) -> bool:
    """Prints the run's outcome and returns whether it passed."""
    reports = [s.report() for s in sims]
    facing = [r for r in reports if r["faces_device"]]
    unmatched = [r for r in facing if r["pending"]]
    rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    print(f"[sim] {len(sims)} participants started in {startup_s:.1f} s, peak RSS {rss_mb:.0f} MB")
    print(f"[sim] {len(facing) - len(unmatched)} of {len(facing)} participants that match the "
          f"device matched it")
    for r in unmatched:
        print(f"[sim]   {r['participant']}: never matched {', '.join(r['pending'])}")

    ok = not unmatched
    if watch:
        seen = [r["watch_seen_s"] for r in reports if r["watch_seen_s"] is not None]
        latest = f", last after {max(seen):.1f} s" if seen else ""
        print(f"[sim] {len(seen)} of {len(reports)} participants list {watch}{latest}")
        ok = ok and len(seen) == len(reports)

    # Probes of the device decide the outcome; probes of simulated peers
    # are the same measurement without the device, for comparison.
    for r in reports:
        for name, rtts in r["probes"].items():
            to_device = name in r["device_probes"]
            tag = "device" if to_device else "peer"
            if not rtts:
                print(f"[sim] {tag} probe {r['participant']} {name}: no call completed")
                ok = ok and not to_device
                continue

            q = [rtts[0]] * 99
            if len(rtts) > 1:
                q = statistics.quantiles(rtts, n=100, method="inclusive")

            print(f"[sim] {tag} probe {r['participant']} {name}: {len(rtts)} calls, "
                  f"p50 {q[49]:.1f} ms, p99 {q[98]:.1f} ms, max {max(rtts):.1f} ms")

    return ok


def main() -> int:
    """Runs the simulated participants for the requested time."""
    args = parse_args()
    everyone = topo.expand(topo.load(args.topology))
    device = [p for p in everyone if p.label in args.exclude]
    facing = {(topo.COUNTERPART[ep.kind], ep.name, ep.type)
              for p in device for n in p.nodes for ep in n.endpoints}
    chosen = [p for p in everyone
              if p.label not in args.exclude and (not args.only or p.label in args.only)]

    t0 = time.monotonic()
    sims = []
    for spec in chosen:
        sim = SimParticipant(spec, args.domain, facing, args.watch_node, t0)
        sim.thread.start()
        sims.append(sim)
        if args.stagger_ms > 0:
            time.sleep(args.stagger_ms / 1000.0)

    startup_s = time.monotonic() - t0
    print(f"[sim] running {len(sims)} participants for {args.duration_sec:.0f} s", flush=True)
    time.sleep(max(0.0, args.duration_sec - startup_s))
    ok = summarize(sims, args.watch_node, startup_s)
    if args.report:
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump([s.report() for s in sims], f, indent=1)

    for sim in sims:
        sim.shutdown()

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
