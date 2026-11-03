#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
"""Discovery-scale topology: which participants a network has, and who matches whom.

Architecture:
    A topology file (YAML) describes a ROS 2 network by role: how many
    instances of the role exist, which participants (processes) each
    instance runs, the nodes in each participant, and each node's
    publishers, subscriptions, services, clients, action servers and action
    clients. Strings may contain `${expr}`, a Python expression over the
    instance index `i`, the role's instance count `n`, and, inside an
    endpoint with a `for` list, the list item `j`; that is how instance 3 of
    a vision station names the controller boards it drives.

    An endpoint's `fill` gives field values for what it sends (a
    publisher's messages, a client's requests, an action client's goals),
    so payloads have the sizes a real application's do rather than those
    of an empty message. A role's `lifetime_s` makes its participants come
    and go: each is shut down and created again after about that long, as
    command-line tools and short-lived processes do on a real network.

    expand() turns the file into a flat list of Participant records, each
    with its fully qualified endpoints. analyze() then pairs endpoints the
    way DDS discovery does (same name, same type, opposite direction) and
    reports, per participant, how many other participants it shares an
    application match with: the peers a board must keep full discovery state
    for. Endpoints every stock rclpy node creates on its own (parameter
    services, /rosout, /parameter_events) are counted toward a participant's
    endpoint total, since every peer announces them, but they match nothing
    here.

Verification:
    `python3 discovery_topology.py <file>` prints the participant
    count, the endpoint count, and the matched-peer distribution per role;
    a topology whose roles share no names reports zero matched peers for
    every participant.

References:
    - OMG DDSI-RTPS 2.5 §8.5.5.1 (endpoints match on topic and type name).
    - ROS 2 design, "Topic and Service name mapping to DDS" (rt/, rq/, rr/
      prefixes; an action is three services and two topics).
"""

from __future__ import annotations

import argparse
import dataclasses
import re
import statistics
import sys
from collections import defaultdict

import yaml

ENDPOINT_KINDS = ("publisher", "subscription", "service", "client", "action_server",
                  "action_client")

# DDS endpoints one ROS entity creates: (writers, readers).
DDS_ENDPOINTS = {
    "publisher": (1, 0),
    "subscription": (0, 1),
    "service": (1, 1),
    "client": (1, 1),
    "action_server": (5, 3),  # 3 reply writers + feedback and status writers
    "action_client": (3, 5),  # 3 request writers; 3 reply + feedback + status readers
}

# A stock rclpy node: 6 parameter services, /rosout and /parameter_events writers.
STOCK_NODE_ENDPOINTS = 6 * 2 + 2

# The kind on the other side of a match.
COUNTERPART = {
    "publisher": "subscription",
    "subscription": "publisher",
    "service": "client",
    "client": "service",
    "action_server": "action_client",
    "action_client": "action_server",
}


@dataclasses.dataclass(frozen=True)
class Endpoint:
    """One ROS entity of a node, with its fully qualified name."""

    kind: str
    name: str
    type: str
    qos: str = "reliable"
    rate_hz: float = 0.0
    probe_period_s: float = 0.0
    fill: dict | None = dataclasses.field(default=None, compare=False)


@dataclasses.dataclass
class Node:
    """A ROS node: its name, namespace, whether it is a stock rclpy node, its entities."""

    name: str
    namespace: str
    stock: bool
    endpoints: list[Endpoint]

    @property
    def fqn(self) -> str:
        """The node's fully qualified name."""
        return f"{self.namespace.rstrip('/')}/{self.name}"


@dataclasses.dataclass
class Participant:
    """One DDS participant (one ROS process) of one role instance."""

    role: str
    index: int
    slot: int
    slots: int
    nodes: list[Node]
    lifetime_s: float = 0.0

    @property
    def label(self) -> str:
        """`role:index` or `role:index.slot` for a role with several participants."""
        return f"{self.role}:{self.index}" + (f".{self.slot}" if self.slots > 1 else "")

    def dds_endpoints(self) -> int:
        """DDS writers plus readers this participant announces through SEDP."""
        total = 0
        for node in self.nodes:
            total += STOCK_NODE_ENDPOINTS if node.stock else 0
            for ep in node.endpoints:
                total += sum(DDS_ENDPOINTS[ep.kind])

        return total


_TEMPLATE = re.compile(r"\$\{([^}]+)\}")


def substitute(text: str, env: dict) -> str:
    """Replaces every `${expr}` in `text` with the value of `expr` over `env`.

    Args:
        text: A string from the topology file.
        env: Names visible to the expressions (`i`, `n`, and `j` in a `for`).

    Returns:
        The string with each expression replaced by its value.
    """
    return _TEMPLATE.sub(lambda m: str(eval(m.group(1), {"__builtins__": {}}, env)), text)


def resolve(name: str, node: Node) -> str:
    """Qualifies an entity name the way rcl does: `~/x`, relative `x`, or absolute `/x`."""
    if name.startswith("~/"):
        return f"{node.fqn}/{name[2:]}"

    if name.startswith("/"):
        return name

    return f"{node.namespace.rstrip('/')}/{name}"


def expand_endpoints(specs: list[dict], node: Node, env: dict) -> list[Endpoint]:
    """Expands a node's endpoint list, repeating an entry once per item of its `for` list.

    Args:
        specs: The node's `endpoints` entries from the topology file.
        node: The node the endpoints belong to; names are resolved against it.
        env: The instance's template names.

    Returns:
        The node's endpoints with resolved names.
    """
    out = []
    for spec in specs:
        if spec["kind"] not in ENDPOINT_KINDS:
            raise ValueError(f"unknown endpoint kind {spec['kind']!r}")

        items = [None]
        if "for" in spec:
            items = list(eval(str(spec["for"]), {"__builtins__": {"range": range}}, env))

        for j in items:
            local = dict(env, j=j)
            out.append(Endpoint(
                kind=spec["kind"],
                name=resolve(substitute(spec["name"], local), node),
                type=spec["type"],
                qos=spec.get("qos", "reliable"),
                rate_hz=float(spec.get("rate_hz", 0.0)),
                probe_period_s=float(spec.get("probe_period_s", 0.0)),
                fill=spec.get("fill"),
            ))

    return out


def expand(topology: dict) -> list[Participant]:
    """Turns a topology file's roles into the network's participants.

    Args:
        topology: The parsed topology file.

    Returns:
        Every participant, in role order, then instance order.
    """
    participants = []
    for role, spec in topology["roles"].items():
        count = int(spec.get("count", 1))
        for i in range(count):
            env = {"i": i, "n": count}
            for slot, pspec in enumerate(spec["participants"]):
                nodes = []
                for nspec in pspec["nodes"]:
                    node = Node(name=substitute(nspec["name"], env),
                                namespace=substitute(nspec.get("namespace", "/"), env),
                                stock=bool(nspec.get("stock", True)), endpoints=[])
                    node.endpoints = expand_endpoints(nspec.get("endpoints", []), node, env)
                    nodes.append(node)

                participants.append(Participant(role, i, slot, len(spec["participants"]), nodes,
                                                float(spec.get("lifetime_s", 0.0))))

    return participants


def analyze(participants: list[Participant]) -> dict[str, set[str]]:
    """Finds, for each participant, the other participants it shares a match with.

    Args:
        participants: The expanded network.

    Returns:
        A map from participant label to the labels of its matched peers.
    """
    by_key = defaultdict(set)
    for p in participants:
        for node in p.nodes:
            for ep in node.endpoints:
                by_key[(ep.kind, ep.name, ep.type)].add(p.label)

    peers = {p.label: set() for p in participants}
    for (kind, name, type_), owners in by_key.items():
        others = by_key.get((COUNTERPART[kind], name, type_), set())
        for owner in owners:
            peers[owner].update(o for o in others if o != owner)

    return peers


def load(path: str) -> dict:
    """Reads a topology file."""
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def print_analysis(participants: list[Participant], focus: str | None) -> None:
    """Prints the network's size and each role's matched-peer distribution."""
    peers = analyze(participants)
    endpoints = sum(p.dds_endpoints() for p in participants)
    print(f"participants: {len(participants)}")
    print(f"DDS endpoints announced: {endpoints}")
    print(f"{'role':<24} {'parts':>5} {'endpoints':>9} {'matched peers min/median/max':>30}")
    roles = {}
    for p in participants:
        roles.setdefault(p.role, []).append(p)

    for role, members in roles.items():
        counts = [len(peers[p.label]) for p in members]
        eps = members[0].dds_endpoints()
        spread = f"{min(counts)}/{statistics.median(counts):g}/{max(counts)}"
        print(f"{role:<24} {len(members):>5} {eps:>9} {spread:>30}")

    if focus:
        print(f"\n{focus} matches {len(peers[focus])} peers:")
        for label in sorted(peers[focus]):
            print(f"  {label}")


def main() -> int:
    """Analyzes a topology file from the command line."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("topology")
    parser.add_argument("--focus", help="list the matched peers of this participant (role:index)")
    parser.add_argument("--node-of", help="print the first node of this participant and exit")
    args = parser.parse_args()
    participants = expand(load(args.topology))
    if args.node_of:
        found = [p.nodes[0].fqn for p in participants if p.label == args.node_of]
        if not found:
            print(f"no participant {args.node_of}", file=sys.stderr)
            return 1

        print(found[0])
        return 0

    print_analysis(participants, args.focus)
    return 0


if __name__ == "__main__":
    sys.exit(main())
