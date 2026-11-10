# Formal models

TLA+ models of Rosette's discovery protocol, checked with TLC. They are
layered so that each Rosette choice is checked against every behavior the
DDSI-RTPS 2.5 specification allows a peer, not against one vendor's:

| Module | Holds |
|---|---|
| `SedpReliable.tla` | What the specification requires and permits: a peer's SEDP announcer, a network that loses, duplicates and reorders, and the WriterProxy bookkeeping of §8.4.10.4. No Rosette choice. |
| `RosettePeerTiers.tla` | Rosette's choices on top: the two-tier peer table (`docs/design/two_tier_peer_table.md`) and the graph cache's refresh of a reactivated peer. |
| `MCRosettePeerTiers.tla` | The model-checking harness (endpoint symmetry). |
| `RosettePeerTable.tla` | The table as a whole: several peers competing for fewer active slots, with each peer's SEDP exchange abstracted to one step. |

Where Rosette has a choice to make, the module carries it as a constant, with
the code's choice and its alternatives side by side, so a check shows what
each choice costs.

## Running

```sh
tests/formal/run_tlc.sh <delivery> <sweep_rule> <park_rule> <property> [max_sn] [net_cap]
tests/formal/run_tlc.sh in_order caught_up keep_refresh_target EventuallyConsistent 3 2
```

```sh
tests/formal/run_table.sh <property> [peers] [slots] [records] [unreachable] [budget]
tests/formal/run_table.sh EveryPeerServed 3 2 3 '{1}' 4
```

TLC runs in an `eclipse-temurin:21-jre` container; `tla2tools.jar` 1.7.4 is
expected under `~/.local/share/tlaplus/v1.7.4` (or `TLA_TOOLS_DIR`) and is
checked against its pinned SHA-256. Safety properties use endpoint symmetry;
`EventuallyConsistent` is a liveness property, checked under `FairSpec`
without it.

## Modelling assumptions

* An endpoint GUID is never announced again once withdrawn (§8.2.4.3).
* A datagram's delay is shorter than `RTPS_PEER_SETTLE_MS`: nothing sent
  before a peer settled is still in flight when it is parked. Reordering is
  otherwise unrestricted.
* Parking is local. The peer keeps this side's SEDP reader matched and drops
  a disposal only once this side has acknowledged it.
* ACKNACKs are not modelled: the announcer may send any DATA it holds, any
  GAP and any HEARTBEAT at any time, which covers every response an ACKNACK
  could request.
* Fairness: the reader's tick runs, and a message the announcer keeps
  offering eventually arrives (`Transmit`).

## Properties

| Property | Kind | Meaning |
|---|---|---|
| `NoListAfterDisposal` | invariant | An endpoint whose disposal this side delivered is never listed again. |
| `UnconfirmedOnlyInRefresh` | invariant | A refresh, once started, is not abandoned. |
| `NoLiveDrop` | action | A listed endpoint leaves the listing only when the peer has withdrawn it. |
| `EventuallyConsistent` | liveness | Once the peer is quiet, the listing settles on exactly its endpoints. |

## Phase 1 results

Bounds: two endpoints, `MaxSN = 3`, two messages in flight. `SweepRule` is
`caught_up` (the code) unless stated.

| Delivery | ParkRule | NoListAfterDisposal | UnconfirmedOnlyInRefresh | EventuallyConsistent |
|---|---|---|---|---|
| on_arrival (code) | code | violated | violated | violated |
| on_arrival (code) | keep_refresh_target | violated | holds | violated |
| in_order (§8.4.10.4) | code | holds | violated | violated |
| in_order (§8.4.10.4) | keep_refresh_target | holds | holds | **holds** |

`NoLiveDrop` is violated under both sweep rules, `caught_up` and
`post_activation_hb`. What each violation shows:

* **Delivery on arrival re-lists a withdrawn endpoint.** A reannouncement or
  repair DATA overtaken by the disposal that supersedes it is still
  unresolved when it arrives, so `StatefulReader` delivers it after the
  disposal (`stateful_reader.cpp`, `is_new_change()` then `deliver()`). The
  endpoint stays listed for as long as the peer lives. Delivering only up to
  `available_changes_max`, as §8.4.10.4 describes, removes it.
* **Parking during a refresh abandons it.** An owed reactivation whose
  replay is lost leaves a fresh proxy that is trivially caught up; the peer
  settles, and `park_peer()` keeps `available_changes_max`, 0, as the parked
  number. The next activation then starts no refresh, and an endpoint the
  peer withdrew while parked stays listed. Keeping the larger of the refresh
  target and `available_changes_max` resumes the refresh.
* **No sweep rule prevents a transient drop.** After reactivation, a GAP
  for an old announcement can arrive, and the reader be caught up to every
  number it knows of, before the DATA of that endpoint's newer announcement.
  A HEARTBEAT received since reactivation (the rule proposed on PR #8) does
  not close it: a HEARTBEAT sent before the peer's latest announcement can
  arrive after reactivation within one settle period. With both fixes above,
  the listing converges (`EventuallyConsistent`); the drop is a flicker that
  the newer DATA repairs.

## Phase 2 results

`RosettePeerTable` with three peers (four in the larger runs), one to three
slots, a record for every peer, and four or five environment events (joins,
leaves, announcements, local endpoint changes) per behavior.

* **The invariants hold:** at most `RTPS_MAX_PEERS` peers are active, the
  oldest-first order covers exactly the active peers, a parked peer is never
  matched, and a matched peer is never parked to make room.
* **Liveness holds when the board has room.** Once the network is quiet and
  fewer live peers share a match than there are slots, every peer is
  eventually exchanged with and every shared match is made, including the
  owed peers that `serve_owed_peer()` serves in table order rather than
  oldest first. More matching peers than slots is the over-capacity case the
  design accepts.
* **A peer that cannot complete its exchange holds its slot for good.** A
  peer whose SPDP multicast arrives but whose SEDP exchange never completes
  (unicast filtered, an unreachable announced locator) never acknowledges
  this side's announcements, so it never settles and is never parked; its
  SPDP keeps its lease alive. It costs a slot exactly as a matched peer
  does: with matched and unreachable peers filling every slot, a reachable
  peer stays pending forever (`run_table.sh EveryPeerServed 3 2 3 '{1}' 4`).

## Phases

The two-tier peer table is modelled in phases, each adding one part of the
design to the same modules:

1. **One peer, inbound SEDP.** Settle, park, promotion by DATA or HEARTBEAT
   above the kept number or by an owed reactivation, and the graph cache's
   refresh.
2. **The table.** Several peers and fewer active slots: admission, parking on
   demand (the settled peer active longest without an exchange first),
   over-capacity promotion retries, lease expiry and disposal in both tiers.
   Properties: no lockout, a peer with a real match is never parked, at most
   `RTPS_MAX_PEERS` active.
3. **Outbound.** Owed peers caught up through borrowed reader slots, oldest
   first, and `ros_discovery_info`. Properties: every known peer eventually
   acknowledges this side's latest announcements; nothing is acknowledged
   before it is processed.
4. **Quiescence.** HEARTBEAT and ACKNACK response rules, to check that a
   settled network stops exchanging SEDP (the HEARTBEAT storm of PR #41).
5. **Refinement.** A mapping from `RosettePeerTiers` to `SedpReliable`: every
   behavior of Rosette's reader is one the specification allows.

The design's timing questions (a high-priority loop starving the watchdog)
are outside TLA+: they need schedulability analysis or the bench.
