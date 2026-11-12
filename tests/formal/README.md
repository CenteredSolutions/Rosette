# Formal models

TLA+ models of Rosette's discovery protocol, checked with TLC. They are
layered so that each Rosette choice is checked against every behavior the
DDSI-RTPS 2.5 specification allows a peer, not against one vendor's:

| Module | Holds |
|---|---|
| `SedpReliable.tla` | What the specification requires and permits: a peer's SEDP announcer, a network that loses, duplicates and reorders, and the WriterProxy bookkeeping of §8.4.10.4. No Rosette choice. |
| `RosettePeerTiers.tla` | Rosette's choices on top: the two-tier peer table (`docs/design/two_tier_peer_table.md`) and the graph cache's refresh of a reactivated peer. |
| `MCRosettePeerTiers.tla` | The model-checking harness (endpoint symmetry). |
| `RosettePeerOutbound.tla` | The outbound half for one peer: whether a parked peer owed a new `ros_discovery_info` sample receives it, across lossy reactivation. |
| `RosettePeerTable.tla` | The table as a whole: several peers competing for fewer active slots, with each peer's SEDP exchange abstracted to one step. |

Where Rosette has a choice to make, the module carries it as a constant, with
the code's choice and its alternatives side by side, so a check shows what
each choice costs.

## Running

```sh
tests/formal/run_tlc.sh <delivery> <sweep_rule> <park_rule> <settle_rule> <property> [max_sn] [net_cap]
tests/formal/run_tlc.sh in_order caught_up keep_refresh_target heard_writer EventuallyConsistent
```

```sh
tests/formal/run_table.sh <property> [peers] [slots] [records] [unreachable] [budget] [stall_rule] [serve_order]
tests/formal/run_table.sh EveryPeerServed 4 2 4 '{1,2}' 5 deadline oldest
```

```sh
tests/formal/run_outbound.sh <settle_rule> <acknack_retry> <property> [versions] [announcements]
tests/formal/run_outbound.sh heard_writer TRUE EventuallyInformed
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
* A reactivated peer answers. Phases 1 and 2 let its HEARTBEAT reach the
  fresh proxy eventually; phase 3 checks that resending the preemptive
  ACKNACK (`RTPS_PREEMPTIVE_ACKNACK_RETRY_MS`) is what makes it answer.

## Properties

| Property | Kind | Meaning |
|---|---|---|
| `NoListAfterDisposal` | invariant | An endpoint whose disposal this side delivered is never listed again. |
| `UnconfirmedOnlyInRefresh` | invariant | A refresh, once started, is not abandoned. |
| `NoLiveDrop` | action | A listed endpoint leaves the listing only when the peer has withdrawn it. |
| `EventuallyConsistent` | liveness | Once the peer is quiet, the listing settles on exactly its endpoints. |

## Phase 1 results

Bounds: two endpoints, `MaxSN = 3`, two messages in flight (the
`run_tlc.sh` defaults). `SweepRule` is `caught_up` (the code) unless stated.

| SettleRule | Delivery | ParkRule | NoListAfterDisposal | UnconfirmedOnlyInRefresh | EventuallyConsistent |
|---|---|---|---|---|---|
| caught_up | on_arrival (code) | resolved_only | violated | violated | violated |
| caught_up | on_arrival (code) | keep_refresh_target (code) | violated | holds | violated |
| caught_up | in_order (§8.4.10.4) | resolved_only | holds | violated | violated |
| caught_up | in_order (§8.4.10.4) | keep_refresh_target (code) | holds | holds | holds |
| heard_writer (code) | on_arrival (code) | resolved_only | violated | holds | violated |
| heard_writer (code) | on_arrival (code) | keep_refresh_target (code) | violated | holds | violated |
| heard_writer (code) | in_order (§8.4.10.4) | resolved_only | holds | holds | violated |
| heard_writer (code) | in_order (§8.4.10.4) | keep_refresh_target (code) | holds | holds | **holds** |

`NoLiveDrop` is violated under both sweep rules, `caught_up` and
`post_activation_hb`, with either settle rule. What each violation shows:

* **Delivery on arrival re-lists a withdrawn endpoint.** A reannouncement or
  repair DATA overtaken by the disposal that supersedes it is still
  unresolved when it arrives, so `StatefulReader` delivers it after the
  disposal (`stateful_reader.cpp`, `is_new_change()` then `deliver()`). The
  endpoint stays listed for as long as the peer lives. Delivering only up to
  `available_changes_max`, as §8.4.10.4 describes, removes it.
* **Parking during a refresh must not abandon it, or lower its target.**
  Under `caught_up`, an owed reactivation whose replay is lost leaves a fresh
  proxy that is trivially caught up; the peer settles, and keeping
  `available_changes_max`, 0, as the parked number (`resolved_only`) means
  the next activation starts no refresh, and an endpoint the peer withdrew
  while parked stays listed. `heard_writer` forbids that park, but being
  heard is not reaching the target: a proxy that has heard only a GAP for an
  old number is caught up to it, so `resolved_only` lowers the target to that
  number. The next owed reactivation then sweeps once the same GAP arrives,
  before the replay reaches the announcement that confirms a live endpoint,
  and the endpoint flickers out of the listing on every reactivation.
  `park_peer()` keeps the larger of the refresh target and
  `available_changes_max`, which resumes the refresh (TC-PART-8.5.3-018).
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
leaves, announcements, local endpoint changes) per behavior. A peer settles only
once its announcements are processed, which covers being heard since
activation, as `peer_is_settled()` requires.

* **The invariants hold:** at most `RTPS_MAX_PEERS` peers are active, the
  oldest-first order covers exactly the active peers, a parked peer is never
  matched, and a matched peer is never parked to make room.
* **Liveness holds when the board has room.** Once the network is quiet and
  fewer live peers share a match than there are slots, every peer is
  eventually exchanged with and every shared match is made. More matching
  peers than slots is the over-capacity case the design accepts.
* **A peer that cannot complete its exchange must not hold its slot for
  good.** A peer whose SPDP multicast arrives but whose SEDP exchange never
  completes (unicast filtered, an unreachable announced locator) never
  acknowledges this side's announcements, so it never settles; its SPDP
  keeps its lease alive. Without a limit (`StallRule = "none"`) it costs a
  slot exactly as a matched peer does, and with matched and unreachable
  peers filling every slot a reachable peer stays pending forever.
  `peer_is_stalled()` (`"deadline"`) lets a peer with no match whose exchange
  has not completed within `RTPS_PEER_EXCHANGE_TIMEOUT_MS` be parked like a
  settled one, but owed, so it is tried again.
* **Owed peers must be served oldest first.** With the deadline alone, two
  unreachable peers can take turns in one slot forever: each is parked owed,
  and `serve_owed_peer()` in table order (`ServeOrder = "table"`) always
  serves the lower-numbered one, starving a reachable owed peer behind them.
  Serving the owed peer whose last SEDP DATA or activation is oldest
  (`"oldest"`, which the design document specifies) lets every owed peer
  reach the front.

| StallRule | ServeOrder | Unreachable | EveryPeerServed | EveryMatchMade |
|---|---|---|---|---|
| none | table | {1} of 3 | violated | holds |
| none | oldest | {1} of 3 | violated | holds |
| deadline | table | {1, 2} of 4 | violated | holds |
| deadline | oldest (code) | {1} of 3, {1, 2} of 4 | **holds** | **holds** |

  The invariants and `NeverParkMatched` hold under every combination.

## Phase 3 results

`RosettePeerOutbound` with one peer, three discovery samples and two peer
announcements per behavior.

| SettleRule | AcknackRetry | NoParkUninformed | EventuallyInformed | EventuallyParkable |
|---|---|---|---|---|
| caught_up | FALSE | violated | violated | holds |
| caught_up | TRUE | violated | violated | holds |
| heard_writer | FALSE | holds | violated | violated |
| heard_writer (code) | TRUE (code) | holds | **holds** | **holds** |

* **A peer reactivated to receive a new `ros_discovery_info` sample can be
  parked again without it.** Reactivation rebuilds the SEDP reader proxies,
  which send one preemptive ACKNACK. If it, or the HEARTBEAT answering it,
  is lost, the peer sends nothing more: its writers' proxies for this side
  still hold everything acknowledged before parking, and the reader sends
  ACKNACKs only in answer to a HEARTBEAT. The replay never starts, so the
  discovery writer never matches the peer's reader again, and
  `StatefulWriter::acknowledged_by_participant()` holds for a peer it has no
  proxy for. The peer settles once it acknowledges this side's SEDP, is
  parked, and its owed flag is cleared: it keeps the old sample until the
  next change. Reproduced against the real `RTPSParticipant` by a probe test
  built on TC-PART-8.5.3-013.
* **Requiring the writer to be heard alone trades the lost update for a
  stuck slot.** A peer whose reader proxies have heard nothing since
  activation is then never settled, and with one lost ACKNACK nothing is
  ever heard: it holds its slot for good, as an unreachable peer does in
  phase 2.
* **Both together hold:** settle only once each SEDP reader proxy has heard
  its writer since activation (`SedpReader::heard_from()`), and resend the
  preemptive ACKNACK until it has (`RTPS_PREEMPTIVE_ACKNACK_RETRY_MS`). The
  peer then always receives the current sample and can always be parked
  again (TC-PART-8.5.3-019). A peer that cannot be reached at all is never
  heard; the phase 2 deadline is what frees its slot.

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
3. **Outbound.** One owed peer and this side's `ros_discovery_info` sample
   across a lossy reactivation. Properties: a peer is never parked without
   the current sample, eventually receives it, and can always be parked
   again.
4. **Quiescence.** HEARTBEAT and ACKNACK response rules, to check that a
   settled network stops exchanging SEDP (the HEARTBEAT storm of PR #41).
5. **Refinement.** A mapping from `RosettePeerTiers` to `SedpReliable`: every
   behavior of Rosette's reader is one the specification allows.

The design's timing questions (a high-priority loop starving the watchdog)
are outside TLA+: they need schedulability analysis or the bench.
