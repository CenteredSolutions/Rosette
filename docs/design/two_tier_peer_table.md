# Two-tier peer table

Scope: how an `rmw_rosette` participant stays reachable by every peer on a
network with more participants than it can afford full discovery state for.
The motivating network is a machine of 70 to 90 participants: a few dozen
controller boards, each running an example firmware based on RP2350, plus
camera and ML nodes, control nodes, a few orchestrators, and a handheld
operator console. Each board matches only a handful of them.

## The problem

`SpdpReader` keeps up to `RTPS_MAX_PEERS` participants, first come, first
served. A slot frees only when its peer's lease expires or the peer disposes
itself (DDSI-RTPS 2.5 §8.5.3.2). A participant that arrives at a full table is
dropped on every announcement, so neither side ever exchanges SEDP with the
other: an operator console that starts after the table filled waits in
`wait_for_service()` forever, although the board would match its client.
Meanwhile peers the board matches nothing with hold their slots indefinitely.

`tests/ros2/run_rmw_peer_lockout_loopback.sh` reproduces it: a board built
with two slots, two filler participants that share nothing with it, and a late
participant that calls the board's service. It fails on every run with the
filler slots taken, and passes with a free slot.

Raising `RTPS_MAX_PEERS` to the network size is the alternative this design
avoids; the RAM budget below shows why.

## The RAM budget

Measured on an example firmware based on RP2040 (264 KB of SRAM, 256 KB of it
in the main banks), built with 8 peer slots, 8 matches per local endpoint, 18
local writers and 13 local readers, before the locator savings of #9 and the
origin change that follows it. `.data`, `.bss` and the heap and stack
reserves take 199 KB of the 256 KB, which leaves about 57 KB free.

The participant itself is 84.5 KB, and most of it does not depend on the
network size at all:

| Part | Octets | Scales with |
|---|---|---|
| Writer slots, 18 × 1,680 (each holds 8 reader proxies) | 30,240 | local writers × matches per endpoint |
| Reader slots, 13 × 1,600 (each holds 8 writer proxies) | 20,800 | local readers × matches per endpoint |
| Local endpoint names in the match table | 10,656 | local endpoints |
| Match records, 248 × 32 | 7,936 | local endpoints × matches per endpoint |
| Remote endpoint records, 159 × 16 | 2,544 | configured, not peers |
| Per-peer discovery state, 8 × 528 | 4,224 | **peers** |
| SEDP announcement storage and the rest | about 8,000 | local endpoints |

So the network size reaches RAM through one line: 528 B per peer slot
(peer record 176, SEDP proxies 2 × 80 + 2 × 64, match-table origin 64). The
endpoint lines are already sized by what the board talks to, not by who else
is on the network.

| Peers the board can hold | Per-peer RAM | Free of the 57 KB |
|---|---|---|
| 8 full slots (today) | 4.2 KB | 57 KB |
| 90 full slots, today's sizes | 47.5 KB | about 14 KB |
| 90 full slots, after #9 and step 3 (about 430 B each, estimated) | 38.7 KB | about 22 KB |
| 90 known-peer records + 12 active slots (this design) | 6.1 + 4.8 = 10.9 KB | about 50 KB |

Ninety full slots would technically fit, but leave too little headroom for
the application and for a board that matches a few more endpoints than
planned. Two tiers keep the per-peer cost at the size of a small record, and
spend the full 400 B only on the peers the board actually exchanges data with.

A separate lever, not part of this design: matches per local endpoint. Every
writer slot reserves 8 reader proxies even when only one or two peers read
it. Lowering that limit to 4 would save about 13 KB on this firmware.

## Goals

* Every participant on the network is admitted and stays discoverable: a peer
  that starts late, or adds an endpoint late, matches the board as quickly as
  one that started first.
* Every participant sees the board's nodes (`ros2 node list`), including the
  updates published on `ros_discovery_info` when the board's entities change.
* RAM grows with the number of peers the board actually talks to, plus a
  small fixed record per known peer.
* Peers see a conforming RTPS participant. Nothing here needs cooperation
  from, or a configuration change on, a stock Fast DDS or Cyclone DDS peer.
* A host build with room for every peer behaves exactly as it does today.

Not a goal: making a board match more endpoints at once than its
`RTPS_MAX_MATCHED_READERS` and match-table limits allow. A board that really
talks to 40 peers needs 40 active slots.

## Two tiers

Every known participant has a **known-peer record**. A peer that discovery is
working with also holds an **active slot**.

| | Known-peer record (every peer) | Active slot (some peers) |
|---|---|---|
| Identity and liveness | GUID prefix, lease expiry | |
| Addressing | one metatraffic unicast locator (24 B, IPv4 or IPv6) and the default unicast port | the ranked locator lists (`DiscoveredParticipant`) |
| Their SEDP announcers → our SEDP readers | highest sequence number processed, per announcer | a `WriterProxy` per announcer |
| Our SEDP announcers → their SEDP readers | highest sequence number acknowledged, per announcer | a `ReaderProxy` per announcer |
| Our `ros_discovery_info` writer → their reader | reader entity ID, highest sequence number acknowledged | a `ReaderProxy` in the writer |
| Vendor ID, builtin endpoint set | builtin set as three flag bits | full values |

The active slot is everything today's `PeerEntry` and the per-peer SEDP
proxies hold. The known-peer record is what is left once both directions of
the exchange have settled: enough to recognise the peer, keep its lease, and
resume either direction exactly where it stopped.

A peer whose metatraffic and default unicast addresses differ (only the ports
may differ) keeps its active slot and is never parked. Every stack Rosette
interoperates with announces one address for both, so this costs nothing in
practice and keeps the record at one locator.

### Parked endpoints

The record does not keep proxies. It keeps **parked** ones: the sequence
number state of one reliable endpoint pair, from which the proxy is rebuilt
when it is needed again. One mechanism serves all three pairs in the table:

* **Their SEDP announcer, our SEDP reader.** The record keeps the highest
  sequence number this side processed (its last `WriterProxy` had no gaps up
  to it, and the peer's last HEARTBEAT announced nothing above it). While
  parked, a HEARTBEAT from that announcer is compared with it:
  * `lastSN` not above it: nothing changed. A HEARTBEAT without the FinalFlag
    is answered from the record with an ACKNACK whose base is the stored
    number plus one and whose set is empty, as the reliable reader rules
    require (§8.4.2.3); nothing else happens.
  * `lastSN` above it: the peer announced or removed an endpoint. The peer is
    promoted, and the new `WriterProxy` starts with everything up to the
    stored number marked received, so its ACKNACK asks only for the
    announcements this side never processed.
* **Our SEDP announcer, their SEDP reader.** The record keeps the highest
  sequence number the peer acknowledged. While parked, the writer sends that
  reader nothing: it has acknowledged everything, and a reliable writer sends
  HEARTBEATs only while a reader has unacknowledged changes (the
  StatefulWriter behaviour of §8.4.9.2). When the announcer writes a new
  change, the parked reader is owed it and is caught up (below).
* **Our `ros_discovery_info` writer, their reader.** Identical to the SEDP
  writer case, with the reader's entity ID stored as well, since stock
  participants create that reader with an ordinary entity ID. This is the
  rule settled on 2026-10-04: once a reader has acknowledged the current
  sample, its `ReaderProxy` is released and the acknowledged number is kept,
  so a later change to the board's nodes reaches it without the peer staying
  active. Because the parked number is the one the reader acknowledged (taken
  from its ACKNACK, not from a HEARTBEAT this side sent), a resumed proxy can
  never skip a sample the reader has not received.

Resuming from the stored numbers, instead of starting the proxies over, keeps
both peers' views consistent. A rebuilt `WriterProxy` that started from zero
would send ACKNACK base 0 or 1; Fast DDS 2.6 then rewinds and resends the
whole SEDP history (`ReaderProxy::acked_changes_set` for non-volatile
writers), which works but costs a full replay per promotion, and other
vendors are not known to do the same. A rebuilt `ReaderProxy` that started
from zero would replay the history to a reader that already has it. Fast DDS
discards those as duplicates, but the traffic is wasted.

#### Counts survive parking

HEARTBEAT and ACKNACK counts must keep increasing per writer-reader pair, and
a receiver ignores a count it has already seen (§8.3.7.1, §8.3.7.5).
HEARTBEAT counts already belong to the writer (`StatefulWriter`), so rebuilt
`ReaderProxy` objects inherit them. ACKNACK counts belong to the
`WriterProxy`, so a rebuilt one would start again at 1 and the peer would
ignore its ACKNACKs. The participant therefore keeps one ACKNACK count
high-water mark: parking a `WriterProxy` raises it to that proxy's count, and
a rebuilt `WriterProxy` starts above it. One 32-bit value for the
participant, nothing per peer.

### Catching up parked readers

When one of this participant's announcers (SEDP publications, SEDP
subscriptions, `ros_discovery_info`) writes a change, every parked reader of
it is owed that change. The writer catches them up through its own matched
reader slots, a few at a time:

1. Take an owed parked reader and an idle matched-reader slot.
2. Rebuild a `ReaderProxy` in the slot, with everything up to the stored
   number acknowledged. The normal reliable machinery sends the new changes
   and a HEARTBEAT, and repairs from the reader's ACKNACKs.
3. Once the reader acknowledges the writer's last change, store the new
   number, release the slot, and take the next owed reader.

The peer itself is not promoted: only one proxy is borrowed, for one round
trip. A writer's matched-reader slots are therefore a working pool, not a
limit on how many peers can read it. This also removes a limit that exists
today: the `ros_discovery_info` writer has `RTPS_MAX_MATCHED_READERS` reader
slots (8 in the firmware profile, 16 on a host), and `matched_reader_add()`
refuses a reader when they are full, so on a network of 70 participants all
but the first few never receive the board's node list. The section below
covers what this writer costs at that scale.

Catch-up is ordered oldest owed first, so a burst of changes (a node created
at boot, then its endpoints) reaches every peer in a bounded time instead of
starving the last ones.

### `ros_discovery_info` at 90 peers

Every stock ROS 2 participant reads `ros_discovery_info`, so this one writer
has a reader in every peer. Raising `RTPS_MAX_MATCHED_READERS` to the network
size is not the way to serve them: every writer slot in the participant is
sized for the largest writer (`RTPS_PARTICIPANT_MAX_WRITER_BYTES`), so the
`ReaderProxy` array of every writer would grow, not just this one. The match
table would also need a match record and a remote reader record for every
peer's reader, and since those readers match a local writer, the table's
displacement rule would protect them ahead of application endpoints.

Parked, a peer's reader costs 8 B in its known-peer record (entity ID and
acknowledged number); its match record and remote reader record are released
with the proxy. The writer keeps its few reader slots as the catch-up pool.

The cost that does grow with the network is traffic. The writer keeps one
sample (KEEP_LAST 1) holding the whole `ParticipantEntitiesInfo`, about 24 B
of GID per endpoint plus the node names, and sends it unicast to each reader.
A board that declares its entities before the participant first runs
publishes one sample before discovery and almost never changes it, so the
traffic is one send per peer as it is discovered. Two cases repeat it to
every peer:

* **A change to the board's nodes or endpoints.** Rare on a board, and paced
  by the catch-up pool.
* **A peer that loses the board's lease.** The rematch republish
  (`RosDiscoveryInfoWriter::on_reader_rematched()`) writes the same content
  under a new sequence number, because Fast DDS discards a replay of a number
  it delivered before; every other reader is then owed it too. Readers that
  already hold the content can be sent a GAP for the new number instead of
  the DATA, since a writer sends GAP for a change irrelevant to a reader
  (§8.3.7.4, §8.4.9.2); that keeps one peer's lease loss from costing a
  full send to every other peer.

Reading `ros_discovery_info` is the expensive direction at this scale: each
peer's message lists all of its nodes and endpoints, and keeping them is a
whole graph cache. Boards do not read it; a firmware that needs other
participants' node names is a separate design.

## Lifecycle

### Admission

An SPDP announcement from an unknown participant creates a known-peer record.
The record table is sized for the whole network (`RTPS_MAX_KNOWN_PEERS`); at
about 68 B a record (estimate below), 96 records cost about 6.5 KB. A full
record table is the only remaining lockout, and it is logged and counted as
it is today.

A new peer needs an active slot for its first exchange, since nothing about
its endpoints is known yet. If a slot is free it gets one at once. If not, a
settled active peer is parked to make room (see "Making room"). If no active
peer can be parked, the new peer waits in the record table, and it is
admitted on its next SPDP announcement after a slot frees. While it waits,
its SEDP traffic is dropped as it is today for an unmatched writer, and this
side sends it no SEDP. It is not locked out, only delayed.

### Settled

An active peer is **settled** when all of these hold:

* It completes no match with any local application endpoint. The
  `ros_discovery_info` writer is not an application endpoint: every stock
  ROS 2 participant reads it, so counting it would leave no peer settled.
* Every one of its SEDP announcers that this side reads has been processed,
  with no gaps, up to the `lastSN` of its latest HEARTBEAT.
* Its SEDP readers have acknowledged everything both of this side's SEDP
  announcers have written.
* Its `ros_discovery_info` reader, if it has one, has acknowledged the
  current sample.
* It has been settled for at least `RTPS_PEER_SETTLE_MS`, so a peer in the
  middle of creating its endpoints (rclcpp creates them over tens of
  milliseconds) is not parked between two of them.

### Making room (parking)

Peers are parked **on demand**, not as soon as they settle. A slot is freed
only when a new peer or a promotion needs one and none is free. The settled
peer that has been active the longest without an exchange is parked first.

Parking on demand means a host build, or a board on a small network, never
parks anyone and behaves exactly as today. On the motivating network, the first
boot wave parks peers about as fast as they settle.

Parking a peer:

1. Stores the parked numbers of its three endpoint pairs in its record, and
   raises the participant's ACKNACK count high-water mark.
2. Removes its SEDP proxies and its `ros_discovery_info` proxy.
3. Keeps its remote endpoint records in `EndpointMatchTable`. They match no
   local endpoint (that is what settled means), so they are already the first
   ones the table displaces when it fills; while they stay, they let a local
   endpoint created later find the peer.
4. Releases the active slot.

### Promotion

A parked peer is promoted (given an active slot, its proxies rebuilt from the
record) when:

* **It announces something new:** a HEARTBEAT or DATA from one of its SEDP
  announcers carries a sequence number above the parked one. This covers the
  operator console subscribing to a board's topic long after both started.
* **A local endpoint matches one of its stored remote endpoints.** This is
  the case where this side creates an endpoint late. On a board it does not
  happen: firmware declares every entity before the participant first runs
  (`participant_driver.hpp`), so the local endpoint set is fixed before
  discovery. On a host it can.

Promotion needs a free or freeable slot. If every active peer has a real
match, the board is over capacity: the promotion is counted and logged, and
retried when the peer's next HEARTBEAT arrives. A peer with a real match is
never parked to make room.

A promoted peer that turns out to match nothing (it announced an endpoint
nobody here uses) settles again and is parked again on the next demand.

### Lease and disposal

The lease stays in the record, so it is kept in both tiers exactly as today.
When it expires or the peer disposes itself, the record, any active slot, and
the peer's remote endpoint records go, and the matches dissolve through
`on_spdp_peer_removed()` as they do now. A participant that comes back after
that is a new participant, and the `ros_discovery_info` rematch republish
(`RosDiscoveryInfoWriter::on_reader_rematched()`) still applies to it.

## Latency for matched peers

The participant runs in the same loop as the board's matched traffic, so
every datagram discovery processes delays that traffic by the time it takes.
Measured on the same board: reading a datagram over SPI takes about 0.3 ms,
and decoding an SEDP sample 0.4 to 1.8 ms. With 90 participants announcing
every 3 s, about 30 SPDP datagrams a second arrive by multicast, whatever the
board does with them. At about 1.5 ms each, that is 4 to 5 % of the CPU in
stalls of up to 1.5 ms at a time, and a whole burst at once when the machine
powers up.

Four rules keep discovery out of the way of matched traffic:

1. **Matched traffic first.** Discovery and user traffic arrive on separate
   sockets (the SPDP multicast port, the metatraffic unicast port, the user
   unicast port). The participant drains the user socket first, and handles
   a bounded number of discovery datagrams per pass. Discovery is built to
   lose datagrams: SPDP repeats about six times per lease, and SEDP is
   reliable. User traffic is not built to wait.
2. **A known peer's SPDP costs a lookup, not a decode.** The sender's GUID
   prefix is in the RTPS header. If it names a known peer and the payload has
   the same length as last time, the lease is refreshed and the rest of the
   datagram is skipped without reading it over SPI. The full announcement is
   decoded again at most once per lease, which catches a changed address
   within one lease period. Of 30 SPDP datagrams a second, almost all take
   this path.
3. **A parked peer costs nothing else.** Beyond its SPDP, it only sends an
   occasional SEDP HEARTBEAT, answered from its record without a decode.
4. **The active tier bounds the boot burst.** Only peers with an active slot
   exchange SEDP, so at most `RTPS_MAX_PEERS` peers' endpoint announcements
   are in flight at once, instead of every participant on the machine
   announcing its endpoints to the board in the same second.

## What peers see

From any peer's point of view the board is an ordinary participant:

* It announces SPDP on the usual period and never lets a lease lapse.
* It answers every HEARTBEAT that requires an answer.
* It sends HEARTBEATs to a reader only while that reader has
  unacknowledged changes.
* It never acknowledges a change it has not processed: a parked
  `WriterProxy` is stored with gaps resolved, and resumes from there.

Parking and promotion are, on the wire, `matched_reader_remove` followed by
`matched_reader_add` (§8.4.1.4) and the reader-side equivalents (§8.4.7.4),
with the proxy state remembered across the gap instead of reset. The spec
allows both operations at any time, and the remembered state only saves
retransmissions.

## Known-peer record

All figures are estimates for Cortex-M0+; the ARM `sizeof` probe confirms
them before and after the change.

Known-peer record, about 68 B:

| Field | Octets |
|---|---|
| GUID prefix | 12 |
| Lease expiry | 4 |
| Metatraffic unicast locator | 24 |
| Default unicast port | 2 |
| Builtin set flags, tier, owed bits | 2 |
| Their two SEDP announcers: processed sequence numbers | 2 × 4 |
| Our two SEDP announcers: acknowledged sequence numbers | 2 × 4 |
| `ros_discovery_info` reader: entity ID and acknowledged sequence number | 4 + 4 |

Sequence numbers are kept as their low 32 bits. Neither side's discovery
writers come anywhere near 2^32 changes; a peer whose numbers do not fit is
simply never parked.

Active slot: the remainder of a full peer, about 400 B (the ranked locator
lists and four SEDP proxies). A board needs as many active slots as the peers
it really matches, plus a few for peers in their first exchange.

## Configuration

* `RTPS_MAX_KNOWN_PEERS`: known-peer records; size it for the whole network.
* `RTPS_MAX_PEERS`: active slots; unchanged meaning (full discovery state).
  `RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS` keeps setting it, and a new
  `RMW_ROSETTE_MAX_KNOWN_PARTICIPANTS` sets the record table. A host default of
  equal values reproduces today's behaviour.
* `RTPS_PEER_SETTLE_MS`: how long a peer must stay settled before it can be
  parked; one SPDP period by default.

## Verification

Acceptance: `tests/ros2/run_rmw_peer_lockout_loopback.sh` passes with the
two-active-slot build and two fillers, and with six fillers when the record
table holds eight peers. It fails on `main` today.

Spec tests, IDs assigned when written:

* SPDP (`TC-SPDP-8.5.3-*`): a new peer at a full active tier is admitted by
  parking a settled peer; a peer with a real match is never parked; with no
  settled peer, the new peer waits and is admitted after a slot frees; lease
  expiry removes a parked peer and its endpoint records.
* SEDP reader (`TC-SEDP-8.5.5.1-*`, `TC-SRR-8.4.7.4-*`): a parked announcer's
  HEARTBEAT without new numbers is answered from the record and promotes
  nothing; one with a higher `lastSN` promotes the peer, and the first
  ACKNACK requests only the numbers above the parked one, with a count above
  every count sent before.
* SEDP and `ros_discovery_info` writers (`TC-SWR-8.4.1.4-*`): a change owed
  to parked readers reaches each of them through the slot pool; a parked
  reader is never sent a change it acknowledged; a reader that acknowledges
  less than its parked number is treated as started over, as it is today.
* Settling: a peer between two endpoint announcements is not parked; a peer
  with unacknowledged SEDP is not parked.

Interop: a stock participant parked on the board lists the board's node with
`ros2 node list`, and lists a node the board creates after the peer was
parked.

## Measure before building

1. **A network at scale, on one host.** `rclpy` can open many contexts in
   one process, and each is a stock participant, so 90 participants with the
   motivating network's topics fit on a workstation. That scales the lockout
   test to the target size and gives the active-peer count per board, with
   the network's topology as an input rather than a measurement.
2. **Per-peer cost after #9 and step 3.** Re-run the ARM `sizeof` probe; the
   430 B figure above is arithmetic.
5. **Discovery load on the board.** With the board on the LAN and 90
   simulated participants on a workstation, the time the board spends per
   SPDP datagram (decoded and skipped), and the round trip of a service call
   from a matched peer with and without that load.
3. **Fast DDS 2.6 SEDP HEARTBEATs to a fully acknowledged reader.** Whether
   they keep coming, how often, and whether they carry the FinalFlag decides
   how much traffic parked peers still cost.
4. **The `ros_discovery_info` reader limit.** Confirm with 20 stock peers on
   the default host build that only 16 of them list the board's node, and
   measure the size of the board's `ParticipantEntitiesInfo` sample.

## Decisions

* **Parking on demand, not eagerly** (recommended above). Eager parking would
  save no RAM, since slots are allocated statically, and would add churn on
  small networks.
* **Stored remote endpoint records stay displaceable.** A late local endpoint
  finds a parked peer only while its records survive; once the table has
  displaced them, Fast DDS can resend them on an ACKNACK with base 0, and that
  path is built only if a host use case needs it.
* **Active-tier priority.** Peers with a real match are never parked. The
  open question is whether some peers (the operator console, orchestrators) should
  also be pinned active by configuration so that their first exchange never
  waits for a slot.

## References

* OMG DDSI-RTPS 2.5 §8.3.7.1 (ACKNACK count), §8.3.7.5 (HEARTBEAT count and
  FinalFlag), §8.4.1.4 (`matched_reader_add` / `matched_reader_remove`),
  §8.4.2.3 (required reader behaviour), §8.4.7.4 (`matched_writer_add` /
  `matched_writer_remove`), §8.4.9.2 (reliable StatefulWriter behaviour),
  §8.5.3.2 (SPDP lease), §8.5.4.1 (SEDP builtin endpoints).
* Fast DDS 2.6 (Humble) `src/cpp/rtps/writer/ReaderProxy.cpp`,
  `acked_changes_set`.
* `rmw_dds_common` (Humble) `ParticipantEntitiesInfo` and `GraphCache`.
* Prior art for keeping a small record in place of full connection state:
  the Linux TCP `tcp_timewait_sock`, which replaces a closed socket with the
  few fields TIME_WAIT still needs.
