# Shared proxy pools

Scope: how a participant stores the per-match state of its local writers and
readers, so that a microcontroller can host the endpoints of a large ROS 2
graph, spend RAM only on the matches that exist, and still bound the work any
one match costs. The sizing example throughout is an RP2040 firmware with
18 local writers and 13 local readers, most of them services that
one client uses now and then, on a network of 70 to 90 participants.

## Per-endpoint proxy arrays

Without pools, every local endpoint carries the proxies for the most matches it could ever
have, `RTPS_MAX_MATCHED_READERS` reader proxies in each writer and
`RTPS_MAX_MATCHED_WRITERS` writer proxies in each reader (§8.4.1.1, §8.4.7.1).
`rmw_rosette` sets both from `RMW_ROSETTE_MAX_MATCHED_ENDPOINTS`, 8 on the
firmware:

| Storage | Count | Size | Octets |
|---|---:|---:|---:|
| `ReaderProxy`, 18 writers × 8 | 144 | 80 | 11,520 |
| `WriterProxy`, 13 readers × 8 | 104 | 64 | 6,656 |
| Fragment assembly slots, 13 readers × 8 | 104 | 128 | 13,312 |
| **Per-match storage** | | | **31,488** |

That is the worst case of every endpoint at once: all 18 writers each read by
8 remote readers. A firmware's endpoints are not used like that. A motion
service may have a few clients; a locate service or an I/O configuration
service has one client for a minute a day; a status publisher may have a
dozen subscribers. The per-endpoint limit is therefore wrong in both
directions: it reserves 7 idle proxies in each rarely used service, and it
refuses the 9th subscriber of the one topic every peer reads.

Per-reader fragment assembly is sized the same way, and the bound it reserves cannot be
reached: a fragmented sample is larger than one datagram, so it is reassembled
into one buffer-pool block of the datagram class or larger, and the firmware pool has 4 datagram blocks and 2
large blocks (`RMW_ROSETTE_LARGE_BUFFERS`), shared with every received datagram
and every unacknowledged large sample. At most 6 of the 104 assembly slots can
ever hold a buffer.

Per-endpoint arrays also make two refusals permanent. When a writer's
proxies are full, the reader is refused, but the match record stays in `EndpointMatchTable`, so the pair counts as
reported, but no proxy is ever bound to it: when another reader leaves, the
refused reader is still never served. When the match record table is full,
the pair is dropped and nothing re-derives it. A
remote endpoint is announced again only when it changes, so a pair refused at
either point stays unmatched until the remote endpoint changes or is
re-created.

## Goals

* Per-match storage scales with the matches the firmware has at once, which
  the firmware designer chooses as one total for writers and one for readers,
  not with endpoints × the most any one endpoint could have.
* Every local endpoint is guaranteed its first match (more, if configured),
  proven when the firmware compiles, never decided at run time.
* No endpoint can take the whole pool: a per-endpoint cap still holds.
* A match the pool cannot serve now is served when a proxy frees, oldest
  first, and is never lost.
* Every operation on the publish and receive path keeps its present bound.
  Binding a waiting match adds a bounded amount of work to `tick()`.
* Application code is stock `rclc`. Nothing here needs a per-endpoint setting
  that `rclc` cannot express, and the defaults reproduce the capacity of
  per-endpoint arrays exactly.
* One compiled `StatefulWriter` and one compiled `StatefulReader` serve every
  endpoint, builtin or user.

## Pools

The participant owns one pool of `ReaderProxy` for its user writers and one
pool of `WriterProxy` for its user readers. Both are arrays of fixed size,
sized at build time; a proxy never moves while it is bound, so the pointers
the rest of the stack holds into it (`FragmentRepair::proxy`, the locator
lists a proxy refers to) stay valid for as long as it is bound.

```
RTPSParticipant
├── ReaderProxyPool  reader_proxies_   RTPS_READER_PROXY_POOL × ReaderProxy
├── WriterProxyPool  writer_proxies_   RTPS_WRITER_PROXY_POOL × WriterProxy
├── writer slots     each a SharedPoolStatefulWriter<Depth>: history only
│     └── bound proxies: a list threaded through reader_proxies_
└── reader slots     each a SharedPoolStatefulReader: no proxy array
      └── bound proxies: a list threaded through writer_proxies_
```

A pool is the storage pattern the stack already uses
(`include/rosette/rtps/proxy_pool.hpp`). `ProxyPool` is a plain class whose
operations are compiled once in `src/rtps/proxy_pool.cpp`; it works on proxy
indices only. `TypedProxyPool<Proxy>` adds typed access, and
`ProxyPoolStorage<Proxy, N>` owns the N proxies and derives from it, as
`StatefulWriterStorage` owns a history.

The pool keeps one two-octet link per proxy. A bound proxy's link chains it
into its endpoint's list (`ProxyList`: head, count, floor, cap, kept by the
endpoint); a free proxy's link chains it into the pool's free list. A proxy is
on exactly one of those lists, so one link serves both: binding pops the free
list in O(1), and iterating an endpoint's matches walks only its own proxies,
k of them for k matches, never stepping over free ones. A release unlinks the
proxy from its endpoint's list, O(k), the same cost as the GUID lookup that
finds it.

User endpoints are `SharedPoolStatefulWriter<Depth>` (a history; proxies from
the participant's pool) and `SharedPoolStatefulReader`. The proxy count is not
part of their layout: on the example firmware a writer slot is 1,048 B
instead of 1,680 B with an array, and a reader slot 1,092 B instead of
1,600 B (Cortex-M0+, measured). `StatefulWriterStorage<Depth, MaxReaders>` and
`BoundedStatefulReader<N>` keep their meaning for an endpoint whose capacity
is its own: each owns a private pool of MaxReaders or N proxies, all promised
to it.

### Builtin endpoints keep their own pools

The SEDP writers and readers match exactly one endpoint per active peer
(`RTPS_MAX_PEERS`), and discovery must never wait for a proxy. Each builtin
endpoint gets a private `ProxyPoolStorage` of `RTPS_MAX_PEERS` proxies with its
floor equal to its cap, the same capacity as a per-endpoint array, through
the same compiled code.

The `ros_discovery_info` writer is a user writer in the participant's pool,
with floor and cap both `RTPS_MAX_PEERS`: every active peer reads it, and
`ros2 node list` on a peer depends on it. Its proxies are reserved like any
other floor.

## Who may take a proxy

Each pool keeps, for every endpoint it serves, the number of proxies bound
(`bound`), the endpoint's floor (`floor`) and cap (`cap`). The pool also keeps
how many free proxies are promised to floors not yet used:

```
reserved = Σ over endpoint slots of max(0, floor − bound)
```

An endpoint may bind a proxy when both hold:

```
bound < cap
bound < floor   or   free > reserved
```

Below its floor an endpoint always succeeds, because its unused floor is part
of `reserved` and so a free proxy was set aside for it. Above its floor it
borrows only from proxies nobody is promised. The invariant `free ≥ reserved`
holds after every bind and every release, and binding, releasing and the
check are O(1) on these counters.

Floors are reserved for every endpoint **slot**, created or not, so the
guarantee does not depend on the order endpoints are created in or on what
has been borrowed meanwhile. The participant checks it when it compiles:

```
static_assert(RTPS_READER_PROXY_POOL
              >= RTPS_MAX_USER_WRITERS × RTPS_MIN_MATCHES_PER_ENDPOINT
                 + (RTPS_MAX_PEERS − RTPS_MIN_MATCHES_PER_ENDPOINT));  // ros_discovery_info
static_assert(RTPS_WRITER_PROXY_POOL
              >= RTPS_MAX_USER_READERS × RTPS_MIN_MATCHES_PER_ENDPOINT);
```

A firmware sizes its slots to the endpoints it creates, so reserving for an
empty slot costs it nothing; a host that leaves many slots unused sizes the
pool larger.

The floor and the cap are uniform across user endpoints. A larger floor for
one endpoint (a motion service that must always serve three clients) needs a
way to name that endpoint that stock `rclc` does not have, and so does
pinning a peer active in the two-tier peer table. Neither is provided.

## Waiting matches

A match the pool refuses is not dropped. `EndpointMatchTable::MatchRecord`
carries a `waiting` flag and a `ticket`, the value of a 16-bit counter taken
when the pair was reported:

* `report()` records the pair and calls `on_match`, which binds a proxy for
  every local half of the pair and returns whether it could. Refused, the
  record stays with `waiting` set. The pair counts as matched in the table,
  so dissolving it later still pairs every report with an unmatch, and the
  two-tier table does not park a peer with which it shares one.
* Releasing a user endpoint's proxy, through an unmatch or `delete_writer()`
  / `delete_reader()`, marks a rebind due in the participant, and
  `ms_until_next_timer()` returns 0. The next `tick()` calls
  `match_oldest_waiting()`, which binds the waiting record with the oldest
  ticket (modulo-2^16 comparison) whose endpoints may now take a proxy; the
  participant answers that through `MayMatch`, the same admission test
  `bind()` applies. It binds at most one pair per tick, so a tick replays at
  most one TRANSIENT_LOCAL history; the rebind stays due while a tick binds
  one and clears on the first tick that binds none.
* A pair reported while an older waiting pair may bind is recorded waiting
  without trying, so a match discovered between the release and the next
  tick does not take the freed proxy ahead of the matches already waiting.
  Pairs bind in the order they were reported.
* At most `RTPS_MAX_WAITING_MATCHES` records wait at once. `MayMatch`
  tells `report()` before anything binds whether a pair would wait, and a
  pair that would wait beyond the limit is not recorded and binds no proxy.
  A topic that many peers read therefore cannot take the records that
  matches which bind at once need.
* Offering a waiting pair again binds only the half that waited: a half
  that already holds its proxy is left alone, which matters for a pair of
  two local endpoints.
* `matched_reader_count()` and `matched_writer_count()` count bound proxies
  only, so `rmw_publisher_count_matched_subscriptions`,
  `rmw_subscription_count_matched_publishers` and
  `rmw_service_server_is_available` report what the endpoint actually serves.

Every bound match holds at least one proxy, so a record table of one record
per proxy in both pools plus `RTPS_MAX_WAITING_MATCHES` always has a record
for a match that binds; `participant.hpp` `static_assert`s that size. A pair
turned away because the waiting matches are full, or because the record
table is, has no record. Nothing is lost by that: a pair is derived from two
stored endpoints that are compatible, and both endpoints stay in the table.
What is lost is the trigger, since SEDP announces an endpoint again only when
it changes. So `report()` sets a flag when it turns a pair away, and
dissolving a record or binding a waiting pair sets a second one. With both
set, the first `tick()` that binds no waiting pair calls
`rescan_unrecorded()`, which matches every local endpoint against the stored
endpoints again and reports each compatible pair without a record through
`report()`, under the usual rules: it binds, waits behind older waiting
pairs, or is turned away again. The walk stops at the first pair that binds,
so a tick still replays at most one TRANSIENT_LOCAL history, and resumes on
the next tick; a walk that binds nothing ends the rescan until another pair
is turned away and room frees again.

A remote endpoint whose only pair was turned away is in no record, so the
table's capacity rule may clear it for a newcomer that completes a match
(displacement keeps only endpoints in a record). Its pair is then found again
only when the endpoint is announced again.

### Services under pressure

A client's request writer and reply reader bind independently. The client
reports the server available only when both are bound, so `rclc` waits in
`rmw_service_server_is_available` rather than sending into a pair the server
cannot answer.

On the server, a request arrives through a bound writer proxy of the request
reader. If the reply writer's proxy for that client is still waiting, the
reply stays in the reply writer's history, which is TRANSIENT_LOCAL for
exactly this case (`release_replies_on_delivery()`): when the proxy binds, the
replay that `matched_reader_add()` performs delivers it, provided the history
still holds it (the reply writer's depth bounds how many replies wait). A
server does not discard a reply because its client's reader matched late.

## Deadlines

| Operation | Per-endpoint arrays | With pools |
|---|---|---|
| Send, HEARTBEAT, ACKNACK, NACK handling | O(cap) over the proxy array | O(k) over the k bound proxies |
| Find a proxy by GUID on receive | O(cap) | O(k) |
| Bind on a new match (SEDP DATA) | O(cap) free-slot scan + TRANSIENT_LOCAL replay | O(1) admission and free-list pop + O(k) append + the same replay |
| Bind a waiting match | never happens | at most one per `tick()`: O(match records × k) admission checks + one replay |
| Rescan after a pair was turned away | never happens | O(local × stored endpoints × match records) per walk, one walk per tick while it binds, then none until room frees again |
| Whether an endpoint gets its floor | depends on creation order | proven by `static_assert` |

The data path gets faster, not slower. The only new work is in `tick()`, it
is bounded by table sizes the firmware chooses, and it runs only when a match
waited.

## What peers see

A conforming RTPS participant. A remote reader whose pair is waiting has
discovered the writer through SEDP and receives neither DATA nor HEARTBEAT
from it until the pair binds, which is what it sees from any participant whose
resource limits are full; eProsima Fast DDS bounds matches the same way when
its `matched_subscriber_allocation` limit is configured. No peer needs a setting
changed, and once the pair binds the writer starts it exactly as a newly
matched reader (§8.4.1.4).

## With the two-tier peer table

The two designs are independent and compose:

* SEDP builtin proxies stay sized by active peers (`RTPS_MAX_PEERS`), and
  the `ros_discovery_info` writer reserves the same count.
* A peer is settled, and may be parked, only if it shares no match but
  `ros_discovery_info`. A waiting pair is a match, so a peer that wants
  something the pool cannot serve yet stays active until it is served.
* Parking a settled peer frees only builtin and `ros_discovery_info` proxies;
  user proxies free when matches dissolve.

## Fragment reassembly pool

Reassembly has the same structure. Each reader's `FragmentAssembler` holds
no slots of its own: the user readers share one participant-wide
`FragmentAssemblyPool` of `RTPS_ASSEMBLY_SLOTS` slots, each tagged with the
assembler that took it, and a reader slot is 76 B.

* A reader holds at most `RTPS_ASSEMBLY_SLOTS_PER_READER` slots. A new
  sample takes a free slot while the reader is below that limit.
* Otherwise the reader replaces one of its own partial samples: the one from
  the same writer first (KEEP_LAST(1) per writer, §8.3.3.1), then its oldest
  by the order the slots were taken. It never replaces another reader's, so a
  burst on one topic cannot discard a partial motion command.
* A reader that holds no slot while the pool is full is refused. The sample
  stays MISSING, the next ACKNACK requests it whole, and the writer's resend
  assembles once a slot has freed (§8.4.12).
* The default slot count is the number of blocks that can hold a reassembled
  sample, the datagram-class plus large-class block counts, because a slot
  without a buffer assembles nothing. The per-reader default is all but one
  slot, so one topic's burst always leaves a slot for another.
* The SEDP builtin readers keep private pools, one slot per writer they can
  match up to the per-reader limit, so discovery never waits on user
  traffic. A destroyed reader returns its slots and their blocks.
* **Limit:** a best-effort sample whose fragments were lost holds its slot
  and its block until the same reader replaces it, the writer unmatches, or a
  GAP covers it, so a quiet writer's lost sample can hold a slot
  indefinitely. At the default size slots and blocks are one to one: partial
  samples that fill the pool have pinned every block a sample could assemble
  in, which stopped per-reader assemblers just the same. A pool smaller than
  its blocks runs out of slots first. Ageing partial samples out would remove
  the limit.

## RAM

The example firmware (18 writers, 13 readers, 8 active peers, a cap of 8),
with per-endpoint proxy arrays and with pools sized for its graph. The pool
sizes are an illustration of the arithmetic, not a library default. The
first two columns are measured `sizeof` values on Cortex-M0+
(arm-none-eabi-g++ 10.3, -O2):

| Part | Per-endpoint arrays | Pools: 32 reader proxies, 24 writer proxies | And 6 assembly slots |
|---|---:|---:|---:|
| Writer slots | 18 × 1,680 = 30,240 | 18 × 1,048 = 18,864 | 18,864 |
| `ReaderProxy` pool | in the slots | 32 × (80 + 2) + 16 = 2,640 | 2,640 |
| Reader slots | 13 × 1,600 = 20,800 | 13 × 1,092 = 14,196 | 13 × 76 = 988 |
| `WriterProxy` pool | in the slots | 24 × (64 + 2) + 16 = 1,600 | 1,600 |
| Assembly pool | in the slots | in the slots | 6 × 136 + 8 = 824 |
| **Total** | **51,040** | **37,300** | **24,916** |
| `RTPSParticipant` | 82,392 | 68,808 | |

The proxy pools save 13.6 KB of the participant, and reassembly 12.4 KB more.
An assembly slot is the 128 B `FragmentedChange` plus its owner and age. The
two SEDP readers' private assembly pools shrink from 8 slots to 5, a further
672 B. Measured whole, the `rclc_chatter` firmware for RP2040 (11 readers, 12
writers) loses 11,008 B of `.bss` to the assembly pool, and 2.2 KB of `.text`
because the assembler is not a template. The 32 reader proxies cover the 8 the `ros_discovery_info` writer
reserves, one floor for each of the other 17 writers, and 7 to borrow. At the
default pool sizes, which hold every endpoint at its cap, the participant is
83,272 B: the pools cost 880 B over per-endpoint arrays, the per-proxy links
and pool headers.

Waiting matches add a ticket and a flag to each match record, 4 B with
padding, and a few bytes of table and participant state: 272 B on the
example firmware's 64 records, which are its 32 + 24 proxies and 8 waiting
matches.

## Configuration

| `rmw_rosette` knob | RTPS macro | Default | Meaning |
|---|---|---|---|
| `RMW_ROSETTE_MATCHED_READERS` | `RTPS_READER_PROXY_POOL` | (writers − 1) × `MAX_MATCHED_ENDPOINTS` + `MAX_REMOTE_PARTICIPANTS` | remote readers matched across all local writers at once |
| `RMW_ROSETTE_MATCHED_WRITERS` | `RTPS_WRITER_PROXY_POOL` | readers × `MAX_MATCHED_ENDPOINTS` | remote writers matched across all local readers at once |
| `RMW_ROSETTE_MAX_MATCHED_ENDPOINTS` | `RTPS_MAX_MATCHED_READERS` / `_WRITERS` | 8 (firmware) | cap per endpoint |
| `RMW_ROSETTE_MIN_MATCHED_ENDPOINTS` | `RTPS_MIN_MATCHES_PER_ENDPOINT` | 1 | floor per endpoint, reserved per slot |
| `RMW_ROSETTE_MATCH_WAITERS` | `RTPS_MAX_WAITING_MATCHES` | `MAX_MATCHED_ENDPOINTS` (firmware), 64 (host) | matches waiting for a proxy at once |
| (derived) | `RTPS_MAX_MATCH_RECORDS` | both pools + `MATCH_WAITERS` | match records: every bound and waiting pair |
| `RMW_ROSETTE_ASSEMBLY_SLOTS` | `RTPS_ASSEMBLY_SLOTS` | datagram + large block counts | fragmented samples in reassembly at once |
| (none) | `RTPS_ASSEMBLY_SLOTS_PER_READER` | `RTPS_ASSEMBLY_SLOTS` − 1 (at least 1) | fragmented samples one reader holds at once |

With the defaults the pools hold every endpoint at its cap at once, and the
`ros_discovery_info` writer at one proxy per active peer, so every floor is
met and nothing borrows beyond a cap: a build that sets nothing serves every
match a per-endpoint array would, without its two permanent refusals.
`rmw_rosette/include/rmw_rosette/capacity.hpp` `static_assert`s the floor
sums, and `docs/mcu_memory_model.md` gives the sizing step: list the
endpoints that need more than one match and add their extra to the floors.

## Not in this design

* **Writer history by depth.** Writer slots stay uniform at depth 8. `rclc`
  and the ROS 2 default profiles ask for depth 10, which maps to 8, so the
  history in most slots is used; a depth-1 writer saves 700 B in a slot of its
  own size, which would bring back slot-size classes for little.
* **Per-endpoint floors and caps** need a naming mechanism (see above).

## Verification

Spec tests, under the areas they belong to:

* Pool admission (`tests/spec`, writer and reader): an endpoint below its
  floor binds with every unreserved proxy borrowed; above its floor it binds
  only an unreserved proxy; it never exceeds its cap; `free ≥ reserved` after
  every bind and release.
* Waiting matches (participant, TC-PART-8.4.1.4-001/002): with an endpoint
  full, a new pair waits; the first release binds the oldest waiting pair at
  the next `tick()`, ahead of a pair reported after the release, one per
  tick; the bound reader receives the writer's TRANSIENT_LOCAL history;
  matched counts report bound pairs only.
* Waiting limit (TC-PART-8.4.1.4-004): with `RTPS_MAX_WAITING_MATCHES`
  pairs waiting, a further pair that would wait is not recorded and never
  binds, while a pair that may bind is still matched.
* Rescan (TC-PART-8.4.1.4-005/006): a pair turned away at the waiting limit
  is recorded on the first tick after room frees that binds no waiting pair,
  and binds after the pairs that waited before it; when room frees for
  several turned-away pairs at once, each tick's rescan binds one.
* Service under pressure (TC-PART-8.4.1.4-003): a reply written while the
  client's reply reader waits is delivered when it binds, and the waiting
  pair keeps the client's peer in a match; the client reports the server
  unavailable until both of its endpoints are bound.
* Reassembly pool (TC-FAS-8.4.14-009/010, TC-SRR-8.4.12-004): a reader
  replaces only its own partial samples, the same writer's first and then its
  oldest, and never holds more than its limit, even with a slot free; a
  reader holding no slot is refused while the pool is full and evicts
  nothing; a destroyed reader returns its slots; a refused fragmented sample
  from a reliable writer is requested whole and completes after repair.
* Every existing spec suite, and the loopback suites against `rmw_fastrtps_cpp`
  (interop, lockout, lease re-match, graph), pass unchanged at the default
  sizes, which reproduce per-endpoint capacity.
* `tests/ros2/discovery_scale` with the example firmware sizes and a topic
  read by more subscribers than a per-endpoint cap: every
  subscriber up to the pool's capacity is served.

## Decisions

* One shared pool per proxy type, sized by the firmware; the library's
  defaults reproduce per-endpoint capacity.
* A uniform floor reserved for every endpoint slot, proven at compile time,
  and a uniform per-endpoint cap.
* A refused match waits and binds oldest first when a proxy frees; it is never
  dropped.
* Builtin discovery endpoints never share a pool with user endpoints.
* Reassembly slots are pooled the same way, sized by the blocks that can hold
  a sample, and a reader only ever replaces its own partial samples.

## References

* OMG DDSI-RTPS 2.5 (formal/2022-04-01): §8.4.1.1 (ReaderProxy), §8.4.1.4
  (matched_reader_add / remove), §8.4.7.1 (WriterProxy), §8.4.7.4
  (matched_writer_add / remove), §8.4.12 (NACK_FRAG), §8.4.14 (fragment
  reassembly), §8.7.2.2 (TRANSIENT_LOCAL replay), §8.3.3.1 (KEEP_LAST).
* OMG DDS-RPC 1.0 §7.8.2 (related sample identity, for reply routing).
* eProsima Fast DDS documentation, DataWriter and DataReader resource limits
  (`matched_subscriber_allocation`, `matched_publisher_allocation`).
* A. K. Choudhury and E. L. Hahne, "Dynamic queue length thresholds for
  shared-memory packet switches," IEEE/ACM Transactions on Networking 6(2),
  1998: a shared buffer with per-queue guarantees and a bounded share for the
  rest, the same admission shape as the floor and cap here.
* `docs/design/two_tier_peer_table.md` (active peers, parking, pinning).
* `docs/mcu_memory_model.md` (per-object sizes).
