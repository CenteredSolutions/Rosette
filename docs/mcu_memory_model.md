# Memory on a Microcontroller

How Rosette holds its objects, why it holds them that way, and which knobs
change how much RAM a firmware image needs. Every number here was measured on
the W5100S-EVB-Pico2 (RP2350, Cortex-M33, `-O3`) with the example firmwares,
using the methods in [Measuring](#measuring).

## The rule: no heap in the stack

Rosette's RTPS core and `rmw_rosette` never call `malloc` or `new`. Every
object either lives in a fixed table inside its owner or comes from a fixed
pool, and every table and pool is sized at build time from a handful of
limits. The consequences are deliberate:

- **Failure moves to creation time.** Running out of room shows up as a
  `create_writer()` that returns null, or an `rmw_create_*()` that returns an
  error, when the firmware boots, not as fragmentation after a week in the
  field.
- **The RAM budget is visible in the linker map.** `arm-none-eabi-size`
  reports it, and `nm --size-sort` names each piece.
- **Steady state allocates nothing.** Publishing, receiving, discovery, and
  repair reuse the same blocks forever.

Two things outside Rosette do use the heap:

| Heap user | When | Measured |
| --- | --- | --- |
| `rcl` / `rclc` (stock C client libraries) | Initialization: context, node, entity handles, executor | `heap_in_use` steady at about 4 KB on `rclc_chatter`, unchanged across thousands of messages |
| Your messages | Whatever your message structs own | Zero if you give strings and sequences fixed buffers, as `rclc_chatter` does |

`rclc_chatter` prints `heap_in_use` (newlib `mallinfo().uordblks`) every five
seconds; a number that grows is a leak in application code.

## Where the RAM goes

Static RAM (`.bss`) of the two example firmwares, built with the default
firmware capacity:

| | `rtps_chatter` (RTPS core, direct) | `rclc_chatter` (rclc over `rmw_rosette`) |
| --- | ---: | ---: |
| Flash (`.text`) | 204,488 B | 285,236 B |
| Static RAM (`.bss`) | **62,836 B** | **99,816 B** |
| RTPS participant (all tables below) | 28,824 | 55,392 |
|   writer slots | 4 × 1,048 | 7 × 1,048 + 4 × 648 |
|   reader slots | 4 × 76 | 10 × 76 |
|   `ReaderProxy` / `WriterProxy` pools | 2,640 + 2,128 | 7,232 + 5,296 |
|   assembly pool (6 slots) | 824 | 824 |
|   `EndpointMatchTable` | 9,760 | 15,616 |
|   SEDP writer + reader | 4,048 + 2,632 | 4,784 + 2,632 |
|   SPDP reader (peer table) | 1,532 | 7,516 |
| Buffer pool (`rosette::rtps::g_buf_pool`) | 26,988 | 34,080 |
| Receive buffer (one datagram) | 1,472 | (inside the rmw context) |
| rmw entity pools (context, publishers, subscriptions, ...) | – | ~8 KB beyond the participant |
| USB stdio, Pico SDK, board network | ~5 KB | ~5 KB |

Two observations drive most tuning:

1. **Declared capacity drives the participant.** Writer slots, both proxy
   pools, and the match records all scale with the number of local
   endpoints, and in `rclc_chatter` they are sized for the firmware defaults
   of 4 publishers, 4 subscriptions, 4 services, and 2 clients. The example
   uses one publisher and one subscription.
2. **The buffer pool is the largest single item**, and its small-block
   counts scale with the number of local endpoints.

Declaring what the firmware actually creates fixes both at once (see
[Worked example](#worked-example-rclc_chatter-sized-to-what-it-creates)).

## Object model

```
RTPSParticipant ─────────────────────────────── one per process / firmware
├── SpdpWriter, SpdpReader        participant discovery; SpdpReader holds the
│                                 peer table: RTPS_MAX_KNOWN_PEERS × PeerEntry
│                                 records, RTPS_MAX_PEERS active slots
├── SedpWriter, SedpReader        endpoint discovery (ordinary reliable
│                                 StatefulWriter/StatefulReader underneath)
├── EndpointMatchTable            local endpoints with full names; remote
│                                 endpoints as 16-byte records; match pairs
├── writer_storage_[RTPS_MAX_USER_WRITERS]   each slot holds one
│       SharedPoolStatefulWriter<Depth>
│         └── HistoryCache of Depth CacheChanges ─► BufRef into the pool
├── reader_storage_[RTPS_MAX_USER_READERS]   each slot holds one
│       SharedPoolStatefulReader (a FragmentAssembler view, no slots)
├── reader_proxies_   RTPS_READER_PROXY_POOL × ReaderProxy, bound by the
│                     user writers as remote readers match
├── writer_proxies_   RTPS_WRITER_PROXY_POOL × WriterProxy, bound by the
│                     user readers as remote writers match
└── assembly_slots_   RTPS_ASSEMBLY_SLOTS × AssemblySlot, each a fragmented
                      sample in reassembly ─► BufRef into the pool

rosette::rtps::g_buf_pool   five size classes of reference-counted blocks; every
                   payload in the system lives in one of these blocks

rmw_rosette        Pool<T, N> per entity kind (context, node, publisher,
                   subscription, service, client, guard condition, wait set);
                   each subscription/service/client owns a ChangeRing of
                   BufRefs waiting for rmw_take()
```

### Endpoints are placement-constructed into fixed slots

`RTPSParticipant::create_writer<Depth>()` placement-constructs a
`SharedPoolStatefulWriter<Depth>` into a free byte slot and returns a
`StatefulWriter*`; `delete_writer()` runs its destructor, which releases every
payload its history holds, and frees the slot. Readers work the same way with
`create_reader()` and `delete_reader()`. Nothing is ever allocated.

`SharedPoolStatefulWriter<D>` is a template only over *storage*: all writer
logic is compiled once in `src/rtps/stateful_writer.cpp` and works on the
history the storage hands it and the proxies it binds from the pool. A writer of depth 1 and one of
depth 8 differ in layout, never in code, so adding depths costs RAM, not
flash.

**Writer slots come in two sizes.** A deep slot is
`RTPS_PARTICIPANT_MAX_WRITER_BYTES` long, by default
`sizeof(SharedPoolStatefulWriter<RTPS_DEEP_WRITER_DEPTH>)` (1,048 B at the
default depth of 8), and holds a writer of any depth up to
`RTPS_DEEP_WRITER_DEPTH` (1 to 255; `RMW_ROSETTE_MAX_WRITER_DEPTH` in
`rmw_rosette`). `RTPS_SHALLOW_WRITER_DEPTH` is at most that, so a shallow slot
is never larger than a deep one. A shallow slot is `sizeof(SharedPoolStatefulWriter<RTPS_SHALLOW_WRITER_DEPTH>)`
long; `RTPS_MAX_SHALLOW_WRITERS` of the `RTPS_MAX_USER_WRITERS` slots are
shallow. The creator names the class (`WriterConfig::slotClass`): a shallow
writer takes only shallow slots and every other writer only deep ones. The
participant does not infer the class from a writer's depth, because a
shallow-depth writer in a shallow slot, or a shallow writer in a deep one,
would leave the other class short after endpoint churn. Each class is sized for
its own writers, so writers can be created and destroyed in any order while
each class stays within its count.

`rmw_rosette` gives each service reply writer a shallow slot. A reply writer
drops each reply once its client has it, so its history holds only the
replies in flight, `RMW_ROSETTE_REPLY_DEPTH` (4) of them, whatever the
service's QoS requests: a depth-4 reply writer takes 648 B where a deep slot
takes 1,048 B. Every other writer takes a deep slot, where a depth-1 writer
(352 B) still occupies 1,048 B.

### Per-match state lives in shared pools

A writer's `ReaderProxy`s, a reader's `WriterProxy`s, and a reader's partial
fragmented samples are not stored in the endpoint. They come from three
participant-wide pools, so RAM follows the matches the firmware has at once
rather than endpoints × the most matches any one endpoint could have:

- **Proxies.** Each endpoint slot has `RTPS_MIN_MATCHES_PER_ENDPOINT`
  proxies reserved from the start (its floor), and borrows unreserved ones up
  to its cap (`RTPS_MAX_MATCHED_READERS` / `_WRITERS`) as it matches. A match
  the pool or the cap refuses waits, up to `RTPS_MAX_WAITING_MATCHES` at once,
  and binds oldest first as proxies free; a pair turned away beyond that is
  found again from the stored endpoints once room frees.
- **Assembly slots.** A fragmented sample takes one slot and one pool block
  until its last fragment arrives. The default `RTPS_ASSEMBLY_SLOTS` is the
  number of blocks that can hold such a sample (the datagram and largest
  classes), because a slot without a block assembles nothing. A reader holds
  at most `RTPS_ASSEMBLY_SLOTS_PER_READER` (by default all but one) and
  replaces only its own partial samples, so a burst on one topic never
  discards another topic's.

The SEDP builtin endpoints keep private pools, so discovery never waits on
user traffic. The rules, and why they are what they are, are in
[`docs/design/shared_proxy_pools.md`](design/shared_proxy_pools.md).

### Per-object costs

Measured `sizeof` on Cortex-M (the RP2040 and RP2350 lay structures out
identically) at the firmware defaults:

| Object | Size | Scales with |
| --- | ---: | --- |
| `CacheChange` (one history entry) | 100 B | writer depth |
| `ReaderProxy` (one matched remote reader) and its pool link | 80 + 2 B | `RTPS_READER_PROXY_POOL` |
| `WriterProxy` (one matched remote writer) and its pool link | 64 + 2 B | `RTPS_WRITER_PROXY_POOL` |
| `SharedPoolStatefulWriter<1>` / `<4>` / `<8>` | 352 / 648 / 1,048 B | depth × 100 + ~250 |
| Writer slot | 1,048 B | `RTPS_PARTICIPANT_MAX_WRITER_BYTES` |
| Reader slot (`SharedPoolStatefulReader`) | 76 B | `RTPS_MAX_USER_READERS` |
| `AssemblySlot` (one fragmented sample in reassembly) | 136 B | `RTPS_ASSEMBLY_SLOTS` |
| SEDP builtin reader (`BoundedStatefulReader<8>`: private proxies and 5 assembly slots) | 1,312 B | `RTPS_MAX_PEERS` |
| `PeerEntry` (one known remote participant: identity, lease, one locator) | 68 B | `RTPS_MAX_KNOWN_PEERS` |
| Active slot (`DiscoveredParticipant`: both ranked locator lists) | 120 B | `RTPS_MAX_PEERS` |
| Remote origin (GUID prefix + locator list index) | 16 B | `RTPS_MAX_REMOTE_ORIGINS` (= peers) |
| Remote locator list (endpoint locators other than its participant's defaults) | 52 B | `RTPS_MAX_REMOTE_LOCATOR_LISTS` (4) |
| Remote endpoint record | 16 B | `RTPS_MAX_DISCOVERED_WRITERS` / `_READERS` |
| Local endpoint record (two 128-octet names + QoS) | 296 B | local endpoints |
| Match record (one writer-reader pair) | 36 B | `RTPS_MAX_MATCH_RECORDS`: one per pooled proxy plus `RTPS_MAX_WAITING_MATCHES` |
| `ChangeRing` (rmw take history), `PendingChange` 72 B each | 732 B | `RMW_ROSETTE_MAX_PENDING_CHANGES` (10) |
| Pool block header | 12 B | every pool block |

A writer's `ReaderProxy` records each reader's progress as sequence-number
bounds plus a 64-bit window of requested changes, not a row per history slot,
so its size does not grow with depth (DDSI-RTPS 2.5 §8.4.7 describes the
reader-proxy state; Rosette keeps it in this bounded form).

Neither proxy stores locators. Each keeps a pointer to the one locator it
sends to, inside the record that already holds it: the SPDP peer record for a
participant's locators, or a remote locator list for locators an endpoint
announced itself. The proxy therefore costs the same however many locators
its peer announced, and a peer that keeps its GUID and moves to a new address
is reached there as soon as its SPDP refresh updates the record.

### Remote endpoints are kept small, and displaced when full

A network can announce far more endpoints than a microcontroller can record.
`EndpointMatchTable` keeps each remote endpoint as a 16-byte record: its
EntityId, QoS, a 32-bit FNV-1a key of the topic name and one of the type name,
and an index into a shared 16-byte *origin*. All endpoints of one participant
share its origin: its GUID prefix, and no locators, because Fast DDS and
Rosette endpoints announce their participant's default locators and Cyclone DDS
endpoints announce none, so they are reached at the defaults the peer record
already keeps. Only an endpoint that announces other locators takes an origin
with one of `RTPS_MAX_REMOTE_LOCATOR_LISTS` locator lists, and when those are
all taken it is kept under the same rule as a full table, below. Names are
compared exactly while the announcement is in hand; later comparisons use the
keys, and a false match needs both 32-bit keys of two different endpoints to
collide.

When the remote table is full, an announcement that completes a match with a
local endpoint takes the slot of a record that is in no match; one that
completes none is dropped. A record in a match is never displaced. The table
therefore needs room only for the remote endpoints the firmware actually
matches, not for the whole graph. This works because only `rmw_wait()` (or
`ParticipantDriver::spin_until()`) runs the participant: a firmware that
creates all its entities before it first spins, as `rclc` applications do, is
matching against its full endpoint set from the first SEDP datagram.

### Payloads live once, in the buffer pool

A sample is serialized once, into one pool block, and sent from that block. A
received sample is copied once, from the receive buffer into a block, and
handed along by reference from then on:

```
rcl_publish ─► serialize into a pool block ─► writer history (BufRef)
            ─► DATA, or DATA_FRAG pieces of the same block ─► wire

wire ─► receive buffer (one datagram) ─► DATA payload copied into a block,
                                         or DATA_FRAG pieces assembled into one
     ─► reader callback ─► ChangeRing keeps the BufRef ─► rmw_take deserializes
```

Blocks are reference-counted (`BufRef`), so the writer history, an in-flight
send, and a subscription's `ChangeRing` can share one block; it returns to the
pool when the last reference is released. The pool has five size classes;
`buf_alloc(n)` takes the smallest class that fits, and when that class is
empty it tries larger ones, **except the largest**. The largest class exists
only for samples bigger than one datagram, so small traffic draining the
small classes cannot starve large samples.

A sample is always stored contiguously in one block (fragmented only on the
wire). This keeps serialization, repair, and reassembly simple: every
operation sees one span of octets, never a chain.

Who holds blocks at any moment:

| Holder | Blocks held | Released when |
| --- | --- | --- |
| Writer history | up to depth samples per writer | every matched reliable reader acknowledges, or depth is exceeded |
| SEDP writers | one announcement per local endpoint, plus one disposal | the endpoint is deleted |
| Fragment reassembly | one per sample being reassembled, at most `RTPS_ASSEMBLY_SLOTS` | the last fragment arrives, the reader replaces the sample with a newer one, or the writer is unmatched |
| `ChangeRing` (rmw) | up to the entity's KEEP_LAST depth | `rmw_take()`, or replaced by a newer sample |
| Message builder | one while a datagram is built | the datagram is sent |

A subscription that is not taken keeps up to its depth of samples pinned. A
firmware that subscribes to large messages with depth 10 needs room for ten
of them.

### Concurrency

One mutex guards the participant and everything its callbacks touch
(`ParticipantDriver::mutex()`); RTPS objects take no locks of their own. The
buffer pool has its own mutex so any thread or core can allocate and release.
Interrupt handlers must not allocate or release blocks; the WIZnet INT pin
only signals the driver's wake event.

## What happens when a limit is reached

| Limit reached | Behavior |
| --- | --- |
| Writer or reader slot table full | `create_writer()`/`create_reader()` return null; `rmw_create_*()` returns null with an error message |
| rmw entity pool full | `rmw_create_*()` returns null; the error message names the exhausted pool |
| Topic or type name longer than `RTPS_MAX_TOPIC_NAME_LEN - 1` | local endpoint refused; remote endpoint ignored, never matched by a truncated prefix |
| Serialized sample larger than the largest class | publish fails; a received one is marked LOST on its first fragment |
| Buffer pool exhausted | publish returns an error; a received DATA is not recorded, so a reliable writer repairs it later |
| `ChangeRing` full | the oldest unread sample is released (KEEP_LAST semantics) |
| Remote endpoint table full | displacement rule above; an unmatched announcement is dropped |
| Proxy pool empty, or an endpoint at its cap | the match waits (it still counts as matched for discovery but not in `rmw_*_count_matched_*`) and binds oldest first when a proxy frees (`endpoint match waits for a proxy`) |
| `RTPS_MAX_WAITING_MATCHES` matches waiting | a further match is not recorded (`waiting matches full`); once a match dissolves or a waiter binds, the next `tick()` rescans the stored endpoints and finds it again |
| Assembly pool full, and the reader holds no slot | the fragmented sample is refused; a reliable writer repairs it once a slot frees |
| Peer table full | a new participant is dropped (logged as `peer table full`) until an entry frees by lease expiry or disposal |

## Knobs

### rmw_rosette firmware (`rclc` applications)

Set these as CMake cache variables (`-D` on the command line, or `set(... CACHE
...)` before Rosette is added). `rmw_rosette/cmake/rmw_rosette_capacity.cmake`
derives every RTPS table and pool size from them, and
`rmw_rosette/include/rmw_rosette/capacity.hpp` `static_assert`s the
relationships, so an inconsistent set fails to compile.

The cost per unit of the entity counts is the growth in `.bss` when one is
added to the worked example's declared `rclc_chatter` (1 publisher,
1 subscription) on the RP2350. It includes the entity's share of the proxy
pools and match records at the default cap of 8, and the small pool blocks
and local endpoint record that follow the larger of the writer and reader
counts, which is why a publisher costs more than a subscription there.

| Knob | Firmware default | What it sizes | Cost per unit |
| --- | ---: | --- | --- |
| `RMW_ROSETTE_MAX_PUBLISHERS` | 4 | writer slot, 8 reader proxies, local endpoint and match records, SEDP history, publisher handle | 3,388 B |
| `RMW_ROSETTE_MAX_SUBSCRIPTIONS` | 4 | reader slot, 8 writer proxies, match records, subscription handle with its `ChangeRing` | 936 B |
| `RMW_ROSETTE_MAX_SERVICES` | 4 | one reply writer in a shallow slot **and** one reader, with their proxies, records, and handle | 3,668 B |
| `RMW_ROSETTE_MAX_CLIENTS` | 2 | one writer and one reader, with their proxies, records, and handle | 4,068 B |
| `RMW_ROSETTE_MAX_NODES` | 1 | node pool; multiplies the per-node entities below | |
| `RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE` | 0 | 6 for `rclc_parameter_server`, 0 without | 6 services per node (≈ 24 KB) |
| `RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE` | 0 | `/rosout`, `/parameter_events` | 0 when rcl logging is off, as in the firmware C stack |
| `RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS` | 8 | active peers: active slot, SEDP reader and writer proxies, remote origins | ≈ 420 B |
| `RMW_ROSETTE_MAX_KNOWN_PARTICIPANTS` | 96 | known-peer records, active or parked (`PeerEntry`) | 68 B |
| `RMW_ROSETTE_MAX_REMOTE_ENDPOINTS` | 64 | remote writer and reader records | 32 B (16 B in each table) |
| `RMW_ROSETTE_MAX_MATCHED_ENDPOINTS` | 8 | cap: the most remote readers one writer serves, or remote writers one reader hears; sizes the proxy pools when they are left empty | through the pools |
| `RMW_ROSETTE_MATCHED_READERS` | empty: (writers − 1) × cap + remote participants | the `ReaderProxy` pool, shared by every writer | 82 B, plus a 36 B match record |
| `RMW_ROSETTE_MATCHED_WRITERS` | empty: readers × cap | the `WriterProxy` pool, shared by every reader | 66 B, plus a 36 B match record |
| `RMW_ROSETTE_MIN_MATCHED_ENDPOINTS` | 1 | floor: proxies reserved for every endpoint slot, so its first matches never wait | raises the pools' minimum size |
| `RMW_ROSETTE_MATCH_WAITERS` | 8 (`MAX_MATCHED_ENDPOINTS`) | matches waiting for a proxy at once, oldest bound first | 36 B (one match record) |
| `RMW_ROSETTE_ASSEMBLY_SLOTS` | empty: the datagram and large block counts (6) | fragmented samples in reassembly at once, across all readers | 136 B |
| `RMW_ROSETTE_MAX_WRITER_DEPTH` | 8 | deepest KEEP_LAST history a publisher keeps (1 to 255); deeper requests and KEEP_ALL run at this depth; sizes the deep writer slots | 100 B per history slot per deep writer |
| `RMW_ROSETTE_REPLY_DEPTH` | 4 | replies each service holds until acknowledged (1 to `RMW_ROSETTE_MAX_WRITER_DEPTH`); a reply written while this many are unacknowledged displaces the oldest; sizes the shallow writer slots | 100 B per reply per service |
| `RMW_ROSETTE_MAX_PENDING_CHANGES` | 10 | `ChangeRing` depth ceiling per subscription, service, and client | 72 B per entry per entity, plus the blocks unread entries pin |
| `RMW_ROSETTE_MAX_DATAGRAM_SIZE` | 1472 | receive buffer, datagram pool class, DATA_FRAG threshold | the datagram class × 4 blocks |
| `RMW_ROSETTE_MAX_SAMPLE_SIZE` | 4096 | largest pool class: the biggest message sent or received | × `RMW_ROSETTE_LARGE_BUFFERS` |
| `RMW_ROSETTE_LARGE_BUFFERS` | 2 | blocks of `MAX_SAMPLE_SIZE`: large samples in flight at once | `MAX_SAMPLE_SIZE` each |
| `RMW_ROSETTE_POOL_PROFILE` | `firmware` | pool layout: `firmware` sizes classes by role; `host` for many peers and deep histories | |

Each service and client is one RTPS writer plus one RTPS reader, and the
participant adds one writer of its own for `ros_discovery_info`, which is how
stock `ros2 node list` sees the firmware's node.

### RTPS core used directly (`rosette_rtps`, as in `rtps_chatter`)

`cmake/rosette_rtps.cmake` takes a smaller set of board inputs:

| Knob | Default | What it sizes |
| --- | ---: | --- |
| `ROSETTE_RTPS_MAX_DATAGRAM_SIZE` | 1472 | receive buffer, datagram pool class |
| `ROSETTE_RTPS_MAX_SAMPLE_SIZE` | 4096 | largest pool class |
| `ROSETTE_RTPS_MAX_SAMPLE_BLOCKS` | 2 | count of the largest class |
| `ROSETTE_RTPS_MAX_PEERS` | 8 | peer table |
| `ROSETTE_RTPS_MAX_REMOTE_ENDPOINTS` | 64 | remote writer and reader records |
| `ROSETTE_RTPS_LOG_LEVEL` | 2 | log verbosity (flash, not RAM) |

The endpoint slot counts are fixed at four writers and four readers in that
file; a firmware with a different shape edits them there.

### WIZnet socket receive buffers

The WIZnet chip holds received datagrams in per-socket receive memory until
firmware reads them, and drops any that no longer fit. Every fragment of a
large sample arrives back to back, so the socket that carries RTPS user
traffic needs room for a whole sample; a DHCP or SNTP socket needs one
datagram. The driver therefore sizes the sockets unevenly. This memory is on
the chip, not in MCU RAM.

| Chip | Receive memory | Default layout (KiB per socket) |
| --- | ---: | --- |
| W5100S | 8 KiB, 4 sockets | 4 / 2 / 1 / 1 |
| W6100 | 16 KiB, 8 sockets | 8 / 2 / 1 / 1 / 1 / 1 / 1 / 1 |
| W6300 | 32 KiB, 8 sockets | 16 / 4 / 2 / 2 / 2 / 2 / 2 / 2 |

A plain `open()` takes the smallest free buffer. A socket that needs more asks
before `bind()` with `Socket::set_receive_buffer_size()` and moves to the
smallest buffer that holds the request, or the largest free one. The RTPS
user socket asks for the largest pool block plus an eighth for fragment
framing (`RTPS_USER_SOCKET_RECEIVE_BUFFER`, 4608 B with
`RMW_ROSETTE_MAX_SAMPLE_SIZE` 4096), the metatraffic socket for 2 KiB
(`RTPS_META_SOCKET_RECEIVE_BUFFER`), and the micro-ROS agent backend's XRCE
socket for a reliable-stream window (`ROSETTE_XRCE_SOCKET_RECEIVE_BUFFER`,
stream history × MTU). Transmit memory stays split evenly: a socket holds one
outgoing datagram at a time.

Set `ROSETTE_WIZNET_RX_LAYOUT_KIB` (for example `8;4;1;1;1;1;0;0`) to override
the layout; `boards/common/rp2_wiznet/wiznet_board_types.hpp` rejects at
compile time a size that is not 0 or a power of two, or a layout that exceeds
the chip's memory, and a socket given 0 KiB is never handed out. When the user
socket's buffer is smaller than a sample's fragments (the W5100S's 4 KiB against
a full 4096-octet sample), the reader
requests the dropped fragments with NACK_FRAG and the sample still arrives, one
writer heartbeat period later.

### Low-level macros

Both paths end in these preprocessor definitions, which the headers read. Set
them through the CMake modules above rather than by hand: the participant's
tables are laid out in headers that application code includes too, so every
translation unit must see the same values (the one-definition rule).

| Macro | Header | Notes |
| --- | --- | --- |
| `RTPS_MAX_USER_WRITERS`, `RTPS_MAX_USER_READERS` | `participant.hpp` | slot counts |
| `RTPS_DEEP_WRITER_DEPTH` | `participant.hpp` | deepest history any writer keeps (1 to 255), which deep writer slots are sized for; `rmw_rosette` sets it to `RMW_ROSETTE_MAX_WRITER_DEPTH` |
| `RTPS_MAX_SHALLOW_WRITERS`, `RTPS_SHALLOW_WRITER_DEPTH` | `participant.hpp` | shallow writer slots, out of `RTPS_MAX_USER_WRITERS`, and the deepest history one holds (at most `RTPS_DEEP_WRITER_DEPTH`); `rmw_rosette` sets one per service at `RMW_ROSETTE_REPLY_DEPTH` |
| `RTPS_PARTICIPANT_MAX_WRITER_BYTES` | `participant.hpp` | deep slot size; lowering it below `StatefulWriterStorage<8, R>` saves ~700 B per slot but makes `create_writer<8>()` fail to compile, and `rmw_rosette` instantiates depths 1, 4, and 8 (`kSupportedWriterDepths` in `qos.hpp`) |
| `RTPS_MAX_MATCHED_READERS`, `RTPS_MAX_MATCHED_WRITERS` | `stateful_writer.hpp`, `types.hpp` | cap: proxies per endpoint |
| `RTPS_READER_PROXY_POOL`, `RTPS_WRITER_PROXY_POOL`, `RTPS_MIN_MATCHES_PER_ENDPOINT` | `participant.hpp`, `proxy_pool.hpp` | shared proxy pools and the floor reserved per endpoint slot; a `static_assert` proves the pools hold every floor |
| `RTPS_MAX_WAITING_MATCHES` | `endpoint_match_table.hpp` | matches waiting for a proxy; `RTPS_MAX_MATCH_RECORDS` must cover both pools plus these |
| `RTPS_ASSEMBLY_SLOTS`, `RTPS_ASSEMBLY_SLOTS_PER_READER` | `fragment_assembler.hpp` | shared reassembly slots (default: datagram and largest class block counts) and the most one reader holds (default: all but one) |
| `RTPS_MAX_PEERS`, `RTPS_MAX_PEER_LOCATORS` | `types.hpp`, `spdp.hpp` | peer table |
| `RTPS_MAX_DISCOVERED_WRITERS/_READERS`, `RTPS_MAX_REMOTE_ORIGINS`, `RTPS_MAX_REMOTE_LOCATOR_LISTS`, `RTPS_MAX_MATCH_RECORDS` | `endpoint_match_table.hpp` | remote records, endpoint locator lists, and match pairs |
| `RTPS_MAX_TOPIC_NAME_LEN` | `endpoint_match_table.hpp` | 128; ROS 2 service topics embed the node name, so shorter limits refuse real names |
| `RTPS_POOL_CLASS<n>_SIZE/_COUNT` | `buf_pool.hpp` | buffer pool layout |

## Worked example: `rclc_chatter` sized to what it creates

`rclc_chatter` creates one publisher, one subscription, and one timer. Telling
the build so:

```bash
cmake -S . -B build-hw/pico2-lean -DROSETTE_BOARD=w5100s-evb-pico2 \
      -DROSETTE_ROS2_C_STACK_DIR=<micro-ROS workspace>/firmware/mcu_ws/install \
      -DRMW_ROSETTE_MAX_PUBLISHERS=1 -DRMW_ROSETTE_MAX_SUBSCRIPTIONS=1 \
      -DRMW_ROSETTE_MAX_SERVICES=0 -DRMW_ROSETTE_MAX_CLIENTS=0
cmake --build build-hw/pico2-lean --target rclc_chatter
```

| | Default capacity | Declared capacity | Saved |
| --- | ---: | ---: | ---: |
| `.bss` total | 99,816 B | **63,980 B** | 35,836 B |
| rmw context (participant, transport, rx buffer) | 57,560 | 24,880 | 32,680 |
| buffer pool | 34,080 | 31,668 | 2,412 |
| subscription + publisher pools | 4,976 | 1,256 | 3,720 |
| `.text` | 285,236 | 279,108 | 6,128 |

The participant now has two writer slots (the publisher and
`ros_discovery_info`), one reader slot, and proxy pools sized for them (16
reader proxies, 8 writer proxies). The SPDP peer table, 7.5 KB for 96 known
participants, is the largest part left; it is sized by the network, not the
firmware. Nothing about the protocol changed.

## Sizing a new firmware

1. **Count local entities.** Publishers, subscriptions, services, clients per
   node, plus what the client library adds (parameter services if you start
   `rclc_parameter_server`; `/rosout` if rcl logging is on). Set the
   `RMW_ROSETTE_MAX_*` limits to exactly those counts.
2. **Count the network.** `RMW_ROSETTE_MAX_KNOWN_PARTICIPANTS` must cover
   every ROS 2 process on the domain (each process is one participant; a peer
   beyond it is ignored). `RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS` is how many of
   them exchange endpoint discovery at once: the peers the firmware has a
   match with, plus a few slots through which the rest of the network rotates
   (a settled peer with no match is parked to make room; see
   `src/rtps/participant_peer_tiers.cpp`). More rotating slots let a large
   network find the firmware's matches sooner after power-up.
   `RMW_ROSETTE_MAX_REMOTE_ENDPOINTS` needs to cover only the remote endpoints
   your endpoints match, thanks to displacement; more lets
   `rmw_count_publishers()` see more of the graph.
3. **Match fan-out.** `RMW_ROSETTE_MAX_MATCHED_ENDPOINTS` is the most remote
   readers any one of your writers serves, or remote writers any one of your
   readers hears. Left alone, the proxy pools hold every endpoint at that cap
   at once, which serves every match but pays for the worst case. To pay for
   the matches you have, list the endpoints that need more than one match at
   once (a status topic many nodes read, a service several clients call) and
   set `RMW_ROSETTE_MATCHED_READERS` to one floor per writer, plus one per
   active peer for `ros_discovery_info`, plus those endpoints' extra; likewise
   `RMW_ROSETTE_MATCHED_WRITERS` for readers. A match beyond the pool waits
   and is served when another leaves; the build rejects a pool smaller than
   its floors.
4. **Fragmented samples.** `RMW_ROSETTE_ASSEMBLY_SLOTS` defaults to the blocks
   that can hold a large sample, which is the most that can ever reassemble
   at once; it need not be set unless you want fewer.
5. **Largest message.** `RMW_ROSETTE_MAX_SAMPLE_SIZE` is the largest
   serialized message in either direction (CDR size, including the 4-octet
   encapsulation header). `RMW_ROSETTE_LARGE_BUFFERS` is how many such
   messages can be in flight at once: writer histories awaiting
   acknowledgment, reassembly, and unread subscription entries.
6. **Link.** Keep `RMW_ROSETTE_MAX_DATAGRAM_SIZE` at 1472 on WIZnet hardware
   UDP, which cannot reassemble IP fragments.
7. **Depth.** Ask for the QoS depth you mean. A depth-10 subscription (the
   `rclc` default) pins up to ten samples; depth 1 pins one.
8. **Build and read the numbers** (below). Then run the firmware against the
   real network and watch for creation failures and pool errors.

## Measuring

**Totals and the largest objects**:

```bash
arm-none-eabi-size build-hw/pico2/examples/rclc_chatter/rclc_chatter.elf
arm-none-eabi-nm -C -S --size-sort build-hw/pico2/examples/rclc_chatter/rclc_chatter.elf \
  | grep -E ' [bBdD] ' | tail -20
```

**The size of one type** at the board's exact configuration: compile a
translation unit with the firmware's flags (copy them from
`compile_commands.json`) and `-S`, then read the constants from the assembly:

```cpp
#include "rosette/rtps/participant.hpp"
extern const unsigned s_participant = sizeof(rosette::rtps::RTPSParticipant);
extern const unsigned s_match       = sizeof(rosette::rtps::EndpointMatchTable);
extern const unsigned s_reader      = sizeof(rosette::rtps::DefaultStatefulReader);
```

```bash
grep -A1 '^s_' probe.s
```

**Heap**: print `mallinfo().uordblks` periodically, as `rclc_chatter` does.

**Pool pressure**: configure with `-DROSETTE_RTPS_POOL_STATS=ON` and call
`rosette::rtps::buf_pool_stats_log(true)` from a periodic status report
(`rtps/buf_pool_stats.hpp`). For each class it logs supply (blocks served,
live, peak) and demand: the requests whose best fit is that class, their
smallest, mean, and largest size, a histogram of those sizes across the
class's range, how many spilled up to a larger class because this one was
empty, and how many got no block. A refusal is also logged as it happens, on
the 1st, 2nd, 4th, ... of its class. Sizes are what the caller requested,
headroom included; the pool does not see how much of the block is then
written.

Read it as: a peak well below the count means the class can shrink; spills
or refusals mean it is too small; a histogram crowded at the top of a class's
range means the class below just misses a common size, and one crowded at
the bottom means most of each block goes unused.

For a single leak or a block held too long, `-DRTPS_BUFPOOL_DEBUG` adds an
allocation event ring and the allocating PC of every live block
(`MultiBufPool::debug_class_stats()`, `debug_recent_event()`).

**Datagrams**: `rtps_chatter` prints the transport counters (largest datagram
sent and received, oversize, truncated); on a host, `RMW_ROSETTE_TRANSPORT_STATS=1`
prints them at shutdown.

## Design decisions

| Decision | Rationale |
| --- | --- |
| No heap in the stack | Bounded, visible RAM; failures at boot; no fragmentation over long uptimes. The same discipline as MISRA C:2012 Rule 21.3 and the JPL/NASA "Power of Ten" rule 3. |
| Capacity from a few application limits, derived in one CMake module | An application states what it creates; the derived tables cannot be sized inconsistently, and `capacity.hpp` asserts it. |
| Templates lay out storage; logic compiles once | `SharedPoolStatefulWriter<D>`, `HistoryCacheStorage<N>`, `ProxyPoolStorage<P, N>`, and `FragmentAssemblyPoolStorage<N>` hold arrays; `StatefulWriter`, `HistoryCache`, `ProxyPool`, and `FragmentAssembler` are plain classes in `.cpp` files. Depth is a RAM decision that costs no flash (the approach of LLVM's `SmallVector` / `SmallVectorImpl`). |
| Two writer slot classes | A deep slot takes any depth and a shallow slot only the writers the creator names as shallow (service replies), so creation and deletion order never fragment either class, while shallow writers do not pay for the deepest history. |
| Per-match state in shared pools, with floors | RAM follows the matches a firmware has at once, not endpoints × cap; a floor per slot means an endpoint's first match never depends on what others borrowed, and a refused match waits instead of being dropped. A shared buffer with per-queue guarantees, as in shared-memory packet switches (Choudhury and Hahne, 1998). |
| Contiguous samples, fragmented only on the wire | Serialization, repair, and reassembly work on one span; the largest pool class is reserved so small traffic cannot starve large samples. |
| Reference-counted blocks | Zero-copy from serialization to the wire and from reassembly to `rmw_take()`. |
| Remote endpoints as keyed 16-byte records with displacement | A board on a network of dozens of participants records only what it matches; a matched record is never displaced, so matching never loses a pair it reported. |
| `ChangeRing` depth equals the requested QoS depth | The RTPS reader has already acknowledged what the ring holds, so the ring is the reader history, with KEEP_LAST semantics (DDS 1.4 §2.2.3.18). |
| One participant lock, no locks inside RTPS objects | Simple lock ordering; interrupt handlers only signal a wake event. |
