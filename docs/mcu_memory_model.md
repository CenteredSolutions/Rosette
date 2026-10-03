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
| Flash (`.text`) | 157,124 B | 237,796 B |
| Static RAM (`.bss`) | **74,124 B** | **123,720 B** |
| RTPS participant (all tables below) | 36,000 | 68,232 |
|   writer slots | 4 × 2,016 | 11 × 2,016 |
|   reader slots | 4 × 1,984 | 10 × 1,984 |
|   `EndpointMatchTable` | 9,376 | 14,816 |
|   SEDP writer + reader | 4,624 + 3,976 | 5,360 + 3,976 |
|   SPDP reader (peer table) | 1,388 | 1,388 |
| Buffer pool (`rtps::g_buf_pool`) | 32,228 | 42,464 |
| Receive buffer (one datagram) | 1,472 | (inside the rmw context) |
| rmw entity pools (context, publishers, subscriptions, ...) | – | ~6 KB beyond the participant |
| USB stdio, Pico SDK, board network | ~5 KB | ~5 KB |

Two observations drive most tuning:

1. **Endpoint slots dominate the participant.** In `rclc_chatter`, 42 KB of
   the 68 KB participant is writer and reader slots, sized for the firmware
   defaults of 4 publishers, 4 subscriptions, 4 services, and 2 clients. The
   example uses one publisher and one subscription.
2. **The buffer pool is the second-largest item**, and its small-block counts
   scale with the number of local endpoints.

Declaring what the firmware actually creates fixes both at once (see
[Worked example](#worked-example-rclc_chatter-at-half-the-ram)).

## Object model

```
RTPSParticipant ─────────────────────────────── one per process / firmware
├── SpdpWriter, SpdpReader        participant discovery; SpdpReader holds the
│                                 peer table (RTPS_MAX_PEERS × PeerEntry)
├── SedpWriter, SedpReader        endpoint discovery (ordinary reliable
│                                 StatefulWriter/StatefulReader underneath)
├── EndpointMatchTable            local endpoints with full names; remote
│                                 endpoints as 16-byte records; match pairs
├── writer_storage_[RTPS_MAX_USER_WRITERS]   each slot holds one
│       StatefulWriterStorage<Depth, MaxReaders>
│         ├── HistoryCache of Depth CacheChanges ─► BufRef into the pool
│         └── MaxReaders × ReaderProxy
└── reader_storage_[RTPS_MAX_USER_READERS]   each slot holds one
        BoundedStatefulReader<RTPS_MAX_MATCHED_WRITERS>
          ├── MaxWriters × WriterProxy
          └── FragmentAssembler ─────────────► BufRef into the pool

rtps::g_buf_pool   five size classes of reference-counted blocks; every
                   payload in the system lives in one of these blocks

rmw_rosette        Pool<T, N> per entity kind (context, node, publisher,
                   subscription, service, client, guard condition, wait set);
                   each subscription/service/client owns a ChangeRing of
                   BufRefs waiting for rmw_take()
```

### Endpoints are placement-constructed into fixed slots

`RTPSParticipant::create_writer<Depth>()` placement-constructs a
`StatefulWriterStorage<Depth, MaxReaders>` into a free byte slot and returns a
`StatefulWriter*`; `delete_writer()` runs its destructor, which releases every
payload its history holds, and frees the slot. Readers work the same way with
`create_reader()` and `delete_reader()`. Nothing is ever allocated.

`StatefulWriterStorage<D, R>` is a template only over *storage*: all writer
logic is compiled once in `src/rtps/stateful_writer.cpp` and works on the
history and proxy arrays the storage hands it. A writer of depth 1 and one of
depth 8 differ in layout, never in code, so adding depths costs RAM, not
flash.

**Every writer slot is the same size**: `RTPS_PARTICIPANT_MAX_WRITER_BYTES`,
which defaults to `sizeof(StatefulWriterStorage<8, RTPS_MAX_MATCHED_READERS>)`
(1,680 B at 8 matched readers). A table of uniform slots means any slot can
hold a writer of any depth, so writers can be created and destroyed in any
order without fragmenting the table. The cost is that a depth-1 writer
(984 B) still occupies a 1,680 B slot.

### Per-object costs

Measured `sizeof` on Cortex-M (the RP2040 and RP2350 lay structures out
identically) at the firmware defaults
(`RTPS_MAX_MATCHED_READERS = RTPS_MAX_MATCHED_WRITERS = 8`):

| Object | Size | Scales with |
| --- | ---: | --- |
| `CacheChange` (one history entry) | 100 B | writer depth |
| `ReaderProxy` (one matched remote reader) | 80 B | `RTPS_MAX_MATCHED_READERS` per writer |
| `WriterProxy` (one matched remote writer) | 64 B | `RTPS_MAX_MATCHED_WRITERS` per reader |
| `StatefulWriterStorage<1, 8>` / `<4, 8>` / `<8, 8>` | 984 / 1,280 / 1,680 B | depth × 100 + readers × 80 + ~240 |
| Writer slot | 1,680 B | `RTPS_PARTICIPANT_MAX_WRITER_BYTES` |
| Reader slot (`BoundedStatefulReader<8>`) | 1,600 B | 8 × `WriterProxy` + an 8-slot `FragmentAssembler` (1,024 B) |
| `PeerEntry` (one remote participant) | 128 B | `RTPS_MAX_PEERS` |
| Remote origin (GUID prefix + locators) | 64 B | `RTPS_MAX_REMOTE_ORIGINS` (= peers) |
| Remote endpoint record | 16 B | `RTPS_MAX_DISCOVERED_WRITERS` / `_READERS` |
| Local endpoint record (two 128-octet names + QoS) | 296 B | local endpoints |
| Match record (one writer-reader pair) | 32 B | `RTPS_MAX_MATCH_RECORDS` |
| `ChangeRing` (rmw take history), `PendingChange` 72 B each | 732 B | `RMW_ROSETTE_MAX_PENDING_CHANGES` (10) |
| Pool block header | 12 B | every pool block |

A writer's `ReaderProxy` records each reader's progress as sequence-number
bounds plus a 64-bit window of requested changes, not a row per history slot,
so its size does not grow with depth (DDSI-RTPS 2.5 §8.4.7 describes the
reader-proxy state; Rosette keeps it in this bounded form).

Neither proxy stores locators. Each keeps a pointer to the one locator it
sends to, inside the record that already holds it: the SPDP peer record for a
participant's locators, or the remote origin for locators an endpoint
announced itself. The proxy therefore costs the same however many locators
its peer announced, and a peer that keeps its GUID and moves to a new address
is reached there as soon as its SPDP refresh updates the record.

### Remote endpoints are kept small, and displaced when full

A network can announce far more endpoints than a microcontroller can
record. `EndpointMatchTable` keeps each remote endpoint as a 16-byte record: its
EntityId, QoS, a 32-bit FNV-1a key of the topic name and one of the type name,
and an index into a shared *origin* (GUID prefix plus announced locators). All
endpoints of one participant share its origin. Names are compared exactly
while the announcement is in hand; later comparisons use the keys, and a false
match needs both 32-bit keys of two different endpoints to collide.

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
| Fragment reassembly | one per sample being reassembled | the last fragment arrives, or the writer is unmatched |
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
| Peer table full | a new participant is dropped (logged as `peer table full`) until an entry frees by lease expiry or disposal |

## Knobs

### rmw_rosette firmware (`rclc` applications)

Set these as CMake cache variables (`-D` on the command line, or `set(... CACHE
...)` before Rosette is added). `rmw_rosette/cmake/rmw_rosette_capacity.cmake`
derives every RTPS table and pool size from them, and
`rmw_rosette/include/rmw_rosette/capacity.hpp` `static_assert`s the
relationships, so an inconsistent set fails to compile.

| Knob | Firmware default | What it sizes | Approximate cost per unit |
| --- | ---: | --- | --- |
| `RMW_ROSETTE_MAX_PUBLISHERS` | 4 | writer slot, local endpoint and match records, SEDP history, publisher handle | ≈ 2.6 KB, plus 1.25 KB of small pool blocks when writers outnumber readers |
| `RMW_ROSETTE_MAX_SUBSCRIPTIONS` | 4 | reader slot, records, SEDP history, subscription handle with its `ChangeRing` | ≈ 3.3 KB, plus 1.25 KB of pool blocks when readers outnumber writers |
| `RMW_ROSETTE_MAX_SERVICES` | 4 | one writer **and** one reader, with their records and handle | ≈ 6.2 KB |
| `RMW_ROSETTE_MAX_CLIENTS` | 2 | one writer and one reader, with their records and handle | ≈ 6.2 KB |
| `RMW_ROSETTE_MAX_NODES` | 1 | node pool; multiplies the per-node entities below | |
| `RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE` | 0 | 6 for `rclc_parameter_server`, 0 without | 6 services per node (≈ 37 KB) |
| `RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE` | 0 | `/rosout`, `/parameter_events` | 0 when rcl logging is off, as in the firmware C stack |
| `RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS` | 8 | peer table, remote origins, SEDP proxies per peer | ≈ 0.5 KB (`PeerEntry`, one origin, SEDP reader and writer proxies) |
| `RMW_ROSETTE_MAX_REMOTE_ENDPOINTS` | 64 | remote writer and reader records | 32 B (16 B in each table) |
| `RMW_ROSETTE_MAX_MATCHED_ENDPOINTS` | 8 | readers per writer, writers per reader, match records, reassembly slots | 80 B per writer slot + 192 B per reader slot + 32 B per local endpoint |
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
| `RTPS_PARTICIPANT_MAX_WRITER_BYTES` | `participant.hpp` | slot size; lowering it below `StatefulWriterStorage<8, R>` saves ~700 B per slot but makes `create_writer<8>()` fail to compile, and `rmw_rosette` instantiates depths 1, 4, and 8 (`kSupportedWriterDepths` in `qos.hpp`) |
| `RTPS_MAX_MATCHED_READERS`, `RTPS_MAX_MATCHED_WRITERS` | `stateful_writer.hpp`, `types.hpp` | proxies per endpoint |
| `RTPS_MAX_ASSEMBLY_SLOTS` | `fragment_assembler.hpp` | samples reassembled at once per reader (defaults to matched writers) |
| `RTPS_MAX_PEERS`, `RTPS_MAX_PEER_LOCATORS` | `types.hpp`, `spdp.hpp` | peer table |
| `RTPS_MAX_DISCOVERED_WRITERS/_READERS`, `RTPS_MAX_REMOTE_ORIGINS`, `RTPS_MAX_MATCH_RECORDS` | `endpoint_match_table.hpp` | remote records and match pairs |
| `RTPS_MAX_TOPIC_NAME_LEN` | `endpoint_match_table.hpp` | 128; ROS 2 service topics embed the node name, so shorter limits refuse real names |
| `RTPS_POOL_CLASS<n>_SIZE/_COUNT` | `buf_pool.hpp` | buffer pool layout |

## Worked example: `rclc_chatter` at half the RAM

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
| `.bss` total | 123,720 B | **59,228 B** | 64,492 B |
| rmw context (participant, transport, rx buffer) | 70,424 | 21,496 | 48,928 |
| buffer pool | 42,464 | 30,620 | 11,844 |
| subscription + publisher pools | 4,992 | 1,272 | 3,720 |
| `.text` | 237,796 | 235,860 | 1,936 |

The participant now has two writer slots (the publisher and
`ros_discovery_info`) and one reader slot. Nothing about the protocol changed.

## Sizing a new firmware

1. **Count local entities.** Publishers, subscriptions, services, clients per
   node, plus what the client library adds (parameter services if you start
   `rclc_parameter_server`; `/rosout` if rcl logging is on). Set the
   `RMW_ROSETTE_MAX_*` limits to exactly those counts.
2. **Count the network.** `RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS` must cover
   every ROS 2 process on the domain that the firmware talks to (each process
   is one participant; a peer beyond the table is ignored).
   `RMW_ROSETTE_MAX_REMOTE_ENDPOINTS` needs to cover only the remote endpoints
   your endpoints match, thanks to displacement; more lets
   `rmw_count_publishers()` see more of the graph.
3. **Match fan-out.** `RMW_ROSETTE_MAX_MATCHED_ENDPOINTS` is the most remote
   readers any one of your writers serves, or remote writers any one of your
   readers hears.
4. **Largest message.** `RMW_ROSETTE_MAX_SAMPLE_SIZE` is the largest
   serialized message in either direction (CDR size, including the 4-octet
   encapsulation header). `RMW_ROSETTE_LARGE_BUFFERS` is how many such
   messages can be in flight at once: writer histories awaiting
   acknowledgment, reassembly, and unread subscription entries.
5. **Link.** Keep `RMW_ROSETTE_MAX_DATAGRAM_SIZE` at 1472 on WIZnet hardware
   UDP, which cannot reassemble IP fragments.
6. **Depth.** Ask for the QoS depth you mean. A depth-10 subscription (the
   `rclc` default) pins up to ten samples; depth 1 pins one.
7. **Build and read the numbers** (below). Then run the firmware against the
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
extern const unsigned s_participant = sizeof(rtps::RTPSParticipant);
extern const unsigned s_match       = sizeof(rtps::EndpointMatchTable);
extern const unsigned s_reader      = sizeof(rtps::DefaultStatefulReader);
```

```bash
grep -A1 '^s_' probe.s
```

**Heap**: print `mallinfo().uordblks` periodically, as `rclc_chatter` does.

**Pool pressure**: build with `-DRTPS_BUFPOOL_DEBUG` to get per-class `live`,
`high_water`, and `alloc_fail` counters (`MultiBufPool::debug_class_stats()`)
and an allocation event ring. A class whose high-water mark sits well below
its count can shrink; one with `alloc_fail > 0` is too small.

**Datagrams**: `rtps_chatter` prints the transport counters (largest datagram
sent and received, oversize, truncated); on a host, `RMW_ROSETTE_TRANSPORT_STATS=1`
prints them at shutdown.

## Design decisions

| Decision | Rationale |
| --- | --- |
| No heap in the stack | Bounded, visible RAM; failures at boot; no fragmentation over long uptimes. The same discipline as MISRA C:2012 Rule 21.3 and the JPL/NASA "Power of Ten" rule 3. |
| Capacity from a few application limits, derived in one CMake module | An application states what it creates; the derived tables cannot be sized inconsistently, and `capacity.hpp` asserts it. |
| Templates lay out storage; logic compiles once | `StatefulWriterStorage<D, R>` and `HistoryCacheStorage<N>` hold arrays; `StatefulWriter`, `HistoryCache`, and `ReaderProxy` are plain classes in `.cpp` files. Depth is a RAM decision that costs no flash (the approach of LLVM's `SmallVector` / `SmallVectorImpl`). |
| Uniform writer slots | Any slot takes any depth, so creation and deletion order never fragment the table. |
| Contiguous samples, fragmented only on the wire | Serialization, repair, and reassembly work on one span; the largest pool class is reserved so small traffic cannot starve large samples. |
| Reference-counted blocks | Zero-copy from serialization to the wire and from reassembly to `rmw_take()`. |
| Remote endpoints as keyed 16-byte records with displacement | A board on a network of dozens of participants records only what it matches; a matched record is never displaced, so matching never loses a pair it reported. |
| `ChangeRing` depth equals the requested QoS depth | The RTPS reader has already acknowledged what the ring holds, so the ring is the reader history, with KEEP_LAST semantics (DDS 1.4 §2.2.3.18). |
| One participant lock, no locks inside RTPS objects | Simple lock ordering; interrupt handlers only signal a wake event. |
