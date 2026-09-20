# rmw_rosette

An `rmw` implementation over `rosette::rtps` (`include/rosette/rtps`,
`src/rtps` at the repo root) — lets stock `rcl`/`rclc` join the ROS 2 graph
directly as an RTPS participant, no `micro_ros_agent` in the loop.

## Status: scaffold, not functional

Every `rmw_*` entry point in `src/` is a stub returning
`RMW_RET_UNSUPPORTED` except where noted. Nothing here builds against real
ROS 2 headers yet — this lays out the file structure and the specific
`rtps::` call each function will eventually make, so the next pass is
"fill in the TODO" rather than "figure out the shape."

## Known gaps, ranked by size

1. **Transport wiring in `rmw_init`** — `RTPSParticipant` takes an
   `ITransport&`; which concrete `rosette::net` backend to construct and
   how domain/board config reaches `rmw_init()` isn't decided.
2. **`rosidl_typesupport_microxrcedds_c` struct layout** — `type_support.hpp`
   is written from memory, not the vendored header. Confirm before writing
   real code against it.
3. **Change buffering between `StatefulReader::OnChangeAvailable` and
   `rmw_take()`** — the callback fires from `tick()`/`parse_received()`
   context; something has to queue between that and the poll-driven
   `rmw_wait()`/`rmw_take()` calls (see `types.hpp`, `wait_set.hpp`).
4. **`rmw_get_node_names`** — needs a `ParticipantEntitiesInfo`-equivalent
   builtin topic; SEDP alone doesn't carry node identity. This is a small
   protocol addition, not just glue, unlike everything else here.

Resolved since the initial scaffold: writer history depth. `rtps::IStatefulWriter`
now declares `write_reliable()`/`write_async()` directly (their signatures never
depended on `CacheDepth`, only internal storage did), so `qos.hpp::create_writer_for_qos()`
picks the smallest of a small closed set of depths (`kSupportedWriterDepths`) at
runtime and hands back the type-erased interface — no more one fixed depth for
every publisher. Proven working end-to-end (not just compiling) in
`tests/loopback/rtps_loopback_smoke.cpp`, which publishes through exactly this
`IStatefulWriter*` path.

## Proof the underlying rtps core works: `tests/loopback/`

`tests/loopback/rtps_loopback_smoke.cpp` is a from-scratch smoke test, independent
of this package: two in-process `RTPSParticipant`s bridged directly to each
other's `parse_received()` (no sockets, no ROS 2, no rosidl) that completes
SPDP discovery, SEDP matching, and a reliable `DATA` delivery. It passes today:

```
clang++ -std=c++17 -DRTPS_NO_FREERTOS -DRTPS_LOG_LEVEL=0 \
  -I. -Iinclude -Iinclude/rosette/rtps \
  tests/loopback/rtps_loopback_smoke.cpp \
  src/rtps/{buf_pool,message_parser,participant,sedp,spdp,stateful_reader}.cpp \
  -o /tmp/rtps_loopback_smoke && /tmp/rtps_loopback_smoke
```

This is the layer `rmw_rosette` sits on top of — if this test ever regresses,
nothing built on top of it can work either.

## Build integration (not yet wired)

This needs to be added as an extra source repo alongside `rcl`/`rclc` in
whatever `.repos` file drives the `micro_ros_setup` firmware workspace,
with that build's `RMW_IMPLEMENTATION` pinned to `rmw_rosette` instead of
`rmw_microxrcedds_c`. The `ament_index_register_resource(...)` call in
`CMakeLists.txt` is what makes the implementation name resolvable — verify
it against the `rmw_implementation_cmake` package's current macro, which
may have moved between ROS 2 distros.
