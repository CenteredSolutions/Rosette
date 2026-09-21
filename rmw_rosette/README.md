# rmw_rosette

An `rmw` implementation over `rosette::rtps` (`include/rosette/rtps`,
`src/rtps` at the repo root) — lets stock `rcl`/`rclc` join the ROS 2 graph
directly as an RTPS participant, no `micro_ros_agent` in the loop.

## Status: core pub/sub path is real and verified

`rmw_init`, `rmw_create_node`, `rmw_create_publisher`, `rmw_create_subscription`,
`rmw_publish`, `rmw_create_wait_set`, `rmw_wait`, and `rmw_take(_with_info)` are
implemented against real UDP sockets (`rtps::SocketUdpTransport`) and real
Micro-CDR-based serialization (`rosidl_typesupport_microxrcedds_c` callbacks) —
not stubs, not a test-only shortcut. This was verified by compiling and
running this code against the *actual* `rmw`/`rcutils`/`rosidl_runtime_c`/
`rosidl_typesupport_microxrcedds_c` headers and sources (cloned from upstream,
pinned to the `humble` branch to match this project's default distro — see
`cmake/micro_ros.cmake`'s `ROSETTE_MICRO_ROS_DISTRO`), driving a hand-written
message type support through the real ABI: `rmw_init` → `rmw_create_node` →
`rmw_create_publisher`/`rmw_create_subscription` → `rmw_publish` → `rmw_wait`
→ `rmw_take`, round-tripping a value end to end. That harness isn't part of
the repo (it depends on a full ROS 2 header/source checkout this project
doesn't vendor), but every fix it drove — corrected struct layouts, the
CDR encapsulation-header accounting below, the writer-depth dispatch — is
here in the committed source.

Not implemented: services/clients (`rmw_service.cpp`/`rmw_client.cpp`),
`rmw_serialize`/`rmw_deserialize` (see the caution comment in
`rmw_serialize.cpp`), graph introspection (`rmw_graph.cpp`), and guard
conditions (accepted by `rmw_wait` but always reported not-ready).

## Known gaps, ranked by size

1. **`rmw_get_node_names`** — needs a `ParticipantEntitiesInfo`-equivalent
   builtin topic; SEDP alone doesn't carry node identity. This is a small
   protocol addition, not just glue, unlike everything else here.
2. **Services/clients** — `rmw_service.cpp`/`rmw_client.cpp` are still
   stubs. The correlation machinery they need (`SampleIdentity`/
   `InlineQos`) already exists in `rtps/types.hpp`; this is wiring, same
   shape as `rmw_publisher.cpp`/`rmw_subscription.cpp`.
3. **`rmw_serialize`/`rmw_deserialize` header asymmetry** — see the
   caution comment at the top of `rmw_serialize.cpp`. `type_support.hpp`'s
   `serialize_message()`/`deserialize_message()` work at the field level
   (no encapsulation header — the writer adds that); a standalone
   `rmw_serialize()` has no writer downstream to do that for it and needs
   to prepend the header itself.
4. **Graph introspection / guard conditions** — `rmw_graph.cpp`'s
   `GraphCache` has no feed yet (see that header's open question about
   where discovery events hook in); guard conditions have no producer.
5. **Multi-backend transport selection** — `rmw_init.cpp` hardcodes
   `rtps::SocketUdpTransport` over `rosette::net::posix::PosixSocketProvider`.
   Real board targets (FreeRTOS+TCP, WIZnet) need this to come from board
   config the way `rosette::App` already resolves it for the micro-ROS
   path — same seam, not yet threaded through to `rmw_init()`.

Resolved since the initial scaffold:

- **Writer history depth.** `rtps::IStatefulWriter` now declares
  `write_reliable()`/`write_async()` directly (their signatures never
  depended on `CacheDepth`, only internal storage did), so
  `qos.hpp::create_writer_for_qos()` picks the smallest of a small closed
  set of depths (`kSupportedWriterDepths`, capped at 8 — see that
  constant's comment for why) at runtime and hands back the type-erased
  interface. Proven working end-to-end in both
  `tests/loopback/rtps_loopback_smoke.cpp` and the rmw-level verification
  above.
- **`rosidl_typesupport_microxrcedds_c` struct layout.** Confirmed against
  the real, `humble`-pinned header — `type_support.hpp` is no longer
  written from memory.
- **Change buffering** between `StatefulReader::OnChangeAvailable` and
  `rmw_take()` — `Subscription::push_change()`/`pop_change()` (types.hpp)
  is a fixed-capacity ring (`RMW_ROSETTE_MAX_PENDING_CHANGES`), no heap.
- **CDR encapsulation-header double-accounting.** `MessageBuilder::append_data()`
  always prepends its own 4-byte header to whatever payload bytes it's
  given (confirmed by `tests/spec`'s `TC-BLD-10.2-001`), so every
  `CacheChange::payload` this stack ever delivers carries one.
  `deserialize_message()` skips it (`kRtpsEncapsulationHeaderSize` in
  `type_support.hpp`); `serialize_message()` correctly does *not* add a
  second one, since the writer already will.
- **No heap for rmw entity handles.** Every `rmw_create_*()` draws from a
  fixed `Pool<T,N>` (`pool.hpp`), sized by `RMW_ROSETTE_MAX_*` macros —
  the same posture `RTPSParticipant` itself already takes for its
  writer/reader tables. `Context` places `RTPSParticipant`/
  `SocketUdpTransport` via placement-new into aligned static storage for
  the same reason (neither type is default-constructible, so they can't
  just be `Pool<T,N>` elements directly).
- **No `std::chrono`.** `rmw_wait()`'s deadline math and poll delay go
  through `rtps_platform_get_ms()` and `rosette_platform_delay_ms()`
  (`rosette/platform/host_runtime.hpp`) — the same clock/delay
  abstraction the rest of Rosette already uses, not a hosted-runtime
  assumption this stack doesn't need.

## Proof the underlying rtps core works: `tests/loopback/`

`tests/loopback/rtps_loopback_smoke.cpp` is a from-scratch smoke test, independent
of this package: two in-process `RTPSParticipant`s bridged directly to each
other's `parse_received()` (no sockets, no ROS 2, no rosidl) that completes
SPDP discovery, SEDP matching, and a reliable `DATA` delivery. It passes today,
and is wired into CMake/CTest (`tests/loopback/CMakeLists.txt`):

```
cmake -S tests/loopback -B /tmp/build && cmake --build /tmp/build && ctest --test-dir /tmp/build
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
