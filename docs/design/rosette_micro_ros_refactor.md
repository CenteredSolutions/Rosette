# Rosette micro-ROS Refactor

Status: in progress · Branch: `refactor/excise-rtps` (off `feature/micro_ros`) · Date: 2026-09-01

Progress: phases 1–5 complete (5b — the file-management ROS service — pending).
RTPS archived and excised, wrapper promoted to `rosette::`, CMake restructured,
`rosette_smoke` builds against a prebuilt `libmicroros.a`, and the
content-agnostic storage layer (`rosette::storage::Volume` / `FileStore` /
`DeviceConfig`) is built and passing on the host. See §9.

## 1. Motivation

Rosette began as an attempt to bring DDSI-RTPS to a constrained controller board
and then emulate the ROS 2 graph on top of that custom stack. The RTPS work is
sound engineering, but the *emulation layer* is a permanent tax: to stay useful
it has to track `rmw` behaviour, QoS semantics, and the ROS 2 graph protocol
indefinitely, on hardware where full DDS was never meant to run. That is exactly
the problem micro-ROS / Micro XRCE-DDS was created to solve, and it is maintained
by eProsima and Bosch as part of the ROS 2 project.

Decision: **micro-ROS + `micro_ros_agent` becomes the only backend.** The custom
RTPS stack is archived, not deleted.

The microcontroller-first posture does not change. What changes is that the MCU
is an XRCE client that talks to an Agent, and the Agent joins the DDS graph on
its behalf.

## 2. Scope

### Cut from `main` (preserved under tag `archive/rtps-stack`)

- `src/rtps/`, `include/rosette/rtps/` — the DDSI-RTPS stack (~11.8k LOC)
- `include/rosette/ros/*.hpp` — the RTPS-backed ROS facade (`context.hpp`,
  `node.hpp`, `publisher.hpp`, `subscription.hpp`, `service.hpp`, `action.hpp`,
  `timer.hpp`, `discovery.hpp`, `cdr.h`, `topic_names.hpp`, `ros.hpp`)
- `tests/spec/` — RTPS conformance suite and fuzzers
- `tests/ros2/` — POSIX chatter interop smoke
- `examples/main_rp2350.cpp` — RTPS firmware example
- `tools/msg_gen/` — custom `.msg` → CDR C++ codegen (micro-ROS uses
  rosidl-generated C types instead)
- `src/rtps/persistence.cpp` — the RTPS history file-store on LittleFS

### Kept and unchanged in intent

- `rosette::net` — `SocketProvider` abstraction with POSIX, FreeRTOS+TCP, and
  WIZnet MACRAW backends. Backend-agnostic; this is the load-bearing asset.
- `boards/` — `posix`, `w5100s-evb-pico`, `w5100s-evb-pico2`, `bc-gen2`
- `cmake/` — `boards.cmake`, `pico_sdk.cmake`, `freertos_tcp.cmake`,
  `wiznet.cmake`, `littlefs.cmake`, `micro_ros.cmake`
- `src/net/ntp_sync.cpp` — time-source sync for XRCE sessions
- **LittleFS** (`cmake/littlefs.cmake`, `rosette_fs` + `rosette_storage`
  targets) — retained and repurposed from RTPS history storage to a generic
  persistent volume. Rosette provides the volume, an atomic-write file API, a
  content-agnostic keyed blob store, and its own boot-config record; file
  formats are the application's. Survives power cycles and reflashing. See §9.

### Kept and promoted (the new mainline)

- `include/rosette/ros/micro_ros/` — the entire wrapper tree moves up to
  `include/rosette/ros/` and the `milkros::micro_ros` namespace collapses to
  `rosette`.

### Parked

- `bindings/micropython/` — rides on the same wrapper; the namespace rename
  breaks it regardless. Revisit once the C++ surface stabilises.

## 3. Naming cleanup

Done during the same churn so there is one rename, not three:

| Before | After |
| --- | --- |
| `milkros::micro_ros::Foo` | `rosette::Foo` |
| `include/rosette/ros/micro_ros/*.hpp` | `include/rosette/ros/*.hpp` |
| `-DRTPS_BOARD=` | `-DROSETTE_BOARD=` |
| targets `rtps_stack`, `rtps_bsp`, `rtps_example` | `rosette_net`, `rosette_bsp`, `rosette_example` |
| target `rtps_micro_ros_board_smoke` | `rosette_smoke` |
| `RTPS_PROCESSOR`, `RTPS_BOARD_*` cmake vars | `ROSETTE_PROCESSOR`, `ROSETTE_BOARD_*` |

32 source files reference `milkros`; all are in-tree and move together.

## 4. Target architecture

```
  application firmware
        │
  rosette::App                 lifecycle: agent ping → build → spin → loss → rebuild
        │
  rosette::Node                entity factory; auto-computes executor handle count
    ├── Publisher<T>           typed RAII over rcl_publisher_t
    ├── Subscription<T,Cb>     typed RAII, added to shared executor with context
    ├── Timer<Cb>              rcl timer on the shared executor
    ├── Service<T,H> / Client  typed request/response
    └── ActionServer<T,G,E>    cooperative, advanced from App::tick()
        │
  rosette::Context             owns rcl init options, rclc support, executor,
        │                      transport selection, agent resolution
        ├── transport: rmw custom (FreeRTOS/WIZnet) | rmw UDP (POSIX)
        └── agent locator: multicast XRCE GET_INFO/INFO | static endpoint | rmw discovery
        │
  micro_ros_vendor             libmicroros.a from micro_ros_setup generate_lib
        │
  rosette::net::SocketProvider POSIX | FreeRTOS+TCP | WIZnet MACRAW
```

Layers below `Context` already exist and work. The new code is `rosette::App`
and the handle-accounting logic in `Node`.

## 5. API design

### 5.1 Primitives stay class-based

The typed move-only RAII wrappers (`Publisher<T>`, `Subscription<T,Cb>`, …) are
the right primitive and are kept. A purely functional/handle-less API is
rejected: `rclc_executor` needs exact handle accounting and agent reconnect
needs deterministic teardown order, and both fight a handle-less design.

### 5.2 `rosette::App` + `rosette::Module` — the lifecycle layer (new)

The single most important addition. Every real micro-ROS deployment needs the
Agent connection state machine; the wrapper should own it so applications never
hand-roll it. The shape is taken directly from the reference controller's proven
MicroPython `RosService` / `Section` design (see §10), collapsed to Rosette's
single-threaded cooperative model.

`App` owns the session. A **`Module`** is one slice of the firmware's ROS
surface — device, io, motion — that groups the entities and callbacks for a
feature area:

```cpp
class MotionModule : public rosette::Module {
public:
    const char* name() const override { return "motion"; }
    uint32_t period_ms() const override { return 100; }   // 0 => no periodic publish

    void register_(rosette::Node& node) override {        // once, before first connect
        state_ = node.create_publisher<MotorState>("motion/state");
        node.create_service<StopMotor>("motion/stop", [this](auto& rq, auto& rs){ stop(rq, rs); });
        node.create_subscription<MotionTarget>("motion/target", [this](const auto& m){ on_target(m); });
    }
    void on_connect() override { /* reconnect edge: burst current state, clear latches */ }
    void publish() override    { /* one status frame; called while connected */ }
};

rosette::App app({ .namespace_ = rosette::board_scoped_ns("mach", "controller") });
app.add<DeviceModule>();
app.add<IoModule>();
app.add<MotionModule>();
app.spin();   // never returns; or app.spin_once(now_ms) from your own task/loop
```

`spin()` runs one state machine and drives every module cooperatively — no task
or thread per module:

```
WAITING_AGENT   ── ping_agent() ok ──▶ AGENT_AVAILABLE
AGENT_AVAILABLE ── context.start(); Module::register_() for each; on_setup() ──▶ CONNECTED
CONNECTED       ── each tick: executor spin_some + action tick
                             + Module::publish() for each module whose period elapsed
CONNECTED       ── on the connect edge: Module::on_connect() for each
CONNECTED       ── ping_agent() fails N times ──▶ DISCONNECTED
DISCONNECTED    ── Module::on_teardown(); destroy entities; context.fini() ──▶ WAITING_AGENT
```

For the simplest firmwares, `app.on_setup(fn)` / `app.on_loop(fn)` /
`app.on_teardown(fn)` register an anonymous single-module equivalent, so a
one-topic publisher needs no `Module` subclass.

This is the pattern the micro-ROS reference apps implement by hand with
`RCCHECK`/`EXECUTE_EVERY_N_MS`. `App` + `Module` make it declarative.

### 5.3 `Node` computes the executor budget

`ContextOptions::executor_handles` is currently a hand-set integer and the
number-one footgun. `Node` accumulates entities as they are created and sizes
(or resizes, before `spin`) the executor automatically:

```
handles = subscriptions + timers + services + clients
        + (action_servers  * ACTION_SERVER_HANDLES)
        + (action_clients  * ACTION_CLIENT_HANDLES)
```

Applications may still pin a fixed budget for static allocation.

### 5.4 Actions — keep the cooperative model

The existing model is correct and matches `rclc`'s single-threaded executor:

- `on_goal(goal_id, goal) -> ACCEPT | REJECT`
- `on_cancel(goal_handle) -> ACCEPT | REJECT`
- `on_execute(GoalHandle<ActionT>&)` — called from `App::tick()`, advances the
  goal state machine one step, calls `handle.publish_feedback()` /
  `handle.succeed()` / `handle.abort()` / `handle.cancel()`

No thread per goal, no async runtime. Constraints carried forward: one active
goal per server (documented), action entities live as long as the `Context`
because `rclc` has no remove-from-executor for actions.

Refinement: allow a compile-time `MaxConcurrentGoals` on `ActionServer` so
multi-goal servers are possible where the entity budget allows, defaulting to 1.

### 5.5 Message types

Lean fully on the rosidl-generated C types shipped in `libmicroros`.
`rosette::message_traits<T>` slims to:

```cpp
template<> struct rosette::message_traits<std_msgs__msg__Float32> {
    static const rosidl_message_type_support_t* type_support() noexcept;
    // optional, only for types owning sequences/strings:
    static void init(std_msgs__msg__Float32&) noexcept;
    static void fini(std_msgs__msg__Float32&) noexcept;
};
```

Custom interface packages (`.msg` / `.srv` / `.action`) are built into
`libmicroros` by adding them to the `generate_lib` extra-packages input in
`cmake/micro_ros.cmake`. No on-device codegen.

## 6. CMake restructure

```
rosette_net      STATIC   src/net/*        (SocketProvider + backends, NTP)
rosette_bsp      STATIC   boards/<b>/*     (board init, WIZnet MACRAW, pico glue)
micro_ros_vendor INTERFACE                 (libmicroros.a + generated include tree)
rosette_ros      INTERFACE include/rosette/ros/*   (header-only wrappers + App)
rosette_example  EXE      examples/temperature_publisher/
rosette_smoke    EXE      examples/... board-aware CI smoke
```

`ROSETTE_ENABLE_MICRO_ROS` stops being optional — it is the build. The
`micro_ros_setup` bootstrap in `cmake/micro_ros.cmake` stays as-is (transport
auto-selects `custom` for FreeRTOS boards, `udp` for POSIX).

## 7. Phased execution

1. **Archive & branch protection.** ✅ Tag `archive/rtps-stack` pushed to `origin`.
2. **Excise.** ✅ Removed everything in §2 "Cut". `ROSETTE_BOARD=posix` configures
   and builds `rosette_net` clean.
3. **Promote & rename.** ✅ Wrapper headers moved to `include/rosette/ros/`.
   `milkros::micro_ros` → `rosette` (0 residual). `RTPS_*` → `ROSETTE_*` across
   build system and BSP. LittleFS restored as `rosette_fs`.
4. **CMake restructure** per §6. ✅ Targets renamed; `rosette_ros` INTERFACE
   target; `micro_ros.cmake` gained a prebuilt fast path
   (`-DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF` + a workspace dir with a built
   `libmicroros.a`). Non-RTPS host platform shim added
   (`rosette/platform/host_runtime.hpp` + `src/platform/posix/host_runtime.cpp`).
   `rosette_smoke` (new minimal `examples/rosette_smoke.cpp`, stock UDP) builds
   and links on `posix`. Embedded (`w5100s-evb-pico2`) build still needs the
   Pico SDK + arm toolchain — verify in CI or on a dev box with the SDK present.
5. **Persistent storage — content-agnostic (LittleFS).** ✅ See §9.
   - Block device: `rosette::platform::littlefs_mount()` / `littlefs_unmount()`
     for RP2040/RP2350 flash (`src/platform/pico_littlefs.cpp`) over
     `rosette_platform_flash_*` and a per-board reserved region
     (`ROSETTE_STORAGE_FLASH_OFFSET` / `_SIZE`). Host: file-backed volume
     (`src/platform/posix/host_littlefs.cpp`).
   - `rosette::storage::Volume` — RAII mount, atomic whole-file write, dir ops.
   - `rosette::storage::FileStore` — generic keyed blob store (`put/get/list/
     erase`), no content interpretation. This is what per-unit config, whatever
     its format, is stored through.
   - `rosette::storage::DeviceConfig` — Rosette's own `{magic,version,crc32}`
     boot record (net mode + addressing, Agent host/port, client key, domain,
     namespace). `App` reads it (phase 6) so a board is re-addressable without a
     rebuild.
   - New `rosette_storage` target; `examples/storage_demo.cpp` exercises all
     three on the host (passes).
   - **5b:** `rosette_msgs/srv` file-management service (`PutFile` / `GetFile` /
     `ListFiles` / `EraseFile`) over a `FileStore`, plus `colcon.meta` payload
     knobs (`RMW_UXRCE_MAX_INPUT_BUFFER_SIZE`, `_TRANSPORT_MTU`, history) and a
     `libmicroros` regen. Application file formats (e.g. the stepper JSON
     configs, parsed with the app's own JSON reader) stay out of Rosette.
6. **Build `rosette::App`** per §5.2 with the reconnect state machine.
   Host test against a real `micro_ros_agent` (start agent, run app, kill agent,
   restart agent, assert re-link).
7. **`Node` executor auto-sizing** per §5.3.
8. **Examples:** `temperature_publisher`, `fibonacci_action`, `reconnect_demo`,
   `motor_config_store` (Put a motor config over the service, reboot, Get it
   back, confirm the drive picked up the parsed params).
9. **Docs:** rewrite `README.md` and `GETTING_STARTED.md` around micro-ROS only;
   fold `docs/micro_ros_backend.md` content into the main docs.
10. **MicroPython:** re-point `bindings/micropython` at `rosette::` and confirm
    the host smoke path still compiles (separate PR).

## 8. Risks and open questions

- **`rclc` action remove-from-executor.** Still no upstream API. Action entities
  are effectively context-lifetime. Acceptable for MCU use; documented.
- **Custom transport reconnect.** `rmw_uros` custom transport `open`/`close` must
  be idempotent across reconnect cycles. The WIZnet/FreeRTOS `CustomTransport`
  implementations need an audit for socket-reuse leaks.
- **Entity limits are libmicroros build-time.** `RMW_UXRCE_MAX_*` in the
  `colcon.meta` must be kept in sync with what `App`/`Node` will actually
  allocate. Consider generating `colcon.meta` maxes from a single config header.
- **Multi-goal actions.** Deferred; default stays 1. Revisit after the examples
  land (phase 8).
- **`DeviceConfig` schema drift.** The record is versioned (`{magic,version}`)
  but there is no migration path yet — a v1→v2 change on deployed boards falls
  back to defaults. Add a versioned upgrade hook before the field footprint
  grows.
- **POSIX discovery is Linux-only upstream.** Already handled by the multicast
  locator + host responder; keep the responder script.
- **File-management payload size vs. XRCE limits.** An 8 KiB `body` string needs
  `RMW_UXRCE_MAX_INPUT_BUFFER_SIZE` / `_TRANSPORT_MTU` and history depth raised
  in `colcon.meta` (phase 5b), and the device must reject an over-cap blob
  rather than truncate it (`FileStore` already does). Blobs past ~16 KiB want a
  chunked or file-transfer path instead.
- **LittleFS `FetchContent_Populate` deprecation.** `cmake/littlefs.cmake`,
  `pico_sdk.cmake`, and `micro_ros.cmake` all use the deprecated call (CMP0169).
  Modernize the three to `FetchContent_MakeAvailable` in one pass.
- **Storage is now a hard build dependency.** `rosette_fs` / `rosette_storage`
  always build (LittleFS fetched if absent). Add `-DROSETTE_ENABLE_STORAGE=OFF`
  if a consumer genuinely wants it out.
## 9. Persistent storage

Rosette provides a **content-agnostic** storage layer: a mounted volume, a
keyed blob store, and its own client-configuration record. It does not know or
care what file formats an application uses — JSON stepper configs, packed
calibration structs, retained certificates are all just bytes. Parsing and
schema live in the application.

One LittleFS volume sits in a per-board reserved flash region
(`ROSETTE_STORAGE_FLASH_OFFSET` / `_SIZE` in `board_config.h`). On the host the
volume is a backing file so the same code runs under `ROSETTE_BOARD=posix`.

### rosette::storage::Volume  (`storage/volume.hpp`)

RAII mount of the board region — formats a blank region on first boot. Exposes
whole-file read, **atomic** whole-file write (temp file + `lfs_rename`), remove,
exists, `list_dir`, `make_dir`. Every writer above it inherits power-fail
safety from the atomic write.

### rosette::storage::FileStore  (`storage/file_store.hpp`)

A keyed blob store over one directory: `put` / `get` / `list` / `erase` /
`exists`, key → `<dir>/<key><ext>`. Keys are restricted to `[A-Za-z0-9._-]`.
Contents are never inspected; `put` only enforces a configurable size cap. This
is the reusable primitive for "store blobs keyed by a name" — per-unit config
by serial number, calibration, captures. The management ROS service (below)
wraps one `FileStore`.

### rosette::storage::DeviceConfig  (`storage/device_config.hpp`)

The micro-ROS client's own boot configuration — net mode + addressing, Agent
host/port, XRCE client key, domain, node namespace — as one fixed-layout record
at `/device.cfg` behind a `{magic, version, size, crc32}` header. `load()`
returns compiled defaults on any mismatch so a board always boots; `store()`
writes atomically. `App` reads this so a deployed board is re-addressable
without a rebuild.

### Management service (phase 5b)

`rosette_msgs/srv`, served on the normal micro-ROS executor (no extra socket),
operating on a `FileStore` — generic keys and opaque blobs:

| Service | Request | Response |
| --- | --- | --- |
| `PutFile`   | `string key, string body` | `bool ok, string error` |
| `GetFile`   | `string key` | `bool ok, string body` |
| `ListFiles` | — | `string[] keys` |
| `EraseFile` | `string key` | `bool ok` |

`body` is bounded (target 8 KiB). See §8 for the XRCE buffer-sizing knobs a
payload that size needs in `colcon.meta`.

### Open questions

- **Flash-write stall.** A `PutFile` on a live session runs the atomic write
  synchronously; on RP2040 that pauses XIP for tens of ms and the executor with
  it. The service handler should stage the bytes and commit from the tick loop
  where a missed Agent ping is tolerable.
- **First-boot log noise.** A blank volume makes LittleFS print one
  "Corrupted dir pair" line before `littlefs_mount()` formats and remounts.
  Harmless; silence by lowering the LittleFS log level if it bothers a product.

## 10. Reference application — the reference motion controller

The first firmware built on Rosette is the reference controller (a multi-axis stepper
controller around the TMC5160). Its interface package `controller_msgs` and its current
MicroPython implementation (`controller-firmware/modules/ros`, `.../motors`) are the
concrete target Rosette's API is measured against: **porting that firmware to
C++/Rosette should be a mechanical translation, not a redesign.**

### What Rosette must make trivial (from the reference)

- **`RosService` → `rosette::App`.** Owns the session; resolves the Agent
  (discovery with a static fallback); builds the context and node; runs the
  reconnect state machine; keeps module state alive across a hard failure.
- **`Section` → `rosette::Module`.** `name`, `period_ms`, `register_()`,
  `on_connect()`, `publish()`. The reference's per-section `async run()` loop
  becomes App-driven cooperative ticking.
- **Namespace convention.** Board-local entities use *relative* names and land
  under `/<root>/<board-id>/…`; fleet-wide entities (`/mach/faults`) use
  *absolute* names. `board-id` is the chip unique id with a leading-letter
  prefix (a ROS name token cannot start with a digit) — provide
  `rosette::board_id()` and `rosette::board_scoped_ns()`.
- **Fault reporting.** A cross-cutting module publishing `controller_msgs/msg/Fault` on
  an absolute topic; modules call `raise_(source, level, code, msg)` /
  `clear(source)`; active faults re-assert on a timer and on every reconnect.
  The `Fault` message is application-owned; the "re-assert retained state on
  reconnect" behavior is a Rosette pattern worth a small helper.
- **Sync services, async work.** Service callbacks are `(request, response)`
  and must return promptly. Long operations (`ConfigureMotor`, firmware apply)
  return "accepted" and continue on the tick loop; the client polls status.
  Genuinely long, cancellable, progress-reporting operations are **actions**
  (`Home`, `JointMotion`, `Calibrate`, `UpdateFirmware` all are) — the
  cooperative action server (§5.4) is exactly this.
- **Executor budget.** The reference hand-counts `NUM_HANDLES = 24` "with
  headroom". `Node` auto-sizing (§5.3) removes that.
- **IRQ-safe streaming.** Input-edge events set a bitmask + flag in the ISR and
  publish on the loop. Rosette does not need to provide this, but the tick model
  must accommodate "publish outside the executor callback".
- **Config files by name.** `ConfigureMotor.srv` carries a `config_filename`;
  the firmware loads that file from storage and applies it to the driver. This
  is exactly `FileStore::get(key)` + the application's own JSON reader (the
  reference's `MotorConfiguration` maps JSON sections to TMC register writes).
  Rosette stores and serves the bytes; it never parses them.

### What stays out of Rosette

The TMC5160 register model, motion profiles, unit-aware scaling, homing, and
StallGuard tuning (`controller-firmware/modules/motors`) are the *application's*
motor framework. Rosette knows nothing about motors — a `Module` wires the
app's driver objects into ROS entities and that is the whole seam.

### A note on `controller_msgs` nesting limits

`controller_msgs/README.md` warns of "message nesting up to 2 hops" — that is a limit
of the reference's **MicroPython dict codec generator**, not micro-ROS. Rosette
C++ uses the rosidl-generated C types from `libmicroros` directly and has no
such limit; `.msg` files written for Rosette need not contort around it.
