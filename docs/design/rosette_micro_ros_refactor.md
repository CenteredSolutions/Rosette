# Rosette micro-ROS Refactor

Status: proposed · Branch: `feature/micro_ros` · Date: 2026-09-01

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
- `src/rtps/persistence.cpp` + LittleFS wiring specific to RTPS history

### Kept and unchanged in intent

- `rosette::net` — `SocketProvider` abstraction with POSIX, FreeRTOS+TCP, and
  WIZnet MACRAW backends. Backend-agnostic; this is the load-bearing asset.
- `boards/` — `posix`, `w5100s-evb-pico`, `w5100s-evb-pico2`, `bc-gen2`
- `cmake/` — `boards.cmake`, `pico_sdk.cmake`, `freertos_tcp.cmake`,
  `wiznet.cmake`, `littlefs.cmake`, `micro_ros.cmake`
- `src/net/ntp_sync.cpp` — time-source sync for XRCE sessions

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

### 5.2 `rosette::App` — the lifecycle layer (new)

The single most important addition. Every real micro-ROS deployment needs the
Agent connection state machine; the wrapper should own it so applications never
hand-roll it.

```cpp
rosette::AppOptions opts;
opts.client_key = 0x11223344u;
opts.domain_id  = 0u;
// transport + agent locator resolved from board config by default

rosette::App app(opts);

app.on_setup([](rosette::Node& node) {
    // Called on every (re)connect to the Agent. Build all entities here.
    auto temp = node.create_publisher<std_msgs__msg__Float32>("mcu_temperature_c");
    node.create_subscription<geometry_msgs__msg__Twist>("cmd_vel", on_cmd);
    node.create_action_server<Fibonacci>("fibonacci", on_goal, on_execute);
});

app.on_loop([](rosette::LoopContext& lc) {
    // Called every tick while CONNECTED. Publish, read sensors, etc.
});

app.on_teardown([](rosette::Node&) { /* optional: release app resources */ });

app.spin();   // never returns; or app.spin_once(now_ms) from your own task loop
```

Internally `spin()` runs:

```
WAITING_AGENT   ── ping_agent() ok ──▶ AGENT_AVAILABLE
AGENT_AVAILABLE ── context.start(), on_setup() ok ──▶ CONNECTED
CONNECTED       ── tick(): spin_some + action tick + on_loop
CONNECTED       ── ping_agent() fails N times ──▶ DISCONNECTED
DISCONNECTED    ── on_teardown(), destroy entities, context.fini() ──▶ WAITING_AGENT
```

This is the pattern the micro-ROS reference apps implement by hand with
`RCCHECK`/`EXECUTE_EVERY_N_MS`. `App` makes it declarative.

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

1. **Archive & branch protection.** Tag `archive/rtps-stack` at current `HEAD`.
   Confirm the tag pushes to `origin`.
2. **Excise.** Remove everything in §2 "Cut". Repo must still configure for
   `ROSETTE_BOARD=posix` (nothing to build yet is fine).
3. **Promote & rename.** Move `include/rosette/ros/micro_ros/*` up one level.
   `milkros::micro_ros` → `rosette` across 32 files. `RTPS_*` → `ROSETTE_*`.
4. **CMake restructure** per §6. Green build of `rosette_smoke` for
   `ROSETTE_BOARD=w5100s-evb-pico2 -DROSETTE_ENABLE_MICRO_ROS=ON` and for
   `posix`.
5. **Build `rosette::App`** per §5.2 with the reconnect state machine.
   Host test against a real `micro_ros_agent` (start agent, run app, kill agent,
   restart agent, assert re-link).
6. **`Node` executor auto-sizing** per §5.3.
7. **Examples:** `temperature_publisher`, `fibonacci_action`, `reconnect_demo`.
8. **Docs:** rewrite `README.md` and `GETTING_STARTED.md` around micro-ROS only;
   fold `ros/micro_ros/README.md` content into the main docs.
9. **MicroPython:** re-point `bindings/micropython` at `rosette::` and confirm
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
- **Multi-goal actions.** Deferred; default stays 1. Revisit after §7.7.
- **POSIX discovery is Linux-only upstream.** Already handled by the multicast
  locator + host responder; keep the responder script.
