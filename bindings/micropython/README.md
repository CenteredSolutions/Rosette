# Rosette MicroPython Binding

This directory defines a MicroPython-facing binding for Rosette's
`milkros::micro_ros` backend. The binding is designed for two deployment
styles:

- a built-in user C module linked into a board firmware image
- a native `.mpy` module that uses the dynamic runtime glue exported by
  MicroPython

The design keeps the MicroPython API compact while reusing Rosette's existing
micro-ROS transport, context, node, publisher, and subscription seams.

## Design Goals

- Preserve Rosette's existing mental model: context, node, publisher,
  subscription.
- Reuse the micro-ROS backend rather than exposing the direct RTPS stack
  first.
- Keep the transport path compatible with MicroPython networking ports that
  already delegate socket operations through a NIC protocol implementation.
- Expose poll-friendly subscription objects so the binding integrates with
  `select.poll()` and `uasyncio` style event loops.
- Represent runtime message schemas as compact descriptors that drive generic
  CDR encoding and decoding, rather than generating C on-device.

## Current Status

The binding currently implements the shortest useful compile-and-smoke path:

- `rosette.Context(...)`
- `Context.start()`
- `Context.tick()`
- `Context.close()`
- `Context.started()`
- `Context.connected()`
- `Context.send_packet(payload)`
- `Context.recv_packet([max_bytes])`
- `Context.create_node(name, namespace="")`
- `rosette.schema(name, msg_text)`
- `Node` object creation and inspection

Publisher, subscription, and runtime CDR codec work are scaffolded but not yet
implemented.

When a prebuilt `libmicroros.a` and generated include tree are available, the
binding can also start a real `milkros::micro_ros::Context` and create a real
ROS node. The host unix-port path prefers stock UDP with a static agent
endpoint because that is the shortest route to a live XRCE session. The same
binding surface can switch to Rosette's custom transport path when the
generated micro-ROS library was built with custom transport enabled.

## Current Python API

The currently implemented API is:

```python
import rosette

ctx = rosette.Context(
    agent_ip="192.168.1.50",
    agent_port=8888,
    local_port=0,
    client_key=0x01020304,
    domain_id=0,
    executor_handles=8,
    spin_timeout_ms=1,
)
ctx.start()

node = ctx.create_node("demo", "/robot")
schema = rosette.schema(
    "std_msgs/String",
    "string data\n",
)

print(ctx)
print(ctx.connected())
print(node)
print(schema)
print(ctx.tick())
ctx.close()
```

## Runtime Responsibilities

### `Context`

`Context` owns:

- the MicroPython UDP socket object
- the custom micro-ROS transport callbacks that send and receive through that
  socket
- the `milkros::micro_ros::Context` backend instance
- child subscription registration so `ioctl(POLL)` can surface message
  readiness

For dynamic native modules, the socket is created by importing `socket` and
calling the module's `socket()` constructor through the MicroPython object
runtime. This keeps the binding at the supported object layer rather than
depending on internal NIC structs.

### `Schema`

The current implementation of `Schema` is a name-carrying stub created by
`rosette.schema(name, msg_text)`. It is enough for import-time smoke testing
and gives the binding a stable Python-facing constructor while the codec work
is still being built.

The intended design is that `Schema` will parse one `.msg` definition into a
compact runtime descriptor. The descriptor will record field names, field
kinds, bounds, and nested-schema links, and the same descriptor will drive:

- conversion from Python dictionaries into CDR payloads
- conversion from received CDR payloads back into Python dictionaries

The first real parser implementation will focus on primitive scalar fields, strings, and
fixed-size arrays so the smoke path can cover `std_msgs/String`,
`std_msgs/Int32`, and similar messages early.

### `Subscription`

`Subscription` owns:

- one micro-ROS subscription handle
- a bounded ring buffer of raw CDR payloads
- a reference to the schema used to decode those payloads into Python objects

The ring buffer stores raw bytes rather than `mp_obj_t` payloads. That keeps
the transport callback path predictable and avoids making the receive hot path
depend on MicroPython object allocation or garbage collection.

`recv()` and the poll integration path are planned next. The ring-buffer design
is already part of the binding object layout so the context and transport code
do not need to be redesigned when subscriptions come online.

## File Map

- [micropython.mk](path/to/Rosette/bindings/micropython/micropython.mk):
  classic user module integration for Make-based MicroPython ports
- [micropython.cmake](path/to/Rosette/bindings/micropython/micropython.cmake):
  CMake-based user module integration
- [rosette_mpy_types.h](path/to/Rosette/bindings/micropython/rosette_mpy_types.h):
  concrete MicroPython object layouts and runtime descriptors
- [rosette_mpy_backend.hpp](path/to/Rosette/bindings/micropython/rosette_mpy_backend.hpp):
  C++ backend entry points that connect the binding to Rosette's micro-ROS
  layer
- [modrosette.c](path/to/Rosette/bindings/micropython/modrosette.c):
  MicroPython module registration surface
- [rosette_mpy_context.cpp](path/to/Rosette/bindings/micropython/rosette_mpy_context.cpp):
  `Context` type implementation
- [rosette_mpy_node.cpp](path/to/Rosette/bindings/micropython/rosette_mpy_node.cpp):
  `Node` type implementation
- [rosette_mpy_pubsub.cpp](path/to/Rosette/bindings/micropython/rosette_mpy_pubsub.cpp):
  `Publisher` and `Subscription` type implementation
- [rosette_mpy_schema.cpp](path/to/Rosette/bindings/micropython/rosette_mpy_schema.cpp):
  runtime `.msg` parser and schema-driven codecs
- [rosette_mpy_transport.cpp](path/to/Rosette/bindings/micropython/rosette_mpy_transport.cpp):
  custom transport callbacks that drive a MicroPython socket object
- [rosette_mpy_stream.cpp](path/to/Rosette/bindings/micropython/rosette_mpy_stream.cpp):
  poll and stream integration for subscriptions

## Build Shape

The binding is defined as a user module so it can be consumed by different
MicroPython ports without changing Rosette's top-level build. A typical board
firmware build points `USER_C_MODULES` or the CMake user-module path at this
directory and provides Rosette's include path plus a prebuilt micro-ROS
library.

The build glue understands three important knobs:

- `ROSETTE_MICRO_ROS_INCLUDE_DIR`: generated `include/` directory from a
  `generate_lib` micro-ROS build
- `ROSETTE_MICRO_ROS_LIBRARY`: path to `libmicroros.a`
- `ROSETTE_MICRO_ROS_TRANSPORT`: `udp` or `custom`

If those paths are not supplied explicitly, the Make and CMake integration
will look for cached values in
`path/to/Rosette/build-micro-ros/CMakeCache.txt`. The real
backend is enabled only when both the include tree and `libmicroros.a` exist.
Otherwise the user module falls back to the compileable backend stub.

The binding source layout keeps transport, schema, and poll integration in
separate files because those responsibilities evolve at different rates:

- transport code depends on port networking behavior
- schema code depends on supported `.msg` surface
- poll integration depends on the Python object experience

Smaller files make it easier to iterate on one of those concerns without
dragging the others into every review.

## Minimal Smoke Build

The shortest validation path is to build the MicroPython unix port with this
directory as a user C module, then run a tiny script that exercises context
construction and teardown. The current official MicroPython user-module docs
describe `USER_C_MODULES` for Make-based ports and show the unix-port pattern:

- docs: [MicroPython external C modules](https://docs.micropython.org/en/latest/develop/cmodules.html)

With a local MicroPython checkout, the shape is:

```bash
cd micropython/ports/unix
make clean
make USER_C_MODULES=/abs/path/to/Rosette/bindings/micropython
./build-standard/micropython /abs/path/to/Rosette/bindings/micropython/smoke_context.py
```

That smoke run is expected to validate:

- the module registers and imports as `rosette`
- `Context` can import `socket` and create a UDP socket
- `Context.start()`, `tick()`, and `close()` succeed on the current backend stub
- `Context.create_node()` and `rosette.schema()` return objects with sane
  representations

The constructor now resolves a UDP-specific address object through
`socket.getaddrinfo(..., AF_INET, SOCK_DGRAM)` and defaults to
`connected_udp=True`. When a local smoke environment rejects the resulting
`connect()` with `OSError(EPERM)`, the binding falls back to an unconnected
socket by default through `connect_fallback_eperm=True`.

## Real micro-ROS Host Build

When a generated micro-ROS host library is available, the unix-port build can
link the real backend and point it at a static micro-ROS agent:

```bash
cd path/to/micropython
make -C ports/unix BUILD=build-rosette \
  USER_C_MODULES=path/to/Rosette/bindings/micropython \
  ROSETTE_MICRO_ROS_INCLUDE_DIR=/abs/path/to/firmware/build/include \
  ROSETTE_MICRO_ROS_LIBRARY=/abs/path/to/firmware/build/libmicroros.a \
  ROSETTE_MICRO_ROS_TRANSPORT=udp
```

That host path is intentionally stock UDP first. It proves the binding can
start a real micro-ROS session and create a node against a live agent before
the board-oriented custom-transport path takes over for non-POSIX ports.
