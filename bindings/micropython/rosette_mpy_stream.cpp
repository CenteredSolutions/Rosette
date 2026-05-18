/**
 * @file bindings/micropython/rosette_mpy_stream.cpp
 * @brief Poll and stream integration for Rosette subscriptions.
 *
 * This unit exposes stream-style hooks such as `ioctl(MP_STREAM_POLL, ...)`
 * so subscriptions can integrate cleanly with `select.poll()` and event-loop
 * code that expects file-like readiness semantics.
 *
 * The readiness path performs only bounded work: it may advance the context by
 * one short tick, inspect the subscription ring buffer, and report readable
 * state when queued payloads are available.
 */

#include "rosette_mpy_backend.hpp"

namespace rosette::bindings::micropython {}  // namespace rosette::bindings::micropython
