/**
 * @file src/ros/agent_rmw_link.cpp
 * @brief `rmw_agent_link()`: the agent gate's operations over rmw_microxrcedds.
 *
 * The XRCE client is C and calls its custom transport through four function
 * pointers plus an opaque `args`. The trampolines here recover the
 * `CustomTransport` from `args` and forward each call, so any Rosette
 * transport object can carry the XRCE session. `attach` registers them as
 * rmw_microxrcedds's process-wide default transport, which a ping with no
 * session uses directly and `rmw_init_options_init()` copies into every
 * session's options.
 */

#include "rosette/ros/agent_session_gate.hpp"

#include <rcutils/error_handling.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"

namespace rosette::ros {
namespace {

/// @brief The Rosette transport an XRCE custom-transport handle carries in `args`.
CustomTransport* transport_of(uxrCustomTransport* handle) noexcept {
    return handle != nullptr ? static_cast<CustomTransport*>(handle->args) : nullptr;
}

/// @brief XRCE open callback: opens the Rosette transport's socket.
bool open_transport(uxrCustomTransport* handle) {
    CustomTransport* transport = transport_of(handle);
    return transport != nullptr && transport->open();
}

/// @brief XRCE close callback: closes the Rosette transport's socket.
bool close_transport(uxrCustomTransport* handle) {
    CustomTransport* transport = transport_of(handle);
    return transport != nullptr && transport->close();
}

/// @brief XRCE write callback: sends one XRCE message as one datagram; returns the bytes sent.
size_t write_transport(uxrCustomTransport* handle, const uint8_t* data, size_t length,
                       uint8_t* error) {
    CustomTransport* transport = transport_of(handle);
    return transport != nullptr ? transport->write(data, length, error) : 0u;
}

/// @brief XRCE read callback: waits up to `timeout_ms` for one datagram; returns its length.
size_t read_transport(uxrCustomTransport* handle, uint8_t* data, size_t length, int timeout_ms,
                      uint8_t* error) {
    CustomTransport* transport = transport_of(handle);
    return transport != nullptr ? transport->read(data, length, timeout_ms, error) : 0u;
}

bool attach(CustomTransport& transport) {
    // Unframed: each XRCE message travels as one UDP datagram (framing is for byte streams).
    return rmw_uros_set_custom_transport(false, &transport, open_transport, close_transport,
                                         write_transport, read_transport)
           == RMW_RET_OK;
}

bool ping(int timeout_ms) {
    return rmw_uros_ping_agent(timeout_ms, 1u) == RMW_RET_OK;
}

/**
 * @brief One `rmw_uros_sync_session()` exchange; on success, sets platform::WallClock from it.
 *
 * rmw_microxrcedds keeps the measured offset inside the XRCE session, where
 * it is lost when the session is destroyed. The agent's epoch is read back
 * at once and stored as WallClock's offset over the monotonic tick, where it
 * outlives the session and is what ros/time_sync.hpp, RTPS source
 * timestamps, and rcl's system clock all read.
 *
 * @param[in] timeout_ms Reply timeout of the exchange.
 * @return True when the agent answered and WallClock was set; false (with no
 *   rcl or rmw error state left behind) otherwise.
 *
 * Verification:
 * - Correct means: after a true return, WallClock reads the agent host's
 *   clock to within the exchange's round trip, and keeps advancing with the
 *   tick after the session is destroyed.
 * - Covered by: tests/agent/run_agent_live.sh (TC-ROS-AGT-LIVE-001,
 *   TC-ROS-AGT-LIVE-002: synchronized from a deliberately wrong clock, and
 *   still synchronized while the agent is gone).
 *
 * References:
 * - micro-ROS rmw_microxrcedds `rmw_microros/time_sync.h` (Humble)
 * - eProsima Micro XRCE-DDS client `uxr_sync_session()`
 */
bool sync_time(int timeout_ms) {
    if (rmw_uros_sync_session(timeout_ms) != RMW_RET_OK) {
        // Leave no error state behind to be misreported by the application's next rcl call.
        rcutils_reset_error();
        return false;
    }

    const int64_t epoch_us = rmw_uros_epoch_nanos() / 1000;
    platform::WallClock::set_offset_us(epoch_us - static_cast<int64_t>(platform::monotonic_us()));
    return true;
}

}  // namespace

AgentLink rmw_agent_link() noexcept {
    return AgentLink{attach, ping, sync_time};
}

}  // namespace rosette::ros
