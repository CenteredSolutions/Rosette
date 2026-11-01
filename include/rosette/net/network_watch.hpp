// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file net/network_watch.hpp
 * @brief How a session supervisor drives and observes a network backend, whatever the backend.
 *
 * A network backend (the WIZnet board network, and in turn FreeRTOS+TCP or
 * lwIP) exposes three functions: one that services it from the main loop, one
 * that says whether the network meets the firmware's `NetworkConfig::ready_when`,
 * and one that returns its address snapshot. A `NetworkWatch` bundles them so
 * the ROS session supervisor (`rosette/ros/session_supervisor.hpp`) runs the
 * network in every state and rebuilds the session when the address it was
 * created on goes away, without depending on any one backend.
 *
 *     rosette::ros::SessionSupervisor session(net::board_network_watch(), kApp);
 */

#include <cstdint>

#include "address_state.hpp"

namespace rosette::net {

/// A network backend, as seen by the code that runs sessions on it.
struct NetworkWatch {
    /// Services the backend (DHCP, SLAAC, carrier, conflicts); never blocks.
    void (*poll)(uint32_t now_ms);
    /// Whether the backend is up and the firmware's `ready_when` holds.
    bool (*ready)();
    /// The backend's live address snapshot.
    const AddressSnapshot& (*snapshot)();
};

}  // namespace rosette::net
