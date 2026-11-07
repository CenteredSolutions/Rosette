// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "rosette/net/socket.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::net::freertos {

/**
 * @brief UDP sockets on FreeRTOS+TCP.
 *
 * Receive wake-up: FreeRTOS+TCP gives a socket's user semaphore
 * (FREERTOS_SO_SET_SEMAPHORE) right after it queues a datagram for it, and a
 * FreeRTOS platform::Event is a binary semaphore. wait_readable() therefore
 * points each socket it is given at the caller's event before it sleeps.
 *
 * Families: the stack finds a UDP socket by port number alone, so one socket
 * per port carries IPv4 and IPv6 alike (dual_stack_sockets()).
 */
class FreeRtosSocketProvider final : public SocketProvider {
public:
    std::unique_ptr<Socket> open(SocketDomain   domain = SocketDomain::ipv4,
                                 SocketType     type = SocketType::datagram,
                                 SocketProtocol protocol = SocketProtocol::udp) noexcept override;

    /// @brief True: FreeRTOS+TCP binds a UDP port for both families at once.
    bool dual_stack_sockets() const noexcept override { return true; }

    /**
     * @brief Sleeps until a datagram is queued on one of @p sockets, @p wake is signaled, or the
     * timeout.
     *
     * Each socket's user semaphore becomes @p wake's. A socket bound to a
     * different event before is checked once for a datagram that arrived
     * while it pointed elsewhere (a non-consuming peek), and the wait is
     * skipped when one is waiting.
     *
     * @param[in] sockets    Sockets this provider opened; may contain nulls.
     * @param[in] count      Entries in @p sockets.
     * @param[in] wake       The caller's event; also signaled by other threads.
     * @param[in] timeout_ms Longest wait; platform::kWaitForever blocks.
     *
     * Verification:
     * - Correct means: an echo round trip on the MIMXRT1020-EVK takes far
     *   less than the caller's timeout, so the wait ended on the datagram.
     * - Covered by: examples/udp_echo with tools/udp_echo_probe.py.
     *
     * References: FreeRTOS+TCP FreeRTOS_setsockopt() FREERTOS_SO_SET_SEMAPHORE
     * (ipconfigSOCKET_HAS_USER_SEMAPHORE); FreeRTOS_recvfrom() FREERTOS_MSG_PEEK.
     */
    void wait_readable(Socket* const* sockets, std::size_t count, rosette::platform::Event& wake,
                       uint32_t timeout_ms) noexcept override;
};

}  // namespace rosette::net::freertos
