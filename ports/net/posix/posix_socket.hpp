#pragma once

#include "rosette/net/socket.hpp"

namespace rosette::net::posix {

/**
 * @brief Socket backend over POSIX (BSD) datagram sockets.
 */
class PosixSocketProvider final : public SocketProvider {
public:
    std::unique_ptr<Socket> open(
        SocketDomain domain = SocketDomain::ipv4,
        SocketType type = SocketType::datagram,
        SocketProtocol protocol = SocketProtocol::udp) noexcept override;

    /**
     * @brief Blocks in one poll() on every socket plus `wake`'s pipe.
     *
     * POSIX sockets can be waited on directly, so a received datagram and a
     * signal from another thread end the same wait. A signaled `wake` is
     * consumed here.
     *
     * @param[in] sockets    Sockets from this provider's open(); nulls ignored.
     * @param[in] count      Entries in `sockets` (at most 7 are waited on).
     * @param[in] wake       Event whose signal ends the wait.
     * @param[in] timeout_ms Longest wait; platform::kWaitForever blocks.
     *
     * Verification:
     * - Correct means: returns within milliseconds of a datagram or a signal.
     * - Covered by: rmw_rosette/test/test_guard_condition.cpp (a trigger from
     *   another thread ends a blocked rmw_wait()).
     */
    void wait_readable(Socket* const* sockets, std::size_t count,
                       rosette::platform::Event& wake,
                       uint32_t timeout_ms) noexcept override;
};

}  // namespace rosette::net::posix
