/**
 * @file rtps/transport.hpp
 * @brief RTPS UDP transport abstraction backed by the shared socket module.
 *
 * The RTPS stack only needs a small subset of UDP/IP behavior:
 * open one metatraffic socket, open one user-data socket, send datagrams,
 * and poll for received datagrams. This header keeps the RTPS-facing API
 * stable while routing all socket operations through the backend-agnostic
 * socket layer in `net/socket.hpp`.
 */

#pragma once

#include <cstring>
#include <cstdint>
#include <memory>

#include "buf_pool.hpp"
#include "types.hpp"
#include "rosette/net/socket.hpp"

namespace rtps {

class RTPSParticipant;

struct ITransport {
    virtual ~ITransport() = default;
    virtual bool send_unicast(const BufRef& buf,
                              const Locator& dest) noexcept = 0;
    virtual bool send_multicast(const BufRef& buf,
                                const Locator& dest) noexcept = 0;
    // Local ports used for SPDP/SEDP/user-data locator advertisement.
    // Return 0 to indicate "unknown" and let callers apply defaults.
    virtual uint16_t local_metatraffic_unicast_port() const noexcept { return 0u; }
    virtual uint16_t local_default_unicast_port() const noexcept { return 0u; }
};

struct ITransportRuntime {
    virtual ~ITransportRuntime() = default;
    virtual bool start() noexcept = 0;
    virtual void spin_some(Octet*           rx_buffer,
                           uint32_t         rx_buffer_size,
                           RTPSParticipant& participant) noexcept = 0;
};

/**
 * @brief UDP transport that operates on two backend-provided datagram sockets.
 *
 * `meta_socket_` handles RTPS metatraffic such as SPDP and SEDP, while
 * `user_socket_` handles user-data traffic. The concrete backend is selected
 * by the supplied `SocketProvider`, so the RTPS code no longer needs to know
 * whether packets are traveling through FreeRTOS+TCP or a Wiznet offload path.
 */
class SocketUdpTransport : public ITransport {
public:
    SocketUdpTransport(rosette::net::SocketProvider& provider,
                       uint16_t meta_port,
                       uint16_t user_port,
                       uint32_t multicast_addr) noexcept;
    SocketUdpTransport(rosette::net::SocketProvider& provider,
                       uint16_t meta_port,
                       uint16_t user_port,
                       const Locator& multicast_locator) noexcept;
    ~SocketUdpTransport() noexcept;

    bool open() noexcept;
    void close() noexcept;

    uint32_t poll_meta(Octet*    buf,
                       uint32_t  buf_size,
                       uint32_t& src_addr,
                       uint16_t& src_port) noexcept;

    uint32_t poll_user(Octet*    buf,
                       uint32_t  buf_size,
                       uint32_t& src_addr,
                       uint16_t& src_port) noexcept;

    bool send_unicast(const BufRef& buf,
                      const Locator& dest) noexcept override;
    bool send_multicast(const BufRef& buf,
                        const Locator& dest) noexcept override;
    uint16_t local_metatraffic_unicast_port() const noexcept override { return meta_port_; }
    uint16_t local_default_unicast_port() const noexcept override { return user_port_; }

private:
    rosette::net::Socket* socket_for(const Locator& dest) const noexcept;
    static rosette::net::SocketAddress locator_to_socket_address(
        const Locator& dest) noexcept;
    static rosette::net::SocketMulticastMembership locator_to_multicast_membership(
        const Locator& dest) noexcept;
    rosette::net::SocketAddress any_bind_address(uint16_t port) const noexcept;

    rosette::net::SocketProvider& provider_;
    uint16_t meta_port_;
    uint16_t user_port_;
    Locator multicast_locator_{};
    std::unique_ptr<rosette::net::Socket> meta_socket_;
    std::unique_ptr<rosette::net::Socket> user_socket_;
    bool meta_open_ = false;
    bool user_open_ = false;
};

class SocketTransportRuntime : public ITransportRuntime {
public:
    explicit SocketTransportRuntime(SocketUdpTransport& transport) noexcept
        : transport_(transport) {}

    bool start() noexcept override;
    void spin_some(Octet*           rx_buffer,
                   uint32_t         rx_buffer_size,
                   RTPSParticipant& participant) noexcept override;

private:
    SocketUdpTransport& transport_;
};

#ifdef RTPS_TEST_BUILD

class LoopbackTransport : public ITransport {
public:
    static constexpr uint32_t kSlots = 8;
    static constexpr uint32_t kSlotSize = RTPS_BUF_BLOCK_SIZE;

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override
    {
        return enqueue(buf);
    }

    bool send_multicast(const BufRef& buf, const Locator&) noexcept override
    {
        return enqueue(buf);
    }

    uint32_t poll(Octet* dst, uint32_t cap) noexcept
    {
        if (read_ == write_) {
            return 0;
        }

        const uint32_t len = slot_lens_[read_ % kSlots];
        if (len > cap) {
            return 0;
        }

        memcpy(dst, slots_[read_ % kSlots], len);
        ++read_;
        return len;
    }

    bool empty() const noexcept { return read_ == write_; }

private:
    bool enqueue(const BufRef& buf) noexcept
    {
        if (write_ - read_ >= kSlots) {
            return false;
        }

        const uint32_t n = buf.size < kSlotSize ? buf.size : kSlotSize;
        memcpy(slots_[write_ % kSlots], buf.data, n);
        slot_lens_[write_ % kSlots] = n;
        ++write_;
        return true;
    }

    Octet slots_[kSlots][kSlotSize];
    uint32_t slot_lens_[kSlots] = {};
    uint32_t read_ = 0;
    uint32_t write_ = 0;
};

#endif

}  // namespace rtps
