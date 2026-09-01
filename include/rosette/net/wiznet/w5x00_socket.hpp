/**
 * @file net/wiznet/w5x00_socket.hpp
 * @brief Poll-mode `SocketProvider` over the WIZnet ioLibrary hardware sockets.
 *
 * The W5100S / W5500 run the whole UDP/IP stack in silicon — a Pico only pushes
 * bytes over SPI. This provider wraps the ioLibrary's synchronous
 * `socket()` / `sendto()` / `recvfrom()` C API and needs no RTOS: `recv_from()`
 * polls the chip's RX register and paces the wait with
 * `rosette_platform_delay_ms()`.
 *
 * It is the transport under `rosette::UdpSocketTransport` for the micro-ROS
 * custom-transport path on WIZnet boards. One `Socket` maps to one hardware
 * socket; the chip has four (W5100S) or eight (W5500).
 *
 * Bring-up (chip init, PHY link, DHCP or static addressing) is
 * `rosette::net::wiznet::WiznetLink` — construct that first.
 */

#pragma once

#include "rosette/net/socket.hpp"
#include "rosette/platform/host_runtime.hpp"

extern "C" {
#include "socket.h"
#include "wizchip_conf.h"
}

#include <array>
#include <cstdint>
#include <memory>
#include <new>

namespace rosette::net::wiznet {

/// Total hardware sockets the fitted chip provides.
#ifndef ROSETTE_WIZNET_SOCKET_COUNT
#define ROSETTE_WIZNET_SOCKET_COUNT 4u
#endif

/**
 * @brief One hardware UDP socket.
 */
class W5x00Socket final : public Socket {
public:
    W5x00Socket(uint8_t sn, bool* claimed_slot) noexcept
        : sn_(sn), claimed_slot_(claimed_slot) {}

    ~W5x00Socket() override { close(); }

    bool is_open() const noexcept override { return open_; }

    int joinMulticastGroup(const SocketMulticastMembership&) noexcept override
    {
        // UDP multicast RX needs Sn_DIPR/Sn_DPORT/Sn_DHAR set before socket();
        // the static-Agent path does not use it. Added with Agent discovery.
        last_error_ = SocketError::not_supported;
        return -1;
    }

    int leaveMulticastGroup(const SocketMulticastMembership&) noexcept override
    {
        last_error_ = SocketError::none;
        return 0;
    }

    /**
     * @brief Opens the hardware socket bound to @p local.port.
     */
    int bind(const SocketAddress& local) noexcept override
    {
        if (local.domain != SocketDomain::ipv4) {
            last_error_ = SocketError::not_supported;
            return -1;
        }
        const int8_t rc = ::socket(sn_, Sn_MR_UDP, local.port, SF_IO_NONBLOCK);
        if (rc != sn_) {
            last_error_ = SocketError::io;
            return -1;
        }
        open_ = true;
        last_error_ = SocketError::none;
        return 0;
    }

    int set_receive_timeout(uint32_t timeout_ms) noexcept override
    {
        receive_timeout_ms_ = timeout_ms;
        last_error_ = SocketError::none;
        return 0;
    }

    std::ptrdiff_t send_to(const void* data, std::size_t length,
                           const SocketAddress& remote) noexcept override
    {
        if (!open_) {
            last_error_ = SocketError::closed;
            return -1;
        }
        if (remote.domain != SocketDomain::ipv4) {
            last_error_ = SocketError::not_supported;
            return -1;
        }
        std::array<uint8_t, 4> addr = remote.ipv4();
        const int32_t rc = ::sendto(sn_,
                                    const_cast<uint8_t*>(static_cast<const uint8_t*>(data)),
                                    static_cast<uint16_t>(length),
                                    addr.data(), remote.port);
        if (rc < 0) {
            last_error_ = SocketError::io;
            return -1;
        }
        last_error_ = SocketError::none;
        return rc;
    }

    /**
     * @brief Reads one datagram, polling until data arrives or the timeout
     * elapses.
     *
     * Non-blocking `recvfrom()` returns 0 when the RX buffer is empty; this
     * loop then sleeps 1 ms and retries until `receive_timeout_ms_` is up.
     */
    std::ptrdiff_t recv_from(void* data, std::size_t capacity,
                             SocketAddress* remote) noexcept override
    {
        if (!open_) {
            last_error_ = SocketError::closed;
            return -1;
        }

        const uint32_t start = rosette_platform_get_ms();
        for (;;) {
            std::array<uint8_t, 4> addr{};
            uint16_t port = 0u;
            const int32_t rc = ::recvfrom(sn_, static_cast<uint8_t*>(data),
                                          static_cast<uint16_t>(capacity),
                                          addr.data(), &port);
            if (rc > 0) {
                if (remote != nullptr) {
                    *remote = make_socket_address_v4(addr, port);
                }
                last_error_ = SocketError::none;
                return rc;
            }
            if (rc < 0) {
                last_error_ = SocketError::io;
                return -1;
            }
            // rc == 0: nothing yet.
            if (receive_timeout_ms_ == 0u ||
                (rosette_platform_get_ms() - start) >= receive_timeout_ms_) {
                last_error_ = receive_timeout_ms_ == 0u ? SocketError::would_block
                                                       : SocketError::timed_out;
                return -1;
            }
            rosette_platform_delay_ms(1u);
        }
    }

    void close() noexcept override
    {
        if (open_) {
            (void)::close(sn_);
            open_ = false;
        }
        if (claimed_slot_ != nullptr) {
            *claimed_slot_ = false;
            claimed_slot_ = nullptr;
        }
        last_error_ = SocketError::none;
    }

    SocketError last_error() const noexcept override { return last_error_; }

private:
    uint8_t sn_;
    bool* claimed_slot_;
    uint32_t receive_timeout_ms_ = 0u;
    SocketError last_error_ = SocketError::none;
    bool open_ = false;
};

/**
 * @brief Hands out the chip's hardware sockets as `Socket` objects.
 */
class W5x00SocketProvider final : public SocketProvider {
public:
    std::unique_ptr<Socket> open(SocketDomain domain = SocketDomain::ipv4,
                                 SocketType type = SocketType::datagram,
                                 SocketProtocol protocol = SocketProtocol::udp) noexcept override
    {
        if (domain != SocketDomain::ipv4 || type != SocketType::datagram ||
            protocol != SocketProtocol::udp) {
            return {};
        }
        for (uint8_t sn = 0; sn < ROSETTE_WIZNET_SOCKET_COUNT; ++sn) {
            if (!claimed_[sn]) {
                claimed_[sn] = true;
                auto sock = std::unique_ptr<Socket>(
                    new (std::nothrow) W5x00Socket(sn, &claimed_[sn]));
                if (!sock) {
                    claimed_[sn] = false;
                }
                return sock;
            }
        }
        return {};
    }

private:
    std::array<bool, ROSETTE_WIZNET_SOCKET_COUNT> claimed_{};
};

}  // namespace rosette::net::wiznet
