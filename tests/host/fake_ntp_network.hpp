// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file tests/host/fake_ntp_network.hpp
 * @brief A simulated NTP server, network path, and local tick for SntpClient tests.
 *
 * Time is a single fake tick (`FakeNtpNetwork::tick_us`). The server's
 * clock reads `tick + server_offset_us + drift_ppm * tick`, so a test sets
 * exactly how far and how fast the local tick is off UTC. A request sent at
 * tick T reaches the server after `out_delay_us`; the server holds it
 * `hold_us` and its reply returns after `back_delay_us`. `recv_from`
 * advances the tick to the reply's arrival, or by the whole receive timeout
 * when the server stays silent, as a real blocking socket would.
 *
 * The clock under discipline is `FakeNtpNetwork::clock()`: the same tick
 * plus an offset the client sets, so the tests read the client's result
 * directly.
 */

#include <array>
#include <cstdint>
#include <memory>

#include "rosette/net/sntp_client.hpp"
#include "rosette/net/socket.hpp"

namespace fake {

namespace net = rosette::net;

/// How the server answers the next request. `unresolved` is the socket dropping it before the
/// wire, as a WIZnet socket does while its next hop is resolved: no server sees it.
enum class NtpBehavior : uint8_t { answer, silent, kiss_rate, stale_then_answer, unresolved };

class FakeNtpNetwork final : public net::SocketProvider {
public:
    inline static uint64_t tick_us = 1000000u;
    inline static int64_t clock_offset_us = 0;

    int64_t server_offset_us = 1'790'000'000'000'000;
    double drift_ppm = 0.0;
    uint64_t out_delay_us = 2000u;
    uint64_t back_delay_us = 2000u;
    uint64_t hold_us = 50u;
    NtpBehavior behavior = NtpBehavior::answer;

    int open_sockets = 0;
    int requests = 0;
    net::SocketAddress last_to{};

    /// @brief The server's Unix time, in microseconds, at local tick `tick`.
    int64_t server_time_at(uint64_t tick) const {
        return static_cast<int64_t>(tick) + server_offset_us
               + static_cast<int64_t>(drift_ppm * static_cast<double>(tick) * 1e-6);
    }

    /// @brief The true offset (server time minus tick) at tick `tick`.
    int64_t true_offset_at(uint64_t tick) const {
        return server_time_at(tick) - static_cast<int64_t>(tick);
    }

    static net::SntpClock clock() {
        return net::SntpClock{
            [] { return tick_us; },
            [] { return clock_offset_us; },
            [](int64_t offset) { clock_offset_us = offset; },
        };
    }

    std::unique_ptr<net::Socket> open(net::SocketDomain, net::SocketType,
                                      net::SocketProtocol) noexcept override;

    /// @brief Builds the reply the server sends to `request`, as bytes.
    std::array<uint8_t, 48> reply_to(const uint8_t* request, uint64_t arrival_tick, uint8_t stratum,
                                     bool stale) const {
        std::array<uint8_t, 48> r{};
        r[0] = 0x24u;  // LI 0, VN 4, mode 4.
        r[1] = stratum;
        if (stratum == 0u) {
            r[12] = 'R';
            r[13] = 'A';
            r[14] = 'T';
            r[15] = 'E';
        }

        for (std::size_t i = 0; i < 8; ++i) {
            r[24 + i] = request[40 + i];
        }

        // A stale reply answers an earlier request: another originate timestamp and times a second
        // old.
        const int64_t age_us = stale ? 1000000 : 0;
        if (stale) {
            r[31] ^= 0x01u;
        }

        put(r, 32, net::unix_to_ntp(server_time_at(arrival_tick) - age_us));
        put(r, 40, net::unix_to_ntp(server_time_at(arrival_tick + hold_us) - age_us));
        return r;
    }

private:
    static void put(std::array<uint8_t, 48>& r, std::size_t at, net::NtpTimestamp ts) {
        const uint32_t words[2] = {ts.seconds, ts.fraction};
        for (std::size_t w = 0; w < 2; ++w) {
            for (std::size_t b = 0; b < 4; ++b) {
                r[at + w * 4 + b] = static_cast<uint8_t>(words[w] >> (24 - 8 * b));
            }
        }
    }
};

/// One exchange's socket: holds the reply (or replies) the request produced.
class FakeNtpSocket final : public net::Socket {
public:
    explicit FakeNtpSocket(FakeNtpNetwork& network) : network_(network) { ++network_.open_sockets; }

    ~FakeNtpSocket() override { close(); }

    bool is_open() const noexcept override { return open_; }

    int joinMulticastGroup(const net::SocketMulticastMembership&) noexcept override { return -1; }

    int leaveMulticastGroup(const net::SocketMulticastMembership&) noexcept override { return -1; }

    int bind(const net::SocketAddress&) noexcept override { return 0; }

    int set_receive_timeout(uint32_t timeout_ms) noexcept override {
        timeout_ms_ = timeout_ms;
        return 0;
    }

    std::ptrdiff_t send_to(const void* data, std::size_t length,
                           const net::SocketAddress& remote) noexcept override {
        if (network_.behavior == NtpBehavior::unresolved) {
            error_ = net::SocketError::would_block;
            return static_cast<std::ptrdiff_t>(length);
        }

        error_ = net::SocketError::none;
        ++network_.requests;
        network_.last_to = remote;
        const auto* request = static_cast<const uint8_t*>(data);
        const uint64_t arrival = FakeNtpNetwork::tick_us + network_.out_delay_us;
        const uint64_t back = arrival + network_.hold_us + network_.back_delay_us;
        server_ = remote;
        switch (network_.behavior) {
            case NtpBehavior::answer:
                queue(network_.reply_to(request, arrival, 2u, false), back);
                break;
            case NtpBehavior::kiss_rate:
                queue(network_.reply_to(request, arrival, 0u, false), back);
                break;
            case NtpBehavior::stale_then_answer:
                queue(network_.reply_to(request, arrival, 2u, true), back);
                queue(network_.reply_to(request, arrival, 2u, false), back + 10u);
                break;
            case NtpBehavior::silent:
            case NtpBehavior::unresolved:
                break;
        }

        return static_cast<std::ptrdiff_t>(length);
    }

    std::ptrdiff_t recv_from(void* data, std::size_t capacity,
                             net::SocketAddress* remote) noexcept override {
        if (next_ >= queued_) {
            FakeNtpNetwork::tick_us += uint64_t{timeout_ms_} * 1000u;
            return -1;
        }

        const Datagram& d = datagrams_[next_++];
        if (d.at_us > FakeNtpNetwork::tick_us) {
            FakeNtpNetwork::tick_us = d.at_us;
        }

        const std::size_t n = capacity < d.bytes.size() ? capacity : d.bytes.size();
        for (std::size_t i = 0; i < n; ++i) {
            static_cast<uint8_t*>(data)[i] = d.bytes[i];
        }

        *remote = server_;
        return static_cast<std::ptrdiff_t>(n);
    }

    void close() noexcept override {
        if (open_) {
            open_ = false;
            --network_.open_sockets;
        }
    }

    net::SocketError last_error() const noexcept override { return error_; }

private:
    struct Datagram {
        std::array<uint8_t, 48> bytes{};
        uint64_t at_us = 0u;
    };

    void queue(const std::array<uint8_t, 48>& bytes, uint64_t at_us) {
        datagrams_[queued_++] = Datagram{bytes, at_us};
    }

    FakeNtpNetwork& network_;
    net::SocketAddress server_{};
    std::array<Datagram, 2> datagrams_{};
    std::size_t             queued_ = 0;
    std::size_t             next_ = 0;
    uint32_t timeout_ms_ = 0u;
    bool open_ = true;
    net::SocketError error_ = net::SocketError::none;
};

inline std::unique_ptr<net::Socket> FakeNtpNetwork::open(net::SocketDomain, net::SocketType,
                                                         net::SocketProtocol) noexcept {
    return std::make_unique<FakeNtpSocket>(*this);
}

}  // namespace fake
