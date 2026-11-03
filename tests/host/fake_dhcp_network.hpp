// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file tests/host/fake_dhcp_network.hpp
 * @brief A scripted `SocketProvider` and a minimal DHCP server encoder for DHCP client tests.
 *
 * `FakeDhcpNetwork` hands out UDP sockets that record every datagram sent
 * and return datagrams the test queued, never blocking: `recv_from` with an
 * empty queue fails with `would_block`, as a socket with a zero receive
 * timeout does. It also counts how many sockets are open, and fails the test
 * if a socket is read before its receive timeout is set to zero, which is
 * how the tests prove a `poll()` cannot wait.
 *
 * `server_reply()` builds the OFFER / ACK / NAK a server would send, and
 * `SentMessage` decodes what the client sent, so the tests read like an
 * RFC 2131 exchange.
 */

#include <array>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <vector>

#include "rosette/net/socket.hpp"
#include "test_util.hpp"

namespace fake {

namespace net = rosette::net;

/// @brief One datagram the client sent, decoded.
struct SentMessage {
    uint8_t type = 0u;  ///< Option 53.
    uint32_t xid = 0u;
    uint16_t flags = 0u;
    std::array<uint8_t, 4> ciaddr{};
    std::array<uint8_t, 4> requested_ip{};  ///< Option 50, zero when absent.
    std::array<uint8_t, 4> server_id{};     ///< Option 54, zero when absent.
    std::array<uint8_t, 4> to{};
    uint16_t to_port = 0u;
    std::vector<uint8_t> raw;

    bool broadcast_to() const { return to == std::array<uint8_t, 4>{255u, 255u, 255u, 255u}; }
};

inline SentMessage decode_sent(const uint8_t* p, std::size_t n, const net::SocketAddress& to) {
    SentMessage m{};
    m.raw.assign(p, p + n);
    m.xid = (uint32_t{p[4]} << 24) | (uint32_t{p[5]} << 16) | (uint32_t{p[6]} << 8) | p[7];
    m.flags = static_cast<uint16_t>((p[10] << 8) | p[11]);
    std::memcpy(m.ciaddr.data(), p + 12, 4u);
    m.to = to.ipv4();
    m.to_port = to.port;
    for (std::size_t at = 240u; at + 1u < n && p[at] != 255u;) {
        const uint8_t code = p[at];
        const uint8_t len = p[at + 1u];
        const uint8_t* v = p + at + 2u;
        if (code == 53u) {
            m.type = v[0];
        } else if (code == 50u) {
            std::memcpy(m.requested_ip.data(), v, 4u);
        } else if (code == 54u) {
            std::memcpy(m.server_id.data(), v, 4u);
        }

        at += 2u + len;
    }

    return m;
}

/// @brief The fields of a server reply the tests vary.
struct ServerReply {
    uint8_t type = 2u;  ///< OFFER; 5 = ACK, 6 = NAK.
    uint32_t xid = 0u;
    std::array<uint8_t, 6> mac{};
    std::array<uint8_t, 4> yiaddr{192u, 168u, 32u, 79u};
    std::array<uint8_t, 4> server_id{192u, 168u, 32u, 99u};
    std::array<uint8_t, 4> mask{255u, 255u, 255u, 0u};
    std::array<uint8_t, 4> router{192u, 168u, 32u, 1u};
    uint32_t lease_s = 3600u;  ///< 0 omits options 51, 58, 59.
    uint32_t t1_s = 0u;        ///< 0 omits option 58.
    uint32_t t2_s = 0u;        ///< 0 omits option 59.
};

inline void put_be32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v >> 24));
    out.push_back(static_cast<uint8_t>(v >> 16));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v));
}

/// @brief Encodes a BOOTREPLY as a DHCP server sends it (RFC 2131 §2, Table 3).
inline std::vector<uint8_t> server_reply(const ServerReply& r) {
    std::vector<uint8_t> out(240u, 0u);
    out[0] = 2u;
    out[1] = 1u;
    out[2] = 6u;
    out[4] = static_cast<uint8_t>(r.xid >> 24);
    out[5] = static_cast<uint8_t>(r.xid >> 16);
    out[6] = static_cast<uint8_t>(r.xid >> 8);
    out[7] = static_cast<uint8_t>(r.xid);
    std::memcpy(out.data() + 16, r.yiaddr.data(), 4u);
    std::memcpy(out.data() + 28, r.mac.data(), 6u);
    out[236] = 0x63u;
    out[237] = 0x82u;
    out[238] = 0x53u;
    out[239] = 0x63u;
    auto opt4 = [&out](uint8_t code, const std::array<uint8_t, 4>& v) {
        out.insert(out.end(), {code, 4u, v[0], v[1], v[2], v[3]});
    };
    out.insert(out.end(), {53u, 1u, r.type});
    opt4(54u, r.server_id);
    if (r.type != 6u) {
        opt4(1u, r.mask);
        opt4(3u, r.router);
        if (r.lease_s != 0u) {
            out.insert(out.end(), {51u, 4u});
            put_be32(out, r.lease_s);
        }

        if (r.t1_s != 0u) {
            out.insert(out.end(), {58u, 4u});
            put_be32(out, r.t1_s);
        }

        if (r.t2_s != 0u) {
            out.insert(out.end(), {59u, 4u});
            put_be32(out, r.t2_s);
        }
    }

    out.push_back(255u);
    return out;
}

/// @brief The shared state behind every socket the fake provider opens.
struct FakeDhcpNetwork : net::SocketProvider {
    std::vector<SentMessage> sent;
    std::deque<std::vector<uint8_t>> inbound;
    int opens = 0;
    int open_now = 0;
    bool refuse_open = false;

    class Udp : public net::Socket {
    public:
        explicit Udp(FakeDhcpNetwork& n) : n_(n) { ++n_.open_now; }

        ~Udp() override { close(); }

        bool is_open() const noexcept override { return open_; }

        int joinMulticastGroup(const net::SocketMulticastMembership&) noexcept override {
            return 0;
        }

        int leaveMulticastGroup(const net::SocketMulticastMembership&) noexcept override {
            return 0;
        }

        int bind(const net::SocketAddress& local) noexcept override {
            port_ = local.port;
            return 0;
        }

        int set_receive_timeout(uint32_t ms) noexcept override {
            timeout_ms_ = ms;
            return 0;
        }

        std::ptrdiff_t send_to(const void* data, std::size_t length,
                               const net::SocketAddress& remote) noexcept override {
            n_.sent.push_back(decode_sent(static_cast<const uint8_t*>(data), length, remote));
            return static_cast<std::ptrdiff_t>(length);
        }

        std::ptrdiff_t recv_from(void* data, std::size_t capacity,
                                 net::SocketAddress* remote) noexcept override {
            if (timeout_ms_ != 0u) {
                check(false, "a DHCP socket is read only with a zero receive timeout");
            }

            if (n_.inbound.empty()) {
                error_ = net::SocketError::would_block;
                return -1;
            }

            const std::vector<uint8_t> d = n_.inbound.front();
            n_.inbound.pop_front();
            const std::size_t k = d.size() < capacity ? d.size() : capacity;
            std::memcpy(data, d.data(), k);
            if (remote != nullptr) {
                *remote = net::make_socket_address_v4({192u, 168u, 32u, 99u}, 67u);
            }

            return static_cast<std::ptrdiff_t>(d.size());
        }

        void close() noexcept override {
            if (open_) {
                open_ = false;
                --n_.open_now;
            }
        }

        net::SocketError last_error() const noexcept override { return error_; }

    private:
        FakeDhcpNetwork& n_;
        bool open_ = true;
        uint16_t port_ = 0u;
        uint32_t timeout_ms_ = 0xFFFFFFFFu;  // a fresh socket blocks until told otherwise
        net::SocketError error_ = net::SocketError::none;
    };

    std::unique_ptr<net::Socket> open(net::SocketDomain, net::SocketType,
                                      net::SocketProtocol) noexcept override {
        if (refuse_open) {
            return nullptr;
        }

        ++opens;
        return std::make_unique<Udp>(*this);
    }

    const SentMessage& last() const { return sent.back(); }

    /// @brief Queues the server's reply to the client's most recent message.
    void answer(ServerReply reply, const std::array<uint8_t, 6>& mac) {
        reply.xid = last().xid;
        reply.mac = mac;
        inbound.push_back(server_reply(reply));
    }
};

}  // namespace fake
