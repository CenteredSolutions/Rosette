/**
 * @file ports/net/wiznet/dhcp_client.cpp
 * @brief The non-blocking DHCPv4 client state machine; see dhcp_client.hpp.
 */

#include "dhcp_client.hpp"

#include "rosette/support/fnv1a.hpp"

namespace rosette::net::wiznet {
namespace {

constexpr uint32_t kFirstRetransmitMs = 2000u;
constexpr uint32_t kMaxRetransmitMs = 64000u;  // RFC 2131 §4.1
constexpr uint32_t kMinAttemptGapMs = 60000u;  // RFC 2131 §4.4.5
constexpr uint32_t kDeclineWaitMs = 10000u;    // RFC 2131 §3.1 step 5
constexpr std::size_t kMaxRepliesPerPoll = 8u;
constexpr std::array<uint8_t, 4> kBroadcast{255u, 255u, 255u, 255u};

bool due(uint32_t deadline_ms, uint32_t now_ms) noexcept {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

bool is_zero(const std::array<uint8_t, 4>& a) noexcept {
    return (a[0] | a[1] | a[2] | a[3]) == 0u;
}

uint32_t timer_ms(uint32_t seconds) noexcept {
    return (seconds < DhcpClient::kMaxTimerS ? seconds : DhcpClient::kMaxTimerS) * 1000u;
}

}  // namespace

DhcpClient::DhcpClient(SocketProvider& sockets, const std::array<uint8_t, 6>& mac,
                       const DhcpClientConfig& config) noexcept
    : sockets_(sockets), mac_(mac), config_(config) {}

void DhcpClient::start() noexcept {
    stop();
    state_ = DhcpClientState::init_wait;
    pending_start_ = true;
}

void DhcpClient::stop() noexcept {
    close_window();
    state_ = DhcpClientState::idle;
    lease_ = DhcpLease{};
    offer_ = DhcpLease{};
    pending_start_ = false;
}

DhcpEvent DhcpClient::poll(uint32_t now_ms) noexcept {
    if (state_ == DhcpClientState::idle) {
        return DhcpEvent::none;
    }

    if (pending_start_) {
        pending_start_ = false;
        attempt_ms_ = now_ms;
    }

    const DhcpEvent event = receive(now_ms);
    if (event != DhcpEvent::none) {
        return event;
    }

    return on_timers(now_ms);
}

void DhcpClient::decline(uint32_t now_ms) noexcept {
    if (!has_lease()) {
        return;
    }

    close_window();
    std::unique_ptr<Socket> socket = sockets_.open(SocketDomain::ipv4);
    if (socket != nullptr && socket->bind(make_socket_address_v4({}, config_.client_port)) == 0) {
        DhcpClientMessage message{};
        message.type = DhcpMessageType::decline;
        message.xid = next_xid(now_ms);
        message.mac = mac_;
        message.requested_ip = lease_.ip;
        message.server_id = lease_.server_id;
        std::array<uint8_t, kDhcpMaxMessageBytes> packet{};
        const std::size_t length = encode_dhcp_message(message, packet);
        (void)socket->send_to(packet.data(), length,
                              make_socket_address_v4(kBroadcast, config_.server_port));
    }

    if (socket != nullptr) {
        socket->close();
    }

    lease_ = DhcpLease{};
    state_ = DhcpClientState::init_wait;
    attempt_ms_ =
        now_ms
        + (config_.retry_delay_ms > kDeclineWaitMs ? config_.retry_delay_ms : kDeclineWaitMs);
}

bool DhcpClient::open_window(uint32_t now_ms) noexcept {
    close_window();
    socket_ = sockets_.open(SocketDomain::ipv4);
    if (socket_ == nullptr || socket_->bind(make_socket_address_v4({}, config_.client_port)) != 0
        || socket_->set_receive_timeout(0u) != 0) {
        close_window();
        return false;
    }

    window_end_ms_ = now_ms + config_.exchange_timeout_ms;
    retransmit_gap_ms_ = kFirstRetransmitMs;
    return true;
}

void DhcpClient::close_window() noexcept {
    if (socket_ != nullptr) {
        socket_->close();
        socket_.reset();
    }
}

void DhcpClient::transmit(uint32_t now_ms) noexcept {
    DhcpClientMessage message{};
    message.xid = xid_;
    message.mac = mac_;
    message.host_name = config_.host_name;
    std::array<uint8_t, 4> to = kBroadcast;
    switch (state_) {
        case DhcpClientState::selecting:
            message.type = DhcpMessageType::discover;
            break;
        case DhcpClientState::requesting:
            message.type = DhcpMessageType::request;
            message.requested_ip = offer_.ip;
            message.server_id = offer_.server_id;
            break;
        case DhcpClientState::renewing:
            message.type = DhcpMessageType::request;
            message.ciaddr = lease_.ip;
            if (!is_zero(lease_.server_id)) {
                to = lease_.server_id;
            }

            break;
        case DhcpClientState::rebinding:
            message.type = DhcpMessageType::request;
            message.ciaddr = lease_.ip;
            break;
        default:
            return;
    }

    std::array<uint8_t, kDhcpMaxMessageBytes> packet{};
    const std::size_t length = encode_dhcp_message(message, packet);
    (void)socket_->send_to(packet.data(), length, make_socket_address_v4(to, config_.server_port));
    retransmit_ms_ = now_ms + retransmit_gap_ms_;
    retransmit_gap_ms_ =
        retransmit_gap_ms_ < kMaxRetransmitMs / 2u ? retransmit_gap_ms_ * 2u : kMaxRetransmitMs;
}

DhcpEvent DhcpClient::receive(uint32_t now_ms) noexcept {
    std::array<uint8_t, kDhcpMaxMessageBytes> packet{};
    for (std::size_t i = 0u; i < kMaxRepliesPerPoll && socket_ != nullptr; ++i) {
        SocketAddress from{};
        const std::ptrdiff_t received = socket_->recv_from(packet.data(), packet.size(), &from);
        if (received <= 0) {
            break;
        }

        const std::size_t length = static_cast<std::size_t>(received) < packet.size()
                                       ? static_cast<std::size_t>(received)
                                       : packet.size();
        DhcpServerMessage reply{};
        if (!decode_dhcp_message(packet.data(), length, xid_, mac_, reply)) {
            continue;
        }

        const DhcpEvent event = on_reply(reply, now_ms);
        if (event != DhcpEvent::none) {
            return event;
        }
    }

    return DhcpEvent::none;
}

DhcpEvent DhcpClient::on_reply(const DhcpServerMessage& reply, uint32_t now_ms) noexcept {
    DhcpLease lease = reply.lease;
    switch (state_) {
        case DhcpClientState::selecting:
            if (reply.type == DhcpMessageType::offer && !is_zero(lease.ip)) {
                offer_ = lease;
                state_ = DhcpClientState::requesting;
                requested_ms_ = now_ms;
                retransmit_gap_ms_ = kFirstRetransmitMs;
                transmit(now_ms);
            }

            return DhcpEvent::none;
        case DhcpClientState::requesting:
            if (!is_zero(lease.server_id) && lease.server_id != offer_.server_id) {
                return DhcpEvent::none;  // a reply to another server's offer (RFC 2131 §4.4.1)
            }

            if (reply.type == DhcpMessageType::ack) {
                lease.server_id = offer_.server_id;
                close_window();
                enter_bound(lease, now_ms);
                return DhcpEvent::bound;
            }

            if (reply.type == DhcpMessageType::nak) {
                close_window();
                state_ = DhcpClientState::init_wait;
                attempt_ms_ = now_ms + config_.retry_delay_ms;
                return DhcpEvent::failed;
            }

            return DhcpEvent::none;
        case DhcpClientState::renewing:
        case DhcpClientState::rebinding:
            if (reply.type == DhcpMessageType::ack) {
                if (is_zero(lease.server_id)) {
                    lease.server_id = lease_.server_id;
                }

                const bool changed = !lease.same_config(lease_);
                close_window();
                enter_bound(lease, now_ms);
                return changed ? DhcpEvent::bound : DhcpEvent::renewed;
            }

            return reply.type == DhcpMessageType::nak ? lose_lease(now_ms) : DhcpEvent::none;
        default:
            return DhcpEvent::none;
    }
}

DhcpEvent DhcpClient::on_timers(uint32_t now_ms) noexcept {
    switch (state_) {
        case DhcpClientState::init_wait:
            return due(attempt_ms_, now_ms) ? begin_init(now_ms) : DhcpEvent::none;
        case DhcpClientState::selecting:
        case DhcpClientState::requesting:
            if (due(window_end_ms_, now_ms)) {
                close_window();
                state_ = DhcpClientState::init_wait;
                attempt_ms_ = now_ms + config_.retry_delay_ms;
                return DhcpEvent::failed;
            }

            if (due(retransmit_ms_, now_ms)) {
                transmit(now_ms);
            }

            return DhcpEvent::none;
        case DhcpClientState::bound:
            if (!timed_ || !due(t1_ms_, now_ms)) {
                return DhcpEvent::none;
            }

            state_ = DhcpClientState::renewing;
            attempt_ms_ = now_ms;
            break;
        default:
            break;
    }

    if (due(expiry_ms_, now_ms)) {
        return lose_lease(now_ms);
    }

    if (state_ == DhcpClientState::renewing && due(t2_ms_, now_ms)) {
        close_window();
        state_ = DhcpClientState::rebinding;
        attempt_ms_ = now_ms;
    }

    const uint32_t until_ms = state_ == DhcpClientState::renewing ? t2_ms_ : expiry_ms_;
    if (socket_ != nullptr) {
        if (due(window_end_ms_, now_ms)) {
            close_window();
            schedule_attempt(until_ms, now_ms);
        } else if (due(retransmit_ms_, now_ms)) {
            transmit(now_ms);
        }
    } else if (due(attempt_ms_, now_ms)) {
        xid_ = next_xid(now_ms);
        requested_ms_ = now_ms;
        if (open_window(now_ms)) {
            transmit(now_ms);
        } else {
            schedule_attempt(until_ms, now_ms);
        }
    }

    return DhcpEvent::none;
}

DhcpEvent DhcpClient::begin_init(uint32_t now_ms) noexcept {
    state_ = DhcpClientState::selecting;
    offer_ = DhcpLease{};
    xid_ = next_xid(now_ms);
    if (!open_window(now_ms)) {
        state_ = DhcpClientState::init_wait;
        attempt_ms_ = now_ms + config_.retry_delay_ms;
        return DhcpEvent::failed;
    }

    transmit(now_ms);
    return DhcpEvent::none;
}

DhcpEvent DhcpClient::lose_lease(uint32_t now_ms) noexcept {
    close_window();
    lease_ = DhcpLease{};
    (void)begin_init(now_ms);
    return DhcpEvent::lost;
}

void DhcpClient::enter_bound(const DhcpLease& lease, uint32_t /*now_ms*/) noexcept {
    lease_ = lease;
    state_ = DhcpClientState::bound;
    timed_ = lease.lease_s != kDhcpInfiniteLease;
    if (!timed_) {
        return;
    }

    // RFC 2131 §4.4.1: times count from when the REQUEST was sent, not from the ACK.
    expiry_ms_ = requested_ms_ + timer_ms(lease.lease_s);
    const uint32_t t2 = timer_ms(lease.t2_s < lease.lease_s ? lease.t2_s : lease.lease_s);
    const uint32_t t1 = timer_ms(lease.t1_s < lease.t2_s ? lease.t1_s : lease.t2_s);
    t2_ms_ = requested_ms_ + t2;
    t1_ms_ = requested_ms_ + t1;
}

void DhcpClient::schedule_attempt(uint32_t until_ms, uint32_t now_ms) noexcept {
    const uint32_t remaining = due(until_ms, now_ms) ? 0u : until_ms - now_ms;
    attempt_ms_ = now_ms + (remaining / 2u > kMinAttemptGapMs ? remaining / 2u : kMinAttemptGapMs);
}

uint32_t DhcpClient::next_xid(uint32_t now_ms) noexcept {
    ++exchanges_;
    const uint32_t words[2] = {exchanges_, now_ms};
    uint64_t h = support::fnv1a64(mac_.data(), mac_.size());
    h = support::fnv1a64(reinterpret_cast<const uint8_t*>(words), sizeof(words), h);
    return static_cast<uint32_t>(h ^ (h >> 32u));
}

}  // namespace rosette::net::wiznet
