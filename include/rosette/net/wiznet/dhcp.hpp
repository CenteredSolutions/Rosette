/**
 * @file net/wiznet/dhcp.hpp
 * @brief DHCPv4 client over a `WiznetSocketProvider`.
 *
 * `DhcpProtocol` is the execution-independent core: it builds and parses the
 * DISCOVER/OFFER/REQUEST/ACK wire format and runs one exchange to completion.
 * The address write goes through `apply_config()` — `WiznetDevice::configure_ipv4`
 * by default, or a `set_apply()` callback (`DhcpAddressProvider` routes it
 * through `NetInterface::apply()`). Deadlines are measured with
 * `rosette_platform_get_ms()`, so it carries no RTOS dependency.
 *
 * Two thin drivers pace it:
 *   - @ref DhcpPoller — `acquire()` once, then `poll()` from the superloop.
 *     No threads; a renewal exchange briefly blocks the caller.
 *   - @ref DhcpTask — a FreeRTOS task that acquires, sleeps to T1, renews.
 *     Present only when FreeRTOS is available.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>

#include "rosette/platform/host_runtime.hpp"

#include "device.hpp"
#include "socket_provider.hpp"

#ifndef ROSETTE_NO_FREERTOS
extern "C" {
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
}
#endif

namespace rosette::net::wiznet {

struct DhcpLease {
    std::array<uint8_t, 4> ip{};
    std::array<uint8_t, 4> gateway{};
    std::array<uint8_t, 4> subnet{};
    std::array<std::array<uint8_t, 4>, 3> dns{};
    std::array<std::array<uint8_t, 4>, 3> ntp{};
    std::array<uint8_t, 4> server_identifier{};
    uint8_t dns_count = 0u;
    uint8_t ntp_count = 0u;
    uint32_t lease_seconds = 0u;
    uint32_t t1_seconds = 0u;
    uint32_t t2_seconds = 0u;
    bool valid = false;
};

enum class DhcpState : uint8_t {
    init = 0u,
    acquiring,
    leased,
    renewing,
    failed,
};

struct DhcpConfig {
    uint16_t client_port = 68u;
    uint16_t server_port = 67u;
    uint32_t receive_timeout_ms = 1000u;
    uint32_t exchange_timeout_ms = 60000u;
    uint32_t retry_delay_ms = 10000u;
    const char* host_name = nullptr;
};

/**
 * @brief Execution-independent DHCPv4 protocol engine.
 *
 * Drivers call the protected `run_exchange()` and read `lease()` / `state()`.
 */
template<typename Chip, typename Bus,
         template<typename> class ExecutorT = DefaultExecutor>
class DhcpProtocol {
public:
    using device_type = WiznetDevice<Chip, Bus>;
    using socket_provider_type = WiznetSocketProvider<Chip, Bus, ExecutorT>;

    using assign_callback_type = void (*)(const DhcpLease& lease, void* user);
    using change_callback_type = void (*)(const DhcpLease& old_lease,
                                          const DhcpLease& new_lease,
                                          void* user);

    DhcpProtocol(device_type& device,
                 socket_provider_type& provider,
                 const DhcpConfig& config = {}) noexcept
        : device_(device)
        , provider_(provider)
        , config_(config) {}

    /// Copies the current lease into @p out; returns whether it is valid.
    bool lease(DhcpLease& out) const noexcept
    {
        lease_lock();
        out = lease_;
        lease_unlock();
        return out.valid;
    }

    DhcpState state() const noexcept { return state_; }

    const DhcpConfig& config() const noexcept { return config_; }

    void set_callbacks(assign_callback_type on_assign,
                       change_callback_type on_change,
                       void* user) noexcept
    {
        on_assign_ = on_assign;
        on_change_ = on_change;
        callback_user_ = user;
    }

    /// Redirect the "write the address onto the interface" step. When set, the
    /// protocol calls this instead of `WiznetDevice::configure_ipv4()` — an
    /// invalid `DhcpLease` means "clear". `DhcpAddressProvider` routes it through
    /// `NetInterface::apply()`; unset, the default keeps legacy callers working.
    using apply_callback_type = void (*)(const DhcpLease& lease, void* user);
    void set_apply(apply_callback_type on_apply, void* user) noexcept
    {
        on_apply_ = on_apply;
        apply_user_ = user;
    }

protected:
    void set_state(DhcpState state) noexcept { state_ = state; }

    /// Apply @p lease to the interface (or clear it when `lease.valid` is false).
    void apply_config(const DhcpLease& lease) noexcept
    {
        if (on_apply_ != nullptr) {
            on_apply_(lease, apply_user_);
            return;
        }
        const std::array<uint8_t, 4> zero{};
        device_.configure_ipv4(lease.valid ? lease.ip : zero,
                               lease.valid ? lease.subnet : zero,
                               lease.valid ? lease.gateway : zero);
    }

    /**
     * @brief Broadcasts one DHCPDECLINE for a conflicted lease (RFC 2131 §4.4.4).
     *
     * Called by a driver's conflict handler after the chip flags a duplicate
     * address. Does not wait for a reply — DECLINE is unacknowledged; the
     * caller then restarts acquisition (with a fresh xid) after the RFC's
     * ≥10 s pause.
     */
    bool send_decline(const std::array<uint8_t, 4>& declined_ip,
                      const std::array<uint8_t, 4>& server_id) noexcept
    {
        std::unique_ptr<Socket> socket = provider_.open();
        if (!socket) {
            return false;
        }
        if (socket->bind({.address = {0u, 0u, 0u, 0u}, .port = config_.client_port}) != 0) {
            return false;
        }

        std::array<uint8_t, kPacketBytes> packet{};
        const std::size_t length =
            build_decline(packet, next_xid(), declined_ip, server_id);
        return socket->send_to(packet.data(),
                               length,
                               make_socket_address_v4({255u, 255u, 255u, 255u},
                                                      config_.server_port)) ==
               static_cast<std::ptrdiff_t>(length);
    }

    /// Drops the stored lease so `lease()` reports invalid.
    void clear_lease() noexcept
    {
        lease_lock();
        lease_ = DhcpLease{};
        lease_unlock();
    }

    /**
     * @brief Runs one DISCOVER/OFFER/REQUEST/ACK exchange.
     *
     * Blocks (via the socket's receive timeout) until a lease is obtained,
     * a server NAKs, an error occurs, or `config_.exchange_timeout_ms` lapses.
     * On success the lease is applied to the chip and stored.
     */
    bool run_exchange() noexcept
    {
        std::printf("[DHCP] resetting IPv4 configuration before exchange\n");
        apply_config(DhcpLease{});

        std::unique_ptr<Socket> socket = provider_.open();
        if (!socket) {
            std::printf("[DHCP] failed to open DHCP socket\n");
            return false;
        }

        if (socket->bind({.address = {0u, 0u, 0u, 0u}, .port = config_.client_port}) != 0) {
            std::printf("[DHCP] bind failed on client port %u, error=%s\n",
                        config_.client_port,
                        socket_error_string(socket->last_error()));
            return false;
        }
        std::printf("[DHCP] bound to UDP/%u\n", config_.client_port);

        if (socket->set_receive_timeout(config_.receive_timeout_ms) != 0) {
            std::printf("[DHCP] failed to set receive timeout=%lu ms, error=%s\n",
                        static_cast<unsigned long>(config_.receive_timeout_ms),
                        socket_error_string(socket->last_error()));
            return false;
        }

        ExchangeContext exchange{};
        exchange.xid = next_xid();
        std::printf("[DHCP] exchange xid=0x%08lx timeout=%lu ms\n",
                    static_cast<unsigned long>(exchange.xid),
                    static_cast<unsigned long>(config_.exchange_timeout_ms));

        if (!send_discover(*socket, exchange.xid)) {
            std::printf("[DHCP] DISCOVER send failed\n");
            return false;
        }
        std::printf("[DHCP] DISCOVER sent\n");

        const uint32_t deadline = rosette_platform_get_ms() + config_.exchange_timeout_ms;
        if (!await_offer(*socket, exchange, deadline)) {
            std::printf("[DHCP] OFFER wait failed for xid=0x%08lx\n",
                        static_cast<unsigned long>(exchange.xid));
            return false;
        }

        std::printf("[DHCP] OFFER received ");
        log_ipv4("ip=", exchange.offer.ip);
        std::printf(" ");
        log_ipv4("server=", exchange.offer.server_identifier);
        std::printf("\n");

        if (!send_request(*socket, exchange.xid, exchange.offer)) {
            std::printf("[DHCP] REQUEST send failed\n");
            return false;
        }
        std::printf("[DHCP] REQUEST sent\n");

        DhcpLease leased{};
        if (!await_ack(*socket, exchange, deadline, leased)) {
            std::printf("[DHCP] ACK wait failed for xid=0x%08lx\n",
                        static_cast<unsigned long>(exchange.xid));
            return false;
        }

        std::printf("[DHCP] ACK received ");
        log_ipv4("ip=", leased.ip);
        std::printf(" ");
        log_ipv4("gateway=", leased.gateway);
        std::printf(" lease=%lu s t1=%lu s t2=%lu s\n",
                    static_cast<unsigned long>(leased.lease_seconds),
                    static_cast<unsigned long>(leased.t1_seconds),
                    static_cast<unsigned long>(leased.t2_seconds));

        store_lease(leased);
        return true;
    }

    void lease_lock() const noexcept
    {
#ifndef ROSETTE_NO_FREERTOS
        if (lease_mutex_ != nullptr) {
            xSemaphoreTake(lease_mutex_, portMAX_DELAY);
        }
#endif
    }

    void lease_unlock() const noexcept
    {
#ifndef ROSETTE_NO_FREERTOS
        if (lease_mutex_ != nullptr) {
            xSemaphoreGive(lease_mutex_);
        }
#endif
    }

    device_type& device_;
    socket_provider_type& provider_;
    DhcpConfig config_{};
    DhcpLease lease_{};
    volatile DhcpState state_ = DhcpState::init;
    assign_callback_type on_assign_ = nullptr;
    change_callback_type on_change_ = nullptr;
    void* callback_user_ = nullptr;
    apply_callback_type on_apply_ = nullptr;
    void* apply_user_ = nullptr;
    uint32_t xid_seed_ = 0x4d445343u;
#ifndef ROSETTE_NO_FREERTOS
    SemaphoreHandle_t lease_mutex_ = nullptr;
#endif

private:
    static constexpr std::size_t kPacketBytes = 548u;
    static constexpr std::size_t kMinPacketBytes = 240u;
    static constexpr uint8_t kBootRequest = 1u;
    static constexpr uint8_t kBootReply = 2u;
    static constexpr uint8_t kHwTypeEthernet = 1u;
    static constexpr uint8_t kHwLengthEthernet = 6u;
    static constexpr uint16_t kBroadcastFlag = 0x8000u;
    static constexpr uint32_t kMagicCookie = 0x63825363u;

    static constexpr uint8_t kOptionPad = 0u;
    static constexpr uint8_t kOptionSubnetMask = 1u;
    static constexpr uint8_t kOptionRouter = 3u;
    static constexpr uint8_t kOptionDnsServer = 6u;
    static constexpr uint8_t kOptionHostName = 12u;
    static constexpr uint8_t kOptionRequestedIp = 50u;
    static constexpr uint8_t kOptionLeaseTime = 51u;
    static constexpr uint8_t kOptionMessageType = 53u;
    static constexpr uint8_t kOptionServerIdentifier = 54u;
    static constexpr uint8_t kOptionParameterRequestList = 55u;
    static constexpr uint8_t kOptionRenewalTime = 58u;
    static constexpr uint8_t kOptionRebindingTime = 59u;
    static constexpr uint8_t kOptionClientIdentifier = 61u;
    static constexpr uint8_t kOptionNtpServer = 42u;
    static constexpr uint8_t kOptionEnd = 255u;

    static constexpr uint8_t kMessageDiscover = 1u;
    static constexpr uint8_t kMessageOffer = 2u;
    static constexpr uint8_t kMessageRequest = 3u;
    static constexpr uint8_t kMessageDecline = 4u;
    static constexpr uint8_t kMessageAck = 5u;
    static constexpr uint8_t kMessageNak = 6u;

    static constexpr uint32_t kInitialRetransmitMs = 2000u;
    static constexpr uint32_t kMaxRetransmitMs = 16000u;

    struct ExchangeContext {
        uint32_t xid = 0u;
        DhcpLease offer{};
    };

    /**
     * @brief Exponential-backoff retransmit clock (RFC 2131 §4.1).
     *
     * `due()` returns true at ~2 s, 4 s, 8 s, 16 s, 16 s, ... after
     * construction. UDP is lossy and the first broadcast DISCOVER often goes
     * out before the switch port forwards (STP), so the request must be resent
     * rather than waited on.
     */
    struct RetransmitTimer {
        uint32_t next_ms = rosette_platform_get_ms() + kInitialRetransmitMs;
        uint32_t interval_ms = kInitialRetransmitMs;

        bool due() noexcept
        {
            const uint32_t now = rosette_platform_get_ms();
            if (static_cast<int32_t>(now - next_ms) < 0) {
                return false;
            }
            interval_ms = interval_ms < kMaxRetransmitMs ? (interval_ms * 2u)
                                                         : kMaxRetransmitMs;
            next_ms = now + interval_ms;
            return true;
        }
    };

    static const char* message_type_string(uint8_t type) noexcept
    {
        switch (type) {
        case kMessageDiscover: return "DISCOVER";
        case kMessageOffer:    return "OFFER";
        case kMessageRequest:  return "REQUEST";
        case kMessageAck:      return "ACK";
        case kMessageNak:      return "NAK";
        default:               return "UNKNOWN";
        }
    }

    static void log_ipv4(const char* label,
                         const std::array<uint8_t, 4>& address) noexcept
    {
        std::printf("%s%u.%u.%u.%u",
                    label, address[0], address[1], address[2], address[3]);
    }

    bool send_discover(Socket& socket, uint32_t xid) noexcept
    {
        std::array<uint8_t, kPacketBytes> packet{};
        const std::size_t length = build_discover(packet, xid);
        return socket.send_to(packet.data(),
                              length,
                              make_socket_address_v4({255u, 255u, 255u, 255u},
                                                     config_.server_port)) ==
               static_cast<std::ptrdiff_t>(length);
    }

    bool send_request(Socket& socket,
                      uint32_t xid,
                      const DhcpLease& offer) noexcept
    {
        std::array<uint8_t, kPacketBytes> packet{};
        const std::size_t length = build_request(packet, xid, offer);
        return socket.send_to(packet.data(),
                              length,
                              make_socket_address_v4({255u, 255u, 255u, 255u},
                                                     config_.server_port)) ==
               static_cast<std::ptrdiff_t>(length);
    }

    bool await_offer(Socket& socket,
                     ExchangeContext& exchange,
                     uint32_t deadline_ms) noexcept
    {
        std::array<uint8_t, kPacketBytes> packet{};
        RetransmitTimer retransmit{};
        for (;;) {
            if (time_expired(deadline_ms)) {
                return false;
            }

            if (retransmit.due()) {
                std::printf("[DHCP] retransmitting DISCOVER xid=0x%08lx\n",
                            static_cast<unsigned long>(exchange.xid));
                (void)send_discover(socket, exchange.xid);
            }

            SocketAddress remote{};
            const std::ptrdiff_t received =
                socket.recv_from(packet.data(), packet.size(), &remote);
            if (received < 0) {
                if ((socket.last_error() == SocketError::timed_out) ||
                    (socket.last_error() == SocketError::would_block)) {
                    continue;
                }
                std::printf("[DHCP] receive error while waiting for OFFER: %s\n",
                            socket_error_string(socket.last_error()));
                return false;
            }

            DhcpLease offer{};
            uint8_t message_type = 0u;
            if (!parse_packet(packet.data(),
                              static_cast<std::size_t>(received),
                              exchange.xid,
                              offer,
                              message_type)) {
                std::printf("[DHCP] ignored packet while waiting for OFFER from %u.%u.%u.%u:%u\n",
                            remote.address[0], remote.address[1],
                            remote.address[2], remote.address[3],
                            remote.port);
                continue;
            }

            std::printf("[DHCP] received %s from %u.%u.%u.%u:%u\n",
                        message_type_string(message_type),
                        remote.address[0], remote.address[1],
                        remote.address[2], remote.address[3],
                        remote.port);

            if (message_type == kMessageOffer) {
                exchange.offer = offer;
                return true;
            }
        }
    }

    bool await_ack(Socket& socket,
                   const ExchangeContext& exchange,
                   uint32_t deadline_ms,
                   DhcpLease& leased) noexcept
    {
        std::array<uint8_t, kPacketBytes> packet{};
        RetransmitTimer retransmit{};
        for (;;) {
            if (time_expired(deadline_ms)) {
                return false;
            }

            if (retransmit.due()) {
                std::printf("[DHCP] retransmitting REQUEST xid=0x%08lx\n",
                            static_cast<unsigned long>(exchange.xid));
                (void)send_request(socket, exchange.xid, exchange.offer);
            }

            SocketAddress remote{};
            const std::ptrdiff_t received =
                socket.recv_from(packet.data(), packet.size(), &remote);
            if (received < 0) {
                if ((socket.last_error() == SocketError::timed_out) ||
                    (socket.last_error() == SocketError::would_block)) {
                    continue;
                }
                std::printf("[DHCP] receive error while waiting for ACK: %s\n",
                            socket_error_string(socket.last_error()));
                return false;
            }

            uint8_t message_type = 0u;
            if (!parse_packet(packet.data(),
                              static_cast<std::size_t>(received),
                              exchange.xid,
                              leased,
                              message_type)) {
                std::printf("[DHCP] ignored packet while waiting for ACK from %u.%u.%u.%u:%u\n",
                            remote.address[0], remote.address[1],
                            remote.address[2], remote.address[3],
                            remote.port);
                continue;
            }

            std::printf("[DHCP] received %s from %u.%u.%u.%u:%u\n",
                        message_type_string(message_type),
                        remote.address[0], remote.address[1],
                        remote.address[2], remote.address[3],
                        remote.port);

            if (message_type == kMessageNak) {
                std::printf("[DHCP] server rejected request with NAK\n");
                return false;
            }

            if (message_type == kMessageAck) {
                leased.valid = true;
                if (leased.lease_seconds == 0u) {
                    leased.lease_seconds = 86400u;
                }
                if (leased.t1_seconds == 0u) {
                    leased.t1_seconds = leased.lease_seconds / 2u;
                }
                if (leased.t2_seconds == 0u) {
                    leased.t2_seconds = leased.lease_seconds * 7u / 8u;
                }
                return true;
            }
        }
    }

    void store_lease(const DhcpLease& lease) noexcept
    {
        std::printf("[DHCP] applying lease ");
        log_ipv4("ip=", lease.ip);
        std::printf(" ");
        log_ipv4("subnet=", lease.subnet);
        std::printf(" ");
        log_ipv4("gateway=", lease.gateway);
        std::printf("\n");

        apply_config(lease);

        DhcpLease old{};
        lease_lock();
        old = lease_;
        lease_ = lease;
        lease_unlock();

        if (!old.valid) {
            if (on_assign_ != nullptr) {
                on_assign_(lease, callback_user_);
            }
            return;
        }

        if ((on_change_ != nullptr) &&
            ((old.ip != lease.ip) || (old.gateway != lease.gateway) ||
             (old.subnet != lease.subnet))) {
            on_change_(old, lease, callback_user_);
        }
    }

    bool parse_packet(const uint8_t* packet,
                      std::size_t length,
                      uint32_t expected_xid,
                      DhcpLease& lease,
                      uint8_t& message_type) const noexcept
    {
        if ((packet == nullptr) || (length < kMinPacketBytes)) {
            std::printf("[DHCP] reject packet: too short (%lu bytes)\n",
                        static_cast<unsigned long>(length));
            return false;
        }

        if ((packet[0] != kBootReply) ||
            (packet[1] != kHwTypeEthernet) ||
            (packet[2] != kHwLengthEthernet)) {
            std::printf("[DHCP] reject packet: invalid BOOTP header op=%u htype=%u hlen=%u\n",
                        packet[0], packet[1], packet[2]);
            return false;
        }

        const uint32_t packet_xid = read_be32(packet + 4u);
        if (packet_xid != expected_xid) {
            std::printf("[DHCP] reject packet: xid mismatch got=0x%08lx expected=0x%08lx\n",
                        static_cast<unsigned long>(packet_xid),
                        static_cast<unsigned long>(expected_xid));
            return false;
        }

        const auto mac = device_.mac_address();
        if (std::memcmp(packet + 28u, mac.data(), mac.size()) != 0) {
            std::printf("[DHCP] reject packet: chaddr mismatch\n");
            return false;
        }

        if (read_be32(packet + 236u) != kMagicCookie) {
            std::printf("[DHCP] reject packet: missing magic cookie\n");
            return false;
        }

        std::memcpy(lease.ip.data(), packet + 16u, lease.ip.size());
        parse_options(packet + 240u, length - 240u, lease, message_type);
        if (message_type == 0u) {
            std::printf("[DHCP] reject packet: missing DHCP message type option\n");
        }
        return message_type != 0u;
    }

    void parse_options(const uint8_t* options,
                       std::size_t length,
                       DhcpLease& lease,
                       uint8_t& message_type) const noexcept
    {
        std::size_t i = 0u;
        while (i < length) {
            const uint8_t option = options[i++];
            if (option == kOptionPad) {
                continue;
            }
            if (option == kOptionEnd) {
                break;
            }
            if (i >= length) {
                break;
            }

            const uint8_t option_length = options[i++];
            if ((i + option_length) > length) {
                break;
            }

            const uint8_t* value = options + i;
            switch (option) {
            case kOptionMessageType:
                if (option_length >= 1u) {
                    message_type = value[0];
                }
                break;
            case kOptionSubnetMask:
                if (option_length >= lease.subnet.size()) {
                    std::memcpy(lease.subnet.data(), value, lease.subnet.size());
                }
                break;
            case kOptionRouter:
                if (option_length >= lease.gateway.size()) {
                    std::memcpy(lease.gateway.data(), value, lease.gateway.size());
                }
                break;
            case kOptionDnsServer:
                lease.dns_count = copy_server_list(value,
                                                   option_length,
                                                   lease.dns.data(),
                                                   lease.dns.size());
                break;
            case kOptionNtpServer:
                lease.ntp_count = copy_server_list(value,
                                                   option_length,
                                                   lease.ntp.data(),
                                                   lease.ntp.size());
                break;
            case kOptionServerIdentifier:
                if (option_length >= lease.server_identifier.size()) {
                    std::memcpy(lease.server_identifier.data(),
                                value,
                                lease.server_identifier.size());
                }
                break;
            case kOptionLeaseTime:
                if (option_length >= 4u) {
                    lease.lease_seconds = read_be32(value);
                }
                break;
            case kOptionRenewalTime:
                if (option_length >= 4u) {
                    lease.t1_seconds = read_be32(value);
                }
                break;
            case kOptionRebindingTime:
                if (option_length >= 4u) {
                    lease.t2_seconds = read_be32(value);
                }
                break;
            default:
                break;
            }

            i += option_length;
        }
    }

    static uint8_t copy_server_list(const uint8_t* data,
                                    std::size_t length,
                                    std::array<uint8_t, 4>* out,
                                    std::size_t out_count) noexcept
    {
        if ((data == nullptr) || (out == nullptr)) {
            return 0u;
        }

        uint8_t copied = 0u;
        for (std::size_t offset = 0u;
             (offset + 4u) <= length && copied < out_count;
             offset += 4u, ++copied) {
            std::memcpy(out[copied].data(), data + offset, 4u);
        }
        return copied;
    }

    std::size_t build_discover(std::array<uint8_t, kPacketBytes>& packet,
                               uint32_t xid) const noexcept
    {
        std::size_t option_index = build_packet_header(packet, xid);
        option_index = append_option_u8(packet, option_index, kOptionMessageType, kMessageDiscover);
        option_index = append_client_identifier(packet, option_index);
        option_index = append_parameter_request_list(packet, option_index);
        option_index = append_host_name(packet, option_index);
        packet[option_index++] = kOptionEnd;
        return option_index;
    }

    std::size_t build_request(std::array<uint8_t, kPacketBytes>& packet,
                              uint32_t xid,
                              const DhcpLease& offer) const noexcept
    {
        std::size_t option_index = build_packet_header(packet, xid);
        option_index = append_option_u8(packet, option_index, kOptionMessageType, kMessageRequest);
        option_index = append_client_identifier(packet, option_index);
        option_index = append_option_bytes(packet,
                                           option_index,
                                           kOptionRequestedIp,
                                           offer.ip.data(),
                                           offer.ip.size());
        option_index = append_option_bytes(packet,
                                           option_index,
                                           kOptionServerIdentifier,
                                           offer.server_identifier.data(),
                                           offer.server_identifier.size());
        option_index = append_parameter_request_list(packet, option_index);
        option_index = append_host_name(packet, option_index);
        packet[option_index++] = kOptionEnd;
        return option_index;
    }

    std::size_t build_decline(std::array<uint8_t, kPacketBytes>& packet,
                              uint32_t xid,
                              const std::array<uint8_t, 4>& declined_ip,
                              const std::array<uint8_t, 4>& server_id) const noexcept
    {
        // RFC 2131 §4.4.4 / table 5: DHCPDECLINE carries the requested-IP and
        // server-identifier options; ciaddr stays 0.
        std::size_t option_index = build_packet_header(packet, xid);
        option_index = append_option_u8(packet, option_index, kOptionMessageType, kMessageDecline);
        option_index = append_client_identifier(packet, option_index);
        option_index = append_option_bytes(packet,
                                           option_index,
                                           kOptionRequestedIp,
                                           declined_ip.data(),
                                           declined_ip.size());
        option_index = append_option_bytes(packet,
                                           option_index,
                                           kOptionServerIdentifier,
                                           server_id.data(),
                                           server_id.size());
        packet[option_index++] = kOptionEnd;
        return option_index;
    }

    std::size_t build_packet_header(std::array<uint8_t, kPacketBytes>& packet,
                                    uint32_t xid) const noexcept
    {
        packet.fill(0u);
        packet[0] = kBootRequest;
        packet[1] = kHwTypeEthernet;
        packet[2] = kHwLengthEthernet;
        write_be32(packet.data() + 4u, xid);
        write_be16(packet.data() + 10u, kBroadcastFlag);

        const auto mac = device_.mac_address();
        std::memcpy(packet.data() + 28u, mac.data(), mac.size());
        write_be32(packet.data() + 236u, kMagicCookie);
        return 240u;
    }

    std::size_t append_client_identifier(std::array<uint8_t, kPacketBytes>& packet,
                                         std::size_t index) const noexcept
    {
        const auto mac = device_.mac_address();
        if ((index + 2u + 1u + mac.size()) >= packet.size()) {
            return index;
        }

        packet[index++] = kOptionClientIdentifier;
        packet[index++] = static_cast<uint8_t>(1u + mac.size());
        packet[index++] = kHwTypeEthernet;
        std::memcpy(packet.data() + index, mac.data(), mac.size());
        return index + mac.size();
    }

    std::size_t append_parameter_request_list(std::array<uint8_t, kPacketBytes>& packet,
                                              std::size_t index) const noexcept
    {
        static constexpr std::array<uint8_t, 6> request_list = {
            kOptionSubnetMask,
            kOptionRouter,
            kOptionDnsServer,
            kOptionNtpServer,
            kOptionLeaseTime,
            kOptionServerIdentifier,
        };

        return append_option_bytes(packet,
                                   index,
                                   kOptionParameterRequestList,
                                   request_list.data(),
                                   request_list.size());
    }

    std::size_t append_host_name(std::array<uint8_t, kPacketBytes>& packet,
                                 std::size_t index) const noexcept
    {
        if (config_.host_name == nullptr) {
            return index;
        }

        std::size_t length = 0u;
        while ((config_.host_name[length] != '\0') && (length < 32u)) {
            ++length;
        }

        return append_option_bytes(packet,
                                   index,
                                   kOptionHostName,
                                   reinterpret_cast<const uint8_t*>(config_.host_name),
                                   length);
    }

    static std::size_t append_option_u8(std::array<uint8_t, kPacketBytes>& packet,
                                        std::size_t index,
                                        uint8_t option,
                                        uint8_t value) noexcept
    {
        return append_option_bytes(packet, index, option, &value, 1u);
    }

    static std::size_t append_option_bytes(std::array<uint8_t, kPacketBytes>& packet,
                                           std::size_t index,
                                           uint8_t option,
                                           const uint8_t* data,
                                           std::size_t length) noexcept
    {
        if ((data == nullptr) || (length == 0u) ||
            (length > 255u) || ((index + 2u + length) >= packet.size())) {
            return index;
        }

        packet[index++] = option;
        packet[index++] = static_cast<uint8_t>(length);
        std::memcpy(packet.data() + index, data, length);
        return index + length;
    }

    static bool time_expired(uint32_t deadline_ms) noexcept
    {
        return static_cast<int32_t>(rosette_platform_get_ms() - deadline_ms) >= 0;
    }

    uint32_t next_xid() noexcept
    {
        const auto mac = device_.mac_address();
        xid_seed_ ^= (static_cast<uint32_t>(mac[2]) << 24u) |
                     (static_cast<uint32_t>(mac[3]) << 16u) |
                     (static_cast<uint32_t>(mac[4]) << 8u) |
                     static_cast<uint32_t>(mac[5]);
        xid_seed_ += 0x01010101u;
        xid_seed_ ^= rosette_platform_get_ms();
        return xid_seed_;
    }

    static uint16_t read_be16(const uint8_t* data) noexcept
    {
        return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8u) | data[1]);
    }

    static uint32_t read_be32(const uint8_t* data) noexcept
    {
        return (static_cast<uint32_t>(data[0]) << 24u) |
               (static_cast<uint32_t>(data[1]) << 16u) |
               (static_cast<uint32_t>(data[2]) << 8u) |
               static_cast<uint32_t>(data[3]);
    }

    static void write_be16(uint8_t* data, uint16_t value) noexcept
    {
        data[0] = static_cast<uint8_t>(value >> 8u);
        data[1] = static_cast<uint8_t>(value);
    }

    static void write_be32(uint8_t* data, uint32_t value) noexcept
    {
        data[0] = static_cast<uint8_t>(value >> 24u);
        data[1] = static_cast<uint8_t>(value >> 16u);
        data[2] = static_cast<uint8_t>(value >> 8u);
        data[3] = static_cast<uint8_t>(value);
    }
};

/**
 * @brief Cooperative DHCP driver for a bare superloop.
 *
 *     DhcpPoller<W5100S, PicoSpiBus> dhcp{device, provider};
 *     while (!dhcp.acquire()) { rosette_platform_delay_ms(1000u); }
 *     for (;;) { ...; dhcp.poll(); }        // ~1 Hz is plenty
 *
 * `acquire()` and a due `poll()` block the caller for one exchange (bounded by
 * `config().exchange_timeout_ms`; a working server answers in tens of ms).
 */
template<typename Chip, typename Bus,
         template<typename> class ExecutorT = DefaultExecutor>
class DhcpPoller : public DhcpProtocol<Chip, Bus, ExecutorT> {
public:
    using base = DhcpProtocol<Chip, Bus, ExecutorT>;
    using base::base;

    /// Runs one exchange now. Returns true once a lease is applied.
    bool acquire() noexcept
    {
        this->set_state(DhcpState::acquiring);
        if (this->run_exchange()) {
            arm_renew();
            this->set_state(DhcpState::leased);
            return true;
        }
        this->set_state(DhcpState::failed);
        retry_at_ms_ = rosette_platform_get_ms() + this->config().retry_delay_ms;
        return false;
    }

    /**
     * @brief React to a duplicate-address conflict on the current lease.
     *
     * Sends a DHCPDECLINE for the conflicted address (if a lease is held),
     * clears the chip's IPv4 config and the stored lease, and schedules a fresh
     * DISCOVER after the RFC 2131 §3.1 ≥10 s pause. `poll()` then drives the
     * re-acquisition. The new exchange uses a fresh xid and no requested-IP
     * hint, so the server is free to offer a different address.
     */
    void handle_conflict() noexcept
    {
        DhcpLease held{};
        if (this->lease(held) && held.valid) {
            (void)this->send_decline(held.ip, held.server_identifier);
        }
        this->clear_lease();
        this->apply_config(DhcpLease{});
        this->set_state(DhcpState::failed);

        const uint32_t wait_ms = this->config().retry_delay_ms < 10000u
                                     ? 10000u
                                     : this->config().retry_delay_ms;
        retry_at_ms_ = rosette_platform_get_ms() + wait_ms;
    }

    /// Superloop tick: renews at T1, retries after a failure.
    void poll() noexcept
    {
        const uint32_t now = rosette_platform_get_ms();
        switch (this->state()) {
        case DhcpState::leased:
            if (static_cast<int32_t>(now - renew_at_ms_) < 0) {
                return;
            }
            this->set_state(DhcpState::renewing);
            if (this->run_exchange()) {
                arm_renew();
                this->set_state(DhcpState::leased);
            } else {
                this->set_state(DhcpState::failed);
                retry_at_ms_ = now + this->config().retry_delay_ms;
            }
            return;
        case DhcpState::failed:
            if (static_cast<int32_t>(now - retry_at_ms_) >= 0) {
                (void)acquire();
            }
            return;
        default:
            return;
        }
    }

private:
    void arm_renew() noexcept
    {
        DhcpLease current{};
        (void)this->lease(current);
        const uint32_t t1_s = current.t1_seconds != 0u ? current.t1_seconds : 1800u;
        renew_at_ms_ = rosette_platform_get_ms() + (t1_s * 1000u);
    }

    uint32_t renew_at_ms_ = 0u;
    uint32_t retry_at_ms_ = 0u;
};

#ifndef ROSETTE_NO_FREERTOS

/**
 * @brief FreeRTOS DHCP driver: acquires, sleeps to T1, renews, on its own task.
 */
template<typename Chip, typename Bus,
         template<typename> class ExecutorT = DefaultExecutor>
class DhcpTask : public DhcpProtocol<Chip, Bus, ExecutorT> {
public:
    using base = DhcpProtocol<Chip, Bus, ExecutorT>;
    using base::base;

    ~DhcpTask()
    {
        if (task_handle_ != nullptr) {
            vTaskDelete(task_handle_);
        }
    }

    struct TaskOptions {
        uint16_t stack_words = 768u;
        UBaseType_t priority = tskIDLE_PRIORITY + 1u;
        const char* name = "dhcp";
    };

    bool start(typename base::assign_callback_type on_assign = nullptr,
               typename base::change_callback_type on_change = nullptr,
               void* callback_user = nullptr,
               const TaskOptions& options = {}) noexcept
    {
        this->set_callbacks(on_assign, on_change, callback_user);

        if (this->lease_mutex_ == nullptr) {
            this->lease_mutex_ = xSemaphoreCreateMutex();
            if (this->lease_mutex_ == nullptr) {
                return false;
            }
        }

        if (task_handle_ != nullptr) {
            return true;
        }

        options_ = options;
        return xTaskCreate(&task_entry,
                           options_.name,
                           options_.stack_words,
                           this,
                           options_.priority,
                           &task_handle_) == pdPASS;
    }

    void force_renew() noexcept { force_renew_ = true; }

private:
    static void task_entry(void* arg) noexcept
    {
        static_cast<DhcpTask*>(arg)->task_main();
    }

    void task_main() noexcept
    {
        for (;;) {
            DhcpLease snapshot{};
            (void)this->lease(snapshot);

            if (!snapshot.valid) {
                this->set_state(DhcpState::acquiring);
                if (this->run_exchange()) {
                    this->set_state(DhcpState::leased);
                } else {
                    this->set_state(DhcpState::failed);
                    vTaskDelay(pdMS_TO_TICKS(this->config().retry_delay_ms));
                }
                continue;
            }

            this->set_state(DhcpState::leased);
            TickType_t wake_at =
                xTaskGetTickCount() + pdMS_TO_TICKS(snapshot.t1_seconds * 1000u);
            while (!force_renew_) {
                const TickType_t now = xTaskGetTickCount();
                if (static_cast<int32_t>(now - wake_at) >= 0) {
                    break;
                }
                const TickType_t remaining = wake_at - now;
                const TickType_t chunk = pdMS_TO_TICKS(5000u);
                vTaskDelay(remaining < chunk ? remaining : chunk);
            }

            force_renew_ = false;
            this->set_state(DhcpState::renewing);
            if (this->run_exchange()) {
                this->set_state(DhcpState::leased);
            } else {
                this->set_state(DhcpState::failed);
                vTaskDelay(pdMS_TO_TICKS(this->config().retry_delay_ms));
            }
        }
    }

    TaskHandle_t task_handle_ = nullptr;
    TaskOptions options_{};
    volatile bool force_renew_ = false;
};

#endif  // ROSETTE_NO_FREERTOS

}  // namespace rosette::net::wiznet
