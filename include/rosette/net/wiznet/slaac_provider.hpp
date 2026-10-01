#pragma once

/**
 * @file net/wiznet/slaac_provider.hpp
 * @brief IPv6 stateless address autoconfiguration (RFC 4862) on the W6100 / W6300.
 *
 * The W6100 and W6300 carry a hardware IPv6 stack: once the host programs a
 * link-local address (LLAR), a global address (GUAR), and a default router
 * (GA6R), the chip answers neighbor solicitations and echo requests for its
 * addresses and routes UDP6 traffic. Choosing those addresses is left to the
 * host, and this provider does it as one `AddressProvider` for IPv6:
 *
 *   1. Link-local: fe80::/64 plus the modified EUI-64 interface identifier of
 *      the board MAC (RFC 4291 §2.5.1, Appendix A). Duplicate address
 *      detection sends one neighbor solicitation (the chip's SLCR.NS command);
 *      silence for RetransTimer means unique, and the address goes to LLAR.
 *      An answered probe moves to the next candidate of
 *      `ipv6_address_candidate()` (RFC 7217 DAD_Counter), up to IDGEN_RETRIES
 *      times. The provider is `bound` from here on.
 *   2. Router discovery: the provider claims a hardware socket, opens it in
 *      IPRAW6 mode for ICMPv6, and sends router solicitations (SLCR.RS),
 *      MAX_RTR_SOLICITATIONS of them RTR_SOLICITATION_INTERVAL apart
 *      (RFC 4861 §6.3.7). Router advertisements reach the socket because
 *      ICMP6BLKR diverts them from the chip, and parse_router_advert() decodes
 *      them. The chip's own RA prefix registers are not used: the W6100
 *      decodes the prefix option at the wrong offset unless an 8-octet option
 *      precedes it, which RFC 4861 does not require. The socket is released as
 *      soon as an advertisement arrives or the solicitations run out.
 *   3. Global: an autonomous /64 prefix plus an interface identifier, chosen
 *      and checked by DAD the way the link-local address is (with its own
 *      conflict counter per prefix), is written to GUAR with its prefix mask,
 *      and the advertising router to GA6R when it offers itself as a default
 *      router. The lease then lists both addresses.
 *   4. Refresh: before the router lifetime (or, for a prefix-only router, the
 *      preferred or valid lifetime) runs out, the provider solicits again,
 *      updating lifetimes and following a changed prefix. A router lifetime
 *      that ends withdraws GA6R; a preferred lifetime that ends marks the
 *      global address deprecated; a valid lifetime that ends withdraws it.
 *      Without any router the provider keeps soliciting with the exponential
 *      backoff of RFC 7559.
 *
 * The provider holds a hardware socket only while it is soliciting, a few
 * seconds per refresh, so it shares the chip's sockets with DHCP, NTP, and the
 * application. When none is free it keeps its addresses and tries again a
 * second later.
 *
 * Every address write goes through `NetInterface::apply()`
 * (`WiznetInterface`) with the full lease, so the chip's registers always
 * match the lease's address list. When every link-local candidate is
 * duplicated, IPv6 stops on the interface (RFC 4862 §5.4.5) and the provider
 * reports `failed`; when every global candidate for a prefix is duplicated,
 * the link-local address stays in service and that prefix is not retried
 * until a different one is advertised.
 *
 *     static wiz::WiznetSlaacProvider<Chip, Bus> slaac{device, sockets, iface, mac};
 *     static rosette::net::AddressProvider* v6_list[] = {&slaac};
 *     static rosette::net::AddressManager v6{v6_list};
 *     static rosette::net::NetworkAddress address{v4, &v6, &iface};
 */

#include <array>
#include <cstddef>
#include <cstdint>

#include "rosette/net/address_candidate.hpp"
#include "rosette/net/address_provider.hpp"
#include "rosette/net/interface.hpp"
#include "rosette/net/ipv6_router_advert.hpp"
#include "rosette/platform/platform.hpp"

#include "device.hpp"
#include "socket_provider.hpp"

namespace rosette::net::wiznet {

/**
 * @brief Drives SLAAC for a W6100 / W6300.
 *
 * @tparam Chip A chip trait with `has_hw_ipv6()`.
 * @tparam Bus  The chip's bus.
 */
template <typename Chip, typename Bus>
class WiznetSlaacProvider final : public rosette::net::AddressProvider {
    static_assert(Chip::has_hw_ipv6(), "SLAAC needs a WIZnet part with hardware IPv6");

public:
    using device_type = WiznetDevice<Chip, Bus>;
    using sockets_type = WiznetSocketProvider<Chip, Bus>;
    using socket_type = typename device_type::socket_type;

    /// RetransTimer for duplicate address detection (RFC 4861 §10).
    static constexpr uint16_t kDadTimeoutMs = 1000u;
    /// RTR_SOLICITATION_INTERVAL (RFC 4861 §10): the wait for an advertisement after each RS.
    static constexpr uint32_t kSolicitIntervalMs = 4000u;
    /// MAX_RTR_SOLICITATIONS (RFC 4861 §10).
    static constexpr uint8_t kMaxSolicitations = 3u;
    /// First wait after a round of solicitations went unanswered; doubles per round.
    static constexpr uint32_t kFirstBackoffMs = 60000u;
    /// MAX_RTR_SOLICITATION_INTERVAL (RFC 7559 §2): the backoff ceiling, and the
    /// longest time between refreshes while a router is known.
    static constexpr uint32_t kMaxIntervalMs = 3600000u;
    /// Retry period when every hardware socket is in use.
    static constexpr uint32_t kSocketRetryMs = 1000u;
    /// A DAD probe the chip never completes is abandoned after this.
    static constexpr uint32_t kCommandDeadlineMs = 5000u;
    /// ICMPv6 next-header value (RFC 4443).
    static constexpr uint8_t kNextHeaderIcmp6 = 58u;

    WiznetSlaacProvider(device_type& device, sockets_type& sockets,
                        rosette::net::NetInterface& iface,
                        const std::array<uint8_t, 6>& mac) noexcept
        : device_(device), sockets_(sockets), iface_(iface), mac_(mac) {
        reset_lease();
    }

    void begin() noexcept override
    {
        const uint32_t now_ms = rosette::platform::monotonic_ms();
        release_socket();
        reset_lease();
        lease_.state = rosette::net::AddressPhase::acquiring;
        link_local_conflicts_ = 0u;
        link_local_ = rosette::net::ipv6_address_candidate(kLinkLocalPrefix, mac_, 0u);
        global_ = {};
        prefix_ = {};
        global_conflicts_ = 0u;
        deprecated_ = false;
        router_ = {};
        backoff_ms_ = kFirstBackoffMs;
        device_.set_icmp6_block(Chip::icmp6_block_router_advert());
        (void)device_.poll_socketless_status();    // drop latches from before begin()
        start_dad(Stage::dad_link_local, link_local_, now_ms);
    }

    void stop() noexcept override
    {
        release_socket();
        if (stage_ != Stage::idle) {
            iface_.apply(lease_.empty());    // withdraw LLAR, GUAR, GA6R
            device_.set_icmp6_block(0u);
        }
        stage_ = Stage::idle;
        reset_lease();
    }

    /**
     * @brief Advances detection, router discovery, and lifetime tracking.
     *
     * Costs one register read per call while a DAD probe is outstanding or
     * the socket is held, and none otherwise.
     *
     * @param[in] now_ms Monotonic milliseconds.
     * @return The IPv6 lease: `acquiring` until a link-local candidate passes
     *   DAD, then `bound`, listing the link-local address and, when one is
     *   configured, the global address; `failed` once IDGEN_RETRIES + 1
     *   link-local candidates have all been duplicated.
     *
     * Verification:
     * - Correct means: every lease address is one the chip holds and DAD
     *   has passed; a duplicate moves to the next candidate for the same
     *   prefix; a global address appears only from an autonomous /64
     *   advertised by a link-local router, GA6R holds that router only while
     *   its router lifetime lasts, the global address is deprecated when its
     *   preferred lifetime ends and leaves when its valid lifetime ends
     *   without a refresh.
     * - Covered by: tests/host/wiznet_slaac_provider_test.cpp
     *   (TC-NET-SLAAC-002 through -010); on hardware,
     *   examples/wiznet_udp_echo with tools/wiznet_udp_echo_probe.py.
     *
     * References: RFC 4862 §5.4 (DAD), §5.5.3 (prefix processing), §5.5.4
     * (deprecation); RFC 7217 §5 and §6 (next candidate after a conflict);
     * RFC 4861 §6.3.4, §6.3.7; RFC 7559 §2 (solicitation backoff).
     */
    const rosette::net::AddressLease& poll(uint32_t now_ms) noexcept override
    {
        using rosette::net::AddressFlags;
        rosette::net::set_flag(lease_.flags, AddressFlags::duplicate, false);

        switch (stage_) {
        case Stage::dad_link_local:
        case Stage::dad_global:
            poll_dad(now_ms);
            break;
        case Stage::soliciting:
            poll_solicitation(now_ms);
            break;
        case Stage::settled:
            if (due(now_ms, next_solicit_ms_)) start_solicitation(now_ms);
            break;
        default:
            break;
        }
        expire(now_ms);
        return lease_;
    }

    /// @brief The link-local address, valid once the lease is `bound`.
    const std::array<uint8_t, 16>& link_local_address() const noexcept { return link_local_; }

    /// @brief The global address, all zero until one passes DAD.
    const std::array<uint8_t, 16>& global_address() const noexcept { return global_; }

    /// @brief The default router programmed into GA6R, all zero when none.
    const std::array<uint8_t, 16>& default_router() const noexcept { return router_; }

private:
    enum class Stage : uint8_t {
        idle,
        dad_link_local,    ///< NS outstanding for the tentative link-local address.
        soliciting,        ///< Socket held; RS sent, waiting for an advertisement.
        dad_global,        ///< NS outstanding for the tentative global address.
        settled,           ///< Addresses in service; waiting for the next solicitation.
        failed,            ///< Link-local address duplicated; IPv6 is off.
    };

    static constexpr std::array<uint8_t, 16> kLinkLocalPrefix{0xFEu, 0x80u};
    static constexpr std::array<uint8_t, 16> kNoAddress{};

    static bool due(uint32_t now_ms, uint32_t at_ms) noexcept
    {
        return static_cast<int32_t>(now_ms - at_ms) >= 0;
    }

    void start_dad(Stage stage, const std::array<uint8_t, 16>& target, uint32_t now_ms) noexcept
    {
        stage_ = stage;
        deadline_ms_ = now_ms + kCommandDeadlineMs;
        device_.start_socketless_command(Chip::socketless_command_neighbor_solicit(), target, kDadTimeoutMs);
    }

    // DAD outcome: a neighbor answering the probe means the address is taken,
    // and the next candidate for the same prefix is probed, up to IDGEN_RETRIES.
    void poll_dad(uint32_t now_ms) noexcept
    {
        const uint8_t status = device_.poll_socketless_status();
        const bool answered = (status & Chip::socketless_status_neighbor_solicit()) != 0u;
        const bool unique =
            !answered
            && ((status & Chip::socketless_status_timeout()) != 0u || due(now_ms, deadline_ms_));
        if (answered) {
            rosette::net::set_flag(lease_.flags, rosette::net::AddressFlags::duplicate, true);
        }

        if (stage_ == Stage::dad_link_local) {
            if (answered && ++link_local_conflicts_ > rosette::net::kIdgenRetries) {
                lease_.state = rosette::net::AddressPhase::failed;
                stage_ = Stage::failed;
            } else if (answered) {
                link_local_ = rosette::net::ipv6_address_candidate(kLinkLocalPrefix, mac_,
                                                                   link_local_conflicts_);
                start_dad(Stage::dad_link_local, link_local_, now_ms);
            } else if (unique) {
                publish();
                start_solicitation(now_ms);
            }

            return;
        }

        if (answered && ++global_conflicts_ > rosette::net::kIdgenRetries) {
            candidate_ = {};
            settle(now_ms, refresh_delay_ms());
        } else if (answered) {
            candidate_ = rosette::net::ipv6_address_candidate(prefix_, mac_, global_conflicts_);
            start_dad(Stage::dad_global, candidate_, now_ms);
        } else if (unique) {
            global_ = candidate_;
            publish();
            settle(now_ms, refresh_delay_ms());
        }
    }

    void start_solicitation(uint32_t now_ms) noexcept
    {
        socket_ = sockets_.claim_raw_socket();
        if (socket_ >= Chip::socket_count() || !device_.open_ipraw6_socket(socket_, kNextHeaderIcmp6)) {
            release_socket();
            settle(now_ms, kSocketRetryMs);
            return;
        }
        stage_ = Stage::soliciting;
        solicitations_ = 0u;
        send_solicitation(now_ms);
    }

    void send_solicitation(uint32_t now_ms) noexcept
    {
        ++solicitations_;
        next_rs_ms_ = now_ms + kSolicitIntervalMs;
        device_.start_socketless_command(Chip::socketless_command_router_solicit(), {},
                                         static_cast<uint16_t>(kDadTimeoutMs));
    }

    // Reads every advertisement waiting on the socket; the first valid one
    // ends the solicitation. Unanswered, RS repeats up to kMaxSolicitations.
    void poll_solicitation(uint32_t now_ms) noexcept
    {
        (void)device_.poll_socketless_status();    // RS completes by timeout; the RA comes to the socket
        for (;;) {
            UdpEndpoint from{};
            const auto n = device_.ipraw_receive_from(socket_, rx_.data(), rx_.size(), &from);
            if (n == 0) break;
            rosette::net::RouterAdvert ra{};
            if (n > 0 && from.ipv6 &&
                rosette::net::parse_router_advert(rx_.data(), static_cast<std::size_t>(n), from.address, ra)) {
                release_socket();
                backoff_ms_ = kFirstBackoffMs;
                accept(ra, now_ms);
                return;
            }
        }
        if (!due(now_ms, next_rs_ms_)) return;
        if (solicitations_ < kMaxSolicitations) {
            send_solicitation(now_ms);
            return;
        }
        release_socket();
        settle(now_ms, backoff_ms_);
        backoff_ms_ = backoff_ms_ >= kMaxIntervalMs / 2u ? kMaxIntervalMs : backoff_ms_ * 2u;
    }

    // RFC 4862 §5.5.3 for the chosen prefix, plus the router as default router.
    // The same prefix refreshes the address in service; a new prefix starts
    // DAD at conflict counter 0; a prefix whose candidates were all
    // duplicated is not retried.
    void accept(const rosette::net::RouterAdvert& ra, uint32_t now_ms) noexcept
    {
        router_lifetime_ms_ = static_cast<uint32_t>(ra.router_lifetime_s) * 1000u;
        router_until_ms_ = now_ms + router_lifetime_ms_;
        router_ = ra.router_lifetime_s != 0u ? ra.router : kNoAddress;
        if (!ra.has_prefix) {
            if (global_ != kNoAddress) {
                publish();
            }

            settle(now_ms, refresh_delay_ms());
            return;
        }

        set_lifetimes(ra.valid_lifetime_s, ra.preferred_lifetime_s, now_ms);
        if (ra.prefix != prefix_) {
            prefix_ = ra.prefix;
            global_conflicts_ = 0u;
        } else if (global_ != kNoAddress) {
            publish();  // same prefix: new lifetimes and router
            settle(now_ms, refresh_delay_ms());
            return;
        }

        if (global_conflicts_ > rosette::net::kIdgenRetries) {
            settle(now_ms, refresh_delay_ms());
            return;
        }

        candidate_ = rosette::net::ipv6_address_candidate(prefix_, mac_, global_conflicts_);
        start_dad(Stage::dad_global, candidate_, now_ms);
    }

    // Lifetimes past ~24 days do not fit the millisecond clock; they are
    // treated as infinite, and the refresh cycle renews them long before.
    void set_lifetimes(uint32_t valid_s, uint32_t preferred_s, uint32_t now_ms) noexcept {
        constexpr uint32_t kLongest = 0x7FFFFFFFu / 1000u;
        valid_s_ = valid_s;
        valid_infinite_ = valid_s > kLongest;
        valid_until_ms_ = now_ms + (valid_infinite_ ? 0u : valid_s * 1000u);
        preferred_s_ = preferred_s;
        preferred_infinite_ = preferred_s > kLongest;
        preferred_until_ms_ = now_ms + (preferred_infinite_ ? 0u : preferred_s * 1000u);
        deprecated_ = preferred_s == 0u;
    }

    // Half the shortest lifetime still in force, at most kMaxIntervalMs.
    uint32_t refresh_delay_ms() const noexcept
    {
        uint32_t delay = kMaxIntervalMs;
        if (router_lifetime_ms_ != 0u && router_lifetime_ms_ / 2u < delay) {
            delay = router_lifetime_ms_ / 2u;
        }

        if (global_ != kNoAddress && !valid_infinite_ && valid_s_ * 500u < delay) {
            delay = valid_s_ * 500u;
        }

        if (global_ != kNoAddress && !preferred_infinite_ && !deprecated_
            && preferred_s_ * 500u < delay) {
            delay = preferred_s_ * 500u;
        }

        return delay < kSocketRetryMs ? kSocketRetryMs : delay;
    }

    void settle(uint32_t now_ms, uint32_t delay_ms) noexcept
    {
        stage_ = Stage::settled;
        next_solicit_ms_ = now_ms + delay_ms;
    }

    // Router lifetime over: withdraw GA6R. Preferred lifetime over: deprecate
    // the global address. Valid lifetime over: withdraw GUAR.
    void expire(uint32_t now_ms) noexcept
    {
        if (router_ != kNoAddress && due(now_ms, router_until_ms_)) {
            router_ = {};
            if (global_ != kNoAddress) {
                publish();
            }
        }

        if (global_ != kNoAddress && !deprecated_ && !preferred_infinite_
            && due(now_ms, preferred_until_ms_)) {
            deprecated_ = true;
            publish();
        }

        if (global_ != kNoAddress && !valid_infinite_ && due(now_ms, valid_until_ms_)) {
            global_ = {};
            publish();
        }
    }

    // Lists the addresses in service and programs the chip to match. Six
    // transitions call it, so it stays out of line.
    [[gnu::noinline]] void publish() noexcept {
        lease_.state = rosette::net::AddressPhase::bound;
        lease_.addresses.clear();
        lease_.add(link_local_, 64u);
        if (global_ != kNoAddress) {
            lease_.add(global_, 64u, valid_s_, preferred_s_, deprecated_);
        }

        lease_.gateway_v6 = router_;
        iface_.apply(lease_);
    }

    void release_socket() noexcept
    {
        if (socket_ < Chip::socket_count()) {
            device_.close_socket(socket_);
            sockets_.release(socket_);
        }
        socket_ = Chip::socket_count();
    }

    void reset_lease() noexcept
    {
        lease_ = rosette::net::AddressLease{};
        lease_.family = rosette::net::AddressFamily::ipv6;
        lease_.source = rosette::net::AddressSource::slaac;
    }

    device_type& device_;
    sockets_type& sockets_;
    rosette::net::NetInterface& iface_;
    std::array<uint8_t, 6> mac_;
    rosette::net::AddressLease lease_{};
    Stage stage_ = Stage::idle;
    socket_type socket_ = Chip::socket_count();
    std::array<uint8_t, 16> link_local_{};
    std::array<uint8_t, 16> global_{};
    std::array<uint8_t, 16> candidate_{};
    std::array<uint8_t, 16>
        prefix_{};  ///< Prefix the global address and its conflict counter belong to.
    std::array<uint8_t, 16> router_{};
    uint8_t link_local_conflicts_ = 0u;
    uint8_t global_conflicts_ = 0u;
    uint32_t valid_s_ = 0u;
    bool valid_infinite_ = false;
    uint32_t valid_until_ms_ = 0u;
    uint32_t preferred_s_ = 0u;
    bool preferred_infinite_ = false;
    bool deprecated_ = false;
    uint32_t preferred_until_ms_ = 0u;
    uint32_t router_lifetime_ms_ = 0u;
    uint32_t router_until_ms_ = 0u;
    uint32_t deadline_ms_ = 0u;
    uint32_t next_rs_ms_ = 0u;
    uint32_t next_solicit_ms_ = 0u;
    uint32_t backoff_ms_ = kFirstBackoffMs;
    uint8_t solicitations_ = 0u;
    /// One advertisement; a larger one is discarded (RFC 4861 allows up to the link MTU).
    std::array<uint8_t, 640> rx_{};
};

}  // namespace rosette::net::wiznet
