#pragma once

#include <array>
#include <cstddef>

#include "address_provider.hpp"

namespace rosette::net {

/**
 * @brief Default no-op link-local observer used when no fallback source exists.
 *
 * Some platforms provide a separate Zeroconf or AutoIP mechanism, while others
 * either do not support link-local fallback or do not need it. This observer
 * satisfies the `DhcpAddressProvider` contract without forcing every caller to
 * provide an auxiliary address source.
 */
class NullLinkLocalObserver {
public:
    /**
     * @brief Start the observer.
     *
     * @return Always `true`.
     */
    bool start() noexcept { return true; }

    /**
     * @brief Attempt to read a link-local IPv4 address.
     *
     * @param[out] ipv4 Destination for the link-local address when present.
     * @return Always `false`.
     */
    bool readLinkLocalIpv4(std::array<uint8_t, 4>* ipv4) const noexcept {
        (void)ipv4;
        return false;
    }
};

/**
 * @brief Rosette address provider backed by a DHCP client and optional link-local observer.
 *
 * This provider normalizes common IPv4 address-management policy into one
 * reusable surface:
 * - use a DHCP lease when one is valid
 * - fall back to a link-local observer when DHCP is unavailable
 * - report lifecycle state independently from the underlying network stack
 *
 * The wrapped DHCP client type is expected to provide:
 * - `using DhcpLease`
 * - `using DhcpState`
 * - `bool start(assign_callback_type, change_callback_type, void*)`
 * - `bool lease(DhcpLease&) const`
 * - `DhcpState state() const`
 * - `void force_renew()`
 *
 * The link-local observer type is expected to provide:
 * - `bool start()`
 * - `bool readLinkLocalIpv4(std::array<uint8_t, 4>*) const`
 */
template <typename DhcpClient, typename LinkLocalObserver = NullLinkLocalObserver>
class DhcpAddressProvider : public IAddressProvider {
public:
    using DhcpLease = typename DhcpClient::DhcpLease;
    using DhcpState = typename DhcpClient::DhcpState;

    /**
     * @brief Construct a provider over one DHCP client with the default no-op fallback.
     *
     * @param[in] dhcp_client DHCP client that owns lease acquisition and renewal.
     */
    explicit DhcpAddressProvider(DhcpClient& dhcp_client) noexcept
            : dhcp_client_(dhcp_client),
              owned_link_local_observer_(),
              link_local_observer_(&owned_link_local_observer_) {}

    /**
     * @brief Construct a provider over one DHCP client and one fallback observer.
     *
     * The fallback observer is held by reference so callback-driven or
     * externally-updated link-local state remains visible to subsequent
     * `step()` calls.
     *
     * @param[in] dhcp_client DHCP client that owns lease acquisition and renewal.
     * @param[in] link_local_observer Observer that reports link-local fallback
     *         state such as Zeroconf or AutoIP.
     */
    DhcpAddressProvider(DhcpClient& dhcp_client,
                        LinkLocalObserver& link_local_observer) noexcept
            : dhcp_client_(dhcp_client), link_local_observer_(&link_local_observer) {}

    /**
     * @brief Start DHCP address acquisition and optional link-local observation.
     *
     * The provider starts the auxiliary link-local observer first so fallback
     * state can be reported immediately if the platform already owns it. DHCP is
     * then started with no callbacks because the provider polls the lease and
     * state explicitly during `step()`.
     *
     * @return `true` when the DHCP client started successfully or was already
     *         running; otherwise `false`.
     */
    bool start() noexcept override {
        if (started_) {
            return true;
        }

        link_local_started_ = link_local_observer_->start();
        started_ = dhcp_client_.start(nullptr, nullptr, nullptr);
        refreshSnapshot();
        return started_;
    }

    /**
     * @brief Stop address reporting for this provider.
     *
     * Stopping the provider clears the public snapshot to an inactive state. The
     * underlying DHCP client may continue to exist, depending on the platform's
     * lifecycle model.
     */
    void stop() noexcept override {
        started_ = false;
        link_local_started_ = false;
        snapshot_ = AddressSnapshot();
    }

    /**
     * @brief Refresh DHCP and link-local state and report whether it changed.
     *
     * The polling algorithm prefers a valid DHCP lease, then falls back to a
     * link-local observer, and finally reports acquisition or failure based on
     * the DHCP client state. This gives supervisors a consistent readiness model
     * regardless of whether the underlying platform is polling or callback
     * driven.
     *
     * @return `true` when the visible address snapshot changed.
     */
    bool step() noexcept override {
        const AddressSnapshot before = snapshot_;
        refreshSnapshot();
        return before != snapshot_;
    }

    /**
     * @brief Read the latest provider snapshot.
     *
     * @return Current normalized address state.
     */
    AddressSnapshot snapshot() const noexcept override { return snapshot_; }

    /**
     * @brief Request early DHCP renewal from the wrapped client.
     *
     * This hook is useful for higher-level policy that wants to refresh a lease
     * after a link transition or after recovering from a transient network error.
     */
    void requestRenew() noexcept { dhcp_client_.force_renew(); }

private:
    /**
     * @brief Refresh the normalized snapshot from DHCP and fallback state.
     *
     * The provider builds a fresh snapshot on every poll so the public state is a
     * complete translation of the current DHCP lease, fallback observer, and DHCP
     * lifecycle status.
     *
     * References:
     * - RFC 2131 sections 3.1 and 4.4.5 for DHCP lease acquisition and renewal.
     * - RFC 3927 section 2.1 for link-local IPv4 addressing.
     */
    void refreshSnapshot() noexcept {
        snapshot_ = AddressSnapshot();
        snapshot_.active = started_;
        snapshot_.status =
                started_ ? AddressStatus::acquiring : AddressStatus::inactive;

        if (!started_) {
            return;
        }

        DhcpLease lease{};
        if (dhcp_client_.lease(lease) && lease.valid) {
            populateFromLease(lease);
            return;
        }

        if (link_local_started_) {
            std::array<uint8_t, 4> link_local{};
            if (link_local_observer_->readLinkLocalIpv4(&link_local)) {
                snapshot_.assigned = true;
                snapshot_.status = AddressStatus::assigned;
                snapshot_.mode = AddressMode::zeroconf_v4;
                snapshot_.quality = AddressQuality::link_local;
                snapshot_.ipv4 = link_local;
                snapshot_.has_ipv4 = true;
                return;
            }
        }

        switch (dhcp_client_.state()) {
            case DhcpState::failed:
                snapshot_.status = AddressStatus::failed;
                break;
            case DhcpState::leased:
                snapshot_.status = AddressStatus::assigned;
                break;
            case DhcpState::init:
            case DhcpState::acquiring:
            case DhcpState::renewing:
            default:
                snapshot_.status = AddressStatus::acquiring;
                break;
        }
    }

    /**
     * @brief Translate one DHCP lease into the normalized public snapshot.
     *
     * The provider classifies lease quality from the leased IPv4 address and
     * preserves DHCP-supplied NTP server information so time synchronization
     * services can adopt it later.
     *
     * @param[in] lease Current DHCP lease exported by the wrapped client.
     */
    void populateFromLease(const DhcpLease& lease) noexcept {
        snapshot_.assigned = true;
        snapshot_.status = AddressStatus::assigned;
        snapshot_.mode = AddressMode::dhcp_v4;
        snapshot_.has_ipv4 = true;
        snapshot_.lease_remaining_s = lease.lease_seconds;

        for (std::size_t octet = 0; octet < snapshot_.ipv4.size(); ++octet) {
            snapshot_.ipv4[octet] = lease.ip[octet];
        }

        snapshot_.quality = IsIpv4LinkLocal(snapshot_.ipv4)
                                                        ? AddressQuality::link_local
                                                        : AddressQuality::routable;

        snapshot_.ntp_server_count =
                lease.ntp_count > snapshot_.ntp_servers_v4.size()
                        ? static_cast<uint8_t>(snapshot_.ntp_servers_v4.size())
                        : lease.ntp_count;
        for (std::size_t i = 0; i < snapshot_.ntp_server_count; ++i) {
            for (std::size_t octet = 0; octet < snapshot_.ntp_servers_v4[i].size();
                     ++octet) {
                snapshot_.ntp_servers_v4[i][octet] = lease.ntp[i][octet];
            }
        }
    }

    DhcpClient& dhcp_client_;
    LinkLocalObserver owned_link_local_observer_{};
    LinkLocalObserver* link_local_observer_ = nullptr;
    AddressSnapshot snapshot_{};
    bool started_ = false;
    bool link_local_started_ = false;
};

}    // namespace rosette::net
