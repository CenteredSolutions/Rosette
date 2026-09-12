#pragma once

/**
 * @file net/address.hpp
 * @brief Combines the per-family `AddressManager`s into the interface's overall
 *        address state, supervises the physical link, and is the piece `main()`
 *        actually drives.
 *
 * One `AddressManager` runs the IPv4 provider list; an optional second runs the
 * IPv6 list. `NetworkAddress` folds their leases into one `AddressSnapshot`,
 * publishes the addresses to the platform runtime, fires a single `on_change`
 * callback, and — given a `NetInterface` — watches carrier: a link drop releases
 * the address, its return kicks a fresh acquisition (DHCP DISCOVER, not renew).
 *
 *     static rosette::net::AddressManager v4{providers};
 *     static rosette::net::NetworkAddress net{v4, nullptr, &iface};
 *     net.on_change([](const rosette::net::AddressSnapshot& s){ ... });
 *     net.begin();
 *     for (;;) {
 *         const uint32_t now = rosette_platform_get_ms();
 *         net.poll(now);
 *         if (net.has_address()) { app.spin_once(now); }
 *         rosette_platform_wait_for_event(...);
 *     }
 */

#include <array>
#include <cstdint>

#include "rosette/platform/host_runtime.hpp"
#include "rosette/support/inplace_function.hpp"

#include "address_manager.hpp"
#include "address_state.hpp"
#include "interface.hpp"

namespace rosette::net {

class NetworkAddress {
public:
    using ChangeCb = rosette::InplaceFunction<void(const AddressSnapshot&), 24u>;

    /**
     * @param v4    IPv4 provider list manager (required).
     * @param v6    IPv6 manager, or `nullptr`.
     * @param link  Interface to read carrier from, or `nullptr` to trust the
     *              managers' own state (FreeRTOS+TCP, host).
     */
    explicit NetworkAddress(AddressManager& v4, AddressManager* v6 = nullptr,
                            NetInterface* link = nullptr) noexcept
            : v4_(v4), v6_(v6), link_(link) {}

    NetworkAddress(AddressManager& v4, NetInterface* link) noexcept
            : NetworkAddress(v4, nullptr, link) {}

    void begin() noexcept {
        started_ = true;
        link_up_ = link_ == nullptr || link_->link_up();
        if (link_up_) {
            v4_.begin();
            if (v6_ != nullptr) {
                v6_->begin();
            }
        }
        rebuild();
    }

    void stop() noexcept {
        v4_.stop();
        if (v6_ != nullptr) {
            v6_->stop();
        }
        started_ = false;
        snapshot_ = AddressSnapshot{};
    }

    /// One cooperative step. Call every superloop iteration with the loop's `now_ms`.
    bool poll(uint32_t now_ms) noexcept {
        if (!started_) {
            return false;
        }

        // ── carrier supervision ────────────────────────────────────────────
        if (link_ != nullptr) {
            const bool up = link_->link_up();
            if (up != link_up_) {
                link_up_ = up;
                if (up) {
                    v4_.begin();                       // fresh acquisition
                    if (v6_ != nullptr) v6_->begin();
                } else {
                    v4_.stop();                        // release everything
                    if (v6_ != nullptr) v6_->stop();
                }
            }
        }

        if (link_up_) {
            (void)v4_.poll(now_ms);
            if (v6_ != nullptr) {
                (void)v6_->poll(now_ms);
            }
        }

        const AddressSnapshot before = snapshot_;
        rebuild();
        if (snapshot_ != before) {
            publish(before);
            if (on_change_) {
                on_change_(snapshot_);
            }
        }

        return snapshot_ != before;
    }

    /// Optional: pump `poll()` until an address is assigned or the budget lapses.
    bool wait(uint32_t timeout_ms) noexcept {
        const uint32_t deadline = rosette_platform_get_ms() + timeout_ms;
        for (;;) {
            poll(rosette_platform_get_ms());
            if (has_address()) {
                return true;
            }
            if (static_cast<int32_t>(rosette_platform_get_ms() - deadline) >= 0) {
                return false;
            }
            rosette_platform_delay_ms(50u);
        }
    }

    const AddressSnapshot& snapshot() const noexcept { return snapshot_; }
    bool has_address() const noexcept { return snapshot_.assigned; }
    bool stable() const noexcept { return link_up_ && v4_.stable(); }
    bool link_up() const noexcept { return link_up_; }

    /// Fired whenever the supervisor-visible snapshot changes — address gained,
    /// lost, changed, or a duplicate flagged. The callback inspects the snapshot.
    template <typename F> void on_change(F&& fn) noexcept { on_change_ = static_cast<F&&>(fn); }

private:
    void rebuild() noexcept {
        AddressSnapshot s{};
        s.active = started_;
        v4_.lease().fill_family(s);
        if (v6_ != nullptr) {
            v6_->lease().fill_family(s);
        }
        s.assigned = s.v4.has_address || s.v6.has_address;
        snapshot_ = s;
    }

    static uint32_t v4_host_order(const std::array<uint8_t, 16>& a) noexcept {
        return (static_cast<uint32_t>(a[0]) << 24) |
               (static_cast<uint32_t>(a[1]) << 16) |
               (static_cast<uint32_t>(a[2]) << 8) | static_cast<uint32_t>(a[3]);
    }

    void publish(const AddressSnapshot& before) noexcept {
        if (snapshot_.v4.has_address != before.v4.has_address ||
            snapshot_.v4.address != before.v4.address) {
            rosette_platform_set_ipv4_addr(
                    snapshot_.v4.has_address ? v4_host_order(snapshot_.v4.address)
                                             : 0u);
        }
        if (snapshot_.v6.has_address != before.v6.has_address ||
            snapshot_.v6.address != before.v6.address) {
            std::array<uint8_t, 16> zero{};
            rosette_platform_set_ipv6_addr(snapshot_.v6.has_address
                                                   ? snapshot_.v6.address.data()
                                                   : zero.data());
        }
    }

    AddressManager& v4_;
    AddressManager* v6_;
    NetInterface* link_;
    AddressSnapshot snapshot_{};
    bool started_ = false;
    bool link_up_ = true;

    ChangeCb on_change_{};
};

}    // namespace rosette::net
