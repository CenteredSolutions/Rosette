#pragma once

/**
 * @file net/address.hpp
 * @brief Combines the per-family `AddressManager`s into the interface's overall
 *        address state, supervises the physical link, and is the piece `main()`
 *        actually drives.
 *
 * One `AddressManager` runs the IPv4 provider list; an optional second runs the
 * IPv6 list. `NetworkAddress` folds their leases into one `AddressSnapshot`,
 * fires a single `on_change` callback, and — given a `NetInterface` — watches carrier: a link drop
 * releases the address, its return kicks a fresh acquisition (DHCP DISCOVER, not renew).
 *
 *     static rosette::net::AddressManager v4{providers};
 *     static rosette::net::NetworkAddress net{v4, nullptr, &iface};
 *     net.on_change([](const rosette::net::AddressSnapshot& s){ ... });
 *     net.begin();
 *     for (;;) {
 *         const uint32_t now = rosette::platform::monotonic_ms();
 *         net.poll(now);
 *         if (net.has_address()) { app.spin_once(now); }
 *         rosette::platform::sleep_ms(...);
 *     }
 */

#include <array>
#include <cstdint>

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
            if (on_change_) {
                on_change_(snapshot_);
            }
        }

        return snapshot_ != before;
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

        s.assigned = s.v4.has_address() || s.v6.has_address();
        snapshot_ = s;
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
