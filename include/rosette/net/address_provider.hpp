#pragma once

#include "address_state.hpp"

namespace rosette::net {

/**
 * @brief Common lifecycle contract for platform-specific address providers.
 *
 * Rosette supports multiple networking environments with different ownership
 * models. Some platforms require Rosette to run a DHCP client directly, while
 * others expose address state through callbacks or begin already configured.
 * `IAddressProvider` gives higher-level runtimes one pollable surface that
 * hides those platform details.
 */
class IAddressProvider {
public:
    virtual ~IAddressProvider() {}

    /**
     * @brief Start the provider and begin reporting address readiness.
     *
     * Providers may launch background tasks, register callbacks, or simply mark
     * a static address as active. A successful start means the provider is ready
     * to participate in supervisor polling.
     *
     * @return `true` when the provider is active or was already active;
     *         otherwise `false`.
     */
    virtual bool start() noexcept = 0;

    /**
     * @brief Stop the provider and return it to an inactive state.
     *
     * The exact stop semantics depend on the underlying network stack. Some
     * providers may only quiesce their reporting surface while lower-level tasks
     * continue running. Higher layers should treat the resulting snapshot as
     * inactive regardless of the underlying platform behavior.
     */
    virtual void stop() noexcept = 0;

    /**
     * @brief Refresh the provider snapshot.
     *
     * Polling runtimes call `step()` cooperatively so callback-driven and
     * task-driven address sources can be normalized into a single state machine.
     *
     * @return `true` when the visible address snapshot changed during this step.
     */
    virtual bool step() noexcept = 0;

    /**
     * @brief Read the latest address snapshot.
     *
     * @return Current platform-neutral address state.
     */
    virtual AddressSnapshot snapshot() const noexcept = 0;
};

/**
 * @brief Deterministic address provider for host tests and static deployments.
 *
 * POSIX harnesses and boards with preconfigured network stacks often do not
 * need an active DHCP client. `StaticAddressProvider` lets those environments
 * participate in the same supervisor flow as dynamic-address boards by
 * exposing a stable snapshot through the standard Rosette provider contract.
 */
class StaticAddressProvider : public IAddressProvider {
public:
    /**
     * @brief Construct a provider with a caller-supplied address snapshot.
     *
     * The configured snapshot describes the address state that should appear
     * whenever the provider is active. `start()` and `stop()` only change the
     * provider lifecycle fields, not the configured address contents.
     *
     * @param[in] configured_snapshot Address state to expose while active.
     */
    explicit StaticAddressProvider(
            const AddressSnapshot& configured_snapshot = {}) noexcept
            : configured_snapshot_(configured_snapshot) {
        snapshot_ = configured_snapshot_;
        normalize_for_stop();
    }

    /**
     * @brief Replace the configured snapshot exposed while the provider is active.
     *
     * This is useful for host tests that need to exercise ready, degraded, and
     * unconfigured address states without swapping the provider object itself.
     *
     * @param[in] configured_snapshot New configured address state.
     */
    void setSnapshot(const AddressSnapshot& configured_snapshot) noexcept {
        configured_snapshot_ = configured_snapshot;
        snapshot_ = configured_snapshot_;
        if (started_) {
            normalize_for_start();
        } else {
            normalize_for_stop();
        }
    }

    /**
     * @brief Activate the configured snapshot.
     *
     * @return Always `true`.
     */
    bool start() noexcept override {
        started_ = true;
        snapshot_ = configured_snapshot_;
        normalize_for_start();
        return true;
    }

    /**
     * @brief Mark the provider inactive while preserving the configured snapshot.
     */
    void stop() noexcept override {
        started_ = false;
        snapshot_ = configured_snapshot_;
        normalize_for_stop();
    }

    /**
     * @brief Refresh the provider snapshot.
     *
     * Static providers have no background state, so a polling step never changes
     * the visible snapshot unless the caller previously updated it through
     * `setSnapshot()`.
     *
     * @return Always `false`.
     */
    bool step() noexcept override { return false; }

    /**
     * @brief Read the current provider snapshot.
     *
     * @return Current active or inactive snapshot.
     */
    AddressSnapshot snapshot() const noexcept override { return snapshot_; }

private:
    /**
     * @brief Normalize lifecycle fields for the active state.
     *
     * Static providers should behave like an already-configured network stack
     * once started. The method ensures the supervisor sees an active provider
     * with either `assigned` or `acquiring` status depending on whether the
     * configured snapshot contains an address.
     */
    void normalize_for_start() noexcept {
        snapshot_.active = true;
        snapshot_.status =
                snapshot_.assigned ? AddressStatus::assigned : AddressStatus::acquiring;
        if (!snapshot_.assigned) {
            snapshot_.quality = AddressQuality::none;
            snapshot_.has_ipv4 = false;
            snapshot_.has_ipv6 = false;
            snapshot_.lease_remaining_s = 0u;
            snapshot_.ntp_server_count = 0u;
        }
    }

    /**
     * @brief Normalize lifecycle fields for the inactive state.
     *
     * Inactive providers should not present stale address readiness to a
     * supervisor. The configured snapshot is preserved separately so a later
     * `start()` can restore it.
     */
    void normalize_for_stop() noexcept {
        snapshot_.active = false;
        snapshot_.assigned = false;
        snapshot_.status = AddressStatus::inactive;
        snapshot_.quality = AddressQuality::none;
        snapshot_.has_ipv4 = false;
        snapshot_.has_ipv6 = false;
        snapshot_.lease_remaining_s = 0u;
        snapshot_.ntp_server_count = 0u;
    }

    AddressSnapshot configured_snapshot_{};
    AddressSnapshot snapshot_{};
    bool started_ = false;
};

}    // namespace rosette::net
