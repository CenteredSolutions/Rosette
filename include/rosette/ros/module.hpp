/**
 * @file ros/module.hpp
 * @brief A `Module` is one slice of a firmware's ROS surface.
 *
 * A module groups the publishers, subscriptions, services, timers, and action
 * servers for one feature area (device, io, motion, …) with the callbacks that
 * back them, so the firmware's `main` is just a list of modules handed to
 * `rosette::App`.
 *
 * Lifecycle, driven by `App`:
 *
 *  - `register_(Node&)` — once per Agent connection, before traffic flows.
 *    Create every entity here (into `std::optional<...>` members so the next
 *    connection can rebuild them).
 *  - `on_connect()` — the connect edge. Burst-publish current state, clear
 *    latches, re-assert retained faults.
 *  - `publish()` — one status frame. Called while connected, at most once per
 *    `period_ms()`. A `period_ms()` of 0 means "no periodic publish".
 *  - `on_disconnect()` — the Agent was lost. Release entity slots; `register_`
 *    will run again on reconnect.
 *
 * This mirrors the reference controller's proven `Section` design, collapsed to
 * Rosette's cooperative single-threaded model — there is no task or thread per
 * module.
 */

#pragma once

#include <stdint.h>

namespace rosette {

class Node;

/**
 * @brief Base class for a feature slice of a firmware's ROS surface.
 */
class Module {
public:
    virtual ~Module() = default;

    /**
     * @brief Short identifier used in logs and as the default sub-namespace.
     */
    virtual const char* name() const noexcept = 0;

    /**
     * @brief Periodic publish interval in milliseconds; 0 disables it.
     */
    virtual uint32_t period_ms() const noexcept { return 0u; }

    /**
     * @brief rclc executor slots this module needs.
     *
     * Subscriptions, services, timers, and action servers each consume slots;
     * publishers consume none. `App` sums this across all modules to size the
     * executor before the context starts, so applications never hand-count.
     */
    virtual uint8_t executor_handles() const noexcept { return 4u; }

    /**
     * @brief Creates this module's entities on @p node.
     *
     * Runs once per Agent connection. Store entities in `std::optional<...>`
     * members so they can be rebuilt after a reconnect.
     */
    virtual void register_(Node& node) noexcept = 0;

    /**
     * @brief The Agent connection has just come up.
     */
    virtual void on_connect() noexcept {}

    /**
     * @brief The Agent connection has just been lost; release entity slots.
     */
    virtual void on_disconnect() noexcept {}

    /**
     * @brief Publish one status frame. Called only while connected.
     */
    virtual void publish() noexcept {}
};

}  // namespace rosette
