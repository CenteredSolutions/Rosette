#pragma once

/**
 * @file net/sntp_client.hpp
 * @brief Keeps the board's wall clock on UTC with SNTP, driven from the main loop.
 *
 * A board boots knowing only its monotonic tick. `SntpClient` measures the
 * offset from that tick to UTC against an NTP server and keeps
 * `platform::WallClock` (monotonic time plus one offset) on it, so ROS
 * stamps, RTPS source timestamps, and rcl's system clock all read UTC. The
 * board network runs it (net/board_network.hpp) whenever the board
 * holds an IPv4 address.
 *
 * The client asks only servers it has been told about: `SntpConfig::server`
 * (set by the firmware, from its static configuration or a stored
 * `storage::DeviceConfig::ntp_server`) first, then the servers DHCP offered
 * (option 42). With neither, it sends nothing and the clock stays unset;
 * there is no built-in public server.
 *
 * On the micro-ROS agent backend the agent is the board's time source
 * (`AgentSessionGate` synchronizes with it), so `SntpConfig::enabled`
 * defaults to false in builds that define `ROSETTE_ROS_BACKEND_AGENT`, which
 * the build sets for every target when `ROSETTE_ENABLE_MICRO_ROS` is on. A
 * firmware that wants SNTP there anyway sets `enabled` itself.
 *
 * `poll()` is a main-loop step like the DHCP client's: it never loops or
 * spawns a task. When an exchange is due it sends one request and waits for
 * the reply through the socket's receive timeout (at most
 * `reply_timeout_ms`), which is how the socket backend waits under any
 * scheduler: the WIZnet `PollingExecutor` parks the CPU until the chip's
 * INT line fires, the `FreeRtosExecutor` blocks the calling task. Waiting
 * inside the call is deliberate: the reply's arrival time (T4) is read the
 * moment the socket returns it, where a later main-loop pass would add its
 * own latency to the measured delay and bias the offset.
 *
 * Clock discipline:
 * - Filter: the last eight samples are kept and the one with the smallest
 *   distance (half its round-trip delay plus 15 ppm of its age) is used,
 *   since queueing delay is what corrupts an offset and age is what makes
 *   it stale; a sample is used only when it is newer than the last one
 *   used (RFC 5905 §10).
 * - Phase: the used sample's offset, carried forward from its measurement
 *   to now at the estimated frequency, is written to WallClock.
 * - Frequency: the drift of the monotonic tick against UTC is measured
 *   between successive used samples and folded into the estimate with a
 *   weight of the measurement's span over `frequency_averaging_s` (at most
 *   1), because timing jitter corrupts a measurement in inverse proportion
 *   to its span: short polls average many measurements, long ones are
 *   trusted almost alone (the FLL half of RFC 5905 §11.3). The estimate is
 *   applied to WallClock on every `poll()`, so the clock stays on UTC
 *   between exchanges.
 * - Poll interval: `min_poll_ms` for the first eight exchanges or while
 *   the clock is off by more than `settled_error_us`, doubling up to
 *   `max_poll_ms` once it holds; `retry_ms` after a failed exchange.
 *
 * Concurrency: `poll()` runs on the one thread that owns the network;
 * WallClock itself may be read from any thread.
 */

#include <array>
#include <cstdint>

#include "rosette/net/address_state.hpp"
#include "rosette/net/sntp_wire.hpp"
#include "rosette/net/socket.hpp"
#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"

namespace rosette::net {

#if defined(ROSETTE_ROS_BACKEND_AGENT)
/// The agent backend takes its time from the agent, so SNTP starts disabled.
inline constexpr bool kSntpEnabledByDefault = false;
#else
/// Without an agent, SNTP is the board's time source whenever it learns of a server.
inline constexpr bool kSntpEnabledByDefault = true;
#endif

/// Where the board gets UTC from, and how often it asks.
struct SntpConfig {
    /// Whether the board network runs the client at all.
    bool enabled = kSntpEnabledByDefault;
    /// A configured server, asked before any DHCP offered; a port of 0 (the default) means none.
    SocketAddress server{};
    /// Local UDP port of the request socket, which is open only during an exchange.
    uint16_t local_port = 12300u;
    /// Longest wait for a reply; the longest one `poll()` call blocks.
    uint32_t reply_timeout_ms = 200u;
    /// Interval between exchanges until the clock settles.
    uint32_t min_poll_ms = 64000u;
    /// Longest interval between exchanges once it has.
    uint32_t max_poll_ms = 1024000u;
    /// Wait after a failed exchange (RFC 4330 §10 sets 15 s as the floor).
    uint32_t retry_ms = 16000u;
    /// Clock error above which polling returns to `min_poll_ms`.
    int64_t settled_error_us = 2000;
    /// Span, in seconds, at which one frequency measurement replaces the estimate outright;
    /// a shorter one is weighted by its span over this.
    double frequency_averaging_s = 1024.0;
    /// Largest frequency correction applied, in parts per million.
    double max_frequency_ppm = 500.0;
};

/// The clock the client measures with and disciplines.
struct SntpClock {
    /// The local monotonic tick, in microseconds.
    uint64_t (*monotonic_us)();
    /// Unix time minus the tick, in microseconds.
    int64_t (*offset_us)();
    /// Replaces that offset.
    void (*set_offset_us)(int64_t offset_us);
};

/// @brief The platform's clocks: monotonic_us() under platform::WallClock.
inline SntpClock wall_clock_for_sntp() noexcept {
    return SntpClock{
        [] { return platform::monotonic_us(); },
        [] { return platform::WallClock::offset_us(); },
        [](int64_t offset_us) { platform::WallClock::set_offset_us(offset_us); },
    };
}

/// An SNTP client that disciplines one clock.
class SntpClient {
public:
    /**
     * @brief A client that has not yet exchanged with any server.
     *
     * @param[in] sockets The provider the request socket is opened on; must outlive the client.
     * @param[in] config  Servers and timing.
     * @param[in] clock   The clock to measure and discipline (`wall_clock_for_sntp()` on firmware).
     */
    SntpClient(SocketProvider& sockets, const SntpConfig& config, SntpClock clock) noexcept;

    /**
     * @brief Applies the frequency correction and, when one is due, runs one exchange.
     *
     * Call on every pass of the main loop. With no IPv4 address, or no
     * server (none configured and none offered by DHCP), nothing is sent.
     * A due exchange goes to the current server, the configured one first;
     * a failure moves on to the next server and retries after `retry_ms`; a
     * kiss-o'-death moves on and waits `max_poll_ms`.
     *
     * @param[in] now_ms   Monotonic time in milliseconds.
     * @param[in] snapshot The network's addresses and DHCP-offered NTP servers.
     *
     * Blocking: at most `reply_timeout_ms`, and only when an exchange is due.
     *
     * Verification:
     * - Correct means: after an exchange with a server whose clock is
     *   `offset` ahead of the tick, the clock's offset equals `offset` to
     *   within half the round-trip asymmetry; between exchanges the offset
     *   advances at the estimated frequency; exchanges are spaced by the
     *   poll rules above; only the configured and DHCP-offered servers are
     *   asked, the configured one first.
     * - Covered by: tests/host/sntp_client_test.cpp (TC-NET-SNTP-CLI-001 to
     *   TC-NET-SNTP-CLI-007); tests/net/sntp_live_probe.cpp against a
     *   public server.
     *
     * References:
     * - RFC 4330 §5 (client operation), §10 (poll floor)
     * - RFC 5905 §7.4 (kiss-o'-death), §8 (on-wire), §10 (clock filter)
     */
    void poll(uint32_t now_ms, const AddressSnapshot& snapshot) noexcept;

    /// @brief Whether the client has set the clock at least once.
    bool synced() const noexcept { return synced_; }

    /// @brief Exchanges that produced a sample.
    uint32_t samples() const noexcept { return samples_; }

    /// @brief The estimated drift of the tick against UTC, in parts per million.
    double frequency_ppm() const noexcept { return frequency_ppm_; }

    /// @brief The interval until the next scheduled exchange.
    uint32_t poll_interval_ms() const noexcept { return poll_interval_ms_; }

private:
    /// One filter entry: a sample and the tick time it was measured at.
    struct FilterEntry {
        SntpSample sample;
        uint64_t at_us = 0u;
    };

    /// How one exchange ended.
    enum class Exchange : uint8_t { sampled, no_reply, kiss_of_death };

    bool pick_server(const AddressSnapshot& snapshot, SocketAddress& server) const noexcept;
    Exchange exchange(const SocketAddress& server, FilterEntry& entry) noexcept;
    const FilterEntry& select_sample(uint64_t now_us) const noexcept;
    int64_t discipline(const FilterEntry& latest) noexcept;
    void apply_frequency(uint64_t now_us) noexcept;
    void schedule_after_sample(int64_t error_us, uint32_t now_ms) noexcept;

    SocketProvider& sockets_;
    SntpConfig config_;
    SntpClock clock_;

    std::array<FilterEntry, 8> filter_{};
    uint8_t filter_count_ = 0u;
    uint8_t filter_next_ = 0u;
    FilterEntry last_used_{};

    bool synced_ = false;
    bool started_ = false;
    uint32_t samples_ = 0u;
    uint8_t server_index_ = 0u;
    uint32_t next_exchange_ms_ = 0u;
    uint32_t poll_interval_ms_ = 0u;
    double frequency_ppm_ = 0.0;
    uint64_t frequency_applied_us_ = 0u;
    double frequency_carry_us_ = 0.0;  ///< Correction not yet applied, below one microsecond.
};

}  // namespace rosette::net
