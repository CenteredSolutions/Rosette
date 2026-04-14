#pragma once

#include <array>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "event_groups.h"
}

#include "socket.hpp"

namespace milkdds::net {

/**
 * @brief Configuration for the software NTP discipline loop.
 *
 * The defaults are chosen for a small embedded client that synchronizes over
 * ordinary Ethernet without hardware timestamping. The service computes the
 * RFC 5905 clock offset and delay terms from each exchange, filters samples by
 * minimum round-trip delay, then disciplines a software UTC offset that is
 * added to the local monotonic hardware timer.
 *
 * References:
 * - RFC 5905 section 8: on-wire packet fields and timestamp exchange.
 * - RFC 5905 section 10: clock filter and clock discipline concepts.
 */
struct NtpSyncConfig {
    std::array<uint8_t, 4> server_ip{ {216, 239, 35, 0} };
    uint16_t server_port = 123u;
    uint16_t local_port = 12300u;
    uint32_t receive_timeout_ms = 5000u;
    uint32_t min_poll_ms = 64000u;
    uint32_t max_poll_ms = 1024000u;
    int64_t step_threshold_us = 128000ll;

    // The old code used a low phase gain (~1/8), which intentionally leaves
    // large residual phase error after each sync. Default to full phase
    // correction and reserve smoothing for the frequency estimator.
    double phase_gain = 1.0;
    double freq_gain = 0.0625;
    double max_freq_correction_us_per_s = 500.0;

    uint8_t filter_size = 8u;
    uint16_t task_stack_words = 1024u;
    UBaseType_t task_priority = tskIDLE_PRIORITY + 1u;
    const char* task_name = "ntp";

    EventGroupHandle_t ready_events = nullptr;
    EventBits_t ready_mask = 0u;
    EventGroupHandle_t notify_events = nullptr;
    EventBits_t synced_bit = 0u;
};

class NtpSyncService {
public:
    /**
     * @brief Construct an NTP synchronization service.
     *
     * @param config Runtime configuration for polling, filtering, event
     * notification, and control-loop gains.
     */
    explicit NtpSyncService(SocketProvider& socket_provider,
                            const NtpSyncConfig& config = {}) noexcept;

    /**
     * @brief Start the background FreeRTOS task that performs NTP exchanges.
     *
     * The task waits for any configured network-ready event bits, polls the
     * configured server, updates the software UTC offset, and applies frequency
     * correction between polls.
     *
     * @return `true` if the task was created or was already running, `false`
     * if FreeRTOS task creation failed.
     */
    bool start() noexcept;

    /**
     * @brief Update the remote NTP server endpoint used for future polls.
     *
     * This can be used to adopt DHCP-advertised NTP servers after the service
     * has been constructed.
     *
     * @param server IPv4 address and UDP port of the remote NTP server.
     */
    void set_server(const SocketAddressV4& server) noexcept;

    /**
     * @brief Read the current software UTC correction in microseconds.
     *
     * This value is added to the hardware monotonic microsecond counter to
     * produce the current best UTC estimate.
     *
     * @return Signed offset in microseconds.
     */
    int64_t offset_us() const noexcept;

    /**
     * @brief Report whether at least one successful synchronization occurred.
     *
     * @return `true` after the first accepted NTP sample has been disciplined
     * into the software clock.
     */
    bool is_synced() const noexcept;

    /**
     * @brief Return the current UTC estimate in microseconds since the Unix epoch.
     *
     * @return Current UTC time expressed as Unix-epoch microseconds.
     */
    int64_t current_time_us() const noexcept;

private:
    /**
     * @brief FreeRTOS task trampoline.
     *
     * @param arg Opaque pointer to the owning `NtpSyncService` instance.
     */
    static void task_entry(void* arg) noexcept;

    /**
     * @brief Background task body.
     *
     * Waits for network readiness, periodically performs one NTP exchange,
     * filters the resulting samples, disciplines the software clock, and
     * applies frequency correction between polls.
     */
    void task_main() noexcept;

    /**
     * @brief Perform one NTP client/server exchange.
     *
     * Builds an RFC 5905 client request, timestamps T1 before transmit and T4
     * immediately after receive, then computes:
     * - theta = ((T2 - T1) + (T3 - T4)) / 2
     * - delta = (T4 - T1) - (T3 - T2)
     *
     * @param[out] theta_us Estimated clock offset in microseconds. Positive
     * means the local clock is behind the server.
     * @param[out] delay_us Estimated round-trip delay in microseconds.
     * @return `true` on a valid NTP server reply, otherwise `false`.
     */
    bool exchange(int64_t& theta_us, int64_t& delay_us) noexcept;

    /**
     * @brief Insert a new offset/delay sample into the clock filter.
     *
     * @param offset_us Measured clock offset in microseconds.
     * @param delay_us Measured round-trip delay in microseconds.
     */
    void push_sample(int64_t offset_us, int64_t delay_us) noexcept;

    /**
     * @brief Select the current best sample from the filter window.
     *
     * Following RFC 5905 clock-filter practice, the best sample is chosen as
     * the one with the minimum round-trip delay.
     *
     * @param[out] offset_us Best offset sample in microseconds.
     * @param[out] delay_us Delay associated with the best sample.
     * @param[out] age_ms Age of the chosen sample in milliseconds.
     * @return `true` if the filter contains at least one sample.
     */
    bool best_sample(int64_t& offset_us,
                     int64_t& delay_us,
                     uint32_t& age_ms) const noexcept;

    /**
     * @brief Apply one phase/frequency discipline step to the software clock.
     *
     * Large phase errors are stepped immediately. Smaller errors are corrected
     * using the configured phase gain so the software clock stays close to the
     * server rather than intentionally lagging behind it. Frequency error is
     * estimated from the change in measured offset over time and then smoothed.
     *
     * @param measured_offset_us Best current offset estimate in microseconds.
     * @return Residual phase error prior to the current discipline update.
     */
    int64_t discipline(int64_t measured_offset_us) noexcept;

    /**
     * @brief Apply accumulated frequency correction between poll cycles.
     *
     * This continuously slews the software UTC offset based on the estimated
     * oscillator error, helping the clock remain close to true time between
     * explicit NTP exchanges.
     */
    void apply_freq_correction() noexcept;

    /**
     * @brief Store the current UTC offset atomically with respect to readers.
     *
     * @param value New UTC offset in microseconds.
     */
    void write_offset(int64_t value) noexcept;

    /**
     * @brief Read the current UTC offset atomically.
     *
     * @return Current UTC offset in microseconds.
     */
    int64_t read_offset() const noexcept;

private:
    SocketProvider& socket_provider_;
    NtpSyncConfig config_;
    TaskHandle_t task_handle_ = nullptr;

    mutable int64_t offset_us_ = 0;
    bool synced_ = false;
    double freq_us_per_s_ = 0.0;
    int64_t last_measured_offset_us_ = 0;
    uint64_t last_sync_hw_us_ = 0;
    uint64_t last_freq_hw_us_ = 0;

    std::array<int64_t, 8> filter_offset_us_{};
    std::array<int64_t, 8> filter_delay_us_{};
    std::array<uint32_t, 8> filter_sample_ms_{};
    uint8_t filter_index_ = 0;
    uint8_t filter_count_ = 0;
};

}  // namespace milkdds::net
