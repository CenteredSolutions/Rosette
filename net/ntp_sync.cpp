#include "ntp_sync.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "pico/stdlib.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace milkdds::net {

namespace {

/**
 * @brief NTP client mode packet flags for VN=4, Mode=3.
 *
 * This is the minimal request form defined by the NTPv4 wire format. The
 * request carries only the transmit timestamp; the server fills the receive
 * and transmit timestamps used in the four-timestamp exchange.
 *
 * Reference:
 * - RFC 5905 section 7.3: leap/version/mode field layout.
 */
constexpr uint8_t kNtpClientFlags = 0x23u;

/**
 * @brief Size in bytes of the fixed NTPv4 packet header.
 *
 * Reference:
 * - RFC 5905 section 7.3: base packet format is 48 octets.
 */
constexpr std::size_t kNtpPacketSize = 48u;

/**
 * @brief Seconds between the NTP and Unix epochs.
 *
 * NTP timestamps count from 1900-01-01, while the rest of the service uses
 * Unix microseconds from 1970-01-01.
 *
 * Reference:
 * - RFC 5905 section 6: NTP timestamp date origin.
 */
constexpr uint64_t kNtpUnixOffsetSeconds = 2208988800ull;

/**
 * @brief Packed representation of the RFC 5905 wire packet.
 *
 * Only the fields needed for a basic client exchange are interpreted by this
 * implementation. The transmit timestamp in the request becomes T1, and the
 * server-provided receive/transmit timestamps become T2 and T3.
 *
 * Reference:
 * - RFC 5905 section 7.3: packet header fields and timestamp positions.
 */
struct __attribute__((packed)) NtpPacket {
    uint8_t flags;
    uint8_t stratum;
    uint8_t poll;
    int8_t precision;
    uint32_t root_delay;
    uint32_t root_dispersion;
    uint32_t ref_id;
    uint32_t ref_ts_sec;
    uint32_t ref_ts_frac;
    uint32_t orig_ts_sec;
    uint32_t orig_ts_frac;
    uint32_t rx_ts_sec;
    uint32_t rx_ts_frac;
    uint32_t tx_ts_sec;
    uint32_t tx_ts_frac;
};

static_assert(sizeof(NtpPacket) == kNtpPacketSize, "NTP packet must be 48 bytes");

/**
 * @brief Convert a 32-bit NTP field between host and big-endian order.
 *
 * The NTP wire format is network byte order, so every 32-bit integer field in
 * the packet must be swapped on little-endian MCUs before arithmetic is done.
 *
 * @param value 32-bit host or network ordered value.
 * @return The same bit pattern with byte order reversed.
 */
inline uint32_t bswap32(uint32_t value) noexcept
{
    return __builtin_bswap32(value);
}

/**
 * @brief Convert an NTP fixed-point timestamp to Unix microseconds.
 *
 * This decodes the 64-bit NTP timestamp split into seconds and fractional
 * seconds, converts it from the 1900 epoch to the Unix epoch, then scales the
 * fractional field into microseconds for the local discipline loop.
 *
 * @param sec_be Network-order NTP seconds field.
 * @param frac_be Network-order NTP fractional-seconds field.
 * @return Signed microseconds since the Unix epoch.
 *
 * Reference:
 * - RFC 5905 section 6: 64-bit timestamp format.
 */
int64_t ntp_ts_to_unix_us(uint32_t sec_be, uint32_t frac_be) noexcept
{
    const uint32_t seconds = bswap32(sec_be);
    const uint32_t fraction = bswap32(frac_be);
    const int64_t unix_seconds =
        static_cast<int64_t>(seconds) - static_cast<int64_t>(kNtpUnixOffsetSeconds);
    const int64_t fraction_us =
        static_cast<int64_t>((static_cast<uint64_t>(fraction) * 1000000ull) >> 32);
    return unix_seconds * 1000000ll + fraction_us;
}

/**
 * @brief Encode Unix microseconds as an NTP seconds/fraction pair.
 *
 * The client stamps T1 into the request transmit timestamp so the server can
 * echo it back as the originate timestamp and the client can solve the RFC
 * offset and delay equations on reply.
 *
 * @param unix_us Signed microseconds since the Unix epoch.
 * @param[out] sec_be Encoded network-order NTP seconds field.
 * @param[out] frac_be Encoded network-order NTP fractional-seconds field.
 *
 * Reference:
 * - RFC 5905 section 6: timestamp representation.
 */
void unix_us_to_ntp_ts(int64_t unix_us, uint32_t& sec_be, uint32_t& frac_be) noexcept
{
    int64_t seconds = unix_us / 1000000ll;
    int64_t micros = unix_us % 1000000ll;
    if (micros < 0) {
        --seconds;
        micros += 1000000ll;
    }

    const uint32_t ntp_seconds =
        static_cast<uint32_t>(seconds + static_cast<int64_t>(kNtpUnixOffsetSeconds));
    const uint32_t ntp_fraction =
        static_cast<uint32_t>((static_cast<uint64_t>(static_cast<uint32_t>(micros)) << 32) /
                              1000000ull);

    sec_be = bswap32(ntp_seconds);
    frac_be = bswap32(ntp_fraction);
}

/**
 * @brief Compute the absolute value of a signed 64-bit quantity.
 *
 * This keeps the sync logic simple when comparing phase error against step and
 * poll-backoff thresholds.
 *
 * @param value Signed input value.
 * @return Absolute magnitude of `value`.
 */
int64_t abs_i64(int64_t value) noexcept
{
    return value < 0 ? -value : value;
}

}  // namespace

/**
 * @brief Construct the service with the supplied sync policy and task settings.
 *
 * The configuration captures the poll interval, filter depth, phase/frequency
 * gains, and optional event-group integration used by the background loop.
 *
 * @param config Runtime configuration for this service instance.
 */
NtpSyncService::NtpSyncService(SocketProvider& socket_provider,
                               const NtpSyncConfig& config) noexcept
    : socket_provider_(socket_provider)
    , config_(config)
{
}

/**
 * @brief Start the FreeRTOS task that maintains the software UTC estimate.
 *
 * The task periodically exchanges timestamps with the configured NTP server,
 * chooses the lowest-delay sample, applies a phase correction, and slews the
 * offset between polls to account for oscillator error.
 *
 * @return `true` if the task is already running or was created successfully;
 * otherwise `false`.
 */
bool NtpSyncService::start() noexcept
{
    if (task_handle_ != nullptr) {
        return true;
    }

    const BaseType_t rc = xTaskCreate(task_entry,
                                      config_.task_name,
                                      config_.task_stack_words,
                                      this,
                                      config_.task_priority,
                                      &task_handle_);
    return rc == pdPASS;
}

void NtpSyncService::set_server(const SocketAddressV4& server) noexcept
{
    taskENTER_CRITICAL();
    config_.server_ip = server.address;
    config_.server_port = server.port;
    taskEXIT_CRITICAL();
}

/**
 * @brief Return the currently disciplined UTC offset.
 *
 * The monotonic hardware timer is not rewritten. Instead, the service keeps a
 * software correction term that is added to `time_us_64()` whenever UTC is
 * requested.
 *
 * @return Signed offset in microseconds to add to the local monotonic clock.
 */
int64_t NtpSyncService::offset_us() const noexcept
{
    return read_offset();
}

/**
 * @brief Report whether the service has accepted at least one valid sample.
 *
 * The first successful discipline step initializes the UTC offset and enables
 * frequency slewing between later polls.
 *
 * @return `true` once the service has synchronized at least once.
 */
bool NtpSyncService::is_synced() const noexcept
{
    return synced_;
}

/**
 * @brief Return the current best UTC estimate in Unix microseconds.
 *
 * This is the sum of the local monotonic microsecond counter and the current
 * disciplined UTC offset maintained by the NTP loop.
 *
 * @return Unix time in microseconds.
 */
int64_t NtpSyncService::current_time_us() const noexcept
{
    return static_cast<int64_t>(time_us_64()) + read_offset();
}

/**
 * @brief FreeRTOS trampoline that forwards into the instance task body.
 *
 * @param arg Pointer to the owning `NtpSyncService`.
 */
void NtpSyncService::task_entry(void* arg) noexcept
{
    static_cast<NtpSyncService*>(arg)->task_main();
}

/**
 * @brief Main polling and discipline loop.
 *
 * The loop optionally waits for network readiness, performs one exchange per
 * poll interval, feeds the sample into a minimum-delay filter, disciplines the
 * software clock with the selected sample, and applies small frequency slews
 * once per second between polls. Polling backs off only after the residual
 * phase error is consistently small.
 *
 * References:
 * - RFC 5905 section 10: clock filter and poll process concepts.
 */
void NtpSyncService::task_main() noexcept
{
    if ((config_.ready_events != nullptr) && (config_.ready_mask != 0u)) {
        xEventGroupWaitBits(config_.ready_events,
                            config_.ready_mask,
                            pdFALSE,
                            pdFALSE,
                            portMAX_DELAY);
    }

    uint32_t poll_ms = config_.min_poll_ms;
    uint32_t sync_count = 0u;

    for (;;) {
        int64_t theta_us = 0;
        int64_t delay_us = 0;
        const bool ok = exchange(theta_us, delay_us);

        if (ok && (delay_us >= 0)) {
            push_sample(theta_us, delay_us);

            int64_t best_offset = 0;
            int64_t best_delay = 0;
            uint32_t best_age_ms = 0u;
            if (best_sample(best_offset, best_delay, best_age_ms)) {
                const int64_t local_offset_before = read_offset();
                const int64_t theta_delta_us = theta_us - local_offset_before;
                const int64_t best_delta_us = best_offset - local_offset_before;
                const int64_t error = discipline(best_offset);
                const int64_t local_offset_after = read_offset();
                const int64_t corrected_delta_us = best_offset - local_offset_after;

                if ((config_.notify_events != nullptr) && (config_.synced_bit != 0u)) {
                    xEventGroupSetBits(config_.notify_events, config_.synced_bit);
                }

                if ((sync_count < 8u) ||
                    (abs_i64(error) > (config_.step_threshold_us / 2))) {
                    poll_ms = config_.min_poll_ms;
                } else if (poll_ms < config_.max_poll_ms) {
                    poll_ms = (poll_ms <= (config_.max_poll_ms / 2u))
                        ? (poll_ms * 2u)
                        : config_.max_poll_ms;
                }

                ++sync_count;

                std::printf("[ntp] current_delta=%lld us "
                            "new_sample_delta=%lld us new_delay=%lld us "
                            "best_delta=%lld us best_delay=%lld us best_age=%lu ms "
                            "corrected_delta=%lld us freq=%.3f us/s\n",
                            static_cast<long long>(error),
                            static_cast<long long>(theta_delta_us),
                            static_cast<long long>(delay_us),
                            static_cast<long long>(best_delta_us),
                            static_cast<long long>(best_delay),
                            static_cast<unsigned long>(best_age_ms),
                            static_cast<long long>(corrected_delta_us),
                            freq_us_per_s_);
                std::printf("[ntp] offsets local_before=%lld us local_after=%lld us "
                            "sample_abs=%lld us best_abs=%lld us\n",
                            static_cast<long long>(local_offset_before),
                            static_cast<long long>(local_offset_after),
                            static_cast<long long>(theta_us),
                            static_cast<long long>(best_offset));
            }
        } else {
            poll_ms = config_.min_poll_ms;
            std::printf("[ntp] exchange failed, retrying in %lu ms\n",
                        static_cast<unsigned long>(poll_ms));
        }

        uint32_t remaining = poll_ms;
        while (remaining > 0u) {
            const uint32_t tick = remaining > 1000u ? 1000u : remaining;
            vTaskDelay(pdMS_TO_TICKS(tick));
            remaining -= tick;
            apply_freq_correction();
        }
    }
}

/**
 * @brief Perform one complete client/server NTP exchange over UDP.
 *
 * T1 is sampled immediately before transmit using the local monotonic clock
 * plus the current software UTC offset. T4 is sampled immediately after the
 * receive call returns to minimize host-side latency bias. The reply provides
 * T2 and T3. From those four timestamps the function computes:
 * - theta = ((T2 - T1) + (T3 - T4)) / 2
 * - delta = (T4 - T1) - (T3 - T2)
 *
 * `theta_us` is the phase correction the client should apply so its UTC
 * estimate aligns with the server, while `delay_us` is used to reject noisier
 * samples in favor of the lowest-delay measurement.
 *
 * @param[out] theta_us Estimated client clock offset in microseconds.
 * @param[out] delay_us Estimated round-trip network delay in microseconds.
 * @return `true` if a valid server reply was received and parsed; otherwise
 * `false`.
 *
 * References:
 * - RFC 5905 section 8: on-wire timestamp exchange.
 * - RFC 5905 section 8, equations for clock offset and round-trip delay.
 */
bool NtpSyncService::exchange(int64_t& theta_us, int64_t& delay_us) noexcept
{
    auto socket = socket_provider_.open(SocketDomain::ipv4,
                                        SocketType::datagram,
                                        SocketProtocol::udp);
    if (!socket) {
        return false;
    }

    const SocketAddressV4 bind_addr{
        .address = {0u, 0u, 0u, 0u},
        .port = config_.local_port,
    };
    if (socket->bind(bind_addr) != 0) {
        return false;
    }

    if (socket->set_receive_timeout(config_.receive_timeout_ms) != 0) {
        return false;
    }

    NtpPacket request{};
    request.flags = kNtpClientFlags;

    const uint64_t t1_hw_us = time_us_64();
    const int64_t t1_unix_us = static_cast<int64_t>(t1_hw_us) + read_offset();
    uint32_t tx_ts_sec = 0u;
    uint32_t tx_ts_frac = 0u;
    unix_us_to_ntp_ts(t1_unix_us, tx_ts_sec, tx_ts_frac);
    request.tx_ts_sec = tx_ts_sec;
    request.tx_ts_frac = tx_ts_frac;

    SocketAddressV4 server{};
    taskENTER_CRITICAL();
    server.address = config_.server_ip;
    server.port = config_.server_port;
    taskEXIT_CRITICAL();

    const std::ptrdiff_t sent = socket->send_to(&request, sizeof(request), server);
    if (sent != static_cast<std::ptrdiff_t>(sizeof(request))) {
        return false;
    }

    NtpPacket response{};
    SocketAddressV4 from{};
    const std::ptrdiff_t received = socket->recv_from(&response,
                                                      sizeof(response),
                                                      &from);
    const uint64_t t4_hw_us = time_us_64();

    if (received < static_cast<std::ptrdiff_t>(sizeof(response))) {
        return false;
    }

    if ((from.address != server.address) || (from.port != server.port)) {
        return false;
    }

    const uint8_t mode = response.flags & 0x07u;
    const uint8_t version = (response.flags >> 3) & 0x07u;
    if ((mode != 4u) || (version < 3u) || (response.stratum == 0u)) {
        return false;
    }

    const int64_t t2_unix_us = ntp_ts_to_unix_us(response.rx_ts_sec, response.rx_ts_frac);
    const int64_t t3_unix_us = ntp_ts_to_unix_us(response.tx_ts_sec, response.tx_ts_frac);

    theta_us = ((t2_unix_us - static_cast<int64_t>(t1_hw_us)) +
                (t3_unix_us - static_cast<int64_t>(t4_hw_us))) / 2ll;
    delay_us = (static_cast<int64_t>(t4_hw_us - t1_hw_us)) - (t3_unix_us - t2_unix_us);
    return true;
}

/**
 * @brief Insert one measured offset/delay pair into the filter window.
 *
 * The filter is a bounded circular buffer. Later selection prefers the sample
 * with the smallest round-trip delay because it is least distorted by queueing
 * and path asymmetry.
 *
 * @param offset_us Measured RFC clock offset in microseconds.
 * @param delay_us Measured round-trip delay in microseconds.
 *
 * Reference:
 * - RFC 5905 section 10: clock filter retains recent samples and prefers
 *   lower-delay observations.
 */
void NtpSyncService::push_sample(int64_t offset_us, int64_t delay_us) noexcept
{
    const uint8_t limit = std::min<uint8_t>(config_.filter_size,
                                            static_cast<uint8_t>(filter_offset_us_.size()));
    if (limit == 0u) {
        return;
    }

    filter_offset_us_[filter_index_] = offset_us;
    filter_delay_us_[filter_index_] = delay_us;
    filter_sample_ms_[filter_index_] = static_cast<uint32_t>(
        pdTICKS_TO_MS(xTaskGetTickCount()));
    filter_index_ = static_cast<uint8_t>((filter_index_ + 1u) % limit);
    if (filter_count_ < limit) {
        ++filter_count_;
    }
}

/**
 * @brief Choose the lowest-delay sample currently held in the filter.
 *
 * This implements the simplest and most important part of the NTP clock
 * filter: lower delay usually means less variable path latency, so the
 * corresponding phase measurement is the best candidate to discipline the
 * local clock.
 *
 * @param[out] offset_us Best current offset estimate in microseconds.
 * @param[out] delay_us Delay associated with the chosen sample.
 * @return `true` if at least one sample is available; otherwise `false`.
 *
 * Reference:
 * - RFC 5905 section 10: filter selection by minimum delay.
 */
bool NtpSyncService::best_sample(int64_t& offset_us,
                                 int64_t& delay_us,
                                 uint32_t& age_ms) const noexcept
{
    if (filter_count_ == 0u) {
        return false;
    }

    uint8_t best_index = 0u;
    int64_t best_delay = filter_delay_us_[0];
    for (uint8_t i = 1u; i < filter_count_; ++i) {
        if (filter_delay_us_[i] < best_delay) {
            best_delay = filter_delay_us_[i];
            best_index = i;
        }
    }

    offset_us = filter_offset_us_[best_index];
    delay_us = filter_delay_us_[best_index];
    age_ms = static_cast<uint32_t>(pdTICKS_TO_MS(xTaskGetTickCount())) -
             filter_sample_ms_[best_index];
    return true;
}

/**
 * @brief Apply one phase/frequency discipline update from the best sample.
 *
 * The service first applies any outstanding frequency slew so the current
 * software offset reflects elapsed time since the previous update. It then
 * compares the measured server offset with the current disciplined offset:
 * - large errors are stepped immediately;
 * - smaller errors are corrected by `phase_gain`.
 *
 * Unlike the older C implementation, the default phase gain here is 1.0, so a
 * good sample pulls the software UTC estimate essentially onto the server time
 * instead of intentionally leaving most of the phase error behind.
 *
 * Frequency error is estimated from the change in measured offset across the
 * elapsed poll interval and then smoothed and clamped. That frequency term is
 * later integrated by `apply_freq_correction()` to keep the clock close to UTC
 * between NTP polls.
 *
 * @param measured_offset_us Selected best offset sample in microseconds.
 * @return Residual phase error observed before the new correction was applied.
 *
 * References:
 * - RFC 5905 section 10: clock discipline and poll/frequency behavior.
 */
int64_t NtpSyncService::discipline(int64_t measured_offset_us) noexcept
{
    const uint64_t now_hw_us = time_us_64();
    apply_freq_correction();

    const int64_t current_offset_us = read_offset();
    const int64_t error_us = measured_offset_us - current_offset_us;

    if (!synced_) {
        write_offset(measured_offset_us);
        synced_ = true;
        last_measured_offset_us_ = measured_offset_us;
        last_sync_hw_us_ = now_hw_us;
        last_freq_hw_us_ = now_hw_us;
        return error_us;
    }

    if (abs_i64(error_us) > config_.step_threshold_us) {
        write_offset(measured_offset_us);
    } else {
        const int64_t phase_adjust_us =
            static_cast<int64_t>(static_cast<double>(error_us) * config_.phase_gain);
        write_offset(current_offset_us + phase_adjust_us);
    }

    if (last_sync_hw_us_ != 0u) {
        const double dt_s = static_cast<double>(now_hw_us - last_sync_hw_us_) * 1e-6;
        if (dt_s > 1.0) {
            const double observed_freq =
                static_cast<double>(measured_offset_us - last_measured_offset_us_) / dt_s;

            // Smooth frequency changes; do not leave a large phase residue after sync.
            freq_us_per_s_ += (observed_freq - freq_us_per_s_) * config_.freq_gain;
            freq_us_per_s_ = std::clamp(freq_us_per_s_,
                                        -config_.max_freq_correction_us_per_s,
                                        config_.max_freq_correction_us_per_s);
        }
    }

    last_measured_offset_us_ = measured_offset_us;
    last_sync_hw_us_ = now_hw_us;
    last_freq_hw_us_ = now_hw_us;
    return error_us;
}

/**
 * @brief Integrate the estimated oscillator error into the UTC offset.
 *
 * This is the continuous part of the discipline loop. Between explicit NTP
 * exchanges, the service advances the software UTC correction by
 * `freq_us_per_s_ * dt`, which slews the estimate in the direction needed to
 * cancel local crystal drift.
 *
 * @return No value.
 */
void NtpSyncService::apply_freq_correction() noexcept
{
    if (!synced_ || (freq_us_per_s_ == 0.0)) {
        return;
    }

    const uint64_t now_hw_us = time_us_64();
    if (last_freq_hw_us_ == 0u) {
        last_freq_hw_us_ = now_hw_us;
        return;
    }

    const uint64_t dt_hw_us = now_hw_us - last_freq_hw_us_;
    last_freq_hw_us_ = now_hw_us;

    const int64_t correction_us =
        static_cast<int64_t>(freq_us_per_s_ * (static_cast<double>(dt_hw_us) * 1e-6));
    if (correction_us != 0) {
        write_offset(read_offset() + correction_us);
    }
}

/**
 * @brief Atomically update the disciplined UTC offset.
 *
 * Readers may fetch the offset from other tasks, so this uses a short critical
 * section to keep the 64-bit value coherent on 32-bit MCUs.
 *
 * @param value New UTC offset in microseconds.
 */
void NtpSyncService::write_offset(int64_t value) noexcept
{
    taskENTER_CRITICAL();
    offset_us_ = value;
    taskEXIT_CRITICAL();
}

/**
 * @brief Atomically read the disciplined UTC offset.
 *
 * @return Current UTC offset in microseconds.
 */
int64_t NtpSyncService::read_offset() const noexcept
{
    taskENTER_CRITICAL();
    const int64_t value = offset_us_;
    taskEXIT_CRITICAL();
    return value;
}

}  // namespace milkdds::net
