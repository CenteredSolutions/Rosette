/**
 * @file src/net/sntp_client.cpp
 * @brief SntpClient: one bounded exchange per due poll, a distance filter, phase and frequency
 * discipline.
 *
 * See net/sntp_client.hpp for the discipline rules. The offset the client
 * measures and writes is always "Unix time minus the monotonic tick", the
 * same quantity WallClock stores, so a sample, the filter, and the clock
 * never convert between time scales.
 */

#include "rosette/net/sntp_client.hpp"

#include <cstdio>

namespace rosette::net {
namespace {

/// RFC 5905 §7.2 PHI: the dispersion a sample gains per second of age, as a fraction.
constexpr double kDispersionRate = 15e-6;

/// Samples at the start that always keep the poll interval at its minimum.
constexpr uint32_t kSettlingSamples = 8u;

bool due(uint32_t deadline_ms, uint32_t now_ms) noexcept {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

int64_t magnitude(int64_t v) noexcept {
    return v < 0 ? -v : v;
}

bool same_endpoint(const SocketAddress& a, const SocketAddress& b) noexcept {
    return a.domain == b.domain && a.address == b.address && a.port == b.port;
}

void print_server(const char* what, const SocketAddress& server) noexcept {
    std::printf("[ntp] %s %u.%u.%u.%u\n", what, server.address[0], server.address[1],
                server.address[2], server.address[3]);
}

}  // namespace

SntpClient::SntpClient(SocketProvider& sockets, const SntpConfig& config, SntpClock clock) noexcept
    : sockets_(sockets), config_(config), clock_(clock), poll_interval_ms_(config.min_poll_ms) {}

void SntpClient::poll(uint32_t now_ms, const AddressSnapshot& snapshot) noexcept {
    apply_frequency(clock_.monotonic_us());
    if (!config_.enabled || !snapshot.v4.has_address()) {
        return;
    }

    if (started_ && !due(next_exchange_ms_, now_ms)) {
        return;
    }

    started_ = true;
    SocketAddress server{};
    if (!pick_server(snapshot, server)) {
        next_exchange_ms_ = now_ms + config_.retry_ms;
        return;
    }

    FilterEntry latest{};
    const Exchange result = exchange(server, latest);

    // A request dropped before the wire reached no server: ask the same one again soon, once.
    const bool quick_retry = result == Exchange::not_sent && !retried_unsent_;
    retried_unsent_ = quick_retry;
    if (quick_retry) {
        next_exchange_ms_ = now_ms + config_.unsent_retry_ms;
        print_server("not sent yet to", server);
        return;
    }

    if (result == Exchange::sampled) {
        ++samples_;
        const bool was_synced = synced_;
        const int64_t error_us = discipline(latest);
        schedule_after_sample(error_us, now_ms);
        if (!was_synced) {
            print_server("clock set from", server);
        } else {
            std::printf(
                "[ntp] offset error %lld us, delay %lld us, drift %.2f ppm, next in %lu s\n",
                static_cast<long long>(error_us), static_cast<long long>(latest.sample.delay_us),
                frequency_ppm_, static_cast<unsigned long>(poll_interval_ms_ / 1000u));
        }

        return;
    }

    // A server that did not answer, or asked to be left alone, gives way to the next one.
    ++server_index_;
    poll_interval_ms_ = result == Exchange::kiss_of_death ? config_.max_poll_ms : config_.retry_ms;
    next_exchange_ms_ = now_ms + poll_interval_ms_;
    print_server(result == Exchange::kiss_of_death ? "refused by"
                 : result == Exchange::not_sent    ? "not sent to"
                                                   : "no reply from",
                 server);
}

/**
 * @brief The server for this exchange: the configured one, then DHCP's in order, round robin.
 *
 * @param[in]  snapshot The network snapshot carrying DHCP option 42.
 * @param[out] server   The chosen server.
 * @return False when no server is configured or offered.
 */
bool SntpClient::pick_server(const AddressSnapshot& snapshot,
                             SocketAddress& server) const noexcept {
    const bool configured = config_.server.port != 0u;
    const uint8_t offered = snapshot.ntp_server_count;
    const uint8_t count = static_cast<uint8_t>(offered + (configured ? 1u : 0u));
    if (count == 0u) {
        return false;
    }

    uint8_t pick = static_cast<uint8_t>(server_index_ % count);
    if (configured) {
        if (pick == 0u) {
            server = config_.server;
            return true;
        }

        --pick;
    }

    server = make_socket_address_v4(snapshot.ntp_servers_v4[pick], kNtpServerPort);
    return true;
}

/**
 * @brief Sends one request to `server` and waits up to `reply_timeout_ms` for its answer.
 *
 * The request socket exists only for the exchange, so the client holds no
 * hardware socket between polls. T1 is read just before the send and T4 as
 * soon as a datagram is returned; datagrams from another endpoint, or that
 * fail the reply checks (including a stale reply to an earlier request),
 * are skipped and the wait continues until the deadline.
 *
 * @param[in]  server The server.
 * @param[out] entry  The sample and the tick time of T4, when sampled.
 * @return `sampled`; `not_sent` when the socket dropped the request before
 *   the wire (`would_block`), so no server saw it; `no_reply` (including a
 *   socket that could not be opened); or `kiss_of_death`.
 *
 * References:
 * - RFC 4330 §5 (request, reply checks); RFC 5905 §8 (T1..T4)
 */
SntpClient::Exchange SntpClient::exchange(const SocketAddress& server,
                                          FilterEntry& entry) noexcept {
    auto socket = sockets_.open(SocketDomain::ipv4);
    if (socket == nullptr
        || socket->bind(make_socket_address_v4({0u, 0u, 0u, 0u}, config_.local_port)) != 0) {
        return Exchange::no_reply;
    }

    std::array<uint8_t, kSntpPacketBytes> packet{};
    const uint64_t t1_us = clock_.monotonic_us();
    const NtpTimestamp sent = unix_to_ntp(static_cast<int64_t>(t1_us) + clock_.offset_us());
    encode_sntp_request(sent, packet);
    if (socket->send_to(packet.data(), packet.size(), server)
        != static_cast<std::ptrdiff_t>(packet.size())) {
        return Exchange::no_reply;
    }

    if (socket->last_error() == SocketError::would_block) {
        return Exchange::not_sent;
    }

    const uint64_t deadline_us = t1_us + uint64_t{config_.reply_timeout_ms} * 1000u;
    for (;;) {
        const uint64_t now_us = clock_.monotonic_us();
        if (now_us >= deadline_us) {
            return Exchange::no_reply;
        }

        (void)socket->set_receive_timeout(
            static_cast<uint32_t>((deadline_us - now_us + 999u) / 1000u));
        std::array<uint8_t, 64> reply_bytes{};
        SocketAddress from{};
        const std::ptrdiff_t received =
            socket->recv_from(reply_bytes.data(), reply_bytes.size(), &from);
        const uint64_t t4_us = clock_.monotonic_us();
        if (received < 0) {
            return Exchange::no_reply;
        }

        SntpReply reply{};
        const SntpReplyError error =
            same_endpoint(from, server)
                ? decode_sntp_reply(reply_bytes.data(), static_cast<std::size_t>(received), sent,
                                    reply)
                : SntpReplyError::wrong_origin;
        if (error == SntpReplyError::kiss_of_death) {
            return Exchange::kiss_of_death;
        }

        if (error != SntpReplyError::none) {
            continue;
        }

        entry.sample = solve_sntp_exchange(static_cast<int64_t>(t1_us), reply.receive_unix_us,
                                           reply.transmit_unix_us, static_cast<int64_t>(t4_us));
        entry.at_us = t4_us;
        return entry.sample.delay_us >= 0 ? Exchange::sampled : Exchange::no_reply;
    }
}

/**
 * @brief The filter entry with the smallest distance at `now_us`.
 *
 * Distance is half the round-trip delay plus the dispersion the sample has
 * gained with age (PHI = 15 ppm), so a slightly slower fresh sample beats a
 * fast one measured long ago.
 *
 * References:
 * - RFC 5905 §10 and Appendix A.5.2 (clock_filter)
 */
const SntpClient::FilterEntry& SntpClient::select_sample(uint64_t now_us) const noexcept {
    const FilterEntry* best = &filter_[0];
    double best_distance = 0.0;
    for (uint8_t i = 0u; i < filter_count_; ++i) {
        const FilterEntry& e = filter_[i];
        const double distance = static_cast<double>(e.sample.delay_us) / 2.0
                                + kDispersionRate * static_cast<double>(now_us - e.at_us);
        if (i == 0u || distance < best_distance) {
            best = &e;
            best_distance = distance;
        }
    }

    return *best;
}

/**
 * @brief Adds `latest` to the filter and, when the filter yields a new sample, sets the clock from
 * it.
 *
 * The first used sample sets the clock and anchors the frequency lock
 * (update_frequency()); every later one updates the frequency first. The
 * selected sample's offset is then carried from its measurement to now at
 * the estimated frequency before it is written.
 *
 * @param[in] latest The sample just measured.
 * @return The latest sample's offset minus the clock's offset before the
 *   update: how far off the clock was, which paces the polling.
 *
 * Verification:
 * - Correct means: the clock's offset is the selected sample's carried
 *   forward; a sample not newer than the last one used changes nothing.
 * - Covered by: TC-NET-SNTP-CLI-001, TC-NET-SNTP-CLI-003, TC-NET-SNTP-CLI-007.
 *
 * References:
 * - RFC 5905 §10 (a sample not newer than the last one used is not used)
 */
int64_t SntpClient::discipline(const FilterEntry& latest) noexcept {
    filter_[filter_next_] = latest;
    filter_next_ = static_cast<uint8_t>((filter_next_ + 1u) % filter_.size());
    if (filter_count_ < filter_.size()) {
        ++filter_count_;
    }

    const int64_t error_us = latest.sample.offset_us - clock_.offset_us();
    const uint64_t now_us = clock_.monotonic_us();
    const FilterEntry& best = select_sample(now_us);
    if (synced_ && best.at_us <= last_used_.at_us) {
        return error_us;
    }

    if (synced_) {
        update_frequency(best);
    } else {
        frequency_anchor_ = best;
    }

    const double carried_us = frequency_ppm_ * static_cast<double>(now_us - best.at_us) * 1e-6;
    clock_.set_offset_us(best.sample.offset_us + static_cast<int64_t>(carried_us));
    frequency_applied_us_ = now_us;
    frequency_carry_us_ = 0.0;
    last_used_ = best;
    synced_ = true;
    return error_us;
}

/**
 * @brief Updates the frequency estimate from a newly used sample.
 *
 * A frequency is an offset change over a span, so an error of e in either
 * sample's offset costs the measurement e / span. Until one measurement
 * spans `frequency_averaging_s`, the client is locking: each measurement
 * runs from the first sample used (the anchor) and replaces the estimate,
 * so the estimate tracks the tick from the second exchange and its error
 * shrinks as the span grows. A first sample 770 us off, as one taken while
 * the board is busy starting up can be, costs 12 ppm at 64 s, under 2 ppm
 * by the end of the `min_poll_ms` exchanges, and 0.75 ppm at 1024 s. Once
 * the span reaches `frequency_averaging_s` the lock is done, and each
 * measurement runs from the previous used sample and is folded in with a
 * weight of its span over `frequency_averaging_s`.
 *
 * RFC 5905's reference discipline makes the same long first measurement in
 * its FREQ state, but leaves the clock uncorrected until it is done (900 s,
 * 31 ms at 35 ppm); measuring from the anchor at every exchange corrects
 * the clock all the while.
 *
 * @param[in] best The sample being used, newer than the last one used.
 *
 * Verification:
 * - Correct means: while locking, the estimate is the offset change since
 *   the anchor over the time since it; once a span from the anchor reaches
 *   `frequency_averaging_s`, estimates move toward each measurement by its
 *   span's weight; the estimate never exceeds `max_frequency_ppm`.
 * - Covered by: TC-NET-SNTP-CLI-004, TC-NET-SNTP-CLI-008, TC-NET-SNTP-CLI-009.
 *
 * References:
 * - RFC 5905 §11.3 (frequency-lock contribution weighted by the update
 *   interval); Appendix A.5.5.1 (local_clock(): the FREQ state measures the
 *   frequency from the first offset over at least WATCH seconds)
 */
void SntpClient::update_frequency(const FilterEntry& best) noexcept {
    const FilterEntry& from = frequency_locked_ ? last_used_ : frequency_anchor_;
    const double span_s = static_cast<double>(best.at_us - from.at_us) * 1e-6;
    const double measured_ppm =
        static_cast<double>(best.sample.offset_us - from.sample.offset_us) / span_s;
    double weight = 1.0;
    if (frequency_locked_ && span_s < config_.frequency_averaging_s) {
        weight = span_s / config_.frequency_averaging_s;
    }

    frequency_ppm_ += (measured_ppm - frequency_ppm_) * weight;
    frequency_locked_ = frequency_locked_ || span_s >= config_.frequency_averaging_s;
    if (frequency_ppm_ > config_.max_frequency_ppm) {
        frequency_ppm_ = config_.max_frequency_ppm;
    } else if (frequency_ppm_ < -config_.max_frequency_ppm) {
        frequency_ppm_ = -config_.max_frequency_ppm;
    }
}

/**
 * @brief Advances the clock's offset by the estimated drift since the last call.
 *
 * Corrections below a microsecond accumulate in a carry, so frequent calls
 * lose nothing to truncation.
 *
 * @param[in] now_us The tick now.
 */
void SntpClient::apply_frequency(uint64_t now_us) noexcept {
    if (!synced_ || frequency_ppm_ == 0.0) {
        frequency_applied_us_ = now_us;
        return;
    }

    frequency_carry_us_ +=
        frequency_ppm_ * static_cast<double>(now_us - frequency_applied_us_) * 1e-6;
    frequency_applied_us_ = now_us;
    const int64_t whole_us = static_cast<int64_t>(frequency_carry_us_);
    if (whole_us != 0) {
        frequency_carry_us_ -= static_cast<double>(whole_us);
        clock_.set_offset_us(clock_.offset_us() + whole_us);
    }
}

/**
 * @brief Sets the next exchange: minimum interval while settling or off, else double up to the
 * maximum.
 *
 * @param[in] error_us How far off the clock was at this exchange.
 * @param[in] now_ms   Monotonic milliseconds.
 */
void SntpClient::schedule_after_sample(int64_t error_us, uint32_t now_ms) noexcept {
    if (samples_ <= kSettlingSamples || magnitude(error_us) > config_.settled_error_us) {
        poll_interval_ms_ = config_.min_poll_ms;
    } else if (poll_interval_ms_ < config_.max_poll_ms) {
        poll_interval_ms_ = poll_interval_ms_ <= config_.max_poll_ms / 2u ? poll_interval_ms_ * 2u
                                                                          : config_.max_poll_ms;
    }

    next_exchange_ms_ = now_ms + poll_interval_ms_;
}

}  // namespace rosette::net
