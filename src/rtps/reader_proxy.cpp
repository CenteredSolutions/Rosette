// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file reader_proxy.cpp
 * @brief ReaderProxy change tracking as sequence-number bounds (§8.4.1.1, §8.4.6).
 *
 * See rtps/reader_proxy.hpp. The proxy's state is a handful of sequence
 * numbers and a 64-bit requested window; queries that depend on which
 * changes exist scan the writer's history, which holds at most its KEEP_LAST
 * depth.
 */

#include "rosette/rtps/reader_proxy.hpp"

namespace rosette::rtps {

void ReaderProxy::init(const Guid& reader_guid, const Locator* unicast_locs, uint8_t unicast_count,
                       bool expects_inline_qos, const SequenceNumber& first_relevant_sn,
                       bool reliable) noexcept {
    clear();
    remoteReaderGuid = reader_guid;
    expectsInlineQos = expects_inline_qos;
    isActive         = true;
    isReliable = reliable;
    unicastLocator = preferred_unicast_locator(unicast_locs, unicast_count);
    firstRelevantSN_ = first_relevant_sn;
    highestSentSN_ = first_relevant_sn - 1;
}

void ReaderProxy::restart(const SequenceNumber& first_relevant_sn) noexcept {
    replaying = false;
    highestAckedSN = SEQUENCENUMBER_ZERO;
    lastAcknackCount = kNoCount;
    lastNackFragCount = kNoCount;
    nackPending = false;
    nackPendingSinceMs = 0;
    requestedBase_ = SEQUENCENUMBER_ZERO;
    requestedBits_ = 0u;
    firstRelevantSN_ = first_relevant_sn;
    highestSentSN_ = first_relevant_sn - 1;
}

void ReaderProxy::clear() noexcept {
    remoteReaderGuid    = GUID_UNKNOWN;
    expectsInlineQos    = false;
    isActive            = false;
    unicastLocator = nullptr;
    isReliable = true;
    replaying = false;
    highestAckedSN      = SEQUENCENUMBER_ZERO;
    lastAcknackCount = kNoCount;
    lastNackFragCount = kNoCount;
    nackPending         = false;
    nackPendingSinceMs  = 0;
    firstRelevantSN_    = SequenceNumber::from_int64(1);
    highestSentSN_      = SEQUENCENUMBER_ZERO;
    requestedBase_      = SEQUENCENUMBER_ZERO;
    requestedBits_      = 0u;
}

void ReaderProxy::acked_changes_set(const SequenceNumber& committed_sn) noexcept {
    if (committed_sn > highestAckedSN) highestAckedSN = committed_sn;
    // An acknowledged change is no longer UNSENT either.
    mark_sent(committed_sn);
    if (requestedBits_ == 0u || committed_sn < requestedBase_) return;
    const int64_t drop = (committed_sn - requestedBase_) + 1;
    requestedBits_ = drop >= static_cast<int64_t>(kRequestedWindow) ? 0u : requestedBits_ >> drop;
    requestedBase_ = committed_sn + 1;
}

bool ReaderProxy::request(const SequenceNumber& sn) noexcept {
    if (sn <= highestAckedSN) return false;
    if (requestedBits_ == 0u) {
        requestedBase_ = sn;
        requestedBits_ = 1u;
        return true;
    }
    if (sn < requestedBase_) {
        // Keep the lowest requests: rebase on `sn` and let the highest fall
        // off the window; the reader NACKs those again after this round.
        const int64_t shift = requestedBase_ - sn;
        requestedBits_ = shift >= static_cast<int64_t>(kRequestedWindow)
                             ? 1u
                             : (requestedBits_ << shift) | 1u;
        requestedBase_ = sn;
        return true;
    }
    const int64_t offset = sn - requestedBase_;
    if (offset >= static_cast<int64_t>(kRequestedWindow)) return false;
    const uint64_t bit = uint64_t{1} << offset;
    if ((requestedBits_ & bit) != 0u) return false;
    requestedBits_ |= bit;
    return true;
}

void ReaderProxy::requested_changes_set(const SequenceNumberSet& sn_set, uint32_t now_ms) noexcept {
    bool any_requested = false;
    sn_set.for_each([&](const SequenceNumber& sn) noexcept {
        if (request(sn)) any_requested = true;
    });
    if (any_requested && !nackPending) {
        nackPending        = true;
        nackPendingSinceMs = now_ms;
    }
}

void ReaderProxy::mark_underway(const SequenceNumber& sn) noexcept {
    if (sn < requestedBase_) return;
    const int64_t offset = sn - requestedBase_;
    if (offset >= static_cast<int64_t>(kRequestedWindow)) return;
    requestedBits_ &= ~(uint64_t{1} << offset);
}

SequenceNumber ReaderProxy::next_unsent_change(const HistoryCache& history) const noexcept {
    SequenceNumber best = SEQUENCENUMBER_UNKNOWN;
    history.for_each_change([&](const CacheChange& c) noexcept {
        const SequenceNumber sn = c.sequenceNumber;
        if (!(highestSentSN_ < sn) || !is_relevant(sn)) return;
        if (best == SEQUENCENUMBER_UNKNOWN || sn < best) best = sn;
    });
    return best;
}

bool ReaderProxy::is_announcing(const HistoryCache& history) const noexcept {
    const SequenceNumber newest = history.get_seq_num_max();
    if (newest == SEQUENCENUMBER_UNKNOWN) return false;
    return is_relevant(newest) && highestAckedSN < newest;
}

SequenceNumber ReaderProxy::next_requested_change() const noexcept {
    if (requestedBits_ == 0u) return SEQUENCENUMBER_UNKNOWN;
    return requestedBase_ + static_cast<int64_t>(__builtin_ctzll(requestedBits_));
}

}  // namespace rosette::rtps
