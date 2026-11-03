// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file history_cache.cpp
 * @brief HistoryCache operations over caller-provided slots (§8.3.1).
 *
 * See rtps/history_cache.hpp. Every operation is a linear scan of the slot
 * array: capacities are a KEEP_LAST depth (a handful of slots), where a scan
 * is smaller and faster than any index structure.
 */

#include "rosette/rtps/history_cache.hpp"

namespace rosette::rtps {

CacheChange* HistoryCache::add_change(CacheChange&& change) noexcept {
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (!slots_[i].is_valid()) {
            slots_[i] = static_cast<CacheChange&&>(change);
            return &slots_[i];
        }
    }
    // Full: KEEP_LAST evicts this writer's oldest change (§8.3.3.1), or the
    // globally oldest when no slot holds one of this writer's changes.
    CacheChange* victim = find_oldest_for_writer(change.writerGuid);
    if (!victim) victim = find_globally_oldest();
    if (!victim) return nullptr;
    victim->clear();
    *victim = static_cast<CacheChange&&>(change);
    return victim;
}

bool HistoryCache::remove_change(const SequenceNumber& sequenceNumber) noexcept {
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (slots_[i].is_valid() && slots_[i].sequenceNumber == sequenceNumber) {
            slots_[i].clear();
            return true;
        }
    }
    return false;
}

const CacheChange* HistoryCache::get_change(const SequenceNumber& sequenceNumber) const noexcept {
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (slots_[i].is_valid() && slots_[i].sequenceNumber == sequenceNumber) {
            return &slots_[i];
        }
    }
    return nullptr;
}

SequenceNumber HistoryCache::get_seq_num_min() const noexcept {
    SequenceNumber result = SEQUENCENUMBER_UNKNOWN;
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (!slots_[i].is_valid()) continue;
        if (result == SEQUENCENUMBER_UNKNOWN || slots_[i].sequenceNumber < result) {
            result = slots_[i].sequenceNumber;
        }
    }
    return result;
}

SequenceNumber HistoryCache::get_seq_num_max() const noexcept {
    SequenceNumber result = SEQUENCENUMBER_UNKNOWN;
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (!slots_[i].is_valid()) continue;
        if (result == SEQUENCENUMBER_UNKNOWN || slots_[i].sequenceNumber > result) {
            result = slots_[i].sequenceNumber;
        }
    }
    return result;
}

uint8_t HistoryCache::size() const noexcept {
    uint8_t count = 0;
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (slots_[i].is_valid()) ++count;
    }
    return count;
}

void HistoryCache::clear_all() noexcept {
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (slots_[i].is_valid()) slots_[i].clear();
    }
}

CacheChange* HistoryCache::find_oldest_for_writer(const Guid& guid) noexcept {
    CacheChange* oldest = nullptr;
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (!slots_[i].is_valid() || slots_[i].writerGuid != guid) continue;
        if (!oldest || slots_[i].sequenceNumber < oldest->sequenceNumber) oldest = &slots_[i];
    }
    return oldest;
}

CacheChange* HistoryCache::find_globally_oldest() noexcept {
    CacheChange* oldest = nullptr;
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (!slots_[i].is_valid()) continue;
        if (!oldest || slots_[i].sequenceNumber < oldest->sequenceNumber) oldest = &slots_[i];
    }
    return oldest;
}

}  // namespace rosette::rtps
