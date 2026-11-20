// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file history_cache.cpp
 * @brief HistoryCache operations over caller-provided slots (§8.3.1).
 *
 * See rtps/history_cache.hpp. Lookups are a linear scan of the slot array:
 * capacities are a KEEP_LAST depth, where a scan is smaller than any index
 * structure. The one exception is the newest change, whose slot the cache
 * remembers: a writer asks for it per matched reader on every tick (is the
 * reader still announcing?) and per replayed datagram, which made the scan the
 * largest single cost of the stack. Only removing or evicting the newest
 * change scans for its successor.
 */

#include "rosette/rtps/history_cache.hpp"

namespace rosette::rtps {

HistoryChange* HistoryCache::add_change(HistoryChange&& change) noexcept {
    HistoryChange* slot = nullptr;
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (!slots_[i].is_valid()) {
            slot = &slots_[i];
            break;
        }
    }

    // Full: KEEP_LAST evicts the oldest change (§8.3.3.1).
    if (slot == nullptr) {
        slot = find_oldest();
        slot->clear();
    }

    *slot = static_cast<HistoryChange&&>(change);
    const auto index = static_cast<uint8_t>(slot - slots_);
    if (index == newest_) {
        newest_ = find_newest();
    } else if (newest_ == kNoSlot || slots_[newest_].sequenceNumber < slot->sequenceNumber) {
        newest_ = index;
    }

    return slot;
}

bool HistoryCache::remove_change(const SequenceNumber& sequenceNumber) noexcept {
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (slots_[i].is_valid() && slots_[i].sequenceNumber == sequenceNumber) {
            slots_[i].clear();
            if (i == newest_) {
                newest_ = find_newest();
            }

            return true;
        }
    }

    return false;
}

const HistoryChange* HistoryCache::get_change(const SequenceNumber& sequenceNumber) const noexcept {
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

    newest_ = kNoSlot;
}

HistoryChange* HistoryCache::find_oldest() noexcept {
    HistoryChange* oldest = nullptr;
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (!slots_[i].is_valid()) {
            continue;
        }

        if (!oldest || slots_[i].sequenceNumber < oldest->sequenceNumber) {
            oldest = &slots_[i];
        }
    }

    return oldest;
}

uint8_t HistoryCache::find_newest() const noexcept {
    uint8_t newest = kNoSlot;
    for (uint8_t i = 0; i < capacity_; ++i) {
        if (!slots_[i].is_valid()) {
            continue;
        }

        if (newest == kNoSlot || slots_[newest].sequenceNumber < slots_[i].sequenceNumber) {
            newest = i;
        }
    }

    return newest;
}

}  // namespace rosette::rtps
