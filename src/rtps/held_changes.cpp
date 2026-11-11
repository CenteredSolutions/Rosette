// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/held_changes.cpp
 * @brief HeldChanges: a reliable reader's changes waiting behind a gap.
 */

#include "held_changes.hpp"

namespace rosette::rtps {

bool HeldChanges::hold(CacheChange& change) noexcept {
    if (full()) {
        return false;
    }

    slots_[count_] = static_cast<CacheChange&&>(change);
    change = CacheChange{};
    ++count_;
    return true;
}

bool HeldChanges::take_below(const Guid& writer, const SequenceNumber& limit,
                             CacheChange& out) noexcept {
    uint8_t lowest = count_;
    for (uint8_t i = 0; i < count_; ++i) {
        const CacheChange& held = slots_[i];
        if (held.writerGuid == writer && held.sequenceNumber < limit
            && (lowest == count_ || held.sequenceNumber < slots_[lowest].sequenceNumber)) {
            lowest = i;
        }
    }

    if (lowest == count_) {
        return false;
    }

    out = static_cast<CacheChange&&>(slots_[lowest]);
    remove_at(lowest);
    return true;
}

bool HeldChanges::evict_highest_above(const Guid& writer, const SequenceNumber& sn,
                                      SequenceNumber& evicted) noexcept {
    uint8_t highest = count_;
    for (uint8_t i = 0; i < count_; ++i) {
        const CacheChange& held = slots_[i];
        if (held.writerGuid == writer && sn < held.sequenceNumber
            && (highest == count_ || slots_[highest].sequenceNumber < held.sequenceNumber)) {
            highest = i;
        }
    }

    if (highest == count_) {
        return false;
    }

    evicted = slots_[highest].sequenceNumber;
    slots_[highest].clear();
    remove_at(highest);
    return true;
}

void HeldChanges::discard_writer(const Guid& writer) noexcept {
    uint8_t i = 0;
    while (i < count_) {
        if (slots_[i].writerGuid == writer) {
            slots_[i].clear();
            remove_at(i);
        } else {
            ++i;
        }
    }
}

void HeldChanges::clear() noexcept {
    for (uint8_t i = 0; i < count_; ++i) {
        slots_[i].clear();
    }

    count_ = 0;
}

void HeldChanges::remove_at(uint8_t index) noexcept {
    --count_;
    if (index != count_) {
        slots_[index] = static_cast<CacheChange&&>(slots_[count_]);
    }

    slots_[count_] = CacheChange{};
}

}  // namespace rosette::rtps
