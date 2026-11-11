// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/held_changes.cpp
 * @brief HeldChangePool slot bookkeeping and each reader's HeldChanges view.
 *
 * Used slots are kept dense at the front of the pool, so every scan stops
 * at the pool's count. Removing a slot moves the last used one into it.
 */

#include "held_changes.hpp"

namespace rosette::rtps {

void HeldChangePool::remove_at(uint8_t index) noexcept {
    slots_[index].change.clear();
    --count_;
    if (index != count_) {
        slots_[index].change = static_cast<CacheChange&&>(slots_[count_].change);
        slots_[index].owner = slots_[count_].owner;
    }

    slots_[count_].change = CacheChange{};
    slots_[count_].owner = nullptr;
}

uint8_t HeldChanges::count() const noexcept {
    uint8_t n = 0;
    for (uint8_t i = 0; i < pool_.count_; ++i) {
        if (pool_.slots_[i].owner == owner_) {
            ++n;
        }
    }

    return n;
}

bool HeldChanges::hold(CacheChange& change) noexcept {
    if (pool_.count_ == pool_.capacity_ || count() >= limit_) {
        return false;
    }

    HeldSlot& slot = pool_.slots_[pool_.count_];
    slot.change = static_cast<CacheChange&&>(change);
    slot.owner = owner_;
    change = CacheChange{};
    ++pool_.count_;
    return true;
}

bool HeldChanges::take_below(const Guid& writer, const SequenceNumber& limit,
                             CacheChange& out) noexcept {
    uint8_t lowest = pool_.count_;
    for (uint8_t i = 0; i < pool_.count_; ++i) {
        const HeldSlot& held = pool_.slots_[i];
        if (held.owner == owner_ && held.change.writerGuid == writer
            && held.change.sequenceNumber < limit
            && (lowest == pool_.count_
                || held.change.sequenceNumber < pool_.slots_[lowest].change.sequenceNumber)) {
            lowest = i;
        }
    }

    if (lowest == pool_.count_) {
        return false;
    }

    out = static_cast<CacheChange&&>(pool_.slots_[lowest].change);
    pool_.remove_at(lowest);
    return true;
}

bool HeldChanges::evict_for_block(const Guid& writer, HeldEviction& evicted) noexcept {
    if (pool_.count_ == 0u) {
        return false;
    }

    // The victim's reader and writer: this reader's own `writer` when it
    // holds one of its changes, otherwise those of the last used slot.
    const HeldSlot* last = &pool_.slots_[pool_.count_ - 1u];
    StatefulReader* owner = last->owner;
    Guid            victim_writer = last->change.writerGuid;
    for (uint8_t i = 0; i < pool_.count_; ++i) {
        if (pool_.slots_[i].owner == owner_ && pool_.slots_[i].change.writerGuid == writer) {
            owner = owner_;
            victim_writer = writer;
            break;
        }
    }

    uint8_t highest = pool_.count_;
    for (uint8_t i = 0; i < pool_.count_; ++i) {
        const HeldSlot& held = pool_.slots_[i];
        if (held.owner == owner && held.change.writerGuid == victim_writer
            && (highest == pool_.count_
                || pool_.slots_[highest].change.sequenceNumber < held.change.sequenceNumber)) {
            highest = i;
        }
    }

    evicted.owner = owner;
    evicted.writer = victim_writer;
    evicted.sn = pool_.slots_[highest].change.sequenceNumber;
    pool_.remove_at(highest);
    return true;
}

void HeldChanges::discard_writer(const Guid& writer) noexcept {
    uint8_t i = 0;
    while (i < pool_.count_) {
        if (pool_.slots_[i].owner == owner_ && pool_.slots_[i].change.writerGuid == writer) {
            pool_.remove_at(i);
        } else {
            ++i;
        }
    }
}

void HeldChanges::clear() noexcept {
    uint8_t i = 0;
    while (i < pool_.count_) {
        if (pool_.slots_[i].owner == owner_) {
            pool_.remove_at(i);
        } else {
            ++i;
        }
    }
}

}  // namespace rosette::rtps
