/**
 * @file reader_proxy.cpp
 * @brief ReaderProxy change tracking over its writer-owned row (§8.4.1.1, §8.4.6).
 *
 * See rtps/reader_proxy.hpp. Each operation scans the proxy's row of
 * ChangeForReader slots, which holds at most the writer's KEEP_LAST depth.
 */

#include "rosette/rtps/reader_proxy.hpp"

namespace rtps {

void ReaderProxy::init(const Guid& reader_guid, const Locator* unicast_locs,
                       uint8_t unicast_count, bool expects_inline_qos) noexcept {
    clear();
    remoteReaderGuid = reader_guid;
    expectsInlineQos = expects_inline_qos;
    isActive         = true;
    const uint8_t n = unicast_count < RTPS_MAX_READER_PROXY_LOCATORS
                          ? unicast_count
                          : static_cast<uint8_t>(RTPS_MAX_READER_PROXY_LOCATORS);
    for (uint8_t i = 0; i < n; ++i) unicastLocatorList[i] = unicast_locs[i];
    unicastLocatorCount = n;
}

void ReaderProxy::clear() noexcept {
    remoteReaderGuid    = GUID_UNKNOWN;
    expectsInlineQos    = false;
    isActive            = false;
    unicastLocatorCount = 0;
    highestAckedSN      = SEQUENCENUMBER_ZERO;
    lastAcknackCount    = 0;
    nackPending         = false;
    nackPendingSinceMs  = 0;
    for (uint8_t i = 0; i < depth_; ++i) changes_[i].clear();
}

bool ReaderProxy::add_change(const SequenceNumber& sn) noexcept {
    ChangeForReader* slot = find_slot(sn);
    if (!slot) slot = evict_oldest();
    if (!slot) return false;
    slot->sequenceNumber = sn;
    slot->status         = ChangeForReaderStatus::UNSENT;
    return true;
}

bool ReaderProxy::tracks(const SequenceNumber& sn) const noexcept {
    for (uint8_t i = 0; i < depth_; ++i) {
        if (changes_[i].is_valid() && changes_[i].sequenceNumber == sn) return true;
    }
    return false;
}

void ReaderProxy::remove_change(const SequenceNumber& sn) noexcept {
    for (uint8_t i = 0; i < depth_; ++i) {
        if (changes_[i].sequenceNumber == sn) {
            changes_[i].clear();
            return;
        }
    }
}

void ReaderProxy::acked_changes_set(const SequenceNumber& committed_sn) noexcept {
    if (committed_sn > highestAckedSN) highestAckedSN = committed_sn;
    for (uint8_t i = 0; i < depth_; ++i) {
        if (changes_[i].is_valid() && changes_[i].sequenceNumber <= committed_sn) {
            changes_[i].status = ChangeForReaderStatus::ACKNOWLEDGED;
        }
    }
}

void ReaderProxy::requested_changes_set(const SequenceNumberSet& sn_set, uint32_t now_ms) noexcept {
    bool any_requested = false;
    sn_set.for_each([&](const SequenceNumber& sn) noexcept {
        for (uint8_t i = 0; i < depth_; ++i) {
            if (changes_[i].sequenceNumber == sn &&
                changes_[i].status != ChangeForReaderStatus::ACKNOWLEDGED) {
                changes_[i].status = ChangeForReaderStatus::REQUESTED;
                any_requested = true;
            }
        }
    });
    if (any_requested && !nackPending) {
        nackPending        = true;
        nackPendingSinceMs = now_ms;
    }
}

void ReaderProxy::mark_underway(const SequenceNumber& sn) noexcept {
    for (uint8_t i = 0; i < depth_; ++i) {
        if (changes_[i].sequenceNumber == sn &&
            changes_[i].status == ChangeForReaderStatus::REQUESTED) {
            changes_[i].status = ChangeForReaderStatus::UNDERWAY;
        }
    }
}

void ReaderProxy::mark_sent(const SequenceNumber& sn) noexcept {
    for (uint8_t i = 0; i < depth_; ++i) {
        if (changes_[i].sequenceNumber == sn &&
            changes_[i].status == ChangeForReaderStatus::UNSENT) {
            changes_[i].status = ChangeForReaderStatus::UNACKNOWLEDGED;
        }
    }
}

bool ReaderProxy::has_unsent_changes() const noexcept {
    for (uint8_t i = 0; i < depth_; ++i) {
        if (changes_[i].is_valid() && changes_[i].status == ChangeForReaderStatus::UNSENT) {
            return true;
        }
    }
    return false;
}

const ChangeForReader* ReaderProxy::next_unsent_change() const noexcept {
    const ChangeForReader* best = nullptr;
    for (uint8_t i = 0; i < depth_; ++i) {
        if (!changes_[i].is_valid() || changes_[i].status != ChangeForReaderStatus::UNSENT) continue;
        if (!best || changes_[i].sequenceNumber < best->sequenceNumber) best = &changes_[i];
    }
    return best;
}

bool ReaderProxy::has_unacked_changes() const noexcept {
    for (uint8_t i = 0; i < depth_; ++i) {
        if (!changes_[i].is_valid()) continue;
        const ChangeForReaderStatus s = changes_[i].status;
        if (s == ChangeForReaderStatus::UNACKNOWLEDGED || s == ChangeForReaderStatus::REQUESTED ||
            s == ChangeForReaderStatus::UNDERWAY) {
            return true;
        }
    }
    return false;
}

bool ReaderProxy::is_announcing() const noexcept {
    for (uint8_t i = 0; i < depth_; ++i) {
        if (changes_[i].is_valid() && changes_[i].status != ChangeForReaderStatus::ACKNOWLEDGED) {
            return true;
        }
    }
    return false;
}

bool ReaderProxy::has_requested_changes() const noexcept {
    for (uint8_t i = 0; i < depth_; ++i) {
        if (changes_[i].is_valid() && changes_[i].status == ChangeForReaderStatus::REQUESTED) {
            return true;
        }
    }
    return false;
}

const ChangeForReader* ReaderProxy::next_requested_change() const noexcept {
    const ChangeForReader* best = nullptr;
    for (uint8_t i = 0; i < depth_; ++i) {
        if (!changes_[i].is_valid() || changes_[i].status != ChangeForReaderStatus::REQUESTED) continue;
        if (!best || changes_[i].sequenceNumber < best->sequenceNumber) best = &changes_[i];
    }
    return best;
}

const Locator& ReaderProxy::best_locator() const noexcept {
    for (uint8_t i = 0; i < unicastLocatorCount; ++i) {
        const Long kind = unicastLocatorList[i].kind;
        if (kind == locator_kind::UDPv4 || kind == locator_kind::UDPv6) return unicastLocatorList[i];
    }
    if (unicastLocatorCount > 0) return unicastLocatorList[0];
    return LOCATOR_INVALID;
}

ChangeForReader* ReaderProxy::find_slot(const SequenceNumber& sn) noexcept {
    for (uint8_t i = 0; i < depth_; ++i) {
        if (!changes_[i].is_valid() || changes_[i].sequenceNumber == sn) return &changes_[i];
    }
    return nullptr;
}

ChangeForReader* ReaderProxy::evict_oldest() noexcept {
    ChangeForReader* oldest = nullptr;
    for (uint8_t i = 0; i < depth_; ++i) {
        if (!oldest || changes_[i].sequenceNumber < oldest->sequenceNumber) oldest = &changes_[i];
    }
    if (oldest) oldest->clear();
    return oldest;
}

}  // namespace rtps
