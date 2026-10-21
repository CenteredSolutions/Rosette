/**
 * @file rtps/spdp_peer_records.cpp
 * @brief Known-peer records and active slots of the SPDP peer table.
 *
 * Every known participant has a record (PeerEntry) sized for the whole
 * network; only active peers hold a slot with their full discovery data
 * (see "Known and active peers" in rtps/spdp.hpp). This file moves a peer's
 * data between the two: what a record keeps of an announcement, how a slot
 * is filled from an announcement or rebuilt from a record, and how slots are
 * handed out and freed.
 *
 * Verification: TC-SPDP-8.5.3-012 (tests/spec), and through the participant
 * TC-PART-8.5.3-011 and -012.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.5.3.2, Table 8.73 (SPDPdiscoveredParticipantData)
 * - docs/design/two_tier_peer_table.md ("Known-peer record")
 */

#include "spdp.hpp"

#include <string.h>

namespace rosette::rtps {

namespace {

/// Whether two locators share kind and address, whatever their ports.
bool same_address(const Locator& a, const Locator& b) noexcept {
    return a.kind == b.kind && memcmp(a.address, b.address, sizeof(a.address)) == 0;
}

}  // namespace

void PeerEntry::remember(const DiscoveredParticipant& announced) noexcept {
    guidPrefix = announced.guidPrefix;
    vendorId = announced.vendorId;
    availableBuiltinEndpoints = announced.availableBuiltinEndpoints;
    metatrafficLocator = announced.metatrafficUnicastLocatorCount > 0u
                             ? announced.metatrafficUnicastLocatorList[0]
                             : Locator{};

    // The default list is ranked like the metatraffic list, so the first
    // default locator at the metatraffic address is the one sending uses.
    defaultUnicastPort = 0u;
    for (uint8_t i = 0; i < announced.defaultUnicastLocatorCount; ++i) {
        const Locator& candidate = announced.defaultUnicastLocatorList[i];
        if (has_metatraffic_locator() && same_address(candidate, metatrafficLocator)) {
            defaultUnicastPort = static_cast<uint16_t>(candidate.port);
            break;
        }
    }
}

DiscoveredParticipant PeerEntry::rebuild() const noexcept {
    DiscoveredParticipant p{};
    p.guidPrefix = guidPrefix;
    p.vendorId = vendorId;
    p.availableBuiltinEndpoints = availableBuiltinEndpoints;
    if (has_metatraffic_locator()) {
        p.metatrafficUnicastLocatorList[0] = metatrafficLocator;
        p.metatrafficUnicastLocatorCount = 1u;
    }

    if (rebuildable()) {
        p.defaultUnicastLocatorList[0] = metatrafficLocator;
        p.defaultUnicastLocatorList[0].port = defaultUnicastPort;
        p.defaultUnicastLocatorCount = 1u;
    }

    return p;
}

const DiscoveredParticipant* SpdpReader::activate(PeerEntry& peer) noexcept {
    if (peer.active()) {
        return &slots_[peer.activeSlot];
    }

    if (!can_activate(peer)) {
        return nullptr;
    }

    const bool announcing =
        announcement_ != nullptr && announcement_->guidPrefix == peer.guidPrefix;
    for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
        if (slot_used_[i]) {
            continue;
        }

        slots_[i] = announcing ? *announcement_ : peer.rebuild();
        slot_used_[i] = true;
        peer.activeSlot = i;
        return &slots_[i];
    }

    return nullptr;
}

void SpdpReader::deactivate(PeerEntry& peer) noexcept {
    if (!peer.active()) {
        return;
    }

    slot_used_[peer.activeSlot] = false;
    slots_[peer.activeSlot] = DiscoveredParticipant{};
    peer.activeSlot = PeerEntry::kNoActiveSlot;
}

}  // namespace rosette::rtps
