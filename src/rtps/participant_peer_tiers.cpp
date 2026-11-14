// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/participant_peer_tiers.cpp
 * @brief Which known peers are active: admission, parking and reactivation.
 *
 * A participant knows every peer SPDP announces (RTPS_MAX_KNOWN_PEERS
 * records), but exchanges SEDP with at most RTPS_MAX_PEERS of them at a
 * time: the builtin SEDP endpoints hold one proxy pair per active peer, and
 * those proxies, with the remote endpoint records SEDP fills in, are what a
 * peer really costs. The rest are pending (waiting for a slot) or parked
 * (see "Known and active peers" in rtps/spdp.hpp).
 *
 * Admission. A new or pending peer is activated when a slot is free.
 * Otherwise the active peer that has been settled longest is parked to make
 * room: a settled peer has exchanged everything with this participant and
 * shares no match that counts (peer_is_settled()). A peer that shares a
 * match is never parked, so the slots end up held by the peers this
 * participant really talks to, and the remaining slots rotate through the
 * rest of the network.
 *
 * Parking forgets nothing the peer needs. It already holds every
 * announcement and every `ros_discovery_info` sample this participant wrote,
 * and keeps it as long as SPDP keeps this participant alive. This
 * participant keeps the highest sequence number of each of the peer's SEDP
 * announcers, so a parked peer that announces a new endpoint (a HEARTBEAT
 * or DATA above that number) is reactivated and its announcements are
 * received again in full. When this participant's own announcements
 * change, or its discovery writer writes new contents
 * (discovery_info_changed()), every parked peer is owed them and is
 * reactivated in turn.
 *
 * Parking is invisible to the graph observer, which lists the network's
 * endpoints rather than the active tier's. Reactivation starts a refresh of
 * the peer's listing, which finish_peer_refreshes() completes once the
 * replay has caught up, so an endpoint withdrawn while the peer was parked
 * leaves the listing too.
 *
 * Verification: TC-PART-8.5.3-011 to -019 (tests/spec), and at network
 * scale tests/ros2/discovery_scale.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.5.3.2 (SPDP lease), §8.5.4.1 (SEDP builtin
 *   endpoints), §8.4.7.5 and §8.4.10.4 (reader and writer proxies)
 * - docs/design/two_tier_peer_table.md
 */

#include "participant.hpp"

#include "log.hpp"

namespace rosette::rtps {

bool RTPSParticipant::admit_peer(PeerEntry& peer, uint32_t now_ms) noexcept {
    if (peer.active()) {
        return true;
    }

    if (!peer.has_metatraffic_locator()) {
        RTPS_LOG_W("participant", "peer has no metatraffic locator, skipping SEDP");
        return false;
    }

    if (!spdp_reader_.can_activate(peer)) {
        return false;
    }

    if (spdp_reader_.active_count() >= RTPS_MAX_PEERS && !park_settled_peer(now_ms)) {
        return false;
    }

    activate_peer(peer, now_ms);
    return true;
}

void RTPSParticipant::activate_peer(PeerEntry& peer, uint32_t now_ms) noexcept {
    const DiscoveredParticipant* data = spdp_reader_.activate(peer);
    if (data == nullptr) {
        return;
    }

    RTPS_LOG_I("participant", "peer active %02x%02x%02x%02x%02x%02x%s", peer.guidPrefix.value[0],
               peer.guidPrefix.value[1], peer.guidPrefix.value[2], peer.guidPrefix.value[3],
               peer.guidPrefix.value[4], peer.guidPrefix.value[5],
               peer.parked ? " (was parked)" : "");
    const bool refresh = peer.parked
                         && (peer.parkedPublicationsSn != 0u || peer.parkedSubscriptionsSn != 0u)
                         && match_table_.participant_refreshing(peer.guidPrefix);
    if (!refresh) {
        peer.parkedPublicationsSn = 0u;
        peer.parkedSubscriptionsSn = 0u;
    }

    peer.parked = false;
    peer.owed = false;
    peer.lastSedpMs = now_ms;

    // §8.5.3.2: direct SPDP to the peer first, so it knows this participant
    // (and has matched its builtin SEDP writers) before any SEDP DATA
    // arrives; a peer drops DATA from writers it has not matched.
    spdp_writer_.announce_unicast(data->metatrafficUnicastLocatorList[0]);

    // Matching the peer's builtin SEDP readers replays every live endpoint
    // announcement to it (TRANSIENT_LOCAL, §8.5.4.1); matching its SEDP
    // writers lets the local builtin readers receive and repair its own.
    sedp_writer_.on_peer_discovered(*data);
    sedp_reader_.on_peer_discovered(*data);
    notify_graph_change();
}

bool RTPSParticipant::peer_is_settled(const PeerEntry& peer, uint32_t now_ms) noexcept {
    if (!peer.active() || !peer.rebuildable() || now_ms - peer.lastSedpMs < RTPS_PEER_SETTLE_MS) {
        return false;
    }

    const GuidPrefix& prefix = peer.guidPrefix;
    if (match_table_.has_match_with(prefix, discovery_writer_guid_)) {
        return false;
    }

    SequenceNumber publications;
    SequenceNumber subscriptions;
    if (!sedp_reader_.caught_up_with(prefix, publications, subscriptions)
        || !sedp_writer_.acknowledged_by(prefix)) {
        return false;
    }

    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i) {
        if (writer_alive_[i] && writers_[i] && !writers_[i]->acknowledged_by_participant(prefix)) {
            return false;
        }
    }

    return true;
}

void RTPSParticipant::park_peer(PeerEntry& peer) noexcept {
    const GuidPrefix prefix = peer.guidPrefix;
    SequenceNumber publications;
    SequenceNumber subscriptions;
    (void)sedp_reader_.caught_up_with(prefix, publications, subscriptions);
    RTPS_LOG_I("participant", "peer parked %02x%02x%02x%02x%02x%02x at SEDP %lu/%lu",
               prefix.value[0], prefix.value[1], prefix.value[2], prefix.value[3], prefix.value[4],
               prefix.value[5], static_cast<unsigned long>(publications.low),
               static_cast<unsigned long>(subscriptions.low));

    // A number that does not fit the record's 32 bits is kept as the
    // largest that does; any announcement above it still reactivates. While
    // a refresh is under way the record holds its target, which a replay
    // that has not arrived yet leaves above what the reader has resolved:
    // the larger number is kept, so the next activation resumes the refresh
    // instead of dropping it.
    const auto kept = [](uint32_t target, const SequenceNumber& resolved) noexcept {
        const uint32_t low = resolved.high != 0 ? UINT32_MAX : resolved.low;
        return low > target ? low : target;
    };

    peer.parkedPublicationsSn = kept(peer.parkedPublicationsSn, publications);
    peer.parkedSubscriptionsSn = kept(peer.parkedSubscriptionsSn, subscriptions);
    peer.parked = true;
    peer.owed = false;

    // The proxies go first: they refer to the active slot's locators, which
    // stay in place until the slot is freed.
    match_table_.release_participant(prefix);
    sedp_writer_.on_peer_removed(prefix);
    sedp_reader_.on_peer_removed(prefix);
    spdp_reader_.deactivate(peer);
}

bool RTPSParticipant::park_settled_peer(uint32_t now_ms) noexcept {
    PeerEntry* oldest = nullptr;
    spdp_reader_.for_each_peer([&](PeerEntry& peer) noexcept {
        if (!peer.active()) {
            return;
        }

        const bool older =
            oldest == nullptr || now_ms - peer.lastSedpMs > now_ms - oldest->lastSedpMs;
        if (older && peer_is_settled(peer, now_ms)) {
            oldest = &peer;
        }
    });

    if (oldest == nullptr) {
        return false;
    }

    park_peer(*oldest);
    return true;
}

void RTPSParticipant::on_announcer_traffic(const GuidPrefix& prefix, const EntityId& writer_id,
                                           const SequenceNumber& sn, bool is_data,
                                           uint32_t now_ms) noexcept {
    PeerEntry* peer = spdp_reader_.find_peer(prefix);
    if (peer == nullptr) {
        return;
    }

    if (peer->active()) {
        if (is_data) {
            peer->lastSedpMs = now_ms;
        }

        return;
    }

    if (peer->parked) {
        const uint32_t parked_sn = writer_id == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER
                                       ? peer->parkedPublicationsSn
                                       : peer->parkedSubscriptionsSn;
        if (sn.high == 0 && sn.low <= parked_sn) {
            return;
        }
    }

    (void)admit_peer(*peer, now_ms);
}

void RTPSParticipant::owe_parked_peers() noexcept {
    spdp_reader_.for_each_peer([](PeerEntry& peer) noexcept {
        if (peer.parked) {
            peer.owed = true;
        }
    });
}

void RTPSParticipant::finish_peer_refreshes() noexcept {
    if (!match_table_.tracks_refreshes()) {
        return;
    }

    // A number kept as UINT32_MAX stood for one above 32 bits: any such is reached.
    const auto reaches = [](const SequenceNumber& resolved, uint32_t kept) noexcept {
        return resolved.high != 0 || resolved.low >= kept;
    };

    spdp_reader_.for_each_peer([&](PeerEntry& peer) noexcept {
        if (!peer.active()
            || (peer.parkedPublicationsSn == 0u && peer.parkedSubscriptionsSn == 0u)) {
            return;
        }

        // A caught-up announcer that has sent a HEARTBEAT holds nothing more,
        // even when its numbers restarted below the kept one.
        SequenceNumber publications;
        SequenceNumber subscriptions;
        bool           heard_publications = false;
        bool           heard_subscriptions = false;
        sedp_reader_.heard_from(peer.guidPrefix, heard_publications, heard_subscriptions);
        if (!sedp_reader_.caught_up_with(peer.guidPrefix, publications, subscriptions)
            || !(heard_publications || reaches(publications, peer.parkedPublicationsSn))
            || !(heard_subscriptions || reaches(subscriptions, peer.parkedSubscriptionsSn))) {
            return;
        }

        peer.parkedPublicationsSn = 0u;
        peer.parkedSubscriptionsSn = 0u;
        match_table_.participant_refreshed(peer.guidPrefix);
    });
}

void RTPSParticipant::serve_owed_peer(uint32_t now_ms) noexcept {
    PeerEntry* owed = nullptr;
    spdp_reader_.for_each_peer([&owed](PeerEntry& peer) noexcept {
        if (owed == nullptr && peer.owed) {
            owed = &peer;
        }
    });

    if (owed != nullptr) {
        (void)admit_peer(*owed, now_ms);
    }
}

}  // namespace rosette::rtps
