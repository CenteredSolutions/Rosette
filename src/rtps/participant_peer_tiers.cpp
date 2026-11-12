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
 * Otherwise the idle active peer whose last SEDP DATA (or activation) is
 * oldest is parked to make room. An active peer is idle when it is settled
 * or stalled, and neither ever shares a match that counts (one other than
 * with the discovery writer), so the slots end up held by the peers this
 * participant really talks to and the rest rotate through the network.
 *
 * | Active peer | No SEDP DATA for              | And also                        | Parked as  |
 * |-------------|-------------------------------|---------------------------------|------------|
 * | settled     | RTPS_PEER_SETTLE_MS           | both announcers heard, caught   | not owed   |
 * |             |                               | up; its readers acknowledged    |            |
 * |             |                               | every local SEDP and user write |            |
 * | stalled     | RTPS_PEER_EXCHANGE_TIMEOUT_MS | nothing else                    | owed       |
 * | neither     |                               |                                 | not parked |
 *
 * Settling waits for both of the peer's announcers to be heard, not only
 * caught up, because of what this participant sends. Its discovery writer
 * has a proxy for the peer's reader only once the announcers' replay has
 * announced that reader again, and an acknowledgment check passes for a
 * reader it has no proxy for. A reactivated peer whose replay never started
 * (the preemptive ACKNACK lost) would settle without the
 * `ros_discovery_info` sample it was reactivated to receive. Stalling
 * bounds a peer whose SPDP arrives but whose SEDP exchange never completes
 * (an unreachable unicast locator, a firewall): SPDP keeps its lease alive,
 * and without the deadline it would hold its slot for good. It is parked
 * owed, so it is tried again when a slot is free.
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
 * reactivated in turn, oldest last SEDP DATA or activation first
 * (serve_owed_peer()). Every activation restamps a peer, so stalled peers
 * taking turns in a slot fall behind one that has been waiting; table order
 * would let two of them starve a reachable peer behind them.
 *
 * Parking is invisible to the graph observer, which lists the network's
 * endpoints rather than the active tier's. Reactivation starts a refresh of
 * the peer's listing, which finish_peer_refreshes() completes once the
 * replay has caught up, so an endpoint withdrawn while the peer was parked
 * leaves the listing too.
 *
 * Verification: TC-PART-8.5.3-011 to -021 (tests/spec), at network scale
 * tests/ros2/discovery_scale, and as a design by the TLA+ models
 * RosettePeerTable and RosettePeerOutbound (tests/formal).
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

    if (spdp_reader_.active_count() >= RTPS_MAX_PEERS && !park_idle_peer(now_ms)) {
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
    if (!sedp_reader_.heard_from(prefix)
        || !sedp_reader_.caught_up_with(prefix, publications, subscriptions)
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

bool RTPSParticipant::peer_is_stalled(const PeerEntry& peer, uint32_t now_ms) noexcept {
    return peer.active() && peer.rebuildable()
           && now_ms - peer.lastSedpMs >= RTPS_PEER_EXCHANGE_TIMEOUT_MS
           && !match_table_.has_match_with(peer.guidPrefix, discovery_writer_guid_);
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

bool RTPSParticipant::park_idle_peer(uint32_t now_ms) noexcept {
    PeerEntry* oldest = nullptr;
    bool       oldest_settled = false;
    spdp_reader_.for_each_peer([&](PeerEntry& peer) noexcept {
        if (!peer.active()) {
            return;
        }

        const bool older =
            oldest == nullptr || now_ms - peer.lastSedpMs > now_ms - oldest->lastSedpMs;
        if (!older) {
            return;
        }

        const bool settled = peer_is_settled(peer, now_ms);
        if (settled || peer_is_stalled(peer, now_ms)) {
            oldest = &peer;
            oldest_settled = settled;
        }
    });

    if (oldest == nullptr) {
        return false;
    }

    park_peer(*oldest);

    // A stalled peer has not received everything this participant
    // announced; owing it them brings it back when a slot is free.
    oldest->owed = !oldest_settled;
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
    // A number kept as UINT32_MAX stood for one above 32 bits: any such is reached.
    const auto reaches = [](const SequenceNumber& resolved, uint32_t kept) noexcept {
        return resolved.high != 0 || resolved.low >= kept;
    };

    spdp_reader_.for_each_peer([&](PeerEntry& peer) noexcept {
        if (!peer.active()
            || (peer.parkedPublicationsSn == 0u && peer.parkedSubscriptionsSn == 0u)) {
            return;
        }

        SequenceNumber publications;
        SequenceNumber subscriptions;
        if (!sedp_reader_.caught_up_with(peer.guidPrefix, publications, subscriptions)
            || !reaches(publications, peer.parkedPublicationsSn)
            || !reaches(subscriptions, peer.parkedSubscriptionsSn)) {
            return;
        }

        peer.parkedPublicationsSn = 0u;
        peer.parkedSubscriptionsSn = 0u;
        match_table_.participant_refreshed(peer.guidPrefix);
    });
}

void RTPSParticipant::serve_owed_peer(uint32_t now_ms) noexcept {
    // Oldest first: every activation stamps lastSedpMs, so peers that take
    // turns in a slot (stalled peers parked owed again) move behind one that
    // has waited, and every owed peer is eventually served.
    PeerEntry* owed = nullptr;
    spdp_reader_.for_each_peer([&](PeerEntry& peer) noexcept {
        if (peer.owed
            && (owed == nullptr || now_ms - peer.lastSedpMs > now_ms - owed->lastSedpMs)) {
            owed = &peer;
        }
    });

    if (owed != nullptr) {
        (void)admit_peer(*owed, now_ms);
    }
}

}  // namespace rosette::rtps
