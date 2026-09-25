/**
 * @file rtps/writer_proxy.cpp
 * @brief WriterProxy: changes_from_writer and HEARTBEAT handling (§8.4.10.4).
 */

#include "writer_proxy.hpp"

#include "gap_encoding.hpp"

namespace rtps {

void WriterProxy::init(const Guid& writer_guid, const Locator* unicast_locs,
                       uint8_t unicast_count) noexcept {
    clear();
    remoteWriterGuid = writer_guid;
    isAlive          = true;
    const uint8_t n = unicast_count < RTPS_MAX_WRITER_PROXY_LOCATORS
                      ? unicast_count : RTPS_MAX_WRITER_PROXY_LOCATORS;
    for (uint8_t i = 0; i < n; ++i) unicastLocatorList[i] = unicast_locs[i];
    unicastLocatorCount = n;
}

void WriterProxy::received_change_set(const SequenceNumber& sn) noexcept {
    (void)changes_.resolve(sn);
    missing_changes_update(sn);
}

void WriterProxy::irrelevant_change_set(const SequenceNumber&    gap_start,
                                        const SequenceNumberSet& gap_list) noexcept {
    const SequenceNumber run_end = gap_run_end(gap_start, gap_list);
    if (gap_start < run_end) {
        changes_.resolve_range(gap_start, run_end);
        missing_changes_update(run_end - 1);
    }
    gap_list.for_each([&](const SequenceNumber& sn) noexcept {
        (void)changes_.resolve(sn);
        missing_changes_update(sn);
    });
}

bool WriterProxy::on_heartbeat(const SequenceNumber& first_sn,
                               const SequenceNumber& last_sn,
                               int32_t               hb_count,
                               bool                  final_flag,
                               uint32_t              now_ms,
                               uint32_t              response_delay_ms) noexcept {
    if (hb_count <= lastHBCount) return false;
    lastHBCount = hb_count;

    lost_changes_update(first_sn);
    missing_changes_update(last_sn);

    const bool must_reply = !final_flag || has_missing_changes();
    if (must_reply) {
        acknackPending = true;
        heartbeatResponseDeadlineMs = now_ms + response_delay_ms;
    }
    return must_reply;
}

const Locator& WriterProxy::best_locator() const noexcept {
    for (uint8_t i = 0; i < unicastLocatorCount; ++i) {
        const Long kind = unicastLocatorList[i].kind;
        if (kind == locator_kind::UDPv4 || kind == locator_kind::UDPv6) {
            return unicastLocatorList[i];
        }
    }
    if (unicastLocatorCount > 0) return unicastLocatorList[0];
    return LOCATOR_INVALID;
}

}  // namespace rtps
