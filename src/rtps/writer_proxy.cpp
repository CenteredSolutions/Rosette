// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/writer_proxy.cpp
 * @brief WriterProxy: changes_from_writer and HEARTBEAT handling (§8.4.10.4).
 */

#include "writer_proxy.hpp"

#include "gap_encoding.hpp"

namespace rosette::rtps {

void WriterProxy::init(const Guid& writer_guid, const Locator* unicast_locs,
                       uint8_t unicast_count) noexcept {
    clear();
    remoteWriterGuid = writer_guid;
    unicastLocator = preferred_unicast_locator(unicast_locs, unicast_count);
}

// Any HEARTBEAT, DATA or GAP is the writer heard: a preemptive ACKNACK
// still pending is no longer needed, since the writer now knows this reader
// and its HEARTBEATs drive the exchange from here.
void WriterProxy::on_heard() noexcept {
    if (!heard) {
        heard = true;
        acknackPending = false;
        heartbeatResponseDeadlineMs = 0;
    }
}

void WriterProxy::received_change_set(const SequenceNumber& sn) noexcept {
    on_heard();
    (void)changes_.resolve(sn);
    missing_changes_update(sn);
}

void WriterProxy::irrelevant_change_set(const SequenceNumber&    gap_start,
                                        const SequenceNumberSet& gap_list) noexcept {
    on_heard();
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
    if (hb_count <= lastHBCount) {
        return false;
    }

    lastHBCount = hb_count;
    on_heard();
    lost_changes_update(first_sn);
    missing_changes_update(last_sn);

    const bool must_reply = !final_flag || has_missing_changes();
    if (must_reply && !acknackPending) {
        acknackPending = true;
        heartbeatResponseDeadlineMs = now_ms + response_delay_ms;
    }
    return must_reply;
}

}  // namespace rosette::rtps
