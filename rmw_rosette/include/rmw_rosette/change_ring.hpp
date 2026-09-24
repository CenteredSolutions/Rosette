/**
 * @file rmw_rosette/change_ring.hpp
 * @brief Fixed-capacity buffer between RTPS reader callbacks and rmw takes.
 *
 * `rtps::StatefulReader::OnChangeAvailable` hands over a CacheChange that is
 * valid only during the callback, and it fires from inside rmw_wait()'s
 * participant pump, while rmw_take*() runs later from the executor. Each
 * subscription, service, and client therefore keeps received changes in a
 * ChangeRing: a retained reference to the change's pool buffer (no copy, so a
 * reassembled 64 KiB sample costs no more than a small one) plus the metadata
 * a take needs (the writer GUID and sequence number that identify a service
 * request, and the related sample identity that correlates a service reply).
 *
 * The ring never allocates; payload memory is the RTPS buffer pool's. When
 * the ring is full, the oldest unread change is overwritten and its buffer
 * released, the KEEP_LAST behavior a DDS reader history has, so a writer
 * that outruns the application loses its oldest samples rather than growing
 * memory.
 */

#pragma once

#include "rosette/rtps/buf_pool.hpp"
#include "rosette/rtps/history_cache.hpp"
#include "rosette/rtps/types.hpp"

#include <cstdint>

#ifndef RMW_ROSETTE_MAX_PENDING_CHANGES
/// Changes buffered per subscription, service, or client between rmw takes.
#  define RMW_ROSETTE_MAX_PENDING_CHANGES 4
#endif

namespace rmw_rosette {

/// One received change, held until an rmw take.
struct PendingChange {
    rtps::BufRef         payload;  ///< Retained serializedPayload, header included.
    rtps::Guid           writer_guid = rtps::GUID_UNKNOWN;
    rtps::SequenceNumber sequence_number = rtps::SEQUENCENUMBER_UNKNOWN;
    rtps::Timestamp      source_timestamp = rtps::TIME_ZERO;
    bool                 has_related_sample_identity = false;
    rtps::SampleIdentity related_sample_identity = {};
};

/**
 * @brief KEEP_LAST ring of received changes for one rmw entity.
 */
class ChangeRing {
public:
    /**
     * @brief Copies one received change into the ring.
     *
     * Called from the reader's OnChangeAvailable trampoline. When the ring
     * is full the oldest unread change is overwritten.
     *
     * @param[in] change Change delivered by the RTPS reader; its payload
     *   buffer is retained, the rest is copied.
     * @return `true` when the change was stored.
     *
     * Verification:
     * - Correct means: after N pushes into a ring of capacity C, pop()
     *   returns the last min(N, C) changes in arrival order.
     */
    bool push(const rtps::CacheChange& change) noexcept {
        if (!change.payload.valid() || change.payload.size == 0) return false;
        PendingChange& slot = slots_[head_];
        // Share the reader's pool buffer: a new reference, released when the
        // slot is taken or overwritten.
        rtps::BufRef ref{};
        ref.block = change.payload.block;
        ref.data = change.payload.data;
        ref.size = change.payload.size;
        ref.retain();
        slot.payload = static_cast<rtps::BufRef&&>(ref);
        slot.writer_guid = change.writerGuid;
        slot.sequence_number = change.sequenceNumber;
        slot.source_timestamp = change.sourceTimestamp;
        slot.has_related_sample_identity = change.inlineQos.hasRelatedSampleIdentity;
        slot.related_sample_identity = change.inlineQos.relatedSampleIdentity;
        head_ = (head_ + 1u) % RMW_ROSETTE_MAX_PENDING_CHANGES;
        if (count_ < RMW_ROSETTE_MAX_PENDING_CHANGES) ++count_;
        return true;
    }

    /**
     * @brief Removes the oldest buffered change.
     *
     * @param[out] out Receives the change; untouched when the ring is empty.
     * @return `true` when a change was removed.
     */
    bool pop(PendingChange& out) noexcept {
        if (count_ == 0u) return false;
        const uint32_t tail =
            (head_ + RMW_ROSETTE_MAX_PENDING_CHANGES - count_) % RMW_ROSETTE_MAX_PENDING_CHANGES;
        out.payload = static_cast<rtps::BufRef&&>(slots_[tail].payload);
        out.writer_guid = slots_[tail].writer_guid;
        out.sequence_number = slots_[tail].sequence_number;
        out.source_timestamp = slots_[tail].source_timestamp;
        out.has_related_sample_identity = slots_[tail].has_related_sample_identity;
        out.related_sample_identity = slots_[tail].related_sample_identity;
        --count_;
        return true;
    }

    /// @brief True when at least one change is waiting; rmw_wait() readiness.
    bool ready() const noexcept { return count_ != 0u; }

private:
    PendingChange slots_[RMW_ROSETTE_MAX_PENDING_CHANGES] = {};
    uint32_t      head_ = 0;   ///< Next slot to fill.
    uint32_t      count_ = 0;  ///< Unread changes.
};

}  // namespace rmw_rosette
