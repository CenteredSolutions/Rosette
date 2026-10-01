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
 * A ring is the rmw entity's reader history, so it keeps exactly the
 * KEEP_LAST depth the entity reports through its actual QoS (effective_qos()
 * in qos.hpp): when `depth` changes are unread, the next one replaces the
 * oldest and releases its buffer, so a writer that outruns the application
 * loses its oldest samples rather than growing memory. The RTPS reader has
 * already acknowledged a replaced change, so the depth must be the one the
 * application asked for; RMW_ROSETTE_MAX_PENDING_CHANGES (capacity.hpp,
 * default 10 like `rmw_qos_profile_default`) bounds it.
 *
 * The ring never allocates; payload memory is the RTPS buffer pool's.
 */

#pragma once

#include "rmw_rosette/capacity.hpp"
#include "rosette/rtps/buf_pool.hpp"
#include "rosette/rtps/history_cache.hpp"
#include "rosette/rtps/types.hpp"

#include <cstdint>

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
     * Called from the reader's OnChangeAvailable trampoline. When `depth()`
     * changes are unread the oldest is replaced. Only ALIVE changes
     * with a payload are stored: ROS 2 topics are keyless, so an rmw take
     * returns samples and never instance state, and a disposal or
     * unregistration (NOT_ALIVE, often without payload) is not a message.
     *
     * @param[in] change Change delivered by the RTPS reader; its payload
     *   buffer is retained, the rest is copied.
     * @return `true` when the change was stored; `false` for a NOT_ALIVE
     *   change or an empty payload.
     *
     * Verification:
     * - Correct means: after N pushes into a ring of depth D, pop()
     *   returns the last min(N, D) changes in arrival order.
     * - Covered by: rmw_rosette/test/test_change_ring.cpp.
     *
     * References:
     * - OMG DDS 1.4 §2.2.3.18 (HISTORY KEEP_LAST)
     */
    bool push(const rtps::CacheChange& change) noexcept {
        if (change.kind != rtps::ChangeKind::ALIVE) return false;
        if (!change.payload.valid() || change.payload.size == 0) return false;
        if (count_ == depth_) {
            slots_[tail()].payload.reset();
            --count_;
        }
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
        ++count_;
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
        const uint32_t tail = this->tail();
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

    /**
     * @brief Sets how many unread changes the ring keeps.
     *
     * Called once when the owning entity is created, before any push, with
     * the KEEP_LAST depth its actual QoS reports, so the history a take sees
     * is the one rclcpp, rclpy, and rclc report to the application.
     *
     * @param[in] depth KEEP_LAST depth; 0 and values above
     *   RMW_ROSETTE_MAX_PENDING_CHANGES select RMW_ROSETTE_MAX_PENDING_CHANGES.
     *
     * Verification:
     * - Correct means: depth() equals the clamped value, and a ring never
     *   holds more than depth() changes.
     * - Covered by: rmw_rosette/test/test_change_ring.cpp.
     */
    void set_depth(uint32_t depth) noexcept {
        depth_ = (depth == 0u || depth > RMW_ROSETTE_MAX_PENDING_CHANGES)
                     ? RMW_ROSETTE_MAX_PENDING_CHANGES : depth;
    }

    /// @brief Unread changes the ring keeps before replacing the oldest.
    uint32_t depth() const noexcept { return depth_; }

private:
    /// @brief Index of the oldest unread change.
    uint32_t tail() const noexcept {
        return (head_ + RMW_ROSETTE_MAX_PENDING_CHANGES - count_) % RMW_ROSETTE_MAX_PENDING_CHANGES;
    }

    PendingChange slots_[RMW_ROSETTE_MAX_PENDING_CHANGES] = {};
    uint32_t      head_ = 0;   ///< Next slot to fill.
    uint32_t      count_ = 0;  ///< Unread changes.
    uint32_t      depth_ = RMW_ROSETTE_MAX_PENDING_CHANGES;  ///< KEEP_LAST depth.
};

}  // namespace rmw_rosette
