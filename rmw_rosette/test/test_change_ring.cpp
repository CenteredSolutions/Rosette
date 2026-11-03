// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_change_ring.cpp
 * @brief Unit tests for ChangeRing, the KEEP_LAST history behind rmw takes.
 *
 * A ChangeRing change has already been acknowledged by the RTPS reader, so
 * a change the ring drops is lost for good. These tests pin the history the
 * ring must keep: the depth its entity reports, never less (a burst within
 * the depth is taken whole) and never more (a shallow history hands out the
 * newest samples, not stale ones).
 */

#include <gtest/gtest.h>

#include "rmw_rosette/change_ring.hpp"

#include <cstring>

namespace rtps = rosette::rtps;

namespace {

/// Builds an ALIVE change with sequence number `sn` and a one-octet payload.
rtps::CacheChange make_change(int64_t sn) {
    rtps::CacheChange change{};
    change.kind = rtps::ChangeKind::ALIVE;
    change.sequenceNumber = rtps::SequenceNumber::from_int64(sn);
    change.payload = rtps::buf_alloc(1);
    change.payload.data[0] = static_cast<uint8_t>(sn);
    change.payload.size = 1;
    return change;
}

/// Pushes changes first..last (inclusive) into `ring`.
void push_range(rmw_rosette::ChangeRing& ring, int64_t first, int64_t last) {
    for (int64_t sn = first; sn <= last; ++sn) {
        ASSERT_TRUE(ring.push(make_change(sn)));
    }
}

/// Pops every change from `ring` and returns their sequence numbers in order.
std::vector<int64_t> drain(rmw_rosette::ChangeRing& ring) {
    std::vector<int64_t> sns;
    rmw_rosette::PendingChange out;
    while (ring.pop(out)) {
        sns.push_back(out.sequence_number.to_int64());
        out.payload.reset();
    }
    return sns;
}

}  // namespace

TEST(ChangeRingTest, BurstWithinDefaultDepthIsTakenWhole) {
    rmw_rosette::ChangeRing ring;
    ring.set_depth(10);  // rmw_qos_profile_default
    push_range(ring, 1, 5);
    EXPECT_EQ(drain(ring), (std::vector<int64_t>{1, 2, 3, 4, 5}));
}

TEST(ChangeRingTest, KeepLastReplacesOldestBeyondDepth) {
    rmw_rosette::ChangeRing ring;
    ring.set_depth(3);
    push_range(ring, 1, 5);
    EXPECT_EQ(drain(ring), (std::vector<int64_t>{3, 4, 5}));
}

TEST(ChangeRingTest, DepthOneKeepsOnlyTheNewest) {
    rmw_rosette::ChangeRing ring;
    ring.set_depth(1);
    push_range(ring, 1, 4);
    EXPECT_EQ(drain(ring), (std::vector<int64_t>{4}));
}

TEST(ChangeRingTest, DepthZeroOrTooDeepSelectsCapacity) {
    rmw_rosette::ChangeRing ring;
    ring.set_depth(0);
    EXPECT_EQ(ring.depth(), static_cast<uint32_t>(RMW_ROSETTE_MAX_PENDING_CHANGES));
    ring.set_depth(RMW_ROSETTE_MAX_PENDING_CHANGES + 1u);
    EXPECT_EQ(ring.depth(), static_cast<uint32_t>(RMW_ROSETTE_MAX_PENDING_CHANGES));
}

TEST(ChangeRingTest, ReplacementWrapsAroundTheSlots) {
    rmw_rosette::ChangeRing ring;
    ring.set_depth(4);
    // Interleave takes so head and tail wrap past the end of the slot array.
    for (int64_t round = 0; round < 3; ++round) {
        const int64_t base = round * 20;
        push_range(ring, base + 1, base + 7);
        EXPECT_EQ(drain(ring),
                  (std::vector<int64_t>{base + 4, base + 5, base + 6, base + 7}));
    }
}

TEST(ChangeRingTest, NotAliveAndEmptyChangesAreNotSamples) {
    rmw_rosette::ChangeRing ring;
    rtps::CacheChange disposed = make_change(1);
    disposed.kind = rtps::ChangeKind::NOT_ALIVE_DISPOSED;
    EXPECT_FALSE(ring.push(disposed));
    rtps::CacheChange empty{};
    empty.kind = rtps::ChangeKind::ALIVE;
    EXPECT_FALSE(ring.push(empty));
    EXPECT_FALSE(ring.ready());
}
