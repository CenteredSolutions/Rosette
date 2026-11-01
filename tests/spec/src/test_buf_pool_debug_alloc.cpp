// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_buf_pool_debug_alloc.cpp
 * @brief The buffer pool's debug allocation macros, called as a firmware calls them:
 * TC-BUF-DBG-001 and TC-BUF-DBG-002.
 *
 * RTPS_BUF_ALLOC_DBG and RTPS_BUF_ALLOC_WITH_HEADROOM_DBG exist only with RTPS_BUFPOOL_DEBUG, so
 * no other program compiles them. This one is built with it, as buf_pool_debug_spec_tests, and
 * calls both macros from functions in the global namespace with no `rtps` alias in scope: each
 * macro must name the stack by its full namespace to compile here. The tests then check what the
 * debug build records for each allocation, the per-class live count and high-water mark that size
 * a firmware's pool classes (docs/design/imxrt_memory_budget.md, step 4).
 *
 *   ./buf_pool_debug_spec_tests
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/buf_pool.hpp"

#include <string>

#ifndef RTPS_BUFPOOL_DEBUG
#  error "buf_pool_debug_spec_tests is built with RTPS_BUFPOOL_DEBUG (tests/spec/CMakeLists.txt)"
#endif

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

/// Octets the tests request: small enough for the pool's smallest class.
constexpr uint32_t kRequest = 16u;
/// Headroom TC-BUF-DBG-002 reserves ahead of the payload.
constexpr uint32_t kHeadroom = 8u;

/// The smallest class, the one a kRequest allocation is served from.
constexpr uint8_t kSmallestClass = 0u;

/**
 * @brief TC-BUF-DBG-001: RTPS_BUF_ALLOC_DBG allocates and records the block's class use.
 *
 * Verification:
 * - Correct means: the block is valid with at least kRequest octets of capacity; the smallest
 *   class's live count and high-water mark rise by one; the newest debug event is an Alloc of
 *   kRequest octets; releasing the block returns the live count and keeps the high-water mark.
 */
bool tc_buf_dbg_001(std::string& err) {
    const auto&    pool = ::rosette::rtps::g_buf_pool;
    const auto     before = pool.debug_class_stats(kSmallestClass);
    const uint32_t events_before = pool.debug_event_count();

    ::rosette::rtps::BufRef buf = RTPS_BUF_ALLOC_DBG(kRequest);
    CHECK(buf.valid(), "RTPS_BUF_ALLOC_DBG returned no block");
    CHECK(buf.capacity() >= kRequest, "the block is smaller than requested");

    const auto during = pool.debug_class_stats(kSmallestClass);
    CHECK(during.live == before.live + 1u, "the smallest class's live count did not rise by one");
    CHECK(during.high_water >= during.live, "the high-water mark is below the live count");

    ::rosette::rtps::BufPoolDebugEvent event{};
    CHECK(pool.debug_event_count() > events_before, "no debug event was recorded");
    CHECK(pool.debug_recent_event(0u, event), "the newest debug event cannot be read");
    CHECK(event.op == static_cast<uint8_t>(::rosette::rtps::BufPoolDebugOp::Alloc),
          "the newest debug event is not an Alloc");
    CHECK(event.requested_size == kRequest, "the Alloc event does not record the requested size");

    buf.reset();
    const auto after = pool.debug_class_stats(kSmallestClass);
    CHECK(after.live == before.live, "releasing the block did not return the live count");
    CHECK(after.high_water == during.high_water, "releasing the block moved the high-water mark");
    return true;
}

/**
 * @brief TC-BUF-DBG-002: RTPS_BUF_ALLOC_WITH_HEADROOM_DBG reserves headroom ahead of the payload.
 *
 * Verification:
 * - Correct means: the block is valid; its data starts kHeadroom octets into the block, so
 *   headroom_used() is kHeadroom and the capacity left is at least kRequest; the newest debug
 *   event is an Alloc of kRequest + kHeadroom octets.
 */
bool tc_buf_dbg_002(std::string& err) {
    const auto& pool = ::rosette::rtps::g_buf_pool;

    ::rosette::rtps::BufRef buf = RTPS_BUF_ALLOC_WITH_HEADROOM_DBG(kRequest, kHeadroom);
    CHECK(buf.valid(), "RTPS_BUF_ALLOC_WITH_HEADROOM_DBG returned no block");
    CHECK(buf.headroom_used() == kHeadroom, "the data does not start after the headroom");
    CHECK(buf.capacity() >= kRequest, "the capacity after the headroom is smaller than requested");

    ::rosette::rtps::BufPoolDebugEvent event{};
    CHECK(pool.debug_recent_event(0u, event), "the newest debug event cannot be read");
    CHECK(event.requested_size == kRequest + kHeadroom,
          "the Alloc event does not count the headroom in the requested size");

    buf.reset();
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const spec_test::TestCase tests[] = {
        {"TC-BUF-DBG-001",
         "RTPS_BUF_ALLOC_DBG, called outside the rtps namespace, allocates a block and records the "
         "class's live count, high-water mark, and an Alloc event.",
         tc_buf_dbg_001},
        {"TC-BUF-DBG-002",
         "RTPS_BUF_ALLOC_WITH_HEADROOM_DBG, called outside the rtps namespace, places the data "
         "after the headroom and records the whole request.",
         tc_buf_dbg_002},
    };

    return spec_test::run_standalone_tests(tests, sizeof(tests) / sizeof(tests[0]), argc, argv);
}
