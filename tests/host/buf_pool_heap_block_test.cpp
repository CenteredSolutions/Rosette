// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file buf_pool_heap_block_test.cpp
 * @brief Heap blocks for samples above the largest pool class, and repair sets.
 *
 * Built with RTPS_HEAP_BLOCKS, a 1 MiB sample limit, and a 2 MiB budget (see
 * CMakeLists.txt), so the limit and the budget are each reached in a few
 * allocations. Checks that small requests stay in the pool, that a heap block
 * behaves as a reference-counted BufRef, and that every failure leaves the
 * accounting unchanged.
 */

#include "test_util.hpp"

#include "rosette/rtps/buf_pool.hpp"
#include "rosette/rtps/stateful_writer.hpp"

using namespace rosette::rtps;

namespace {

constexpr uint32_t kOneMiB = 1048576u;

}  // namespace

int main() {
    // A request that fits a class never reaches the heap.
    {
        BufRef sample = buf_alloc(kPoolMaxBlockSize);
        check(sample.valid() && sample.capacity() == kPoolMaxBlockSize,
              "a request of the largest class stays in the pool");
        check(g_buf_pool.heap_octets() == 0u, "a pooled request holds no heap octets");
    }

    // One above the largest class is a heap block of exactly that size.
    {
        const uint32_t size = kPoolMaxBlockSize + 1u;
        BufRef         sample = buf_alloc(size);
        check(sample.valid() && sample.capacity() == size, "a larger request gets a heap block");
        check(g_buf_pool.heap_octets() == size, "the heap block is accounted");
    }

    check(g_buf_pool.heap_octets() == 0u, "releasing a heap block returns its octets");

    // A shared heap block lives until its last reference.
    {
        BufRef sample = buf_alloc(kOneMiB);
        check(sample.valid(), "a request at the limit is served");
        sample.data[kOneMiB - 1u] = 0x5Au;
        sample.retain();

        BufRef view{};
        view.block = sample.block;
        view.data = sample.data;
        view.size = kOneMiB;
        sample.reset();
        check(g_buf_pool.heap_octets() == kOneMiB, "a shared block outlives one reference");
        check(view.data[kOneMiB - 1u] == 0x5Au, "the last reference still reads the sample");
    }

    check(g_buf_pool.heap_octets() == 0u, "the last release frees a shared block");

    // Refusals change nothing.
    {
        BufRef too_big = buf_alloc(kOneMiB + 1u);
        check(!too_big.valid(), "a request above the sample limit is refused");
        check(g_buf_pool.heap_octets() == 0u, "a refusal above the limit accounts nothing");

        BufRef first = buf_alloc(kOneMiB);
        BufRef second = buf_alloc(kOneMiB);
        BufRef third = buf_alloc(kOneMiB);
        check(first.valid() && second.valid(), "requests within the budget are served");
        check(!third.valid(), "a request past the budget is refused");
        check(g_buf_pool.heap_octets() == 2u * kOneMiB,
              "a refusal past the budget accounts nothing");
    }

    check(g_buf_pool.heap_octets() == 0u, "the budget is whole again");

    // A repair set names every fragment a receiver can track.
    {
        FragmentRepairSet set{};
        check(set.add(1u) && set.add(65u) && set.add(256u), "fragments 1, 65 and 256 are nameable");
        check(set.contains(1u) && set.contains(65u) && set.contains(256u),
              "added fragments read back");
        check(!set.contains(2u) && !set.contains(64u), "other fragments are absent");
        check(!set.add(0u) && !set.add(257u), "numbers outside 1..256 are refused");

        FragmentRepairSet other{};
        check(other.add(200u), "fragment 200 is nameable");
        set.merge(other);
        check(set.contains(200u) && set.contains(65u), "a merge keeps both sets");
    }

    return test_summary();
}
