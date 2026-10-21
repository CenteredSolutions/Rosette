/**
 * @file buf_pool_class_test.cpp
 * @brief Size-class selection in the RTPS buffer pool (rtps/buf_pool.hpp).
 *
 * Built with two blocks per class (see CMakeLists.txt) so every class can be
 * exhausted in a few allocations. Checks that a request takes the smallest
 * class that fits, promotes to a larger class when that one is exhausted,
 * and never promotes into the largest class, which is reserved for whole
 * samples too large for any other.
 */

#include "test_util.hpp"

#include "rosette/rtps/buf_pool.hpp"

using namespace rosette::rtps;

int main() {
    // Smallest fit.
    {
        BufRef a = buf_alloc(RTPS_POOL_CLASS0_SIZE);
        check(a.valid() && a.capacity() == RTPS_POOL_CLASS0_SIZE,
              "a request that fits class 0 takes a class-0 block");
    }

    // Promotion below the largest class.
    {
        BufRef a = buf_alloc(RTPS_POOL_CLASS0_SIZE);
        BufRef b = buf_alloc(RTPS_POOL_CLASS0_SIZE);
        BufRef c = buf_alloc(RTPS_POOL_CLASS0_SIZE);
        check(a.valid() && b.valid() && c.valid() && c.capacity() == RTPS_POOL_CLASS1_SIZE,
              "with class 0 exhausted, a class-0 request promotes to class 1");
    }

    // No promotion into the largest class.
    {
        BufRef a = buf_alloc(RTPS_POOL_CLASS3_SIZE);
        BufRef b = buf_alloc(RTPS_POOL_CLASS3_SIZE);
        BufRef c = buf_alloc(RTPS_POOL_CLASS3_SIZE);
        check(a.valid() && b.valid(), "both class-3 blocks are allocated");
        check(!c.valid(), "with class 3 exhausted, a class-3 request does not take a sample block");
        BufRef big = buf_alloc(RTPS_POOL_CLASS3_SIZE + 1u);
        check(big.valid() && big.capacity() == RTPS_POOL_CLASS4_SIZE,
              "a request larger than class 3 still gets a sample block");
    }

    // The largest class serves every request it alone can hold.
    {
        BufRef a = buf_alloc(RTPS_POOL_CLASS4_SIZE);
        BufRef b = buf_alloc(RTPS_POOL_CLASS4_SIZE);
        BufRef c = buf_alloc(RTPS_POOL_CLASS4_SIZE);
        check(a.valid() && b.valid() && !c.valid(),
              "sample blocks run out only after every one is in use");
        BufRef too_big = buf_alloc(RTPS_POOL_CLASS4_SIZE + 1u);
        check(!too_big.valid(), "a request larger than every class fails");
    }

    return test_summary();
}
