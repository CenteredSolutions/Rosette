/**
 * @file test_buf_pool_stats.cpp
 * @brief The buffer pool's usage statistics (rtps/buf_pool_stats.hpp): TC-BUF-STATS-001 to 004.
 *
 * Built as buf_pool_stats_spec_tests with RTPS_BUFPOOL_STATS, the library's default class sizes
 * (128, 256, 512, 1024, 2048) and two blocks per class, so a class runs out in a few allocations.
 * The program supplies rtps_log_write() itself and keeps the lines it is given, so the tests can
 * check the refusal warnings and the report of buf_pool_stats_log().
 *
 *   ./buf_pool_stats_spec_tests
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/buf_pool.hpp"
#include "rosette/rtps/buf_pool_stats.hpp"

#include <stdarg.h>
#include <stdio.h>

#include <string>
#include <vector>

#ifndef RTPS_BUFPOOL_STATS
#  error "buf_pool_stats_spec_tests is built with RTPS_BUFPOOL_STATS (tests/spec/CMakeLists.txt)"
#endif

namespace rtps = rosette::rtps;

namespace {

/// Every line logged since the last clear, as "<tag>: <message>".
std::vector<std::string> g_lines;

}  // namespace

extern "C" void rtps_log_write(int /*level*/, const char* tag, const char* fmt, ...) {
    char    text[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    g_lines.push_back(std::string(tag) + ": " + text);
}

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

/**
 * @brief TC-BUF-STATS-001: a request is counted in its best-fit class, size and histogram bin.
 *
 * Verification:
 * - Correct means: requests of 100 and 200 octets count in classes 0 and 1; class 1's minimum,
 *   maximum, and summed sizes record 200; 200 lands in bin 4 of class 1's range (128, 256],
 *   whose bins are 16 octets wide; served, live, and high-water count each block, and releasing
 *   it lowers live but not high-water.
 */
bool tc_buf_stats_001(std::string& err) {
    rtps::g_buf_pool.stats_reset();
    {
        rtps::BufRef small = rtps::buf_alloc(100u);
        rtps::BufRef mid = rtps::buf_alloc(200u);
        CHECK(small.valid() && mid.valid(), "the pool refused a request it has blocks for");

        const rtps::BufPoolClassStats c0 = rtps::g_buf_pool.stats(0u);
        const rtps::BufPoolClassStats c1 = rtps::g_buf_pool.stats(1u);
        CHECK(c0.requests == 1u && c1.requests == 1u, "a request is not counted in its class");
        CHECK(c1.request_min == 200u && c1.request_max == 200u && c1.request_octets == 200u,
              "class 1 does not record the size requested");
        CHECK(rtps::g_buf_pool.stats_bin_lower(1u) == 128u
                  && rtps::g_buf_pool.stats_bin_width(1u) == 16u,
              "class 1's histogram does not span (128, 256] in 16-octet bins");
        CHECK(c1.request_bins[4] == 1u, "200 octets is not in bin 4 of class 1");
        CHECK(c1.served == 1u && c1.live == 1u && c1.high_water == 1u,
              "class 1 does not count the block it served");
    }

    const rtps::BufPoolClassStats after = rtps::g_buf_pool.stats(1u);
    CHECK(after.live == 0u && after.high_water == 1u,
          "releasing the block does not lower live, or lowers high-water");
    return true;
}

/**
 * @brief TC-BUF-STATS-002: a spill counts against the class asked for; refusals are logged
 *   sparsely.
 *
 * Verification:
 * - Correct means: a third 100-octet request, with class 0's two blocks out, is served by
 *   class 1 and counts as spilled up from class 0; a third and fourth 1024-octet request get no
 *   block (the largest class is reserved) and count as refusals of class 3, each logged; a fifth
 *   is counted but not logged, since only the 1st, 2nd, 4th, ... refusal is.
 */
bool tc_buf_stats_002(std::string& err) {
    rtps::g_buf_pool.stats_reset();
    g_lines.clear();
    rtps::BufRef small[3] = {rtps::buf_alloc(100u), rtps::buf_alloc(100u), rtps::buf_alloc(100u)};
    CHECK(small[2].valid() && small[2].capacity() == 256u, "the third request did not spill up");

    const rtps::BufPoolClassStats c0 = rtps::g_buf_pool.stats(0u);
    const rtps::BufPoolClassStats c1 = rtps::g_buf_pool.stats(1u);
    CHECK(c0.requests == 3u && c0.spilled_up == 1u && c0.served == 2u,
          "class 0 does not count the request it could not serve as spilled up");
    CHECK(c1.requests == 0u && c1.served == 1u,
          "class 1 counts the spilled request as its own demand, or not as served");

    rtps::BufRef large[5] = {};
    for (rtps::BufRef& ref : large) {
        ref = rtps::buf_alloc(1024u);
    }

    const rtps::BufPoolClassStats c3 = rtps::g_buf_pool.stats(3u);
    CHECK(!large[2].valid() && c3.requests == 5u && c3.refused == 3u,
          "class 3 does not count three refusals");
    CHECK(g_lines.size() == 2u, "refusals 1 and 2 are not each logged once, or 3 is logged");
    CHECK(g_lines[1].find("no block for 1024 octets (class 3, refusal 2") != std::string::npos,
          "the refusal warning does not name the size, class, and count");
    return true;
}

/**
 * @brief TC-BUF-STATS-003: buf_pool_stats_log() reports what stats() holds, then resets.
 *
 * Verification:
 * - Correct means: with one 300-octet block live, the report has class 2's two lines, giving
 *   its served count, live count, peak, and the request in bin 1 of (256, 512]; classes with no
 *   use are not reported; afterwards the counts are clear and high-water equals live.
 */
bool tc_buf_stats_003(std::string& err) {
    rtps::g_buf_pool.stats_reset();
    rtps::BufRef held = rtps::buf_alloc(300u);
    g_lines.clear();
    rtps::buf_pool_stats_log(true);
    CHECK(g_lines.size() == 2u, "the report is not two lines for the one class used");
    CHECK(g_lines[0] == "pool: 512 x2: served 1, live 1, peak 1 | asked 1 (300..300, mean 300), "
                        "up 0, refused 0",
          "the supply and demand line does not match stats(): " + g_lines[0]);
    CHECK(g_lines[1] == "pool: 512 sizes >256 by 32: 0 1 0 0 0 0 0 0",
          "the histogram line does not match stats(): " + g_lines[1]);

    const rtps::BufPoolClassStats after = rtps::g_buf_pool.stats(2u);
    CHECK(after.requests == 0u && after.served == 0u && after.live == 1u && after.high_water == 1u,
          "the reset does not clear the counts and keep live as the high-water mark");
    return true;
}

/**
 * @brief TC-BUF-STATS-004: the summed request size does not wrap with the request count.
 *
 * Verification:
 * - Correct means: after 70,000 requests of 65,532 octets (more than the 65,541 a 32-bit sum
 *   holds), the class records the exact sum and a mean of 65,532 octets.
 */
bool tc_buf_stats_004(std::string& err) {
    static constexpr uint16_t kSizes[2] = {256u, 65532u};
    rtps::BufPoolStats<2>     stats(kSizes);
    constexpr uint32_t        kRequests = 70000u;
    for (uint32_t i = 0; i < kRequests; ++i) {
        (void)stats.on_request(65532u, 1u);
    }

    const rtps::BufPoolClassStats& c1 = stats[1];
    CHECK(c1.requests == kRequests, "the requests are not all counted");
    CHECK(c1.request_octets == static_cast<uint64_t>(kRequests) * 65532u,
          "the summed request size wrapped");
    CHECK(c1.request_octets / c1.requests == 65532u, "the mean request size is not 65,532");
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const spec_test::TestCase tests[] = {
        {"TC-BUF-STATS-001",
         "A request counts in its best-fit class's demand, sizes, and histogram, and its block in "
         "that class's supply.",
         tc_buf_stats_001},
        {"TC-BUF-STATS-002",
         "A request served by a larger class counts as spilled up from its own; refusals count, "
         "and the 1st, 2nd, 4th, ... are logged.",
         tc_buf_stats_002},
        {"TC-BUF-STATS-003",
         "buf_pool_stats_log() reports each used class's statistics and starts a new window.",
         tc_buf_stats_003},
        {"TC-BUF-STATS-004",
         "The summed request size of a class does not wrap while the request count is far "
         "from wrapping.",
         tc_buf_stats_004},
    };

    return spec_test::run_standalone_tests(tests, sizeof(tests) / sizeof(tests[0]), argc, argv);
}
