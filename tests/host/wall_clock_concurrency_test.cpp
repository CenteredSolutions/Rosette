/**
 * @file tests/host/wall_clock_concurrency_test.cpp
 * @brief WallClock's lock-free reads under concurrent writes: no torn offset, ever.
 *
 * The offset is published as two 32-bit halves under a sequence lock
 * (src/platform/wall_clock.cpp). Two writer threads alternate it between two
 * values whose halves both differ for kWriteTime, while reader threads read
 * it as fast as they can; a read that mixed the halves of the two values
 * would return neither. Without the sequence lock, this run sees hundreds
 * of thousands of torn reads on a four-core Cortex-A76.
 *
 * - TC-PLAT-WC-001: every offset a reader sees is one a writer stored.
 * - TC-PLAT-WC-002: try_now_us() and now_us() agree with the offset, so the
 *   synchronized flag travels with it.
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"
#include "test_util.hpp"

namespace pf = rosette::platform;

namespace {

/// The fake tick: constant, so now_us() is the offset plus this.
constexpr uint64_t kTickUs = 1000u;

/// Two offsets that differ in both 32-bit halves.
constexpr int64_t kOffsetA = 0x0000'0001'0000'0000LL;
constexpr int64_t kOffsetB = 0x0000'0002'FFFF'FFFFLL;

constexpr int                       kWriters = 2;
constexpr int                       kReaders = 4;
/// How long the writers store: long enough that reads overlap writes millions of times.
constexpr std::chrono::milliseconds kWriteTime{500};

bool is_stored(int64_t offset) {
    return offset == kOffsetA || offset == kOffsetB;
}

}  // namespace

namespace rosette::platform {

uint64_t monotonic_us() noexcept {
    return kTickUs;
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    (void)unix_us;
    return false;
}

}  // namespace rosette::platform

int main() {
    pf::WallClock::set_offset_us(kOffsetA);

    std::atomic<bool>     writing{true};
    std::atomic<uint64_t> torn{0u};
    std::atomic<uint64_t> mismatched{0u};
    std::atomic<uint64_t> reads{0u};

    std::vector<std::thread> readers;
    for (int r = 0; r < kReaders; ++r) {
        readers.emplace_back([&] {
            uint64_t local_reads = 0u;
            while (writing.load(std::memory_order_relaxed)) {
                if (!is_stored(pf::WallClock::offset_us())) {
                    torn.fetch_add(1u);
                }

                int64_t    unix_us = 0;
                const bool synced = pf::WallClock::try_now_us(unix_us);
                if (!synced || !is_stored(unix_us - static_cast<int64_t>(kTickUs))
                    || !is_stored(pf::WallClock::now_us() - static_cast<int64_t>(kTickUs))) {
                    mismatched.fetch_add(1u);
                }

                ++local_reads;
            }

            reads.fetch_add(local_reads);
        });
    }

    std::vector<std::thread> writers;
    for (int w = 0; w < kWriters; ++w) {
        writers.emplace_back([w] {
            const auto deadline = std::chrono::steady_clock::now() + kWriteTime;
            for (int i = w; std::chrono::steady_clock::now() < deadline; ++i) {
                pf::WallClock::set_offset_us((i % 2 == 0) ? kOffsetA : kOffsetB);
            }
        });
    }

    for (std::thread& writer : writers) {
        writer.join();
    }

    writing.store(false);
    for (std::thread& reader : readers) {
        reader.join();
    }

    std::printf("reads=%llu torn=%llu mismatched=%llu\n",
                static_cast<unsigned long long>(reads.load()),
                static_cast<unsigned long long>(torn.load()),
                static_cast<unsigned long long>(mismatched.load()));
    check(reads.load() > 0u, "readers ran while the writers stored");
    check(torn.load() == 0u, "TC-PLAT-WC-001 no reader saw a torn offset");
    check(mismatched.load() == 0u,
          "TC-PLAT-WC-002 try_now_us() and now_us() read a whole, synchronized offset");
    return test_summary();
}
