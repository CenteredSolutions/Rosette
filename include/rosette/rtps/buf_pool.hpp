// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/buf_pool.hpp
 * @brief Multi-size-class slab pool allocator for RTPS payload buffers.
 *
 * Overview
 * --------
 * A single fixed-size block pool wastes memory when small control
 * messages (HEARTBEAT, ACKNACK, GAP — a few dozen bytes) must compete
 * with large DATA payloads and fragment-reassembly buffers (up to 2 KB).
 *
 * This pool partitions its storage into N independent size classes.
 * buf_alloc(min_size) finds the smallest class whose block capacity is
 * >= min_size and pops one block from it.  Callers get a BufRef whose
 * block pointer is aware of which class it belongs to, so release()
 * returns the block to the correct class automatically.
 *
 * Key properties
 * --------------
 *  - Zero heap.  All storage is statically allocated inside MultiBufPool.
 *  - O(N) alloc where N is the number of size classes (≤ 8 in practice).
 *  - Atomic reference counting (memory_order_release/acquire) so TX and
 *    RX tasks may retain/release the same block concurrently.
 *  - Free lists guarded by one pool-wide platform Mutex (buf_pool_mutex()),
 *    so threads, tasks, and both cores of a dual-core MCU may allocate and
 *    release concurrently. Interrupt handlers must not allocate or release
 *    (platform.hpp concurrency rule).
 *  - BufRef stays 12 bytes on ARM32 (block* + data* + size).
 *    The size class index lives inside BufBlock, not in BufRef.
 *  - BufRef::capacity() returns the actual usable bytes of the block,
 *    which is >= the requested size.  MessageBuilder and other builders
 *    use this at runtime instead of a compile-time constant.
 *
 * buf_alloc(min_size) behaviour
 * ------------------------------
 * 1. Find the smallest class whose block_size >= min_size.
 * 2. If that class has a free block, return it (exact-fit).
 * 3. If not, try each larger class in order (promote-on-exhaustion), except
 *    the largest: it holds whole samples too large for any other class
 *    (a sample is always stored contiguously, and fragmented only on the
 *    wire), so a request that fits a smaller class never takes one. Small
 *    traffic running a class dry therefore cannot starve large samples.
 *    The caller can inspect BufRef::capacity() to know what it received.
 * 4. If all eligible classes are exhausted, return an invalid BufRef.
 *
 * Size-class configuration
 * ------------------------
 * Every class size and count is a RTPS_POOL_CLASS<n>_SIZE / _COUNT macro.
 * Firmware builds set them from board inputs in cmake/rosette_rtps.cmake
 * (control messages, discovery announcements, small samples, one datagram,
 * and ROSETTE_RTPS_MAX_SAMPLE_SIZE); rmw_rosette sets its own in
 * rmw_rosette/cmake/rmw_rosette_capacity.cmake. The defaults below serve
 * host unit tests:
 *
 *  Class 0:  128 bytes, 8 slots
 *  Class 1:  256 bytes, 8 slots
 *  Class 2:  512 bytes, 8 slots
 *  Class 3: 1024 bytes, 4 slots
 *  Class 4: 2048 bytes, 4 slots  (largest: samples only)
 *
 * MessageBuilder compatibility
 * ----------------------------
 * MessageBuilder::write_bytes() previously compared against the compile-
 * time constant RTPS_POOL_BLOCK_SIZE.  It now calls buf_.capacity() so
 * that the correct runtime limit is used regardless of which size class
 * the builder's buffer came from.
 */

#pragma once

#include <string.h>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include "log.hpp"
#include "types.hpp"

#include "rosette/platform/platform.hpp"

#ifdef RTPS_BUFPOOL_STATS
#  include "rosette/rtps/buf_pool_stats.hpp"
#endif

namespace rosette::rtps {

/// Guards every size class's free list, the debug counters, and the usage statistics. The critical
/// sections are a few pointer moves long, so one lock for all classes costs
/// nothing measurable and keeps lock ordering trivial.
inline rosette::platform::Mutex& buf_pool_mutex() noexcept {
    static rosette::platform::Mutex mutex;
    return mutex;
}


// ============================================================
// Size-class configuration
//
// Each pair is { block_size_in_bytes, number_of_blocks }.
// Override any of these in your project config.hpp before
// including rtps.hpp.
// ============================================================

#ifndef RTPS_POOL_CLASS0_SIZE
#  define RTPS_POOL_CLASS0_SIZE   128u
#endif
#ifndef RTPS_POOL_CLASS0_COUNT
#  define RTPS_POOL_CLASS0_COUNT    8u
#endif
#ifndef RTPS_POOL_CLASS1_SIZE
#  define RTPS_POOL_CLASS1_SIZE   256u
#endif
#ifndef RTPS_POOL_CLASS1_COUNT
#  define RTPS_POOL_CLASS1_COUNT    8u
#endif
#ifndef RTPS_POOL_CLASS2_SIZE
#  define RTPS_POOL_CLASS2_SIZE   512u
#endif
#ifndef RTPS_POOL_CLASS2_COUNT
#  define RTPS_POOL_CLASS2_COUNT    8u
#endif
#ifndef RTPS_POOL_CLASS3_SIZE
#  define RTPS_POOL_CLASS3_SIZE  1024u
#endif
#ifndef RTPS_POOL_CLASS3_COUNT
#  define RTPS_POOL_CLASS3_COUNT    4u
#endif
#ifndef RTPS_POOL_CLASS4_SIZE
#  define RTPS_POOL_CLASS4_SIZE  2048u
#endif
#ifndef RTPS_POOL_CLASS4_COUNT
#  define RTPS_POOL_CLASS4_COUNT    4u
#endif

/// Number of size classes.  Increase this if you add more classes above.
static constexpr uint8_t kPoolClassCount = 5u;

/// The largest single block size — used as the upper bound for requests.
static constexpr uint32_t kPoolMaxBlockSize = RTPS_POOL_CLASS4_SIZE;

// Heap blocks hold samples larger than every static class on a hosted build.
// A sample that fits a class never reaches the heap, so small messages keep
// the pool's fixed cost, and a build without RTPS_HEAP_BLOCKS has no heap path.
#ifndef RTPS_HEAP_BLOCKS
#  define RTPS_HEAP_BLOCKS 0
#endif
#ifndef RTPS_MAX_HEAP_BLOCK_SIZE
#  define RTPS_MAX_HEAP_BLOCK_SIZE RTPS_POOL_CLASS4_SIZE
#endif
#ifndef RTPS_HEAP_BLOCK_BUDGET
#  define RTPS_HEAP_BLOCK_BUDGET (64u * 1024u * 1024u)
#endif

/// Largest sample this build can store: what senders and receivers are held to.
static constexpr uint32_t kMaxSampleOctets =
    RTPS_HEAP_BLOCKS ? RTPS_MAX_HEAP_BLOCK_SIZE : kPoolMaxBlockSize;

/// BufBlock::size_class of a heap block charged to samples this process wrote.
static constexpr uint8_t kHeapSizeClass = 0xFEu;
/// BufBlock::size_class of a heap block charged to samples peers announced.
static constexpr uint8_t kHeapReceivedSizeClass = 0xFDu;

/// One half of RTPS_HEAP_BLOCK_BUDGET. Peers choose how much a DATA_FRAG
/// sequence allocates, so received samples have their own half and can never
/// consume the half that publishing depends on.
enum class HeapBudgetSide : uint8_t { Local = 0u, Received = 1u };

static_assert(kMaxSampleOctets < (1u << 24), "BufBlock::block_cap holds 24 bits");
#if RTPS_HEAP_BLOCKS
static_assert(RTPS_HEAP_BLOCK_BUDGET / 2u >= RTPS_MAX_HEAP_BLOCK_SIZE,
              "each half of the heap budget holds at least one largest sample");
#endif

#ifndef RTPS_BUF_BLOCK_SIZE
#  define RTPS_BUF_BLOCK_SIZE RTPS_POOL_CLASS4_SIZE
#endif

#ifdef RTPS_BUFPOOL_DEBUG
#  ifndef RTPS_BUFPOOL_DEBUG_EVENT_CAP
#    define RTPS_BUFPOOL_DEBUG_EVENT_CAP 256u
#  endif
#endif

#if defined(__GNUC__) || defined(__clang__)
#  define RTPS_BUFPOOL_CALLER_PC() \
    (static_cast<uintptr_t>(reinterpret_cast<uintptr_t>(__builtin_return_address(0))))
#else
#  define RTPS_BUFPOOL_CALLER_PC() (0u)
#endif

#ifdef RTPS_BUFPOOL_DEBUG
enum class BufPoolDebugOp : uint8_t {
    Alloc = 1u,
    AllocFail = 2u,
    Retain = 3u,
    Release = 4u,
    Free = 5u
};

struct BufPoolDebugClassStats {
    uint32_t alloc_ok = 0;
    uint32_t alloc_fail = 0;
    uint32_t retain_calls = 0;
    uint32_t release_calls = 0;
    uint32_t frees = 0;
    uint32_t live = 0;
    uint32_t high_water = 0;
};

struct BufPoolDebugEvent {
    uint32_t tick_ms = 0;
    uint32_t seq = 0;
    uintptr_t block = 0;
    uintptr_t caller_pc = 0;
    uint32_t requested_size = 0;
    uint16_t block_cap = 0;
    uint8_t  size_class = 0xFFu;
    uint8_t  op = 0;
    int16_t  ref_before = 0;
    int16_t  ref_after = 0;
};
#endif

// ============================================================
// BufBlock — internal per-block header
//
// Stored immediately before the payload bytes in each slab slot.
// The size_class field allows release() to find the owning slab
// without any extra pointer arithmetic or a back-pointer to the pool.
// ============================================================

struct BufBlock {
    std::atomic<int> refcount;      ///< 0 == on the free list; ≥1 == live.
    BufBlock*    next_free;     ///< Intrusive free-list link (valid when refcount==0).
    uint32_t         block_cap : 24;  ///< Usable bytes following this header.
    uint32_t         size_class : 8;  ///< Index into the classes, or a heap class.
#ifdef RTPS_BUFPOOL_DEBUG
    uintptr_t    dbg_alloc_pc = 0;
    uint32_t     dbg_alloc_seq = 0;
    uintptr_t    dbg_last_touch_pc = 0;
#endif

    /// Payload starts immediately after the header.
    Octet* payload() noexcept {
        return reinterpret_cast<Octet*>(this + 1);
    }
};

#ifndef RTPS_BUFPOOL_DEBUG
static_assert(sizeof(BufBlock) == (sizeof(void*) == 4u ? 12u : 24u),
              "a block header stays three words on a 32-bit target");
#endif

// ============================================================
// BufRef — public reference-counted buffer handle
//
// Layout (ARM32): block*(4) + data*(4) + size(4) = 12 bytes.
//
// Headroom model
// --------------
// When a BufRef is allocated with headroom, `data` is positioned
// headroom bytes *after* the start of the block's payload region:
//
//   block->payload()          data                  data + capacity()
//        │                     │                           │
//        ▼                     ▼                           ▼
//   ┌────────────────────────────────────────────────────────┐
//   │  <── headroom_used() ──>│<──── capacity() ────────────>│
//   │   (reserved for prepend)│  (caller writes CDR here)    │
//   └────────────────────────────────────────────────────────┘
//   ◄──────────────── block->block_cap ─────────────────────►
//
// capacity() and headroom_used() are computed from `data` and
// `block->payload()` — no extra field stored in BufRef itself.
//
// Prepend path (zero-copy TX)
// ---------------------------
// After writing CDR payload at `data`, the caller calls prepend(n)
// to claim n bytes immediately before `data` for a message header.
// prepend() decrements `data` by n and increments `size` by n.
// The resulting BufRef describes the complete contiguous region
// [header | payload] with no copy required.
// ============================================================

struct BufRef {
    BufBlock* block = nullptr;  ///< Owning pool block; null == not allocated.
    Octet*    data  = nullptr;  ///< Start of valid/writable region.
    uint32_t  size  = 0;        ///< Bytes at [data .. data+size).

    bool valid() const noexcept { return block != nullptr; }

    // ----------------------------------------------------------
    // capacity()
    //
    // Usable bytes from `data` to the end of the block's payload
    // area.  Always >= the originally requested size.
    //
    // Callers use this for overflow-checking instead of any
    // compile-time constant, so correct limits are enforced
    // regardless of which size class was allocated.
    // ----------------------------------------------------------
    uint32_t capacity() const noexcept {
        if (!block) return 0u;
        // data is always >= block->payload() (guaranteed by alloc/prepend).
        const uint32_t used = static_cast<uint32_t>(
            data - block->payload());
        return static_cast<uint32_t>(block->block_cap) - used;
    }

    // ----------------------------------------------------------
    // headroom_used()
    //
    // Bytes between block->payload() and the current `data` pointer.
    // These bytes have been reserved for future prepend() calls and
    // may not yet be written.
    // ----------------------------------------------------------
    uint32_t headroom_used() const noexcept {
        if (!block) return 0u;
        return static_cast<uint32_t>(data - block->payload());
    }

    // ----------------------------------------------------------
    // headroom_available()
    //
    // Bytes already reserved (between block start and data) that
    // can still be claimed by further prepend() calls.
    //
    // headroom_available() == headroom_used() — bytes already
    // prepended since allocation.  In other words: if you called
    // buf_alloc_with_headroom(payload_size, H) and have not yet
    // called prepend(), headroom_available() == H.
    // ----------------------------------------------------------
    uint32_t headroom_available() const noexcept {
        return headroom_used();   // All reserved bytes are claimable.
    }

    // ----------------------------------------------------------
    // prepend(n)
    //
    // Claim n bytes immediately before the current `data` pointer
    // and expand `size` to include them.  Returns a pointer to the
    // new start of the data region (i.e. the space for the header).
    //
    // This is the zero-copy prepend operation:
    //
    //   Before prepend(20):
    //     data ──► [CDR bytes]      size = payload_len
    //   After prepend(20):
    //     data ──► [20-byte header][CDR bytes]  size = 20 + payload_len
    //
    // The caller writes the header into the returned pointer.
    //
    // @param n  Number of bytes to prepend.  Must be <= headroom_available().
    // @returns  Pointer to the newly claimed region, or nullptr if
    //           insufficient headroom.
    // ----------------------------------------------------------
    Octet* prepend(uint32_t n) noexcept {
        if (!block || n == 0u) return nullptr;
        const uint32_t avail = headroom_available();
        if (n > avail) return nullptr;
        data -= n;
        size += n;
        return data;
    }

    // ----------------------------------------------------------
    // append(n)
    //
    // Claim n bytes immediately after the current region
    // [data .. data+size) and expand `size` to include them.
    // Returns a pointer to the claimed bytes for the caller to fill.
    //
    // This is the tail counterpart of prepend(): a writer appends a
    // HEARTBEAT submessage after a DATA it framed in place, so both
    // travel in one datagram (§8.4.15.4). Allocations that want this
    // reserve kRtpsHeartbeatTailroom beyond their payload.
    //
    // @param n  Number of bytes to append.  Must be <= capacity() - size.
    // @returns  Pointer to the newly claimed region, or nullptr if the
    //           block has fewer than n bytes past the region.
    // ----------------------------------------------------------
    Octet* append(uint32_t n) noexcept {
        if (!block || n == 0u || n > capacity() - size) return nullptr;
        Octet* tail = data + size;
        size += n;
        return tail;
    }

    // ----------------------------------------------------------
    // Lifecycle
    // ----------------------------------------------------------

    void retain() const noexcept;
    void release() noexcept;

    void reset() noexcept {
        if (valid()) release();
        block = nullptr;
        data  = nullptr;
        size  = 0;
    }

    // Move-only — no accidental copies.
    BufRef() noexcept = default;
    ~BufRef() noexcept { reset(); }

    /**
     * @brief Move constructor for BufRef.
     *
     * Transfers ownership of the buffer resources from another BufRef instance (`o`)
     * to this instance. After the move, the source BufRef (`o`) is left in a
     * valid but empty state (its pointers are set to nullptr and size to 0).
     *
     * @param o The BufRef instance to move from.
     */
    BufRef(BufRef&& o) noexcept
        : block(o.block), data(o.data), size(o.size)
    { o.block = nullptr; o.data = nullptr; o.size = 0; }

    BufRef& operator=(BufRef&& o) noexcept {
        if (this != &o) {
            if (valid()) release();
            block = o.block; data = o.data; size = o.size;
            o.block = nullptr; o.data = nullptr; o.size = 0;
        }
        return *this;
    }

    BufRef(const BufRef&)            = delete;
    BufRef& operator=(const BufRef&) = delete;
};

// ============================================================
// SlabClass<BlockSize, BlockCount>
//
// One size class: BlockCount contiguous slots, each comprising a
// BufBlock header followed by BlockSize bytes of payload.
//
// Using a flat array of StorageSlot (header + payload as a single
// struct) ensures correct alignment and simplifies pointer arithmetic.
// BufBlock* is cast directly from &slots_[i].hdr.
// ============================================================

template<uint16_t BlockSize, uint8_t BlockCount>
class SlabClass {
    static_assert(BlockSize  >= 16,  "BlockSize must be >= 16");
    static_assert(BlockCount >= 1,   "BlockCount must be >= 1");

public:

    static constexpr uint16_t kBlockSize  = BlockSize;
    static constexpr uint8_t  kBlockCount = BlockCount;

    /// Initialise the free list.  class_idx is stored in every block header.
    void init(uint8_t class_idx) noexcept {
        free_head_ = nullptr;
        for (int8_t i = static_cast<int8_t>(BlockCount) - 1; i >= 0; --i) {
            BufBlock* b     = &slots_[i].hdr;
            b->refcount     = 0;
            b->block_cap    = BlockSize;
            b->size_class   = class_idx;
            b->next_free    = free_head_;
            free_head_      = b;
        }
    }

    /// Pop one block.  Returns nullptr on exhaustion.  Thread-safe; not for ISRs.
    BufBlock* alloc() noexcept {
        BufBlock* b = nullptr;
        buf_pool_mutex().lock();
        if (free_head_) {
            b            = free_head_;
            free_head_   = b->next_free;
            b->next_free = nullptr;
        }
        buf_pool_mutex().unlock();
        if (b) {
            b->refcount = 1;
        }
        return b;
    }

    /// Push a block back onto the free list.  Thread-safe; not for ISRs.
    void free_block(BufBlock* b) noexcept {
        std::atomic_thread_fence(std::memory_order_acquire);
        buf_pool_mutex().lock();
        b->next_free = free_head_;
        free_head_   = b;
        buf_pool_mutex().unlock();
    }

    /// Snapshot of free-block count (diagnostic only; not locked during walk).
    uint8_t available() const noexcept {
        BufBlock* head;
        buf_pool_mutex().lock();
        head = free_head_;
        buf_pool_mutex().unlock();
        uint8_t n = 0;
        for (BufBlock* b = head; b; b = b->next_free) ++n;
        return n;
    }

private:

    /// One contiguous slot: header + payload in the same struct so that
    /// `&hdr + 1` is always the start of the payload.
    struct StorageSlot {
        BufBlock hdr;
        Octet    payload[BlockSize];
    };

    StorageSlot slots_[BlockCount];
    BufBlock*   free_head_ = nullptr;
};

// ============================================================
// MultiBufPool
//
// Aggregates all size classes into one object.  A single instance
// lives for the program lifetime (typically a global in buf_pool.cpp).
//
// The classes are ordered by ascending block size.  alloc(min_size)
// performs a linear scan from class 0 upward.  With five classes
// this is at most five iterations — negligible cost.
// ============================================================

class MultiBufPool {
public:

    MultiBufPool() noexcept { init(); }

    MultiBufPool(const MultiBufPool&)            = delete;
    MultiBufPool& operator=(const MultiBufPool&) = delete;

    /**
     * @brief Allocates a block of at least `min_size` usable bytes.
     *
     * Takes the smallest class that fits and, when it is exhausted, the
     * next larger one, but never promotes into the largest class: that
     * class is reserved for samples no smaller class can hold.
     *
     * @param[in] min_size  Minimum usable bytes; defaults to the smallest
     *   class size.
     * @param[in] caller_pc Allocation site recorded by RTPS_BUFPOOL_DEBUG,
     *   and logged with a refusal under RTPS_BUFPOOL_STATS.
     * @return A block whose capacity() may exceed `min_size`, or an invalid
     *   BufRef when every eligible class is exhausted or `min_size` exceeds
     *   the largest class. With RTPS_HEAP_BLOCKS, a request above the largest
     *   class is served from the heap, and refused above
     *   RTPS_MAX_HEAP_BLOCK_SIZE or past the local half of
     *   RTPS_HEAP_BLOCK_BUDGET.
     *
     * With RTPS_BUFPOOL_STATS, every call is counted (stats()), and a
     * refusal is logged as a warning on the 1st, 2nd, 4th, ... refusal of
     * the request's best-fit class.
     *
     * Verification:
     * - Correct means: smallest fit first; promotion stops below the
     *   largest class for requests a smaller class can hold.
     * - Covered by: tests/host/buf_pool_class_test.cpp; the statistics by
     *   TC-BUF-STATS-001 and TC-BUF-STATS-002.
     */
    BufRef alloc(uint32_t                   min_size = RTPS_POOL_CLASS0_SIZE,
                 [[maybe_unused]] uintptr_t caller_pc = 0u) noexcept {
#if RTPS_HEAP_BLOCKS
        if (min_size > kPoolMaxBlockSize) {
            return alloc_heap(min_size, HeapBudgetSide::Local);
        }
#endif

        constexpr uint8_t kLargest = kPoolClassCount - 1u;
        const bool fits_smaller_class = min_size <= class_sizes_[kLargest - 1u];
        for (uint8_t c = 0; c < kPoolClassCount; ++c) {
            if (class_sizes_[c] < min_size) continue;  // Class too small.
            if (c == kLargest && fits_smaller_class) break;  // Reserved for samples.
            BufBlock* b = dispatch_alloc(c);
            if (!b) continue;  // Class exhausted — promote to next.
#ifdef RTPS_BUFPOOL_DEBUG
            debug_on_alloc(b, min_size, caller_pc);
#endif
#ifdef RTPS_BUFPOOL_STATS
            stats_on_alloc(min_size, c, caller_pc);
#endif
            BufRef r{};
            r.block = b;
            r.data  = b->payload();  // data starts at block start; no headroom.
            r.size  = 0;
            // capacity() is now computed from (block->block_cap - (data - payload()))
            // so no stored field to set here.
            return r;
        }
#ifdef RTPS_BUFPOOL_DEBUG
        debug_on_alloc_fail(min_size, caller_pc);
#endif
#ifdef RTPS_BUFPOOL_STATS
        stats_on_alloc(min_size, kPoolClassCount, caller_pc);
#endif
        return {};  // All eligible classes exhausted.
    }

    // ----------------------------------------------------------
    // retain(block) / release(block)
    //
    // Called from BufRef::retain() and BufRef::release().
    // release() routes the block back to its owning size class
    // via the size_class index stored in the block header.
    // ----------------------------------------------------------
    static int retain(BufBlock* b) noexcept {
        if (!b) return 0;
        return b->refcount.fetch_add(1, std::memory_order_relaxed);
    }

    int release(BufBlock* b) noexcept {
        if (!b) return 0;
        int prev = b->refcount.fetch_sub(1, std::memory_order_acq_rel);
#ifdef RTPS_BUFPOOL_DEBUG
        debug_on_release(b, prev);
#endif
        if (prev == 1) {
#if RTPS_HEAP_BLOCKS
            if (b->size_class == kHeapSizeClass || b->size_class == kHeapReceivedSizeClass) {
                free_heap(b);
                return prev;
            }
#endif

#ifdef RTPS_BUFPOOL_STATS
            buf_pool_mutex().lock();
            stats_.on_freed(b->size_class);
            buf_pool_mutex().unlock();
#endif
            dispatch_free(b);
        }
        if (prev == 0) {
            RTPS_LOG_E("bufpool", "release called on block with refcount 0 (double free?)");
        }
        return prev;
    }

    // ----------------------------------------------------------
    // Diagnostics
    // ----------------------------------------------------------

    struct ClassInfo {
        uint16_t block_size;
        uint8_t  total;
        uint8_t  available;
    };

    /**
     * @brief Allocates storage for a sample whose size a remote peer announced.
     *
     * Behaves as alloc(), except that a request above the largest class is
     * charged to the received half of the heap budget. The charge lasts until
     * the last reference is released, so a completed sample waiting in a
     * reader's history still counts against peers.
     *
     * @param[in] min_size Payload octets the peer announced.
     * @return A block of at least `min_size` octets, or an invalid BufRef when
     *   the pool, the sample limit, or the received half refuses it.
     *
     * Verification:
     * - Correct means: received samples can never reduce the octets available
     *   to alloc(), and the reverse.
     * - Covered by: tests/host/buf_pool_heap_block_test.cpp.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.3 (DataFrag)
     */
    BufRef alloc_received(uint32_t min_size) noexcept {
#if RTPS_HEAP_BLOCKS
        if (min_size > kPoolMaxBlockSize) {
            return alloc_heap(min_size, HeapBudgetSide::Received);
        }
#endif
        return alloc(min_size, RTPS_BUFPOOL_CALLER_PC());
    }

#if RTPS_HEAP_BLOCKS
    /// @brief Octets currently held by heap blocks on both sides of the budget.
    size_t heap_octets() const noexcept {
        return heap_octets(HeapBudgetSide::Local) + heap_octets(HeapBudgetSide::Received);
    }

    /// @brief Octets currently held by heap blocks charged to one side.
    size_t heap_octets(HeapBudgetSide side) const noexcept {
        return heap_octets_[static_cast<uint8_t>(side)].load(std::memory_order_relaxed);
    }
#endif

    ClassInfo class_info(uint8_t c) const noexcept {
        if (c >= kPoolClassCount) return {};
        ClassInfo ci{};
        ci.block_size = class_sizes_[c];
        ci.total      = class_counts_[c];
        ci.available  = dispatch_available(c);
        return ci;
    }

    uint8_t class_count() const noexcept { return kPoolClassCount; }

#ifdef RTPS_BUFPOOL_STATS
    /// @brief A copy of class @p c's usage statistics (buf_pool_stats.hpp);
    ///   empty when @p c is out of range.
    BufPoolClassStats stats(uint8_t c) const noexcept {
        if (c >= kPoolClassCount) {
            return {};
        }

        buf_pool_mutex().lock();
        const BufPoolClassStats s = stats_[c];
        buf_pool_mutex().unlock();
        return s;
    }

    /// @brief Low end of class @p c's request-size histogram (BufPoolStats::bin_lower()).
    uint32_t stats_bin_lower(uint8_t c) const noexcept { return stats_.bin_lower(c); }

    /// @brief Width of one bin of class @p c's histogram (BufPoolStats::bin_width()).
    uint32_t stats_bin_width(uint8_t c) const noexcept { return stats_.bin_width(c); }

    /// @brief Starts a new measurement window (BufPoolStats::reset()).
    void stats_reset() noexcept {
        buf_pool_mutex().lock();
        stats_.reset();
        buf_pool_mutex().unlock();
    }
#endif

#ifdef RTPS_BUFPOOL_DEBUG
    BufPoolDebugClassStats debug_class_stats(uint8_t c) const noexcept {
        if (c >= kPoolClassCount) return {};
        return dbg_stats_[c];
    }

    uint32_t debug_event_count() const noexcept { return dbg_event_seq_; }

    bool debug_recent_event(uint32_t newest_index, BufPoolDebugEvent& out) const noexcept {
        const uint32_t total = dbg_event_seq_;
        if (newest_index >= total) return false;
        const uint32_t ring_cap = static_cast<uint32_t>(RTPS_BUFPOOL_DEBUG_EVENT_CAP);
        const uint32_t slot = (total - 1u - newest_index) % ring_cap;
        out = dbg_events_[slot];
        return true;
    }

    uint32_t debug_live_total() const noexcept {
        uint32_t total = 0;
        for (uint8_t c = 0; c < kPoolClassCount; ++c) total += dbg_stats_[c].live;
        return total;
    }

    void debug_mark_retain(BufBlock* b, int prev, uintptr_t caller_pc) noexcept {
        if (!b || b->size_class >= kPoolClassCount) return;
        buf_pool_mutex().lock();
        BufPoolDebugClassStats& s = dbg_stats_[b->size_class];
        ++s.retain_calls;
        b->dbg_last_touch_pc = caller_pc;
        debug_record_event(BufPoolDebugOp::Retain, b, b->block_cap, prev, prev + 1, caller_pc);
        buf_pool_mutex().unlock();
    }
#endif

private:

    // ── Concrete slab instances ───────────────────────────────

    SlabClass<RTPS_POOL_CLASS0_SIZE, RTPS_POOL_CLASS0_COUNT> c0_;
    SlabClass<RTPS_POOL_CLASS1_SIZE, RTPS_POOL_CLASS1_COUNT> c1_;
    SlabClass<RTPS_POOL_CLASS2_SIZE, RTPS_POOL_CLASS2_COUNT> c2_;
    SlabClass<RTPS_POOL_CLASS3_SIZE, RTPS_POOL_CLASS3_COUNT> c3_;
    SlabClass<RTPS_POOL_CLASS4_SIZE, RTPS_POOL_CLASS4_COUNT> c4_;

    // Metadata tables — parallel arrays indexed by class index.
    static constexpr uint16_t class_sizes_[kPoolClassCount] = {
        RTPS_POOL_CLASS0_SIZE, RTPS_POOL_CLASS1_SIZE,
        RTPS_POOL_CLASS2_SIZE, RTPS_POOL_CLASS3_SIZE,
        RTPS_POOL_CLASS4_SIZE
    };
    static constexpr uint8_t class_counts_[kPoolClassCount] = {
        RTPS_POOL_CLASS0_COUNT, RTPS_POOL_CLASS1_COUNT,
        RTPS_POOL_CLASS2_COUNT, RTPS_POOL_CLASS3_COUNT,
        RTPS_POOL_CLASS4_COUNT
    };

#ifdef RTPS_BUFPOOL_DEBUG
    BufPoolDebugClassStats dbg_stats_[kPoolClassCount] = {};
    BufPoolDebugEvent dbg_events_[RTPS_BUFPOOL_DEBUG_EVENT_CAP] = {};
    uint32_t dbg_event_seq_ = 0;
#endif

#ifdef RTPS_BUFPOOL_STATS
    BufPoolStats<kPoolClassCount> stats_{class_sizes_};

    /// @brief Counts one alloc() served from class @p c, or refused when
    ///   @p c is kPoolClassCount, and logs the refusals stats_ selects.
    void stats_on_alloc(uint32_t min_size, uint8_t c, uintptr_t caller_pc) noexcept {
        buf_pool_mutex().lock();
        const bool log_refusal = stats_.on_request(min_size, c);
        if (c < kPoolClassCount) {
            stats_.on_served(c);
        }

        const uint8_t  home = stats_.best_fit(min_size);
        const uint32_t refused = home < kPoolClassCount ? stats_[home].refused : 0u;
        buf_pool_mutex().unlock();
        if (log_refusal) {
            RTPS_LOG_W("bufpool", "no block for %lu octets (class %u, refusal %lu, caller 0x%lx)",
                       static_cast<unsigned long>(min_size), static_cast<unsigned>(home),
                       static_cast<unsigned long>(refused), static_cast<unsigned long>(caller_pc));
        }
    }
#endif

    void init() noexcept {
        c0_.init(0); c1_.init(1); c2_.init(2);
        c3_.init(3); c4_.init(4);
    }

#if RTPS_HEAP_BLOCKS
    /**
     * @brief Allocates a sample larger than every static class from the heap.
     *
     * The block has the same header as a slab block, so reference counting,
     * sharing, and prepend() work unchanged. Each side of the budget gets half
     * of RTPS_HEAP_BLOCK_BUDGET, and the block's size class records its side so
     * release returns the octets to the half that was charged.
     *
     * @param[in] min_size Payload octets; above kPoolMaxBlockSize.
     * @param[in] side     The half of the budget to charge.
     * @return A block of exactly `min_size` octets, or an invalid BufRef when
     *   the size or that half is exceeded or the heap is exhausted.
     *
     * Verification:
     * - Correct means: a refused request changes no accounting; every block
     *   returns its octets to the budget when the last reference is released.
     * - Covered by: tests/host/buf_pool_heap_block_test.cpp.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.3 (DataFrag)
     */
    BufRef alloc_heap(uint32_t min_size, HeapBudgetSide side) noexcept {
        if (min_size > RTPS_MAX_HEAP_BLOCK_SIZE) {
            return {};
        }

        std::atomic<size_t>& octets = heap_octets_[static_cast<uint8_t>(side)];
        const size_t         used = octets.fetch_add(min_size, std::memory_order_relaxed);
        if (used + min_size > RTPS_HEAP_BLOCK_BUDGET / 2u) {
            octets.fetch_sub(min_size, std::memory_order_relaxed);
            return {};
        }

        void* memory = std::malloc(sizeof(BufBlock) + min_size);
        if (memory == nullptr) {
            octets.fetch_sub(min_size, std::memory_order_relaxed);
            return {};
        }

        BufBlock* b = new (memory) BufBlock;
        b->refcount = 1;
        b->next_free = nullptr;
        b->block_cap = min_size;
        b->size_class = side == HeapBudgetSide::Received ? kHeapReceivedSizeClass : kHeapSizeClass;

        BufRef r{};
        r.block = b;
        r.data = b->payload();
        r.size = 0;
        return r;
    }

    /// @brief Returns a heap block's octets to the budget and frees it.
    void free_heap(BufBlock* b) noexcept {
        const HeapBudgetSide side = b->size_class == kHeapReceivedSizeClass
                                        ? HeapBudgetSide::Received
                                        : HeapBudgetSide::Local;
        heap_octets_[static_cast<uint8_t>(side)].fetch_sub(b->block_cap, std::memory_order_relaxed);
        b->~BufBlock();
        std::free(b);
    }

    std::atomic<size_t> heap_octets_[2]{{0u}, {0u}};
#endif

    BufBlock* dispatch_alloc(uint8_t c) {
        switch (c) {
            case 0: return c0_.alloc();
            case 1: return c1_.alloc();
            case 2: return c2_.alloc();
            case 3: return c3_.alloc();
            case 4: return c4_.alloc();
            default: return nullptr;
        }
    }

    void dispatch_free(BufBlock* b) noexcept {
        switch (b->size_class) {
            case 0: c0_.free_block(b); break;
            case 1: c1_.free_block(b); break;
            case 2: c2_.free_block(b); break;
            case 3: c3_.free_block(b); break;
            case 4: c4_.free_block(b); break;
            default: break;  // Should never happen.
        }
    }

    uint8_t dispatch_available(uint8_t c) const noexcept {
        switch (c) {
            case 0: return c0_.available();
            case 1: return c1_.available();
            case 2: return c2_.available();
            case 3: return c3_.available();
            case 4: return c4_.available();
            default:
#if defined(RTPS_BUFPOOL_DEBUG) && RTPS_BUFPOOL_DEBUG_EVENT_CAP > 0
                //debug_recent_event(5, {});
#endif
                return 0;
        }
    }

#ifdef RTPS_BUFPOOL_DEBUG
    void debug_on_alloc(BufBlock* b, uint32_t min_size, uintptr_t caller_pc) noexcept {
        if (!b || b->size_class >= kPoolClassCount) return;
        buf_pool_mutex().lock();
        BufPoolDebugClassStats& s = dbg_stats_[b->size_class];
        ++s.alloc_ok;
        ++s.live;
        if (s.live > s.high_water) s.high_water = s.live;
        b->dbg_alloc_pc = caller_pc;
        b->dbg_last_touch_pc = caller_pc;
        b->dbg_alloc_seq = dbg_event_seq_ + 1u;
        debug_record_event(BufPoolDebugOp::Alloc, b, min_size, 0, 1, caller_pc);
        buf_pool_mutex().unlock();
    }

    void debug_on_alloc_fail(uint32_t min_size, uintptr_t caller_pc) noexcept {
        buf_pool_mutex().lock();
        for (uint8_t c = 0; c < kPoolClassCount; ++c) {
            if (class_sizes_[c] >= min_size) ++dbg_stats_[c].alloc_fail;
        }
        debug_record_event(BufPoolDebugOp::AllocFail, nullptr, min_size, 0, 0, caller_pc);
        buf_pool_mutex().unlock();
    }

    void debug_on_release(BufBlock* b, int prev) noexcept {
        if (!b || b->size_class >= kPoolClassCount) return;
        buf_pool_mutex().lock();
        BufPoolDebugClassStats& s = dbg_stats_[b->size_class];
        ++s.release_calls;
        const int after = prev - 1;
        b->dbg_last_touch_pc = RTPS_BUFPOOL_CALLER_PC();
        debug_record_event(BufPoolDebugOp::Release, b, b->block_cap, prev, after, b->dbg_last_touch_pc);
        if (prev == 1) {
            if (s.live > 0) --s.live;
            ++s.frees;
            debug_record_event(BufPoolDebugOp::Free, b, b->block_cap, 1, 0, b->dbg_last_touch_pc);
        }
        buf_pool_mutex().unlock();
    }

    void debug_record_event(BufPoolDebugOp op,
                            const BufBlock* b,
                            uint32_t requested_size,
                            int ref_before,
                            int ref_after,
                            uintptr_t caller_pc) noexcept
    {
        const uint32_t ring_cap = static_cast<uint32_t>(RTPS_BUFPOOL_DEBUG_EVENT_CAP);
        if (ring_cap == 0u) return;
        const uint32_t seq = ++dbg_event_seq_;
        const uint32_t slot = (seq - 1u) % ring_cap;
        BufPoolDebugEvent& e = dbg_events_[slot];
        e.tick_ms = rosette::platform::monotonic_ms();
        e.seq = seq;
        e.block = static_cast<uintptr_t>(reinterpret_cast<uintptr_t>(b));
        e.caller_pc = caller_pc;
        e.requested_size = requested_size;
        e.block_cap = b ? b->block_cap : 0u;
        e.size_class = b ? b->size_class : 0xFFu;
        e.op = static_cast<uint8_t>(op);
        e.ref_before = static_cast<int16_t>(ref_before);
        e.ref_after = static_cast<int16_t>(ref_after);
    }
#endif
};

// ============================================================
// Global pool singleton — defined in buf_pool.cpp
// ============================================================
extern MultiBufPool g_buf_pool;

// ============================================================
// BufRef method bodies
// ============================================================

inline void BufRef::retain() const noexcept {
    [[maybe_unused]] const int prev = MultiBufPool::retain(block);
#ifdef RTPS_BUFPOOL_DEBUG
    g_buf_pool.debug_mark_retain(block, prev, RTPS_BUFPOOL_CALLER_PC());
#endif
}

inline void BufRef::release() noexcept {
    g_buf_pool.release(block);
    block = nullptr;
    data  = nullptr;
    size  = 0;
    // capacity() is computed from block->block_cap; no field to clear.
}

// ============================================================
// buf_alloc() — plain allocation, data == block->payload().
//
// @param min_size  Minimum required payload capacity in bytes.
//                  Defaults to the smallest class (128 bytes).
//                  All call sites that do not pass a size get the
//                  most economical block and retain backward compat.
// ============================================================

inline BufRef buf_alloc(uint32_t min_size = RTPS_POOL_CLASS0_SIZE) noexcept {
    return g_buf_pool.alloc(min_size, RTPS_BUFPOOL_CALLER_PC());
}

/// @brief buf_alloc() for a sample whose size a remote peer announced; see
/// MultiBufPool::alloc_received().
inline BufRef buf_alloc_received(uint32_t min_size) noexcept {
    return g_buf_pool.alloc_received(min_size);
}

// ============================================================
// buf_alloc_with_headroom() — allocate with a prepend reserve.
//
// Allocates a block large enough for (headroom + min_payload_size)
// bytes and advances `data` by headroom bytes, reserving that prefix
// for later BufRef::prepend() calls.
//
// Zero-copy DATA TX example
// -------------------------
//   // kRtpsDataHeadroom = sizeof(RTPSMessageHeader)       (20 B)
//   //                   + sizeof(DataSubmessageHeader)    (24 B)
//   //                   = 44 bytes
//   BufRef buf = buf_alloc_with_headroom(payload_size, kRtpsDataHeadroom);
//
//   // Write CDR payload directly — no scratch buffer needed.
//   uint32_t n = my_serialiser.write(buf.data, buf.capacity());
//   buf.size = n;
//
//   // Prepend DATA submessage header in-place (zero copy):
//   auto* sm = reinterpret_cast<DataSubmessageHeader*>(
//                  buf.prepend(sizeof(DataSubmessageHeader)));
//   populate_data_header(sm, reader_id, writer_id, sn);
//
//   // Prepend RTPS message header in-place (zero copy):
//   auto* mh = reinterpret_cast<RTPSMessageHeader*>(
//                  buf.prepend(sizeof(RTPSMessageHeader)));
//   mh->init(my_guid_prefix, my_vendor_id);
//
//   // buf now describes [RTPS hdr | DATA hdr | CDR] — zero copies.
//   transport.send_unicast(buf, dest);
//
// @param min_payload_size  Minimum bytes needed for CDR payload.
// @param headroom          Bytes to reserve before `data` for prepend.
// @returns BufRef with data offset by headroom, or invalid on exhaustion.
// ============================================================

inline BufRef buf_alloc_with_headroom(uint32_t min_payload_size,
                                      uint32_t headroom) noexcept
{
    BufRef r = g_buf_pool.alloc(min_payload_size + headroom,
                                RTPS_BUFPOOL_CALLER_PC());
    if (!r.valid()) return r;
    // Advance data pointer past the headroom reservation.
    // capacity() will now correctly return (block_cap - headroom).
    r.data += headroom;
    return r;
}

#ifdef RTPS_BUFPOOL_DEBUG
inline BufRef buf_alloc_dbg(uint32_t min_size,
                            uintptr_t caller_pc) noexcept {
    return g_buf_pool.alloc(min_size, caller_pc);
}

inline BufRef buf_alloc_with_headroom_dbg(uint32_t min_payload_size,
                                          uint32_t headroom,
                                          uintptr_t caller_pc) noexcept {
    BufRef r = g_buf_pool.alloc(min_payload_size + headroom, caller_pc);
    if (!r.valid()) return r;
    r.data += headroom;
    return r;
}

#  define RTPS_BUF_ALLOC_DBG(min_size) \
      ::rosette::rtps::buf_alloc_dbg((min_size), RTPS_BUFPOOL_CALLER_PC())

#  define RTPS_BUF_ALLOC_WITH_HEADROOM_DBG(min_payload_size, headroom)             \
      ::rosette::rtps::buf_alloc_with_headroom_dbg((min_payload_size), (headroom), \
                                                   RTPS_BUFPOOL_CALLER_PC())
#endif

// ============================================================
// Standard headroom constants
//
// kRtpsDataHeadroom reserves space for the RTPS message header
// (§9.4.2, 20 bytes) plus a DATA submessage header (§9.4.5.3,
// 24 bytes) = 44 bytes total.  Use this with buf_alloc_with_headroom()
// for zero-copy DATA transmission.
//
// kRtpsCtrlHeadroom is narrower — just the RTPS message header plus
// a fixed submessage header — sufficient for HEARTBEAT, ACKNACK, GAP.
//
// kRtpsHeartbeatTailroom is the space after a payload for one
// HEARTBEAT submessage, claimed with BufRef::append().
// ============================================================

/// RTPS message header (20) + DATA submessage header (24) = 44 bytes.
inline constexpr uint32_t kRtpsDataHeadroom = 44u;

/// RTPS message header (20) + generic submessage header (4) = 24 bytes.
inline constexpr uint32_t kRtpsCtrlHeadroom = 24u;

/// HEARTBEAT submessage header (4) + body (28) = 32 bytes, reserved after a
/// reliable writer's payload so a piggybacked HEARTBEAT (§8.4.15.4) shares
/// the DATA's datagram. See BufRef::append().
inline constexpr uint32_t kRtpsHeartbeatTailroom = 32u;

// ============================================================
// Sizing helpers
// ============================================================

/// Control submessage (HB, ACKNACK, GAP) — class 0 (128 B) always fits.
inline constexpr uint32_t kCtrlBufMinSize = RTPS_POOL_CLASS0_SIZE;

/// SPDP/SEDP PL_CDR payload — class 1 (256 B).
inline constexpr uint32_t kDiscoveryBufMinSize = RTPS_POOL_CLASS1_SIZE;

/// Default TX scratch buffer — class 2 (512 B).
inline constexpr uint32_t kTxScratchMinSize = RTPS_POOL_CLASS2_SIZE;

/// Fragment reassembly — class 4 (2048 B).
inline constexpr uint32_t kFragBufMinSize = RTPS_POOL_CLASS4_SIZE;

}  // namespace rosette::rtps
