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
 *  - Free-list protected by a FreeRTOS critical section (portENTER/EXIT
 *    _CRITICAL) — safe from tasks and from ISRs on Cortex-M.
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
 * 3. If not, try each larger class in order (promote-on-exhaustion).
 *    Promotion is explicit; the caller can inspect BufRef::capacity()
 *    to know what it actually received.
 * 4. If all eligible classes are exhausted, return an invalid BufRef.
 *
 * Default size-class configuration (override in config.hpp)
 * ----------------------------------------------------------
 *  Class 0:  128 bytes,  8 slots  — HEARTBEAT, ACKNACK, GAP, small ctrl
 *  Class 1:  256 bytes,  6 slots  — SPDP/SEDP PL_CDR, small DATA
 *  Class 2:  512 bytes,  4 slots  — medium DATA, default TX scratch
 *  Class 3: 1024 bytes,  2 slots  — large DATA
 *  Class 4: 2048 bytes,  2 slots  — DATA_FRAG reassembly
 *
 * Total managed payload: 8×128 + 6×256 + 4×512 + 2×1024 + 2×2048
 *                       = 1024 + 1536 + 2048 + 2048 + 4096 = 10 752 bytes
 *
 * MessageBuilder compatibility
 * ----------------------------
 * MessageBuilder::write_bytes() previously compared against the compile-
 * time constant RTPS_POOL_BLOCK_SIZE.  It now calls buf_.capacity() so
 * that the correct runtime limit is used regardless of which size class
 * the builder's buffer came from.
 */

#pragma once

#include "log.hpp"
#include "types.hpp"
#include <atomic>
#include <cstdint>
#include <string.h>

#ifndef portENTER_CRITICAL
#  define portENTER_CRITICAL()  do {} while (0)
#  define portEXIT_CRITICAL()   do {} while (0)
#endif

namespace rtps {

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
    uint16_t     block_cap;     ///< Usable bytes following this header.
    uint8_t      size_class;    ///< Index into MultiBufPool::classes_[].
    uint8_t      _pad = 0;
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

    /// Pop one block.  Returns nullptr on exhaustion.  ISR-safe.
    BufBlock* alloc() noexcept {
        BufBlock* b = nullptr;
        portENTER_CRITICAL();
        if (free_head_) {
            b            = free_head_;
            free_head_   = b->next_free;
            b->next_free = nullptr;
        }
        portEXIT_CRITICAL();
        if (b) {
            b->refcount = 1;
        }
        return b;
    }

    /// Push a block back onto the free list.  ISR-safe.
    void free_block(BufBlock* b) noexcept {
        std::atomic_thread_fence(std::memory_order_acquire);
        portENTER_CRITICAL();
        b->next_free = free_head_;
        free_head_   = b;
        portEXIT_CRITICAL();
    }

    /// Snapshot of free-block count (diagnostic only; not locked during walk).
    uint8_t available() const noexcept {
        BufBlock* head;
        portENTER_CRITICAL();
        head = free_head_;
        portEXIT_CRITICAL();
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

    // ----------------------------------------------------------
    // alloc(min_size)
    //
    // Returns a BufRef from the smallest size class whose block
    // capacity >= min_size.  If the ideal class is exhausted, the
    // next larger class is tried (promote-on-exhaustion).
    //
    // BufRef::capacity() reflects the actual block size received,
    // which may be larger than min_size.
    //
    // Returns an invalid BufRef if all eligible classes are full.
    //
    // @param min_size  Minimum usable bytes required.  Defaults to
    //                  the smallest class size so callers that don't
    //                  care about size get the most economical block.
    // ----------------------------------------------------------
    BufRef alloc(uint32_t min_size = RTPS_POOL_CLASS0_SIZE,
                 uintptr_t caller_pc = 0u) noexcept {
        for (uint8_t c = 0; c < kPoolClassCount; ++c) {
            if (class_sizes_[c] < min_size) continue;  // Class too small.
            BufBlock* b = dispatch_alloc(c);
            if (!b) continue;  // Class exhausted — promote to next.
#ifdef RTPS_BUFPOOL_DEBUG
            debug_on_alloc(b, min_size, caller_pc);
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

    ClassInfo class_info(uint8_t c) const noexcept {
        if (c >= kPoolClassCount) return {};
        ClassInfo ci{};
        ci.block_size = class_sizes_[c];
        ci.total      = class_counts_[c];
        ci.available  = dispatch_available(c);
        return ci;
    }

    uint8_t class_count() const noexcept { return kPoolClassCount; }

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
        portENTER_CRITICAL();
        BufPoolDebugClassStats& s = dbg_stats_[b->size_class];
        ++s.retain_calls;
        b->dbg_last_touch_pc = caller_pc;
        debug_record_event(BufPoolDebugOp::Retain, b, b->block_cap, prev, prev + 1, caller_pc);
        portEXIT_CRITICAL();
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

    void init() noexcept {
        c0_.init(0); c1_.init(1); c2_.init(2);
        c3_.init(3); c4_.init(4);
    }

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
#if defined(RTPS_BUFPOOL_DEBUG) && RTPS_BUFPOOL_DEBUG_EVENT_CAP > 0
                //debug_recent_event(5, {});
#endif
                return 0;
        }
    }

#ifdef RTPS_BUFPOOL_DEBUG
    void debug_on_alloc(BufBlock* b, uint32_t min_size, uintptr_t caller_pc) noexcept {
        if (!b || b->size_class >= kPoolClassCount) return;
        portENTER_CRITICAL();
        BufPoolDebugClassStats& s = dbg_stats_[b->size_class];
        ++s.alloc_ok;
        ++s.live;
        if (s.live > s.high_water) s.high_water = s.live;
        b->dbg_alloc_pc = caller_pc;
        b->dbg_last_touch_pc = caller_pc;
        b->dbg_alloc_seq = dbg_event_seq_ + 1u;
        debug_record_event(BufPoolDebugOp::Alloc, b, min_size, 0, 1, caller_pc);
        portEXIT_CRITICAL();
    }

    void debug_on_alloc_fail(uint32_t min_size, uintptr_t caller_pc) noexcept {
        portENTER_CRITICAL();
        for (uint8_t c = 0; c < kPoolClassCount; ++c) {
            if (class_sizes_[c] >= min_size) ++dbg_stats_[c].alloc_fail;
        }
        debug_record_event(BufPoolDebugOp::AllocFail, nullptr, min_size, 0, 0, caller_pc);
        portEXIT_CRITICAL();
    }

    void debug_on_release(BufBlock* b, int prev) noexcept {
        if (!b || b->size_class >= kPoolClassCount) return;
        portENTER_CRITICAL();
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
        portEXIT_CRITICAL();
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
        e.tick_ms = RTPS_GET_MS();
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
    const int prev = MultiBufPool::retain(block);
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

#define RTPS_BUF_ALLOC_DBG(min_size) \
    ::rtps::buf_alloc_dbg((min_size), RTPS_BUFPOOL_CALLER_PC())

#define RTPS_BUF_ALLOC_WITH_HEADROOM_DBG(min_payload_size, headroom) \
    ::rtps::buf_alloc_with_headroom_dbg((min_payload_size), (headroom), \
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
// ============================================================

/// RTPS message header (20) + DATA submessage header (24) = 44 bytes.
inline constexpr uint32_t kRtpsDataHeadroom = 44u;

/// RTPS message header (20) + generic submessage header (4) = 24 bytes.
inline constexpr uint32_t kRtpsCtrlHeadroom = 24u;

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

} // namespace rtps
