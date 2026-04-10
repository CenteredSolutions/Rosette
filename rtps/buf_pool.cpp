/**
 * @file rtps/buf_pool.cpp
 * @brief Global MultiBufPool singleton definition.
 *
 * The pool lives in .bss and is zero-initialised by the C runtime before
 * the MultiBufPool constructor runs.  The constructor chains init() calls
 * on each SlabClass which build the per-class free lists.
 *
 * On RP2350 this object occupies approximately:
 *   8 slots  × (sizeof(BufBlock) + 128)  =  8 × 136  = 1 088 bytes
 *   6 slots  × (sizeof(BufBlock) + 256)  =  6 × 264  = 1 584 bytes
 *   4 slots  × (sizeof(BufBlock) + 512)  =  4 × 520  = 2 080 bytes
 *   2 slots  × (sizeof(BufBlock) + 1024) =  2 × 1032 = 2 064 bytes
 *   2 slots  × (sizeof(BufBlock) + 2048) =  2 × 2056 = 4 112 bytes
 *   ─────────────────────────────────────────────────────────────
 *   Total                                              ≈ 10 928 bytes
 *
 * (sizeof(BufBlock) = 4 + 4 + 2 + 1 + 1 = 12 bytes, padded to 16 by
 *  StorageSlot alignment on a 4-byte-minimum-aligned Octet array.)
 *
 * Place this translation unit in SRAM (not flash) by your linker script.
 */

#include "buf_pool.hpp"

namespace rtps {

MultiBufPool g_buf_pool;

} // namespace rtps
