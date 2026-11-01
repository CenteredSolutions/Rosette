// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/buf_pool.cpp
 * @brief Global MultiBufPool singleton definition.
 *
 * The pool lives in .bss and is zero-initialised by the C runtime before
 * the MultiBufPool constructor runs. The constructor chains init() calls
 * on each SlabClass, which build the per-class free lists.
 *
 * Its size is the sum over classes of COUNT x (SIZE + sizeof(BufBlock)),
 * where sizeof(BufBlock) is 12 octets on ARM32. The class sizes and counts
 * come from the build (buf_pool.hpp, "Size-class configuration").
 */

#include "buf_pool.hpp"

namespace rosette::rtps {

#ifdef ROSETTE_RTPS_MAX_DATAGRAM_SIZE
// Class 3 holds one datagram: a received DATA payload, a built message, or
// a sample small enough to go out unfragmented together with its RTPS
// headers (kRtpsDataHeadroom).
static_assert(RTPS_POOL_CLASS3_SIZE >= ROSETTE_RTPS_MAX_DATAGRAM_SIZE + kRtpsDataHeadroom,
              "RTPS_POOL_CLASS3_SIZE must hold one datagram plus its RTPS headers");
#endif

MultiBufPool g_buf_pool;

}  // namespace rosette::rtps
