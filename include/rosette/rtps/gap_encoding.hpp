/**
 * @file rtps/gap_encoding.hpp
 * @brief Encoding and decoding the set of sequence numbers a GAP declares irrelevant.
 *
 * A GAP submessage names the changes a reliable writer will never send to a
 * reader (DDSI-RTPS 2.5 §8.3.7.4): a contiguous run [gapStart,
 * gapList.bitmapBase) plus every sequence number whose bit is set in
 * gapList. An empty bitmap is common and meaningful: Fast DDS declares
 * changes 1-10 irrelevant as gapStart = 1, gapList = {base 11, no bits}.
 *
 * A writer uses encode_gap() to turn the set of changes it will not send
 * (changes written before a VOLATILE reader matched, changes evicted from a
 * KEEP_LAST history) into that form. Readers use gap_run_end() to find the
 * end of the run and walk the run by tracked state rather than by sequence
 * number, since a run may span thousands of changes.
 */

#pragma once

#include "types.hpp"

namespace rosette::rtps {

/**
 * @brief Returns one past the last sequence number of a GAP's contiguous run.
 *
 * @param[in] gap_start The GAP's gapStart.
 * @param[in] gap_list  The GAP's gapList.
 * @return gapList.bitmapBase, so the run is [gap_start, returned value).
 *   When bitmapBase does not exceed gapStart the run is empty and
 *   gap_start is returned.
 *
 * Verification:
 * - Correct means: a GAP {1, base 11, no bits} yields a run covering
 *   1 through 10; {5, base 5, no bits} yields an empty run.
 * - Covered by: TC-SRR-8.4.9-004, TC-SEDP-8.5.4-012.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.3.7.4.5 (GAP interpretation), §9.4.5.5
 */
inline SequenceNumber gap_run_end(const SequenceNumber&    gap_start,
                                  const SequenceNumberSet& gap_list) noexcept {
    return gap_start < gap_list.bitmapBase ? gap_list.bitmapBase : gap_start;
}

/**
 * @brief Encodes a set of irrelevant sequence numbers as a GAP's gapStart and gapList.
 *
 * The lowest irrelevant sequence number starts the contiguous run, the run
 * extends while the next sequence number is also irrelevant, and the rest
 * go in the gapList bitmap relative to the first number after the run.
 * This matches the layout Fast DDS emits (RTPSGapBuilder), so every peer
 * decodes it the same way.
 *
 * @param[in]  irrelevant Sequence numbers the writer will not send, as
 *   bits relative to irrelevant.bitmapBase.
 * @param[out] gap_start  First irrelevant sequence number.
 * @param[out] gap_list   bitmapBase one past the contiguous run, with a bit
 *   for each further irrelevant sequence number.
 * @return `false` when `irrelevant` is empty (send no GAP), `true` otherwise.
 *
 * Verification:
 * - Correct means: decoding the result with gap_run_end() plus the bitmap
 *   yields exactly the input set.
 * - Covered by: TC-SWR-8.4.9-003.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.3.7.4.5, §9.4.5.5
 */
inline bool encode_gap(const SequenceNumberSet& irrelevant,
                       SequenceNumber&          gap_start,
                       SequenceNumberSet&       gap_list) noexcept {
    uint32_t first = 0u;
    while (first < irrelevant.numBits && !irrelevant.test(first)) ++first;
    if (first >= irrelevant.numBits) return false;
    uint32_t run_end = first + 1u;
    while (run_end < irrelevant.numBits && irrelevant.test(run_end)) ++run_end;

    gap_start = irrelevant.bitmapBase + static_cast<int64_t>(first);
    gap_list.clear();
    gap_list.bitmapBase = irrelevant.bitmapBase + static_cast<int64_t>(run_end);
    for (uint32_t i = run_end; i < irrelevant.numBits; ++i) {
        if (irrelevant.test(i)) gap_list.set(i - run_end);
    }
    return true;
}

}  // namespace rosette::rtps
