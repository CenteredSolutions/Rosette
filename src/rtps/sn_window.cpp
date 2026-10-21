/**
 * @file rtps/sn_window.cpp
 * @brief SequenceNumberWindow: a writer's resolved sequence numbers in constant space.
 */

#include "sn_window.hpp"

namespace rosette::rtps {

bool SequenceNumberWindow::is_resolved(const SequenceNumber& sn) const noexcept {
    if (sn < base_) return true;
    if (!covers(sn)) return false;
    return (resolved_ & (uint64_t{1} << (sn - base_))) != 0u;
}

bool SequenceNumberWindow::resolve(const SequenceNumber& sn) noexcept {
    if (!covers(sn)) return false;
    const uint64_t bit = uint64_t{1} << (sn - base_);
    if ((resolved_ & bit) != 0u) return false;
    resolved_ |= bit;
    advance_base();
    return true;
}

void SequenceNumberWindow::resolve_below(const SequenceNumber& first_unresolved) noexcept {
    if (!(base_ < first_unresolved)) return;
    const int64_t shift = first_unresolved - base_;
    resolved_ = shift >= static_cast<int64_t>(kCapacity) ? 0u : resolved_ >> shift;
    base_ = first_unresolved;
    advance_base();
}

void SequenceNumberWindow::resolve_range(const SequenceNumber& first,
                                         const SequenceNumber& end) noexcept {
    if (!(base_ < first)) {
        resolve_below(end);
        return;
    }
    // The run starts above base_, so base_ stays put and only numbers inside
    // the window can be recorded.
    for (SequenceNumber sn = first; sn < end && covers(sn); sn += 1) {
        (void)resolve(sn);
    }
}

void SequenceNumberWindow::unresolved(const SequenceNumber& last, uint32_t max_count,
                                      SequenceNumberSet& out) const noexcept {
    out.clear();
    out.bitmapBase = base_;
    uint32_t listed = 0u;
    for (uint32_t i = 0; i < kCapacity && listed < max_count; ++i) {
        const SequenceNumber sn = base_ + static_cast<int64_t>(i);
        if (last < sn) break;
        if ((resolved_ & (uint64_t{1} << i)) == 0u) {
            out.set(i);
            ++listed;
        }
    }
}

void SequenceNumberWindow::advance_base() noexcept {
    while ((resolved_ & 1u) != 0u) {
        resolved_ >>= 1u;
        base_ += 1;
    }
}

}  // namespace rosette::rtps
