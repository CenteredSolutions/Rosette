/**
 * @file rtps/fragment_assembler.cpp
 * @brief Fragment reassembly in a shared slot pool — §8.4.14 (OMG formal/2022-04-01).
 */

#include "fragment_assembler.hpp"

namespace rtps {

namespace {

/// @brief True when stamp `a` was issued before `b`, across the 16-bit wrap.
bool stamped_before(uint16_t a, uint16_t b) noexcept {
    return static_cast<int16_t>(static_cast<uint16_t>(a - b)) < 0;
}

}  // namespace

uint16_t FragmentAssemblyPool::active_count() const noexcept {
    uint16_t n = 0;
    for (uint16_t i = 0; i < capacity_; ++i) {
        if (slots_[i].change.is_active()) {
            ++n;
        }
    }

    return n;
}

FragmentAssembler::~FragmentAssembler() noexcept {
    for (uint16_t i = 0; i < pool_.capacity_; ++i) {
        AssemblySlot& slot = pool_.slots_[i];
        if (slot.change.is_active() && slot.owner == this) {
            slot.change.clear();
        }
    }
}

bool FragmentAssembler::on_data_frag(const Guid& writer_guid, const SequenceNumber& writer_sn,
                                     uint32_t frag_start, uint16_t frags_in_submsg,
                                     uint16_t frag_size, uint32_t data_size, const Octet* payload,
                                     uint32_t payload_len, const Timestamp& source_ts,
                                     CacheChange& out_change,
                                     const InlineQos* inline_qos) noexcept {
    if (frag_start == 0 || frag_size == 0 || data_size == 0) {
        return false;
    }

    const uint32_t total_frags =
        (data_size + static_cast<uint32_t>(frag_size) - 1u) / static_cast<uint32_t>(frag_size);
    if (total_frags > static_cast<uint32_t>(FragmentedChange::kWords * 32)) {
        return false;
    }

    AssemblySlot* found = find_slot(writer_guid, writer_sn);
    FragmentedChange* slot = found != nullptr ? &found->change : nullptr;
    if (slot == nullptr) {
        slot = alloc_slot(writer_guid, writer_sn, frag_size, data_size, total_frags);
        if (slot == nullptr) {
            return false;
        }
    }

    for (uint16_t i = 0; i < frags_in_submsg; ++i) {
        const uint32_t frag_num = frag_start + static_cast<uint32_t>(i);
        if (frag_num > total_frags) {
            break;
        }

        // The last fragment may be shorter than frag_size (§9.4.5.4).
        const uint32_t frag_byte_offset = (frag_num - 1u) * static_cast<uint32_t>(frag_size);
        const uint32_t frag_byte_len = (frag_num == total_frags) ? (data_size - frag_byte_offset)
                                                                 : static_cast<uint32_t>(frag_size);
        const uint32_t src_off = i * static_cast<uint32_t>(frag_size);
        if (src_off + frag_byte_len > payload_len) {
            break;
        }

        // Only a fragment actually stored counts toward completion.
        if (!slot->write_fragment(frag_num, payload + src_off, frag_byte_len)) {
            break;
        }

        if (inline_qos != nullptr && !inline_qos->empty() && slot->inlineQos.empty()) {
            slot->inlineQos = *inline_qos;
        }

        slot->mark_received(frag_num);
    }

    if (!slot->is_complete()) {
        return false;
    }

    out_change = slot->take_as_cache_change(source_ts);
    return true;
}

bool FragmentAssembler::missing_fragments(const Guid& writer_guid, const SequenceNumber& writer_sn,
                                          uint32_t last_known,
                                          FragmentNumberSet& out) const noexcept {
    const AssemblySlot* slot = find_slot(writer_guid, writer_sn);
    if (slot == nullptr) {
        return false;
    }

    slot->change.missing_fragments(last_known, out);
    return true;
}

void FragmentAssembler::discard(const Guid& writer_guid, const SequenceNumber& writer_sn) noexcept {
    AssemblySlot* slot = find_slot(writer_guid, writer_sn);
    if (slot != nullptr) {
        slot->change.clear();
    }
}

void FragmentAssembler::discard_range(const Guid& writer_guid, const SequenceNumber& first,
                                      const SequenceNumber& end) noexcept {
    for (uint16_t i = 0; i < pool_.capacity_; ++i) {
        FragmentedChange& change = pool_.slots_[i].change;
        if (change.is_active() && pool_.slots_[i].owner == this && change.writerGuid == writer_guid
            && !(change.sequenceNumber < first) && change.sequenceNumber < end) {
            change.clear();
        }
    }
}

void FragmentAssembler::discard_writer(const Guid& writer_guid) noexcept {
    for (uint16_t i = 0; i < pool_.capacity_; ++i) {
        FragmentedChange& change = pool_.slots_[i].change;
        if (change.is_active() && pool_.slots_[i].owner == this
            && change.writerGuid == writer_guid) {
            change.clear();
        }
    }
}

uint16_t FragmentAssembler::active_count() const noexcept {
    uint16_t n = 0;
    for (uint16_t i = 0; i < pool_.capacity_; ++i) {
        if (pool_.slots_[i].change.is_active() && pool_.slots_[i].owner == this) {
            ++n;
        }
    }

    return n;
}

AssemblySlot* FragmentAssembler::find_slot(const Guid& wg,
                                           const SequenceNumber& sn) const noexcept {
    for (uint16_t i = 0; i < pool_.capacity_; ++i) {
        AssemblySlot& slot = pool_.slots_[i];
        if (slot.change.is_active() && slot.owner == this && slot.change.writerGuid == wg
            && slot.change.sequenceNumber == sn) {
            return &slot;
        }
    }

    return nullptr;
}

AssemblySlot* FragmentAssembler::claim_slot(const Guid& wg) noexcept {
    AssemblySlot* free_slot = nullptr;
    AssemblySlot* same_writer = nullptr;
    AssemblySlot* oldest = nullptr;
    uint16_t held = 0;
    for (uint16_t i = 0; i < pool_.capacity_; ++i) {
        AssemblySlot& slot = pool_.slots_[i];
        if (!slot.change.is_active()) {
            if (free_slot == nullptr) {
                free_slot = &slot;
            }

            continue;
        }

        if (slot.owner != this) {
            continue;
        }

        ++held;
        if (same_writer == nullptr && slot.change.writerGuid == wg) {
            same_writer = &slot;
        }

        if (oldest == nullptr || stamped_before(slot.stamp, oldest->stamp)) {
            oldest = &slot;
        }
    }

    if (free_slot != nullptr && held < limit_) {
        return free_slot;
    }

    // KEEP_LAST(1) per writer (§8.3.3.1): a newer sample from the same
    // writer replaces the partial one; otherwise this reader's oldest goes.
    AssemblySlot* victim = same_writer != nullptr ? same_writer : oldest;
    if (victim == nullptr) {
        RTPS_LOG_D("frag", "assembly pool full (%u slots), fragmented sample refused",
                   (unsigned)pool_.capacity_);
        return nullptr;
    }

    victim->change.clear();
    return victim;
}

FragmentedChange* FragmentAssembler::alloc_slot(const Guid& wg, const SequenceNumber& sn,
                                                uint16_t frag_size, uint32_t data_size,
                                                uint32_t total_frags) noexcept {
    AssemblySlot* slot = claim_slot(wg);
    if (slot == nullptr) {
        return nullptr;
    }

    // A sample larger than the largest pool class cannot be reassembled and
    // is refused here rather than delivered short.
    BufRef buf = buf_alloc(data_size);
    if (!buf.valid() || buf.capacity() < data_size) {
        RTPS_LOG_W("frag", "no pool block for a %lu-octet fragmented sample",
                   (unsigned long)data_size);
        return nullptr;
    }

    FragmentedChange& change = slot->change;
    change.writerGuid = wg;
    change.sequenceNumber = sn;
    change.fragmentSize = frag_size;
    change.dataSize = data_size;
    change.totalFragments = total_frags;
    change.inlineQos = {};
    change.buffer = static_cast<BufRef&&>(buf);
    memset(change.receivedBitmap, 0, sizeof(change.receivedBitmap));
    slot->owner = this;
    slot->stamp = pool_.next_stamp_++;
    return &change;
}

}  // namespace rtps
