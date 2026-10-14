/**
 * @file rtps/string_arena.hpp
 * @brief A fixed-capacity arena of interned NUL-terminated strings.
 *
 * The endpoint match table keeps the topic and type names of a participant's
 * own endpoints here, the one place they live: matching compares them, graph
 * queries count by them, and a rebuilding SEDP writer serializes every
 * announcement from them (EndpointMatchTable::local_writer_data()). ROS 2
 * names run to about 20-60 octets, while a name may be as long as
 * RTPS_MAX_TOPIC_NAME_LEN - 1, so a fixed field per name would reserve most
 * of its octets unused. StringArena instead packs the strings end to end,
 * and an owner keeps only each string's 16-bit offset.
 *
 * Each distinct string is stored once. intern() returns the offset of an
 * equal string already in the arena, so endpoints sharing a type (every
 * action server's GoalStatusArray, CancelGoal request and reply) or a topic
 * (a writer and a reader of the same topic) share one copy. The arena keeps
 * no reference counts: its owner holds every offset, and releases a string
 * when it drops the last one.
 *
 * Releasing a string closes the gap at once by moving the strings after it
 * down, so the freed octets return to the free space at the end and the
 * arena never fragments: any set of strings whose total length fits is
 * always storable. The owner lowers every offset above the released one by
 * the length release() returns. The owner's strings are created and deleted
 * rarely (at session start and end), so the move and intern()'s search cost
 * nothing on any path that runs per message.
 *
 * References: OMG DDSI-RTPS 2.5 §8.5.4.2 (topicName, typeName of
 * DiscoveredWriterData / DiscoveredReaderData).
 */

#pragma once

#include <stdint.h>
#include <string.h>

namespace rtps {

/**
 * @brief An arena of interned NUL-terminated strings addressed by 16-bit offsets.
 *
 * @tparam Capacity Octets the arena holds (RTPS_LOCAL_NAME_BYTES in the match table).
 *
 * Example:
 * @code{.cpp}
 * StringArena<256> names;
 * const uint16_t topic = names.intern("rt/chatter");
 * const uint16_t type = names.intern("std_msgs::msg::dds_::String_");
 * if (topic != StringArena<256>::kNone && type != StringArena<256>::kNone) {
 *     printf("%s is %s\n", names.str(topic), names.str(type));
 *     const uint16_t gone = names.release(topic);  // offsets above `topic` drop by `gone`
 * }
 * @endcode
 * Full program: tests/spec/src/test_sedp_8_5_5_1_local_names.cpp.
 */
template <uint32_t Capacity>
class StringArena {
public:
    static_assert(Capacity >= 2u && Capacity < 0xFFFFu,
                  "the arena must hold one string and fit a 16-bit offset below kNone");

    /// The offset of no string.
    static constexpr uint16_t kNone = 0xFFFFu;

    /// Octets the arena holds.
    static constexpr uint32_t kCapacity = Capacity;

    /// @brief Octets @p s takes in the arena, its terminator included.
    static uint32_t size_of(const char* s) noexcept {
        return static_cast<uint32_t>(strlen(s) + 1u);
    }

    /**
     * @brief The offset of the stored string equal to @p s.
     *
     * @param[in] s NUL-terminated string to look for.
     * @return Its offset, or kNone when no stored string equals it.
     *
     * Verification:
     * - Correct means: a stored string is found at the offset intern()
     *   returned for it, and a string that is only a prefix or a suffix of a
     *   stored one is not found.
     * - Covered by: TC-SEDP-8.5.5.1-014.
     */
    uint16_t find(const char* s) const noexcept {
        uint32_t at = 0u;
        while (at < used_) {
            if (strcmp(strings_ + at, s) == 0) {
                return static_cast<uint16_t>(at);
            }

            at += size_of(strings_ + at);
        }

        return kNone;
    }

    /// @brief Octets intern(@p s) would take: none when @p s is stored already.
    uint32_t cost(const char* s) const noexcept { return find(s) == kNone ? size_of(s) : 0u; }

    /**
     * @brief The offset of a string equal to @p s, storing it when there is none.
     *
     * @param[in] s NUL-terminated string.
     * @return The offset of the one copy of @p s, valid until that string or
     *   one below it is released; kNone when @p s is not stored and does not
     *   fit (nothing is written).
     *
     * Verification:
     * - Correct means: str() at the returned offset reads @p s back exactly,
     *   interning an equal string again returns the same offset with used()
     *   unchanged, and a new string grows used() by size_of().
     * - Covered by: TC-SEDP-8.5.5.1-014.
     */
    uint16_t intern(const char* s) noexcept {
        const uint16_t stored = find(s);
        if (stored != kNone) {
            return stored;
        }

        const uint32_t size = size_of(s);
        if (size > free_bytes()) {
            return kNone;
        }

        const uint16_t at = static_cast<uint16_t>(used_);
        memcpy(strings_ + at, s, size);
        used_ += size;
        return at;
    }

    /**
     * @brief Removes the string at @p at and moves the strings after it down over it.
     *
     * @param[in] at An offset intern() returned, which no owner uses any more.
     * @return The string's length with its terminator: every offset above
     *   @p at is now that much lower, and the owner lowers the offsets it
     *   keeps to match.
     *
     * Verification:
     * - Correct means: every other string reads back unchanged at its
     *   lowered offset, used() shrinks by the string's size, and the vacated
     *   octets read as empty strings.
     * - Covered by: TC-SEDP-8.5.5.1-014.
     */
    uint16_t release(uint16_t at) noexcept {
        const uint32_t size = size_of(strings_ + at);
        memmove(strings_ + at, strings_ + at + size, used_ - at - size);
        used_ -= size;
        // Vacated octets read as empty strings, so an offset that was not
        // lowered reads nothing rather than the string that used to be there.
        memset(strings_ + used_, 0, size);
        return static_cast<uint16_t>(size);
    }

    /// @brief The string at @p at.
    const char* str(uint16_t at) const noexcept { return strings_ + at; }

    /// @brief Octets the stored strings take.
    uint32_t used() const noexcept { return used_; }

    /// @brief Octets still free.
    uint32_t free_bytes() const noexcept { return kCapacity - used_; }

private:
    char     strings_[kCapacity] = {};
    uint32_t used_ = 0u;
};

}  // namespace rtps
