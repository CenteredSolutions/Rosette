// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file introspection_cdr_walk.hpp
 * @brief XCDR1 encode/decode walk shared by the C and C++ introspection codecs.
 *
 * The walk is a recursion over a `MessageMembers` table. Each member is a
 * single value, a fixed-size array, or a (bounded or unbounded) sequence of
 * values; each value is a primitive, a string, a wide string, or a nested
 * message. Primitive arrays and sequences take a block-copy path because both
 * language mappings store them contiguously; everything else goes element by
 * element through the member's accessor functions, so the walk assumes nothing
 * about a sequence's layout beyond what introspection exposes.
 *
 * The two introspection families publish structurally identical tables with
 * the same field type numbers and differ only in how a string is stored, how a
 * sequence is resized, and in C++'s packed `std::vector<bool>`. A Traits type
 * supplies those differences at compile time, so a firmware build instantiates
 * the C walk alone and carries no C++ container code.
 *
 * A Traits type provides:
 * - `Members`, `Member`: the family's table types.
 * - `WideChar`: the in-memory type of a `wchar` value.
 * - `kHasPackedBoolSequences`: whether a bool sequence lacks element pointers.
 * - `encode_string`, `encode_wstring`: write one string value.
 * - `assign_string`: store `len` decoded bytes in one string value.
 * - `decode_wstring`: read `count` wide characters into one string value.
 * - `resize`: resize a sequence member, reporting success.
 */

#pragma once

#include <rosidl_typesupport_introspection_c/field_types.h>

#include <cstddef>
#include <cstdint>

#include "rmw_rosette/cdr_stream.hpp"

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#  error \
      "rmw_rosette's CDR codec emits CDR_LE by copying host memory; a big-endian host needs byte swapping."
#endif

namespace rmw_rosette {

/**
 * @brief Returns the in-memory and wire size of a fixed-width primitive.
 *
 * @param[in] type_id Field type number, shared by the C and C++ families.
 * @return Size in bytes, or 0 for types that are not block-copyable: string,
 *   wstring, message, wchar whose wire width differs from its memory width,
 *   bool which needs normalization on decode, and long double.
 */
inline size_t introspection_block_copy_size(uint8_t type_id) noexcept {
    switch (type_id) {
        case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
        case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
            return 1;
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
            return 2;
        case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
            return 4;
        case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
        case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
        case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
            return 8;
        default:
            return 0;
    }
}

/**
 * @brief The recursive XCDR1 walk for one introspection family.
 *
 * Wire rules follow rosidl_typesupport_fastrtps, so payloads are
 * byte-identical to rmw_fastrtps_cpp's whichever family produced them.
 * Functions are not `noexcept` because C++ sequence and string storage can
 * throw; the C++ codec's entry points catch at the outermost level.
 *
 * Verification:
 * - Correct means: the C and C++ families produce identical bytes for the same
 *   message, equal to rmw_fastrtps_cpp's, and decoding restores it.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp,
 *   rmw_rosette/test/test_introspection_cpp_codec.cpp.
 *
 * References:
 * - OMG CDR, CORBA 3.4 Part 2 §9.3.1
 * - OMG DDS-XTypes 1.3 §7.4.3.4
 *
 * @tparam Traits Family description; see the file comment.
 */
template <typename Traits>
struct IntrospectionCdrWalk {
    using Members = typename Traits::Members;
    using Member = typename Traits::Member;

    /// @brief Returns the nested message table of a MESSAGE-typed member.
    static const Members* nested_members(const Member& m) noexcept {
        return m.members_ ? static_cast<const Members*>(m.members_->data) : nullptr;
    }

    /// @brief Reports whether an array member is a variable-length sequence.
    static bool is_sequence(const Member& m) noexcept {
        return m.array_size_ == 0 || m.is_upper_bound_;
    }

    /**
     * @brief Encodes one non-array value of the member's type.
     *
     * @param[in]     m     Member descriptor supplying the type.
     * @param[in]     value Pointer to the single value in message memory.
     * @param[in,out] out   Destination stream.
     * @return `false` for an unsupported type or a nested failure.
     */
    static bool encode_value(const Member& m, const void* value, CdrWriter& out) {
        const size_t block = introspection_block_copy_size(m.type_id_);
        if (block) {
            out.align(block);
            out.put_bytes(value, block);
            return true;
        }

        switch (m.type_id_) {
            case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
                out.put<uint8_t>(*static_cast<const bool*>(value) ? 1 : 0);
                return true;
            case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
                out.put<uint32_t>(*static_cast<const typename Traits::WideChar*>(value));
                return true;
            case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
                Traits::encode_string(value, out);
                return true;
            case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
                Traits::encode_wstring(value, out);
                return true;
            case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
                const Members* sub = nested_members(m);
                return sub && encode(*sub, value, out);
            }
            default:
                return false;  // long double: no interoperable CDR mapping in ROS 2
        }
    }

    /**
     * @brief Decodes one string value, enforcing the member's bound.
     *
     * The wire length counts the NUL terminator; a zero-length encoding is
     * tolerated. The bytes stay in the input until the bound has been checked.
     *
     * @param[in]     m     Member descriptor supplying the bound.
     * @param[in,out] in    Source stream.
     * @param[out]    value Pointer to the initialized string.
     * @return `false` on truncation, a bound violation, or allocation failure.
     */
    static bool decode_string(const Member& m, CdrReader& in, void* value) {
        const uint32_t n = in.get<uint32_t>();
        const uint8_t* bytes = in.take_view(n);
        if (!in.ok()) {
            return false;
        }

        size_t len = n;
        if (len > 0 && bytes[len - 1] == '\0') {
            --len;
        }

        if (m.string_upper_bound_ && len > m.string_upper_bound_) {
            return false;
        }

        return Traits::assign_string(value, reinterpret_cast<const char*>(bytes), len);
    }

    /**
     * @brief Decodes one wide string value, enforcing the member's bound.
     *
     * @param[in]     m     Member descriptor supplying the bound.
     * @param[in,out] in    Source stream.
     * @param[out]    value Pointer to the initialized wide string.
     * @return `false` on truncation, a bound violation, or allocation failure.
     */
    static bool decode_wstring(const Member& m, CdrReader& in, void* value) {
        const uint32_t n = in.get<uint32_t>();
        if (!in.ok() || n > in.remaining() / 4) {
            return false;
        }

        if (m.string_upper_bound_ && n > m.string_upper_bound_) {
            return false;
        }

        return Traits::decode_wstring(value, in, n);
    }

    /**
     * @brief Decodes one non-array value of the member's type.
     *
     * @param[in]     m     Member descriptor supplying the type and bounds.
     * @param[in,out] in    Source stream.
     * @param[out]    value Pointer to the single initialized value.
     * @return `false` on truncation, a bound violation, allocation failure, or
     *   an unsupported type.
     */
    static bool decode_value(const Member& m, CdrReader& in, void* value) {
        const size_t block = introspection_block_copy_size(m.type_id_);
        if (block) {
            in.align(block);
            in.get_bytes(value, block);
            return in.ok();
        }

        switch (m.type_id_) {
            case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
                *static_cast<bool*>(value) = in.get<uint8_t>() != 0;
                return in.ok();
            case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
                *static_cast<typename Traits::WideChar*>(value) =
                    static_cast<typename Traits::WideChar>(in.get<uint32_t>());
                return in.ok();
            case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
                return decode_string(m, in, value);
            case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
                return decode_wstring(m, in, value);
            case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
                const Members* sub = nested_members(m);
                return sub && decode(*sub, in, value);
            }
            default:
                return false;
        }
    }

    /**
     * @brief Encodes the elements of a packed bool sequence one at a time.
     *
     * `std::vector<bool>` has no addressable elements, so its member offers
     * `fetch_function` in place of the pointer accessors.
     *
     * @param[in]     m     Sequence member descriptor.
     * @param[in]     field Pointer to the member inside the message.
     * @param[in]     count Element count.
     * @param[in,out] out   Destination stream.
     * @return `false` when the accessor is missing or the stream overflowed.
     */
    static bool encode_packed_bools(const Member& m, const void* field, size_t count,
                                    CdrWriter& out) {
        if (!m.fetch_function) {
            return false;
        }

        for (size_t i = 0; i < count && out.ok(); ++i) {
            bool element = false;
            m.fetch_function(field, i, &element);
            out.put<uint8_t>(element ? 1 : 0);
        }

        return out.ok();
    }

    /**
     * @brief Decodes the elements of a packed bool sequence one at a time.
     *
     * @param[in]     m     Sequence member descriptor.
     * @param[in,out] in    Source stream.
     * @param[out]    field Pointer to the resized member inside the message.
     * @param[in]     count Element count.
     * @return `false` when the accessor is missing or the input is truncated.
     */
    static bool decode_packed_bools(const Member& m, CdrReader& in, void* field, size_t count) {
        if (!m.assign_function) {
            return false;
        }

        for (size_t i = 0; i < count; ++i) {
            const bool element = in.get<uint8_t>() != 0;
            m.assign_function(field, i, &element);
        }

        return in.ok();
    }

    /**
     * @brief Encodes a fixed array or sequence member.
     *
     * @param[in]     m     Array member descriptor.
     * @param[in]     field Pointer to the member inside the message.
     * @param[in,out] out   Destination stream.
     * @return `false` when an element fails to encode or accessors are missing.
     */
    static bool encode_array(const Member& m, const void* field, CdrWriter& out) {
        if (!m.size_function) {
            return false;
        }

        const size_t count = m.size_function(field);
        if (is_sequence(m)) {
            out.put<uint32_t>(static_cast<uint32_t>(count));
        }

        if constexpr (Traits::kHasPackedBoolSequences) {
            if (!m.get_const_function) {
                return encode_packed_bools(m, field, count, out);
            }
        }

        if (!m.get_const_function) {
            return false;
        }

        if (count == 0) {
            return out.ok();
        }

        const size_t block = introspection_block_copy_size(m.type_id_);
        if (block) {
            out.align(block);
            out.put_bytes(m.get_const_function(field, 0), block * count);
            return out.ok();
        }

        for (size_t i = 0; i < count; ++i) {
            if (!encode_value(m, m.get_const_function(field, i), out)) {
                return false;
            }
        }

        return out.ok();
    }

    /**
     * @brief Returns the fewest CDR bytes one element of a member can occupy.
     *
     * decode_array() multiplies this by a received sequence length to refuse
     * a length the remaining payload cannot hold, before any storage exists.
     * Alignment padding is left out, so the result never exceeds the real
     * encoded size and a valid sample is never refused.
     *
     * @param[in] m Member descriptor; its array-ness is ignored.
     * @return The minimum encoded element size in bytes, at least 1.
     *
     * Verification:
     * - Correct means: no valid encoding of the element is shorter than the result.
     * - Covered by: IntrospectionCppCodec.RejectsASequenceOfNestedMessagesThePayloadCannotHold.
     *
     * References:
     * - OMG CDR, CORBA 3.4 Part 2 §9.3 (primitive, string, and sequence encodings)
     */
    static size_t min_element_wire_size(const Member& m) noexcept {
        const size_t block = introspection_block_copy_size(m.type_id_);
        if (block) {
            return block;
        }

        switch (m.type_id_) {
            case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
            case rosidl_typesupport_introspection_c__ROS_TYPE_STRING:
            case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING:
                return sizeof(uint32_t);
            case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
                const Members* sub = nested_members(m);
                return sub ? min_message_wire_size(*sub) : 1u;
            }
            default:
                return 1u;
        }
    }

    /**
     * @brief Returns the fewest CDR bytes a message of the given type can occupy.
     *
     * Fixed arrays count every element and sequences count only their length
     * prefix, since a sequence may be empty. The sum cannot overflow: it is a
     * small multiple of the in-memory size of a type that had to fit in the
     * address space to be compiled.
     *
     * @param[in] members Introspection table for the message type.
     * @return The minimum encoded size in bytes, at least 1.
     *
     * Verification:
     * - Correct means: no valid encoding of the message is shorter than the result.
     * - Covered by: IntrospectionCppCodec.RejectsASequenceOfNestedMessagesThePayloadCannotHold.
     *
     * References:
     * - OMG CDR, CORBA 3.4 Part 2 §9.3
     */
    static size_t min_message_wire_size(const Members& members) noexcept {
        size_t total = 0u;
        for (uint32_t i = 0; i < members.member_count_; ++i) {
            const Member& m = members.members_[i];
            if (!m.is_array_) {
                total += min_element_wire_size(m);
            } else if (is_sequence(m)) {
                total += sizeof(uint32_t);
            } else {
                total += m.array_size_ * min_element_wire_size(m);
            }
        }

        return total ? total : 1u;
    }

    /**
     * @brief Decodes a fixed array or sequence member, resizing sequences.
     *
     * A sequence length is rejected before resizing when it exceeds the
     * declared bound or when that many elements, each at its minimum encoded
     * size, could not fit in the remaining input. Elements with nested fixed
     * arrays therefore cannot request storage out of proportion to the
     * payload that names them.
     *
     * @param[in]     m     Array member descriptor.
     * @param[in,out] in    Source stream.
     * @param[out]    field Pointer to the member inside the message.
     * @return `false` on truncation, a bound violation, or allocation failure.
     */
    static bool decode_array(const Member& m, CdrReader& in, void* field) {
        size_t count = m.array_size_;
        if (is_sequence(m)) {
            count = in.get<uint32_t>();
            if (!in.ok() || count > in.remaining() / min_element_wire_size(m)) {
                return false;
            }

            if (m.is_upper_bound_ && count > m.array_size_) {
                return false;
            }

            if (!Traits::resize(m, field, count)) {
                return false;
            }
        }

        if constexpr (Traits::kHasPackedBoolSequences) {
            if (!m.get_function) {
                return decode_packed_bools(m, in, field, count);
            }
        }

        if (!m.get_function) {
            return false;
        }

        if (count == 0) {
            return in.ok();
        }

        const size_t block = introspection_block_copy_size(m.type_id_);
        if (block) {
            in.align(block);
            if (count > in.remaining() / block) {
                return false;
            }

            in.get_bytes(m.get_function(field, 0), block * count);
            return in.ok();
        }

        for (size_t i = 0; i < count; ++i) {
            if (!decode_value(m, in, m.get_function(field, i))) {
                return false;
            }
        }

        return in.ok();
    }

    /**
     * @brief Encodes every field of a message in declaration order.
     *
     * @param[in]     members     Introspection table for the message type.
     * @param[in]     ros_message Message instance laid out as `members` says.
     * @param[in,out] out         Destination stream.
     * @return `false` when `out` overflowed or a field has no CDR mapping.
     */
    static bool encode(const Members& members, const void* ros_message, CdrWriter& out) {
        const auto* base = static_cast<const uint8_t*>(ros_message);
        for (uint32_t i = 0; i < members.member_count_; ++i) {
            const Member& m = members.members_[i];
            const void*   field = base + m.offset_;
            const bool ok = m.is_array_ ? encode_array(m, field, out) : encode_value(m, field, out);
            if (!ok || !out.ok()) {
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Decodes every field of a message in declaration order.
     *
     * @param[in]     members     Introspection table for the message type.
     * @param[in,out] in          Source stream.
     * @param[out]    ros_message Initialized message instance to overwrite.
     * @return `false` on truncation, a bound violation, allocation failure, or
     *   an unsupported field type.
     */
    static bool decode(const Members& members, CdrReader& in, void* ros_message) {
        auto* base = static_cast<uint8_t*>(ros_message);
        for (uint32_t i = 0; i < members.member_count_; ++i) {
            const Member& m = members.members_[i];
            void*         field = base + m.offset_;
            const bool ok = m.is_array_ ? decode_array(m, in, field) : decode_value(m, in, field);
            if (!ok || !in.ok()) {
                return false;
            }
        }

        return true;
    }
};

}  // namespace rmw_rosette
