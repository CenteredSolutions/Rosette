/**
 * @file introspection_c_codec.cpp
 * @brief XCDR1 encode/decode of C ROS messages driven by introspection_c metadata.
 *
 * The codec is a recursive walk over `MessageMembers`. Each member is either a
 * single value, a fixed-size array, or a (bounded or unbounded) sequence of
 * values; each value is a primitive, a string, a wide string, or a nested
 * message. Primitive arrays and sequences take a block-copy fast path because
 * the C mapping stores them contiguously; everything else goes element by
 * element through the member's accessor functions, so the walk never assumes
 * the layout of a generated `__Sequence` struct beyond what introspection
 * exposes.
 */

#include "rmw_rosette/introspection_c_codec.hpp"

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/u16string_functions.h>
#include <rosidl_typesupport_introspection_c/field_types.h>

#include <cstdint>

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#  error "rmw_rosette's CDR codec emits CDR_LE by copying host memory; a big-endian host needs byte swapping."
#endif

namespace rmw_rosette {
namespace {

using Member = rosidl_typesupport_introspection_c__MessageMember;

/**
 * @brief Returns the in-memory and wire size of a fixed-width primitive.
 *
 * @param[in] type_id `rosidl_typesupport_introspection_c_field_types` value.
 * @return Size in bytes, or 0 for types that are not block-copyable
 *   (string, wstring, message, wchar whose wire width differs from its C
 *   width, bool which needs normalization on decode, and long double).
 */
size_t block_copy_size(uint8_t type_id) noexcept {
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

/// @brief Returns the nested message table of a MESSAGE-typed member.
const IntrospectionMembers* nested_members(const Member& m) noexcept {
    return m.members_ ? static_cast<const IntrospectionMembers*>(m.members_->data) : nullptr;
}

/**
 * @brief Encodes one non-array value of the member's type.
 *
 * @param[in]     m     Member descriptor supplying the type and bounds.
 * @param[in]     value Pointer to the single value in C memory.
 * @param[in,out] out   Destination stream.
 * @return `false` for an unsupported type or a nested failure.
 */
bool encode_value(const Member& m, const void* value, CdrWriter& out) noexcept {
    const size_t block = block_copy_size(m.type_id_);
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
            out.put<uint32_t>(*static_cast<const uint16_t*>(value));
            return true;
        case rosidl_typesupport_introspection_c__ROS_TYPE_STRING: {
            const auto* s = static_cast<const rosidl_runtime_c__String*>(value);
            const size_t len = s->data ? s->size : 0;
            out.put<uint32_t>(static_cast<uint32_t>(len + 1));
            out.put_bytes(s->data, len);
            out.put<uint8_t>(0);
            return true;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING: {
            const auto* s = static_cast<const rosidl_runtime_c__U16String*>(value);
            const size_t len = s->data ? s->size : 0;
            out.put<uint32_t>(static_cast<uint32_t>(len));
            for (size_t i = 0; i < len; ++i) out.put<uint32_t>(s->data[i]);
            return true;
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
            const IntrospectionMembers* sub = nested_members(m);
            return sub && encode_introspection_c(*sub, value, out);
        }
        default:
            return false;  // long double: no interoperable CDR mapping in ROS 2
    }
}

/**
 * @brief Decodes one non-array value of the member's type into C memory.
 *
 * @param[in]     m     Member descriptor supplying the type and bounds.
 * @param[in,out] in    Source stream.
 * @param[out]    value Pointer to the single initialized value.
 * @return `false` on truncation, a bound violation, allocation failure, or an
 *   unsupported type.
 */
bool decode_value(const Member& m, CdrReader& in, void* value) noexcept {
    const size_t block = block_copy_size(m.type_id_);
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
            *static_cast<uint16_t*>(value) = static_cast<uint16_t>(in.get<uint32_t>());
            return in.ok();
        case rosidl_typesupport_introspection_c__ROS_TYPE_STRING: {
            const uint32_t n = in.get<uint32_t>();
            const uint8_t* bytes = in.take_view(n);
            if (!in.ok()) return false;
            // n counts the NUL terminator; tolerate a zero-length encoding.
            size_t len = n;
            if (len > 0 && bytes[len - 1] == '\0') --len;
            if (m.string_upper_bound_ && len > m.string_upper_bound_) return false;
            return rosidl_runtime_c__String__assignn(
                static_cast<rosidl_runtime_c__String*>(value),
                reinterpret_cast<const char*>(bytes), len);
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING: {
            const uint32_t n = in.get<uint32_t>();
            if (!in.ok() || n > in.remaining() / 4) return false;
            if (m.string_upper_bound_ && n > m.string_upper_bound_) return false;
            auto* s = static_cast<rosidl_runtime_c__U16String*>(value);
            if (!rosidl_runtime_c__U16String__resize(s, n)) return false;
            for (uint32_t i = 0; i < n; ++i) s->data[i] = static_cast<uint16_t>(in.get<uint32_t>());
            return in.ok();
        }
        case rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE: {
            const IntrospectionMembers* sub = nested_members(m);
            return sub && decode_introspection_c(*sub, in, value);
        }
        default:
            return false;
    }
}

/**
 * @brief Encodes a fixed array or sequence member.
 *
 * @param[in]     m     Array member descriptor.
 * @param[in]     field Pointer to the member inside the message.
 * @param[in,out] out   Destination stream.
 * @return `false` when an element fails to encode or accessors are missing.
 */
bool encode_array(const Member& m, const void* field, CdrWriter& out) noexcept {
    if (!m.size_function || !m.get_const_function) return false;
    const size_t count = m.size_function(field);
    const bool is_sequence = m.array_size_ == 0 || m.is_upper_bound_;
    if (is_sequence) out.put<uint32_t>(static_cast<uint32_t>(count));
    if (count == 0) return out.ok();

    const size_t block = block_copy_size(m.type_id_);
    if (block) {
        out.align(block);
        out.put_bytes(m.get_const_function(field, 0), block * count);
        return out.ok();
    }
    for (size_t i = 0; i < count; ++i) {
        if (!encode_value(m, m.get_const_function(field, i), out)) return false;
    }
    return out.ok();
}

/**
 * @brief Decodes a fixed array or sequence member, resizing sequences.
 *
 * A sequence length is rejected before resizing when it exceeds the declared
 * bound or could not possibly fit in the remaining input (every element
 * occupies at least one byte on the wire), which caps allocation by the
 * payload size.
 *
 * @param[in]     m     Array member descriptor.
 * @param[in,out] in    Source stream.
 * @param[out]    field Pointer to the member inside the message.
 * @return `false` on truncation, a bound violation, or allocation failure.
 */
bool decode_array(const Member& m, CdrReader& in, void* field) noexcept {
    if (!m.get_function) return false;
    const bool is_sequence = m.array_size_ == 0 || m.is_upper_bound_;
    size_t count = m.array_size_;
    if (is_sequence) {
        count = in.get<uint32_t>();
        if (!in.ok() || count > in.remaining()) return false;
        if (m.is_upper_bound_ && count > m.array_size_) return false;
        if (!m.resize_function || !m.resize_function(field, count)) return false;
    }
    if (count == 0) return in.ok();

    const size_t block = block_copy_size(m.type_id_);
    if (block) {
        in.align(block);
        if (count > in.remaining() / block) return false;
        in.get_bytes(m.get_function(field, 0), block * count);
        return in.ok();
    }
    for (size_t i = 0; i < count; ++i) {
        if (!decode_value(m, in, m.get_function(field, i))) return false;
    }
    return in.ok();
}

}  // namespace

bool encode_introspection_c(const IntrospectionMembers& members, const void* ros_message,
                            CdrWriter& out) noexcept {
    const auto* base = static_cast<const uint8_t*>(ros_message);
    for (uint32_t i = 0; i < members.member_count_; ++i) {
        const Member& m = members.members_[i];
        const void* field = base + m.offset_;
        const bool ok = m.is_array_ ? encode_array(m, field, out) : encode_value(m, field, out);
        if (!ok || !out.ok()) return false;
    }
    return true;
}

bool decode_introspection_c(const IntrospectionMembers& members, CdrReader& in,
                            void* ros_message) noexcept {
    auto* base = static_cast<uint8_t*>(ros_message);
    for (uint32_t i = 0; i < members.member_count_; ++i) {
        const Member& m = members.members_[i];
        void* field = base + m.offset_;
        const bool ok = m.is_array_ ? decode_array(m, in, field) : decode_value(m, in, field);
        if (!ok || !in.ok()) return false;
    }
    return true;
}

}  // namespace rmw_rosette
