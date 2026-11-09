// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file introspection_c_codec.cpp
 * @brief XCDR1 encode/decode of C ROS messages driven by introspection_c metadata.
 *
 * Supplies the C storage rules of `rosidl_runtime_c__String` and
 * `rosidl_runtime_c__U16String` to the walk in introspection_cdr_walk.hpp.
 */

#include "rmw_rosette/introspection_c_codec.hpp"

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/u16string_functions.h>

#include <cstdint>

#include "introspection_cdr_walk.hpp"

namespace rmw_rosette {
namespace {

/// @brief Storage rules of the C message mapping.
struct IntrospectionCTraits {
    using Members = IntrospectionMembers;
    using Member = rosidl_typesupport_introspection_c__MessageMember;
    using WideChar = uint16_t;

    static constexpr bool kHasPackedBoolSequences = false;

    /// @brief Returns the character count of a string, zero when it has no storage.
    static size_t string_length(const void* value) noexcept {
        const auto* s = static_cast<const rosidl_runtime_c__String*>(value);
        return s->data ? s->size : 0;
    }

    /// @brief Returns the code unit count of a wide string, zero when it has no storage.
    static size_t wstring_length(const void* value) noexcept {
        const auto* s = static_cast<const rosidl_runtime_c__U16String*>(value);
        return s->data ? s->size : 0;
    }

    /// @brief Writes a string as length including the NUL, bytes, NUL.
    static void encode_string(const void* value, CdrWriter& out) noexcept {
        const auto*  s = static_cast<const rosidl_runtime_c__String*>(value);
        const size_t len = s->data ? s->size : 0;
        out.put<uint32_t>(static_cast<uint32_t>(len + 1));
        out.put_bytes(s->data, len);
        out.put<uint8_t>(0);
    }

    /// @brief Writes a wide string as a count, then each unit widened to uint32.
    static void encode_wstring(const void* value, CdrWriter& out) noexcept {
        const auto*  s = static_cast<const rosidl_runtime_c__U16String*>(value);
        const size_t len = s->data ? s->size : 0;
        out.put<uint32_t>(static_cast<uint32_t>(len));
        for (size_t i = 0; i < len; ++i) {
            out.put<uint32_t>(s->data[i]);
        }
    }

    /// @brief Stores decoded bytes; `false` when allocation fails.
    static bool assign_string(void* value, const char* bytes, size_t len) noexcept {
        return rosidl_runtime_c__String__assignn(static_cast<rosidl_runtime_c__String*>(value),
                                                 bytes, len);
    }

    /// @brief Reads `count` wide characters; `false` on allocation failure or truncation.
    static bool decode_wstring(void* value, CdrReader& in, uint32_t count) noexcept {
        auto* s = static_cast<rosidl_runtime_c__U16String*>(value);
        if (!rosidl_runtime_c__U16String__resize(s, count)) {
            return false;
        }

        for (uint32_t i = 0; i < count; ++i) {
            s->data[i] = static_cast<uint16_t>(in.get<uint32_t>());
        }

        return in.ok();
    }

    /// @brief Resizes a sequence member; `false` when it has no resize or allocation fails.
    static bool resize(const Member& m, void* field, size_t count) noexcept {
        return m.resize_function && m.resize_function(field, count);
    }
};

using IntrospectionCWalk = IntrospectionCdrWalk<IntrospectionCTraits>;

}  // namespace

bool encode_introspection_c(const IntrospectionMembers& members, const void* ros_message,
                            CdrWriter& out) noexcept {
    return IntrospectionCWalk::encode(members, ros_message, out);
}

bool decode_introspection_c(const IntrospectionMembers& members, CdrReader& in,
                            void* ros_message) noexcept {
    return IntrospectionCWalk::decode(members, in, ros_message);
}

}  // namespace rmw_rosette
