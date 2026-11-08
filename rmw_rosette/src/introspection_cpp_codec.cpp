// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file introspection_cpp_codec.cpp
 * @brief XCDR1 encode/decode of C++ ROS messages driven by introspection_cpp metadata.
 *
 * Supplies the storage rules of `std::string`, `std::u16string`, and generated
 * C++ sequences to the walk in introspection_cdr_walk.hpp. Compiled only into
 * hosted builds; see introspection_cpp_codec.hpp.
 */

#include "rmw_rosette/introspection_cpp_codec.hpp"

#include <cstdint>
#include <string>

#include "introspection_cdr_walk.hpp"

namespace rmw_rosette {
namespace {

/// @brief Storage rules of the C++ message mapping.
struct IntrospectionCppTraits {
    using Members = IntrospectionCppMembers;
    using Member = rosidl_typesupport_introspection_cpp::MessageMember;
    using WideChar = char16_t;

    static constexpr bool kHasPackedBoolSequences = true;

    /// @brief Writes a string as length including the NUL, bytes, NUL.
    static void encode_string(const void* value, CdrWriter& out) {
        const auto& s = *static_cast<const std::string*>(value);
        out.put<uint32_t>(static_cast<uint32_t>(s.size() + 1));
        out.put_bytes(s.data(), s.size());
        out.put<uint8_t>(0);
    }

    /// @brief Writes a wide string as a count, then each unit widened to uint32.
    static void encode_wstring(const void* value, CdrWriter& out) {
        const auto& s = *static_cast<const std::u16string*>(value);
        out.put<uint32_t>(static_cast<uint32_t>(s.size()));
        for (const char16_t unit : s) {
            out.put<uint32_t>(unit);
        }
    }

    /// @brief Stores decoded bytes in a `std::string`.
    static bool assign_string(void* value, const char* bytes, size_t len) {
        auto& s = *static_cast<std::string*>(value);
        if (len == 0) {
            s.clear();
        } else {
            s.assign(bytes, len);
        }

        return true;
    }

    /// @brief Reads `count` wide characters into a `std::u16string`.
    static bool decode_wstring(void* value, CdrReader& in, uint32_t count) {
        auto& s = *static_cast<std::u16string*>(value);
        s.resize(count);
        for (char16_t& unit : s) {
            unit = static_cast<char16_t>(in.get<uint32_t>());
        }

        return in.ok();
    }

    /// @brief Resizes a sequence member; `false` when it has no resize function.
    static bool resize(const Member& m, void* field, size_t count) {
        if (!m.resize_function) {
            return false;
        }

        m.resize_function(field, count);
        return true;
    }
};

using IntrospectionCppWalk = IntrospectionCdrWalk<IntrospectionCppTraits>;

}  // namespace

bool encode_introspection_cpp(const IntrospectionCppMembers& members, const void* ros_message,
                              CdrWriter& out) noexcept {
    return IntrospectionCppWalk::encode(members, ros_message, out);
}

bool decode_introspection_cpp(const IntrospectionCppMembers& members, CdrReader& in,
                              void* ros_message) noexcept {
    try {
        return IntrospectionCppWalk::decode(members, in, ros_message);
    } catch (...) {
        return false;
    }
}

}  // namespace rmw_rosette
