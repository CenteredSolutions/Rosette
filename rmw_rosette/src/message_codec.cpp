// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file message_codec.cpp
 * @brief Typesupport-family resolution and dispatch for message serialization.
 *
 * See message_codec.hpp for the families and framing rules. The microxrcedds
 * branch is compiled only when CMake found rosidl_typesupport_microxrcedds_c,
 * which keeps desktop builds free of any micro-ROS dependency. The C++
 * introspection branch is compiled only into hosted builds
 * (RMW_ROSETTE_HAVE_INTROSPECTION_CPP_TYPESUPPORT), so firmware links neither
 * C++ message containers nor their heap use.
 */

#include "rmw_rosette/message_codec.hpp"

#include <rosidl_typesupport_introspection_c/identifier.h>
#include <rosidl_typesupport_introspection_c/service_introspection.h>

#include "rmw_rosette/introspection_c_codec.hpp"
#include "rmw_rosette/topic_naming.hpp"

namespace rtps = rosette::rtps;

#ifdef RMW_ROSETTE_HAVE_INTROSPECTION_CPP_TYPESUPPORT
#  include <rcutils/error_handling.h>
#  include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#  include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#  include "rmw_rosette/introspection_cpp_codec.hpp"
#endif

#ifdef RMW_ROSETTE_HAVE_MICROXRCEDDS_TYPESUPPORT
#  include <rosidl_typesupport_microxrcedds_c/identifier.h>
#  include <rosidl_typesupport_microxrcedds_c/message_type_support.h>
#  include <rosidl_typesupport_microxrcedds_c/service_type_support.h>
#endif

namespace rmw_rosette {
namespace {

/// CDR_LE representation identifier bytes, as they appear on the wire.
constexpr uint8_t kCdrLeId[2] = {0x00, 0x01};

const IntrospectionMembers* as_introspection(const MessageCodec& codec) noexcept {
    return static_cast<const IntrospectionMembers*>(codec.impl);
}

#ifdef RMW_ROSETTE_HAVE_INTROSPECTION_CPP_TYPESUPPORT
const IntrospectionCppMembers* as_introspection_cpp(const MessageCodec& codec) noexcept {
    return static_cast<const IntrospectionCppMembers*>(codec.impl);
}
#endif

#ifdef RMW_ROSETTE_HAVE_MICROXRCEDDS_TYPESUPPORT
const message_type_support_callbacks_t* as_microxrcedds(const MessageCodec& codec) noexcept {
    return static_cast<const message_type_support_callbacks_t*>(codec.impl);
}
#endif

}  // namespace

bool resolve_message_codec(const rosidl_message_type_support_t* handle,
                           MessageCodec& codec) noexcept {
    codec = MessageCodec{};
    if (!handle) return false;

#ifdef RMW_ROSETTE_HAVE_MICROXRCEDDS_TYPESUPPORT
    if (const auto* ts = get_message_typesupport_handle(
            handle, rosidl_typesupport_microxrcedds_c__identifier)) {
        codec.kind = CodecKind::MicroXrceDds;
        codec.impl = ts->data;
        return true;
    }
#endif

    if (const auto* ts = get_message_typesupport_handle(
            handle, rosidl_typesupport_introspection_c__identifier)) {
        codec.kind = CodecKind::IntrospectionC;
        codec.impl = ts->data;
        return true;
    }

#ifdef RMW_ROSETTE_HAVE_INTROSPECTION_CPP_TYPESUPPORT
    // A rosidl_typesupport_cpp handle can answer the C lookup by setting the rcutils
    // error state, which is expected here and must not leak to the caller.
    rcutils_reset_error();
    if (const auto* ts = get_message_typesupport_handle(
            handle, rosidl_typesupport_introspection_cpp::typesupport_identifier)) {
        codec.kind = CodecKind::IntrospectionCpp;
        codec.impl = ts->data;
        return true;
    }

    rcutils_reset_error();
#endif

    return false;
}

bool resolve_service_codecs(const rosidl_service_type_support_t* handle,
                            MessageCodec& request, MessageCodec& response) noexcept {
    request = MessageCodec{};
    response = MessageCodec{};
    if (!handle) return false;

#ifdef RMW_ROSETTE_HAVE_MICROXRCEDDS_TYPESUPPORT
    // microxrcedds describes a service as two message type supports.
    if (const auto* ts = get_service_typesupport_handle(
            handle, rosidl_typesupport_microxrcedds_c__identifier)) {
        const auto* cb = static_cast<const service_type_support_callbacks_t*>(ts->data);
        if (resolve_message_codec(cb->request_members_(), request) &&
            resolve_message_codec(cb->response_members_(), response)) {
            return true;
        }
        request = MessageCodec{};
        response = MessageCodec{};
    }
#endif

    if (const auto* ts = get_service_typesupport_handle(
            handle, rosidl_typesupport_introspection_c__identifier)) {
        const auto* members =
            static_cast<const rosidl_typesupport_introspection_c__ServiceMembers*>(ts->data);
        if (!members->request_members_ || !members->response_members_) return false;
        request.kind = CodecKind::IntrospectionC;
        request.impl = members->request_members_;
        response.kind = CodecKind::IntrospectionC;
        response.impl = members->response_members_;
        return true;
    }

#ifdef RMW_ROSETTE_HAVE_INTROSPECTION_CPP_TYPESUPPORT
    // See resolve_message_codec() for why the error state is cleared.
    rcutils_reset_error();
    if (const auto* ts = get_service_typesupport_handle(
            handle, rosidl_typesupport_introspection_cpp::typesupport_identifier)) {
        const auto* members =
            static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers*>(ts->data);
        if (!members->request_members_ || !members->response_members_) {
            return false;
        }

        request.kind = CodecKind::IntrospectionCpp;
        request.impl = members->request_members_;
        response.kind = CodecKind::IntrospectionCpp;
        response.impl = members->response_members_;
        return true;
    }

    rcutils_reset_error();
#endif

    return false;
}

bool compose_codec_type_name(const MessageCodec& codec, char* out, size_t cap) noexcept {
    switch (codec.kind) {
        case CodecKind::IntrospectionC:
            return compose_dds_type_name(out, cap, as_introspection(codec)->message_namespace_,
                                         as_introspection(codec)->message_name_);
#ifdef RMW_ROSETTE_HAVE_INTROSPECTION_CPP_TYPESUPPORT
        case CodecKind::IntrospectionCpp:
            return compose_dds_type_name(out, cap, as_introspection_cpp(codec)->message_namespace_,
                                         as_introspection_cpp(codec)->message_name_);
#endif
#ifdef RMW_ROSETTE_HAVE_MICROXRCEDDS_TYPESUPPORT
        case CodecKind::MicroXrceDds:
            return compose_dds_type_name(out, cap, as_microxrcedds(codec)->message_namespace_,
                                         as_microxrcedds(codec)->message_name_);
#endif
        default:
            if (cap) out[0] = '\0';
            return false;
    }
}

uint32_t serialize_message(const MessageCodec& codec, const void* ros_message,
                           uint8_t* out_buf, uint32_t out_cap) noexcept {
    switch (codec.kind) {
        case CodecKind::IntrospectionC: {
            CdrWriter out(out_buf, out_cap);
            if (!encode_introspection_c(*as_introspection(codec), ros_message, out)) return 0;
            return static_cast<uint32_t>(out.length());
        }
#ifdef RMW_ROSETTE_HAVE_INTROSPECTION_CPP_TYPESUPPORT
        case CodecKind::IntrospectionCpp: {
            CdrWriter out(out_buf, out_cap);
            if (!encode_introspection_cpp(*as_introspection_cpp(codec), ros_message, out)) {
                return 0;
            }

            return static_cast<uint32_t>(out.length());
        }
#endif
#ifdef RMW_ROSETTE_HAVE_MICROXRCEDDS_TYPESUPPORT
        case CodecKind::MicroXrceDds: {
            ucdrBuffer cdr;
            ucdr_init_buffer(&cdr, out_buf, out_cap);
            if (!as_microxrcedds(codec)->cdr_serialize(ros_message, &cdr)) return 0;
            return static_cast<uint32_t>(ucdr_buffer_length(&cdr));
        }
#endif
        default:
            return 0;
    }
}

rtps::BufRef serialize_to_pool(const MessageCodec& codec, const void* ros_message) noexcept {
    uint32_t request = 256u;
    for (;;) {
        rtps::BufRef buf = rtps::buf_alloc(request);
        if (!buf.valid()) return {};
        const uint32_t len = serialize_message(codec, ros_message, buf.data, buf.capacity());
        if (len > 0u) {
            buf.size = len;
            return buf;
        }
        // Did not fit (or failed to encode): retry one pool class up, until
        // the largest class has been tried.
        if (buf.capacity() >= rtps::kMaxSampleOctets) {
            return {};
        }

        // Inside the pool, one class up; on the heap, double, so a large
        // message is encoded a logarithmic number of times.
        const uint32_t capacity = buf.capacity();
        const uint32_t next = capacity < rtps::kPoolMaxBlockSize ? capacity + 1u : capacity * 2u;
        request = next < rtps::kMaxSampleOctets ? next : rtps::kMaxSampleOctets;
    }
}

bool deserialize_message(const MessageCodec& codec, const uint8_t* in_buf, uint32_t in_len,
                         void* ros_message) noexcept {
    if (in_len < kRtpsEncapsulationHeaderSize) return false;
    if (in_buf[0] != kCdrLeId[0] || in_buf[1] != kCdrLeId[1]) return false;
    const uint8_t* body = in_buf + kRtpsEncapsulationHeaderSize;
    const uint32_t body_len = in_len - kRtpsEncapsulationHeaderSize;

    switch (codec.kind) {
        case CodecKind::IntrospectionC: {
            CdrReader in(body, body_len);
            return decode_introspection_c(*as_introspection(codec), in, ros_message);
        }
#ifdef RMW_ROSETTE_HAVE_INTROSPECTION_CPP_TYPESUPPORT
        case CodecKind::IntrospectionCpp: {
            CdrReader in(body, body_len);
            return decode_introspection_cpp(*as_introspection_cpp(codec), in, ros_message);
        }
#endif
#ifdef RMW_ROSETTE_HAVE_MICROXRCEDDS_TYPESUPPORT
        case CodecKind::MicroXrceDds: {
            ucdrBuffer cdr;
            ucdr_init_buffer(&cdr, const_cast<uint8_t*>(body), body_len);
            return as_microxrcedds(codec)->cdr_deserialize(&cdr, ros_message);
        }
#endif
        default:
            return false;
    }
}

}  // namespace rmw_rosette
