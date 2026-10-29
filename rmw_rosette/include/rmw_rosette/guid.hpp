/**
 * @file rmw_rosette/guid.hpp
 * @brief rosette::rtps::Guid <-> rmw_gid_t conversion.
 *
 * rosette::rtps::Guid is exactly 16 bytes (12-octet GuidPrefix + 4-octet EntityId —
 * see rtps/types.hpp). It fills rmw_gid_t's storage (RMW_GID_STORAGE_SIZE: 24
 * octets on Humble, 16 from Iron on) and any remaining bytes are zero padding;
 * only `implementation_identifier` distinguishes gids across rmw vendors, not
 * the padding.
 *
 * `rmw_request_id_t::writer_guid` holds the same 16 octets, typed `int8_t[16]`
 * on Humble and `uint8_t[RMW_GID_STORAGE_SIZE]` from Iron on. The request-writer
 * helpers take the field's own type (RequestWriterGuid), so each distribution's
 * header decides it.
 */

#pragma once

#include "identifier.h"
#include "rosette/rtps/types.hpp"

#include <rmw/types.h>

#include <cstring>
#include <type_traits>

namespace rmw_rosette {

/**
 * @brief Packs an RTPS GUID into the rmw-vendor-agnostic `rmw_gid_t` form.
 *
 * `rmw_get_gid_for_publisher()`/`rmw_get_gid_for_subscription()` and the
 * `publisher_gid` field of `rmw_message_info_t` all identify an RTPS
 * endpoint this way, so rcl/rclcpp code can compare origins across
 * publishers and subscriptions without knowing which rmw vendor produced
 * them.
 *
 * Zero-fills the full `RMW_GID_STORAGE_SIZE` buffer, then packs the
 * 12-octet `GuidPrefix` immediately followed by the 4-octet `EntityId`
 * (3-byte key + 1-byte kind) into the leading 16 bytes, leaving the
 * remaining storage as padding.
 *
 * @param[in] guid RTPS GUID of the writer or reader endpoint.
 * @param[out] out Destination gid; `implementation_identifier` is set to
 *   `rmw_rosette_identifier` and `data` is fully overwritten.
 */
inline void guid_to_rmw_gid(const rosette::rtps::Guid& guid, rmw_gid_t& out) noexcept {
    out.implementation_identifier = rmw_rosette_identifier;
    std::memset(out.data, 0, RMW_GID_STORAGE_SIZE);
    static_assert(
        sizeof(rosette::rtps::GuidPrefix) + sizeof(rosette::rtps::EntityId) <= RMW_GID_STORAGE_SIZE,
        "rosette::rtps::Guid no longer fits in rmw_gid_t storage");
    std::memcpy(out.data, guid.prefix.value, sizeof(guid.prefix.value));
    std::memcpy(out.data + sizeof(guid.prefix.value), guid.entityId.entityKey,
                sizeof(guid.entityId.entityKey));
    out.data[sizeof(guid.prefix.value) + sizeof(guid.entityId.entityKey)] =
        guid.entityId.entityKind;
}

/// `rmw_request_id_t::writer_guid`, as the rmw headers in use declare it.
using RequestWriterGuid = decltype(rmw_request_id_t::writer_guid);

/// Element type of RequestWriterGuid (`int8_t` on Humble, `uint8_t` from Iron on).
using RequestWriterOctet = std::remove_extent_t<RequestWriterGuid>;

static_assert(sizeof(RequestWriterGuid) >= 16,
              "rmw_request_id_t::writer_guid cannot hold an RTPS GUID");

/**
 * @brief Writes an RTPS GUID as the 16-octet form rmw_request_id_t uses.
 *
 * Service request IDs name the request's writer by GUID; stock RMWs store
 * the 12-octet prefix followed by the 4-octet EntityId, and so does this.
 * Octets past the first 16 (present when the field is RMW_GID_STORAGE_SIZE
 * long) are zeroed.
 *
 * @param[in]  guid RTPS GUID.
 * @param[out] out  Destination `rmw_request_id_t::writer_guid`; fully overwritten.
 *
 * Verification:
 * - Correct means: guid_from_request_writer() returns `guid` unchanged, and
 *   the first 16 octets equal what rmw_fastrtps_cpp writes for the same writer.
 * - Covered by: rmw_rosette/test/test_service_departed_client.cpp and
 *   test_service_reply_depth.cpp (a reply reaches the client that sent the request).
 *
 * References:
 * - rmw/types.h, `rmw_request_id_t`
 * - rmw_fastrtps_shared_cpp/guid_utils.hpp, `copy_from_fastrtps_guid_to_byte_array`
 */
inline void guid_to_request_writer(const rosette::rtps::Guid& guid,
                                   RequestWriterGuid&         out) noexcept {
    std::memset(out, 0, sizeof(out));
    std::memcpy(out, guid.prefix.value, 12);
    std::memcpy(out + 12, guid.entityId.entityKey, 3);
    out[15] = static_cast<RequestWriterOctet>(guid.entityId.entityKind);
}

/**
 * @brief Reads the 16-octet request-writer form back into an RTPS GUID.
 *
 * @param[in] in `rmw_request_id_t::writer_guid`.
 * @return The GUID written by guid_to_request_writer().
 */
inline rosette::rtps::Guid guid_from_request_writer(const RequestWriterGuid& in) noexcept {
    rosette::rtps::Guid g{};
    std::memcpy(g.prefix.value, in, 12);
    std::memcpy(g.entityId.entityKey, in + 12, 3);
    g.entityId.entityKind = static_cast<rosette::rtps::Octet>(in[15]);
    return g;
}

/**
 * @brief Converts an RTPS Timestamp (seconds + 2^-32 fractions) to nanoseconds.
 *
 * @param[in] t RTPS time, as carried by INFO_TS (§9.3.2, Time_t).
 * @return Nanoseconds since the Unix epoch, or 0 for TIME_ZERO.
 */
inline int64_t timestamp_to_ns(const rosette::rtps::Timestamp& t) noexcept {
    const int64_t frac_ns =
        static_cast<int64_t>((static_cast<uint64_t>(t.fraction) * 1000000000ull) >> 32);
    return static_cast<int64_t>(t.seconds) * 1000000000ll + frac_ns;
}

}  // namespace rmw_rosette
