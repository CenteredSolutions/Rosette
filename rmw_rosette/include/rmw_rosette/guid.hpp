/**
 * @file rmw_rosette/guid.hpp
 * @brief rtps::Guid <-> rmw_gid_t conversion.
 *
 * rtps::Guid is exactly 16 bytes (12-octet GuidPrefix + 4-octet EntityId —
 * see rtps/types.hpp), which fits inside rmw_gid_t's 24-byte storage
 * (RMW_GID_STORAGE_SIZE) with room to spare. The remaining bytes are
 * zero-padded; only `implementation_identifier` distinguishes gids across
 * rmw vendors, not the padding.
 */

#pragma once

#include "identifier.h"
#include "rosette/rtps/types.hpp"

#include <rmw/types.h>

#include <cstring>

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
inline void guid_to_rmw_gid(const rtps::Guid& guid, rmw_gid_t& out) noexcept {
    out.implementation_identifier = rmw_rosette_identifier;
    std::memset(out.data, 0, RMW_GID_STORAGE_SIZE);
    static_assert(sizeof(rtps::GuidPrefix) + sizeof(rtps::EntityId) <= RMW_GID_STORAGE_SIZE,
                  "rtps::Guid no longer fits in rmw_gid_t storage");
    std::memcpy(out.data, guid.prefix.value, sizeof(guid.prefix.value));
    std::memcpy(out.data + sizeof(guid.prefix.value), guid.entityId.entityKey,
                sizeof(guid.entityId.entityKey));
    out.data[sizeof(guid.prefix.value) + sizeof(guid.entityId.entityKey)] =
        guid.entityId.entityKind;
}

}  // namespace rmw_rosette
