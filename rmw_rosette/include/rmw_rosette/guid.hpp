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
