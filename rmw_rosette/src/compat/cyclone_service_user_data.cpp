// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file cyclone_service_user_data.cpp
 * @brief Composes the `clientid=` and `serviceid=` endpoint USER_DATA of rmw_cyclonedds_cpp.
 */

#include "rmw_rosette/compat/cyclone_service_user_data.hpp"

#include <cstring>

namespace rmw_rosette {

static_assert(kCycloneServiceUserDataSize == 59u,
              "rmw_rosette_capacity.cmake sizes the default name arena with 59 per service");

namespace {

/// Appends @p octet in lowercase hex without leading zeros, as `std::hex` does.
uint32_t append_hex(uint8_t octet, char* out) noexcept {
    static constexpr char kDigits[] = "0123456789abcdef";
    uint32_t              n = 0;
    if (octet >= 0x10u) {
        out[n++] = kDigits[octet >> 4];
    }

    out[n++] = kDigits[octet & 0x0Fu];
    return n;
}

}  // namespace

void compose_cyclone_service_user_data(ServiceRole                      role,
                                       const rosette::rtps::GuidPrefix& participant,
                                       uint32_t                         counter,
                                       char (&out)[kCycloneServiceUserDataSize]) noexcept {
    uint8_t id[16];
    std::memcpy(id, participant.value, 12);
    for (uint32_t i = 0; i < 4; ++i) {
        id[12 + i] = static_cast<uint8_t>(counter >> (24 - 8 * i));
    }

    const char* key = role == ServiceRole::Server ? "serviceid=" : "clientid=";
    uint32_t    n = static_cast<uint32_t>(std::strlen(key));
    std::memcpy(out, key, n);

    // csid_to_string() writes the first octet with std::setw(2) and the
    // default fill, a space.
    if (id[0] < 0x10u) {
        out[n++] = ' ';
    }

    n += append_hex(id[0], out + n);
    for (uint32_t i = 1; i < 16; ++i) {
        out[n++] = '.';
        n += append_hex(id[i], out + n);
    }

    out[n++] = ';';
    out[n] = '\0';
}

}  // namespace rmw_rosette
