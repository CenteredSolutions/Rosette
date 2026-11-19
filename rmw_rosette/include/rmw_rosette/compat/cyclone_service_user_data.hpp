// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/compat/cyclone_service_user_data.hpp
 * @brief The endpoint USER_DATA rmw_cyclonedds_cpp pairs service endpoints by.
 *
 * rmw_cyclonedds_cpp gives each service and client an id, and announces it
 * as SEDP USER_DATA on both of its endpoints:
 *
 * | Role    | Endpoints                         | USER_DATA            |
 * |---------|-----------------------------------|----------------------|
 * | client  | request writer, reply reader      | `clientid=<id>;`     |
 * | service | request reader, reply writer      | `serviceid=<id>;`    |
 *
 * `<id>` is 16 octets, the participant's GUID prefix followed by a
 * per-participant counter, big-endian, written in lowercase hex without
 * leading zeros and separated by dots; the first octet is padded to two
 * characters with a space (`csid_to_string()`).
 *
 * A Cyclone service reads it before it replies: when the request writer
 * announces a `clientid`, rmw_send_response() waits up to 100 ms for a
 * matched reply reader with the same `clientid`, so a reply is not written
 * before the client can receive it. A Cyclone client reads it in
 * rmw_service_server_is_available(): when the service's request reader
 * announces a `serviceid`, the service is available only once a reply
 * writer with the same `serviceid` has matched too. Only equality is
 * compared, so any id unique on the network serves; rosette writes Cyclone's
 * own form so a mixed graph reads the same in every tool.
 *
 * References:
 * - rmw_cyclonedds_cpp 1.3.5 (Humble) and 2.2 (Jazzy) rmw_node.cpp:
 *   get_unique_csid(), csid_to_string(), rmw_init_cs(),
 *   check_for_response_reader(), check_for_service_reader_writer()
 * - OMG DDS 1.4 §2.2.3.1 (USER_DATA); OMG DDSI-RTPS 2.5 §9.6.3.2 (PID_USER_DATA)
 */

#pragma once

#include "rmw_rosette/service_endpoints.hpp"

#include "rosette/rtps/types.hpp"

#include <cstdint>

namespace rmw_rosette {

/// Octets the longest USER_DATA string takes, with its NUL: `serviceid=`, a
/// 47-character id, `;`. rmw_rosette_capacity.cmake adds this much to the
/// default local name arena per service and client.
constexpr uint32_t kCycloneServiceUserDataSize = 10 + 47 + 1 + 1;

/**
 * @brief Writes the USER_DATA a service or client announces on both of its endpoints.
 *
 * @param[in]  role        Server writes `serviceid=`, Client `clientid=`.
 * @param[in]  participant This participant's GUID prefix.
 * @param[in]  counter     A value no other service or client of this participant has used.
 * @param[out] out         The NUL-terminated string.
 *
 * Verification:
 * - Correct means: the string equals what rmw_cyclonedds_cpp's rmw_init_cs()
 *   builds from the same prefix and counter, byte for byte.
 * - Covered by: CycloneServiceUserData.MatchesCycloneForm.
 */
void compose_cyclone_service_user_data(ServiceRole                      role,
                                       const rosette::rtps::GuidPrefix& participant,
                                       uint32_t                         counter,
                                       char (&out)[kCycloneServiceUserDataSize]) noexcept;

}  // namespace rmw_rosette
