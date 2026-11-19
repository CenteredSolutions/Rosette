// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/serialize_for_write.hpp
 * @brief Encodes an outgoing sample, waiting for a pool block as a reliable DDS write does.
 *
 * A reliable writer's history keeps every sample until its readers
 * acknowledge it, and each kept sample pins its pool block. A board sized
 * for a few large samples (RTPS_POOL_CLASS4_COUNT) runs out of large blocks
 * whenever that many are unacknowledged: a service answering back-to-back
 * calls with large replies writes the next reply before the client's
 * ACKNACK for the previous ones arrives. DDS makes such a write block until
 * acknowledgments free room, for at most the RELIABILITY QoS
 * `max_blocking_time`, so the reply is late by one acknowledgment instead
 * of lost. rmw_publish(), rmw_send_request() and rmw_send_response() encode
 * through this function for that reason.
 */

#pragma once

#include "rmw_rosette/message_codec.hpp"
#include "rmw_rosette/types.hpp"

#include <rmw/types.h>

#ifndef RMW_ROSETTE_MAX_BLOCKING_TIME_MS
/// Longest a reliable write waits for a pool block, in milliseconds: the
/// RELIABILITY `max_blocking_time` default of DDS, Fast DDS and Cyclone DDS.
#  define RMW_ROSETTE_MAX_BLOCKING_TIME_MS 100u
#endif

namespace rmw_rosette {

/**
 * @brief Encodes `ros_message` into a pool buffer, waiting for a block when
 *   the pool has none and the writer is reliable.
 *
 * Use it for every sample an rmw entry point writes; serialize_to_pool()
 * alone suits callers that must not run the participant. While it waits,
 * the participant runs (wait_until_ready()), so acknowledgments arrive and
 * release the samples that pin the pool; the pool is checked after every
 * pass, so a block another thread releases is seen at the next pass, which
 * that thread can bring forward with ParticipantDriver::wake(). The caller
 * must not hold the participant lock.
 *
 * @param[in] ctx         Context whose participant runs while this waits.
 * @param[in] codec       Resolved codec for the message's type.
 * @param[in] ros_message Message instance to encode.
 * @param[in] reliability Writer's reliability: RELIABLE waits up to
 *   RMW_ROSETTE_MAX_BLOCKING_TIME_MS; BEST_EFFORT never waits.
 * @param[in] header_size Octets left unwritten at the front of the body for
 *   the caller to fill, as serialize_to_pool() takes them.
 * @return The encoded sample, or an invalid buffer when the message is
 *   larger than RMW_ROSETTE_MAX_SAMPLE_SIZE, or the pool stayed exhausted
 *   for the whole wait.
 *
 * Verification:
 * - Correct means: a reliable write that finds the pool exhausted succeeds
 *   once a block is released within RMW_ROSETTE_MAX_BLOCKING_TIME_MS, and
 *   fails no sooner than that when none is; a best-effort write fails at once.
 * - Covered by: rmw_rosette/test/test_write_waits_for_pool.cpp
 *   (TC-RMW-WRITE-001 to -003).
 *
 * References:
 * - OMG DDS 1.4 §2.2.2.4.2.11 (write blocks on RESOURCE_LIMITS when
 *   RELIABLE), §2.2.3.14 (RELIABILITY `max_blocking_time`, default 100 ms)
 */
rosette::rtps::BufRef serialize_for_write(Context& ctx, const MessageCodec& codec,
                                          const void*                  ros_message,
                                          rmw_qos_reliability_policy_e reliability,
                                          uint32_t                     header_size = 0) noexcept;

}  // namespace rmw_rosette
