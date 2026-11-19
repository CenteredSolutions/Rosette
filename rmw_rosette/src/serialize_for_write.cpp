// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file serialize_for_write.cpp
 * @brief serialize_for_write(): the pool wait of a reliable write.
 */

#include "rmw_rosette/serialize_for_write.hpp"

namespace rtps = rosette::rtps;

namespace rmw_rosette {

rtps::BufRef serialize_for_write(Context& ctx, const MessageCodec& codec, const void* ros_message,
                                 rmw_qos_reliability_policy_e reliability) noexcept {
    uint32_t     refused_size = 0u;
    rtps::BufRef payload = serialize_to_pool(codec, ros_message, &refused_size);
    if (payload.valid() || refused_size == 0u
        || reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
        return payload;
    }

    // Another writer may take the block between the wait and the retry, so
    // the wait repeats until the deadline rather than trusting one wake.
    const uint32_t start_ms = rosette::platform::monotonic_ms();
    for (;;) {
        const uint32_t elapsed_ms = rosette::platform::monotonic_ms() - start_ms;
        if (elapsed_ms >= RMW_ROSETTE_MAX_BLOCKING_TIME_MS) {
            return {};
        }

        // The probe block is released as the predicate returns.
        const bool freed = wait_until_ready(
            ctx, [refused_size]() noexcept { return rtps::buf_alloc(refused_size).valid(); },
            RMW_ROSETTE_MAX_BLOCKING_TIME_MS - elapsed_ms);
        if (!freed) {
            return {};
        }

        payload = serialize_to_pool(codec, ros_message, &refused_size);
        if (payload.valid() || refused_size == 0u) {
            return payload;
        }
    }
}

}  // namespace rmw_rosette
