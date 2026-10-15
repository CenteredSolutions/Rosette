/**
 * @file rmw_rosette/qos.hpp
 * @brief rmw_qos_profile_t <-> rtps::{WriterConfig,ReaderConfig} translation.
 *
 * rosette::rtps implements two QoS axes — ReliabilityKind (BEST_EFFORT/
 * RELIABLE) and DurabilityKind (VOLATILE/TRANSIENT_LOCAL/TRANSIENT/
 * PERSISTENT) — plus KEEP_LAST history at one of a small closed set of
 * depths (kSupportedWriterDepths below). This matches the QoS subset
 * rmw_microxrcedds_c itself supports; DEADLINE, LIVELINESS, LIFESPAN,
 * OWNERSHIP, and unbounded KEEP_ALL history have no representation in
 * the underlying rtps core.
 */

#pragma once

#include "rmw_rosette/capacity.hpp"
#include "rosette/rtps/endpoint_match_table.hpp"
#include "rosette/rtps/participant.hpp"
#include "types.hpp"

#include <rmw/rmw.h>
#include <rmw/qos_profiles.h>

namespace rmw_rosette {

/**
 * @brief Maps an rmw reliability policy onto the closest rtps::ReliabilityKind.
 *
 * fill_writer_config()/fill_reader_config() call this while translating
 * an `rmw_qos_profile_t` into the config structs `RTPSParticipant::
 * create_writer<>()`/`create_reader()` accept.
 *
 * `SYSTEM_DEFAULT`, `BEST_AVAILABLE` (guarded — not part of the humble
 * ABI this project targets by default; see `ROSETTE_MICRO_ROS_DISTRO` in
 * `cmake/micro_ros.cmake`), and `UNKNOWN` all resolve to `RELIABLE`,
 * matching `rmw_microxrcedds_c`'s own default profile.
 *
 * @param[in] policy Reliability policy from an `rmw_qos_profile_t`.
 * @return The corresponding `rtps::ReliabilityKind`.
 */
inline rtps::ReliabilityKind to_rtps_reliability(rmw_qos_reliability_policy_t policy) noexcept {
    switch (policy) {
        case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
            return rtps::ReliabilityKind::BEST_EFFORT;
        case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
        case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
#ifdef RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE
        case RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE:
#endif
        case RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
        default:
            return rtps::ReliabilityKind::RELIABLE;
    }
}

/**
 * @brief Maps an rmw durability policy onto the closest rtps::DurabilityKind.
 *
 * Used the same way as to_rtps_reliability(), and by
 * qos_profiles_compatible() when checking a publisher/subscription pair.
 *
 * `SYSTEM_DEFAULT`, `BEST_AVAILABLE` (guarded, see to_rtps_reliability()),
 * and `UNKNOWN` all resolve to `VOLATILE`. `TRANSIENT`/`PERSISTENT` have
 * no rmw-level counterpart (the rmw QoS enum stops at TRANSIENT_LOCAL),
 * so they are unreachable from this translation direction.
 *
 * @param[in] policy Durability policy from an `rmw_qos_profile_t`.
 * @return The corresponding `rtps::DurabilityKind`.
 */
inline rtps::DurabilityKind to_rtps_durability(rmw_qos_durability_policy_t policy) noexcept {
    switch (policy) {
        case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
            return rtps::DurabilityKind::TRANSIENT_LOCAL;
        case RMW_QOS_POLICY_DURABILITY_VOLATILE:
        case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
#ifdef RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE
        case RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE:
#endif
        case RMW_QOS_POLICY_DURABILITY_UNKNOWN:
        default:
            return rtps::DurabilityKind::VOLATILE;
    }
}

/// The closed set of CacheDepth instantiations rmw_rosette is willing to
/// compile: depth 1, depth 4, and RMW_ROSETTE_MAX_WRITER_DEPTH (the three
/// coincide where the maximum is 4 or less). Small on purpose: each distinct
/// entry is a StatefulWriter<N,...> instantiation baked into the firmware
/// image, so this list is a size/flexibility trade the application makes
/// once through the maximum, not something to open-end. KEEP_ALL maps to the
/// largest entry; there's no bounded representation for genuinely-unbounded
/// history.
///
/// The maximum is also the depth the participant's deep writer slots are
/// sized for (RTPS_DEEP_WRITER_DEPTH, set to match by
/// rmw_rosette_capacity.cmake), so no writer can be deeper.
inline constexpr uint8_t kMaxWriterDepth = RMW_ROSETTE_MAX_WRITER_DEPTH;
inline constexpr uint8_t kMidWriterDepth = kMaxWriterDepth < 4 ? kMaxWriterDepth : 4;
inline constexpr uint8_t kSupportedWriterDepths[] = {1, kMidWriterDepth, kMaxWriterDepth};

/**
 * @brief Resolves an rmw_qos_profile_t's requested history depth to one
 * of the writer storage depths the participant can create.
 *
 * create_writer_for_qos() calls this to decide which `create_writer<D>()`
 * specialization to invoke for a given publisher.
 *
 * Picks the smallest entry of kSupportedWriterDepths that is greater than
 * or equal to `qos.depth`. `KEEP_ALL` history and a `depth` of 0 both map
 * to the largest supported entry, since neither has a smaller bounded
 * representation; a request larger than every supported entry is clamped
 * to the largest one rather than rejected.
 *
 * @param[in] qos Requested QoS profile.
 * @return One of the values in kSupportedWriterDepths.
 */
inline uint8_t select_writer_depth(const rmw_qos_profile_t& qos) noexcept {
    constexpr uint8_t max_depth =
        kSupportedWriterDepths[sizeof(kSupportedWriterDepths) / sizeof(uint8_t) - 1];
    if (qos.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL || qos.depth == 0) {
        return max_depth;
    }
    for (uint8_t d : kSupportedWriterDepths) {
        if (qos.depth <= d) return d;
    }
    return max_depth;  // Request exceeds every supported depth — clamp.
}

/**
 * @brief Creates a writer whose CacheDepth matches an rmw_qos_profile_t's
 * requested history depth.
 *
 * `rmw_create_publisher()` calls this in place of a direct
 * `RTPSParticipant::create_writer<D>()` call, since the template argument
 * `D` has to be resolved from `qos.depth` at run time.
 *
 * Dispatches select_writer_depth()'s result to the matching
 * `create_writer<D>()`; each depth lays out a
 * `SharedPoolStatefulWriter<D>`, and all of them run
 * the same compiled writer. A new entry in kSupportedWriterDepths needs a
 * matching branch added here.
 *
 * @param[in,out] participant Owns the writer's storage slot.
 * @param[in] cfg Reliability/durability/topic/type config for the writer.
 * @param[in] qos Requested QoS profile; only `history`/`depth` affect the
 *   dispatch here (`reliability`/`durability` are expected to already be
 *   reflected in `cfg` via fill_writer_config()).
 * @return The new writer, or `nullptr` if the participant's writer table
 *   is full.
 */
inline rtps::StatefulWriter* create_writer_for_qos(rtps::RTPSParticipant& participant,
                                                   const rtps::WriterConfig& cfg,
                                                   const rmw_qos_profile_t& qos) noexcept {
    switch (select_writer_depth(qos)) {
        case 1:                  return participant.create_writer<1>(cfg);
        case kMidWriterDepth:    return participant.create_writer<kMidWriterDepth>(cfg);
        default:
        case kMaxWriterDepth:    return participant.create_writer<kMaxWriterDepth>(cfg);               
    }
}

/**
 * @brief Translates an rmw_qos_profile_t's reliability/durability into a
 * WriterConfig.
 *
 * `rmw_create_publisher()` fills a `WriterConfig` this way before passing
 * it to create_writer_for_qos(). History depth is not a `WriterConfig`
 * field — it is the template argument create_writer_for_qos() resolves
 * separately via select_writer_depth().
 *
 * @param[out] cfg Config to fill; `entityId`/`topicName`/`typeName` are
 *   left untouched.
 * @param[in] qos Requested QoS profile.
 */
inline void fill_writer_config(rtps::WriterConfig& cfg, const rmw_qos_profile_t& qos) noexcept {
    cfg.reliability = to_rtps_reliability(qos.reliability);
    cfg.durability  = to_rtps_durability(qos.durability);
}

/**
 * @brief Translates an rmw_qos_profile_t's reliability/durability into a
 * ReaderConfig.
 *
 * The reader-side counterpart to fill_writer_config(), called by
 * `rmw_create_subscription()` before `RTPSParticipant::create_reader()`.
 *
 * @param[out] cfg Config to fill; `entityId`/`topicName`/`typeName` are
 *   left untouched.
 * @param[in] qos Requested QoS profile.
 */
inline void fill_reader_config(rtps::ReaderConfig& cfg, const rmw_qos_profile_t& qos) noexcept {
    cfg.reliability = to_rtps_reliability(qos.reliability);
    cfg.durability  = to_rtps_durability(qos.durability);
}

/**
 * @brief Checks whether a publisher's QoS profile is compatible with a
 * subscription's, per the same rule RTPS endpoint matching applies.
 *
 * Backs `rmw_qos_profile_check_compatible()` (rmw_qos.cpp), which
 * rclcpp/rclpy call before wiring up a publisher/subscription pair so
 * applications can detect an incompatible QoS combination up front
 * rather than discovering it as silent non-delivery.
 *
 * Mirrors `rtps::qos_compatible()` (endpoint_match_table.hpp) in terms of
 * `rmw_qos_profile_t`: a `RELIABLE` subscription requires a `RELIABLE`
 * publisher, and the publisher's durability must be at least as strong
 * as the subscription's (`VOLATILE < TRANSIENT_LOCAL < TRANSIENT <
 * PERSISTENT`).
 *
 * @param[in] publisher Publisher-side QoS profile.
 * @param[in] subscription Subscription-side QoS profile.
 * @return `true` when the two profiles are compatible.
 */
inline bool qos_profiles_compatible(const rmw_qos_profile_t& publisher,
                                     const rmw_qos_profile_t& subscription) noexcept {
    if (subscription.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
        publisher.reliability != RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
        return false;
    }
    const auto pub_durability = to_rtps_durability(publisher.durability);
    const auto sub_durability = to_rtps_durability(subscription.durability);
    if (static_cast<uint32_t>(pub_durability) < static_cast<uint32_t>(sub_durability)) {
        return false;
    }
    return true;
}

/**
 * @brief Resolves a requested QoS profile to the one an endpoint actually runs.
 *
 * Backs `rmw_publisher_get_actual_qos()` and
 * `rmw_subscription_get_actual_qos()`. rclcpp and rclpy report these values
 * to users and use them for QoS-compatibility warnings, so they must
 * describe the RTPS endpoint, not echo the request: `SYSTEM_DEFAULT` and
 * `UNKNOWN` policies are replaced by what to_rtps_reliability() and
 * to_rtps_durability() chose, history is always `KEEP_LAST`, and a
 * writer's depth is the one select_writer_depth() picked.
 *
 * @param[in] requested Profile passed to `rmw_create_publisher()` or
 *   `rmw_create_subscription()`.
 * @param[in] is_writer `true` for a publisher, which resolves depth through
 *   select_writer_depth(); `false` for a subscription, whose depth is bounded
 *   by `reader_depth`.
 * @param[in] reader_depth Capacity of the subscription's pending-change ring.
 * @return The effective profile. Deadline, lifespan, and liveliness fields
 *   are copied unchanged.
 *
 * Verification:
 * - Correct means: a round trip through this function is idempotent, and
 *   reliability/durability agree with the RTPS endpoint's configuration.
 *
 * References:
 * - rmw/types.h (Humble), `rmw_qos_profile_t`
 * - OMG DDS 1.4 §2.2.3 (QoS policies)
 */
inline rmw_qos_profile_t effective_qos(const rmw_qos_profile_t& requested, bool is_writer,
                                       uint32_t reader_depth) noexcept {
    rmw_qos_profile_t qos = requested;
    qos.reliability = to_rtps_reliability(requested.reliability) == rtps::ReliabilityKind::RELIABLE
                          ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                          : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    switch (to_rtps_durability(requested.durability)) {
        case rtps::DurabilityKind::VOLATILE:
            qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            break;
        default:
            qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            break;
    }
    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    if (is_writer) {
        qos.depth = select_writer_depth(requested);
    } else if (qos.depth == 0 || qos.depth > reader_depth ||
               requested.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
        qos.depth = reader_depth;
    }
    return qos;
}

}  // namespace rmw_rosette
