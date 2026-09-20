/**
 * @file rmw_rosette/qos.hpp
 * @brief rmw_qos_profile_t <-> rtps::{WriterConfig,ReaderConfig} translation.
 *
 * rosette::rtps only implements two QoS axes today — ReliabilityKind
 * (BEST_EFFORT/RELIABLE) and DurabilityKind (VOLATILE/TRANSIENT_LOCAL/
 * TRANSIENT/PERSISTENT) — plus a compile-time-fixed KEEP_LAST history
 * depth (see types.hpp). That happens to be exactly the subset
 * rmw_microxrcedds_c itself supports, so parity with what you have today
 * is achievable; DEADLINE/LIVELINESS/LIFESPAN/OWNERSHIP and KEEP_ALL
 * history are explicitly out of scope until the rtps core grows them.
 */

#pragma once

#include "rosette/rtps/endpoint_match_table.hpp"
#include "rosette/rtps/participant.hpp"
#include "types.hpp"

#include <rmw/rmw.h>
#include <rmw/qos_profiles.h>

namespace rmw_rosette {

inline rtps::ReliabilityKind to_rtps_reliability(rmw_qos_reliability_policy_t policy) noexcept {
    switch (policy) {
        case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
            return rtps::ReliabilityKind::BEST_EFFORT;
        case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
        case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
        case RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE:
        case RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
        default:
            // Default to RELIABLE, matching rmw_microxrcedds_c's default
            // profile — most micro-ROS publishers/subscriptions today are
            // reliable by default.
            return rtps::ReliabilityKind::RELIABLE;
    }
}

inline rtps::DurabilityKind to_rtps_durability(rmw_qos_durability_policy_t policy) noexcept {
    switch (policy) {
        case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
            return rtps::DurabilityKind::TRANSIENT_LOCAL;
        case RMW_QOS_POLICY_DURABILITY_VOLATILE:
        case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
        case RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE:
        case RMW_QOS_POLICY_DURABILITY_UNKNOWN:
        default:
            return rtps::DurabilityKind::VOLATILE;
    }
}

/// Clamps a requested KEEP_LAST depth to what the fixed-depth writer can
/// hold. KEEP_ALL has no bounded representation here — see types.hpp;
/// callers currently just get RMW_ROSETTE_MAX_HISTORY_DEPTH.
inline uint8_t clamp_history_depth(const rmw_qos_profile_t& qos) noexcept {
    if (qos.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
        return RMW_ROSETTE_MAX_HISTORY_DEPTH;
    }
    if (qos.depth == 0 || qos.depth > RMW_ROSETTE_MAX_HISTORY_DEPTH) {
        return RMW_ROSETTE_MAX_HISTORY_DEPTH;
    }
    return static_cast<uint8_t>(qos.depth);
}

inline void fill_writer_config(rtps::WriterConfig& cfg, const rmw_qos_profile_t& qos) noexcept {
    cfg.reliability = to_rtps_reliability(qos.reliability);
    cfg.durability  = to_rtps_durability(qos.durability);
    // NOTE: cfg carries no history-depth field — depth selects which
    // StatefulWriter<D,...> instantiation create_writer<D>() returns
    // (a template argument, not a runtime WriterConfig member). See the
    // rmw_publisher.cpp TODO for how that call is actually made.
}

inline void fill_reader_config(rtps::ReaderConfig& cfg, const rmw_qos_profile_t& qos) noexcept {
    cfg.reliability = to_rtps_reliability(qos.reliability);
    cfg.durability  = to_rtps_durability(qos.durability);
}

/// Mirrors rtps::qos_compatible() (endpoint_match_table.hpp) in terms of
/// rmw_qos_profile_t, for rmw_qos_profile_check_compatible().
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

}  // namespace rmw_rosette
