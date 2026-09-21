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
#ifdef RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE
        // Added in a later rmw than this project's default target distro
        // (humble) — guarded so this still compiles there without falling
        // behind on a newer one.
        case RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE:
#endif
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
#ifdef RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE
        case RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE:
#endif
        case RMW_QOS_POLICY_DURABILITY_UNKNOWN:
        default:
            return rtps::DurabilityKind::VOLATILE;
    }
}

/// The closed set of CacheDepth instantiations rmw_rosette is willing to
/// compile. Small and power-of-two-ish on purpose — each entry here is a
/// distinct StatefulWriter<N,...> instantiation baked into the firmware
/// image, so this list is a size/flexibility trade the application makes
/// once, not something to open-end. KEEP_ALL maps to the largest entry;
/// there's no bounded representation for genuinely-unbounded history.
///
/// Capped at 8, not higher: RTPSParticipant's writer storage table sizes
/// each slot to fit its default RTPS_PARTICIPANT_MAX_WRITER_BYTES, which
/// is itself sized for StatefulWriter<8, RTPS_MAX_MATCHED_READERS> (see
/// participant.hpp) — create_writer<16>() would trip that class's
/// static_assert unless a board config raises the byte budget to match.
/// Going past 8 is a real option, just not a default one.
inline constexpr uint8_t kSupportedWriterDepths[] = {1, 4, 8};

/// Picks the smallest supported depth that's >= the profile's request.
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

/// Runtime depth -> compile-time create_writer<D>() call, returned through
/// the depth-erased IStatefulWriter interface (see types.hpp). Every arm
/// is a real, distinct StatefulWriter<D, RTPS_MAX_MATCHED_READERS>
/// instantiation; add a case here if kSupportedWriterDepths grows.
inline rtps::IStatefulWriter* create_writer_for_qos(rtps::RTPSParticipant& participant,
                                                     const rtps::WriterConfig& cfg,
                                                     const rmw_qos_profile_t& qos) noexcept {
    switch (select_writer_depth(qos)) {
        case 1:  return participant.create_writer<1>(cfg);
        case 4:  return participant.create_writer<4>(cfg);
        case 8:  return participant.create_writer<8>(cfg);
        default: return nullptr;  // unreachable given select_writer_depth()'s range
    }
}

inline void fill_writer_config(rtps::WriterConfig& cfg, const rmw_qos_profile_t& qos) noexcept {
    cfg.reliability = to_rtps_reliability(qos.reliability);
    cfg.durability  = to_rtps_durability(qos.durability);
    // History depth doesn't live on WriterConfig — it's the template
    // argument create_writer_for_qos() resolves above.
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
