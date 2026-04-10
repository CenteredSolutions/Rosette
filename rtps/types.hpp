/**
 * @file rtps/types.hpp
 * @brief Fundamental RTPS types matching the Platform-Independent Model (PIM).
 *
 * All type and field names are taken directly from the DDSI-RTPS 2.5
 * specification (OMG formal/2022-04-01) §8.2 so that this header can be
 * navigated alongside the PDF without translation.
 *
 * Design constraints
 * ------------------
 *  - No heap allocation; no std::string; no exceptions; no RTTI.
 *  - Plain structs with inline helpers; zero hidden cost.
 *  - C++17, <stdint.h>, <string.h>, <stdatomic.h> only.
 *
 * Spec references are noted per definition.
 */

#pragma once

#include <stdint.h>
#include <string.h>   // memcmp, memcpy, memset
#include <stdbool.h>

#ifndef RTPS_GET_MS
/// Override this macro to provide your platform's monotonic tick.
/// Must return uint32_t milliseconds.
extern "C" uint32_t rtps_platform_get_ms(void);
#define RTPS_GET_MS() rtps_platform_get_ms()
#endif

namespace rtps {

// ============================================================
// §8.2.1  Primitive types
// ============================================================

using Octet  = uint8_t;
using Long   = int32_t;
using ULong  = uint32_t;

// ============================================================
// §8.2.2  SequenceNumber_t
//
// "SequenceNumber_t is a 64-bit signed integer. It is used to identify
//  a CacheChange within a RTPS Writer's HistoryCache."
//  — §8.2.2, Table 8.1, p. 25
//
// Stored as {high, low} matching the wire layout defined in §9.4.2.6.
// ============================================================

struct SequenceNumber {
    Long  high = 0;   ///< Most-significant 32 bits.
    ULong low  = 0;   ///< Least-significant 32 bits.

    /// Construct from a plain int64 — convenience.
    static constexpr SequenceNumber from_int64(int64_t v) noexcept {
        return { static_cast<Long>(v >> 32),
                 static_cast<ULong>(v & 0xFFFFFFFFu) };
    }

    int64_t to_int64() const noexcept {
        return (static_cast<int64_t>(high) << 32) |
               static_cast<int64_t>(low);
    }

    bool operator==(const SequenceNumber& o) const noexcept {
        return high == o.high && low == o.low;
    }
    bool operator!=(const SequenceNumber& o) const noexcept {
        return !(*this == o);
    }
    bool operator<(const SequenceNumber& o) const noexcept {
        return to_int64() < o.to_int64();
    }
    bool operator<=(const SequenceNumber& o) const noexcept {
        return to_int64() <= o.to_int64();
    }
    bool operator>(const SequenceNumber& o) const noexcept {
        return to_int64() > o.to_int64();
    }
    bool operator>=(const SequenceNumber& o) const noexcept {
        return to_int64() >= o.to_int64();
    }

    SequenceNumber operator+(int64_t delta) const noexcept {
        return from_int64(to_int64() + delta);
    }
    SequenceNumber& operator+=(int64_t delta) noexcept {
        *this = *this + delta; return *this;
    }
    SequenceNumber operator-(int64_t delta) const noexcept {
        return from_int64(to_int64() - delta);
    }
    int64_t operator-(const SequenceNumber& o) const noexcept {
        return to_int64() - o.to_int64();
    }
    SequenceNumber& operator++() noexcept {
        *this += 1; return *this;
    }
};

/// Sentinel values — §8.2.2, p. 26.
inline constexpr SequenceNumber SEQUENCENUMBER_UNKNOWN{ -1, 0 };
inline constexpr SequenceNumber SEQUENCENUMBER_ZERO   {  0, 0 };

// ============================================================
// §8.2.3  GuidPrefix_t and EntityId_t
//
// "The GuidPrefix_t is a 12-octet array used as the prefix of a GUID."
// "The EntityId_t is a 4-octet structure."
//  — §8.2.4, Table 8.2, p. 27
// ============================================================

struct GuidPrefix {
    Octet value[12] = {};

    bool operator==(const GuidPrefix& o) const noexcept {
        return memcmp(value, o.value, 12) == 0;
    }
    bool operator!=(const GuidPrefix& o) const noexcept {
        return !(*this == o);
    }
};

inline constexpr GuidPrefix GUIDPREFIX_UNKNOWN{};   ///< All-zeros — §8.2.4.

struct EntityId {
    Octet entityKey[3] = {};    ///< 3-octet key.
    Octet entityKind   = 0;     ///< 1-octet kind.

    bool operator==(const EntityId& o) const noexcept {
        return memcmp(this, &o, sizeof(*this)) == 0;
    }
    bool operator!=(const EntityId& o) const noexcept {
        return !(*this == o);
    }
};

// §8.2.4, Table 8.4 — Pre-defined EntityId values (p. 28-29).
namespace entity_id {
    inline constexpr EntityId UNKNOWN                   = {{0x00,0x00,0x00}, 0x00};
    inline constexpr EntityId PARTICIPANT               = {{0x00,0x00,0x01}, 0xC1};
    inline constexpr EntityId SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER  = {{0x00,0x00,0x03}, 0xC2};
    inline constexpr EntityId SEDP_BUILTIN_PUBLICATIONS_DETECTOR   = {{0x00,0x00,0x03}, 0xC7};
    inline constexpr EntityId SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER = {{0x00,0x00,0x04}, 0xC2};
    inline constexpr EntityId SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR  = {{0x00,0x00,0x04}, 0xC7};
    inline constexpr EntityId SPDP_BUILTIN_PARTICIPANT_WRITER       = {{0x00,0x01,0x00}, 0xC2};
    inline constexpr EntityId SPDP_BUILTIN_PARTICIPANT_READER       = {{0x00,0x01,0x00}, 0xC7};
    inline constexpr EntityId BUILTIN_PARTICIPANT_MESSAGE_WRITER    = {{0x00,0x02,0x00}, 0xC2};
    inline constexpr EntityId BUILTIN_PARTICIPANT_MESSAGE_READER    = {{0x00,0x02,0x00}, 0xC7};
} // namespace entity_id

// ============================================================
// §8.2.4  GUID_t
//
// "GUID_t is a structure that uniquely identifies an RTPS Entity."
//  — §8.2.4, p. 27
// ============================================================

struct Guid {
    GuidPrefix prefix = {};
    EntityId   entityId = {};

    bool operator==(const Guid& o) const noexcept {
        return prefix == o.prefix && entityId == o.entityId;
    }
    bool operator!=(const Guid& o) const noexcept {
        return !(*this == o);
    }
};

inline constexpr Guid GUID_UNKNOWN{};

// ============================================================
// §8.2.5  Locator_t
//
// "Locator_t is used to specify a communication end-point."
// Fields: Kind, Port, Address[16].
//  — §8.2.5, Table 8.7, p. 30
// ============================================================

struct Locator {
    Long  kind    = 0;
    ULong port    = 0;
    Octet address[16] = {};

    bool operator==(const Locator& o) const noexcept {
        return kind == o.kind && port == o.port &&
               memcmp(address, o.address, 16) == 0;
    }
    bool operator!=(const Locator& o) const noexcept {
        return !(*this == o);
    }
};

/// §8.2.5, Table 8.8 — Locator kind constants.
namespace locator_kind {
    inline constexpr Long INVALID  = -1;
    inline constexpr Long RESERVED =  0;
    inline constexpr Long UDPv4    =  1;
    inline constexpr Long UDPv6    =  2;
}

inline constexpr Locator LOCATOR_INVALID{ locator_kind::INVALID, 0, {} };

/// Helper: fill an IPv4 Locator from a 32-bit address and port.
inline Locator make_udpv4_locator(uint32_t ipv4_be, uint16_t port) noexcept {
    Locator l{};
    l.kind = locator_kind::UDPv4;
    l.port = port;
    // §9.4.1.11 — IPv4 occupies the last 4 bytes of the 16-byte address.
    l.address[12] = static_cast<Octet>((ipv4_be >> 24) & 0xFF);
    l.address[13] = static_cast<Octet>((ipv4_be >> 16) & 0xFF);
    l.address[14] = static_cast<Octet>((ipv4_be >>  8) & 0xFF);
    l.address[15] = static_cast<Octet>( ipv4_be        & 0xFF);
    return l;
}

// ============================================================
// §8.2.6  Duration_t
//
// "Duration_t is a time interval. It carries seconds and fraction."
//  — §8.2.6, p. 31
// ============================================================

struct Duration {
    Long  seconds      = 0;
    ULong nanoseconds  = 0;

    static constexpr Duration from_ms(uint32_t ms) noexcept {
        return { static_cast<Long>(ms / 1000u),
                 (ms % 1000u) * 1'000'000u };
    }
    uint32_t to_ms() const noexcept {
        return static_cast<uint32_t>(seconds) * 1000u +
               nanoseconds / 1'000'000u;
    }
    bool operator==(const Duration& o) const noexcept {
        return seconds == o.seconds && nanoseconds == o.nanoseconds;
    }
};

inline constexpr Duration DURATION_ZERO{};
inline constexpr Duration DURATION_INFINITE{ 0x7FFFFFFF, 0xFFFFFFFFu };

// ============================================================
// §8.2.7  Timestamp_t (Time_t in the spec)
//
// "Time_t represents a moment in time expressed in seconds and fraction
//  of a second."  — §8.2.7, p. 31
// ============================================================

struct Timestamp {
    Long  seconds    = 0;
    ULong fraction   = 0;   ///< 2^-32 seconds per count.

    bool operator==(const Timestamp& o) const noexcept {
        return seconds == o.seconds && fraction == o.fraction;
    }

    static Timestamp now() noexcept {
        uint32_t ms = RTPS_GET_MS();
        return { static_cast<Long>(ms / 1000u),
                 static_cast<ULong>((ms % 1000u) * (0xFFFFFFFFu / 1000u)) };
    }
};

inline constexpr Timestamp TIME_ZERO    {  0, 0 };
inline constexpr Timestamp TIME_INVALID { -1, 0xFFFFFFFFu };
inline constexpr Timestamp TIME_INFINITE{  0x7FFFFFFF, 0xFFFFFFFFu };

// ============================================================
// §8.3.5.5  SequenceNumberSet
//
// "SequenceNumberSet_t is a bitmap set of SequenceNumber values relative
//  to a base SequenceNumber."  — §8.3.5.5, p. 54
//
// Max 256-bit (8 × 32-bit words) as specified in §9.4.2.6.
// ============================================================

struct SequenceNumberSet {
    static constexpr int kMaxBits = 256;
    static constexpr int kWords   = kMaxBits / 32;

    SequenceNumber bitmapBase;        ///< Lowest SN represented.
    ULong          numBits = 0;       ///< Number of valid bits (<= 256).
    ULong          bitmap[kWords] = {};

    /// Set bit for (bitmapBase + offset). Returns false if out of range.
    bool set(uint32_t offset) noexcept {
        if (offset >= kMaxBits) return false;
        bitmap[offset / 32] |= (1u << (31u - (offset % 32u)));
        if (offset + 1u > numBits) numBits = offset + 1u;
        return true;
    }

    /// Test bit for (bitmapBase + offset).
    bool test(uint32_t offset) const noexcept {
        if (offset >= numBits) return false;
        return (bitmap[offset / 32] >> (31u - (offset % 32u))) & 1u;
    }

    bool empty() const noexcept { return numBits == 0; }

    void clear() noexcept {
        numBits = 0;
        memset(bitmap, 0, sizeof(bitmap));
    }

    /// Add a single SN to the set.  Sets bitmapBase on first call.
    /// Returns false if the SN is out of range relative to current base.
    bool add(const SequenceNumber& sn) noexcept {
        if (numBits == 0) {
            bitmapBase = sn;
        }
        int64_t offset = sn - bitmapBase;
        if (offset < 0 || offset >= kMaxBits) return false;
        return set(static_cast<uint32_t>(offset));
    }

    /// Iterate over set bits, calling fn(SequenceNumber) for each.
    template<typename Fn>
    void for_each(Fn&& fn) const noexcept {
        for (uint32_t i = 0; i < numBits; ++i) {
            if (test(i)) fn(bitmapBase + static_cast<int64_t>(i));
        }
    }
};

// ============================================================
// §8.2  ProtocolVersion_t and VendorId_t
//
// "ProtocolVersion_t indicates which version of the RTPS protocol
//  is in use."  — §8.2.10, p. 32
// "VendorId_t identifies the vendor of the middleware."
//  — §8.2.11, p. 32
// ============================================================

struct ProtocolVersion {
    Octet major = 0;
    Octet minor = 0;

    bool operator==(const ProtocolVersion& o) const noexcept {
        return major == o.major && minor == o.minor;
    }
};

inline constexpr ProtocolVersion PROTOCOLVERSION_2_5{ 2, 5 };

struct VendorId {
    Octet vendorId[2] = {};

    bool operator==(const VendorId& o) const noexcept {
        return vendorId[0] == o.vendorId[0] &&
               vendorId[1] == o.vendorId[1];
    }
};

/// Vendor ID for this implementation.  Must be registered with OMG.
/// Using 0x0000 as placeholder; replace with a real assignment.
inline constexpr VendorId VENDORID_UNKNOWN   = {};
inline constexpr VendorId VENDORID_THIS_IMPL = { { 0x01, 0xEE } };

// ============================================================
// §8.3.3  ReliabilityKind_t, DurabilityKind_t, ChangeKind_t
//
// Used by QoS policies and history cache operations.
//  — §8.3.3, Table 8.46, p. 71
// ============================================================

enum class ReliabilityKind : uint32_t {
    BEST_EFFORT = 1,    ///< §8.3.3.1
    RELIABLE    = 2,    ///< §8.3.3.2
};

enum class DurabilityKind : uint32_t {
    VOLATILE        = 0,    ///< §8.3.3.3
    TRANSIENT_LOCAL = 1,    ///< §8.3.3.4
    TRANSIENT       = 2,
    PERSISTENT      = 3,
};

/// §8.2.9, Table 8.11  ChangeKind_t — kind of a CacheChange.
enum class ChangeKind : uint32_t {
    ALIVE              = 1,
    NOT_ALIVE_DISPOSED = 2,
    NOT_ALIVE_UNREGISTERED = 4,
};

// ============================================================
// §8.4.1  ChangeForReaderStatus
// §8.4.7  ChangeFromWriterStatus
//
// Used by ReaderProxy and WriterProxy respectively.
//  — §8.4.1.1, Table 8.49, p. 103
//  — §8.4.7.1, Table 8.61, p. 134
// ============================================================

/// Status of a CacheChange as seen by a ReaderProxy (writer side).
/// — §8.4.1.1, p. 103
enum class ChangeForReaderStatus : uint8_t {
    UNSENT         = 0, ///< Change has not yet been sent.
    UNACKNOWLEDGED = 1, ///< Change was sent but not yet acknowledged.
    REQUESTED      = 2, ///< Reader has NACKed and is awaiting retransmit.
    UNDERWAY       = 3, ///< Retransmit is in flight.
    ACKNOWLEDGED   = 4, ///< Reader has acknowledged receipt.
};

/// Status of a CacheChange as seen by a WriterProxy (reader side).
/// — §8.4.7.1, p. 134
enum class ChangeFromWriterStatus : uint8_t {
    UNKNOWN  = 0,   ///< Nothing known about this SN yet.
    MISSING  = 1,   ///< Expected but not yet received.
    RECEIVED = 2,   ///< Received and available.
    LOST     = 3,   ///< Declared lost via GAP; will never be received.
};

// ============================================================
// §8.4.2.1  TopicKind_t
// ============================================================

enum class TopicKind : uint32_t {
    NO_KEY   = 1,
    WITH_KEY = 2,
};

// ============================================================
// Timing helpers
//
// Timing constants are overridable via config.hpp.  All spec timing
// values are in Duration_t (§8.4) but internally we use milliseconds
// for FreeRTOS compatibility.
// ============================================================

/// Default timing constants — §8.4, Table 8.59, p. 132.
#ifndef RTPS_NACK_RESPONSE_DELAY_MS
#  define RTPS_NACK_RESPONSE_DELAY_MS   200u    ///< §8.4.1.1
#endif
#ifndef RTPS_NACK_SUPPRESSION_DURATION_MS
#  define RTPS_NACK_SUPPRESSION_DURATION_MS 0u  ///< §8.4.1.1
#endif
#ifndef RTPS_HEARTBEAT_PERIOD_MS
#  define RTPS_HEARTBEAT_PERIOD_MS      2000u   ///< §8.4.1.1
#endif
#ifndef RTPS_HEARTBEAT_RESPONSE_DELAY_MS
#  define RTPS_HEARTBEAT_RESPONSE_DELAY_MS  0u  ///< §8.4.7.1
#endif

// ============================================================
// §8.5.3.2  SPDPdiscoveredParticipantData
//
// Participant announcement data exchanged via SPDP.
// Only the fields needed for embedded peer-table maintenance
// are stored here; optional QoS not relevant to this target
// are omitted to keep the struct tight.
//  — §8.5.3.2, Table 8.73, p. 162
// ============================================================

/// Maximum number of unicast locators stored per participant.
#ifndef RTPS_MAX_LOCATORS_PER_PARTICIPANT
#  define RTPS_MAX_LOCATORS_PER_PARTICIPANT 4
#endif

struct SPDPdiscoveredParticipantData {
    ProtocolVersion protocolVersion = PROTOCOLVERSION_2_5;
    GuidPrefix      guidPrefix      = {};
    VendorId        vendorId        = VENDORID_UNKNOWN;

    Locator  defaultUnicastLocatorList[RTPS_MAX_LOCATORS_PER_PARTICIPANT]  = {};
    uint8_t  defaultUnicastLocatorCount = 0;

    Locator  defaultMulticastLocatorList[RTPS_MAX_LOCATORS_PER_PARTICIPANT] = {};
    uint8_t  defaultMulticastLocatorCount = 0;

    Locator  metatrafficUnicastLocatorList[RTPS_MAX_LOCATORS_PER_PARTICIPANT]  = {};
    uint8_t  metatrafficUnicastLocatorCount = 0;

    Locator  metatrafficMulticastLocatorList[RTPS_MAX_LOCATORS_PER_PARTICIPANT] = {};
    uint8_t  metatrafficMulticastLocatorCount = 0;

    Duration leaseDuration = { 20, 0 };     ///< Default 20 s — §8.5.3.2.
    uint32_t availableBuiltinEndpoints = 0; ///< Bitmask — §8.5.4.3.
    uint32_t manualLivelinessCount     = 0;
};

// ============================================================
// §8.5.4  BuiltinEndpointSet — bitmask constants
//
// Indicates which builtin endpoints a participant exposes.
//  — §8.5.4.3, Table 8.80, p. 167
// ============================================================

namespace builtin_endpoint {
    inline constexpr uint32_t DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER       = (1u << 0);
    inline constexpr uint32_t DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR        = (1u << 1);
    inline constexpr uint32_t DISC_BUILTIN_ENDPOINT_PUBLICATIONS_ANNOUNCER      = (1u << 2);
    inline constexpr uint32_t DISC_BUILTIN_ENDPOINT_PUBLICATIONS_DETECTOR       = (1u << 3);
    inline constexpr uint32_t DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_ANNOUNCER     = (1u << 4);
    inline constexpr uint32_t DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_DETECTOR      = (1u << 5);
    inline constexpr uint32_t BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_WRITER  = (1u << 10);
    inline constexpr uint32_t BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_READER  = (1u << 11);
}

} // namespace rtps
