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

#include <algorithm>
#include <array>
#include <stdint.h>
#include <string.h>   // memcmp, memcpy, memset
#include <stdbool.h>

#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"

namespace rosette::rtps {

// ============================================================
// §8.2.1  Primitive types
// ============================================================

using Octet  = uint8_t;
using Long   = int32_t;
using ULong  = uint32_t;

#ifndef NO_INSTRINSICS
static inline uint16_t bswap16(uint16_t v) noexcept {
    // Generally theres a built-in bswap16 intrinsic that compiles to a single
    // instruction on Cortex-M33 and Cortex-M0+.
    return __builtin_bswap16(v);
}
#else
static inline uint16_t bswap16(uint16_t v) noexcept {
    return static_cast<uint16_t>((v >> 8u) | (v << 8u));
}
#endif

#ifndef NO_INSTRINSICS
static inline uint32_t bswap32(uint32_t v) noexcept {
    // Generally theres a built-in bswap32 intrinsic that compiles to a single
    // instruction on Cortex-M33 and Cortex-M0+.
    return __builtin_bswap32(v);
}
#else
static inline uint32_t bswap32(uint32_t v) noexcept {
    return ((v & 0x000000FFu) << 24u) |
           ((v & 0x0000FF00u) <<  8u) |
           ((v & 0x00FF0000u) >>  8u) |
           ((v & 0xFF000000u) >> 24u);
}
#endif

static inline int32_t bswap32s(int32_t v) noexcept {
    return static_cast<int32_t>(bswap32(static_cast<uint32_t>(v)));
}

static inline void fixup_u16(uint16_t& v, bool need_swap) noexcept {
    if (need_swap) v = bswap16(v);
}

static inline void fixup_u32(uint32_t& v, bool need_swap) noexcept {
    if (need_swap) v = bswap32(v);
}

static inline void fixup_i32(int32_t& v, bool need_swap) noexcept {
    if (need_swap) v = bswap32s(v);
}

/**
 * @brief Views storage as a `T` the caller has already aligned for `T`.
 *
 * Wire structs are written in place into pool blocks, and readers are built in
 * `alignas`-ed slot storage. Both are aligned for `T` by construction, but the
 * compiler cannot see that through an `Octet*`, and a plain cast to `T*` makes
 * -Wcast-align report it. Casting through `void*` states that the caller owns
 * the alignment guarantee; each call site names the invariant (see
 * "In-place framing alignment" in message_builder.hpp) and the CI sanitizer
 * build checks it at run time (-fsanitize=alignment).
 *
 * @tparam T Object type the storage holds.
 * @param[in] p Storage aligned to alignof(T) and at least sizeof(T) octets
 *   long; ownership and lifetime stay with the caller.
 * @return `p` as a pointer to `T`.
 *
 * Verification:
 * - Correct means: every caller passes storage aligned to alignof(T), which
 *   is what a Cortex-M0+ needs to avoid a HardFault on a word access.
 * - Covered by: the static_asserts in message_builder.hpp and the
 *   ROSETTE_SANITIZE spec-test run, which prepends framing in place.
 *
 * References:
 * - ARMv6-M Architecture Reference Manual, A3.2 (unaligned accesses fault)
 */
template <typename T>
inline T* aligned_as(Octet* p) noexcept {
    return static_cast<T*>(static_cast<void*>(p));
}

/// @brief As above, for read-only storage.
template <typename T>
inline const T* aligned_as(const Octet* p) noexcept {
    return static_cast<const T*>(static_cast<const void*>(p));
}

static inline bool host_is_little_endian() noexcept {
    const uint16_t probe = 0x0001u;
    return *reinterpret_cast<const uint8_t*>(&probe) == 0x01u;
}

static inline bool wire_needs_swap(bool wire_little_endian) noexcept {
    return wire_little_endian != host_is_little_endian();
}

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

    /**
     * @brief The sequence number as one signed 64-bit value.
     *
     * `high` is signed on the wire (SEQUENCENUMBER_UNKNOWN is {-1, 0}), so the
     * halves are joined in unsigned arithmetic, where the shift is defined for
     * every `high`, and the result is reinterpreted as two's complement.
     *
     * Verification:
     * - Correct means: from_int64(v).to_int64() == v for every int64 v,
     *   including negative values.
     * - Covered by: TC-SWR-8.4.1.4-007 (an ACKNACK compared against
     *   SEQUENCENUMBER_UNKNOWN), run under ROSETTE_SANITIZE.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §9.4.2.6 (SequenceNumber_t)
     */
    int64_t to_int64() const noexcept {
        const uint64_t joined =
            (static_cast<uint64_t>(static_cast<uint32_t>(high)) << 32) | static_cast<uint64_t>(low);
        return static_cast<int64_t>(joined);
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

    static bool read_from(SequenceNumber& self,
                          const uint8_t* data,
                          size_t len,
                          bool wire_little_endian) noexcept
    {
        if ((data == nullptr) || (len < 8u)) {
            return false;
        }

        memcpy(&self.high, data, sizeof(self.high));
        memcpy(&self.low, data + sizeof(self.high), sizeof(self.low));

        const bool need_swap = wire_needs_swap(wire_little_endian);
        fixup_i32(self.high, need_swap);
        fixup_u32(self.low, need_swap);
        return true;
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
// SampleIdentity_t
//
// Identifies one RTPS sample by writer GUID + sequence number.
// Used by ROS 2 service reply correlation via InlineQoS.
// ============================================================

struct SampleIdentity {
    Guid           writerGuid      = GUID_UNKNOWN;
    SequenceNumber sequenceNumber  = SEQUENCENUMBER_UNKNOWN;

    bool operator==(const SampleIdentity& o) const noexcept {
        return writerGuid == o.writerGuid &&
               sequenceNumber == o.sequenceNumber;
    }
    bool operator!=(const SampleIdentity& o) const noexcept {
        return !(*this == o);
    }
    bool is_unknown() const noexcept {
        return writerGuid == GUID_UNKNOWN &&
               sequenceNumber == SEQUENCENUMBER_UNKNOWN;
    }
};

// ============================================================
// InlineQoS — the DATA-submessage inline QoS parameters this stack uses
//
// §9.4.5.3 (DATA inlineQos), §9.6.3.1 Table 9.13 (PID_KEY_HASH,
// PID_STATUS_INFO), §9.6.3.3 (StatusInfo_t), §9.6.3.8 (KeyHash_t).
//
//  - relatedSampleIdentity: ROS 2 service request/reply correlation. It is
//    written twice, as the DDS-RPC PID (0x0083) and as eProsima's vendor PID
//    (0x800F): Fast DDS 2.6, the ROS 2 Humble default, reads only the
//    latter, and readers skip parameters they do not understand (§9.4.2.11).
//  - keyHash: the instance a change belongs to. For SEDP it is the
//    endpoint GUID (§9.6.3.8: keys of 16 octets or less are sent as-is).
//  - statusInfo: DISPOSED / UNREGISTERED flags carried by a change that
//    withdraws an instance; SEDP uses it to announce endpoint deletion.
// ============================================================

namespace inline_qos_pid {
    inline constexpr uint16_t SENTINEL                        = 0x0001u;
    inline constexpr uint16_t KEY_HASH                        = 0x0070u;
    inline constexpr uint16_t STATUS_INFO                     = 0x0071u;
    inline constexpr uint16_t RELATED_SAMPLE_IDENTITY         = 0x0083u;
    inline constexpr uint16_t RELATED_SAMPLE_IDENTITY_CUSTOM  = 0x800Fu;
}

/// StatusInfo_t flag bits, stored in the last of its four octets (§9.6.3.3).
namespace status_info {
    inline constexpr uint8_t DISPOSED     = 0x01u;
    inline constexpr uint8_t UNREGISTERED = 0x02u;
}

struct InlineQos {
    bool           hasRelatedSampleIdentity = false;
    SampleIdentity relatedSampleIdentity    = {};
    bool           hasKeyHash               = false;
    Octet          keyHash[16]              = {};
    bool           hasStatusInfo            = false;
    uint8_t        statusInfo               = 0u;   ///< status_info:: flags.

    void clear() noexcept {
        *this = InlineQos{};
    }

    constexpr bool empty() const noexcept {
        return !hasRelatedSampleIdentity && !hasKeyHash && !hasStatusInfo;
    }

    /// Sets the key hash to a GUID, the SEDP instance key (§9.6.3.8).
    void set_key_hash(const Guid& guid) noexcept {
        for (uint8_t i = 0; i < 12u; ++i) keyHash[i] = guid.prefix.value[i];
        for (uint8_t i = 0; i < 3u; ++i)  keyHash[12u + i] = guid.entityId.entityKey[i];
        keyHash[15] = guid.entityId.entityKind;
        hasKeyHash = true;
    }

    /// Reads the key hash back as a GUID; meaningful for SEDP instances.
    Guid key_hash_guid() const noexcept {
        Guid g{};
        for (uint8_t i = 0; i < 12u; ++i) g.prefix.value[i] = keyHash[i];
        for (uint8_t i = 0; i < 3u; ++i)  g.entityId.entityKey[i] = keyHash[12u + i];
        g.entityId.entityKind = keyHash[15];
        return g;
    }

    /// True when the change withdraws its instance.
    bool withdraws_instance() const noexcept {
        return hasStatusInfo &&
               (statusInfo & (status_info::DISPOSED | status_info::UNREGISTERED)) != 0u;
    }
};

inline constexpr uint16_t sample_identity_wire_size() noexcept {
    return 24u;
}

/// Largest inline QoS parameter list serialize_inline_qos() produces.
inline constexpr uint16_t kMaxInlineQosSize =
    2u * (4u + 24u) + (4u + 16u) + (4u + 4u) + 4u;

/**
 * @brief Size in octets of an inline QoS parameter list, sentinel included.
 *
 * @param[in] qos Parameters to encode.
 * @return 0 when `qos` is empty (no inline QoS is sent), otherwise the
 *   exact number of octets serialize_inline_qos() writes.
 */
inline constexpr uint16_t inline_qos_serialized_size(const InlineQos& qos) noexcept {
    if (qos.empty()) return 0u;
    uint16_t n = 4u;  // PID_SENTINEL
    if (qos.hasRelatedSampleIdentity) n = static_cast<uint16_t>(n + 2u * (4u + sample_identity_wire_size()));
    if (qos.hasKeyHash)               n = static_cast<uint16_t>(n + 4u + 16u);
    if (qos.hasStatusInfo)            n = static_cast<uint16_t>(n + 4u + 4u);
    return n;
}

/**
 * @brief Encodes inline QoS as a little-endian ParameterList.
 *
 * Shared by MessageBuilder (copy path) and StatefulWriter's zero-copy path,
 * so both put identical octets on the wire.
 *
 * @param[out] dst Destination; must hold inline_qos_serialized_size(qos) octets.
 * @param[in]  qos Parameters to encode; must not be empty.
 * @return Octets written.
 *
 * Verification:
 * - Correct means: message_parser's inline QoS decoder restores every
 *   parameter; covered by TC-BLD-9.4.5.3-005 (copy path) and
 *   TC-SEDP-8.5.4-008 (zero-copy path) in tests/spec.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §9.4.2.11 (ParameterList), §9.6.3.1 Table 9.13
 */
inline uint16_t serialize_inline_qos(Octet* dst, const InlineQos& qos) noexcept {
    uint16_t off = 0u;
    auto put16 = [&](uint16_t v) noexcept {
        dst[off++] = static_cast<Octet>(v & 0xFFu);
        dst[off++] = static_cast<Octet>(v >> 8u);
    };
    auto put32 = [&](uint32_t v) noexcept {
        for (uint8_t i = 0; i < 4u; ++i) dst[off++] = static_cast<Octet>(v >> (8u * i));
    };
    if (qos.hasRelatedSampleIdentity) {
        const SampleIdentity& id = qos.relatedSampleIdentity;
        for (const uint16_t pid : {inline_qos_pid::RELATED_SAMPLE_IDENTITY,
                                   inline_qos_pid::RELATED_SAMPLE_IDENTITY_CUSTOM}) {
            put16(pid);
            put16(sample_identity_wire_size());
            for (uint8_t i = 0; i < 12u; ++i) dst[off++] = id.writerGuid.prefix.value[i];
            for (uint8_t i = 0; i < 3u; ++i)  dst[off++] = id.writerGuid.entityId.entityKey[i];
            dst[off++] = id.writerGuid.entityId.entityKind;
            put32(static_cast<uint32_t>(id.sequenceNumber.high));
            put32(id.sequenceNumber.low);
        }
    }
    if (qos.hasKeyHash) {
        put16(inline_qos_pid::KEY_HASH);
        put16(16u);
        for (uint8_t i = 0; i < 16u; ++i) dst[off++] = qos.keyHash[i];
    }
    if (qos.hasStatusInfo) {
        put16(inline_qos_pid::STATUS_INFO);
        put16(4u);
        dst[off++] = 0u; dst[off++] = 0u; dst[off++] = 0u;
        dst[off++] = qos.statusInfo;
    }
    put16(inline_qos_pid::SENTINEL);
    put16(0u);
    return off;
}

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

    static bool read_from(Locator& self,
                          const uint8_t* data,
                          size_t len,
                          bool wire_little_endian) noexcept
    {
        if ((data == nullptr) || (len < sizeof(Locator))) {
            return false;
        }

        memcpy(&self.kind, data, sizeof(self.kind));
        memcpy(&self.port, data + sizeof(self.kind), sizeof(self.port));
        memcpy(self.address,
               data + sizeof(self.kind) + sizeof(self.port),
               sizeof(self.address));

        const bool need_swap = wire_needs_swap(wire_little_endian);
        fixup_i32(self.kind, need_swap);
        fixup_u32(self.port, need_swap);
        return true;
    }

    bool is_udpv4() const noexcept { return kind == 1; }
    bool is_udpv6() const noexcept { return kind == 2; }

    const Octet* ipv4_address() const noexcept { return address + 12; }
    Octet* ipv4_address() noexcept { return address + 12; }

    const Octet* ipv6_address() const noexcept { return address; }
    Octet* ipv6_address() noexcept { return address; }

    uint32_t ipv4_address_be() const noexcept
    {
        const Octet* p = ipv4_address();
        return (static_cast<uint32_t>(p[0]) << 24u) |
               (static_cast<uint32_t>(p[1]) << 16u) |
               (static_cast<uint32_t>(p[2]) << 8u) |
               static_cast<uint32_t>(p[3]);
    }

    std::array<Octet, 4> ipv4_address_bytes() const noexcept
    {
        const Octet* p = ipv4_address();
        return {p[0], p[1], p[2], p[3]};
    }

    std::array<Octet, 16> ipv6_address_bytes() const noexcept
    {
        std::array<Octet, 16> out{};
        memcpy(out.data(), address, out.size());
        return out;
    }

    void set_ipv4_address_be(uint32_t ipv4_be) noexcept
    {
        address[12] = static_cast<Octet>((ipv4_be >> 24u) & 0xFFu);
        address[13] = static_cast<Octet>((ipv4_be >> 16u) & 0xFFu);
        address[14] = static_cast<Octet>((ipv4_be >> 8u) & 0xFFu);
        address[15] = static_cast<Octet>(ipv4_be & 0xFFu);
    }

    void set_ipv6_address(const uint8_t (&ipv6)[16]) noexcept
    {
        memcpy(address, ipv6, sizeof(address));
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
    l.set_ipv4_address_be(ipv4_be);
    return l;
}

/// Helper: fill an IPv6 Locator from a 16-byte address and port.
inline Locator make_udpv6_locator(const uint8_t (&ipv6)[16], uint16_t port) noexcept {
    Locator l{};
    l.kind = locator_kind::UDPv6;
    l.port = port;
    l.set_ipv6_address(ipv6);
    return l;
}

/**
 * @brief The entry of a locator list a proxy refers to for sending.
 *
 * ReaderProxy and WriterProxy keep a pointer to one entry of the list they
 * were matched with rather than a copy of the list (see "Locator reference"
 * in rtps/reader_proxy.hpp). This chooses that entry: the first UDP locator,
 * because some peers advertise other transports (for example shared memory)
 * ahead of UDP, else the first entry so a list of other kinds is still
 * addressed as announced.
 *
 * @param[in] list  The locator list; may be null when `count` is 0. The
 *   returned pointer aliases it, so it carries the list's lifetime.
 * @param[in] count Entries in `list`.
 * @return A pointer into `list`; null when `count` is 0 or `list` is null.
 *
 * Verification:
 * - Correct means: the result is the first UDPv4/UDPv6 entry when one exists,
 *   else `list`, else null; it always points into `list`.
 * - Covered by: TC-SWR-8.4.1.1-001, TC-SRR-8.4.10-004.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.4.1.1 (ReaderProxy.unicastLocatorList),
 *   §8.4.10.4 (WriterProxy.unicastLocatorList), §8.2.5 (Locator_t kinds)
 */
inline const Locator* preferred_unicast_locator(const Locator* list, uint8_t count) noexcept {
    if ((list == nullptr) || (count == 0u)) {
        return nullptr;
    }

    for (uint8_t i = 0; i < count; ++i) {
        if (list[i].is_udpv4() || list[i].is_udpv6()) {
            return &list[i];
        }
    }

    return &list[0];
}

/**
 * @brief The current value of a proxy's referenced locator, or LOCATOR_INVALID.
 *
 * The record that owns the referenced entry updates it in place, and empties
 * it (a value-initialized Locator, kind RESERVED) when the list shrinks; a
 * proxy must not send to such an entry.
 *
 * @param[in] ref The proxy's reference (preferred_unicast_locator()); may be null.
 * @return `*ref`, or LOCATOR_INVALID when `ref` is null or its kind is
 *   INVALID or RESERVED.
 *
 * Verification:
 * - Correct means: a send never targets a RESERVED or INVALID locator.
 * - Covered by: TC-SWR-8.4.1.1-001.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.2.5, Table 8.8 (LOCATOR_KIND_INVALID, LOCATOR_KIND_RESERVED)
 */
inline const Locator& referenced_locator(const Locator* ref) noexcept {
    if ((ref == nullptr) || (ref->kind == locator_kind::INVALID)
        || (ref->kind == locator_kind::RESERVED)) {
        return LOCATOR_INVALID;
    }

    return *ref;
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

    static bool read_from(Duration& self,
                          const uint8_t* data,
                          size_t len,
                          bool wire_little_endian) noexcept
    {
        if ((data == nullptr) || (len < 8u)) {
            return false;
        }

        memcpy(&self.seconds, data, sizeof(self.seconds));
        memcpy(&self.nanoseconds, data + sizeof(self.seconds), sizeof(self.nanoseconds));

        const bool need_swap = wire_needs_swap(wire_little_endian);
        fixup_i32(self.seconds, need_swap);
        fixup_u32(self.nanoseconds, need_swap);
        return true;
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

    /**
     * @brief The current wall-clock time as an RTPS Time_t, for a DATA's INFO_TS.
     *
     * The RTPS counterpart of ros::time_stamp(): one lock-free read of the
     * single platform clock (WallClock::now_us()), split with one 64-bit
     * divide by 10^6 that yields the quotient and the remainder together.
     * The sub-second microseconds become 2^-32 fractions by
     * fraction_of_microseconds(), a multiply rather than a second divide.
     * Before the clock is synchronized this is the monotonic time itself,
     * near 1970, which peers read as unsynchronized.
     *
     * @return Seconds and 2^-32 fractions since the Unix epoch; zero (the spec's TIME_ZERO) for
     *   an instant before the epoch, which no time source sets.
     *
     * Verification:
     * - Correct means: seconds is the wall clock's whole seconds and fraction
     *   is floor(sub-second microseconds * 2^32 / 10^6).
     * - Covered by: tests/host/rtps_timestamp_test.cpp (TC-RTPS-TIME-001,
     *   TC-RTPS-TIME-002).
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.9.8 (INFO_TS), §9.3.2 (Time_t mapping)
     */
    static Timestamp now() noexcept {
        const int64_t unix_us = rosette::platform::WallClock::now_us();
        if (unix_us < 0) {
            return {};
        }

        const uint64_t us = static_cast<uint64_t>(unix_us);
        const uint32_t sub_us = static_cast<uint32_t>(us % 1000000u);
        return {static_cast<Long>(us / 1000000u), fraction_of_microseconds(sub_us)};
    }

    /**
     * @brief Converts microseconds within a second to 2^-32 fractions of a second, exactly.
     *
     * floor(sub_us * 2^32 / 10^6) without a 64-bit divide. The product with
     * ceil(2^64 / 10^6), shifted down 32 bits, is never below the exact
     * fraction and at most one above it (no reciprocal that fits 64 bits is
     * exact for every input); one 32x32-to-64 multiply then detects the case
     * that is one above and corrects it.
     *
     * @param[in] sub_us Microseconds past the whole second, below 10^6.
     * @return The fraction, in 2^-32 seconds.
     *
     * Verification:
     * - Correct means: equals floor(sub_us * 2^32 / 10^6) for every sub_us
     *   in [0, 10^6).
     * - Covered by: tests/host/rtps_timestamp_test.cpp (TC-RTPS-TIME-001,
     *   all 10^6 inputs).
     */
    static constexpr ULong fraction_of_microseconds(uint32_t sub_us) noexcept {
        constexpr uint64_t kCeilTwo64PerMillion = 18446744073710u;  // ceil(2^64 / 10^6)
        uint64_t           fraction = (static_cast<uint64_t>(sub_us) * kCeilTwo64PerMillion) >> 32;
        if (fraction * 1000000u > (static_cast<uint64_t>(sub_us) << 32)) {
            --fraction;
        }

        return static_cast<ULong>(fraction);
    }

    static bool read_from(Timestamp& self,
                          const uint8_t* data,
                          size_t len,
                          bool wire_little_endian) noexcept
    {
        if ((data == nullptr) || (len < 8u)) {
            return false;
        }

        memcpy(&self.seconds, data, sizeof(self.seconds));
        memcpy(&self.fraction, data + sizeof(self.seconds), sizeof(self.fraction));

        const bool need_swap = wire_needs_swap(wire_little_endian);
        fixup_i32(self.seconds, need_swap);
        fixup_u32(self.fraction, need_swap);
        return true;
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

/// Vendors whose behavior Rosette adapts to (OMG DDSI-RTPS vendor ID registry).
inline constexpr VendorId VENDORID_RTI_CONNEXT = {{0x01, 0x01}};
inline constexpr VendorId VENDORID_EPROSIMA = {{0x01, 0x0F}};

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

/// Most writers one user StatefulReader matches at once (its cap in the
/// participant's WriterProxy pool); also the default number of fragment
/// reassembly slots per reader.
#ifndef RTPS_MAX_MATCHED_WRITERS
#  define RTPS_MAX_MATCHED_WRITERS 8
#endif

// ============================================================
// §8.5.3.2  SPDPdiscoveredParticipantData
//
// Participant announcement data exchanged via SPDP: what the local
// participant announces, and what one received announcement decodes to.
// Optional QoS not relevant to this target are omitted. The SPDP peer
// table does not store this struct; it keeps the smaller
// DiscoveredParticipant (spdp.hpp) built from it.
//  — §8.5.3.2, Table 8.73, p. 162
// ============================================================

/// Remote participants tracked at once: the SPDP peer table's size, and so
/// the most participants SEDP exchanges endpoints with.
#ifndef RTPS_MAX_PEERS
#  define RTPS_MAX_PEERS 8
#endif

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

}  // namespace rosette::rtps
