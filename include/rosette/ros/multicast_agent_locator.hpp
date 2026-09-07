#pragma once

/**
 * @file ros/multicast_agent_locator.hpp
 * @brief Upstream-compatible Micro XRCE-DDS Agent discovery over UDP multicast.
 *
 * Design goal
 * -----------
 * Reuse Rosette's socket abstraction while speaking the same discovery request
 * and reply format used by the upstream Micro XRCE-DDS client and agent.
 *
 * Why this exists
 * ---------------
 * The upstream discovery helper is documented as Linux-only, but the discovery
 * exchange itself is just UDP plus DDS-XRCE `GET_INFO` and `INFO` submessages.
 * Rosette can therefore preserve wire compatibility on MCU targets by
 * reimplementing that bounded exchange on top of its existing socket layer.
 *
 * References
 * ----------
 * - Micro XRCE-DDS client docs: `uxr_discovery_agents_default()` uses
 *   multicast `239.255.0.2:7400`.
 * - eProsima Micro XRCE-DDS Client `discovery.c`: request shape and callback
 *   behavior for `GET_INFO` / `INFO`.
 * - eProsima Micro XRCE-DDS Agent `Processor::process_get_info_packet()`:
 *   reply shape with agent configuration and transport locator sequence.
 */

#include "agent_locator.hpp"

#include "rosette/net/inet.h"
#include "rosette/net/socket.hpp"

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <cstring>
#include <memory>

namespace rosette {

/**
 * @brief Filters candidate Agent transport locators.
 *
 * An Agent's `INFO` reply can advertise several addresses (for example a
 * container's bridge IP alongside its host-routable one, or an IPv4 locator
 * next to an IPv6 one). This is called once per locator that decoded as a
 * supported address format; return true to accept it, false to keep looking
 * at the remaining locators in the same reply.
 */
using AgentAddressFilter =
    bool (*)(const rosette::net::SocketAddress& address) noexcept;

/// Rejects the unspecified address and loopback, IPv4 ("0.0.0.0",
/// "127.0.0.1") or IPv6 ("::", "::1").
inline bool reject_unreachable_agent_address(
    const rosette::net::SocketAddress& address) noexcept
{
    if (address.is_ipv4()) {
        static constexpr std::array<uint8_t, 4> kUnspecified = {0u, 0u, 0u, 0u};
        static constexpr std::array<uint8_t, 4> kLoopback = {127u, 0u, 0u, 1u};
        const auto ipv4 = address.ipv4();
        return (ipv4 != kUnspecified) && (ipv4 != kLoopback);
    }
    if (address.is_ipv6()) {
        std::array<uint8_t, 16> unspecified{};
        std::array<uint8_t, 16> loopback{};
        loopback[15] = 1u;
        return (address.address != unspecified) && (address.address != loopback);
    }
    return false;
}

struct MulticastAgentLocatorConfig {
    const char* multicast_group = "239.255.0.2";
    uint16_t multicast_port = 7400u;
    uint16_t default_agent_port = 8888u;
    uint32_t receive_timeout_ms = 750u;
    uint8_t attempts = 3u;
    /// Accepts or rejects each candidate locator from the Agent's INFO reply.
    /// Defaults to rejecting 0.0.0.0 and 127.0.0.1; firmware can override this
    /// to pick out whichever advertised address is actually reachable from it.
    AgentAddressFilter address_filter = reject_unreachable_agent_address;
};

namespace detail {

inline constexpr uint8_t kSessionIdWithoutClientKey = 0x80u;
inline constexpr uint8_t kStreamIdNone = 0x00u;
inline constexpr uint8_t kRequestIdLow = 0x00u;
inline constexpr uint8_t kRequestIdHigh = 0x09u;

inline constexpr uint8_t kSubmessageIdGetInfo = 0x02u;
inline constexpr uint8_t kSubmessageIdInfo = 0x06u;
inline constexpr uint8_t kFlagLittleEndianness = 0x01u;

inline constexpr uint8_t kObjectKindAgent = 0x0Du;
inline constexpr uint8_t kAddressFormatSmall = 0x00u;
inline constexpr uint8_t kAddressFormatMedium = 0x01u;
inline constexpr uint8_t kAddressFormatLarge = 0x02u;
inline constexpr uint8_t kAddressFormatString = 0x03u;

inline constexpr uint8_t kStatusOk = 0x00u;
inline constexpr uint8_t kImplementationStatusUnknown = 0x00u;

inline constexpr uint32_t kInfoMaskConfiguration = 0x01u << 0u;
inline constexpr uint32_t kInfoMaskActivity = 0x01u << 1u;
inline constexpr uint32_t kInfoMaskDefault =
    kInfoMaskConfiguration | kInfoMaskActivity;

inline constexpr uint8_t kObjectIdAgent[2] = {0xFFu, 0xFDu};
inline constexpr uint8_t kXrceCookie[4] = {'X', 'R', 'C', 'E'};
inline constexpr uint8_t kXrceVersion[2] = {0x01u, 0x00u};
inline constexpr uint8_t kVendorIdInvalid[2] = {0x00u, 0x00u};

inline constexpr size_t kGetInfoPayloadSize = 8u;
inline constexpr size_t kGetInfoPacketSize = 16u;
inline constexpr size_t kMaxDiscoveryPacketSize = 128u;

class LittleEndianCdrWriter {
public:
    LittleEndianCdrWriter(uint8_t* data,
                          size_t capacity) noexcept
        : data_(data)
        , capacity_(capacity)
    {}

    bool ok() const noexcept { return ok_; }
    size_t size() const noexcept { return pos_; }

    void write_u8(uint8_t value) noexcept
    {
        write_bytes(&value, sizeof(value));
    }

    void write_bool(bool value) noexcept
    {
        write_u8(value ? 1u : 0u);
    }

    void write_i16(int16_t value) noexcept
    {
        align(2u);
        const uint16_t raw = static_cast<uint16_t>(value);
        write_u16(raw);
    }

    void write_u16(uint16_t value) noexcept
    {
        align(2u);
        const uint16_t le = value;
        const uint8_t bytes[2] = {
            static_cast<uint8_t>(le & 0xFFu),
            static_cast<uint8_t>((le >> 8u) & 0xFFu),
        };
        write_bytes(bytes, sizeof(bytes));
    }

    void write_u32(uint32_t value) noexcept
    {
        align(4u);
        const uint8_t bytes[4] = {
            static_cast<uint8_t>(value & 0xFFu),
            static_cast<uint8_t>((value >> 8u) & 0xFFu),
            static_cast<uint8_t>((value >> 16u) & 0xFFu),
            static_cast<uint8_t>((value >> 24u) & 0xFFu),
        };
        write_bytes(bytes, sizeof(bytes));
    }

    void write_bytes(const uint8_t* bytes,
                     size_t length) noexcept
    {
        if ((bytes == nullptr) || !reserve(length)) {
            ok_ = false;
            return;
        }
        std::memcpy(data_ + pos_, bytes, length);
        pos_ += length;
    }

private:
    bool reserve(size_t length) noexcept
    {
        return ok_ && (pos_ + length) <= capacity_;
    }

    void align(size_t alignment) noexcept
    {
        if (alignment <= 1u) {
            return;
        }

        const size_t padding = (alignment - (pos_ % alignment)) % alignment;
        if (!reserve(padding)) {
            ok_ = false;
            return;
        }
        std::memset(data_ + pos_, 0, padding);
        pos_ += padding;
    }

    uint8_t* data_ = nullptr;
    size_t capacity_ = 0u;
    size_t pos_ = 0u;
    bool ok_ = true;
};

class LittleEndianCdrReader {
public:
    LittleEndianCdrReader(const uint8_t* data,
                          size_t size) noexcept
        : data_(data)
        , size_(size)
    {}

    bool ok() const noexcept { return ok_; }
    size_t remaining() const noexcept { return ok_ ? (size_ - pos_) : 0u; }

    bool read_u8(uint8_t& value) noexcept
    {
        return read_bytes(&value, sizeof(value));
    }

    bool read_bool(bool& value) noexcept
    {
        uint8_t raw = 0u;
        if (!read_u8(raw)) {
            return false;
        }
        value = (raw != 0u);
        return true;
    }

    bool read_i16(int16_t& value) noexcept
    {
        uint16_t raw = 0u;
        if (!read_u16(raw)) {
            return false;
        }
        value = static_cast<int16_t>(raw);
        return true;
    }

    bool read_u16(uint16_t& value) noexcept
    {
        align(2u);
        if (!ensure(2u)) {
            return false;
        }
        value = static_cast<uint16_t>(data_[pos_]) |
                (static_cast<uint16_t>(data_[pos_ + 1u]) << 8u);
        pos_ += 2u;
        return true;
    }

    bool read_u32(uint32_t& value) noexcept
    {
        align(4u);
        if (!ensure(4u)) {
            return false;
        }
        value = static_cast<uint32_t>(data_[pos_]) |
                (static_cast<uint32_t>(data_[pos_ + 1u]) << 8u) |
                (static_cast<uint32_t>(data_[pos_ + 2u]) << 16u) |
                (static_cast<uint32_t>(data_[pos_ + 3u]) << 24u);
        pos_ += 4u;
        return true;
    }

    bool read_bytes(uint8_t* out,
                    size_t length) noexcept
    {
        if ((out == nullptr) || !ensure(length)) {
            return false;
        }
        std::memcpy(out, data_ + pos_, length);
        pos_ += length;
        return true;
    }

private:
    bool ensure(size_t length) noexcept
    {
        if (!ok_ || (pos_ + length) > size_) {
            ok_ = false;
            return false;
        }
        return true;
    }

    void align(size_t alignment) noexcept
    {
        if (alignment <= 1u) {
            return;
        }
        const size_t padding = (alignment - (pos_ % alignment)) % alignment;
        if (!ensure(padding)) {
            return;
        }
        pos_ += padding;
    }

    const uint8_t* data_ = nullptr;
    size_t size_ = 0u;
    size_t pos_ = 0u;
    bool ok_ = true;
};

inline void write_message_header(LittleEndianCdrWriter& writer) noexcept
{
    writer.write_u8(kSessionIdWithoutClientKey);
    writer.write_u8(kStreamIdNone);
    writer.write_u16(0u);
}

inline void write_submessage_header(LittleEndianCdrWriter& writer,
                                    uint8_t submessage_id,
                                    uint16_t submessage_length) noexcept
{
    writer.write_u8(submessage_id);
    writer.write_u8(kFlagLittleEndianness);
    writer.write_u16(submessage_length);
}

inline bool build_get_info_packet(uint8_t out[kGetInfoPacketSize]) noexcept
{
    LittleEndianCdrWriter writer(out, kGetInfoPacketSize);
    write_message_header(writer);
    write_submessage_header(writer,
                            kSubmessageIdGetInfo,
                            static_cast<uint16_t>(kGetInfoPayloadSize));
    writer.write_u8(kRequestIdLow);
    writer.write_u8(kRequestIdHigh);
    writer.write_bytes(kObjectIdAgent, sizeof(kObjectIdAgent));
    writer.write_u32(kInfoMaskDefault);
    return writer.ok() && (writer.size() == kGetInfoPacketSize);
}

/**
 * @brief Decode one XRCE transport locator into a socket address.
 *
 * @param[out] out           Decoded endpoint; only meaningful when
 *                           @p out_supported is true.
 * @param[out] out_supported True for the address formats Rosette speaks
 *                           (medium = IPv4, large = IPv6); the small
 *                           (well-known) and string (e.g. serial) formats
 *                           decode but are left unsupported.
 * @return False only when the rdata itself was malformed; an unsupported but
 *         well-formed format still returns true with @p out_supported false.
 */
inline bool parse_transport_locator(
    LittleEndianCdrReader& reader,
    rosette::net::SocketAddress& out,
    bool& out_supported) noexcept
{
    uint8_t format = 0u;
    if (!reader.read_u8(format)) {
        return false;
    }

    out_supported = false;
    switch (format) {
    case kAddressFormatSmall: {
        uint8_t address[2] = {};
        uint8_t port = 0u;
        if (!reader.read_bytes(address, sizeof(address)) ||
            !reader.read_u8(port)) {
            return false;
        }
        break;
    }
    case kAddressFormatMedium: {
        std::array<uint8_t, 4> address{};
        uint16_t port = 0u;
        if (!reader.read_bytes(address.data(), address.size()) ||
            !reader.read_u16(port)) {
            return false;
        }
        out = rosette::net::make_socket_address_v4(address, port);
        out_supported = true;
        break;
    }
    case kAddressFormatLarge: {
        std::array<uint8_t, 16> address{};
        uint32_t port = 0u;
        if (!reader.read_bytes(address.data(), address.size()) ||
            !reader.read_u32(port)) {
            return false;
        }
        // The wire field is 4 bytes, but UDP/TCP ports are 16-bit; the upper
        // half is padding, not part of the port value.
        out = rosette::net::make_socket_address_v6(
            address, static_cast<uint16_t>(port & 0xFFFFu));
        out_supported = true;
        break;
    }
    case kAddressFormatString:
    default:
        return false;
    }

    return true;
}

/**
 * @brief Parse an `INFO` reply into up to @p out_capacity accepted locators.
 *
 * @param[in]  filter       Accepts or rejects each candidate address; a
 *                          locator is skipped (not treated as failure) when
 *                          rejected.
 * @param[out] out          Destination array for accepted endpoints.
 * @param[in]  out_capacity Number of entries @p out can hold.
 * @param[out] out_count    Number of endpoints written to @p out.
 * @return True when at least one endpoint was written.
 */
inline bool parse_info_packet(const uint8_t* packet,
                              size_t length,
                              AgentAddressFilter filter,
                              rosette::net::SocketAddress* out,
                              size_t out_capacity,
                              size_t& out_count) noexcept
{
    out_count = 0u;
    if ((packet == nullptr) || (length < kGetInfoPacketSize) ||
        (out == nullptr) || (out_capacity == 0u)) {
        return false;
    }

    LittleEndianCdrReader reader(packet, length);

    uint8_t session_id = 0u;
    uint8_t stream_id = 0u;
    uint16_t sequence_number = 0u;
    if (!reader.read_u8(session_id) ||
        !reader.read_u8(stream_id) ||
        !reader.read_u16(sequence_number)) {
        return false;
    }
    if ((session_id != kSessionIdWithoutClientKey) ||
        (stream_id != kStreamIdNone)) {
        return false;
    }

    uint8_t submessage_id = 0u;
    uint8_t flags = 0u;
    uint16_t submessage_length = 0u;
    if (!reader.read_u8(submessage_id) ||
        !reader.read_u8(flags) ||
        !reader.read_u16(submessage_length)) {
        return false;
    }
    if ((submessage_id != kSubmessageIdInfo) ||
        ((flags & kFlagLittleEndianness) == 0u) ||
        (reader.remaining() < submessage_length)) {
        return false;
    }

    uint8_t request_id[2] = {};
    uint8_t object_id[2] = {};
    uint8_t status = 0u;
    uint8_t implementation_status = 0u;
    bool optional_config = false;
    if (!reader.read_bytes(request_id, sizeof(request_id)) ||
        !reader.read_bytes(object_id, sizeof(object_id)) ||
        !reader.read_u8(status) ||
        !reader.read_u8(implementation_status) ||
        !reader.read_bool(optional_config)) {
        return false;
    }
    (void)implementation_status;

    if ((request_id[0] != kRequestIdLow) ||
        (request_id[1] != kRequestIdHigh) ||
        (object_id[0] != kObjectIdAgent[0]) ||
        (object_id[1] != kObjectIdAgent[1]) ||
        (status != kStatusOk) ||
        !optional_config) {
        return false;
    }

    uint8_t config_kind = 0u;
    uint8_t cookie[4] = {};
    uint8_t version[2] = {};
    uint8_t vendor[2] = {};
    bool optional_properties = false;
    if (!reader.read_u8(config_kind) ||
        !reader.read_bytes(cookie, sizeof(cookie)) ||
        !reader.read_bytes(version, sizeof(version)) ||
        !reader.read_bytes(vendor, sizeof(vendor)) ||
        !reader.read_bool(optional_properties)) {
        return false;
    }
    if ((config_kind != kObjectKindAgent) ||
        (std::memcmp(cookie, kXrceCookie, sizeof(cookie)) != 0) ||
        (std::memcmp(version, kXrceVersion, sizeof(version)) != 0) ||
        optional_properties) {
        return false;
    }

    bool optional_activity = false;
    uint8_t activity_kind = 0u;
    if (!reader.read_bool(optional_activity) ||
        !optional_activity ||
        !reader.read_u8(activity_kind) ||
        (activity_kind != kObjectKindAgent)) {
        return false;
    }

    int16_t availability = 0;
    uint32_t locator_count = 0u;
    if (!reader.read_i16(availability) ||
        !reader.read_u32(locator_count) ||
        (availability <= 0)) {
        return false;
    }

    for (uint32_t i = 0; (i < locator_count) && (out_count < out_capacity); ++i) {
        rosette::net::SocketAddress candidate{};
        bool supported = false;
        if (!parse_transport_locator(reader, candidate, supported)) {
            // rdata past this point can't be trusted; stop, keep what we have.
            break;
        }

        if (!supported || ((filter != nullptr) && !filter(candidate))) {
            continue;
        }

        out[out_count++] = candidate;
    }

    return out_count > 0u;
}

}  // namespace detail

/**
 * @brief Resolve a micro-ROS Agent with upstream-compatible XRCE discovery.
 *
 * The locator opens a temporary UDP socket, multicasts a `GET_INFO` request to
 * the standard discovery group, and returns the `INFO` reply's locators that
 * pass `MulticastAgentLocatorConfig::address_filter` (an Agent's reply
 * commonly advertises several addresses, only some of which are reachable
 * from here, and can mix IPv4 and IPv6 transports).
 */
class MulticastAgentLocator final : public AgentLocator {
public:
    MulticastAgentLocator(rosette::net::SocketProvider& socket_provider,
                          const MulticastAgentLocatorConfig& config = {}) noexcept
        : socket_provider_(socket_provider)
        , config_(config)
    {}

    size_t resolve(rosette::net::SocketAddress* out,
                   size_t capacity) noexcept override
    {
        if ((out == nullptr) || (capacity == 0u) ||
            (config_.multicast_group == nullptr) ||
            (config_.multicast_group[0] == '\0') ||
            (config_.multicast_port == 0u)) {
            return 0u;
        }

        auto socket = socket_provider_.open(rosette::net::SocketDomain::ipv4,
                                            rosette::net::SocketType::datagram,
                                            rosette::net::SocketProtocol::udp);
        if (!socket) {
            return 0u;
        }

        rosette::net::SocketAddress group{};
        if (!rosette::net::inet::make_socket_address_v4(
                config_.multicast_group, config_.multicast_port, group)) {
            return 0u;
        }

        // Wiznet hardware sockets latch multicast membership into the socket
        // mode register at OPEN time, so this must happen before bind()
        // (which is what triggers the OPEN command), not after.
        const auto membership = rosette::net::make_multicast_membership_v4(
            group.ipv4(), {0u, 0u, 0u, 0u}, config_.multicast_port);
        if (socket->joinMulticastGroup(membership) != 0) {
            return 0u;
        }

        const auto local =
            rosette::net::make_socket_address_v4({0u, 0u, 0u, 0u}, 0u);
        if (socket->bind(local) != 0) {
            return 0u;
        }

        if (socket->set_receive_timeout(config_.receive_timeout_ms) != 0) {
            return 0u;
        }

        const uint8_t attempts = (config_.attempts == 0u) ? 1u : config_.attempts;
        uint8_t probe[detail::kGetInfoPacketSize] = {};
        if (!detail::build_get_info_packet(probe)) {
            return 0u;
        }

        for (uint8_t attempt = 0u; attempt < attempts; ++attempt) {
            if (socket->send_to(probe, sizeof(probe), group) !=
                static_cast<std::ptrdiff_t>(sizeof(probe))) {
                continue;
            }

            uint8_t reply[detail::kMaxDiscoveryPacketSize] = {};
            rosette::net::SocketAddress remote{};
            const std::ptrdiff_t received =
                socket->recv_from(reply, sizeof(reply), &remote);
            if ((received <= 0) || !remote.is_ipv4()) {
                continue;
            }

            size_t count = 0u;
            if (detail::parse_info_packet(reply,
                                          static_cast<size_t>(received),
                                          config_.address_filter,
                                          out,
                                          capacity,
                                          count)) {
                return count;
            }
        }

        return 0u;
    }

private:
    rosette::net::SocketProvider& socket_provider_;
    MulticastAgentLocatorConfig config_{};
};

}  // namespace rosette
