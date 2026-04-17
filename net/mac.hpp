#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace milkdds::net {

using MacAddress = std::array<uint8_t, 6>;

enum class PhySpeed : uint8_t {
    unknown = 0u,
    mbps10,
    mbps100,
    mbps1000,
};

enum class PhyDuplex : uint8_t {
    unknown = 0u,
    half,
    full,
};

struct PhyConfig {
    bool auto_negotiation = true;
    PhySpeed speed = PhySpeed::unknown;
    PhyDuplex duplex = PhyDuplex::unknown;
    bool power_down = false;
};

struct PhyStatus {
    bool link_up = false;
    bool auto_negotiation_complete = false;
    PhySpeed speed = PhySpeed::unknown;
    PhyDuplex duplex = PhyDuplex::unknown;
};

struct PacketBuffer {
    uint8_t* data = nullptr;
    std::size_t size = 0u;

    explicit operator bool() const noexcept
    {
        return (data != nullptr) && (size > 0u);
    }
};

namespace detail {

template<typename...>
using void_t = void;

template<typename D, typename = void_t<>>
struct has_send_packet : std::false_type {};
template<typename D>
struct has_send_packet<D,
                       void_t<decltype(std::declval<D&>().send_packet(
                           static_cast<const uint8_t*>(nullptr),
                           std::declval<std::size_t>()))>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_receive_packet : std::false_type {};
template<typename D>
struct has_receive_packet<D,
                          void_t<decltype(std::declval<D&>().receive_packet(
                              static_cast<uint8_t*>(nullptr),
                              std::declval<std::size_t>()))>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_peek_packet_size : std::false_type {};
template<typename D>
struct has_peek_packet_size<D,
                            void_t<decltype(std::declval<D&>().peek_packet_size())>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_available_packets : std::false_type {};
template<typename D>
struct has_available_packets<D,
                             void_t<decltype(std::declval<D&>().available_packets())>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_set_mac_address : std::false_type {};
template<typename D>
struct has_set_mac_address<D,
                           void_t<decltype(std::declval<D&>().set_mac_address(
                               std::declval<const MacAddress&>()))>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_get_mac_address : std::false_type {};
template<typename D>
struct has_get_mac_address<D,
                           void_t<decltype(std::declval<D&>().get_mac_address())>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_set_phy_config : std::false_type {};
template<typename D>
struct has_set_phy_config<D,
                          void_t<decltype(std::declval<D&>().set_phy_config(
                              std::declval<const PhyConfig&>()))>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_get_phy_config : std::false_type {};
template<typename D>
struct has_get_phy_config<D,
                          void_t<decltype(std::declval<D&>().get_phy_config())>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_get_phy_status : std::false_type {};
template<typename D>
struct has_get_phy_status<D,
                          void_t<decltype(std::declval<D&>().get_phy_status())>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_max_frame_size : std::false_type {};
template<typename D>
struct has_max_frame_size<D,
                          void_t<decltype(std::declval<const D&>().max_frame_size())>> : std::true_type {};

template<typename D>
inline bool call_set_phy_config(D& device, const PhyConfig& config)
{
    if constexpr (has_set_phy_config<D>::value) {
        return static_cast<bool>(device.set_phy_config(config));
    } else {
        (void)device;
        (void)config;
        return false;
    }
}

template<typename D>
inline bool call_send_packet(D& device, const uint8_t* data, std::size_t length)
{
    if constexpr (std::is_void<decltype(device.send_packet(data, length))>::value) {
        device.send_packet(data, length);
        return true;
    } else {
        return static_cast<bool>(device.send_packet(data, length));
    }
}

template<typename D>
inline PhyConfig call_get_phy_config(D& device)
{
    if constexpr (has_get_phy_config<D>::value) {
        return device.get_phy_config();
    } else {
        (void)device;
        return {};
    }
}

template<typename D>
inline PhyStatus call_get_phy_status(D& device)
{
    if constexpr (has_get_phy_status<D>::value) {
        return device.get_phy_status();
    } else {
        (void)device;
        return {};
    }
}

template<typename D>
inline std::size_t call_max_frame_size(const D& device)
{
    if constexpr (has_max_frame_size<D>::value) {
        return static_cast<std::size_t>(device.max_frame_size());
    } else {
        (void)device;
        return 0u;
    }
}

}  // namespace detail

template<typename Device>
struct is_mac_device
    : std::integral_constant<bool,
                             detail::has_send_packet<Device>::value &&
                                 detail::has_receive_packet<Device>::value &&
                                 detail::has_peek_packet_size<Device>::value> {};

template<typename Device, typename Callback>
std::size_t receive_packet_zero_copy(Device& device, Callback&& callback)
{
    static_assert(is_mac_device<Device>::value,
                  "Device must satisfy the MAC data-plane contract");

    const std::size_t receive_size = device.peek_packet_size();
    if (receive_size == 0u) {
        return 0u;
    }

    PacketBuffer buffer = std::forward<Callback>(callback)(receive_size);
    if (!buffer) {
        return 0u;
    }

    return device.receive_packet(buffer.data, buffer.size);
}

template<typename Device>
inline bool set_phy_config(Device& device, const PhyConfig& config) noexcept
{
    return detail::call_set_phy_config(device, config);
}

template<typename Device>
inline PhyConfig get_phy_config(Device& device) noexcept
{
    return detail::call_get_phy_config(device);
}

template<typename Device>
inline PhyStatus get_phy_status(Device& device) noexcept
{
    return detail::call_get_phy_status(device);
}

template<typename Device>
inline std::size_t max_frame_size(const Device& device) noexcept
{
    return detail::call_max_frame_size(device);
}

}  // namespace milkdds::net
