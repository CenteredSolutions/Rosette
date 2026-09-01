#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS_IP.h"
#include "NetworkInterface.h"

#ifdef __cplusplus
extern "C" {
#endif

void wiznet_macraw_init(void);
BaseType_t xNetworkInterfaceInitialise(void);
BaseType_t xNetworkInterfaceOutput(NetworkBufferDescriptor_t* pxDescriptor,
                                   BaseType_t xReleaseAfterSend);
bool wiznet_macraw_link_status(void);
void wiznet_macraw_interrupt_from_isr(void);
void wiznet_macraw_set_mac_address(const uint8_t mac[6]);

#ifdef __cplusplus
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <variant>

#include "rosette/net/freertos/interface.hpp"
#include "rosette/net/mac.hpp"
#include "device.hpp"

namespace rosette::net::wiznet {

template<typename Chip, typename Bus>
class WiznetMacrawDevice {
public:
    using chip_type = Chip;
    using bus_type = Bus;
    using raw_device_type = WiznetDevice<Chip, Bus>;
    using mac_address_type = net::MacAddress;

    explicit WiznetMacrawDevice(Bus bus) noexcept
        : bus_(std::move(bus))
        , device_(bus_) {}

    bool initialise(const mac_address_type& mac) noexcept
    {
        if (!started_) {
            if (!device_.verify()) {
                return false;
            }

            auto result = device_.init_macraw(mac);
            if (auto* err = std::get_if<typename raw_device_type::InvalidChipVersion>(&result)) {
                std::printf("[wiznet] Invalid chip version: %u (expected %u)\n",
                            err->actual,
                            err->expected);
                return false;
            }
            started_ = true;
        }

        device_.set_mac_address(mac);
        return true;
    }

    void set_mac_address(const mac_address_type& mac) noexcept
    {
        device_.set_mac_address(mac);
    }

    mac_address_type get_mac_address() noexcept
    {
        return device_.mac_address();
    }

    std::size_t max_frame_size() const noexcept
    {
        return static_cast<std::size_t>(Chip::max_frame_bytes());
    }

    bool send_packet(const uint8_t* data, std::size_t length) noexcept
    {
        return device_.write_frame(data, length);
    }

    std::size_t receive_packet(uint8_t* buffer, std::size_t capacity) noexcept
    {
        return device_.read_frame(buffer, capacity);
    }

    std::size_t peek_packet_size() noexcept
    {
        return static_cast<std::size_t>(device_.socket_rx_available(Chip::macraw_socket()));
    }

    std::size_t available_packets() noexcept
    {
        return peek_packet_size() > 0u ? 1u : 0u;
    }

    net::PhyStatus get_phy_status() noexcept
    {
        net::PhyStatus status{};
        status.link_up = device_.is_link_up();
        return status;
    }

    bool set_phy_config(const net::PhyConfig& config) noexcept
    {
        (void)config;
        return false;
    }

    net::PhyConfig get_phy_config() noexcept
    {
        return {};
    }

    void handle_interrupt_from_isr() noexcept
    {
#ifndef ROSETTE_NO_FREERTOS
        device_.handle_interrupt_from_isr();
#endif
    }

#ifndef ROSETTE_NO_FREERTOS
    void register_rx_task(TaskHandle_t task) noexcept
    {
        device_.register_socket_rx_task(Chip::macraw_socket(), task);
    }
#endif

private:
    Bus bus_;
    raw_device_type device_;
    bool started_ = false;
};

template<typename Chip, typename Bus>
using MacrawNetworkInterface =
    rosette::net::freertos::NetworkInterface<WiznetMacrawDevice<Chip, Bus>>;

}  // namespace rosette::net::wiznet

#endif
