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

#include "freertos_tcp_interface.hpp"
#include "rosette/net/mac.hpp"
#include "rosette/net/wiznet/device.hpp"

namespace rosette::net::wiznet {

/**
 * @brief WIZnet MACRAW driver presented as a FreeRtosMacDevice.
 *
 * Wraps the register-level WiznetDevice in the whole-frame contract that
 * rosette::net::freertos::NetworkInterface consumes. Socket 0 of the
 * controller is held open in MACRAW mode, so every send_frame()/receive_frame()
 * call maps to a single SPI burst against that socket's TX/RX ring. The chip's
 * hardware interrupt is routed through handle_interrupt_from_isr(), which
 * notifies the interface's RX task registered via register_rx_task(); the
 * task clears the socket's RECV interrupt as it reads (WiznetDevice::read_frame()).
 *
 * The device is templated on the concrete @p Chip register map and @p Bus
 * transport so the board's link translation unit can pin a single hardware
 * combination while the register-level accesses stay statically dispatched.
 * Only the MacDevice contract that NetworkInterface calls is virtual. The
 * object is constructed once and referenced for the lifetime of the stack;
 * it is neither copied nor moved, because the embedded WiznetDevice holds a
 * reference to @c bus_.
 *
 * @tparam Chip Compile-time register map: addresses, socket assignment, and
 *              buffer sizes for the target WIZnet controller.
 * @tparam Bus  SPI transport providing scoped bus locking via acquire().
 */
template<typename Chip, typename Bus>
class WiznetMacrawDevice : public rosette::net::freertos::FreeRtosMacDevice {
public:
    using chip_type = Chip;
    using bus_type = Bus;
    using raw_device_type = WiznetDevice<Chip, Bus>;
    using mac_address_type = net::MacAddress;

    explicit WiznetMacrawDevice(Bus bus) noexcept
        : bus_(std::move(bus))
        , device_(bus_) {}

    bool initialise(const mac_address_type& mac) noexcept override
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

    void set_mac_address(const mac_address_type& mac) noexcept override
    {
        device_.set_mac_address(mac);
    }

    std::size_t max_frame_size() const noexcept override
    {
        return static_cast<std::size_t>(Chip::max_frame_bytes());
    }

    bool send_frame(const uint8_t* data, std::size_t length) noexcept override
    {
        return device_.write_frame(data, length);
    }

    std::size_t receive_frame(uint8_t* buffer, std::size_t capacity) noexcept override
    {
        return device_.read_frame(buffer, capacity);
    }

    std::size_t peek_frame_size() noexcept override
    {
        return static_cast<std::size_t>(device_.socket_rx_available(Chip::macraw_socket()));
    }

    net::PhyStatus phy_status() noexcept override
    {
        net::PhyStatus status{};
        status.link_up = device_.is_link_up();
        return status;
    }

    /// Notifies the RX task; runs in interrupt context and touches no bus.
    void handle_interrupt_from_isr() noexcept override
    {
        BaseType_t should_yield = pdFALSE;
        TaskHandle_t const task = rx_task_;
        if (task != nullptr) {
            vTaskNotifyGiveFromISR(task, &should_yield);
        }

        portYIELD_FROM_ISR(should_yield);
    }

    void register_rx_task(TaskHandle_t task) noexcept override { rx_task_ = task; }

private:
    Bus bus_;
    raw_device_type device_;
    TaskHandle_t volatile rx_task_ = nullptr;
    bool started_ = false;
};

}  // namespace rosette::net::wiznet

#endif
