#pragma once

#include <stdint.h>
#include <stdbool.h>

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

#ifdef __cplusplus
}

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>

#include "FreeRTOS_Routing.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"

#include "net/freertos/igmp.hpp"
#include "device.hpp"

namespace milkdds::net::wiznet {

template<typename Chip, typename Bus>
class MacrawNetworkInterface {
public:
    using device_type = milkdds::net::wiznet::WiznetDevice<Chip, Bus>;
    using InvalidChipVersion = typename device_type::InvalidChipVersion;

    explicit MacrawNetworkInterface(
        Bus bus,
        std::array<uint8_t, 6> fallback_mac = {}) noexcept
        : bus_(std::move(bus))
        , device_(bus_)
        , mac_address_(fallback_mac) {}

    NetworkInterface_t* fill_interface_descriptor(NetworkInterface_t& interface,
                                                  const char* name) {
        std::memset(&interface, 0, sizeof(interface));
        set_name(name);

        interface_ = &interface;
        interface.pcName = name_.data();
        interface.pvArgument = this;
        interface.pfInitialise = &pf_initialise;
        interface.pfOutput = &pf_output;
        interface.pfGetPhyLinkStatus = &pf_get_phy_link_status;
        interface.pfAddAllowedMAC = &pf_add_allowed_mac;
        interface.pfRemoveAllowedMAC = &pf_remove_allowed_mac;
        milkdds::net::freertos_multicast_service().configure_output(&igmp_output, this);
        return &interface;
    }

    NetworkInterface_t* add_to_freertos(NetworkInterface_t& interface,
                                        const char* name) {
        NetworkInterface_t* filled = fill_interface_descriptor(interface, name);
        FreeRTOS_AddNetworkInterface(filled);
        return filled;
    }

    void set_fallback_mac_address(const std::array<uint8_t, 6>& mac) noexcept
    {
        mac_address_ = mac;
        device_.set_mac_address(mac_address_);
    }

    BaseType_t initialise() {
        if (interface_ == nullptr) {
            return pdFALSE;
        }

        set_runtime_mac_address();

        if (!started_) {
            if (!device_.verify()) {
                return pdFALSE;
            }

            auto result = device_.init_macraw(mac_address_);
            if (auto* err = std::get_if<InvalidChipVersion>(&result)) {
                // Handle the error: log, return failure, etc.
                printf("[wiznet] Invalid chip version: %u (expected %u)\n", err->actual, err->expected);
                return pdFALSE;
            }
            started_ = true;
        }

        if (rx_task_handle_ == nullptr) {
            char task_name[16] = {};
            std::snprintf(task_name, sizeof(task_name), "%.12s_rx", name_.data());

            const BaseType_t ret = xTaskCreate(
                &rx_task_entry,
                task_name,
                512u,
                this,
                (configMAX_PRIORITIES - 1),
                &rx_task_handle_);

            if (ret != pdPASS) {
                return pdFALSE;
            }

            device_.register_socket_rx_task(Chip::macraw_socket(), rx_task_handle_);
        }

        return pdTRUE;
    }

    BaseType_t output(NetworkBufferDescriptor_t* pxDescriptor,
                      BaseType_t xReleaseAfterSend) {
        BaseType_t ret = pdFALSE;

        if ((pxDescriptor != nullptr) &&
            (pxDescriptor->pucEthernetBuffer != nullptr) &&
            (pxDescriptor->xDataLength > 0u) &&
            (pxDescriptor->xDataLength <= Chip::max_frame_bytes())) {
            if (device_.write_frame(pxDescriptor->pucEthernetBuffer,
                                    static_cast<uint16_t>(pxDescriptor->xDataLength))) {
                ret = pdTRUE;
            }
        }

        iptraceNETWORK_INTERFACE_TRANSMIT();

        if ((xReleaseAfterSend != pdFALSE) && (pxDescriptor != nullptr)) {
            vReleaseNetworkBufferAndDescriptor(pxDescriptor);
        }

        return ret;
    }

    BaseType_t link_status() {
        return device_.is_link_up() ? pdTRUE : pdFALSE;
    }

    void handle_interrupt_from_isr() noexcept
    {
        device_.handle_interrupt_from_isr();
    }

private:
    static MacrawNetworkInterface* from_interface(NetworkInterface_t* interface) {
        return interface == nullptr
            ? nullptr
            : static_cast<MacrawNetworkInterface*>(interface->pvArgument);
    }

    static BaseType_t pf_initialise(NetworkInterface_t* interface) {
        MacrawNetworkInterface* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->initialise();
    }

    static BaseType_t pf_output(NetworkInterface_t* interface,
                                NetworkBufferDescriptor_t* const descriptor,
                                BaseType_t xReleaseAfterSend) {
        MacrawNetworkInterface* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->output(descriptor, xReleaseAfterSend);
    }

    static BaseType_t pf_get_phy_link_status(NetworkInterface_t* interface) {
        MacrawNetworkInterface* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->link_status();
    }

    static void pf_add_allowed_mac(NetworkInterface_t* interface,
                                   const uint8_t* mac_address) {
        MacrawNetworkInterface* self = from_interface(interface);
        if (self != nullptr) {
            self->add_allowed_mac(mac_address);
        }
    }

    static void pf_remove_allowed_mac(NetworkInterface_t* interface,
                                      const uint8_t* mac_address) {
        MacrawNetworkInterface* self = from_interface(interface);
        if (self != nullptr) {
            self->remove_allowed_mac(mac_address);
        }
    }

    static bool igmp_output(NetworkInterface_t* interface,
                            const uint8_t* frame,
                            std::size_t length,
                            void* context) noexcept {
        auto* self = static_cast<MacrawNetworkInterface*>(context);
        if ((self == nullptr) || (interface != self->interface_)) {
            return false;
        }

        return self->send_raw_frame(frame, length);
    }

    static void rx_task_entry(void* arg) {
        static_cast<MacrawNetworkInterface*>(arg)->rx_task();
    }

    void set_name(const char* name) {
        std::memset(name_.data(), 0, name_.size());
        if (name != nullptr) {
            std::strncpy(name_.data(), name, name_.size() - 1u);
        }
    }

    void set_runtime_mac_address() {
        device_.set_mac_address(mac_address_);
    }

    void add_allowed_mac(const uint8_t* mac_address) noexcept
    {
        if ((mac_address != nullptr) && ((mac_address[0] & 0x01u) != 0u)) {
            ++multicast_accept_refcount_;
        }
    }

    void remove_allowed_mac(const uint8_t* mac_address) noexcept
    {
        if ((mac_address != nullptr) &&
            ((mac_address[0] & 0x01u) != 0u) &&
            (multicast_accept_refcount_ > 0u)) {
            --multicast_accept_refcount_;
        }
    }

    bool send_raw_frame(const uint8_t* frame, std::size_t length) noexcept
    {
        if ((frame == nullptr) || (length == 0u) || (length > Chip::max_frame_bytes())) {
            return false;
        }

        return device_.write_frame(frame, static_cast<uint16_t>(length));
    }

    void rx_task() {
        for (;;) {
            const uint16_t xBytesReceived = device_.socket_rx_available(Chip::macraw_socket());
            if (xBytesReceived == 0) {
                (void)ulTaskNotifyTake(pdTRUE, 64);
                continue;
            }

            NetworkBufferDescriptor_t* pxDescriptor =
                pxGetNetworkBufferWithDescriptor(static_cast<size_t>(xBytesReceived), 0);
            if (pxDescriptor == nullptr) {
                iptraceETHERNET_RX_EVENT_LOST();
                continue;
            }

            uint16_t xBytesRead = device_.read_frame(pxDescriptor->pucEthernetBuffer, xBytesReceived);
            pxDescriptor->xDataLength = static_cast<size_t>(xBytesRead);
            pxDescriptor->pxInterface = interface_;
            milkdds::net::freertos_multicast_service().process_frame(
                pxDescriptor->pucEthernetBuffer,
                pxDescriptor->xDataLength,
                interface_);
            pxDescriptor->pxEndPoint =
                FreeRTOS_MatchingEndpoint(interface_, pxDescriptor->pucEthernetBuffer);

            if (pxDescriptor->pxEndPoint == nullptr) {
                vReleaseNetworkBufferAndDescriptor(pxDescriptor);
                continue;
            }

            IPStackEvent_t event{};
            event.eEventType = eNetworkRxEvent;
            event.pvData = pxDescriptor;

            if (xSendEventStructToIPTask(&event, 0) != pdTRUE) {
                vReleaseNetworkBufferAndDescriptor(pxDescriptor);
                iptraceETHERNET_RX_EVENT_LOST();
            } else {
                iptraceNETWORK_INTERFACE_RECEIVE();
            }
        }
    }

    Bus bus_;
    device_type device_;
    NetworkInterface_t* interface_ = nullptr;
    TaskHandle_t rx_task_handle_ = nullptr;
    std::array<char, 16> name_{};
    std::array<uint8_t, 6> mac_address_{};
    UBaseType_t multicast_accept_refcount_ = 0u;
    bool started_ = false;
};

}  // namespace milkdds::net::wiznet

#endif
