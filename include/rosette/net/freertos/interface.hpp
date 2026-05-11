#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include <utility>

extern "C" {
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Routing.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"
}

#include "rosette/net/freertos/igmp.hpp"
#include "rosette/net/mac.hpp"

namespace rosette::net::freertos {

namespace detail {

template<typename...>
using void_t = void;

template<typename D, typename = void_t<>>
struct has_initialise : std::false_type {};
template<typename D>
struct has_initialise<D,
                      void_t<decltype(std::declval<D&>().initialise(
                          std::declval<const net::MacAddress&>()))>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_register_rx_task : std::false_type {};
template<typename D>
struct has_register_rx_task<D,
                            void_t<decltype(std::declval<D&>().register_rx_task(
                                std::declval<TaskHandle_t>()))>> : std::true_type {};

template<typename D, typename = void_t<>>
struct has_handle_interrupt_from_isr : std::false_type {};
template<typename D>
struct has_handle_interrupt_from_isr<D,
                                     void_t<decltype(std::declval<D&>().handle_interrupt_from_isr())>> : std::true_type {};

template<typename D>
inline bool call_initialise(D& device, const net::MacAddress& mac)
{
    if constexpr (has_initialise<D>::value) {
        return static_cast<bool>(device.initialise(mac));
    } else {
        (void)device;
        (void)mac;
        return true;
    }
}

template<typename D>
inline void call_register_rx_task(D& device, TaskHandle_t task)
{
    if constexpr (has_register_rx_task<D>::value) {
        device.register_rx_task(task);
    } else {
        (void)device;
        (void)task;
    }
}

template<typename D>
inline void call_handle_interrupt_from_isr(D& device) noexcept
{
    if constexpr (has_handle_interrupt_from_isr<D>::value) {
        device.handle_interrupt_from_isr();
    } else {
        (void)device;
    }
}

}  // namespace detail

template<typename Device>
class NetworkInterface {
public:
    using device_type = Device;
    using mac_address_type = net::MacAddress;

    static_assert(net::is_mac_device<device_type>::value,
                  "Device must satisfy MAC data-plane methods");

    explicit NetworkInterface(device_type device,
                              mac_address_type fallback_mac = {}) noexcept
        : device_(std::move(device))
        , mac_address_(fallback_mac) {}

    NetworkInterface_t* fill_interface_descriptor(NetworkInterface_t& interface,
                                                  const char* name)
    {
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
        freertos_multicast_service().configure_output(&igmp_output, this);
        return &interface;
    }

    NetworkInterface_t* add_to_freertos(NetworkInterface_t& interface,
                                        const char* name)
    {
        NetworkInterface_t* filled = fill_interface_descriptor(interface, name);
        FreeRTOS_AddNetworkInterface(filled);
        return filled;
    }

    void set_fallback_mac_address(const mac_address_type& mac) noexcept
    {
        mac_address_ = mac;
        if constexpr (net::detail::has_set_mac_address<device_type>::value) {
            device_.set_mac_address(mac_address_);
        }
    }

    BaseType_t initialise()
    {
        if (interface_ == nullptr) {
            return pdFALSE;
        }

        if (!started_) {
            if (!detail::call_initialise(device_, mac_address_)) {
                return pdFALSE;
            }
            started_ = true;
        } else if constexpr (net::detail::has_set_mac_address<device_type>::value) {
            device_.set_mac_address(mac_address_);
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

            detail::call_register_rx_task(device_, rx_task_handle_);
        }

        return pdTRUE;
    }

    BaseType_t output(NetworkBufferDescriptor_t* pxDescriptor,
                      BaseType_t xReleaseAfterSend)
    {
        BaseType_t ret = pdFALSE;
        const std::size_t max_frame = net::max_frame_size(device_);

        if ((pxDescriptor != nullptr) &&
            (pxDescriptor->pucEthernetBuffer != nullptr) &&
            (pxDescriptor->xDataLength > 0u) &&
            ((max_frame == 0u) || (pxDescriptor->xDataLength <= max_frame))) {
            if (net::detail::call_send_packet(
                    device_, pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength)) {
                ret = pdTRUE;
            }
        }

        iptraceNETWORK_INTERFACE_TRANSMIT();

        if ((xReleaseAfterSend != pdFALSE) && (pxDescriptor != nullptr)) {
            vReleaseNetworkBufferAndDescriptor(pxDescriptor);
        }

        return ret;
    }

    BaseType_t link_status() noexcept
    {
        const net::PhyStatus status = net::get_phy_status(device_);
        return status.link_up ? pdTRUE : pdFALSE;
    }

    void handle_interrupt_from_isr() noexcept
    {
        detail::call_handle_interrupt_from_isr(device_);
    }

    device_type& device() noexcept
    {
        return device_;
    }

private:
    static NetworkInterface* from_interface(NetworkInterface_t* interface)
    {
        return interface == nullptr
            ? nullptr
            : static_cast<NetworkInterface*>(interface->pvArgument);
    }

    static BaseType_t pf_initialise(NetworkInterface_t* interface)
    {
        NetworkInterface* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->initialise();
    }

    static BaseType_t pf_output(NetworkInterface_t* interface,
                                NetworkBufferDescriptor_t* const descriptor,
                                BaseType_t xReleaseAfterSend)
    {
        NetworkInterface* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->output(descriptor, xReleaseAfterSend);
    }

    static BaseType_t pf_get_phy_link_status(NetworkInterface_t* interface)
    {
        NetworkInterface* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->link_status();
    }

    static void pf_add_allowed_mac(NetworkInterface_t* interface,
                                   const uint8_t* mac_address)
    {
        NetworkInterface* self = from_interface(interface);
        if (self != nullptr) {
            self->add_allowed_mac(mac_address);
        }
    }

    static void pf_remove_allowed_mac(NetworkInterface_t* interface,
                                      const uint8_t* mac_address)
    {
        NetworkInterface* self = from_interface(interface);
        if (self != nullptr) {
            self->remove_allowed_mac(mac_address);
        }
    }

    static bool igmp_output(NetworkInterface_t* interface,
                            const uint8_t* frame,
                            std::size_t length,
                            void* context) noexcept
    {
        auto* self = static_cast<NetworkInterface*>(context);
        if ((self == nullptr) || (interface != self->interface_)) {
            return false;
        }

        const std::size_t max_frame = net::max_frame_size(self->device_);
        if ((frame == nullptr) || (length == 0u) ||
            ((max_frame != 0u) && (length > max_frame))) {
            return false;
        }

        return net::detail::call_send_packet(self->device_, frame, length);
    }

    static void rx_task_entry(void* arg)
    {
        static_cast<NetworkInterface*>(arg)->rx_task();
    }

    void set_name(const char* name)
    {
        std::memset(name_.data(), 0, name_.size());
        if (name != nullptr) {
            std::strncpy(name_.data(), name, name_.size() - 1u);
        }
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

    void rx_task()
    {
        for (;;) {
            const std::size_t bytes_received = device_.peek_packet_size();
            if (bytes_received == 0u) {
                (void)ulTaskNotifyTake(pdTRUE, 64);
                continue;
            }

            NetworkBufferDescriptor_t* pxDescriptor =
                pxGetNetworkBufferWithDescriptor(bytes_received, 0);
            if (pxDescriptor == nullptr) {
                iptraceETHERNET_RX_EVENT_LOST();
                continue;
            }

            const std::size_t bytes_read =
                device_.receive_packet(pxDescriptor->pucEthernetBuffer, bytes_received);
            pxDescriptor->xDataLength = bytes_read;
            pxDescriptor->pxInterface = interface_;

            freertos_multicast_service().process_frame(
                pxDescriptor->pucEthernetBuffer,
                pxDescriptor->xDataLength,
                interface_);

            pxDescriptor->pxEndPoint =
                FreeRTOS_MatchingEndpoint(interface_, pxDescriptor->pucEthernetBuffer);

            if ((pxDescriptor->pxEndPoint == nullptr) || (bytes_read == 0u)) {
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

private:
    device_type device_;
    NetworkInterface_t* interface_ = nullptr;
    TaskHandle_t rx_task_handle_ = nullptr;
    std::array<char, 16> name_{};
    mac_address_type mac_address_{};
    UBaseType_t multicast_accept_refcount_ = 0u;
    bool started_ = false;
};

}  // namespace rosette::net::freertos
