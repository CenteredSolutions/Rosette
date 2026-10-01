#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

extern "C" {
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Routing.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"
}

#include "freertos_tcp_igmp.hpp"
#include "rosette/net/mac.hpp"

namespace rosette::net::freertos {

/**
 * @brief net::MacDevice extended with the two hooks the FreeRTOS RX path uses.
 *
 * NetworkInterface drives frame reception from a dedicated task. A driver
 * that can raise an interrupt when a frame arrives overrides
 * register_rx_task() to capture the task handle and
 * handle_interrupt_from_isr() to notify it, turning the RX loop from a poll
 * into an event wait. A driver with no interrupt line inherits the default
 * no-ops and the task polls on a short timeout.
 */
class FreeRtosMacDevice : public net::MacDevice {
public:
    /**
     * @brief Record the task to notify when a received frame is queued.
     *
     * @param[in] task Handle of the owning NetworkInterface's RX task.
     */
    virtual void register_rx_task(TaskHandle_t task) noexcept { (void)task; }

    /**
     * @brief Service the controller's frame-ready interrupt.
     *
     * Called from the board's Ethernet ISR trampoline. Implementations clear
     * the controller interrupt source and notify the registered RX task.
     */
    virtual void handle_interrupt_from_isr() noexcept {}
};

/**
 * @brief Binds a FreeRtosMacDevice to a FreeRTOS+TCP NetworkInterface_t.
 *
 * This is the glue between the driver-neutral MAC contract and the C
 * NetworkInterface_t descriptor that FreeRTOS+TCP registers and calls back
 * through. It owns the RX task, translates the C callback ABI into method
 * calls on the referenced device, and feeds received frames into the IP
 * task after the multicast service has inspected them.
 *
 * One instance backs one hardware interface for the lifetime of the stack.
 * The referenced device must outlive the NetworkInterface; both are
 * typically file-scope objects in the board's link translation unit.
 */
class NetworkInterface {
public:
    using mac_address_type = net::MacAddress;

    /**
     * @brief Construct the binding around @p device.
     *
     * @param[in] device       MAC driver to carry this interface's frames;
     *                         must outlive the NetworkInterface.
     * @param[in] fallback_mac Station address used until FreeRTOS+TCP
     *                         supplies an endpoint MAC.
     */
    explicit NetworkInterface(FreeRtosMacDevice& device,
                              mac_address_type fallback_mac = {}) noexcept
        : device_(device)
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
        device_.set_mac_address(mac_address_);
    }

    BaseType_t initialise()
    {
        if (interface_ == nullptr) {
            return pdFALSE;
        }

        if (!started_) {
            if (!device_.initialise(mac_address_)) {
                return pdFALSE;
            }
            started_ = true;
        } else {
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

            device_.register_rx_task(rx_task_handle_);
        }

        return pdTRUE;
    }

    BaseType_t output(NetworkBufferDescriptor_t* pxDescriptor,
                      BaseType_t xReleaseAfterSend)
    {
        BaseType_t ret = pdFALSE;
        const std::size_t max_frame = device_.max_frame_size();

        if ((pxDescriptor != nullptr) &&
            (pxDescriptor->pucEthernetBuffer != nullptr) &&
            (pxDescriptor->xDataLength > 0u) &&
            ((max_frame == 0u) || (pxDescriptor->xDataLength <= max_frame))) {
            if (device_.send_frame(pxDescriptor->pucEthernetBuffer,
                                   pxDescriptor->xDataLength)) {
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
        return device_.phy_status().link_up ? pdTRUE : pdFALSE;
    }

    void handle_interrupt_from_isr() noexcept
    {
        device_.handle_interrupt_from_isr();
    }

    FreeRtosMacDevice& device() noexcept
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

        const std::size_t max_frame = self->device_.max_frame_size();
        if ((frame == nullptr) || (length == 0u) ||
            ((max_frame != 0u) && (length > max_frame))) {
            return false;
        }

        return self->device_.send_frame(frame, length);
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
            const std::size_t bytes_received = device_.peek_frame_size();
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
                device_.receive_frame(pxDescriptor->pucEthernetBuffer, bytes_received);
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
    FreeRtosMacDevice& device_;
    NetworkInterface_t* interface_ = nullptr;
    TaskHandle_t rx_task_handle_ = nullptr;
    std::array<char, 16> name_{};
    mac_address_type mac_address_{};
    UBaseType_t multicast_accept_refcount_ = 0u;
    bool started_ = false;
};

}  // namespace rosette::net::freertos
