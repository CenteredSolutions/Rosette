#pragma once

/**
 * @file ports/net/freertos_tcp/freertos_tcp_interface.hpp
 * @brief A FreeRTOS+TCP network interface over any rosette::net::MacDevice.
 *
 * FreeRTOS+TCP reaches hardware through a C descriptor, NetworkInterface_t,
 * whose callbacks initialise the MAC, transmit a buffer, and report the link.
 * NetworkInterface<Device> fills that descriptor and forwards each callback
 * to a MAC provider it owns, and runs the receive task that feeds frames to
 * the IP task. The provider is anything that implements the neutral contract
 * in rosette/net/mac.hpp, so this port knows no chip, and no chip driver
 * knows FreeRTOS+TCP.
 *
 * Receive wake-up goes through the platform contract: the board attaches the
 * controller's interrupt pin (attach_rx_interrupt()), whose falling edge
 * signals a platform::Event the receive task sleeps on. Without a pin the
 * task polls the device every kRxPollMs.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include <utility>

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Routing.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"
}

#include "freertos_tcp_igmp.hpp"
#include "rosette/net/mac.hpp"
#include "rosette/platform/gpio_interrupt.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::net::freertos {

/**
 * @brief Binds a MAC provider to a FreeRTOS+TCP NetworkInterface_t.
 *
 * This is the glue between the driver-neutral MAC contract and the C
 * descriptor that FreeRTOS+TCP registers and calls back through. It owns the
 * provider, the receive task, and the event that task sleeps on; it
 * translates the C callback ABI into calls on the provider, and feeds
 * received frames into the IP task after the multicast service has inspected
 * them.
 *
 * One instance backs one hardware interface for the lifetime of the stack,
 * typically as a file-scope object in the board's composition. It is neither
 * copied nor moved, because the descriptor and the interrupt hold its
 * address.
 *
 * @tparam Device Concrete MAC provider, derived from net::MacDevice and
 *                constructed in place from the interface's constructor
 *                arguments.
 */
template <typename Device>
class NetworkInterface {
    static_assert(std::is_base_of_v<net::MacDevice, Device>,
                  "NetworkInterface<Device>: Device must implement rosette::net::MacDevice");

public:
    using device_type = Device;
    using mac_address_type = net::MacAddress;

    /// Receive-task sleep between device polls when no interrupt pin is attached.
    static constexpr uint32_t kRxPollMs = 64u;

    /// Receive-task sleep with an interrupt pin attached; bounds the cost of a lost edge.
    static constexpr uint32_t kRxIdleWaitMs = 1000u;

    /// Receive-task pause when FreeRTOS+TCP has no network buffer free, so the IP task can return
    /// one.
    static constexpr uint32_t kRxBufferRetryMs = 2u;

    /**
     * @brief Constructs the provider in place; nothing touches hardware until FreeRTOS+TCP
     * initialises the interface.
     *
     * @param[in] station_mac Station address programmed by initialise().
     * @param[in] device_args Forwarded to Device's constructor, e.g. the bus.
     */
    template <typename... DeviceArgs>
    explicit NetworkInterface(const mac_address_type& station_mac,
                              DeviceArgs&&... device_args) noexcept
        : device_(std::forward<DeviceArgs>(device_args)...), mac_address_(station_mac) {}

    NetworkInterface(const NetworkInterface&) = delete;
    NetworkInterface& operator=(const NetworkInterface&) = delete;

    /**
     * @brief Fills @p interface so FreeRTOS+TCP calls back into this object.
     *
     * @param[out] interface Descriptor to fill; must outlive this object's use by the stack.
     * @param[in]  name      Interface name, truncated to 15 characters, copied.
     * @return @p interface, ready for FreeRTOS_AddNetworkInterface().
     */
    NetworkInterface_t* fill_interface_descriptor(NetworkInterface_t& interface, const char* name) {
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

    /// @brief fill_interface_descriptor(), then registers the descriptor with FreeRTOS+TCP.
    NetworkInterface_t* add_to_freertos(NetworkInterface_t& interface, const char* name) {
        NetworkInterface_t* filled = fill_interface_descriptor(interface, name);
        FreeRTOS_AddNetworkInterface(filled);
        return filled;
    }

    /**
     * @brief Wakes the receive task on each falling edge of @p pin.
     *
     * The pin is the MAC controller's active-low interrupt output, a board
     * fact the provider does not know. Attach it once, at any time; an edge
     * before the receive task starts leaves the event pending, so no frame
     * waits for a poll.
     *
     * @param[in] pin GPIO wired to the controller's interrupt output.
     * @return Whether the interrupt is attached; on false the receive task
     *   keeps polling every kRxPollMs.
     *
     * Verification:
     * - Correct means: with the pin attached, a frame sent to the board
     *   reaches the IP task within interrupt latency rather than kRxPollMs.
     * - Covered by: no automated test; a host FreeRTOS+TCP harness would
     *   signal the edge from a fake MacDevice and time the delivery.
     */
    bool attach_rx_interrupt(uint32_t pin) noexcept {
        rx_interrupt_attached_ = rosette::platform::attach_falling_edge(
            pin,
            [](void* self) { static_cast<NetworkInterface*>(self)->rx_event_.signal_from_isr(); },
            this);
        return rx_interrupt_attached_;
    }

    /// @brief Sets the station address the next initialise() programs; touches no hardware.
    void set_station_mac_address(const mac_address_type& mac) noexcept { mac_address_ = mac; }

    /**
     * @brief pfInitialise: brings the provider up and starts the receive task once.
     *
     * FreeRTOS+TCP calls this from the IP task at start-up and again after a
     * link-down, until it returns pdTRUE.
     *
     * @return pdTRUE when the provider carries frames and the receive task
     *   runs; pdFALSE when the descriptor is not filled, the provider failed
     *   to initialise, or the task could not be created (the stack retries).
     */
    BaseType_t initialise() {
        if ((interface_ == nullptr) || !device_.initialise(mac_address_)) {
            return pdFALSE;
        }

        if (rx_task_handle_ == nullptr) {
            char task_name[16] = {};
            std::snprintf(task_name, sizeof(task_name), "%.12s_rx", name_.data());
            const BaseType_t ret = xTaskCreate(&rx_task_entry, task_name, 512u, this,
                                               (configMAX_PRIORITIES - 1), &rx_task_handle_);
            if (ret != pdPASS) {
                return pdFALSE;
            }
        }

        return pdTRUE;
    }

    /**
     * @brief pfOutput: transmits one descriptor's frame and releases it when asked.
     *
     * @param[in] pxDescriptor      Frame to send; rejected when empty or longer than
     * max_frame_size().
     * @param[in] xReleaseAfterSend Non-zero when this call owns, and must release, @p pxDescriptor.
     * @return pdTRUE when the provider accepted the frame; pdFALSE otherwise.
     */
    BaseType_t output(NetworkBufferDescriptor_t* pxDescriptor, BaseType_t xReleaseAfterSend) {
        BaseType_t ret = pdFALSE;
        if ((pxDescriptor != nullptr)
            && send_checked(pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength)) {
            ret = pdTRUE;
        }

        iptraceNETWORK_INTERFACE_TRANSMIT();

        if ((xReleaseAfterSend != pdFALSE) && (pxDescriptor != nullptr)) {
            vReleaseNetworkBufferAndDescriptor(pxDescriptor);
        }

        return ret;
    }

    /// @brief pfGetPhyLinkStatus: pdTRUE while the provider reports carrier.
    BaseType_t link_status() noexcept { return device_.phy_status().link_up ? pdTRUE : pdFALSE; }

    /// @brief The owned MAC provider.
    Device& device() noexcept { return device_; }

private:
    static NetworkInterface* from_interface(NetworkInterface_t* interface) {
        return interface == nullptr ? nullptr
                                    : static_cast<NetworkInterface*>(interface->pvArgument);
    }

    static BaseType_t pf_initialise(NetworkInterface_t* interface) {
        NetworkInterface* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->initialise();
    }

    static BaseType_t pf_output(NetworkInterface_t* interface,
                                NetworkBufferDescriptor_t* const descriptor,
                                BaseType_t xReleaseAfterSend) {
        NetworkInterface* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->output(descriptor, xReleaseAfterSend);
    }

    static BaseType_t pf_get_phy_link_status(NetworkInterface_t* interface) {
        NetworkInterface* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->link_status();
    }

    static void pf_add_allowed_mac(NetworkInterface_t* interface, const uint8_t* mac_address) {
        NetworkInterface* self = from_interface(interface);
        if (self != nullptr) {
            self->add_allowed_mac(mac_address);
        }
    }

    static void pf_remove_allowed_mac(NetworkInterface_t* interface, const uint8_t* mac_address) {
        NetworkInterface* self = from_interface(interface);
        if (self != nullptr) {
            self->remove_allowed_mac(mac_address);
        }
    }

    static bool igmp_output(NetworkInterface_t* interface, const uint8_t* frame, std::size_t length,
                            void* context) noexcept {
        auto* self = static_cast<NetworkInterface*>(context);
        if ((self == nullptr) || (interface != self->interface_)) {
            return false;
        }

        return self->send_checked(frame, length);
    }

    static void rx_task_entry(void* arg) { static_cast<NetworkInterface*>(arg)->rx_task(); }

    // Hands a frame to the provider when it is non-empty and within
    // max_frame_size(), the precondition MacDevice::send_frame() relies on.
    bool send_checked(const uint8_t* frame, std::size_t length) noexcept {
        const std::size_t max_frame = device_.max_frame_size();
        if ((frame == nullptr) || (length == 0u) || ((max_frame != 0u) && (length > max_frame))) {
            return false;
        }

        return device_.send_frame(frame, length);
    }

    void set_name(const char* name) {
        std::memset(name_.data(), 0, name_.size());
        if (name != nullptr) {
            std::strncpy(name_.data(), name, name_.size() - 1u);
        }
    }

    void add_allowed_mac(const uint8_t* mac_address) noexcept {
        if ((mac_address != nullptr) && ((mac_address[0] & 0x01u) != 0u)) {
            ++multicast_accept_refcount_;
        }
    }

    void remove_allowed_mac(const uint8_t* mac_address) noexcept {
        if ((mac_address != nullptr) && ((mac_address[0] & 0x01u) != 0u)
            && (multicast_accept_refcount_ > 0u)) {
            --multicast_accept_refcount_;
        }
    }

    /**
     * @brief Receive task: drains the provider into the IP task, then sleeps until the next frame.
     *
     * Each pass sizes a network buffer from peek_frame_size(), copies the
     * head frame into it, lets the multicast service see it, matches it to an
     * endpoint, and posts eNetworkRxEvent. When the queue is empty the task
     * sleeps on rx_event_ (MacDevice's receive-notification rule guarantees
     * an edge for any frame queued after the last receive). When FreeRTOS+TCP
     * has no buffer free, the frame stays queued and the task pauses for
     * kRxBufferRetryMs, so the lower-priority IP task can release one.
     *
     * Verification:
     * - Correct means: every frame the provider queues is posted to the IP
     *   task exactly once or released, and the task never spins without
     *   blocking.
     * - Covered by: no automated test (needs a host FreeRTOS+TCP harness).
     *
     * References:
     * - FreeRTOS+TCP V4 porting guide, "Receiving data": xSendEventStructToIPTask(),
     * eNetworkRxEvent
     */
    void rx_task() {
        for (;;) {
            const std::size_t frame_length = device_.peek_frame_size();
            if (frame_length == 0u) {
                (void)rx_event_.wait_for(rx_interrupt_attached_ ? kRxIdleWaitMs : kRxPollMs);
                continue;
            }

            NetworkBufferDescriptor_t* descriptor =
                pxGetNetworkBufferWithDescriptor(frame_length, 0);
            if (descriptor == nullptr) {
                iptraceETHERNET_RX_EVENT_LOST();
                rosette::platform::sleep_ms(kRxBufferRetryMs);
                continue;
            }

            descriptor->xDataLength =
                device_.receive_frame(descriptor->pucEthernetBuffer, frame_length);
            descriptor->pxInterface = interface_;

            freertos_multicast_service().process_frame(descriptor->pucEthernetBuffer,
                                                       descriptor->xDataLength, interface_);

            descriptor->pxEndPoint =
                FreeRTOS_MatchingEndpoint(interface_, descriptor->pucEthernetBuffer);
            if ((descriptor->pxEndPoint == nullptr) || (descriptor->xDataLength == 0u)) {
                vReleaseNetworkBufferAndDescriptor(descriptor);
                continue;
            }

            IPStackEvent_t event{};
            event.eEventType = eNetworkRxEvent;
            event.pvData = descriptor;

            if (xSendEventStructToIPTask(&event, 0) != pdTRUE) {
                vReleaseNetworkBufferAndDescriptor(descriptor);
                iptraceETHERNET_RX_EVENT_LOST();
            } else {
                iptraceNETWORK_INTERFACE_RECEIVE();
            }
        }
    }

    Device device_;
    rosette::platform::Event rx_event_;
    NetworkInterface_t* interface_ = nullptr;
    TaskHandle_t rx_task_handle_ = nullptr;
    std::array<char, 16> name_{};
    mac_address_type mac_address_{};
    UBaseType_t multicast_accept_refcount_ = 0u;
    bool rx_interrupt_attached_ = false;
};

}  // namespace rosette::net::freertos
