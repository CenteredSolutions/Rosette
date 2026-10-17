#pragma once

/**
 * @file ports/net/freertos_tcp/interface_binding.hpp
 * @brief What every FreeRTOS+TCP network interface over a rosette MAC shares: the descriptor,
 * the receive task, the link.
 *
 * FreeRTOS+TCP reaches hardware through a C descriptor, NetworkInterface_t,
 * whose callbacks initialise the MAC, transmit a buffer, and report the link.
 * Two interfaces fill it, one per MAC contract: NetworkInterface<Device>
 * (interface.hpp) over a copying rosette::net::MacDevice, and
 * LendingNetworkInterface<Device> (lending_interface.hpp) over a
 * rosette::net::LendingMacDevice. They differ only in how frames move, so
 * InterfaceBinding holds the rest and calls the derived class (CRTP) for
 * the data path:
 *
 *   bool prepare_device()     before the first device initialise(), e.g. lend
 *                             the receive ring its buffers
 *   BaseType_t output(...)    pfOutput
 *   bool send_raw(...)        the IGMP / MLD service's frames
 *   void rx_task()            the receive task's body
 *
 * Receive wake-up goes through the platform contract: the receive task sleeps
 * on a platform::Event. An on-chip MAC signals it from its own interrupt
 * (MacLink::set_receive_event()); for a SPI controller the board attaches the
 * controller's interrupt pin (attach_rx_interrupt()), whose falling edge
 * signals it. With neither, the task polls the device every kRxPollMs.
 *
 * Link: the interface reports itself initialised only while the PHY has
 * carrier, so FreeRTOS+TCP brings its endpoints up (and starts DHCP) only
 * with a link. The receive task checks the PHY every kLinkPollMs while idle,
 * which also lets the device follow a renegotiated speed and duplex, and
 * calls FreeRTOS_NetworkDown() when carrier goes; the stack then retries
 * initialise() until carrier returns.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Routing.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"
}

#include "ports/net/freertos_tcp/igmp.hpp"
#include "rosette/net/mac.hpp"
#include "rosette/platform/gpio_interrupt.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::net::freertos {

/**
 * @brief Binds a MAC provider to a FreeRTOS+TCP NetworkInterface_t; the derived class moves the
 * frames.
 *
 * Owns the provider, the receive task, and the event that task sleeps on; it
 * translates the C callback ABI into calls on the derived class and the
 * provider, and hands received network buffers to the IP task after the
 * multicast service has inspected them (deliver()).
 *
 * One instance backs one hardware interface for the lifetime of the stack,
 * typically as a file-scope object in the board's composition. It is neither
 * copied nor moved, because the descriptor and the interrupt hold its
 * address.
 *
 * @tparam Derived The interface (NetworkInterface or LendingNetworkInterface).
 * @tparam Device  Concrete MAC provider, derived from net::MacLink through
 *                 one of the two MAC contracts, constructed in place from the
 *                 interface's constructor arguments.
 */
template <typename Derived, typename Device>
class InterfaceBinding {
public:
    using device_type = Device;
    using mac_address_type = net::MacAddress;

    /// Receive-task sleep between device polls when no interrupt pin is attached.
    static constexpr uint32_t kRxPollMs = 64u;

    /// Receive-task sleep with a receive interrupt; bounds the cost of a lost edge.
    static constexpr uint32_t kRxIdleWaitMs = 1000u;

    /// Interval between PHY checks while the receive task is idle.
    static constexpr uint32_t kLinkPollMs = 1000u;

    /**
     * @brief Constructs the provider in place; nothing touches hardware until FreeRTOS+TCP
     * initialises the interface.
     *
     * @param[in] station_mac Station address programmed by initialise().
     * @param[in] device_args Forwarded to Device's constructor, e.g. the bus.
     */
    template <typename... DeviceArgs>
    explicit InterfaceBinding(const mac_address_type& station_mac,
                              DeviceArgs&&... device_args) noexcept
        : device_(std::forward<DeviceArgs>(device_args)...), mac_address_(station_mac) {}

    InterfaceBinding(const InterfaceBinding&) = delete;
    InterfaceBinding& operator=(const InterfaceBinding&) = delete;

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
            [](void* self) { static_cast<InterfaceBinding*>(self)->rx_event_.signal_from_isr(); },
            this);
        return rx_interrupt_attached_;
    }

    /// @brief Sets the station address the next initialise() programs; touches no hardware.
    void set_station_mac_address(const mac_address_type& mac) noexcept { mac_address_ = mac; }

    /**
     * @brief pfInitialise: brings the provider up and starts the receive task once.
     *
     * FreeRTOS+TCP calls this from the IP task at start-up and again after a
     * link-down, until it returns pdTRUE. Until the receive task exists, the
     * derived class's prepare_device() runs first each time; afterwards the
     * receive task alone feeds the device.
     *
     * @return pdTRUE when the provider carries frames, the PHY has carrier,
     *   and the receive task runs; pdFALSE when the descriptor is not filled,
     *   the provider failed to prepare or initialise, there is no carrier, or
     *   the task could not be created (the stack retries).
     */
    BaseType_t initialise() {
        if (interface_ == nullptr) {
            return pdFALSE;
        }

        if (rx_task_handle_ == nullptr && !derived().prepare_device()) {
            return pdFALSE;
        }

        if (!device_.initialise(mac_address_)) {
            return pdFALSE;
        }

        link_up_ = device_.phy_status().link_up;
        if (!link_up_) {
            return pdFALSE;
        }

        if (rx_task_handle_ == nullptr) {
            rx_interrupt_attached_ =
                device_.set_receive_event(&rx_event_) || rx_interrupt_attached_;
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

    /// @brief pfGetPhyLinkStatus: pdTRUE while the provider reports carrier.
    BaseType_t link_status() noexcept { return device_.phy_status().link_up ? pdTRUE : pdFALSE; }

    /// @brief The owned MAC provider.
    Device& device() noexcept { return device_; }

protected:
    ~InterfaceBinding() = default;

    /// @brief The registered descriptor; null until fill_interface_descriptor().
    NetworkInterface_t* interface() const noexcept { return interface_; }

    /**
     * @brief Sleeps until the device signals a frame, polling the link when nothing came.
     *
     * The receive task calls it once it has drained the device. A device's
     * receive-notification rule guarantees a signal for any frame queued
     * after the drain, so a timeout means the line was quiet.
     */
    void wait_for_frames() noexcept {
        if (!rx_event_.wait_for(rx_interrupt_attached_ ? kRxIdleWaitMs : kRxPollMs)) {
            poll_link();
        }
    }

    /**
     * @brief Hands one received frame to the IP task, or releases it.
     *
     * The multicast service sees every frame first (it answers IGMP and MLD
     * queries itself); then the frame is matched to an endpoint and posted as
     * eNetworkRxEvent. A frame no endpoint wants, or one the IP task's queue
     * has no room for, is released here.
     *
     * @param[in] descriptor A network buffer holding one frame, with
     *   xDataLength set; ownership passes to this call.
     * @return True when the IP task took the frame.
     *
     * Verification:
     * - Correct means: every frame is posted to the IP task exactly once or
     *   released exactly once.
     * - Covered by: no automated test (needs a host FreeRTOS+TCP harness).
     *
     * References:
     * - FreeRTOS+TCP V4 porting guide, "Receiving data": xSendEventStructToIPTask(),
     *   eNetworkRxEvent
     */
    bool deliver(NetworkBufferDescriptor_t* descriptor) noexcept {
        descriptor->pxInterface = interface_;
        freertos_multicast_service().process_frame(descriptor->pucEthernetBuffer,
                                                   descriptor->xDataLength, interface_);

        descriptor->pxEndPoint =
            FreeRTOS_MatchingEndpoint(interface_, descriptor->pucEthernetBuffer);
        if ((descriptor->pxEndPoint == nullptr) || (descriptor->xDataLength == 0u)) {
            vReleaseNetworkBufferAndDescriptor(descriptor);
            return false;
        }

        IPStackEvent_t event{};
        event.eEventType = eNetworkRxEvent;
        event.pvData = descriptor;
        if (xSendEventStructToIPTask(&event, 0) != pdTRUE) {
            vReleaseNetworkBufferAndDescriptor(descriptor);
            iptraceETHERNET_RX_EVENT_LOST();
            return false;
        }

        iptraceNETWORK_INTERFACE_RECEIVE();
        return true;
    }

    /// @brief True when @p length is a frame the provider can carry (MacLink::max_frame_size()).
    bool frame_fits(std::size_t length) const noexcept {
        const std::size_t max_frame = device_.max_frame_size();
        return (length != 0u) && ((max_frame == 0u) || (length <= max_frame));
    }

    Device device_;

private:
    Derived& derived() noexcept { return static_cast<Derived&>(*this); }

    static InterfaceBinding* from_interface(NetworkInterface_t* interface) {
        return interface == nullptr ? nullptr
                                    : static_cast<InterfaceBinding*>(interface->pvArgument);
    }

    static BaseType_t pf_initialise(NetworkInterface_t* interface) {
        InterfaceBinding* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->initialise();
    }

    static BaseType_t pf_output(NetworkInterface_t*              interface,
                                NetworkBufferDescriptor_t* const descriptor,
                                BaseType_t                       xReleaseAfterSend) {
        InterfaceBinding* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->derived().output(descriptor, xReleaseAfterSend);
    }

    static BaseType_t pf_get_phy_link_status(NetworkInterface_t* interface) {
        InterfaceBinding* self = from_interface(interface);
        return self == nullptr ? pdFALSE : self->link_status();
    }

    static void pf_add_allowed_mac(NetworkInterface_t* interface, const uint8_t* mac_address) {
        InterfaceBinding* self = from_interface(interface);
        if (self != nullptr) {
            self->add_allowed_mac(mac_address);
        }
    }

    static void pf_remove_allowed_mac(NetworkInterface_t* interface, const uint8_t* mac_address) {
        InterfaceBinding* self = from_interface(interface);
        if (self != nullptr) {
            self->remove_allowed_mac(mac_address);
        }
    }

    static bool igmp_output(NetworkInterface_t* interface, const uint8_t* frame, std::size_t length,
                            void* context) noexcept {
        auto* self = static_cast<InterfaceBinding*>(context);
        if ((self == nullptr) || (interface != self->interface_) || (frame == nullptr)
            || !self->frame_fits(length)) {
            return false;
        }

        return self->derived().send_raw(frame, length);
    }

    static void rx_task_entry(void* arg) {
        static_cast<InterfaceBinding*>(arg)->derived().rx_task();
    }

    void set_name(const char* name) {
        std::memset(name_.data(), 0, name_.size());
        if (name != nullptr) {
            std::strncpy(name_.data(), name, name_.size() - 1u);
        }
    }

    /// @brief pfAddAllowedMAC: opens the device's multicast filter for one group address.
    void add_allowed_mac(const uint8_t* mac_address) noexcept {
        if ((mac_address != nullptr) && ((mac_address[0] & 0x01u) != 0u)) {
            device_.add_multicast(to_mac(mac_address));
        }
    }

    /// @brief pfRemoveAllowedMAC: withdraws one add_allowed_mac() for the group.
    void remove_allowed_mac(const uint8_t* mac_address) noexcept {
        if ((mac_address != nullptr) && ((mac_address[0] & 0x01u) != 0u)) {
            device_.remove_multicast(to_mac(mac_address));
        }
    }

    static mac_address_type to_mac(const uint8_t* octets) noexcept {
        mac_address_type mac{};
        std::memcpy(mac.data(), octets, mac.size());
        return mac;
    }

    /**
     * @brief Checks the PHY at most every kLinkPollMs, and takes the network down when carrier
     * goes.
     *
     * Verification:
     * - Correct means: unplugging the cable brings the endpoints down within
     *   kLinkPollMs plus one idle wait, and plugging it back in brings them
     *   up again with a fresh DHCP exchange.
     * - Covered by: cable pull on the MIMXRT1020-EVK.
     *
     * References: FreeRTOS+TCP V4 FreeRTOS_NetworkDown().
     */
    void poll_link() noexcept {
        const uint32_t now = rosette::platform::monotonic_ms();
        if (now - last_link_poll_ms_ < kLinkPollMs) {
            return;
        }

        last_link_poll_ms_ = now;
        const bool up = device_.phy_status().link_up;
        if (link_up_ && !up) {
            FreeRTOS_NetworkDown(interface_);
        }

        link_up_ = up;
    }

    rosette::platform::Event rx_event_;
    NetworkInterface_t*      interface_ = nullptr;
    TaskHandle_t             rx_task_handle_ = nullptr;
    std::array<char, 16>     name_{};
    mac_address_type         mac_address_{};
    uint32_t                 last_link_poll_ms_ = 0u;
    bool                     link_up_ = false;
    bool                     rx_interrupt_attached_ = false;
};

}  // namespace rosette::net::freertos
