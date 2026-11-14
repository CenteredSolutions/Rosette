// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file ports/net/freertos_tcp/interface.hpp
 * @brief A FreeRTOS+TCP network interface over any copying rosette::net::MacDevice.
 *
 * NetworkInterface<Device> fills FreeRTOS+TCP's NetworkInterface_t and
 * forwards each callback to a MAC provider it owns, and runs the receive task
 * that feeds frames to the IP task. The provider is anything that implements
 * the neutral contract in rosette/net/mac.hpp, so this port knows no chip,
 * and no chip driver knows FreeRTOS+TCP. Frames are copied: each received
 * frame into a fresh network buffer, each transmitted one out of the
 * stack's buffer into the provider. A DMA controller that can move frames in
 * the stack's own buffers uses ZeroCopyNetworkInterface (zero_copy_interface.hpp)
 * instead.
 *
 * The descriptor, the receive wake-up, the link, and the multicast filter are
 * InterfaceBinding's (interface_binding.hpp); this file is the data path.
 */

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "ports/net/freertos_tcp/interface_binding.hpp"
#include "rosette/net/mac.hpp"

namespace rosette::net::freertos {

/**
 * @brief A FreeRTOS+TCP interface that copies frames to and from a MacDevice.
 *
 * Example:
 * @code{.cpp}
 * NetworkInterface<BoardMacrawDevice> g_wiz0{MacAddress ROSETTE_MAC_ADDR_DEFAULT, bus};
 * NetworkInterface_t g_wiz0_descriptor;
 * g_wiz0.add_to_freertos(g_wiz0_descriptor, "wiz0");
 * g_wiz0.attach_rx_interrupt(ROSETTE_PIN_WIZNET_INT);  // false: the receive task polls
 * @endcode
 * Full program: boards/common/wiznet/wiznet_macraw_interface.cpp.
 *
 * @tparam Device Concrete MAC provider, derived from net::MacDevice and
 *                constructed in place from the interface's constructor
 *                arguments.
 */
template <typename Device>
class NetworkInterface : public InterfaceBinding<NetworkInterface<Device>, Device> {
    static_assert(std::is_base_of_v<net::MacDevice, Device>,
                  "NetworkInterface<Device>: Device must implement rosette::net::MacDevice");

    using Binding = InterfaceBinding<NetworkInterface<Device>, Device>;
    friend Binding;

public:
    /// Receive-task pause when FreeRTOS+TCP has no network buffer free, so the IP task can return
    /// one.
    static constexpr uint32_t kRxBufferRetryMs = 2u;

    using Binding::Binding;

private:
    /// @brief A copying device needs nothing before initialise().
    bool prepare_device() noexcept { return true; }

    /**
     * @brief pfOutput: copies one descriptor's frame into the provider and releases it when asked.
     *
     * @param[in] pxDescriptor      Frame to send; rejected when empty or longer than
     *   max_frame_size().
     * @param[in] xReleaseAfterSend Non-zero when this call owns, and must release, @p pxDescriptor.
     * @return pdTRUE when the provider accepted the frame; pdFALSE otherwise.
     */
    BaseType_t output(NetworkBufferDescriptor_t* pxDescriptor, BaseType_t xReleaseAfterSend) {
        BaseType_t ret = pdFALSE;
        if ((pxDescriptor != nullptr) && (pxDescriptor->pucEthernetBuffer != nullptr)
            && send_raw(pxDescriptor->pucEthernetBuffer, pxDescriptor->xDataLength)) {
            ret = pdTRUE;
        }

        iptraceNETWORK_INTERFACE_TRANSMIT();

        if ((xReleaseAfterSend != pdFALSE) && (pxDescriptor != nullptr)) {
            vReleaseNetworkBufferAndDescriptor(pxDescriptor);
        }

        return ret;
    }

    // Hands a frame to the provider when it is within max_frame_size(), the
    // precondition MacDevice::send_frame() relies on.
    bool send_raw(const uint8_t* frame, std::size_t length) noexcept {
        return this->frame_fits(length) && this->device_.send_frame(frame, length);
    }

    /**
     * @brief Receive task: drains the provider into the IP task, then sleeps until the next frame.
     *
     * Each pass sizes a network buffer from peek_frame_size(), copies the
     * head frame into it, and delivers it (InterfaceBinding::deliver()). When
     * the queue is empty the task sleeps on the receive event. When
     * FreeRTOS+TCP has no buffer free, the frame stays queued and the task
     * pauses for kRxBufferRetryMs, so the lower-priority IP task can release
     * one.
     *
     * Verification:
     * - Correct means: every frame the provider queues is posted to the IP
     *   task exactly once or released, and the task never spins without
     *   blocking.
     * - Covered by: no automated test (needs a host FreeRTOS+TCP harness).
     *
     * References:
     * - FreeRTOS+TCP V4 porting guide, "Receiving data"
     */
    void rx_task() {
        for (;;) {
            const std::size_t frame_length = this->device_.peek_frame_size();
            if (frame_length == 0u) {
                this->wait_for_frames();
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
                this->device_.receive_frame(descriptor->pucEthernetBuffer, frame_length);
            this->deliver(descriptor);
        }
    }
};

}  // namespace rosette::net::freertos
