#pragma once

/**
 * @file ports/net/freertos_tcp/lending_interface.hpp
 * @brief A FreeRTOS+TCP network interface that lends the stack's network buffers to a
 * rosette::net::LendingMacDevice.
 *
 * With a lending MAC (rosette/net/lending_mac.hpp) a frame is never copied
 * between the wire and the IP task. The receive ring holds empty network
 * buffers, and a frame the DMA wrote into one goes to the IP task in that
 * same buffer. A frame the stack built goes to the wire from its own buffer,
 * which comes back to the pool once the device reports it sent. The device
 * keeps no frame storage, so the stack's pool is the only RAM frames
 * occupy.
 *
 * What the stack must provide for that:
 *  - BufferAllocation_1, with every buffer in the device's layout and in
 *    DMA-reachable, non-cached RAM: the board defines
 *    vNetworkInterfaceAllocateRAMToBuffers() over a LentBufferPool
 *    (lent_buffer_pool.hpp);
 *  - ipconfigZERO_COPY_TX_DRIVER = 1, so pfOutput always owns the buffer it
 *    is given (the stack duplicates one it must keep);
 *  - more buffers than the device's receive ring holds, since the ring keeps
 *    its share at all times.
 * ports/net/freertos_tcp/port.cmake sets the first two when the board asks
 * for ROSETTE_FREERTOS_TCP_LENT_BUFFERS.
 *
 * The descriptor, the receive wake-up, the link, and the multicast filter are
 * InterfaceBinding's (interface_binding.hpp); this file is the data path.
 *
 * References: FreeRTOS+TCP V4 porting guide, "Zero copy" (receive and
 * ipconfigZERO_COPY_TX_DRIVER).
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "ports/net/freertos_tcp/interface_binding.hpp"
#include "rosette/net/lending_mac.hpp"
#include "rosette/platform/platform.hpp"

#if ipconfigZERO_COPY_TX_DRIVER == 0
#  error \
      "LendingNetworkInterface needs ipconfigZERO_COPY_TX_DRIVER (ROSETTE_FREERTOS_TCP_LENT_BUFFERS)"
#endif

namespace rosette::net::freertos {

/**
 * @brief A FreeRTOS+TCP interface whose frames stay in the stack's network buffers end to end.
 *
 * Example:
 * @code{.cpp}
 * imxrt::ImxrtEnetLendingRings<4, 4> g_rings;
 * LendingNetworkInterface<imxrt::ImxrtEnetLendingDevice> g_enet0{
 *     MacAddress{}, g_rings, imxrt::ImxrtEnetConfig{2u, 5u, &configure_ksz8081}};
 * NetworkInterface_t g_enet0_descriptor;
 * g_enet0.add_to_freertos(g_enet0_descriptor, "enet0");  // the ring is lent in pfInitialise
 * @endcode
 * Full program: boards/mimxrt1020-evk/enet_lending_interface.cpp.
 *
 * @tparam Device Concrete MAC provider, derived from net::LendingMacDevice
 *                and constructed in place from the interface's constructor
 *                arguments.
 */
template <typename Device>
class LendingNetworkInterface : public InterfaceBinding<LendingNetworkInterface<Device>, Device> {
    static_assert(
        std::is_base_of_v<net::LendingMacDevice, Device>,
        "LendingNetworkInterface<Device>: Device must implement rosette::net::LendingMacDevice");

    using Binding = InterfaceBinding<LendingNetworkInterface<Device>, Device>;
    friend Binding;

public:
    /// Longest a transmit waits for the device to free a transmit descriptor: several full frames'
    /// wire time at 100 Mbit/s.
    static constexpr uint64_t kTxBusyTimeoutUs = 1000u;

    using Binding::Binding;

private:
    /**
     * @brief Lends network buffers to the receive ring until it is full.
     *
     * Runs in pfInitialise until the receive task exists; a ring left short
     * (the pool is empty, or a buffer broke the device's layout) makes the
     * device's initialise() fail, and the next pfInitialise tries again.
     *
     * @return False when the pool ran out before the ring was full.
     */
    bool prepare_device() noexcept {
        for (;;) {
            NetworkBufferDescriptor_t* buffer =
                pxGetNetworkBufferWithDescriptor(ipTOTAL_ETHERNET_FRAME_SIZE, 0);
            if (buffer == nullptr) {
                return false;
            }

            if (!this->device_.lend_receive_buffer(buffer->pucEthernetBuffer, buffer)) {
                vReleaseNetworkBufferAndDescriptor(buffer);
                return true;
            }
        }
    }

    /**
     * @brief pfOutput: queues the descriptor's own buffer on the device, which hands it back once
     * sent.
     *
     * With ipconfigZERO_COPY_TX_DRIVER the stack passes pdTRUE wherever it can
     * give the buffer away; elsewhere (TCP's retransmission paths) the frame
     * goes out of a duplicate, since the stack keeps the original.
     *
     * @param[in] pxDescriptor      Frame to send.
     * @param[in] xReleaseAfterSend Non-zero when this call owns @p pxDescriptor.
     * @return pdTRUE when the device queued the frame; pdFALSE when it was
     *   dropped (the buffer is released either way it is ours).
     */
    BaseType_t output(NetworkBufferDescriptor_t* pxDescriptor, BaseType_t xReleaseAfterSend) {
        collect_sent();
        if (pxDescriptor == nullptr) {
            return pdFALSE;
        }

        NetworkBufferDescriptor_t* owned = pxDescriptor;
        if (xReleaseAfterSend == pdFALSE) {
            owned = pxDuplicateNetworkBufferWithDescriptor(pxDescriptor, pxDescriptor->xDataLength);
            if (owned == nullptr) {
                return pdFALSE;
            }
        }

        iptraceNETWORK_INTERFACE_TRANSMIT();
        return send_owned(owned) ? pdTRUE : pdFALSE;
    }

    /// @brief The multicast service's frames: copied into a network buffer, then sent from it.
    bool send_raw(const uint8_t* frame, std::size_t length) noexcept {
        NetworkBufferDescriptor_t* buffer = pxGetNetworkBufferWithDescriptor(length, 0);
        if (buffer == nullptr) {
            return false;
        }

        std::memcpy(buffer->pucEthernetBuffer, frame, length);
        buffer->xDataLength = length;
        return send_owned(buffer);
    }

    /**
     * @brief Queues a buffer this interface owns, waiting up to kTxBusyTimeoutUs for a free
     * descriptor.
     *
     * @param[in] buffer A frame in a pool buffer; the device or this call releases it.
     * @return True when the device queued the frame.
     */
    bool send_owned(NetworkBufferDescriptor_t* buffer) noexcept {
        if (this->frame_fits(buffer->xDataLength)) {
            const LentFrame frame{buffer->pucEthernetBuffer, buffer->xDataLength, buffer};
            const uint64_t  start = rosette::platform::monotonic_us();
            do {
                if (this->device_.send_lent_frame(frame)) {
                    return true;
                }

                collect_sent();
            } while (rosette::platform::monotonic_us() - start <= kTxBusyTimeoutUs);
        }

        vReleaseNetworkBufferAndDescriptor(buffer);
        return false;
    }

    /// @brief Returns the buffers of every frame the device has finished sending to the pool.
    void collect_sent() noexcept {
        void* owner = nullptr;
        while (this->device_.take_sent_frame(owner)) {
            vReleaseNetworkBufferAndDescriptor(static_cast<NetworkBufferDescriptor_t*>(owner));
        }
    }

    /**
     * @brief Receive task: swaps each received buffer for a fresh one and delivers it.
     *
     * Each pass first returns sent buffers to the pool (the device's
     * transmit interrupt wakes the task for that), then takes one received
     * frame. A fresh network buffer takes the frame's place in the ring
     * before the frame goes to the IP task, so the ring stays full. When the
     * pool has nothing free, the frame is dropped and its own buffer goes
     * straight back to the ring: reception degrades to dropping while the
     * stack is saturated and recovers as soon as it releases buffers.
     *
     * Verification:
     * - Correct means: the device's receive ring holds its full count of
     *   buffers after every pass, including with the pool exhausted; every
     *   received frame is delivered exactly once or dropped; and every sent
     *   buffer returns to the pool.
     * - Covered by: a ping flood with tools/udp_echo_probe.py running against
     *   the MIMXRT1020-EVK, after which the echo still passes (no automated
     *   test; needs a host FreeRTOS+TCP harness).
     *
     * References:
     * - FreeRTOS+TCP V4 porting guide, "Receiving data" and "Zero copy"
     */
    void rx_task() {
        for (;;) {
            collect_sent();

            LentFrame frame;
            if (!this->device_.take_received_frame(frame)) {
                this->wait_for_frames();
                continue;
            }

            auto* received = static_cast<NetworkBufferDescriptor_t*>(frame.owner);
            NetworkBufferDescriptor_t* fresh =
                pxGetNetworkBufferWithDescriptor(ipTOTAL_ETHERNET_FRAME_SIZE, 0);
            if (fresh == nullptr) {
                (void)this->device_.lend_receive_buffer(frame.data, received);
                iptraceETHERNET_RX_EVENT_LOST();
                continue;
            }

            // A slot was freed by the take above, so the ring accepts the fresh buffer.
            if (!this->device_.lend_receive_buffer(fresh->pucEthernetBuffer, fresh)) {
                vReleaseNetworkBufferAndDescriptor(fresh);
            }

            received->xDataLength = frame.length;
            this->deliver(received);
        }
    }
};

}  // namespace rosette::net::freertos
