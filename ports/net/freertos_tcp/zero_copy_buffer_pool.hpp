#pragma once

/**
 * @file ports/net/freertos_tcp/zero_copy_buffer_pool.hpp
 * @brief FreeRTOS+TCP's network buffers, laid out so a zero-copy MAC's DMA can use any of them.
 *
 * ZeroCopyNetworkInterface (zero_copy_interface.hpp) posts the stack's network
 * buffers straight to the MAC, so every buffer must meet the device's layout
 * rules (rosette/net/zero_copy_mac.hpp): its region, which starts
 * FrameOffset octets before pucEthernetBuffer, aligned to Alignment and
 * reachable by the DMA. FreeRTOS+TCP's BufferAllocation_1 takes its buffer
 * storage from the network interface (vNetworkInterfaceAllocateRAMToBuffers()),
 * and this class is that storage: one static array the board places in
 * DMA-reachable, non-cached RAM.
 *
 * Each buffer occupies one stride, a multiple of Alignment:
 *
 *   region start (aligned)
 *   |<- FrameOffset ->|<- frame: ipTOTAL_ETHERNET_FRAME_SIZE ->| ... |<- next buffer's header ->|
 *                     ^ pucEthernetBuffer
 *
 * FreeRTOS+TCP keeps ipBUFFER_PADDING octets in front of pucEthernetBuffer:
 * the pointer back to the descriptor at the start, and the IP version byte
 * ipIP_TYPE_OFFSET octets before the frame. Those that precede the region
 * live at the end of the previous stride (or in the leading pad, for the
 * first buffer), where the DMA never writes.
 *
 * References: FreeRTOS+TCP V4.2.2 BufferAllocation_1.c,
 * vNetworkInterfaceAllocateRAMToBuffers(); FreeRTOS_IP.h ipBUFFER_PADDING;
 * FreeRTOS_IPv4_Private.h ipIP_TYPE_OFFSET.
 */

#include <cstddef>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
}

namespace rosette::net::freertos {

/**
 * @brief Static storage for all ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS network buffers, in a
 * zero-copy MAC's layout.
 *
 * @tparam FrameOffset The device's frame_offset().
 * @tparam Alignment   The device's buffer_alignment(); a power of two.
 *
 * Example:
 * @code{.cpp}
 * using Pool = freertos::ZeroCopyBufferPool<ImxrtEnetZeroCopyDevice::kFrameOffset,
 *                                           ImxrtEnetZeroCopyDevice::kBufferAlignment>;
 * static_assert(Pool::kRegionBytes >= ImxrtEnetZeroCopyDevice::kBufferBytes);
 * Pool g_pool;  // .bss: DTCM, DMA-reachable and not cached
 * extern "C" void vNetworkInterfaceAllocateRAMToBuffers(
 *     NetworkBufferDescriptor_t buffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS]) {
 *     g_pool.assign(buffers);
 * }
 * @endcode
 * Full program: boards/mimxrt1020-evk/enet_zero_copy_interface.cpp.
 */
template <std::size_t FrameOffset, std::size_t Alignment>
class ZeroCopyBufferPool {
    static_assert((Alignment & (Alignment - 1u)) == 0u,
                  "ZeroCopyBufferPool: Alignment is a power of two");
    static_assert(ipBUFFER_PADDING >= FrameOffset + sizeof(void*),
                  "ZeroCopyBufferPool: the back pointer must precede the DMA region");
    static_assert(ipIP_TYPE_OFFSET > static_cast<int32_t>(FrameOffset),
                  "ZeroCopyBufferPool: the IP version byte must precede the DMA region");

public:
    /// Octets of FreeRTOS+TCP's header that precede each region.
    static constexpr std::size_t kHeaderBytes = ipBUFFER_PADDING - FrameOffset;

    /// Octets from one region start to the next.
    static constexpr std::size_t kStride =
        (ipBUFFER_PADDING + ipTOTAL_ETHERNET_FRAME_SIZE + Alignment - 1u) & ~(Alignment - 1u);

    /// Octets from a region start that belong to that buffer alone: the most a DMA may write.
    static constexpr std::size_t kRegionBytes = kStride - kHeaderBytes;

    /// Number of buffers: every network buffer the stack has.
    static constexpr std::size_t kCount = ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS;

    /**
     * @brief Gives each descriptor its buffer: BufferAllocation_1's
     * vNetworkInterfaceAllocateRAMToBuffers().
     *
     * Called once by FreeRTOS+TCP from FreeRTOS_IPInit_Multi(), before any
     * buffer is in use.
     *
     * @param[in,out] descriptors The stack's descriptor array; each gets its
     *   pucEthernetBuffer, and the octets in front of it get the pointer back
     *   to the descriptor that pxPacketBuffer_to_NetworkBuffer() reads.
     *
     * Verification:
     * - Correct means: every pucEthernetBuffer minus FrameOffset is a
     *   multiple of Alignment, buffers do not overlap, and
     *   pxPacketBuffer_to_NetworkBuffer(pucEthernetBuffer) returns its own
     *   descriptor.
     * - Covered by: the zero-copy device's alignment check, which rejects any
     *   buffer that breaks the first rule and keeps the interface down
     *   (bring-up on the MIMXRT1020-EVK).
     */
    void assign(NetworkBufferDescriptor_t descriptors[kCount]) noexcept {
        for (std::size_t i = 0u; i < kCount; ++i) {
            uint8_t* const frame = &storage_[Alignment + i * kStride + FrameOffset];
            descriptors[i].pucEthernetBuffer = frame;
            *reinterpret_cast<NetworkBufferDescriptor_t**>(frame - ipBUFFER_PADDING) =
                &descriptors[i];
        }
    }

private:
    alignas(Alignment) uint8_t storage_[Alignment + kCount * kStride];
};

}  // namespace rosette::net::freertos
