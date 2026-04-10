/**
 * @file net/wiznet_macraw.h
 * @brief WIZnet W5100S MACRAW network interface — public interface.
 *
 * The W5100S is operated in MACRAW mode (socket 0, Sn_MR_MACRAW) so that
 * FreeRTOS+TCP's IP stack handles all IP/TCP/UDP framing, including:
 *   - IP fragmentation and reassembly (transparent to the RTPS layer)
 *   - UDP checksum validation
 *   - IGMP for multicast group membership (SPDP 239.255.0.1)
 *   - ARP
 *
 * Why MACRAW?
 * -----------
 * The W5100S hardware socket API (socket mode) handles UDP internally but
 * cannot reassemble IP-fragmented datagrams — fragments are silently dropped.
 * RTPS DATA_FRAG submessages are designed to avoid IP fragmentation, but
 * interoperating with standard DDS implementations that send large unfragmented
 * UDP datagrams requires a full IP stack.  MACRAW + FreeRTOS+TCP provides this.
 *
 * W5100S MACRAW specifics
 * -----------------------
 * - Only socket 0 supports MACRAW.
 * - The W5100S prepends a 2-byte big-endian length field to each received
 *   Ethernet frame in the RX ring buffer.  wiznet_macraw_rx() strips this.
 * - The Sn_MR_MFEN flag enables multicast reception (required for SPDP).
 * - TX buffer allocation for socket 0 should be maximised (all 16 KB on
 *   W5100S) since it is the only active socket.
 *
 * FreeRTOS+TCP integration
 * ------------------------
 * This driver implements the three functions required by FreeRTOS+TCP's
 * NetworkInterface abstraction:
 *   xNetworkInterfaceInitialise()  — open MACRAW socket, set MAC, start RX task
 *   xNetworkInterfaceOutput()      — send one Ethernet frame via MACRAW
 *   (RX) vNetworkInterfaceTask()   — poll for frames, call xSendEventToIPTask()
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// FreeRTOS+TCP network interface API.
#include "FreeRTOS_IP.h"
#include "NetworkInterface.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// wiznet_macraw_init()
//
// Called from board_init() after the WIZnet ioLibrary SPI callbacks
// have been registered.  Initialises the W5100S chip configuration
// (buffer sizes) and opens socket 0 in MACRAW mode.
//
// Must be called before FreeRTOS_IPInit() / before the scheduler starts.
// ============================================================
void wiznet_macraw_init(void);

// ============================================================
// FreeRTOS+TCP NetworkInterface implementation
//
// These are the three entry points FreeRTOS+TCP calls into the driver.
// Do not call them directly from application code.
// ============================================================

/**
 * xNetworkInterfaceInitialise() — §FreeRTOS+TCP NetworkInterface API.
 *
 * Called by FreeRTOS+TCP at startup.  Sets the MAC address on the W5100S,
 * starts the background RX polling task, and signals to FreeRTOS+TCP that
 * the link is up.
 *
 * @returns pdTRUE on success, pdFALSE on failure.
 */
BaseType_t xNetworkInterfaceInitialise(void);

/**
 * xNetworkInterfaceOutput() — §FreeRTOS+TCP NetworkInterface API.
 *
 * Sends one Ethernet frame (already fully assembled by FreeRTOS+TCP)
 * via the W5100S MACRAW socket.
 *
 * @param pxDescriptor   Network buffer descriptor holding the frame.
 * @param xReleaseAfterSend  If pdTRUE the driver must free pxDescriptor.
 * @returns pdTRUE on success.
 */
BaseType_t xNetworkInterfaceOutput(NetworkBufferDescriptor_t* pxDescriptor,
                                   BaseType_t                 xReleaseAfterSend);

// ============================================================
// wiznet_macraw_link_status()
//
// Returns true if the W5100S reports a physical link.
// Reads PHY_STATUS register — not available on all W5100S revisions;
// falls back to assuming link-up if the register cannot be read.
// ============================================================
bool wiznet_macraw_link_status(void);

#ifdef __cplusplus
}
#endif
