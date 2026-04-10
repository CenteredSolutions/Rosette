/**
 * @file net/wiznet_macraw.c
 * @brief WIZnet W5100S MACRAW driver — FreeRTOS+TCP NetworkInterface.
 *
 * W5100S MACRAW receive frame format (§W5100S datasheet §5.2.3)
 * -------------------------------------------------------------
 * Each frame in the RX ring buffer is preceded by a 2-byte big-endian
 * length field that includes itself:
 *
 *   | len_hi | len_lo | Ethernet frame (len - 2 bytes) |
 *
 * wiznet_macraw_rx() reads and strips this header before passing the
 * raw Ethernet frame to FreeRTOS+TCP.
 *
 * TX format
 * ---------
 * The W5100S MACRAW TX expects a raw Ethernet frame with no length prefix.
 * The frame must include the Ethernet header (dst MAC, src MAC, EtherType)
 * which FreeRTOS+TCP prepends before calling xNetworkInterfaceOutput().
 *
 * Buffer sizes
 * ------------
 * The W5100S has 16 KB TX and 16 KB RX memory shared across all sockets.
 * In MACRAW mode socket 0 should own all of it.  We configure:
 *   TX buffer: 16 KB (socket 0)
 *   RX buffer: 16 KB (socket 0)
 * Other sockets are disabled (0 KB) since we don't use them alongside MACRAW.
 *
 * The wiznet_macraw_rx() frame buffer is allocated on the task stack of
 * the RX task.  The maximum Ethernet frame is 1518 bytes; allow 1536 for
 * alignment.  Ensure the RX task stack is large enough
 * (WIZNET_RX_TASK_STACK_WORDS >= 768 is a safe minimum).
 */

#include "wiznet_macraw.h"
#include "board_config.h"

/* WIZnet ioLibrary */
#include "wizchip_conf.h"
#include "socket.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* FreeRTOS+TCP */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// ============================================================
// Configuration
// ============================================================

/// MACRAW socket number — must be 0 on W5100S.
#define MACRAW_SOCKET           0

/// Maximum Ethernet frame size (including 4-byte FCS if present).
#define MACRAW_MAX_FRAME_BYTES  1536u

/// FreeRTOS task parameters for the background RX polling task.
#define WIZNET_RX_TASK_STACK_WORDS  800u
#define WIZNET_RX_TASK_PRIORITY     (configMAX_PRIORITIES - 1)
#define WIZNET_RX_TASK_NAME         "wiznet_rx"

/// Polling interval when no frame is available (milliseconds).
/// Set to 0 to busy-poll; increase to yield CPU when idle.
#define WIZNET_RX_POLL_MS           1u

// ============================================================
// Internal state
// ============================================================

static uint8_t g_mac_address[6] = RTPS_MAC_ADDR_DEFAULT;
static bool    g_initialised    = false;

// ============================================================
// wiznet_macraw_init()
//
// Called from board_init() before the FreeRTOS scheduler starts.
// Configures W5100S buffer allocation and opens MACRAW socket 0.
//
// W5100S buffer configuration (§W5100S datasheet §5.1):
//   wizchip_init() takes two arrays: TX sizes and RX sizes per socket.
//   Values are in KB; must sum to <= 16.  We give 8 KB to socket 0.
// ============================================================

void wiznet_macraw_init(void)
{
    /* Allocate 8 KB TX and RX to socket 0; disable all others. */
    uint8_t tx_sizes[_WIZCHIP_SOCK_NUM_] = {0};
    uint8_t rx_sizes[_WIZCHIP_SOCK_NUM_] = {0};
    tx_sizes[0] = 8u;
    rx_sizes[0] = 8u;

    /* wizchip_init() resets the chip's internal buffer pointers and
     * applies the TX/RX allocation.  This must be called after the
     * SPI callbacks are registered (done in board_init()). */
    if (wizchip_init(tx_sizes, rx_sizes) != 0) {
        /* Initialisation failed — halt with LED pattern in debug builds. */
        for (;;) { gpio_put(RTPS_PIN_LED, 1); sleep_ms(100);
                   gpio_put(RTPS_PIN_LED, 0); sleep_ms(100); }
    }

    /* Open socket 0 in MACRAW mode with multicast reception enabled.
     * Sn_MR_MFEN = MAC filter off for multicast frames (required for
     * SPDP 239.255.0.1).                                               */
    if (socket(MACRAW_SOCKET, Sn_MR_MACRAW, 0, Sn_MR_MFEN) != MACRAW_SOCKET) {
        for (;;) { gpio_put(RTPS_PIN_LED, 1); sleep_ms(50);
                   gpio_put(RTPS_PIN_LED, 0); sleep_ms(50); }
    }

    g_initialised = true;
}

// ============================================================
// wiznet_macraw_link_status()
//
// Reads W5100S PHY status to determine if a cable is connected.
// The W5100S PHYCFGR register is at 0x002E.
//
// Bit 0 of PHYCFGR: LNK — 1 = link up, 0 = link down.
// ============================================================

bool wiznet_macraw_link_status(void)
{
    /* STUB — replace with actual W5100S PHYCFGR register read.
     *
     * The W5100S does not expose PHYCFGR through the ioLibrary's
     * high-level API; it must be accessed via the low-level register
     * read function:
     *
     *   uint8_t phycfgr = WIZCHIP_READ(PHYCFGR);
     *   return (phycfgr & 0x01u) != 0;
     *
     * PHYCFGR is defined in w5100s.h as 0x002E.
     * Uncomment once w5100s.h is confirmed to define PHYCFGR.
     */
    return true;   /* Assume link-up for now. */
}

// ============================================================
// xNetworkInterfaceInitialise()
//
// FreeRTOS+TCP calls this at startup and after a link-loss event.
// Sets the MAC address on the W5100S and starts the RX polling task.
// ============================================================

static void vNetworkInterfaceRxTask(void* arg);   /* Forward declaration. */
static TaskHandle_t g_rx_task_handle = NULL;

BaseType_t xNetworkInterfaceInitialise(void)
{
    if (!g_initialised) {
        /* board_init() should have been called first. */
        return pdFALSE;
    }

    /* Set the MAC address on the W5100S SHAR register.
     * FreeRTOS+TCP passes its configured MAC in FreeRTOS_IPInit(); we
     * read it back here and program the chip.                          */
    const uint8_t* pucMAC = FreeRTOS_GetMACAddress();
    if (pucMAC != NULL) {
        setSHAR((uint8_t*)pucMAC);
        memcpy(g_mac_address, pucMAC, 6);
    } else {
        /* Fall back to board default (derived from RP2350 unique ID). */
        extern void board_get_mac_address(uint8_t mac[6]);
        board_get_mac_address(g_mac_address);
        setSHAR(g_mac_address);
    }

    /* Start the background RX task if not already running. */
    if (g_rx_task_handle == NULL) {
        BaseType_t ret = xTaskCreate(
            vNetworkInterfaceRxTask,
            WIZNET_RX_TASK_NAME,
            WIZNET_RX_TASK_STACK_WORDS,
            NULL,
            WIZNET_RX_TASK_PRIORITY,
            &g_rx_task_handle);

        if (ret != pdPASS) {
            return pdFALSE;
        }
    }

    return pdTRUE;
}

// ============================================================
// xNetworkInterfaceOutput()
//
// FreeRTOS+TCP calls this with a fully-assembled Ethernet frame.
// We write it directly to the W5100S MACRAW TX buffer.
//
// The W5100S MACRAW TX does NOT require a length prefix — send()
// takes the raw Ethernet frame including the 14-byte Ethernet header.
//
// pxDescriptor->pucEthernetBuffer points to the frame.
// pxDescriptor->xDataLength is the frame length in bytes.
// ============================================================

BaseType_t xNetworkInterfaceOutput(
        NetworkBufferDescriptor_t* pxDescriptor,
        BaseType_t                 xReleaseAfterSend)
{
    BaseType_t ret = pdFALSE;

    if (pxDescriptor == NULL || pxDescriptor->pucEthernetBuffer == NULL) {
        goto done;
    }

    if (pxDescriptor->xDataLength == 0 ||
        pxDescriptor->xDataLength > MACRAW_MAX_FRAME_BYTES) {
        goto done;
    }

    /* send() blocks until the TX buffer has room (W5100S flow control). */
    {
        int32_t sent = send(MACRAW_SOCKET,
                           pxDescriptor->pucEthernetBuffer,
                           (uint16_t)pxDescriptor->xDataLength);
        if (sent == (int32_t)pxDescriptor->xDataLength) {
            ret = pdTRUE;
        }
    }

    /* IPVIEW / packet-capture hook (compile-time optional). */
    iptraceNETWORK_INTERFACE_TRANSMIT();

done:
    if (xReleaseAfterSend != pdFALSE) {
        vReleaseNetworkBufferAndDescriptor(pxDescriptor);
    }
    return ret;
}

// ============================================================
// vNetworkInterfaceRxTask()
//
// Background task that polls the W5100S RX ring buffer and forwards
// each received Ethernet frame to FreeRTOS+TCP via the IP event queue.
//
// W5100S MACRAW RX frame format (§W5100S datasheet §5.2.3):
//
//   Byte 0: frame_len_hi   ─┐  2-byte big-endian length
//   Byte 1: frame_len_lo   ─┘  (includes the 2-byte header itself)
//   Bytes 2..N: raw Ethernet frame
//
// So: ethernet_frame_bytes = frame_len - 2
//
// FreeRTOS+TCP expects us to:
//   1. Allocate a NetworkBufferDescriptor_t via pxGetNetworkBufferWithDescriptor().
//   2. Copy the Ethernet frame (NOT the length header) into it.
//   3. Send an eNetworkRxEvent to the IP task via xSendEventStructToIPTask().
// ============================================================

static void vNetworkInterfaceRxTask(void* arg)
{
    (void)arg;

    /* Stack-local frame buffer.  Stack size must accommodate this. */
    static uint8_t frame_buf[MACRAW_MAX_FRAME_BYTES];

    for (;;) {
        /* Check how many bytes are waiting in the W5100S RX buffer. */
        uint16_t rx_available = getSn_RX_RSR(MACRAW_SOCKET);

        if (rx_available < 2u) {
            /* Nothing available — yield and try again. */
            if (WIZNET_RX_POLL_MS > 0) {
                vTaskDelay(pdMS_TO_TICKS(WIZNET_RX_POLL_MS));
            } else {
                taskYIELD();
            }
            continue;
        }

        /* Read the 2-byte MACRAW length header first. */
        uint8_t len_hdr[2];
        int32_t n = recv(MACRAW_SOCKET, len_hdr, 2);
        if (n != 2) {
            /* Unexpected; flush the socket. */
            setSn_CR(MACRAW_SOCKET, Sn_CR_RECV);
            continue;
        }

        /* Length field is big-endian and includes the 2-byte header. */
        uint16_t total_len = ((uint16_t)len_hdr[0] << 8) | len_hdr[1];
        if (total_len < 2u) {
            setSn_CR(MACRAW_SOCKET, Sn_CR_RECV);
            continue;
        }
        uint16_t frame_len = total_len - 2u;

        if (frame_len == 0 || frame_len > MACRAW_MAX_FRAME_BYTES) {
            /* Oversized or empty — discard by reading and dropping. */
            if (frame_len > 0) {
                /* Read in chunks to clear the RX buffer. */
                uint16_t remaining = frame_len;
                while (remaining > 0) {
                    uint16_t chunk = remaining < sizeof(frame_buf)
                                     ? remaining : sizeof(frame_buf);
                    recv(MACRAW_SOCKET, frame_buf, chunk);
                    remaining -= chunk;
                }
            }
            continue;
        }

        /* Read the raw Ethernet frame into the local buffer. */
        n = recv(MACRAW_SOCKET, frame_buf, frame_len);
        if (n != (int32_t)frame_len) {
            /* Partial read — skip this frame. */
            continue;
        }

        /* Allocate a FreeRTOS+TCP network buffer descriptor. */
        NetworkBufferDescriptor_t* pxDescriptor =
            pxGetNetworkBufferWithDescriptor(frame_len, 0);

        if (pxDescriptor == NULL) {
            /* No buffer available — drop the frame.
             * Increment an error counter here if you have one. */
            iptraceETHERNET_RX_EVENT_LOST();
            continue;
        }

        /* Copy the Ethernet frame into the descriptor's buffer. */
        memcpy(pxDescriptor->pucEthernetBuffer, frame_buf, frame_len);
        pxDescriptor->xDataLength = frame_len;

        /* Send an eNetworkRxEvent to the FreeRTOS+TCP IP task. */
        IPStackEvent_t event;
        event.eEventType = eNetworkRxEvent;
        event.pvData     = (void*)pxDescriptor;

        if (xSendEventStructToIPTask(&event, 0) != pdTRUE) {
            /* IP task queue full — release the buffer. */
            vReleaseNetworkBufferAndDescriptor(pxDescriptor);
            iptraceETHERNET_RX_EVENT_LOST();
        } else {
            iptraceNETWORK_INTERFACE_RECEIVE();
        }
    }
}
