// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/net/freertos_tcp/FreeRTOSIPConfig.h
 * @brief FreeRTOS+TCP configuration for the RTPS stack.
 *
 * Tuned for a UDP-only workload (RTPS uses UDP; TCP is not required).
 * TCP support is disabled to save ~6 KB of flash and reduce stack usage.
 *
 * Network buffer sizing
 * ---------------------
 * ipconfigNETWORK_MTU = 1500 (standard Ethernet MTU), so a network buffer
 * holds ipTOTAL_ETHERNET_FRAME_SIZE = 1522 octets of frame.
 * Buffer descriptors: 10 (covers simultaneous SPDP, SEDP, DATA, ACKNACKs).
 * BufferAllocation_2 (the default) takes each from the FreeRTOS heap as
 * needed; a board whose MAC lends the stack's buffers
 * (ROSETTE_FREERTOS_TCP_LENT_BUFFERS, ports/net/freertos_tcp/port.cmake)
 * uses BufferAllocation_1 and places all ten in its own DMA-reachable RAM.
 *
 * DHCP vs static IP
 * -----------------
 * ipconfigUSE_DHCP = 1 is the default.  For a deterministic embedded
 * deployment you may set it to 0 and use the static IP from board_config.h.
 * The IP address is supplied to FreeRTOS_IPInit() at startup.
 */

#ifndef FREERTOS_IP_CONFIG_H
#define FREERTOS_IP_CONFIG_H

// ── Basic network configuration ───────────────────────────────────────────────
#define ipconfigUSE_IPv4                        1
#define ipconfigUSE_IPv6                        1
#define ipconfigBYTE_ORDER                      pdFREERTOS_LITTLE_ENDIAN

// ── Ethernet / MAC ────────────────────────────────────────────────────────────
#define ipconfigNETWORK_MTU 1500u
#define ipconfigETHERNET_MINIMUM_PACKET_BYTES   0u

// ── Network buffer pool ───────────────────────────────────────────────────────
// Buffers are allocated from the FreeRTOS heap by FreeRTOS+TCP at init.
// 10 descriptors: enough for concurrent SPDP + SEDP + 4 data + 4 control.
#define ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS  10u
#define ipconfigNETWORK_BUFFER_SIZE             ( ipconfigNETWORK_MTU + 36u )

// ── IP task ───────────────────────────────────────────────────────────────────
#define ipconfigIP_TASK_PRIORITY                ( configMAX_PRIORITIES - 2 )
#define ipconfigIP_TASK_STACK_SIZE_WORDS        512u
#define ipconfigEVENT_QUEUE_LENGTH              16u

// ── UDP ───────────────────────────────────────────────────────────────────────
#define ipconfigUSE_UDP                         1
#define ipconfigUDP_MAX_SEND_BLOCK_TIME_TICKS   pdMS_TO_TICKS(100u)
// A send waits at most that long for a network buffer, then fails. The
// stack's default is portMAX_DELAY, and a task that both receives and sends
// would then wait forever for a buffer when its own socket's queue holds the
// rest of the pool, since only that task drains it.
#define ipconfigSOCK_DEFAULT_SEND_BLOCK_TIME ipconfigUDP_MAX_SEND_BLOCK_TIME_TICKS
// A flooded socket keeps at most this many datagrams queued and drops the
// rest, so it cannot take the whole pool from the other sockets and from
// transmit. With a lending MAC, ROSETTE_ENET_RX_BUFFERS of the ten buffers sit
// in the receive ring, which leaves this much for one socket and two for sends.
#define ipconfigUDP_MAX_RX_PACKETS 4u
#define ipconfigUDP_TIME_TO_LIVE                128u
#define ipconfigUDP_MULTICAST_TTL               1u   /* Local subnet only. */

// ── Multicast (required for SPDP 239.255.0.1) ────────────────────────────────
#define ipconfigUSE_IGMP                        1
#define ipconfigIS_VALID_PROG_ADDRESS(x)        1

// ── TCP — disabled (RTPS is UDP-only) ────────────────────────────────────────
#define ipconfigUSE_TCP                         0

// ── DHCP ─────────────────────────────────────────────────────────────────────
#define ipconfigUSE_RA                          1
#define ipconfigUSE_DHCP                        1
#define ipconfigDHCP_REGISTER_HOSTNAME          1
#define ipconfigMAXIMUM_DISCOVER_TX_PERIOD      pdMS_TO_TICKS(5000u)
// RFC 3927 AutoIP fallback when DHCP finds no server. The board network
// (ports/net/freertos_tcp/board_network.cpp) reports the resulting
// 169.254/16 address with AddressSource::zeroconf.
#define ipconfigDHCP_FALL_BACK_AUTO_IP          1
// The board network decides DHCP or a fixed address per endpoint before the
// IP task starts, so the stack needs no per-phase DHCP callback.
#define ipconfigUSE_DHCP_HOOK 0

// ── DNS ───────────────────────────────────────────────────────────────────────
#define ipconfigUSE_DNS                         1
#define ipconfigDNS_REQUEST_ATTEMPTS            2u

// ── ARP ───────────────────────────────────────────────────────────────────────
#define ipconfigARP_CACHE_ENTRIES               8u
#define ipconfigARP_MAX_ARP_RETRANSMISSIONS     5u
#define ipconfigARP_STORES_REMOTE_ADDRESSES     0

// ── Checksum offload ──────────────────────────────────────────────────────────
// Neither the WIZnet chips in MACRAW mode nor the MAC devices here offload
// checksums, so FreeRTOS+TCP computes them in software.
#define ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM  0
#define ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM  0

// ── Zero-copy ─────────────────────────────────────────────────────────────────
// A copying interface (interface.hpp) copies in both directions. A lending
// interface (lending_interface.hpp) sends the stack's own buffers, so it needs
// pfOutput to own every buffer it is given: ipconfigZERO_COPY_TX_DRIVER = 1,
// which port.cmake defines for a board with ROSETTE_FREERTOS_TCP_LENT_BUFFERS.
// Receive needs no setting: a received frame arrives in a network buffer
// either way.
#define ipconfigZERO_COPY_RX_DRIVER             0
#ifndef ipconfigZERO_COPY_TX_DRIVER
#  define ipconfigZERO_COPY_TX_DRIVER 0
#endif

// ── Socket wake-up ────────────────────────────────────────────────────────────
// FreeRtosSocketProvider::wait_readable() points each socket's user semaphore
// at the caller's platform::Event, which the stack gives once it has queued a
// datagram (ports/net/freertos_tcp/socket.hpp).
#define ipconfigSOCKET_HAS_USER_SEMAPHORE 1

// ── Logging / debug ───────────────────────────────────────────────────────────
#define ipconfigHAS_PRINTF                      0
#define ipconfigHAS_DEBUG_PRINTF                0

// ── Packet logger (optional; set to 1 for Wireshark capture support) ─────────
#define ipconfigFILTER_OUT_NON_ETHERNET_II_FRAMES  1
#define ipconfigETHERNET_DRIVER_FILTERS_PACKETS     0

#endif /* FREERTOS_IP_CONFIG_H */
