// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file boards/w6100-evb-pico2/board_config.h
 * @brief Hardware pin mapping for the WIZnet W6100-EVB-Pico2 (RP2350).
 *
 * The W6100-EVB-Pico2 wires the W6100 to SPI0 on the same pins the W5100S EVB
 * uses. The on-chip W6100 RST line is active-low.
 *
 * Reference: WIZnet W6100-EVB-Pico2 board documentation.
 */

#pragma once

// ── Processor ────────────────────────────────────────────────────────────────
#define BOARD_IS_RP2350         1
#define BOARD_IS_RP2040         0

// ── WIZnet part ──────────────────────────────────────────────────────────────
// Names the chip-trait struct (rosette::net::wiznet::<token>) the register-level
// driver builds against; see boards/common/rp2_wiznet/wiznet_board_types.hpp.
#define ROSETTE_WIZNET_CHIP        W6100

// ── SPI peripheral ───────────────────────────────────────────────────────────
// W6100 on SPI0: GPIO 16/17/18/19 are SPI0 RX/CSn/SCK/TX on RP2350.
#define ROSETTE_SPI_INST           spi0

#define ROSETTE_PIN_SPI_SCK        18
#define ROSETTE_PIN_SPI_TX         19   ///< MOSI
#define ROSETTE_PIN_SPI_RX         16   ///< MISO
#define ROSETTE_PIN_SPI_CS         17   ///< W6100 /CS (active-low, GPIO-driven)

// ── W6100 control lines ──────────────────────────────────────────────────────
#define ROSETTE_PIN_WIZNET_RST     20   ///< Active-low reset
#define ROSETTE_PIN_WIZNET_INT     21   ///< Active-low interrupt (optional)

// ── SPI clock speed ─────────────────────────────────────────────────────────
// W6100 maximum SPI clock: 33.3 MHz. Use 31.25 MHz (250/8) on RP2350.
#define ROSETTE_SPI_BAUDRATE_HZ    (31250000u)

// ── W6100 reset timing ──────────────────────────────────────────────────────
// Hold RST low well past the datasheet minimum, then wait for PLL lock.
#define ROSETTE_WIZNET_RST_HOLD_MS   2u
#define ROSETTE_WIZNET_RST_WAIT_MS   150u

// ── User LED ────────────────────────────────────────────────────────────────
#define ROSETTE_PIN_LED            25

// ── MAC address ─────────────────────────────────────────────────────────────
// WIZnet OUI 00:08:DC; last 3 bytes come from the RP2350 unique ID at runtime.
#define ROSETTE_MAC_ADDR_DEFAULT   { 0x02, 0x08, 0xDC, 0x00, 0x00, 0x01 }

// ── Static IP (DHCP is the WiznetLink default) ──────────────────────────────
#define ROSETTE_IP_ADDR            "192.168.1.200"
#define ROSETTE_IP_MASK            "255.255.255.0"
#define ROSETTE_IP_GW              "192.168.1.1"
#define ROSETTE_IP_DNS             "8.8.8.8"

// ── Persistent storage region (LittleFS) ────────────────────────────────────
// 2 MiB flash; reserve the top 256 KiB for rosette::storage.
#define ROSETTE_STORAGE_FLASH_OFFSET  0x001C0000u
#define ROSETTE_STORAGE_FLASH_SIZE    0x00040000u
