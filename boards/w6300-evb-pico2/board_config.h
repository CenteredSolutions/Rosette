/**
 * @file boards/w6300-evb-pico2/board_config.h
 * @brief Hardware pin mapping for the WIZnet W6300-EVB-Pico2 (RP2350).
 *
 * PROVISIONAL. The W6300's host interface is QSPI. This board config drives it
 * single-lane (1-bit) through a GPIO bit-bang bus (`PicoQspiBus`), which is
 * correct but slow. Full-rate 4-bit QSPI needs a PIO program — phase 2 — after
 * which only `ROSETTE_WIZNET_BUS_QSPI`'s implementation changes, not this file.
 *
 * The EVB connects the W6300 to GPIO15..22 (SCK 17 · IO0 18 · IO1 19 · CS 16 ·
 * RST 22). That is not a hardware-SPI pin set, hence the bit-bang bus.
 *
 * Reference: WIZnet W6300-EVB-Pico2 board documentation.
 */

#pragma once

// ── Processor ────────────────────────────────────────────────────────────────
#define BOARD_IS_RP2350         1
#define BOARD_IS_RP2040         0

// ── WIZnet part ──────────────────────────────────────────────────────────────
#define ROSETTE_WIZNET_CHIP        W6300

// Select the QSPI bus (single-lane bit-bang for now); see board.hpp.
#define ROSETTE_WIZNET_BUS_QSPI    1

// ── W6300 QSPI pins ─────────────────────────────────────────────────────────
// Reused SPI-pin macro names so board.hpp's make_board_bus() can stay generic.
#define ROSETTE_PIN_SPI_SCK        17
#define ROSETTE_PIN_SPI_TX         18   ///< IO0 / MOSI (single-lane)
#define ROSETTE_PIN_SPI_RX         19   ///< IO1 / MISO (single-lane)
#define ROSETTE_PIN_SPI_CS         16   ///< W6300 /CS (active-low)

// Not used on this board (the QSPI bus is bit-banged), kept for code that
// unconditionally references the symbol.
#define ROSETTE_SPI_INST           spi0
#define ROSETTE_SPI_BAUDRATE_HZ    (20000000u)

// ── W6300 control lines ─────────────────────────────────────────────────────
#define ROSETTE_PIN_WIZNET_RST     22   ///< Active-low reset
#define ROSETTE_PIN_WIZNET_INT     15   ///< Active-low interrupt (optional)

// ── W6300 reset timing ──────────────────────────────────────────────────────
#define ROSETTE_WIZNET_RST_HOLD_MS   2u
#define ROSETTE_WIZNET_RST_WAIT_MS   150u

// ── User LED ────────────────────────────────────────────────────────────────
#define ROSETTE_PIN_LED            25

// ── MAC address ─────────────────────────────────────────────────────────────
#define ROSETTE_MAC_ADDR_DEFAULT   { 0x02, 0x08, 0xDC, 0x00, 0x00, 0x01 }

// ── Static IP (DHCP is the WiznetLink default) ──────────────────────────────
#define ROSETTE_IP_ADDR            "192.168.1.200"
#define ROSETTE_IP_MASK            "255.255.255.0"
#define ROSETTE_IP_GW              "192.168.1.1"
#define ROSETTE_IP_DNS             "8.8.8.8"

// ── Persistent storage region (LittleFS) ────────────────────────────────────
#define ROSETTE_STORAGE_FLASH_OFFSET  0x001C0000u
#define ROSETTE_STORAGE_FLASH_SIZE    0x00040000u
