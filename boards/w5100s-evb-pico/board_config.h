/**
 * @file boards/w5100s-evb-pico/board_config.h
 * @brief Hardware pin mapping for the WIZnet W5100S-EVB-Pico (RP2040).
 *
 * No-RTOS poll-mode board — same WIZnet wiring as the RP2350 EVB; the RP2040
 * variant is the debugger-friendly one. See w5100s-evb-pico2 for the RP2350.
 */

#pragma once

#define BOARD_IS_RP2350         0
#define BOARD_IS_RP2040         1

// Names the chip-trait struct (rosette::net::wiznet::<token>) the register-level
// driver builds against; see include/rosette/net/wiznet/chip_select.hpp.
#define ROSETTE_WIZNET_CHIP        W5100S

// GPIO 16/17/18/19 are SPI0 RX/CSn/SCK/TX (matches the pico-sdk
// wiznet_w5100s_evb_pico board header).
#define ROSETTE_SPI_INST           spi0
#define ROSETTE_PIN_SPI_SCK        18
#define ROSETTE_PIN_SPI_TX         19
#define ROSETTE_PIN_SPI_RX         16
#define ROSETTE_PIN_SPI_CS         17

#define ROSETTE_PIN_W5100S_RST     20
#define ROSETTE_PIN_WIZNET_INT     21

// RP2040 sys_clk is 125 MHz; SPI0 can achieve up to 62.5 MHz.
// W5100S max is 33.3 MHz — use 31.25 MHz (125/4).
#define ROSETTE_SPI_BAUDRATE_HZ    (31250000u)

#define ROSETTE_W5100S_RST_HOLD_MS   2u
#define ROSETTE_W5100S_RST_WAIT_MS   150u

#define ROSETTE_PIN_LED            25

#define ROSETTE_MAC_ADDR_DEFAULT   { 0x00, 0x08, 0xDC, 0x00, 0x00, 0x02 }

#define ROSETTE_IP_ADDR            "192.168.1.201"
#define ROSETTE_IP_MASK            "255.255.255.0"
#define ROSETTE_IP_GW              "192.168.1.1"
#define ROSETTE_IP_DNS             "8.8.8.8"

// ── Persistent storage region (LittleFS) ────────────────────────────────────
// Reserved for rosette::storage. The application image must stay below this
// offset. RP2040 boards here carry 2 MiB of flash; reserve the top 256 KiB.
#define ROSETTE_STORAGE_FLASH_OFFSET  0x001C0000u
#define ROSETTE_STORAGE_FLASH_SIZE    0x00040000u
