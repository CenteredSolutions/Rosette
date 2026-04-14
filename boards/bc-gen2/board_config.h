/**
 * @file boards/custom-pico2/board_config.h
 * @brief Hardware pin mapping for the custom RP2350-based board.
 *
 * This board uses RP2350 with a W5100S connected to SPI1, but on
 * different GPIO pins than the WIZnet EVB reference design.
 * Adjust the pin numbers to match your schematic.
 *
 * Additional hardware on this board (not present on the EVB):
 *   - External SPI flash (QSPI, handled by the boot stage — no config needed)
 *   - RS-485 transceiver on UART0 (not used by the RTPS stack)
 *   - Board-specific LED on GPIO 14
 */

#pragma once

#define BOARD_IS_RP2350         1
#define BOARD_IS_RP2040         0

// W5100S wired to SPI1 on non-standard pins.
#define RTPS_SPI_INST           spi1
#define RTPS_PIN_SPI_SCK        10
#define RTPS_PIN_SPI_TX         11   // MOSI
#define RTPS_PIN_SPI_RX          8   // MISO
#define RTPS_PIN_SPI_CS          9

#define RTPS_PIN_W5100S_RST     12
#define RTPS_PIN_W5100S_INT     13

// RP2350 at 150 MHz → SPI1 can reach 75 MHz; W5100S max = 33.3 MHz.
#define RTPS_SPI_BAUDRATE_HZ    (31250000u)

#define RTPS_W5100S_RST_HOLD_MS   2u
#define RTPS_W5100S_RST_WAIT_MS   150u

// Board-specific status LED.
#define RTPS_PIN_LED            14

// MAC derived from RP2350 unique ID at runtime; this is the static fallback.
#define RTPS_MAC_ADDR_DEFAULT   { 0x00, 0x08, 0xDC, 0x00, 0x00, 0x03 }

#define RTPS_IP_ADDR            "192.168.1.202"
#define RTPS_IP_MASK            "255.255.255.0"
#define RTPS_IP_GW              "192.168.1.1"
#define RTPS_IP_DNS             "8.8.8.8"

// ── Custom board extras ────────────────────────────────────────────────────────
// RS-485 direction pin (not used by RTPS).
#define CUSTOM_PIN_RS485_DE     15
