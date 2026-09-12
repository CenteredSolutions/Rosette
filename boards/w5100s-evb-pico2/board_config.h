/**
 * @file boards/w5100s-evb-pico2/board_config.h
 * @brief Hardware pin mapping for the WIZnet W5100S-EVB-Pico2.
 *
 * The W5100S-EVB-Pico2 is the RP2350 variant of WIZnet's reference board.
 * The W5100S is wired to SPI1 at the same pins used on the original Pico EVB.
 * The on-chip W5100S RST line is active-low.
 *
 * Reference schematic: WIZnet W5100S-EVB-Pico2 v1.0
 */

#pragma once

// ── Processor ────────────────────────────────────────────────────────────────
#define BOARD_IS_RP2350         1
#define BOARD_IS_RP2040         0

// ── WIZnet part ──────────────────────────────────────────────────────────────
// Names the chip-trait struct (rosette::net::wiznet::<token>) that the
// register-level driver builds against; see include/rosette/net/wiznet/
// chip_select.hpp. The ioLibrary equivalent is the ROSETTE_WIZNET_CHIP cache
// variable in cmake/wiznet.cmake.
#define ROSETTE_WIZNET_CHIP        W5100S

// ── SPI peripheral ───────────────────────────────────────────────────────────
// The W5100S is on SPI0: GPIO 16/17/18/19 are SPI0 RX/CSn/SCK/TX on RP2350
// (matches the pico-sdk wiznet_w5100s_evb_pico2 board header).
#define ROSETTE_SPI_INST           spi0

#define ROSETTE_PIN_SPI_SCK        18   ///< SPI1 SCK
#define ROSETTE_PIN_SPI_TX         19   ///< SPI1 TX (MOSI)
#define ROSETTE_PIN_SPI_RX         16   ///< SPI1 RX (MISO)
#define ROSETTE_PIN_SPI_CS         17   ///< W5100S /CS (active-low, GPIO-driven)

// ── W5100S control lines ──────────────────────────────────────────────────────
#define ROSETTE_PIN_W5100S_RST     20   ///< Active-low reset
#define ROSETTE_PIN_W5100S_INT     21   ///< Active-low interrupt (optional)

// ── SPI clock speed ───────────────────────────────────────────────────────────
// W5100S maximum SPI clock: 33.3 MHz.  Use 31.25 MHz (250/8) on RP2350.
#define ROSETTE_SPI_BAUDRATE_HZ    (31250000u)

// ── W5100S reset timing ───────────────────────────────────────────────────────
// W5100S datasheet §5.5: RST must be held low >= 500 µs.
// Hold for 2 ms to be safe; wait 150 ms after de-assert for PLL lock.
#define ROSETTE_W5100S_RST_HOLD_MS   2u
#define ROSETTE_W5100S_RST_WAIT_MS   150u

// ── User LED ──────────────────────────────────────────────────────────────────
#define ROSETTE_PIN_LED            25

// ── MAC address ───────────────────────────────────────────────────────────────
// WIZnet OUI: 00:08:DC.  The last 3 bytes should be set from the RP2350's
// unique ID at runtime — see rosette_platform_make_guid_prefix().
// This default is used only as a fallback if the unique ID cannot be read.
#define ROSETTE_MAC_ADDR_DEFAULT   { 0x02, 0x08, 0xDC, 0x00, 0x00, 0x01 }

// ── FreeRTOS+TCP static IP (change or enable DHCP via FreeRTOSIPConfig.h) ────
#define ROSETTE_IP_ADDR            "192.168.1.200"
#define ROSETTE_IP_MASK            "255.255.255.0"
#define ROSETTE_IP_GW              "192.168.1.1"
#define ROSETTE_IP_DNS             "8.8.8.8"

// ── Address bring-up ───────────────────────────────────────────────────────
// There is no addressing policy here: firmware declares its own list of
// rosette::net::AddressProviders in main() (DHCP, static, zeroconf, …) and
// hands it to an AddressManager. See examples/led_files/main.cpp.

// ── Persistent storage region (LittleFS) ────────────────────────────────────
// Reserved for rosette::storage. The application linker script / binary must
// stay below this offset. The W5100S-EVB-Pico2 has 2 MiB of flash
// (PICO_FLASH_SIZE_BYTES in the pico-sdk board header); reserve the top 256 KiB.
#define ROSETTE_STORAGE_FLASH_OFFSET  0x001C0000u
#define ROSETTE_STORAGE_FLASH_SIZE    0x00040000u
