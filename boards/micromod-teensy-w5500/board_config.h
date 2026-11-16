// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file boards/micromod-teensy-w5500/board_config.h
 * @brief Hardware mapping for the SparkFun MicroMod Teensy with the MicroMod Ethernet Function
 * Board.
 *
 * The processor board (SparkFun DEV-16402) carries an NXP i.MX RT1062 and
 * plugs into a MicroMod Main Board; the Ethernet Function Board (SparkFun
 * COM-18708, WIZnet W5500) sits in function slot 0. The main board routes the
 * function board's SPI, chip select, interrupt, and reset to the processor's
 * MicroMod SPI_*, D0, and PWM0 signals; this file names the i.MX RT GPIOs
 * behind them (SparkFun MicroMod Teensy hookup guide; PJRC core_pins.h,
 * ARDUINO_TEENSY_MICROMOD):
 *
 *   W5500       MicroMod   Teensy pin   i.MX RT pad      function
 *   SCLK        SPI_SCK    13           GPIO_B0_03       LPSPI4_SCK
 *   MOSI        SPI_COPI   11           GPIO_B0_02       LPSPI4_SDO
 *   MISO        SPI_CIPO   12           GPIO_B0_01       LPSPI4_SDI
 *   SCSn        SPI_CS     10           GPIO_B0_00       GPIO2_IO00
 *   INTn        D0         4            GPIO_EMC_06      GPIO4_IO06
 *   RSTn        PWM0       3            GPIO_EMC_05      GPIO4_IO05
 *   console TX  TX1        1            GPIO_AD_B0_02    LPUART6_TX
 *   console RX  RX1        0            GPIO_AD_B0_03    LPUART6_RX
 *
 * printf() goes to two consoles: the processor's USB port, a CDC ACM serial
 * port (/dev/ttyACM*), and the MicroMod UART1 at 115200 8N1 on the main
 * board's TX1/RX1.
 *
 * The status LED (Teensy pin 13, GPIO_B0_03) shares the pad with SPI_SCK. It
 * flashes once as board_init() starts, before LPSPI4 takes the pad, which
 * shows the firmware got past the boot ROM and the C runtime.
 */

#pragma once

#include "imxrt_gpio_pin.h"

#define ROSETTE_PIN_STATUS_LED ROSETTE_IMXRT_GPIO(2, 3)  ///< Teensy pin 13, shared with SPI_SCK
#define ROSETTE_STATUS_LED_BOOT_FLASH_US 100000u

// ── WIZnet part ──────────────────────────────────────────────────────────────
// Names the chip-trait struct (rosette::net::wiznet::<token>) that the
// register-level driver builds against; see
// boards/common/wiznet/wiznet_board_types.hpp.
#define ROSETTE_WIZNET_CHIP W5500

// ── SPI ──────────────────────────────────────────────────────────────────────
#define ROSETTE_LPSPI_INST LPSPI4
#define ROSETTE_PIN_SPI_CS ROSETTE_IMXRT_GPIO(2, 0)  ///< W5500 SCSn, active low, GPIO-driven

// W5500 SPI clock: 80 MHz rated, 33.3 MHz guaranteed (datasheet §5.5.2). The
// signals cross two MicroMod connectors and the main board, so the bus runs
// well below that; LPSPI4 divides its 105.6 MHz root to the nearest rate at
// or below this one (17.6 MHz).
#define ROSETTE_SPI_BAUDRATE_HZ (20000000u)

// ── W5500 control lines ──────────────────────────────────────────────────────
#define ROSETTE_PIN_WIZNET_RST ROSETTE_IMXRT_GPIO(4, 5)  ///< RSTn, active low
#define ROSETTE_PIN_WIZNET_INT ROSETTE_IMXRT_GPIO(4, 6)  ///< INTn, active low, open drain

// W5500 datasheet §5.5.1: RSTn low for at least 500 µs; the PLL locks within
// 1 ms of its release.
#define ROSETTE_WIZNET_RST_HOLD_US 2000u
#define ROSETTE_WIZNET_RST_WAIT_US 50000u

// ── Console ──────────────────────────────────────────────────────────────────
#define ROSETTE_CONSOLE_LPUART LPUART6
#define ROSETTE_CONSOLE_BAUD 115200u

// ── MAC address ──────────────────────────────────────────────────────────────
// Every Teensy 4.x carries a PJRC-assigned MAC in its OCOTP MAC0/MAC1 fuses
// (OUI 04:E9:E5), which board_get_mac_address() uses. This locally
// administered address is the fallback for a part with blank MAC fuses, and
// the MACRAW composition's construction-time address.
#define ROSETTE_MAC_ADDR_DEFAULT {0x02, 0x04, 0xE9, 0x00, 0x00, 0x01}

// ── Persistent storage ───────────────────────────────────────────────────────
// None: the i.MX RT port reserves no flash region (ports/chip/imxrt/imxrt_flash_region.c).
#define ROSETTE_STORAGE_FLASH_OFFSET 0x00000000u
#define ROSETTE_STORAGE_FLASH_SIZE 0x00000000u
