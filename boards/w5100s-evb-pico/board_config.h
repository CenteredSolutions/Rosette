/**
 * @file boards/w5100s-evb-pico/board_config.h
 * @brief Hardware pin mapping for the WIZnet W5100S-EVB-Pico (RP2040).
 *
 * Identical wiring to the Pico2 EVB; the only difference is the processor
 * variant, which affects the SDK target and clock speeds.
 */

#pragma once

#define BOARD_IS_RP2350         0
#define BOARD_IS_RP2040         1

#define ROSETTE_SPI_INST           spi1
#define ROSETTE_PIN_SPI_SCK        18
#define ROSETTE_PIN_SPI_TX         19
#define ROSETTE_PIN_SPI_RX         16
#define ROSETTE_PIN_SPI_CS         17

#define ROSETTE_PIN_W5100S_RST     20
#define ROSETTE_PIN_W5100S_INT     21

// RP2040 sys_clk is 125 MHz; SPI1 can achieve up to 62.5 MHz.
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
