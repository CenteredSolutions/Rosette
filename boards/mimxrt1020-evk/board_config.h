/**
 * @file boards/mimxrt1020-evk/board_config.h
 * @brief Hardware mapping for the NXP MIMXRT1020-EVK.
 *
 * The board carries an i.MX RT1021 (MIMXRT1021DAG5A) whose ENET MAC reaches a
 * Microchip KSZ8081RNB PHY over RMII; the RT1021 supplies the PHY's 50 MHz
 * reference clock. The on-board DAPLink probe flashes the part over SWD and
 * bridges LPUART1 to a USB virtual serial port. Pads (MIMXRT1020-EVK
 * schematic; MicroPython ports/mimxrt/boards/MIMXRT1020_EVK):
 *
 *   function      i.MX RT pad        GPIO / peripheral
 *   user LED      GPIO_AD_B0_05      GPIO1_IO05, active low
 *   PHY RST_B     GPIO_AD_B0_04      GPIO1_IO04, active low
 *   PHY INTRP     GPIO_AD_B1_06      GPIO1_IO22, active low
 *   RMII          GPIO_AD_B0_08..15  ENET_REF_CLK1, RDATA01/00, RX_EN, RX_ER,
 *                                    TX_EN, TDATA00/01
 *   MDIO, MDC     GPIO_EMC_40, _41   ENET_MDIO, ENET_MDC
 *   console TX    GPIO_AD_B0_06      LPUART1_TX
 *   console RX    GPIO_AD_B0_07      LPUART1_RX
 *
 * printf() goes to LPUART1 at 115200 8N1 (/dev/ttyACM* on the host the
 * probe's USB port is plugged into).
 */

#pragma once

#include "imxrt_gpio_pin.h"

#define ROSETTE_PIN_STATUS_LED ROSETTE_IMXRT_GPIO(1, 5)  ///< user LED, active low
#define ROSETTE_STATUS_LED_BOOT_FLASH_US 100000u

// ── Ethernet PHY ─────────────────────────────────────────────────────────────
#define ROSETTE_PIN_PHY_RST ROSETTE_IMXRT_GPIO(1, 4)   ///< KSZ8081 RST_B, active low
#define ROSETTE_PIN_PHY_INT ROSETTE_IMXRT_GPIO(1, 22)  ///< KSZ8081 INTRP, active low
#define ROSETTE_PHY_MDIO_ADDRESS 2u                    ///< KSZ8081 PHYAD strapping

// ENET DMA rings. The receive ring absorbs a burst while the receive task
// runs; the transmit ring holds frames the wire has not taken yet.
// Zero copy (enet_zero_copy_interface.cpp, the default): the receive ring holds
// that many of the stack's network buffers, and a transmit descriptor costs 12
// octets, since it points at the stack's buffer.
// Copying (enet_copying_interface.cpp): each RX and TX buffer is 1536 octets
// of DTCM that the driver owns.
#define ROSETTE_ENET_RX_BUFFERS 4u
#define ROSETTE_ENET_TX_DESCRIPTORS 4u
#define ROSETTE_ENET_TX_BUFFERS 2u

// ── Console ──────────────────────────────────────────────────────────────────
#define ROSETTE_CONSOLE_LPUART LPUART1
#define ROSETTE_CONSOLE_BAUD 115200u

// ── MAC address ──────────────────────────────────────────────────────────────
// The EVK's OCOTP MAC fuses are blank; board_get_mac_address() keeps this
// locally administered prefix and takes the last three octets from the fuse UID.
#define ROSETTE_MAC_ADDR_DEFAULT {0x02, 0x10, 0x21, 0x00, 0x00, 0x01}

// ── Persistent storage ───────────────────────────────────────────────────────
// None: the i.MX RT port reserves no flash region (ports/chip/imxrt/imxrt_flash_region.c).
#define ROSETTE_STORAGE_FLASH_OFFSET 0x00000000u
#define ROSETTE_STORAGE_FLASH_SIZE 0x00000000u
