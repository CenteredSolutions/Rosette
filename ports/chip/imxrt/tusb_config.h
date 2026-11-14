// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/imxrt/tusb_config.h
 * @brief TinyUSB configuration for the i.MX RT port's USB console: one CDC ACM device, no RTOS.
 *
 * TinyUSB includes this header by name. The port runs TinyUSB as a device on
 * USB_OTG1 at high speed with one CDC interface (imxrt_usb_console.c,
 * imxrt_usb_descriptors.c). CFG_TUSB_MCU (OPT_MCU_MIMXRT1XXX) comes from the
 * build (port.cmake).
 *
 * The transmit FIFO is the console's backlog: it keeps the newest
 * CFG_TUD_CDC_TX_BUFSIZE octets while no terminal holds the port open
 * (TinyUSB overwrites the oldest), and survives the terminal closing, so the
 * boot log is waiting when a terminal opens it.
 *
 * References: TinyUSB src/tusb_option.h and src/class/cdc/cdc_device.h (the
 * CFG_* options); USB CDC 1.2 and PSTN 1.2 (ACM).
 */

#ifndef ROSETTE_IMXRT_TUSB_CONFIG_H
#define ROSETTE_IMXRT_TUSB_CONFIG_H

#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_HIGH_SPEED
#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC 1
#define CFG_TUD_CDC_EP_BUFSIZE 512
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 4096
#define CFG_TUD_CDC_TX_PERSISTENT 1

#endif  // ROSETTE_IMXRT_TUSB_CONFIG_H
