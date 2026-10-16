/**
 * @file ports/chip/imxrt/imxrt_usb_descriptors.c
 * @brief USB descriptors for the i.MX RT port's console: one CDC ACM function, high and full speed.
 *
 * TinyUSB asks for the descriptors through the tud_descriptor_*_cb()
 * callbacks below during enumeration. The device is a single CDC ACM
 * function (a notification endpoint, a bulk OUT, a bulk IN) behind an
 * interface association, so every host binds its stock serial driver to it:
 * /dev/ttyACM* on Linux, /dev/cu.usbmodem* on macOS, a COM port on Windows.
 *
 * The controller runs at high speed (bulk endpoints of 512 octets) and falls
 * back to full speed (64 octets) behind a full-speed hub; the device
 * qualifier and other-speed descriptors describe the speed not in use, as
 * USB 2.0 §9.6.2 and §9.6.4 require of a high-speed-capable device.
 *
 * Identity: ROSETTE_IMXRT_USB_VID / _PID / _PRODUCT from the board
 * (board.cmake), else the pid.codes test VID/PID 1209:0001, which is for
 * private development only. The serial number is the chip's fuse unique ID
 * (OCOTP CFG0/CFG1) in hex, so two boards on one host keep stable, distinct
 * port names (/dev/serial/by-id/).
 *
 * References: USB 2.0 §9.6 (standard descriptors); USB IAD ECN; USB CDC 1.2
 * §5 and PSTN 1.2 §5.3 (ACM functional descriptors); TinyUSB
 * src/device/usbd.h (TUD_CDC_DESCRIPTOR, callbacks); pid.codes 1209:0001.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "fsl_device_registers.h"
#include "tusb.h"

#ifndef ROSETTE_IMXRT_USB_VID
#  define ROSETTE_IMXRT_USB_VID 0x1209u
#endif

#ifndef ROSETTE_IMXRT_USB_PID
#  define ROSETTE_IMXRT_USB_PID 0x0001u
#endif

#ifndef ROSETTE_IMXRT_USB_PRODUCT
#  define ROSETTE_IMXRT_USB_PRODUCT "Rosette console"
#endif

enum {
    kItfCdc = 0,
    kItfCdcData,
    kItfCount,
};

enum {
    kEpCdcNotify = 0x81,
    kEpCdcOut = 0x02,
    kEpCdcIn = 0x82,
};

enum {
    kStrLanguage = 0,
    kStrManufacturer,
    kStrProduct,
    kStrSerial,
    kStrCdc,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const tusb_desc_device_t kDeviceDescriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    // Miscellaneous / Common / IAD: the function is described by its
    // interface association.
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = ROSETTE_IMXRT_USB_VID,
    .idProduct = ROSETTE_IMXRT_USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = kStrManufacturer,
    .iProduct = kStrProduct,
    .iSerialNumber = kStrSerial,
    .bNumConfigurations = 1,
};

static const tusb_desc_device_qualifier_t kDeviceQualifier = {
    .bLength = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 1,
    .bReserved = 0,
};

static const uint8_t kConfigHighSpeed[] = {
    TUD_CONFIG_DESCRIPTOR(1, kItfCount, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(kItfCdc, kStrCdc, kEpCdcNotify, 8, kEpCdcOut, kEpCdcIn, 512),
};

static const uint8_t kConfigFullSpeed[] = {
    TUD_CONFIG_DESCRIPTOR(1, kItfCount, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(kItfCdc, kStrCdc, kEpCdcNotify, 8, kEpCdcOut, kEpCdcIn, 64),
};

/// Other-speed configuration: the speed not in use, with bDescriptorType rewritten.
static uint8_t g_other_speed_config[CONFIG_TOTAL_LEN];

/// String descriptor scratch: header plus up to 31 UTF-16 code units.
static uint16_t g_string[32];

/// @brief The device descriptor (USB 2.0 §9.6.1).
const uint8_t* tud_descriptor_device_cb(void) {
    return (const uint8_t*)&kDeviceDescriptor;
}

/// @brief The device qualifier: what the device would report at the other speed (§9.6.2).
const uint8_t* tud_descriptor_device_qualifier_cb(void) {
    return (const uint8_t*)&kDeviceQualifier;
}

/// @brief The configuration for the speed the bus enumerated at (§9.6.3).
const uint8_t* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return (tud_speed_get() == TUSB_SPEED_HIGH) ? kConfigHighSpeed : kConfigFullSpeed;
}

/// @brief The configuration for the speed not in use, typed OTHER_SPEED_CONFIG (§9.6.4).
const uint8_t* tud_descriptor_other_speed_configuration_cb(uint8_t index) {
    (void)index;
    const uint8_t* other =
        (tud_speed_get() == TUSB_SPEED_HIGH) ? kConfigFullSpeed : kConfigHighSpeed;
    memcpy(g_other_speed_config, other, CONFIG_TOTAL_LEN);
    g_other_speed_config[1] = TUSB_DESC_OTHER_SPEED_CONFIG;
    return g_other_speed_config;
}

/// @brief Writes the fuse unique ID (CFG1:CFG0) as 16 hex digits into `out`.
static uint8_t serial_number_utf16(uint16_t* out) {
    static const char kHex[] = "0123456789ABCDEF";
    const uint64_t    uid = ((uint64_t)OCOTP->CFG1 << 32) | OCOTP->CFG0;
    for (uint8_t i = 0; i < 16u; ++i) {
        out[i] = (uint16_t)kHex[(uid >> (60u - 4u * i)) & 0xFu];
    }

    return 16u;
}

/**
 * @brief A string descriptor (USB 2.0 §9.6.7) in UTF-16LE, built in a static buffer.
 *
 * @param[in] index  String index from a descriptor (kStr*); 0 is the language list.
 * @param[in] langid Requested language; every string is US English (0x0409).
 * @return The descriptor, valid until the next call; NULL for an unknown index,
 *   which TinyUSB answers with a STALL.
 */
const uint16_t* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t length = 0;
    if (index == kStrLanguage) {
        g_string[1] = 0x0409;
        length = 1;
    } else if (index == kStrSerial) {
        length = serial_number_utf16(&g_string[1]);
    } else {
        const char* text = NULL;
        if (index == kStrManufacturer) {
            text = "Rosette";
        } else if (index == kStrProduct) {
            text = ROSETTE_IMXRT_USB_PRODUCT;
        } else if (index == kStrCdc) {
            text = "Rosette console";
        } else {
            return NULL;
        }

        while (text[length] != '\0' && length < 31u) {
            g_string[1 + length] = (uint16_t)text[length];
            ++length;
        }
    }

    g_string[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2u * length + 2u));
    return g_string;
}
