/**
 * @file boards/mimxrt1020-evk/board_init.c
 * @brief Board bring-up for the NXP MIMXRT1020-EVK.
 *
 * Muxes LPUART1 to the pads the DAPLink probe bridges, starts it as the
 * console, prints the reset cause, and flashes the user LED. The clock tree
 * and the time base are the chip port's, already running before main()
 * (ports/chip/imxrt/chip_imxrt.cpp).
 *
 * Pad settings (IOMUXC SW_PAD_CTL): outputs use drive strength R0/6 at
 * medium speed with the keeper on; inputs add the 47 kΩ pull-up and
 * hysteresis.
 *
 * References: i.MX RT1020 Reference Manual §10 (IOMUXC SW_MUX_CTL /
 * SW_PAD_CTL), §21.8.3 (OCOTP MAC0/MAC1); MIMXRT1020-EVK schematic (LPUART1,
 * USER_LED); MCUXpresso SDK fsl_iomuxc.h.
 */

#include <stdio.h>

#include "board.h"
#include "board_config.h"

#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "imxrt_console.h"

/// Output pad: keeper, medium speed, drive strength R0/6.
#define BOARD_PAD_OUTPUT \
    (IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(6))

/// Input pad with the 47 kΩ pull-up and hysteresis.
#define BOARD_PAD_PULLUP_INPUT                                                                  \
    (IOMUXC_SW_PAD_CTL_PAD_HYS(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1) \
     | IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_SPEED(2)                            \
     | IOMUXC_SW_PAD_CTL_PAD_DSE(6))

static void board_init_console(void) {
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_06_LPUART1_TX, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_07_LPUART1_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_06_LPUART1_TX, BOARD_PAD_OUTPUT);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_07_LPUART1_RX, BOARD_PAD_PULLUP_INPUT);
    (void)rosette_imxrt_console_init(ROSETTE_CONSOLE_LPUART, ROSETTE_CONSOLE_BAUD,
                                     BOARD_BOOTCLOCKRUN_UART_CLK_ROOT);
}

void board_init(void) {
    CLOCK_EnableClock(kCLOCK_Iomuxc);
    board_init_console();
    printf("[board] MIMXRT1020-EVK, core %lu Hz, reset cause SRC_SRSR=0x%08lx\n",
           (unsigned long)SystemCoreClock, (unsigned long)SRC->SRSR);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05, BOARD_PAD_OUTPUT);
    const gpio_pin_config_t led = {kGPIO_DigitalOutput, 1U, kGPIO_NoIntmode};
    GPIO_PinInit(rosette_imxrt_gpio_port(ROSETTE_PIN_STATUS_LED),
                 rosette_imxrt_gpio_bit(ROSETTE_PIN_STATUS_LED), &led);
    board_set_led(true);
    SDK_DelayAtLeastUs(ROSETTE_STATUS_LED_BOOT_FLASH_US, SystemCoreClock);
    board_set_led(false);
}

/**
 * @brief The OCOTP MAC fuses when programmed, or a locally administered address from the fuse UID.
 *
 * MAC1[15:0] holds the top two octets and MAC0 the low four. The EVK ships
 * with them blank, so it gets ROSETTE_MAC_ADDR_DEFAULT's prefix with its last
 * three octets from the UID, which differ from one board to the next.
 *
 * @param[out] mac Six octets, network order.
 *
 * Verification:
 * - Correct means: the DHCP server's lease table shows the address the boot
 *   log prints, and two EVKs print different addresses.
 *
 * References: i.MX RT1020 Reference Manual §21.8.3 (HW_OCOTP_MAC0/MAC1).
 */
void board_get_mac_address(uint8_t mac[6]) {
    const uint32_t mac0 = OCOTP->MAC0;
    const uint32_t mac1 = OCOTP->MAC1;
    if ((mac0 | (mac1 & 0xFFFFU)) != 0U) {
        mac[0] = (uint8_t)(mac1 >> 8);
        mac[1] = (uint8_t)mac1;
        mac[2] = (uint8_t)(mac0 >> 24);
        mac[3] = (uint8_t)(mac0 >> 16);
        mac[4] = (uint8_t)(mac0 >> 8);
        mac[5] = (uint8_t)mac0;
        return;
    }

    static const uint8_t kFallback[6] = ROSETTE_MAC_ADDR_DEFAULT;
    const uint32_t       uid = OCOTP->CFG0 ^ OCOTP->CFG1;
    mac[0] = kFallback[0];
    mac[1] = kFallback[1];
    mac[2] = kFallback[2];
    mac[3] = (uint8_t)(uid >> 16);
    mac[4] = (uint8_t)(uid >> 8);
    mac[5] = (uint8_t)uid;
}

void board_set_led(bool on) {
    GPIO_PinWrite(rosette_imxrt_gpio_port(ROSETTE_PIN_STATUS_LED),
                  rosette_imxrt_gpio_bit(ROSETTE_PIN_STATUS_LED), on ? 0U : 1U);
}
