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

#include "FreeRTOS.h"
#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "imxrt_console.h"
#include "imxrt_memory.h"

/// The FreeRTOS heap (configAPPLICATION_ALLOCATED_HEAP), in the OCRAM; heap_4 initialises it.
ROSETTE_IMXRT_OCRAM_BSS uint8_t ucHeap[configTOTAL_HEAP_SIZE];

/// Output pad: keeper, medium speed, drive strength R0/6.
#define BOARD_PAD_OUTPUT \
    (IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(6))

/// Input pad with the 47 kΩ pull-up and hysteresis.
#define BOARD_PAD_PULLUP_INPUT                                                                  \
    (IOMUXC_SW_PAD_CTL_PAD_HYS(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1) \
     | IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_SPEED(2)                            \
     | IOMUXC_SW_PAD_CTL_PAD_DSE(6))

/// RMII data and MDC pads: keeper, 100 MHz, DSE R0/5, fast slew (the SDK EVK's 0xB0E9).
#define BOARD_PAD_RMII 0xB0E9U

/// MDIO: open drain with the 100 kΩ pull-up (the SDK EVK's 0xB829).
#define BOARD_PAD_MDIO 0xB829U

/// ENET_REF_CLK output: fast slew, DSE R0/6 (the SDK EVK's 0x31).
#define BOARD_PAD_REF_CLK 0x31U

/// KSZ8081 RST_B low time (datasheet: at least 10 ms after power is stable).
#define BOARD_PHY_RESET_US 10000U

/// KSZ8081 time from RST_B high to its first MDIO access (datasheet: 100 µs).
#define BOARD_PHY_STARTUP_US 100U

static void board_init_console(void) {
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_06_LPUART1_TX, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_07_LPUART1_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_06_LPUART1_TX, BOARD_PAD_OUTPUT);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_07_LPUART1_RX, BOARD_PAD_PULLUP_INPUT);
    (void)rosette_imxrt_console_init(ROSETTE_CONSOLE_LPUART, ROSETTE_CONSOLE_BAUD,
                                     BOARD_BOOTCLOCKRUN_UART_CLK_ROOT);
}

/**
 * @brief Readies the RMII link to the KSZ8081: pads, the 50 MHz reference clock, and the PHY's
 * reset.
 *
 * The ENET PLL's ENET0 output runs at 50 MHz (loop divider 1) and leaves on
 * GPIO_AD_B0_08 as ENET_REF_CLK1, with GPR1 ENET1_TX_CLK_DIR set so the pad
 * drives it; SION feeds it back to the MAC's own RMII logic. The PHY's
 * INTRP pin is held high while RST_B is low, since the KSZ8081 latches its
 * NAND-tree strap from that pin at reset.
 *
 * Verification:
 * - Correct means: after the ENET MacDevice starts auto-negotiation, the
 *   link partner sees carrier, and MDIO reads of PHY register 2 return the
 *   KSZ8081's OUI (0x0022).
 * - Covered by: bring-up on the MIMXRT1020-EVK.
 *
 * References: i.MX RT1020 Reference Manual, CCM_ANALOG chapter (PLL_ENET)
 * and IOMUXC_GPR chapter (GPR1 ENET1_TX_CLK_DIR); KSZ8081RNA/RNB datasheet,
 * "Strapping Options" and "Reset Timing"; MCUXpresso SDK evkmimxrt1020
 * board.h (BOARD_ENET_PHY_RESET) and its enet examples' pin_mux.c.
 */
static void board_init_ethernet(void) {
    const clock_enet_pll_config_t pll = {.enableClkOutput = true,
                                         .enableClkOutput500M = false,
                                         .enableClkOutput25M = false,
                                         .loopDivider = 1U,
                                         .src = 0U};
    CLOCK_InitEnetPll(&pll);
    IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_08_ENET_REF_CLK1, 1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_08_ENET_REF_CLK1, BOARD_PAD_REF_CLK);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_09_ENET_RDATA01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_ENET_RDATA00, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_11_ENET_RX_EN, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_12_ENET_RX_ER, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_13_ENET_TX_EN, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_14_ENET_TDATA00, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_15_ENET_TDATA01, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_41_ENET_MDC, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_40_ENET_MDIO, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_09_ENET_RDATA01, BOARD_PAD_RMII);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_ENET_RDATA00, BOARD_PAD_RMII);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_11_ENET_RX_EN, BOARD_PAD_RMII);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_12_ENET_RX_ER, BOARD_PAD_RMII);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_13_ENET_TX_EN, BOARD_PAD_RMII);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_14_ENET_TDATA00, BOARD_PAD_RMII);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_15_ENET_TDATA01, BOARD_PAD_RMII);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_41_ENET_MDC, BOARD_PAD_RMII);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_40_ENET_MDIO, BOARD_PAD_MDIO);

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_04_GPIO1_IO04, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_06_GPIO1_IO22, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_04_GPIO1_IO04, BOARD_PAD_OUTPUT);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_06_GPIO1_IO22, BOARD_PAD_OUTPUT);
    const gpio_pin_config_t low = {kGPIO_DigitalOutput, 0U, kGPIO_NoIntmode};
    const gpio_pin_config_t high = {kGPIO_DigitalOutput, 1U, kGPIO_NoIntmode};
    GPIO_PinInit(rosette_imxrt_gpio_port(ROSETTE_PIN_PHY_INT),
                 rosette_imxrt_gpio_bit(ROSETTE_PIN_PHY_INT), &high);
    GPIO_PinInit(rosette_imxrt_gpio_port(ROSETTE_PIN_PHY_RST),
                 rosette_imxrt_gpio_bit(ROSETTE_PIN_PHY_RST), &low);
    SDK_DelayAtLeastUs(BOARD_PHY_RESET_US, SystemCoreClock);
    GPIO_PinWrite(rosette_imxrt_gpio_port(ROSETTE_PIN_PHY_RST),
                  rosette_imxrt_gpio_bit(ROSETTE_PIN_PHY_RST), 1U);
    SDK_DelayAtLeastUs(BOARD_PHY_STARTUP_US, SystemCoreClock);

    // After reset INTRP is the PHY's output; nothing uses it, so it becomes an input.
    const gpio_pin_config_t input = {kGPIO_DigitalInput, 0U, kGPIO_NoIntmode};
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_06_GPIO1_IO22, BOARD_PAD_PULLUP_INPUT);
    GPIO_PinInit(rosette_imxrt_gpio_port(ROSETTE_PIN_PHY_INT),
                 rosette_imxrt_gpio_bit(ROSETTE_PIN_PHY_INT), &input);
}

void board_init(void) {
    CLOCK_EnableClock(kCLOCK_Iomuxc);
    board_init_console();
    board_init_ethernet();
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
