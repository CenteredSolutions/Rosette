/**
 * @file boards/micromod-teensy-w5500/board_init.c
 * @brief Board bring-up for the SparkFun MicroMod Teensy with the Ethernet Function Board, no-RTOS
 * build.
 *
 * Starts the USB console and the LPUART6 console, flashes the status LED,
 * muxes the LPSPI4 and W5500 control pads, initializes LPSPI4 in SPI mode 0,
 * and pulses the W5500's reset. The clock
 * tree and the time base are the chip port's, already running before main()
 * (ports/chip/imxrt/chip_imxrt.cpp); everything past the SPI peripheral
 * (register setup, buffer sizing, PHY, DHCP) is
 * `rosette::net::wiznet::WiznetLink`, which drives the bus through
 * `rosette::platform::ImxrtLpspiBus`.
 *
 * Pad settings (IOMUXC SW_PAD_CTL): outputs use drive strength R0/6 at
 * medium speed with the keeper on; the open-drain INTn input has the 47 kΩ
 * pull-up and hysteresis, as the gpio_interrupt.hpp contract expects of the
 * pin before attach_falling_edge().
 *
 * References: i.MX RT1060 Reference Manual §11.6 (IOMUXC SW_MUX_CTL /
 * SW_PAD_CTL); W5500 datasheet §5.5.1 (reset timing); MCUXpresso SDK
 * fsl_iomuxc.h, fsl_lpspi.h.
 */

#include <stdio.h>

#include "board.h"
#include "board_config.h"

#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"
#include "imxrt_console.h"

/// Output pad: keeper, medium speed, drive strength R0/6.
#define BOARD_PAD_OUTPUT \
    (IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | IOMUXC_SW_PAD_CTL_PAD_DSE(6))

/// Input pad with the 47 kΩ pull-up and hysteresis.
#define BOARD_PAD_PULLUP_INPUT                                                                  \
    (IOMUXC_SW_PAD_CTL_PAD_HYS(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1) \
     | IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_SPEED(2)                            \
     | IOMUXC_SW_PAD_CTL_PAD_DSE(6))

/// Sets a GPIO output to `level`; `pin` is a ROSETTE_IMXRT_GPIO() number.
static void board_gpio_output(uint32_t pin, uint8_t level) {
    const gpio_pin_config_t config = {kGPIO_DigitalOutput, level, kGPIO_NoIntmode};
    GPIO_PinInit(rosette_imxrt_gpio_port(pin), rosette_imxrt_gpio_bit(pin), &config);
}

static void board_init_console(void) {
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_LPUART6_TX, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_LPUART6_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_LPUART6_TX, BOARD_PAD_OUTPUT);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_LPUART6_RX, BOARD_PAD_PULLUP_INPUT);
    (void)rosette_imxrt_console_init(ROSETTE_CONSOLE_LPUART, ROSETTE_CONSOLE_BAUD,
                                     BOARD_BOOTCLOCKRUN_UART_CLK_ROOT);
}

/**
 * @brief Lights the status LED for ROSETTE_STATUS_LED_BOOT_FLASH_US: the firmware reached
 * board_init().
 *
 * The LED's pad is SPI_SCK, so this runs while the W5500's chip select is
 * already high, and board_init_spi() then hands the pad to LPSPI4; the
 * edges it makes are ignored by the deselected W5500.
 */
static void board_flash_status_led(void) {
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_03_GPIO2_IO03, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_03_GPIO2_IO03, BOARD_PAD_OUTPUT);
    board_gpio_output(ROSETTE_PIN_STATUS_LED, 1U);
    SDK_DelayAtLeastUs(ROSETTE_STATUS_LED_BOOT_FLASH_US, SystemCoreClock);
    GPIO_PinWrite(rosette_imxrt_gpio_port(ROSETTE_PIN_STATUS_LED),
                  rosette_imxrt_gpio_bit(ROSETTE_PIN_STATUS_LED), 0U);
}

static void board_init_spi(void) {
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_03_LPSPI4_SCK, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_02_LPSPI4_SDO, 0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_01_LPSPI4_SDI, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_03_LPSPI4_SCK, BOARD_PAD_OUTPUT);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_02_LPSPI4_SDO, BOARD_PAD_OUTPUT);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_01_LPSPI4_SDI, BOARD_PAD_PULLUP_INPUT);

    lpspi_master_config_t config;
    LPSPI_MasterGetDefaultConfig(&config);
    config.baudRate = ROSETTE_SPI_BAUDRATE_HZ;
    config.bitsPerFrame = 8U;
    config.cpol = kLPSPI_ClockPolarityActiveHigh;
    config.cpha = kLPSPI_ClockPhaseFirstEdge;
    config.direction = kLPSPI_MsbFirst;
    LPSPI_MasterInit(ROSETTE_LPSPI_INST, &config, BOARD_BOOTCLOCKRUN_LPSPI_CLK_ROOT);
}

void board_init(void) {
    CLOCK_EnableClock(kCLOCK_Iomuxc);
    (void)rosette_imxrt_usb_console_init();
    board_init_console();
    printf("[board] MicroMod Teensy, reset cause SRC_SRSR=0x%08lx\n", (unsigned long)SRC->SRSR);

    // The W5500's chip select goes high before anything else drives its SCK
    // (the LED flash, then LPSPI4).
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_00_GPIO2_IO00, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_00_GPIO2_IO00, BOARD_PAD_OUTPUT);
    board_gpio_output(ROSETTE_PIN_SPI_CS, 1U);
    board_flash_status_led();
    board_init_spi();

    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_05_GPIO4_IO05, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_05_GPIO4_IO05, BOARD_PAD_OUTPUT);
    board_gpio_output(ROSETTE_PIN_WIZNET_RST, 1U);

    // Active-low, open-drain INTn; the network bring-up attaches its
    // interrupt (WiznetSocketProvider::attach_interrupt()) once the chip answers.
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_06_GPIO4_IO06, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_06_GPIO4_IO06, BOARD_PAD_PULLUP_INPUT);

    // W5500 hardware reset: hold RSTn low, then wait for the PLL to lock.
    GPIO_PinWrite(rosette_imxrt_gpio_port(ROSETTE_PIN_WIZNET_RST),
                  rosette_imxrt_gpio_bit(ROSETTE_PIN_WIZNET_RST), 0U);
    SDK_DelayAtLeastUs(ROSETTE_WIZNET_RST_HOLD_US, SystemCoreClock);
    GPIO_PinWrite(rosette_imxrt_gpio_port(ROSETTE_PIN_WIZNET_RST),
                  rosette_imxrt_gpio_bit(ROSETTE_PIN_WIZNET_RST), 1U);
    SDK_DelayAtLeastUs(ROSETTE_WIZNET_RST_WAIT_US, SystemCoreClock);
    printf("[board] W5500 reset pulsed on GPIO4_IO05 (PWM0)\n");
}

/**
 * @brief Enters the Teensy bootloader, as Teensyduino does for a 134-baud request.
 *
 * The Teensy's bootloader chip watches the processor's debug port and takes
 * over at a `bkpt #251`, which halts the core instead of faulting while it
 * holds the debug port. The USB console calls this in its interrupt.
 *
 * Verification:
 * - Correct means: after `stty -F /dev/ttyACM0 134`, the ACM device leaves
 *   the bus and `teensy_loader_cli --mcu=TEENSY_MICROMOD -w` programs the
 *   board without the button.
 *
 * References: PJRC Teensyduino cores/teensy4/usb.c (usb_reboot_timer, `bkpt #251`).
 */
void board_enter_bootloader(void) {
    __disable_irq();
    __asm volatile("bkpt #251");
    for (;;) {
    }
}

/**
 * @brief The PJRC-fused MAC (OCOTP MAC1:MAC0), or a locally administered one from the fuse UID.
 *
 * PJRC programs a unique address under its OUI 04:E9:E5 into every Teensy
 * 4.x, MAC1[15:0] holding the top two octets and MAC0 the low four, as
 * Teensyduino's teensyMAC() reads it. A part whose MAC fuses are blank gets
 * ROSETTE_MAC_ADDR_DEFAULT's prefix with its last three octets from the UID.
 *
 * @param[out] mac Six octets, network order.
 *
 * Verification:
 * - Correct means: the W5500's SHAR matches the address printed on the
 *   Teensy's PJRC label (or the DHCP server's lease table).
 *
 * References: i.MX RT1060 Reference Manual §23.6.1.41 (HW_OCOTP_MAC0/MAC1).
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
    const uint32_t uid = OCOTP->CFG0 ^ OCOTP->CFG1;
    mac[0] = kFallback[0];
    mac[1] = kFallback[1];
    mac[2] = kFallback[2];
    mac[3] = (uint8_t)(uid >> 16);
    mac[4] = (uint8_t)(uid >> 8);
    mac[5] = (uint8_t)uid;
}

void board_set_led(bool on) {
    (void)on;
}
