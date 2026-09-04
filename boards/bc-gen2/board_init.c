/**
 * @file boards/custom-pico2/board_init.c
 * @brief Board initialisation for the custom RP2350 + W5100S board.
 *
 * Differs from the EVB variants only in GPIO pin assignments (driven by
 * board_config.h) and any board-specific peripheral setup.
 */

#include "board_config.h"
#include "wiznet/macraw.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "wizchip_conf.h"

#include "FreeRTOS.h"
#include "task.h"

static void spi_cs_assert(void)   { gpio_put(ROSETTE_PIN_SPI_CS, 0); }
static void spi_cs_deassert(void) { gpio_put(ROSETTE_PIN_SPI_CS, 1); }
static uint8_t spi_read_byte(void) {
    uint8_t rx = 0;
    spi_read_blocking(ROSETTE_SPI_INST, 0xFF, &rx, 1);
    return rx;
}
static void spi_write_byte(uint8_t tx) {
    spi_write_blocking(ROSETTE_SPI_INST, &tx, 1);
}
static void spi_read_burst(uint8_t* buf, uint16_t len) {
    spi_read_blocking(ROSETTE_SPI_INST, 0xFF, buf, len);
}
static void spi_write_burst(uint8_t* buf, uint16_t len) {
    spi_write_blocking(ROSETTE_SPI_INST, buf, len);
}
static void crit_enter(void) { taskENTER_CRITICAL(); }
static void crit_exit(void)  { taskEXIT_CRITICAL();  }
static void wiznet_irq_gpio_callback(uint gpio, uint32_t events) {
    if ((gpio == ROSETTE_PIN_W5100S_INT) && ((events & GPIO_IRQ_EDGE_FALL) != 0u)) {
        wiznet_macraw_interrupt_from_isr();
    }
}

void board_init(void) {
    stdio_init_all();

    // ── SPI peripheral ────────────────────────────────────────────────────────
    spi_init(ROSETTE_SPI_INST, ROSETTE_SPI_BAUDRATE_HZ);
    spi_set_format(ROSETTE_SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(ROSETTE_PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(ROSETTE_PIN_SPI_TX,  GPIO_FUNC_SPI);
    gpio_set_function(ROSETTE_PIN_SPI_RX,  GPIO_FUNC_SPI);

    gpio_init(ROSETTE_PIN_SPI_CS);
    gpio_set_dir(ROSETTE_PIN_SPI_CS, GPIO_OUT);
    gpio_put(ROSETTE_PIN_SPI_CS, 1);

    // ── W5100S control ────────────────────────────────────────────────────────
    gpio_init(ROSETTE_PIN_W5100S_RST);
    gpio_set_dir(ROSETTE_PIN_W5100S_RST, GPIO_OUT);
    gpio_put(ROSETTE_PIN_W5100S_RST, 1);

    gpio_init(ROSETTE_PIN_W5100S_INT);
    gpio_set_dir(ROSETTE_PIN_W5100S_INT, GPIO_IN);
    gpio_pull_up(ROSETTE_PIN_W5100S_INT);
    gpio_set_irq_enabled_with_callback(ROSETTE_PIN_W5100S_INT,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &wiznet_irq_gpio_callback);

    // ── Board-specific GPIO ───────────────────────────────────────────────────
    gpio_init(ROSETTE_PIN_LED);
    gpio_set_dir(ROSETTE_PIN_LED, GPIO_OUT);
    gpio_put(ROSETTE_PIN_LED, 0);

#ifdef CUSTOM_PIN_RS485_DE
    // Drive DE low to put RS-485 transceiver in receive mode at startup.
    gpio_init(CUSTOM_PIN_RS485_DE);
    gpio_set_dir(CUSTOM_PIN_RS485_DE, GPIO_OUT);
    gpio_put(CUSTOM_PIN_RS485_DE, 0);
#endif

    // ── W5100S reset ──────────────────────────────────────────────────────────
    gpio_put(ROSETTE_PIN_W5100S_RST, 0);
    sleep_ms(ROSETTE_W5100S_RST_HOLD_MS);
    gpio_put(ROSETTE_PIN_W5100S_RST, 1);
    sleep_ms(ROSETTE_W5100S_RST_WAIT_MS);

    // ── WIZnet ioLibrary callbacks ────────────────────────────────────────────
    reg_wizchip_cris_cbfunc(crit_enter, crit_exit);
    reg_wizchip_cs_cbfunc(spi_cs_assert, spi_cs_deassert);
    reg_wizchip_spi_cbfunc(spi_read_byte, spi_write_byte);
    reg_wizchip_spiburst_cbfunc(spi_read_burst, spi_write_burst);

    // ── MACRAW socket init ────────────────────────────────────────────────────
    wiznet_macraw_init();
}

void board_get_mac_address(uint8_t mac[6]) {
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);
    mac[0] = 0x00; mac[1] = 0x08; mac[2] = 0xDC;
    mac[3] = uid.id[5]; mac[4] = uid.id[6]; mac[5] = uid.id[7];
}
