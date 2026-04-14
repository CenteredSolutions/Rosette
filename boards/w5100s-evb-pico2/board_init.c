/**
 * @file boards/w5100s-evb-pico2/board_init.c
 * @brief Board initialisation for the WIZnet W5100S-EVB-Pico2 (RP2350).
 *
 * Initialises:
 *   1. SPI1 peripheral at RTPS_SPI_BAUDRATE_HZ.
 *   2. GPIO pin functions (SPI alternate function + CS/RST/INT as GPIO).
 *   3. W5100S hardware reset sequence.
 *   4. WIZnet ioLibrary SPI callback registration (critical section,
 *      CS assert/deassert, byte read/write).
 *
 * All functions declared in net/wiznet_macraw.h are implemented or
 * called from here.
 */

#include "board_config.h"
#include "wiznet/macraw.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

// WIZnet ioLibrary — chip configuration and critical section callbacks.
#include "wizchip_conf.h"

// ============================================================
// SPI transfer primitives
//
// The WIZnet ioLibrary calls these via function pointers registered
// in board_init_wiznet().  They must be as fast as possible because
// they are called for every byte in a socket send/receive.
// ============================================================

static void spi_cs_assert(void) {
    gpio_put(RTPS_PIN_SPI_CS, 0);
}

static void spi_cs_deassert(void) {
    gpio_put(RTPS_PIN_SPI_CS, 1);
}

static uint8_t spi_read_byte(void) {
    uint8_t rx = 0;
    spi_read_blocking(RTPS_SPI_INST, 0xFF, &rx, 1);
    return rx;
}

static void spi_write_byte(uint8_t tx) {
    spi_write_blocking(RTPS_SPI_INST, &tx, 1);
}

static void spi_read_burst(uint8_t* buf, uint16_t len) {
    spi_read_blocking(RTPS_SPI_INST, 0xFF, buf, len);
}

static void spi_write_burst(uint8_t* buf, uint16_t len) {
    spi_write_blocking(RTPS_SPI_INST, buf, len);
}

// ============================================================
// Critical section callbacks
//
// The ioLibrary wraps multi-byte register accesses in a critical
// section to prevent re-entrant SPI access from an ISR.  On FreeRTOS
// we use taskENTER_CRITICAL() / taskEXIT_CRITICAL().
// ============================================================

#include "FreeRTOS.h"
#include "task.h"

static void crit_enter(void) { taskENTER_CRITICAL(); }
static void crit_exit(void)  { taskEXIT_CRITICAL();  }

static void wiznet_irq_gpio_callback(uint gpio, uint32_t events) {
    if ((gpio == RTPS_PIN_W5100S_INT) && ((events & GPIO_IRQ_EDGE_FALL) != 0u)) {
        wiznet_macraw_interrupt_from_isr();
    }
}

// ============================================================
// board_init()
//
// Call once from main() before starting the FreeRTOS scheduler.
// ============================================================

void board_init(void) {
    // 1. Standard Pico/Pico2 stdio + clocks.
    stdio_init_all();

    // 2. Initialise SPI1 at the configured baud rate.
    spi_init(RTPS_SPI_INST, RTPS_SPI_BAUDRATE_HZ);

    // W5100S uses SPI mode 0 (CPOL=0, CPHA=0).
    spi_set_format(RTPS_SPI_INST,
                   8,           // data bits
                   SPI_CPOL_0,
                   SPI_CPHA_0,
                   SPI_MSB_FIRST);

    // 3. Assign GPIO pin functions.
    gpio_set_function(RTPS_PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(RTPS_PIN_SPI_TX,  GPIO_FUNC_SPI);
    gpio_set_function(RTPS_PIN_SPI_RX,  GPIO_FUNC_SPI);

    // CS is GPIO-driven (not SPI hardware CS) so we control it manually.
    gpio_init(RTPS_PIN_SPI_CS);
    gpio_set_dir(RTPS_PIN_SPI_CS, GPIO_OUT);
    gpio_put(RTPS_PIN_SPI_CS, 1);  // De-assert.

    // RST — active low.
    gpio_init(RTPS_PIN_W5100S_RST);
    gpio_set_dir(RTPS_PIN_W5100S_RST, GPIO_OUT);
    gpio_put(RTPS_PIN_W5100S_RST, 1);  // De-assert.

    // INT — input with pull-up (active low, unused if polling).
    gpio_init(RTPS_PIN_W5100S_INT);
    gpio_set_dir(RTPS_PIN_W5100S_INT, GPIO_IN);
    gpio_pull_up(RTPS_PIN_W5100S_INT);
    gpio_set_irq_enabled_with_callback(RTPS_PIN_W5100S_INT,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &wiznet_irq_gpio_callback);

    // LED.
    gpio_init(RTPS_PIN_LED);
    gpio_set_dir(RTPS_PIN_LED, GPIO_OUT);
    gpio_put(RTPS_PIN_LED, 0);

    // 4. Hardware reset the W5100S.
    // Hold RST low for >= RTPS_W5100S_RST_HOLD_MS.
    gpio_put(RTPS_PIN_W5100S_RST, 0);
    sleep_ms(RTPS_W5100S_RST_HOLD_MS);
    gpio_put(RTPS_PIN_W5100S_RST, 1);
    // Wait for PLL lock and internal initialisation.
    sleep_ms(RTPS_W5100S_RST_WAIT_MS);

    // 5. Register WIZnet ioLibrary SPI + CS + critical section callbacks.
    reg_wizchip_cris_cbfunc(crit_enter, crit_exit);
    reg_wizchip_cs_cbfunc(spi_cs_assert, spi_cs_deassert);
    reg_wizchip_spi_cbfunc(spi_read_byte, spi_write_byte);
    reg_wizchip_spiburst_cbfunc(spi_read_burst, spi_write_burst);

    // 6. Delegate to the MACRAW driver for W5100S chip initialisation
    //    (sets TX/RX buffer sizes, opens MACRAW socket 0).
    wiznet_macraw_init();
}

// ============================================================
// board_get_mac_address()
//
// Derives a unique MAC address from the RP2350's 64-bit unique ID.
// WIZnet OUI prefix: 00:08:DC.
// ============================================================

void board_get_mac_address(uint8_t mac[6]) {
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);

    mac[0] = 0x00;
    mac[1] = 0x08;
    mac[2] = 0xDC;
    // Use bytes 5-7 of the 64-bit ID as the locally-administered suffix.
    mac[3] = uid.id[5];
    mac[4] = uid.id[6];
    mac[5] = uid.id[7];
}
