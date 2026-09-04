/**
 * @file boards/w5100s-evb-pico2/board_init.c
 * @brief Board bring-up for the WIZnet W5100S-EVB-Pico2 (RP2350), no-RTOS build.
 *
 * Sets up SPI1, the GPIO pin functions, the W5100S hardware-reset sequence, and
 * registers the WIZnet ioLibrary's SPI / chip-select / critical-section
 * callbacks. Chip network configuration (buffer sizes, PHY, DHCP) is
 * `rosette::net::wiznet::WiznetLink`, called from the application after
 * `board_init()`.
 *
 * The critical-section callbacks disable interrupts rather than taking an RTOS
 * lock: on a single-core superloop that is the only reentrancy the ioLibrary's
 * multi-byte SPI accesses need protection from.
 */

#include "board.h"
#include "board_config.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "wizchip_conf.h"

// ── SPI transfer primitives (called per byte by the ioLibrary) ───────────────
static void spi_cs_assert(void)   { gpio_put(ROSETTE_PIN_SPI_CS, 0); }
static void spi_cs_deassert(void) { gpio_put(ROSETTE_PIN_SPI_CS, 1); }

static uint8_t spi_read_byte(void)
{
    uint8_t rx = 0u;
    spi_read_blocking(ROSETTE_SPI_INST, 0xFFu, &rx, 1);
    return rx;
}

static void spi_write_byte(uint8_t tx)
{
    spi_write_blocking(ROSETTE_SPI_INST, &tx, 1);
}

static void spi_read_burst(uint8_t* buf, uint16_t len)
{
    spi_read_blocking(ROSETTE_SPI_INST, 0xFFu, buf, len);
}

static void spi_write_burst(uint8_t* buf, uint16_t len)
{
    spi_write_blocking(ROSETTE_SPI_INST, buf, len);
}

// ── Critical section: mask interrupts around multi-byte register access ──────
static uint32_t s_crit_save;
static void crit_enter(void) { s_crit_save = save_and_disable_interrupts(); }
static void crit_exit(void)  { restore_interrupts(s_crit_save); }

// ── WIZnet INT line ─────────────────────────────────────────────────────────
// Active-low, asserted while a socket has an unserviced interrupt (RX). The
// handler does nothing: the ISR merely running pulls the superloop out of the
// WFE in rosette_platform_wait_for_event(), which then re-polls the chip. The
// SDK acknowledges the GPIO IRQ before calling this.
static void wiznet_int_callback(uint gpio, uint32_t events)
{
    (void)gpio;
    (void)events;
}

void board_init(void)
{
    stdio_init_all();

    spi_init(ROSETTE_SPI_INST, ROSETTE_SPI_BAUDRATE_HZ);
    spi_set_format(ROSETTE_SPI_INST, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(ROSETTE_PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(ROSETTE_PIN_SPI_TX,  GPIO_FUNC_SPI);
    gpio_set_function(ROSETTE_PIN_SPI_RX,  GPIO_FUNC_SPI);

    gpio_init(ROSETTE_PIN_SPI_CS);
    gpio_set_dir(ROSETTE_PIN_SPI_CS, GPIO_OUT);
    gpio_put(ROSETTE_PIN_SPI_CS, 1);

    gpio_init(ROSETTE_PIN_W5100S_RST);
    gpio_set_dir(ROSETTE_PIN_W5100S_RST, GPIO_OUT);
    gpio_put(ROSETTE_PIN_W5100S_RST, 1);

    gpio_init(ROSETTE_PIN_W5100S_INT);
    gpio_set_dir(ROSETTE_PIN_W5100S_INT, GPIO_IN);
    gpio_pull_up(ROSETTE_PIN_W5100S_INT);
    gpio_set_irq_enabled_with_callback(ROSETTE_PIN_W5100S_INT,
                                       GPIO_IRQ_EDGE_FALL, true,
                                       &wiznet_int_callback);

    gpio_init(ROSETTE_PIN_LED);
    gpio_set_dir(ROSETTE_PIN_LED, GPIO_OUT);
    gpio_put(ROSETTE_PIN_LED, 0);

    // W5100S hardware reset: hold RST low, then wait for the PLL to lock.
    gpio_put(ROSETTE_PIN_W5100S_RST, 0);
    sleep_ms(ROSETTE_W5100S_RST_HOLD_MS);
    gpio_put(ROSETTE_PIN_W5100S_RST, 1);
    sleep_ms(ROSETTE_W5100S_RST_WAIT_MS);

    reg_wizchip_cris_cbfunc(crit_enter, crit_exit);
    reg_wizchip_cs_cbfunc(spi_cs_assert, spi_cs_deassert);
    reg_wizchip_spi_cbfunc(spi_read_byte, spi_write_byte);
    reg_wizchip_spiburst_cbfunc(spi_read_burst, spi_write_burst);
}

void board_get_mac_address(uint8_t mac[6])
{
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);

    // WIZnet OUI 00:08:DC + a per-board suffix from the RP2350 unique ID.
    mac[0] = 0x00u;
    mac[1] = 0x08u;
    mac[2] = 0xDCu;
    mac[3] = uid.id[5];
    mac[4] = uid.id[6];
    mac[5] = uid.id[7];
}

void board_set_led(bool on)
{
    gpio_put(ROSETTE_PIN_LED, on ? 1 : 0);
}
