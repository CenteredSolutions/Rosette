/**
 * @file boards/w6100-evb-pico2/board_init.c
 * @brief Board bring-up for the WIZnet W6100-EVB-Pico2 (RP2350), no-RTOS build.
 *
 * Sets up SPI0, the GPIO pin functions and the W6100 hardware-reset sequence.
 * Everything past the SPI peripheral — register unlock, buffer sizing, PHY,
 * DHCP — is `rosette::net::wiznet::WiznetLink`, called from the application
 * after `board_init()`. The register-level driver drives the bus through
 * `rosette::platform::PicoSpiBus`.
 */

#include "board.h"
#include "board_config.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

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

    gpio_init(ROSETTE_PIN_WIZNET_RST);
    gpio_set_dir(ROSETTE_PIN_WIZNET_RST, GPIO_OUT);
    gpio_put(ROSETTE_PIN_WIZNET_RST, 1);

    // Active-low INT; the network bring-up attaches its interrupt
    // (attach_rx_interrupt()) once the chip answers.
    gpio_init(ROSETTE_PIN_WIZNET_INT);
    gpio_set_dir(ROSETTE_PIN_WIZNET_INT, GPIO_IN);
    gpio_pull_up(ROSETTE_PIN_WIZNET_INT);

    gpio_init(ROSETTE_PIN_LED);
    gpio_set_dir(ROSETTE_PIN_LED, GPIO_OUT);
    gpio_put(ROSETTE_PIN_LED, 0);

    // W6100 hardware reset: hold RST low, then wait for the PLL to lock.
    gpio_put(ROSETTE_PIN_WIZNET_RST, 0);
    sleep_ms(ROSETTE_WIZNET_RST_HOLD_MS);
    gpio_put(ROSETTE_PIN_WIZNET_RST, 1);
    sleep_ms(ROSETTE_WIZNET_RST_WAIT_MS);
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
