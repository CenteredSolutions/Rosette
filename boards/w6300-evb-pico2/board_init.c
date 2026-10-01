/**
 * @file boards/w6300-evb-pico2/board_init.c
 * @brief Board bring-up for the WIZnet W6300-EVB-Pico2 (RP2350), no-RTOS build.
 *
 * The W6300 bus is `rosette::platform::Rp2QspiBus`, which configures its own
 * SCK / IO0 / IO1 / CS pins. `board_init()` therefore only handles the reset
 * line, the (optional) interrupt line and the user LED, then pulses the W6300
 * hardware reset. Register unlock / buffer sizing / PHY / DHCP are
 * `rosette::net::wiznet::WiznetLink`.
 */

#include "board.h"
#include "board_config.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

void board_init(void)
{
    stdio_init_all();

    gpio_init(ROSETTE_PIN_WIZNET_RST);
    gpio_set_dir(ROSETTE_PIN_WIZNET_RST, GPIO_OUT);
    gpio_put(ROSETTE_PIN_WIZNET_RST, 1);

    // Active-low INT; the network bring-up attaches its interrupt
    // (WiznetSocketProvider::attach_interrupt()) once the chip answers.
    gpio_init(ROSETTE_PIN_WIZNET_INT);
    gpio_set_dir(ROSETTE_PIN_WIZNET_INT, GPIO_IN);
    gpio_pull_up(ROSETTE_PIN_WIZNET_INT);

    gpio_init(ROSETTE_PIN_LED);
    gpio_set_dir(ROSETTE_PIN_LED, GPIO_OUT);
    gpio_put(ROSETTE_PIN_LED, 0);

    // W6300 hardware reset: hold RST low, then wait for the PLL to lock.
    gpio_put(ROSETTE_PIN_WIZNET_RST, 0);
    sleep_ms(ROSETTE_WIZNET_RST_HOLD_MS);
    gpio_put(ROSETTE_PIN_WIZNET_RST, 1);
    sleep_ms(ROSETTE_WIZNET_RST_WAIT_MS);
}

void board_get_mac_address(uint8_t mac[6])
{
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);

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
