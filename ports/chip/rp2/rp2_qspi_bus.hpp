/**
 * @file ports/chip/rp2/rp2_qspi_bus.hpp
 * @brief WIZnet W6300 QSPI bus for RP2040 / RP2350 — single-lane (phase 1).
 *
 * The W6300's host interface is QSPI (single / dual / quad lane). On the
 * RP2040 / RP2350 there is no spare hardware QSPI block — the on-chip QSPI is
 * dedicated to XIP flash — so a real 4-lane driver needs a PIO state-machine
 * program. That is phase 2.
 *
 * For phase 1 this is a **single-lane** implementation: electrically identical
 * to ordinary SPI mode 0, driven by GPIO bit-bang so it works regardless of
 * which pins the board routes the W6300 to (the W6300-EVB-Pico2 does not map
 * the chip onto a hardware-SPI-capable pin set). It presents the same surface
 * as `Rp2SpiBus` — `acquire()`, `write(prefix, …)`, `read(prefix, …)` — so
 * `WiznetDevice<W6300, Rp2QspiBus>` works today, and phase 2 swaps the
 * transfer internals for PIO + widens the data phase without touching the
 * device or trait layers.
 *
 * The address/OP-code phase of a W6300 frame is always single-lane; only the
 * data phase goes wide. That boundary is exactly the `prefix` / `data` split
 * in the transfer methods, so the wide-data work in phase 2 is contained to
 * `transfer_data()`.
 *
 * acquire() holds nothing, for the reason given in rp2_spi_bus.hpp.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "hardware/gpio.h"

namespace rosette::platform {

class Rp2QspiBus {
public:
    using address_type = uint32_t;

    struct Config {
        uint sck_pin = 0u;
        uint io0_pin = 0u;  ///< MOSI in single-lane mode
        uint io1_pin = 0u;  ///< MISO in single-lane mode
        uint cs_pin = 0u;
    };

    explicit Rp2QspiBus(const Config& config) noexcept : cfg_(config) { init_pins(); }

    /// Holds the bus while it lives; see the file comment for why it holds nothing.
    struct Lock {
        ~Lock() {}
    };

    /// @brief Holds the bus for the returned object's lifetime.
    Lock acquire() noexcept { return {}; }

    /// @brief Sends `prefix` single-lane, then `length` octets of `data`, in one transaction.
    void write(const uint8_t* prefix, std::size_t prefix_length, const uint8_t* data,
               std::size_t length) noexcept {
        if (prefix == nullptr || prefix_length == 0u || data == nullptr || length == 0u) {
            return;
        }

        gpio_put(cfg_.cs_pin, 0);
        transfer_phase(prefix, nullptr, prefix_length);  // OP-code + address, single-lane
        transfer_data(data, nullptr, length);            // phase 2: may go wide
        gpio_put(cfg_.cs_pin, 1);
    }

    /// @brief Sends `prefix` single-lane, then clocks `length` octets into `data`.
    void read(const uint8_t* prefix, std::size_t prefix_length, uint8_t* data,
              std::size_t length) noexcept {
        if (prefix == nullptr || prefix_length == 0u || data == nullptr || length == 0u) {
            return;
        }

        gpio_put(cfg_.cs_pin, 0);
        transfer_phase(prefix, nullptr, prefix_length);
        transfer_data(nullptr, data, length);
        gpio_put(cfg_.cs_pin, 1);
    }

private:
    void init_pins() noexcept
    {
        gpio_init(cfg_.sck_pin);
        gpio_init(cfg_.io0_pin);
        gpio_init(cfg_.io1_pin);
        gpio_init(cfg_.cs_pin);
        gpio_set_dir(cfg_.sck_pin, GPIO_OUT);
        gpio_set_dir(cfg_.io0_pin, GPIO_OUT);
        gpio_set_dir(cfg_.io1_pin, GPIO_IN);
        gpio_set_dir(cfg_.cs_pin, GPIO_OUT);
        gpio_put(cfg_.sck_pin, 0);
        gpio_put(cfg_.cs_pin, 1);
    }

    /// One single-lane (1-bit) SPI-mode-0 byte transfer, MSB first.
    uint8_t transfer_byte(uint8_t out) noexcept
    {
        uint8_t in = 0u;
        for (int bit = 7; bit >= 0; --bit) {
            gpio_put(cfg_.io0_pin, (out >> bit) & 1u);
            gpio_put(cfg_.sck_pin, 1);
            in = static_cast<uint8_t>((in << 1) | (gpio_get(cfg_.io1_pin) ? 1u : 0u));
            gpio_put(cfg_.sck_pin, 0);
        }
        return in;
    }

    void transfer_phase(const uint8_t* tx, uint8_t* rx, std::size_t len) noexcept
    {
        for (std::size_t i = 0; i < len; ++i) {
            const uint8_t got = transfer_byte(tx != nullptr ? tx[i] : 0xFFu);
            if (rx != nullptr) {
                rx[i] = got;
            }
        }
    }

    // The data phase. Single-lane today; phase 2 replaces this with a PIO
    // transfer that can clock IO0..IO3 together for dual / quad mode.
    void transfer_data(const uint8_t* tx, uint8_t* rx, std::size_t len) noexcept
    {
        transfer_phase(tx, rx, len);
    }

    Config cfg_;
};

}  // namespace rosette::platform
