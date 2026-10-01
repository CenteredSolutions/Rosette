/**
 * @file ports/chip/rp2/rp2_spi_bus.hpp
 * @brief WIZnet register bus over an RP2040 / RP2350 hardware SPI peripheral.
 *
 * Implements the bus `WiznetDevice<Chip, Bus>` is templated on (see
 * ports/chip/stub/stub_bus.hpp for the concept): each write() or read() is one
 * chip-select-framed transaction of the chip's address/control prefix and its
 * data. The board configures the peripheral (baud rate, pin functions) and
 * the chip-select pin before the first transaction.
 *
 * acquire() holds nothing. The driver nests it (one register access inside
 * another), so a bus shared between threads or cores would need a re-entrant
 * lock, which the platform contract does not offer; every board drives its
 * chip from one thread.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "hardware/gpio.h"
#include "hardware/spi.h"

namespace rosette::platform {

class Rp2SpiBus {
public:
    using address_type = uint32_t;

    struct Config {
        spi_inst_t* spi = nullptr;
        uint cs_pin = 0u;
        uint8_t read_fill = 0xFFu;
    };

    explicit Rp2SpiBus(const Config& config) noexcept
        : spi_(config.spi), cs_pin_(config.cs_pin), read_fill_(config.read_fill) {}

    Rp2SpiBus(spi_inst_t* spi, uint cs_pin) noexcept : Rp2SpiBus(Config{spi, cs_pin, 0xFFu}) {}

    /// Holds the bus while it lives; see the file comment for why it holds nothing.
    struct Lock {
        ~Lock() {}
    };

    /// @brief Holds the bus for the returned object's lifetime.
    Lock acquire() noexcept { return {}; }

    /// @brief Sends `prefix`, then `length` octets of `data`, in one transaction.
    void write(const uint8_t* prefix, std::size_t prefix_length, const uint8_t* data,
               std::size_t length) noexcept {
        if (spi_ == nullptr || prefix == nullptr || prefix_length == 0u || data == nullptr
            || length == 0u) {
            return;
        }

        gpio_put(cs_pin_, 0);
        spi_write_blocking(spi_, prefix, prefix_length);
        spi_write_blocking(spi_, data, length);
        gpio_put(cs_pin_, 1);
    }

    /// @brief Sends `prefix`, then clocks `length` octets into `data`, in one transaction.
    void read(const uint8_t* prefix, std::size_t prefix_length, uint8_t* data,
              std::size_t length) noexcept {
        if (spi_ == nullptr || prefix == nullptr || prefix_length == 0u || data == nullptr
            || length == 0u) {
            return;
        }

        gpio_put(cs_pin_, 0);
        spi_write_blocking(spi_, prefix, prefix_length);
        spi_read_blocking(spi_, read_fill_, data, length);
        gpio_put(cs_pin_, 1);
    }

    spi_inst_t* spi() const noexcept { return spi_; }
    uint cs_pin() const noexcept { return cs_pin_; }

private:
    spi_inst_t* spi_;
    uint cs_pin_;
    uint8_t read_fill_;
};

}  // namespace rosette::platform
