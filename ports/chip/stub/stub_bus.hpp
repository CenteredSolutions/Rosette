/**
 * @file ports/chip/stub/stub_bus.hpp
 * @brief Compile-only register bus for the WIZnet driver: reads return 0xFF, writes vanish.
 *
 * `WiznetDevice<Chip, Bus>` (rosette/net/wiznet/device.hpp) reaches the chip's
 * registers through a `Bus` it is templated on; this is the smallest type
 * that satisfies it. A chip port supplies a real one over its SPI or QSPI
 * peripheral (the rp2 port's Rp2SpiBus and Rp2QspiBus). What the driver
 * needs of it:
 *
 *  - `acquire()` returns an object that holds the bus for its lifetime, so a
 *    register access is not interleaved with another thread's;
 *  - `write(prefix, prefix_length, data, length)` sends the chip's address
 *    and control phase, then `length` data octets, in one transaction;
 *  - `read(prefix, prefix_length, data, length)` sends the same phase, then
 *    clocks `length` octets into `data`.
 *
 * The prefix is built by the Chip type (W5100S, W6100, W6300), so the bus
 * only moves octets.
 *
 * Verification:
 * - Correct means: tests/port_stub instantiates WiznetDevice and
 *   WiznetSocketProvider on this bus for every chip model, and links.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace rosette::platform {

class StubBus {
public:
    /// Holds the bus while it lives; the stub has nothing to hold.
    struct Lock {
        ~Lock() {}
    };

    /// @brief Holds the bus for the returned object's lifetime.
    Lock acquire() noexcept { return {}; }

    /// @brief Sends `prefix`, then `length` octets of `data`; the stub discards them.
    void write(const uint8_t* prefix, std::size_t prefix_length, const uint8_t* data,
               std::size_t length) noexcept {
        (void)prefix;
        (void)prefix_length;
        (void)data;
        (void)length;
    }

    /// @brief Sends `prefix`, then reads `length` octets into `data`; the stub reads an idle line
    /// (0xFF).
    void read(const uint8_t* prefix, std::size_t prefix_length, uint8_t* data,
              std::size_t length) noexcept {
        (void)prefix;
        (void)prefix_length;
        for (std::size_t i = 0; i < length; ++i) {
            data[i] = 0xFFu;
        }
    }
};

}  // namespace rosette::platform
