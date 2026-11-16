// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/imxrt/imxrt_lpspi_bus.hpp
 * @brief WIZnet register bus over an i.MX RT LPSPI peripheral, with a GPIO chip select.
 *
 * Implements the bus `WiznetDevice<Chip, Bus>` is templated on (see
 * ports/chip/stub/stub_bus.hpp for the concept): each write() or read() is one
 * chip-select-framed transaction of the chip's address/control prefix and its
 * data. The prefix and the data are two blocking LPSPI transfers inside one
 * low period of a GPIO-driven chip select, so a frame of any length stays one
 * WIZnet transaction regardless of the LPSPI's own PCS framing. The board
 * initializes the LPSPI (LPSPI_MasterInit(): baud rate, mode 0, MSB first),
 * muxes its pads, and configures the chip-select GPIO as an output driven
 * high before the first transaction.
 *
 * acquire() holds nothing. The driver nests it (one register access inside
 * another), so a bus shared between threads would need a re-entrant lock,
 * which the platform contract does not offer; every board drives its chip
 * from one thread.
 *
 * References: i.MX RT1060 Reference Manual §48 (LPSPI); MCUXpresso SDK
 * fsl_lpspi LPSPI_MasterTransferBlocking(); W5500 datasheet §2.2 (SPI frame).
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "fsl_gpio.h"
#include "fsl_lpspi.h"
#include "imxrt_gpio_pin.h"

namespace rosette::platform {

class ImxrtLpspiBus {
public:
    using address_type = uint32_t;

    struct Config {
        LPSPI_Type* lpspi = nullptr;
        uint32_t    cs_pin = 0u;  ///< ROSETTE_IMXRT_GPIO(port, bit) of the chip select.
        uint8_t     read_fill = 0xFFu;
    };

    /**
     * @brief Binds the bus to an initialized LPSPI and its chip-select GPIO.
     *
     * @param[in] config The LPSPI instance (initialized by the board), the
     *   chip-select pin, and the octet clocked out while reading.
     *
     * Example:
     * @code{.cpp}
     * rosette::platform::ImxrtLpspiBus bus{{LPSPI4, ROSETTE_IMXRT_GPIO(2, 0)}};
     * rosette::net::wiznet::WiznetDevice<rosette::net::wiznet::W5500,
     *                                    rosette::platform::ImxrtLpspiBus> device{bus};
     * if (!device.verify()) { std::printf("no W5500\n"); }
     * @endcode
     * Full program: boards/common/wiznet/wiznet_board_network.cpp
     */
    explicit ImxrtLpspiBus(const Config& config) noexcept
        : lpspi_(config.lpspi),
          cs_port_(rosette_imxrt_gpio_port(config.cs_pin)),
          cs_bit_(rosette_imxrt_gpio_bit(config.cs_pin)) {
        if (lpspi_ != nullptr) {
            LPSPI_SetDummyData(lpspi_, config.read_fill);
        }
    }

    /// Holds the bus while it lives; see the file comment for why it holds nothing.
    struct Lock {
        ~Lock() {}
    };

    /// @brief Holds the bus for the returned object's lifetime.
    Lock acquire() noexcept { return {}; }

    /// @brief Sends `prefix`, then `length` octets of `data`, in one transaction.
    void write(const uint8_t* prefix, std::size_t prefix_length, const uint8_t* data,
               std::size_t length) noexcept {
        if (!ready() || prefix == nullptr || prefix_length == 0u || data == nullptr
            || length == 0u) {
            return;
        }

        select(true);
        transfer(prefix, nullptr, prefix_length);
        transfer(data, nullptr, length);
        select(false);
    }

    /// @brief Sends `prefix`, then clocks `length` octets into `data`, in one transaction.
    void read(const uint8_t* prefix, std::size_t prefix_length, uint8_t* data,
              std::size_t length) noexcept {
        if (!ready() || prefix == nullptr || prefix_length == 0u || data == nullptr
            || length == 0u) {
            return;
        }

        select(true);
        transfer(prefix, nullptr, prefix_length);
        transfer(nullptr, data, length);
        select(false);
    }

private:
    bool ready() const noexcept { return lpspi_ != nullptr && cs_port_ != nullptr; }

    /// Drives the active-low chip select: `true` asserts it.
    void select(bool asserted) noexcept { GPIO_PinWrite(cs_port_, cs_bit_, asserted ? 0u : 1u); }

    /// One blocking transfer; a null `tx` clocks out the read fill, a null `rx` discards.
    void transfer(const uint8_t* tx, uint8_t* rx, std::size_t length) noexcept {
        lpspi_transfer_t xfer{};
        xfer.txData = tx;
        xfer.rxData = rx;
        xfer.dataSize = length;
        xfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
        (void)LPSPI_MasterTransferBlocking(lpspi_, &xfer);
    }

    LPSPI_Type* lpspi_;
    GPIO_Type*  cs_port_;
    uint32_t    cs_bit_;
};

}  // namespace rosette::platform
