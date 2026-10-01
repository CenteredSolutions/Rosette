#pragma once

/**
 * @file ports/net/wiznet/macraw_device.hpp
 * @brief A WIZnet chip in MACRAW mode, presented as a rosette::net::MacDevice.
 *
 * A WIZnet chip is either an IP stack (its hardware sockets, socket_provider.hpp)
 * or a plain Ethernet MAC. In MACRAW mode socket 0 owns the whole buffer memory
 * and carries raw Ethernet frames, so a software IP stack on the
 * microcontroller can drive it. This header provides that MAC through the
 * neutral contract in rosette/net/mac.hpp; it depends on no IP stack and no
 * RTOS, and an IP-stack port (ports/net/freertos_tcp) consumes it through that
 * contract alone.
 */

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <variant>

#include "device.hpp"
#include "rosette/net/mac.hpp"

namespace rosette::net::wiznet {

/**
 * @brief WIZnet MACRAW driver behind the net::MacDevice contract.
 *
 * Wraps the register-level WiznetDevice so that every MacDevice operation is
 * one bus transaction against socket 0's TX or RX ring. Reading a frame
 * clears the socket's RECV interrupt before it reads the received size, so a
 * frame arriving afterwards drives the chip's INT pin low again; a consumer
 * that waits on that pin's falling edge therefore never sleeps through a frame.
 * Wiring INT to a GPIO is the board's job; this class never touches it.
 *
 * The device owns its bus and is templated on the concrete @p Chip register
 * map and @p Bus transport, so register accesses stay statically dispatched.
 * It is neither copied nor moved, because the embedded WiznetDevice holds a
 * reference to @c bus_; a consumer constructs it in place.
 *
 * Concurrency: send_frame() and the receive calls may run on different
 * tasks. Each takes Bus::acquire() for its whole transaction, so the bus must
 * provide real mutual exclusion when more than one task uses the device.
 *
 * @tparam Chip Compile-time register map: addresses, socket assignment, and
 *              buffer sizes for the target WIZnet controller.
 * @tparam Bus  Register transport providing scoped bus locking via acquire()
 *              (contract written down in ports/chip/stub/stub_bus.hpp); a
 *              value type is owned, a reference type is borrowed.
 */
template <typename Chip, typename Bus>
class WiznetMacrawDevice final : public net::MacDevice {
public:
    using chip_type = Chip;
    using bus_type = Bus;
    using raw_device_type = WiznetDevice<Chip, Bus>;

    /**
     * @brief Takes ownership of the chip's bus; the chip is not touched until initialise().
     *
     * @param[in] bus Transport to the chip, moved in and owned for the
     *   device's lifetime; when @p Bus is a reference type, the device
     *   refers to a bus the caller owns, which must outlive the device.
     */
    explicit WiznetMacrawDevice(Bus bus) noexcept : bus_(std::forward<Bus>(bus)), device_(bus_) {}

    WiznetMacrawDevice(const WiznetMacrawDevice&) = delete;
    WiznetMacrawDevice& operator=(const WiznetMacrawDevice&) = delete;

    /**
     * @brief Resets the chip into MACRAW mode on the first call; later calls only reprogram the
     * MAC.
     *
     * The first successful call verifies the chip version, resets it, gives
     * socket 0 the whole buffer memory, unmasks its RECV interrupt, and opens
     * it in MACRAW mode (WiznetDevice::init_macraw()). An IP stack may
     * re-initialise its interface after a link change; resetting the chip
     * then would drop queued frames, so those calls only latch @p mac.
     *
     * @param[in] mac Station hardware address to program.
     * @return True once socket 0 carries frames; false when the chip does not
     *   answer or reports an unexpected version, leaving the device
     *   uninitialised so the next call retries the reset.
     *
     * Verification:
     * - Correct means: after true, Sn_MR of socket 0 holds the MACRAW mode
     *   and SHAR holds @p mac; with a wrong version register the call
     *   returns false.
     * - Covered by: tests/host/wiznet_macraw_device_test.cpp (TC-NET-WIZ-MACRAW-001).
     *
     * References:
     * - W5100S, W6100, and W6300 datasheets: the chip version register, §SHAR, §Sn_MR (MACRAW)
     */
    bool initialise(const MacAddress& mac) noexcept override {
        if (started_) {
            device_.set_mac_address(mac);
            return true;
        }

        if (!device_.verify()) {
            return false;
        }

        auto result = device_.init_macraw(mac);
        if (auto* err = std::get_if<typename raw_device_type::InvalidChipVersion>(&result)) {
            std::printf("[wiznet] Invalid chip version: %u (expected %u)\n", err->actual,
                        err->expected);
            return false;
        }

        started_ = true;
        return true;
    }

    /// @brief Writes @p mac to the chip's source hardware address register.
    void set_mac_address(const MacAddress& mac) noexcept override { device_.set_mac_address(mac); }

    /// @brief The chip's largest MACRAW frame, header and payload without FCS.
    std::size_t max_frame_size() const noexcept override {
        return static_cast<std::size_t>(Chip::max_frame_bytes());
    }

    /// @brief Queues one frame on socket 0's TX ring and issues SEND (WiznetDevice::write_frame()).
    bool send_frame(const uint8_t* data, std::size_t length) noexcept override {
        return device_.write_frame(data, length);
    }

    /// @brief Consumes the head frame of socket 0's RX ring (WiznetDevice::read_frame()).
    std::size_t receive_frame(uint8_t* buffer, std::size_t capacity) noexcept override {
        return device_.read_frame(buffer, capacity);
    }

    /// @brief Length of the head frame, decoded from its MACRAW header
    /// (WiznetDevice::peek_frame_size()).
    std::size_t peek_frame_size() noexcept override { return device_.peek_frame_size(); }

    /// @brief Carrier state from the chip's PHY link register; speed and duplex stay unknown.
    PhyStatus phy_status() noexcept override {
        PhyStatus status{};
        status.link_up = device_.is_link_up();
        return status;
    }

private:
    Bus bus_;
    raw_device_type device_;
    bool started_ = false;
};

}  // namespace rosette::net::wiznet
