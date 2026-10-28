/**
 * @file wiznet_macraw_device_test.cpp
 * @brief `WiznetMacrawDevice` against the rosette::net::MacDevice contract, on a register-file
 * model of each chip.
 *
 * The MACRAW device is the WIZnet port's MAC provider (rosette/net/mac.hpp):
 * an IP-stack port sizes its buffer from peek_frame_size(), copies the head
 * frame with receive_frame(), and transmits with send_frame(). These tests
 * run the driver's real MACRAW paths through the `MacDevice` interface on
 * `fake::FakeWiznetChip` and check what the contract promises a consumer:
 * initialisation, one exact head-frame length at a time, in-order delivery,
 * discarding of records no buffer could hold, and transmitted bytes.
 *
 * Build and run: `cmake -S tests/host -B build/tests-host && ctest --test-dir build/tests-host`.
 */

#include "fake_wiznet_chip.hpp"
#include "test_util.hpp"

#include "ports/net/wiznet/macraw_device.hpp"
#include "ports/net/wiznet/w5100s.hpp"
#include "ports/net/wiznet/w6x00.hpp"
#include "rosette/net/mac.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace {

namespace wiz = rosette::net::wiznet;

const rosette::net::MacAddress kMac{0x02, 0x08, 0xDC, 0x11, 0x22, 0x33};
const rosette::net::MacAddress kMac2{0x02, 0x08, 0xDC, 0x44, 0x55, 0x66};

/// The octets the chip stores for one MACRAW frame: a big-endian length that counts itself, then
/// the frame.
std::vector<uint8_t> macraw_record(const std::vector<uint8_t>& frame) {
    const auto total = static_cast<uint16_t>(frame.size() + 2u);
    std::vector<uint8_t> r(frame.size() + 2u);
    r[0] = static_cast<uint8_t>(total >> 8);
    r[1] = static_cast<uint8_t>(total);
    std::copy(frame.begin(), frame.end(), r.begin() + 2);
    return r;
}

/// A frame of `length` octets whose bytes count up from `first`.
std::vector<uint8_t> frame_of(std::size_t length, uint8_t first) {
    std::vector<uint8_t> f(length);
    for (std::size_t i = 0; i < length; ++i) {
        f[i] = static_cast<uint8_t>(first + i);
    }

    return f;
}

std::string tag(const char* chip, const char* what) {
    return std::string(chip) + ": " + what;
}

template <typename Chip>
using Fake = fake::FakeWiznetChip<Chip>;

/// Runs every case on one chip model; the device reaches the fake through `Fake<Chip>&` as its bus.
template <typename Chip>
void check_chip(const char* name) {
    Fake<Chip> chip;
    wiz::WiznetMacrawDevice<Chip, Fake<Chip>&> device{chip};
    rosette::net::MacDevice& mac = device;

    // TC-NET-WIZ-MACRAW-001: initialise() verifies the chip, opens socket 0 in
    // MACRAW mode, and on a repeat call only reprograms the station address.
    chip.mem[Chip::common_version()] = static_cast<uint8_t>(~Chip::chip_version());
    check(!mac.initialise(kMac), tag(name, "initialise() fails on a wrong chip version").c_str());
    chip.mem[Chip::common_version()] = Chip::chip_version();
    check(mac.initialise(kMac), tag(name, "initialise() succeeds once the chip answers").c_str());
    check(chip.u8(Chip::socket_mode(Chip::macraw_socket())) == Chip::macraw_mode(),
          tag(name, "socket 0 is in MACRAW mode").c_str());
    check(chip.bytes(Chip::common_mac_address(), 6u)
              == std::vector<uint8_t>(kMac.begin(), kMac.end()),
          tag(name, "SHAR holds the station address").c_str());
    check(mac.initialise(kMac2)
              && chip.bytes(Chip::common_mac_address(), 6u)
                     == std::vector<uint8_t>(kMac2.begin(), kMac2.end()),
          tag(name, "repeat initialise() reprograms SHAR").c_str());

    // TC-NET-WIZ-MACRAW-002: send_frame() transmits the frame's octets
    // unchanged from socket 0, and max_frame_size() is the chip's limit.
    const auto out = frame_of(60u, 0x10u);
    check(mac.send_frame(out.data(), out.size()),
          tag(name, "send_frame() accepts a frame").c_str());
    check(chip.sent.size() == 1u && chip.sent[0].socket == Chip::macraw_socket()
              && chip.sent[0].command == Chip::command_send() && chip.sent[0].payload == out,
          tag(name, "the frame leaves socket 0 with SEND, octets unchanged").c_str());
    check(mac.max_frame_size() == Chip::max_frame_bytes(),
          tag(name, "max_frame_size() is the chip's MACRAW limit").c_str());

    // TC-NET-WIZ-MACRAW-003: with two frames queued, peek_frame_size() reports
    // the head frame alone, does not consume it, and receive_frame() delivers
    // the frames in order.
    check(mac.peek_frame_size() == 0u, tag(name, "empty ring peeks 0").c_str());
    const auto a = frame_of(64u, 0xA0u);
    const auto b = frame_of(98u, 0x40u);
    chip.deliver(Chip::macraw_socket(), macraw_record(a));
    chip.deliver(Chip::macraw_socket(), macraw_record(b));
    check(mac.peek_frame_size() == a.size() && mac.peek_frame_size() == a.size(),
          tag(name, "peek reports the head frame, not the queue, and consumes nothing").c_str());
    std::vector<uint8_t> buf(a.size());
    check(mac.receive_frame(buf.data(), buf.size()) == a.size() && buf == a,
          tag(name, "receive_frame() delivers the head frame into a buffer of the peeked size")
              .c_str());
    check(mac.peek_frame_size() == b.size(),
          tag(name, "peek then reports the second frame").c_str());
    buf.assign(b.size(), 0u);
    check(mac.receive_frame(buf.data(), buf.size()) == b.size() && buf == b,
          tag(name, "receive_frame() delivers the second frame").c_str());
    check(mac.peek_frame_size() == 0u, tag(name, "the drained ring peeks 0").c_str());

    // TC-NET-WIZ-MACRAW-004: a record longer than max_frame_size() is
    // discarded by peek_frame_size(), and the frame behind it is intact.
    const auto oversize = frame_of(Chip::max_frame_bytes() + 1u, 0x00u);
    const auto c = frame_of(42u, 0x77u);
    chip.deliver(Chip::macraw_socket(), macraw_record(oversize));
    chip.deliver(Chip::macraw_socket(), macraw_record(c));
    check(mac.peek_frame_size() == 0u, tag(name, "an oversize record peeks 0").c_str());
    check(mac.peek_frame_size() == c.size(),
          tag(name, "the frame after an oversize record is next").c_str());
    buf.assign(c.size(), 0u);
    check(mac.receive_frame(buf.data(), buf.size()) == c.size() && buf == c,
          tag(name, "the frame after an oversize record is delivered intact").c_str());

    // TC-NET-WIZ-MACRAW-006: a length header longer than the received size
    // means the ring is off a frame boundary; everything queued is dropped
    // and the next frame the chip stores reads intact.
    std::vector<uint8_t> misaligned{0x05u, 0xDCu};  // claims 1500 octets
    const auto           behind = frame_of(30u, 0x20u);
    misaligned.insert(misaligned.end(), 40u, 0x55u);
    chip.deliver(Chip::macraw_socket(), misaligned);
    chip.deliver(Chip::macraw_socket(), macraw_record(behind));
    check(mac.peek_frame_size() == 0u,
          tag(name, "a header longer than the received size peeks 0").c_str());
    check(mac.peek_frame_size() == 0u,
          tag(name, "the misaligned ring is emptied, frame behind it included").c_str());
    const auto d = frame_of(50u, 0x90u);
    chip.deliver(Chip::macraw_socket(), macraw_record(d));
    buf.assign(d.size(), 0u);
    check(mac.peek_frame_size() == d.size() && mac.receive_frame(buf.data(), buf.size()) == d.size()
              && buf == d,
          tag(name, "the next frame after a resync is delivered intact").c_str());
    chip.deliver(Chip::macraw_socket(), misaligned);
    buf.assign(Chip::max_frame_bytes(), 0u);
    check(mac.receive_frame(buf.data(), buf.size()) == 0u && mac.peek_frame_size() == 0u,
          tag(name, "receive_frame() also resyncs on a header longer than the received size")
              .c_str());

    // TC-NET-WIZ-MACRAW-005: phy_status() follows the PHY link register.
    check(!mac.phy_status().link_up, tag(name, "link down while the PHY reports none").c_str());
    chip.mem[Chip::common_link_status()] = Chip::link_up_mask();
    check(mac.phy_status().link_up, tag(name, "link up once the PHY reports carrier").c_str());
}

}  // namespace

int main() {
    check_chip<wiz::W5100S>("W5100S");
    check_chip<wiz::W6100>("W6100");
    check_chip<wiz::W6300>("W6300");
    return test_summary();
}
