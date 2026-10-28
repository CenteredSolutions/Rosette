/**
 * @file wiznet_udp_socket_test.cpp
 * @brief `WiznetDevice` UDP receive and send against a register-file model of
 *        each chip, IPv4 and IPv6.
 *
 * The W5100S and the W6100/W6300 store a different packet-info header in
 * front of every received datagram (`Chip::read_udp_rx_header()`), and the
 * W61xx/W63xx send IPv6 with SEND6 to Sn_DIP6R. These tests run the driver's
 * real receive and send paths on `fake::FakeWiznetChip` and check payload,
 * sender, ring position, and the registers the chip would transmit from.
 *
 * Build and run: `cmake -S tests/host -B build/tests-host && ctest --test-dir build/tests-host`.
 */

#include "fake_clock.hpp"
#include "fake_wiznet_chip.hpp"
#include "test_util.hpp"

#include "ports/net/wiznet/device.hpp"
#include "ports/net/wiznet/w5100s.hpp"
#include "ports/net/wiznet/w6x00.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace {

namespace wiz = rosette::net::wiznet;

const std::array<uint8_t, 6> kMac{0x02, 0x08, 0xDC, 0x11, 0x22, 0x33};
const std::array<uint8_t, 16> kPeer6{0xFE, 0x80, 0, 0, 0, 0, 0, 0,
                                     0x02, 0x1C, 0x42, 0xFF, 0xFE, 0x14, 0x87, 0x37};

/// The octets a W6100/W6300 stores for one received datagram.
std::vector<uint8_t> w6x00_record(bool ipv6, const std::vector<uint8_t>& address,
                                  uint16_t port, const std::vector<uint8_t>& payload)
{
    const auto len = static_cast<uint16_t>(payload.size());
    std::vector<uint8_t> r{static_cast<uint8_t>((ipv6 ? 0x80u : 0x00u) | ((len >> 8) & 0x07u)),
                           static_cast<uint8_t>(len)};
    r.insert(r.end(), address.begin(), address.end());
    r.push_back(static_cast<uint8_t>(port >> 8));
    r.push_back(static_cast<uint8_t>(port));
    r.insert(r.end(), payload.begin(), payload.end());
    return r;
}

/// The octets a W5100S stores for one received datagram.
std::vector<uint8_t> w5100s_record(const std::vector<uint8_t>& address, uint16_t port,
                                   const std::vector<uint8_t>& payload)
{
    const auto len = static_cast<uint16_t>(payload.size());
    std::vector<uint8_t> r(address.begin(), address.end());
    r.push_back(static_cast<uint8_t>(port >> 8));
    r.push_back(static_cast<uint8_t>(port));
    r.push_back(static_cast<uint8_t>(len >> 8));
    r.push_back(static_cast<uint8_t>(len));
    r.insert(r.end(), payload.begin(), payload.end());
    return r;
}

std::vector<uint8_t> v(std::initializer_list<uint8_t> bytes) { return bytes; }

std::string tag(const char* chip, const char* what) { return std::string(chip) + ": " + what; }

/**
 * TC-NET-WIZ-RX-004: a packet-info header whose datagram is longer than
 * Sn_RX_RSR is off a datagram boundary. The receive reports -1, drops
 * everything queued (the valid datagram behind it too), counts one resync, and
 * the next datagram the chip stores reads intact. `make_record(len_field,
 * payload)` builds the chip's record with an arbitrary length field.
 */
template <typename Chip, typename MakeRecord>
void check_rx_resync(const char* name, MakeRecord&& make_record) {
    using Chipset = fake::FakeWiznetChip<Chip>;
    Chipset                          chip;
    wiz::WiznetDevice<Chip, Chipset> dev{chip};
    dev.init_socket_mode(kMac);
    check(dev.open_udp_socket(1, 7410), tag(name, "resync: socket opens").c_str());

    chip.deliver(1, make_record(600u, std::vector<uint8_t>(40u, 0x11u)));
    chip.deliver(1, make_record(3u, v({'o', 'k', '?'})));
    std::array<uint8_t, 1472> buf{};
    wiz::UdpEndpoint          from{};
    check(dev.udp_receive_from(1, buf.data(), buf.size(), &from) == -1,
          tag(name, "resync: a header longer than the received size reports -1").c_str());
    check(dev.rx_resyncs() == 1u, tag(name, "resync: counted once").c_str());
    check(dev.socket_rx_available(1) == 0u
              && dev.udp_receive_from(1, buf.data(), buf.size(), &from) == 0,
          tag(name, "resync: the ring is empty afterwards").c_str());

    chip.deliver(1, make_record(5u, v({'h', 'e', 'l', 'l', 'o'})));
    check(dev.udp_receive_from(1, buf.data(), buf.size(), &from) == 5 && buf[0] == 'h'
              && buf[4] == 'o',
          tag(name, "resync: the next datagram reads intact").c_str());

    chip.deliver(1, make_record(3000u, std::vector<uint8_t>(40u, 0x22u)));
    check(dev.udp_receive_from(1, buf.data(), buf.size(), &from) == -1 && dev.rx_resyncs() == 2u
              && dev.socket_rx_available(1) == 0u,
          tag(name,
              "resync: a second misaligned header resyncs again rather than skipping its length")
              .c_str());
}

template<typename Chip>
void check_w6x00(const char* name)
{
    using Chipset = fake::FakeWiznetChip<Chip>;
    using Device = wiz::WiznetDevice<Chip, Chipset>;
    Chipset chip;
    Device dev{chip};
    dev.init_socket_mode(kMac);

    // IPv4 datagram (TC-NET-WIZ-RX-001).
    check(dev.open_udp_socket(1, 7400), tag(name, "IPv4 socket opens").c_str());
    check(chip.u8(Chip::socket_mode(1)) == 0x02u, tag(name, "IPv4 socket uses UDP4 mode").c_str());
    chip.deliver(1, w6x00_record(false, v({192, 168, 32, 96}), 51000, v({'h', 'i', '!'})));
    std::array<uint8_t, 64> buf{};
    wiz::UdpEndpoint from{};
    auto n = dev.udp_receive_from(1, buf.data(), buf.size(), &from);
    check(n == 3 && buf[0] == 'h' && buf[2] == '!', tag(name, "IPv4 payload read after the 8-octet header").c_str());
    check(!from.ipv6 && from.ipv4() == std::array<uint8_t, 4>{192, 168, 32, 96} && from.port == 51000,
          tag(name, "IPv4 sender address and port decoded").c_str());
    check(dev.socket_rx_available(1) == 0u, tag(name, "ring consumed exactly one datagram").c_str());

    // IPv6 datagram (TC-NET-WIZ-RX-002).
    check(dev.open_udp_socket(2, 7400, /*ipv6=*/true), tag(name, "IPv6 socket opens").c_str());
    check(chip.u8(Chip::socket_mode(2)) == 0x0Au, tag(name, "IPv6 socket uses UDP6 mode").c_str());
    chip.deliver(2, w6x00_record(true, {kPeer6.begin(), kPeer6.end()}, 40000, v({1, 2, 3, 4, 5})));
    chip.deliver(2, w6x00_record(true, {kPeer6.begin(), kPeer6.end()}, 40001, v({9})));
    n = dev.udp_receive_from(2, buf.data(), buf.size(), &from);
    check(n == 5 && buf[4] == 5, tag(name, "IPv6 payload read after the 20-octet header").c_str());
    check(from.ipv6 && from.address == kPeer6 && from.port == 40000,
          tag(name, "IPv6 sender address and port decoded").c_str());
    n = dev.udp_receive_from(2, buf.data(), buf.size(), &from);
    check(n == 1 && buf[0] == 9 && from.port == 40001,
          tag(name, "second datagram follows the first in the ring").c_str());

    // Wrap-around and oversize (TC-NET-WIZ-RX-003).
    chip.set_rx_pointer(3, static_cast<uint16_t>(2048u - 10u));
    check(dev.open_udp_socket(3, 7401, true), tag(name, "third socket opens").c_str());
    chip.set_rx_pointer(3, static_cast<uint16_t>(2048u - 10u));
    std::vector<uint8_t> long_payload(40);
    for (std::size_t i = 0; i < long_payload.size(); ++i) long_payload[i] = static_cast<uint8_t>(i);
    chip.deliver(3, w6x00_record(true, {kPeer6.begin(), kPeer6.end()}, 1, long_payload));
    n = dev.udp_receive_from(3, buf.data(), buf.size(), &from);
    check(n == 40 && buf[39] == 39 && from.address == kPeer6,
          tag(name, "datagram spanning the ring end reads intact").c_str());
    chip.deliver(3, w6x00_record(true, {kPeer6.begin(), kPeer6.end()}, 2, long_payload));
    chip.deliver(3, w6x00_record(true, {kPeer6.begin(), kPeer6.end()}, 3, v({7})));
    std::array<uint8_t, 8> small{};
    check(dev.udp_receive_from(3, small.data(), small.size(), &from) == -1,
          tag(name, "oversize datagram reported as -1").c_str());
    n = dev.udp_receive_from(3, small.data(), small.size(), &from);
    check(n == 1 && small[0] == 7 && from.port == 3, tag(name, "oversize datagram skipped whole").c_str());

    // IPv6 send (TC-NET-WIZ-TX-001).
    wiz::UdpEndpoint to{};
    to.ipv6 = true;
    to.address = kPeer6;
    to.port = 7777;
    const uint8_t hello[] = {'h', 'e', 'l', 'l', 'o'};
    check(dev.udp_send_to(2, hello, sizeof(hello), to) == 5, tag(name, "IPv6 send accepted").c_str());
    check(chip.sent.size() == 1 && chip.sent[0].command == 0xA0u,
          tag(name, "IPv6 send issues SEND6").c_str());
    check(chip.sent[0].dest_ip6 == std::vector<uint8_t>(kPeer6.begin(), kPeer6.end()) &&
              chip.sent[0].dest_port == 7777 &&
              chip.sent[0].payload == std::vector<uint8_t>(hello, hello + 5),
          tag(name, "IPv6 send writes Sn_DIP6R, Sn_DPORTR, and the payload").c_str());

    wiz::UdpEndpoint to4{};
    to4.address = {192, 168, 32, 96};
    to4.port = 7777;
    check(dev.udp_send_to(1, hello, 2, to4) == 2 && chip.sent.size() == 2 &&
              chip.sent[1].command == 0x20u &&
              chip.sent[1].dest_ip == std::vector<uint8_t>{192, 168, 32, 96},
          tag(name, "IPv4 send issues SEND to Sn_DIPR").c_str());
}

void check_w5100s()
{
    using Chip = wiz::W5100S;
    using Chipset = fake::FakeWiznetChip<Chip>;
    using Device = wiz::WiznetDevice<Chip, Chipset>;
    Chipset chip;
    Device dev{chip};
    dev.init_socket_mode(kMac);

    check(dev.open_udp_socket(1, 7400), "W5100S: IPv4 socket opens");
    check(!dev.open_udp_socket(2, 7400, true), "W5100S: IPv6 socket refused (no hardware IPv6)");

    chip.set_rx_pointer(1, static_cast<uint16_t>(2048u - 4u));
    chip.deliver(1, w5100s_record(v({10, 0, 0, 7}), 1234, v({'a', 'b', 'c', 'd', 'e', 'f'})));
    std::array<uint8_t, 64> buf{};
    wiz::UdpEndpoint from{};
    const auto n = dev.udp_receive_from(1, buf.data(), buf.size(), &from);
    check(n == 6 && buf[0] == 'a' && buf[5] == 'f', "W5100S: payload after {ip, port, length} header, across the ring end");
    check(!from.ipv6 && from.ipv4() == std::array<uint8_t, 4>{10, 0, 0, 7} && from.port == 1234,
          "W5100S: sender address and port decoded");

    wiz::UdpEndpoint to6{};
    to6.ipv6 = true;
    const uint8_t x = 1;
    check(dev.udp_send_to(1, &x, 1, to6) == -1 && chip.sent.empty(), "W5100S: IPv6 send refused");
}

}  // namespace

int main()
{
    check_w5100s();
    check_rx_resync<wiz::W5100S>("W5100S", [](uint16_t len, const std::vector<uint8_t>& payload) {
        auto r = w5100s_record(v({10, 0, 0, 7}), 1234, payload);
        r[6] = static_cast<uint8_t>(len >> 8);
        r[7] = static_cast<uint8_t>(len);
        return r;
    });
    const auto w6x00_with_length = [](uint16_t len, const std::vector<uint8_t>& payload) {
        auto r = w6x00_record(false, v({10, 0, 0, 7}), 1234, payload);
        r[0] = static_cast<uint8_t>((r[0] & 0xF8u) | ((len >> 8) & 0x07u));
        r[1] = static_cast<uint8_t>(len);
        return r;
    };
    check_rx_resync<wiz::W6100>("W6100", w6x00_with_length);
    check_rx_resync<wiz::W6300>("W6300", w6x00_with_length);
    check_w6x00<wiz::W6100>("W6100");
    check_w6x00<wiz::W6300>("W6300");
    return test_summary();
}
