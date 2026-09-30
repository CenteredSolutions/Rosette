/**
 * @file wiznet_socket_layout_test.cpp
 * @brief WIZnet receive-buffer layout and the provider's size-aware socket claim.
 *
 * A WIZnet chip splits its receive memory across its hardware sockets, and a
 * datagram that no longer fits a socket's buffer is dropped. The driver sizes
 * the buffers unevenly (WiznetDevice::set_rx_layout(): one bulk, one medium,
 * the rest small), a plain open() takes the smallest free socket, and
 * Socket::set_receive_buffer_size() moves a socket, before bind(), to the
 * smallest buffer that holds the request, or else the largest. The RTPS user
 * socket, which receives every fragment of a large sample back to back, so
 * lands on the bulk buffer whatever order the board's protocols open in.
 *
 * Test cases: TC-NET-WIZ-LAYOUT-001 (layouts validated and written to
 * Sn_RXBUF_SIZE; transmit memory split evenly), TC-NET-WIZ-LAYOUT-002
 * (smallest-first claim, best-fit and largest-fallback requests, refusal
 * after bind(), zero-size sockets never handed out).
 *
 * Build and run: `cmake -S tests/host -B build/tests-host && ctest --test-dir build/tests-host`.
 */

#include "fake_clock.hpp"
#include "fake_wiznet_chip.hpp"
#include "test_util.hpp"

#include "rosette/net/wiznet/device.hpp"
#include "rosette/net/wiznet/socket_provider.hpp"
#include "rosette/net/wiznet/w5100s.hpp"
#include "rosette/net/wiznet/w6x00.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

// The header-only host-test target does not link `rosette_net`.
namespace rosette::net {
const char* socket_error_string(SocketError) noexcept {
    return "stub";
}
}  // namespace rosette::net

namespace {

namespace wiz = rosette::net::wiznet;
using rosette::net::Socket;
using rosette::net::SocketDomain;
using rosette::net::SocketError;

const std::array<uint8_t, 6> kMac{0x02, 0x08, 0xDC, 0x44, 0x55, 0x66};

/// What the RTPS user socket asks for with a 4096-octet largest sample.
constexpr std::size_t kUserRequest = 4096u + 4096u / 8u;

std::string tag(const char* chip, const char* what) {
    return std::string(chip) + ": " + what;
}

/// A chip model, its driver after socket-mode init, and a provider over it.
template <typename Chip>
struct Rig {
    using Chipset = fake::FakeWiznetChip<Chip>;
    using Device = wiz::WiznetDevice<Chip, Chipset>;
    using Sockets = wiz::WiznetSocketProvider<Chip, Chipset>;

    Chipset chip;
    Device device{chip};
    Sockets sockets{device};

    explicit Rig(const typename Device::RxLayout* layout = nullptr) {
        if (layout != nullptr) {
            (void)device.set_rx_layout(*layout);
        }

        device.init_socket_mode(kMac);
    }

    /// The hardware socket `socket` sits on: bind it to `port` and find the
    /// socket whose Sn_PORT the driver wrote.
    int hardware_socket(Socket& socket, uint16_t port) {
        if (socket.bind(rosette::net::make_socket_address_v4({0u, 0u, 0u, 0u}, port)) != 0) {
            return -1;
        }

        for (uint8_t s = 0; s < Chip::socket_count(); ++s) {
            if (chip.get_u16(Chip::socket_source_port(s)) == port) {
                return s;
            }
        }

        return -1;
    }
};

// TC-NET-WIZ-LAYOUT-001: each chip's default layout is valid, init writes it
// to Sn_RXBUF_SIZE with the buffers back to back, transmit memory stays
// even, and invalid layouts are refused without changing the current one.
template <typename Chip>
void check_layout(const char* name, const std::array<uint16_t, Chip::socket_count()>& want_rx) {
    using R = Rig<Chip>;
    R rig;
    check(R::Device::is_valid_rx_layout(Chip::default_rx_layout_kib()),
          tag(name, "default layout is valid").c_str());

    bool rx_ok = true;
    bool tx_ok = true;
    for (uint8_t s = 0; s < Chip::socket_count(); ++s) {
        rx_ok = rx_ok && rig.device.socket_rx_capacity(s) == want_rx[s]
                && rig.chip.u8(Chip::socket_rx_buffer_size(s)) == want_rx[s] / 1024u;
        tx_ok = tx_ok
                && rig.device.socket_tx_capacity(s)
                       == Chip::socket_tx_buffer_kib() * 1024u / Chip::socket_count();
    }

    check(rx_ok,
          tag(name, "Sn_RXBUF_SIZE and the cached capacities follow the default layout").c_str());
    check(tx_ok, tag(name, "transmit memory is split evenly").c_str());

    typename R::Device::RxLayout bad{};
    bad[0] = 3u;
    check(!R::Device::is_valid_rx_layout(bad),
          tag(name, "a size that is not a power of two is refused").c_str());
    bad[0] = Chip::socket_rx_buffer_kib();
    bad[1] = 1u;
    check(!R::Device::is_valid_rx_layout(bad),
          tag(name, "a layout beyond the chip's memory is refused").c_str());
    check(!rig.device.set_rx_layout(bad) && rig.device.rx_layout() == Chip::default_rx_layout_kib(),
          tag(name, "set_rx_layout() keeps the current layout when refusing").c_str());
    bad[1] = 0u;
    check(R::Device::is_valid_rx_layout(bad),
          tag(name, "all memory on one socket is valid").c_str());
}

// TC-NET-WIZ-LAYOUT-002 on the W6100's default 8/2/1/1/1/1/1/1 KiB layout.
void check_claims_w6100() {
    using Chip = wiz::W6100;
    Rig<Chip> rig;

    // A plain open takes the smallest buffer, so a protocol that opens first
    // (DHCP) leaves the bulk buffer for the RTPS user socket.
    auto dhcp = rig.sockets.open(SocketDomain::ipv4);
    check(dhcp && dhcp->receive_buffer_size() == 1024u, "W6100: a plain open gets a 1 KiB buffer");
    check(rig.hardware_socket(*dhcp, 68u) == 2, "W6100: the lowest-numbered 1 KiB socket (2)");

    auto user = rig.sockets.open(SocketDomain::ipv4);
    check(user && user->set_receive_buffer_size(kUserRequest) == 0,
          "W6100: a request before bind() is accepted");
    check(user->receive_buffer_size() == 8192u, "W6100: a 4.5 KB request gets the 8 KiB buffer");
    auto meta = rig.sockets.open(SocketDomain::ipv4);
    check(meta && meta->set_receive_buffer_size(2048u) == 0 && meta->receive_buffer_size() == 2048u,
          "W6100: a 2 KiB request gets the 2 KiB buffer, not the larger one");

    // Swapping away frees the socket first held: the user socket began on the
    // lowest free 1 KiB socket (3), and the next plain open gets it back.
    auto next = rig.sockets.open(SocketDomain::ipv4);
    check(next && rig.hardware_socket(*next, 7000u) == 3,
          "W6100: the socket a request moved away from is free again");
    check(rig.hardware_socket(*user, 7411u) == 0 && rig.hardware_socket(*meta, 7410u) == 1,
          "W6100: the user socket binds socket 0 and the metatraffic socket socket 1");

    check(user->set_receive_buffer_size(1024u) == static_cast<int>(SocketError::invalid_argument)
              && user->receive_buffer_size() == 8192u,
          "W6100: a request after bind() is refused and the buffer kept");

    // With the bulk and medium buffers taken, a large request gets the
    // largest buffer left rather than failing.
    auto late = rig.sockets.open(SocketDomain::ipv4);
    check(late && late->set_receive_buffer_size(kUserRequest) == 0
              && late->receive_buffer_size() == 1024u,
          "W6100: with no buffer large enough, the largest free one (1 KiB)");
}

// TC-NET-WIZ-LAYOUT-002 on the W5100S's default 4/2/1/1 KiB layout, and a
// layout that leaves sockets without receive memory.
void check_claims_w5100s() {
    using Chip = wiz::W5100S;
    {
        Rig<Chip> rig;
        auto user = rig.sockets.open(SocketDomain::ipv4);
        check(user && user->set_receive_buffer_size(kUserRequest) == 0
                  && user->receive_buffer_size() == 4096u,
              "W5100S: a 4.5 KB request gets the largest buffer, 4 KiB");
        check(rig.hardware_socket(*user, 7411u) == 0, "W5100S: which is socket 0");
    }

    {
        // An even layout keeps the chip's reset split: 2 KiB spans from 0x6000.
        const Rig<Chip>::Device::RxLayout even{2u, 2u, 2u, 2u};
        Rig<Chip> rig(&even);
        check(
            rig.device.rx_buffer_address(3) == 0x7800u && rig.device.socket_rx_capacity(3) == 2048u,
            "W5100S: an even layout puts socket 3 at 0x7800 with 2 KiB");
    }

    {
        const Rig<Chip>::Device::RxLayout two_sockets{4u, 4u, 0u, 0u};
        Rig<Chip> rig(&two_sockets);
        auto a = rig.sockets.open(SocketDomain::ipv4);
        auto b = rig.sockets.open(SocketDomain::ipv4);
        auto c = rig.sockets.open(SocketDomain::ipv4);
        check(a && b && !c, "W5100S: sockets with no receive memory are never handed out");
    }
}

}  // namespace

int main() {
    check_layout<wiz::W5100S>("W5100S", {4096u, 2048u, 1024u, 1024u});
    check_layout<wiz::W6100>("W6100", {8192u, 2048u, 1024u, 1024u, 1024u, 1024u, 1024u, 1024u});
    check_layout<wiz::W6300>("W6300", {16384u, 4096u, 2048u, 2048u, 2048u, 2048u, 2048u, 2048u});
    check_claims_w6100();
    check_claims_w5100s();
    return test_summary();
}
