/**
 * @file wiznet_address_provider_test.cpp
 * @brief Compile + light behavioural coverage for the WIZnet address stack:
 *        `WiznetInterface` and `DhcpAddressProvider` declared straight into an
 *        `AddressManager` + `NetworkAddress`, the way an example does it.
 *
 * The chip is a `RecordingBus` (no hardware); the point is that the templates
 * instantiate and drive without crashing.
 */

#include "fake_clock.hpp"
#include "fake_wiznet_chip.hpp"
#include "test_util.hpp"

#include "ports/net/wiznet/dhcp_provider.hpp"
#include "ports/net/wiznet/interface.hpp"
#include "ports/net/wiznet/socket_provider.hpp"
#include "ports/net/wiznet/w5100s.hpp"
#include "rosette/net/address.hpp"
#include "rosette/net/ipv4ll_provider.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

// The header-only host-test target does not link `rosette_net`, so provide the
// one out-of-line symbol the DHCP exchange path references.
namespace rosette::net {
const char* socket_error_string(SocketError) noexcept { return "stub"; }
}  // namespace rosette::net

namespace {

struct RecordingBus {
    std::vector<std::vector<uint8_t>> writes;
    std::deque<uint8_t> next_reads;

    struct Lock { ~Lock() {} };
    Lock acquire() noexcept { return {}; }

    void write(const uint8_t*, std::size_t, const uint8_t* data, std::size_t len) {
        writes.push_back({data, data + len});
    }
    void read(const uint8_t*, std::size_t, uint8_t* data, std::size_t len) {
        for (std::size_t i = 0; i < len; ++i) {
            data[i] = next_reads.empty() ? 0u : next_reads.front();
            if (!next_reads.empty()) {
                next_reads.pop_front();
            }
        }
    }
};

namespace wiz = rosette::net::wiznet;
using Chip = wiz::W5100S;
using Device = wiz::WiznetDevice<Chip, RecordingBus>;
using Sockets = wiz::WiznetSocketProvider<Chip, RecordingBus>;

const std::array<uint8_t, 6> kMac{0x02, 0x08, 0xDC, 0x11, 0x22, 0x33};

}  // namespace

int main() {
    using namespace rosette::net;

    RecordingBus bus;
    Device device{bus};
    Sockets sockets{device};

    // ── WiznetInterface: apply writes registers, conflict starts clear ────
    {
        wiz::WiznetInterface<Chip, RecordingBus> iface{device};
        check(!iface.conflict(), "no conflict pending initially");
        AddressLease l{};
        l.family = AddressFamily::ipv4;
        l.add({192u, 168u, 1u, 5u}, 24u);
        iface.apply(l);
        check(!bus.writes.empty(), "apply() wrote chip registers");
        l.addresses.clear();
        iface.apply(l);    // clear path
    }

    // ── TC-NET-WIZ-IF-001: link-local gets mask 0.0.0.0 (RFC 3927 §2.6.2) ──
    {
        fake::FakeWiznetChip<Chip> chip;
        wiz::WiznetDevice<Chip, fake::FakeWiznetChip<Chip>> dev{chip};
        wiz::WiznetInterface<Chip, fake::FakeWiznetChip<Chip>> chip_iface{dev};
        AddressLease l{};
        l.family = AddressFamily::ipv4;
        l.add({169u, 254u, 17u, 78u}, 16u);
        l.gateway_v4 = {169u, 254u, 0u, 1u};
        chip_iface.apply(l);
        const auto ll = dev.ipv4_config();
        check(ll.ip == std::array<uint8_t, 4>{169u, 254u, 17u, 78u}, "IF-001: SIPR = link-local");
        check(ll.subnet == std::array<uint8_t, 4>{} && ll.gateway == std::array<uint8_t, 4>{},
              "IF-001: link-local is written with mask 0.0.0.0 and no gateway");

        l.addresses.clear();
        l.add({192u, 168u, 32u, 79u}, 24u);
        l.gateway_v4 = {192u, 168u, 32u, 1u};
        chip_iface.apply(l);
        const auto routable = dev.ipv4_config();
        check(routable.subnet == std::array<uint8_t, 4>{255u, 255u, 255u, 0u}
                  && routable.gateway == std::array<uint8_t, 4>{192u, 168u, 32u, 1u},
              "IF-001: a routable lease keeps its mask and gateway");
    }

    // ── providers declared straight into a manager + NetworkAddress ───────
    {
        wiz::WiznetInterface<Chip, RecordingBus> iface{device};
        wiz::DhcpAddressProvider dhcp{sockets, kMac, iface, wiz::dhcp_config("host-test")};
        Ipv4LinkLocalProvider zeroconf{kMac, iface};

        AddressProvider* v4l[] = {&dhcp, &zeroconf};
        AddressManager mgr{v4l};
        NetworkAddress net{mgr};   // link supervision covered in network_address_test
        net.begin();
        check(!net.stable(), "not stable yet");
        // Drives DHCP's bounded exchange against the dead bus; it fails fast and
        // the cascade moves on. Just check it does not crash and keeps running.
        net.poll(fake::now());
        net.poll(fake::now());
        check(net.snapshot().active, "network address stays active across polls");
    }

    return test_summary();
}
