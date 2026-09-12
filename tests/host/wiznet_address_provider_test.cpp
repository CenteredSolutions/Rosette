/**
 * @file wiznet_address_provider_test.cpp
 * @brief Compile + light behavioural coverage for the WIZnet address stack:
 *        `WiznetInterface` and `DhcpAddressProvider` declared straight into an
 *        `AddressManager` + `NetworkAddress`, the way an example does it.
 *
 * The chip is a `RecordingBus` (no hardware); the point is that the templates
 * instantiate and drive without crashing.
 */

#include "fake_runtime.hpp"
#include "test_util.hpp"

#include "rosette/net/ipv4ll_provider.hpp"
#include "rosette/net/address.hpp"
#include "rosette/net/wiznet/dhcp_provider.hpp"
#include "rosette/net/wiznet/interface.hpp"
#include "rosette/net/wiznet/w5100s.hpp"

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
        l.has_address = true;
        l.address = {192u, 168u, 1u, 5u};
        l.prefix_len = 24u;
        iface.apply(l);
        check(!bus.writes.empty(), "apply() wrote chip registers");
        l.has_address = false;
        iface.apply(l);    // clear path
    }

    // ── providers declared straight into a manager + NetworkAddress ───────
    {
        wiz::WiznetInterface<Chip, RecordingBus> iface{device};
        wiz::DhcpAddressProvider<Chip, RecordingBus> dhcp{
                device, sockets, iface, wiz::dhcp_config("host-test")};
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
