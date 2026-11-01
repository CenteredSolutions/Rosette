// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_transport_multicast_interface.cpp
 * @brief TC-TRN-9.6.1-001..002: SPDP multicast is joined on and sent from the interface the
 * participant advertises.
 *
 * A participant asks peers that hear its SPDP announcement to answer at its
 * unicast locators, so the announcement must leave on the interface holding
 * that address, and the group must be joined there. These cases open a real
 * SocketUdpTransport over a recording socket backend and check what each
 * socket was asked to do, and in which order.
 *
 * References: OMG DDSI-RTPS 2.5 §8.5.3 (SPDP), §9.6.1.1 (port mapping);
 * rosette/net/socket.hpp (Socket::set_multicast_interface).
 */

#include "participant_test_support.hpp"

#include <array>
#include <string>

namespace spec_test {

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

namespace net = rosette::net;

constexpr uint32_t kInterface = 0x0A2A0001u;  // 10.42.0.1

/// A socket backend that records, per socket, every interface and bind call in order.
class RecordingSocketProvider final : public net::SocketProvider {
public:
    std::vector<std::string> calls{};  ///< "s<n> <call>", in the order made.
    int                      egress_result = 0;
    int                      opened = 0;

    std::unique_ptr<net::Socket> open(net::SocketDomain, net::SocketType,
                                      net::SocketProtocol) noexcept override {
        return std::make_unique<RecordingSocket>(*this, opened++);
    }

    void wait_readable(net::Socket* const*, std::size_t, rosette::platform::Event&,
                       uint32_t) noexcept override {}

private:
    static std::string dotted(const uint8_t* a) {
        return std::to_string(a[0]) + "." + std::to_string(a[1]) + "." + std::to_string(a[2]) + "."
               + std::to_string(a[3]);
    }

    class RecordingSocket final : public net::Socket {
    public:
        RecordingSocket(RecordingSocketProvider& owner, int id) : owner_(owner), id_(id) {}

        bool is_open() const noexcept override { return true; }

        int joinMulticastGroup(const net::SocketMulticastMembership& m) noexcept override {
            record("join " + dotted(m.group.data()) + " on " + dotted(m.interface.data()));
            return 0;
        }

        int leaveMulticastGroup(const net::SocketMulticastMembership&) noexcept override {
            return 0;
        }

        int set_multicast_interface(const net::SocketAddress& local) noexcept override {
            record("egress " + dotted(local.address.data()));
            return owner_.egress_result;
        }

        int bind(const net::SocketAddress& local) noexcept override {
            record("bind " + std::to_string(local.port));
            return 0;
        }

        int set_receive_timeout(uint32_t) noexcept override { return 0; }

        std::ptrdiff_t send_to(const void*, std::size_t length,
                               const net::SocketAddress&) noexcept override {
            return static_cast<std::ptrdiff_t>(length);
        }

        std::ptrdiff_t recv_from(void*, std::size_t, net::SocketAddress*) noexcept override {
            return -1;
        }

        void close() noexcept override {}

        net::SocketError last_error() const noexcept override {
            return net::SocketError::would_block;
        }

    private:
        void record(const std::string& call) {
            owner_.calls.push_back("s" + std::to_string(id_) + " " + call);
        }

        RecordingSocketProvider& owner_;
        int                      id_;
    };
};

/// Index of `call` in `calls`, or -1.
int find_call(const std::vector<std::string>& calls, const std::string& call) {
    for (size_t i = 0; i < calls.size(); ++i) {
        if (calls[i] == call) {
            return static_cast<int>(i);
        }
    }

    return -1;
}

/// Whether every socket opened was given the egress interface before it was bound.
bool every_socket_egresses_before_bind(const RecordingSocketProvider& provider) {
    for (int s = 0; s < provider.opened; ++s) {
        const std::string id = "s" + std::to_string(s) + " ";
        const int         egress = find_call(provider.calls, id + "egress 10.42.0.1");
        bool              bound_later = false;
        for (size_t i = 0; i < provider.calls.size(); ++i) {
            if (provider.calls[i].rfind(id + "bind ", 0) == 0) {
                bound_later = egress >= 0 && static_cast<int>(i) > egress;
            }
        }

        if (!bound_later) {
            return false;
        }
    }

    return true;
}

/// TC-TRN-9.6.1-001: the SPDP group is joined on the named interface, and every socket sends
/// multicast through it, set before bind.
bool tc_trn_9_6_1_001(std::string& err) {
    RecordingSocketProvider provider;
    SocketUdpTransport      transport{provider, 7410u, 7411u, 0xEFFF0001u, 7400u};
    transport.set_multicast_interface(kInterface);
    CHECK(transport.open(), "open() failed");
    CHECK(provider.opened >= 3, "expected SPDP, metatraffic and user-data sockets");
    CHECK(find_call(provider.calls, "s0 join 239.255.0.1 on 10.42.0.1") >= 0
              || find_call(provider.calls, "s1 join 239.255.0.1 on 10.42.0.1") >= 0
              || find_call(provider.calls, "s2 join 239.255.0.1 on 10.42.0.1") >= 0,
          "SPDP membership does not name the interface");
    CHECK(every_socket_egresses_before_bind(provider),
          "a socket was bound without its multicast interface set first");
    transport.close();

    RecordingSocketProvider unset;
    SocketUdpTransport      stack_choice{unset, 7410u, 7411u, 0xEFFF0001u, 7400u};
    CHECK(stack_choice.open(), "open() without an interface failed");
    for (const std::string& call : unset.calls) {
        CHECK(call.find("egress") == std::string::npos,
              "egress set although no interface was named");
        CHECK(
            call.find("join") == std::string::npos || call.find("on 0.0.0.0") != std::string::npos,
            "join named an interface although none was");
    }

    stack_choice.close();
    return true;
}

/// TC-TRN-9.6.1-002: a backend with one interface declines the egress choice and open()
/// succeeds; any other refusal fails open().
bool tc_trn_9_6_1_002(std::string& err) {
    RecordingSocketProvider single;
    single.egress_result = static_cast<int>(net::SocketError::not_supported);
    SocketUdpTransport one_interface{single, 7410u, 7411u, 0xEFFF0001u, 7400u};
    one_interface.set_multicast_interface(kInterface);
    CHECK(one_interface.open(), "not_supported must be accepted");
    one_interface.close();

    RecordingSocketProvider refusing;
    refusing.egress_result = static_cast<int>(net::SocketError::invalid_argument);
    SocketUdpTransport unheld{refusing, 7410u, 7411u, 0xEFFF0001u, 7400u};
    unheld.set_multicast_interface(kInterface);
    CHECK(!unheld.open(), "an address no interface holds must fail open()");
    return true;
}

}  // namespace

void append_transport_multicast_interface_tests(std::vector<TestCase>& tests) {
    tests.push_back({"TC-TRN-9.6.1-001",
                     "SocketUdpTransport: SPDP is joined on the named interface, and every "
                     "socket sends multicast through it, set before bind.",
                     tc_trn_9_6_1_001});
    tests.push_back({"TC-TRN-9.6.1-002",
                     "SocketUdpTransport: a single-interface backend's not_supported is "
                     "accepted; any other refusal fails open().",
                     tc_trn_9_6_1_002});
}

}  // namespace spec_test
