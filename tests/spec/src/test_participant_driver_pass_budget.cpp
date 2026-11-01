/**
 * @file test_participant_driver_pass_budget.cpp
 * @brief TC-DRV-PASS-001..003: a ParticipantDriver pass is bounded in time and fair between
 * sockets.
 *
 * ParticipantDriver::spin_until() runs protocol passes under the participant
 * lock. A pass takes datagrams alternately from the metatraffic and user-data
 * sockets and stops once its budget (RTPS_PUMP_PASS_BUDGET_US) has run out;
 * the next pass follows a short yield (RTPS_PUMP_BACKLOG_YIELD_MS) rather
 * than a wait for data, while datagrams wait. These cases
 * run a real SocketUdpTransport and RTPSParticipant over a socket backend
 * whose sockets hold queued datagrams, on the test clock: each datagram
 * taken can advance it, so the budget's cut-off is exact.
 */

#include "participant_test_support.hpp"

#include "rosette/rtps/participant_driver.hpp"

#include <chrono>
#include <deque>
#include <map>

namespace spec_test {

extern uint32_t g_participant_test_now_ms;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

namespace net = rosette::net;

constexpr uint16_t kMetaPort = 7410u;
constexpr uint16_t kUserPort = 7411u;
constexpr uint16_t kSpdpPort = 7400u;

/// A socket backend whose sockets hand out datagrams queued per bound port.
class QueuedSocketProvider final : public net::SocketProvider {
public:
    std::map<uint16_t, std::deque<std::vector<Octet>>> queued{};
    std::vector<uint16_t> taken{};               ///< Port of each datagram taken, in order.
    uint32_t              ms_per_datagram = 0u;  ///< Test clock advance per datagram taken.
    int                   sleeps = 0;            ///< wait_readable() calls.

    void queue(uint16_t port, int count) {
        for (int i = 0; i < count; ++i) {
            queued[port].push_back(std::vector<Octet>(24u, 0x00u));
        }
    }

    size_t waiting() const {
        size_t n = 0u;
        for (const auto& [port, q] : queued) {
            n += q.size();
        }

        return n;
    }

    std::unique_ptr<net::Socket> open(net::SocketDomain, net::SocketType,
                                      net::SocketProtocol) noexcept override {
        return std::make_unique<QueuedSocket>(*this);
    }

    void wait_readable(net::Socket* const*, std::size_t, rosette::platform::Event&,
                       uint32_t) noexcept override {
        ++sleeps;
    }

private:
    class QueuedSocket final : public net::Socket {
    public:
        explicit QueuedSocket(QueuedSocketProvider& owner) : owner_(owner) {}

        bool is_open() const noexcept override { return true; }

        int joinMulticastGroup(const net::SocketMulticastMembership&) noexcept override {
            return 0;
        }

        int leaveMulticastGroup(const net::SocketMulticastMembership&) noexcept override {
            return 0;
        }

        int bind(const net::SocketAddress& local) noexcept override {
            port_ = local.port;
            return 0;
        }

        int set_receive_timeout(uint32_t) noexcept override { return 0; }

        std::ptrdiff_t send_to(const void*, std::size_t length,
                               const net::SocketAddress&) noexcept override {
            return static_cast<std::ptrdiff_t>(length);
        }

        std::ptrdiff_t recv_from(void* data, std::size_t capacity,
                                 net::SocketAddress*) noexcept override {
            auto& q = owner_.queued[port_];
            if (q.empty()) {
                return -1;
            }

            const std::size_t n = std::min(capacity, q.front().size());
            std::memcpy(data, q.front().data(), n);
            q.pop_front();
            owner_.taken.push_back(port_);
            g_participant_test_now_ms += owner_.ms_per_datagram;
            return static_cast<std::ptrdiff_t>(n);
        }

        void close() noexcept override {}

        net::SocketError last_error() const noexcept override {
            return net::SocketError::would_block;
        }

    private:
        QueuedSocketProvider& owner_;
        uint16_t              port_ = 0u;
    };
};

/// A participant on queued sockets, driven with a pass budget of `budget_us`.
struct DrivenParticipant {
    QueuedSocketProvider provider{};
    SocketUdpTransport   transport{provider, kMetaPort, kUserPort, 0xEFFF0001u, kSpdpPort};
    RTPSParticipant      participant{0u, 0u, transport};
    Octet                rx[1500]{};
    ParticipantDriver    driver;

    explicit DrivenParticipant(uint32_t budget_us)
        : driver(participant, transport, provider, rx, sizeof(rx), budget_us) {}
};

/// TC-DRV-PASS-001: with a budget of 0 each pass takes one datagram, and
/// the passes alternate between the metatraffic and user-data sockets.
bool tc_drv_pass_001(std::string& err) {
    g_participant_test_now_ms = 100u;
    DrivenParticipant d{0u};
    CHECK(d.transport.open(), "transport opens on the queued backend");
    d.provider.queue(kMetaPort, 3);
    d.provider.queue(kUserPort, 3);

    for (size_t pass = 1u; pass <= 6u; ++pass) {
        (void)d.driver.spin_until([] { return true; }, 0u);
        CHECK(d.provider.taken.size() == pass, "each pass takes exactly one datagram");
    }

    const std::vector<uint16_t> order{kMetaPort, kUserPort, kMetaPort,
                                      kUserPort, kMetaPort, kUserPort};
    CHECK(d.provider.taken == order, "passes alternate metatraffic and user data");
    return true;
}

/// TC-DRV-PASS-002: a pass stops taking datagrams once its budget has run
/// out, and spin_until() runs the next pass after the backlog yield, without
/// waiting for data, while datagrams wait; each such pass is counted.
///
/// The participant runs on the test clock, but the yield is a real sleep:
/// the gap between backlogged passes is measured on the steady clock and
/// must be at least the yield. sleep_ms() never returns early, so a slow
/// host cannot fail the check; without the yield the gap is microseconds.
bool tc_drv_pass_002(std::string& err) {
    static_assert(RTPS_PUMP_BACKLOG_YIELD_MS > 0u, "the yield under test is configured");
    using SteadyClock = std::chrono::steady_clock;

    g_participant_test_now_ms = 100u;
    DrivenParticipant d{10000u};
    CHECK(d.transport.open(), "transport opens on the queued backend");
    d.provider.ms_per_datagram = 3u;
    d.provider.queue(kMetaPort, 10);

    std::vector<size_t>                  per_pass{};
    std::vector<SteadyClock::time_point> ended{};
    size_t                               before = 0u;
    const bool                           drained = d.driver.spin_until(
        [&] {
            per_pass.push_back(d.provider.taken.size() - before);
            before = d.provider.taken.size();
            ended.push_back(SteadyClock::now());
            return d.provider.waiting() == 0u;
        },
        rosette::platform::kWaitForever);
    CHECK(drained, "spin_until returns once every datagram is taken");
    CHECK((per_pass == std::vector<size_t>{4u, 4u, 2u}),
          "3 ms datagrams under a 10 ms budget: 4, 4, then the last 2");
    CHECK(d.provider.sleeps == 0, "no wait for data while datagrams wait");
    CHECK(d.driver.backlogged_passes() == 2u, "the two passes the budget cut short are counted");

    const auto yield = std::chrono::milliseconds(RTPS_PUMP_BACKLOG_YIELD_MS);
    for (size_t pass = 1u; pass < ended.size(); ++pass) {
        CHECK(ended[pass] - ended[pass - 1u] >= yield,
              "each backlogged pass is followed by a sleep of at least the backlog yield");
    }

    return true;
}

/// TC-DRV-PASS-003: within its budget a pass takes everything waiting, and
/// a pass that emptied the sockets is followed by a sleep.
bool tc_drv_pass_003(std::string& err) {
    g_participant_test_now_ms = 100u;
    DrivenParticipant d{10000u};
    CHECK(d.transport.open(), "transport opens on the queued backend");
    d.provider.queue(kMetaPort, 5);
    d.provider.queue(kSpdpPort, 2);
    d.provider.queue(kUserPort, 5);

    int    passes = 0;
    size_t first_pass = 0u;
    (void)d.driver.spin_until(
        [&] {
            if (++passes == 1) {
                first_pass = d.provider.taken.size();
            }

            return passes == 2;
        },
        rosette::platform::kWaitForever);
    CHECK(first_pass == 12u, "one pass takes all 12 datagrams");
    CHECK(d.provider.sleeps == 1, "an emptied pass sleeps before the next");
    CHECK(d.driver.backlogged_passes() == 0u, "a pass that emptied the sockets is not backlogged");
    return true;
}

}  // namespace

void append_participant_driver_pass_budget_tests(std::vector<TestCase>& tests) {
    tests.push_back({"TC-DRV-PASS-001",
                     "ParticipantDriver: with a zero budget each pass takes one datagram, "
                     "alternating metatraffic and user data.",
                     tc_drv_pass_001});
    tests.push_back({"TC-DRV-PASS-002",
                     "ParticipantDriver: a pass stops once its budget runs out, and the next "
                     "follows after the backlog yield while datagrams wait; each is counted.",
                     tc_drv_pass_002});
    tests.push_back({"TC-DRV-PASS-003",
                     "ParticipantDriver: within its budget one pass takes every waiting "
                     "datagram, then the driver sleeps.",
                     tc_drv_pass_003});
}

}  // namespace spec_test
