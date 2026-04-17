#include "net/posix/socket.hpp"
#include "ros/context.hpp"
#include "rtps/spdp.hpp"
#include "rtps/transport.hpp"
#include "std_msgs__String.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

extern "C" uint32_t rtps_platform_get_ms(void);
extern "C" uint32_t rtps_platform_get_ipv4_addr(void) noexcept;

namespace {

class PosixDemoContext final : public milkros::Context {
public:
    PosixDemoContext(rtps::ITransport& transport,
                     rtps::ITransportRuntime& runtime,
                     const milkros::ContextOptions& options = {}) noexcept
        : milkros::Context(transport, options)
        , runtime_(runtime) {}

    bool start_runtime() noexcept
    {
        return runtime_.start();
    }

    void tick(uint32_t now_ms) noexcept override
    {
        spin_transport(runtime_);
        tick_participant(now_ms);
        tick_timers(now_ms);
    }

private:
    rtps::ITransportRuntime& runtime_;
};

uint32_t parse_u32(const char* text, uint32_t fallback) noexcept
{
    if (text == nullptr) {
        return fallback;
    }

    const char* p = text;
    uint32_t value = 0u;
    while (*p != '\0') {
        if ((*p < '0') || (*p > '9')) {
            return fallback;
        }
        value = static_cast<uint32_t>(value * 10u + static_cast<uint32_t>(*p - '0'));
        ++p;
    }
    return value;
}

void usage(const char* argv0)
{
    std::printf(
        "Usage: %s [--domain N] [--participant N] [--duration-ms N] [--publish-ms N] [--topic NAME]\n"
        "Environment:\n"
        "  MILKDDS_LOCAL_IP=IPv4   Optional, e.g. 172.18.0.5\n",
        argv0);
}

}  // namespace

int main(int argc, char** argv)
{
    uint8_t domain_id = 0u;
    uint8_t participant_id = 31u;
    uint32_t duration_ms = 15000u;
    uint32_t publish_period_ms = 1000u;
    std::string topic = "chatter";

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--domain") == 0) {
            if (i + 1 >= argc) {
                usage(argv[0]);
                return 2;
            }
            domain_id = static_cast<uint8_t>(parse_u32(argv[++i], domain_id));
            continue;
        }
        if (std::strcmp(argv[i], "--participant") == 0) {
            if (i + 1 >= argc) {
                usage(argv[0]);
                return 2;
            }
            participant_id = static_cast<uint8_t>(parse_u32(argv[++i], participant_id));
            continue;
        }
        if (std::strcmp(argv[i], "--duration-ms") == 0) {
            if (i + 1 >= argc) {
                usage(argv[0]);
                return 2;
            }
            duration_ms = parse_u32(argv[++i], duration_ms);
            continue;
        }
        if (std::strcmp(argv[i], "--publish-ms") == 0) {
            if (i + 1 >= argc) {
                usage(argv[0]);
                return 2;
            }
            publish_period_ms = parse_u32(argv[++i], publish_period_ms);
            continue;
        }
        if (std::strcmp(argv[i], "--topic") == 0) {
            if (i + 1 >= argc) {
                usage(argv[0]);
                return 2;
            }
            topic = argv[++i];
            continue;
        }

        usage(argv[0]);
        return 2;
    }

    const uint32_t local_ipv4 = rtps_platform_get_ipv4_addr();
    if (local_ipv4 == 0u) {
        std::printf("[ros2-smoke] no local IPv4 detected\n");
        return 1;
    }

    milkdds::net::posix::PosixSocketProvider socket_provider;
    const uint16_t meta_port = static_cast<uint16_t>(rtps::spdp_multicast_port(domain_id));
    const uint16_t user_port = static_cast<uint16_t>(rtps::user_unicast_port(domain_id, participant_id));

    rtps::SocketUdpTransport transport(
        socket_provider,
        meta_port,
        user_port,
        rtps::spdp_multicast_locator(domain_id));
    rtps::SocketTransportRuntime runtime(transport);

    milkros::ContextOptions options{};
    options.domain_id = domain_id;
    options.participant_id = participant_id;
    PosixDemoContext context(transport, runtime, options);

    std::printf("[ros2-smoke] local_ipv4=%u.%u.%u.%u\n",
                static_cast<unsigned>((local_ipv4 >> 24u) & 0xFFu),
                static_cast<unsigned>((local_ipv4 >> 16u) & 0xFFu),
                static_cast<unsigned>((local_ipv4 >> 8u) & 0xFFu),
                static_cast<unsigned>(local_ipv4 & 0xFFu));

    if (!context.start_runtime()) {
        std::printf("[ros2-smoke] failed to start transport runtime "
                    "(check multicast availability and ROS_DOMAIN_ID collisions)\n");
        return 1;
    }

    (void)context.participant().add_ipv4_address(local_ipv4);
    if (!context.participant().has_addresses()) {
        std::printf("[ros2-smoke] failed to configure participant IPv4 address\n");
        return 1;
    }

    if (!context.start()) {
        std::printf("[ros2-smoke] failed to start context\n");
        return 1;
    }

    auto node = context.create_node("milkdds_host", "");
    auto chatter = node.create_publisher<std_msgs_String_t>(topic.c_str());
    if (!chatter.valid()) {
        std::printf("[ros2-smoke] failed to create publisher for topic '%s'\n", topic.c_str());
        return 1;
    }

    std::printf("[ros2-smoke] domain=%u participant=%u topic=/%s duration_ms=%lu publish_ms=%lu\n",
                static_cast<unsigned>(domain_id),
                static_cast<unsigned>(participant_id),
                topic.c_str(),
                static_cast<unsigned long>(duration_ms),
                static_cast<unsigned long>(publish_period_ms));

    const uint32_t start_ms = rtps_platform_get_ms();
    uint32_t last_publish_ms = 0u;
    uint32_t sequence = 0u;

    while ((rtps_platform_get_ms() - start_ms) < duration_ms) {
        uint32_t now_ms = rtps_platform_get_ms();
        context.tick(now_ms);

        if ((now_ms - last_publish_ms) >= publish_period_ms) {
            std_msgs_String_t msg{};
            std::snprintf(msg.data,
                          sizeof(msg.data),
                          "milkDDS POSIX hello seq=%lu uptime_ms=%lu",
                          static_cast<unsigned long>(sequence++),
                          static_cast<unsigned long>(now_ms));
            (void)chatter.publish(msg);
            last_publish_ms = now_ms;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::printf("[ros2-smoke] complete\n");
    return 0;
}
