// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file examples/rtps_chatter/main.cpp
 * @brief Agentless ROS 2 talker/listener, straight on RTPS, for any board with a network.
 *
 * The board joins the ROS 2 domain as an ordinary DDS participant: SPDP
 * discovery, SEDP endpoint announcements, and reliable std_msgs/String
 * exchange with any stock middleware on the LAN, with no agent. It runs the
 * Rosette RTPS core on the board's socket provider: a WIZnet chip's hardware
 * UDP sockets or FreeRTOS+TCP over the MIMXRT1020-EVK's ENET. Every datagram
 * stays within ROSETTE_RTPS_MAX_DATAGRAM_SIZE (1472 octets, an Ethernet MTU).
 *
 *   - publishes "hello from rosette #N" on /rosette/chatter every second;
 *   - prints every string received on /rosette/command;
 *   - prints discovery and datagram counters every five seconds, including
 *     sends abandoned because ARP for the destination failed, followed by the
 *     network buffers' and the platform's memory use.
 *
 * From a PC in domain 0 with stock Fast DDS or Cyclone DDS (no configuration):
 *   ros2 topic echo /rosette/chatter std_msgs/msg/String
 *   ros2 topic pub /rosette/command std_msgs/msg/String "{data: hi}"
 *
 * Verification: the PC echoes the board's messages, the board prints the
 * PC's, and the counters show max_tx <= 1472 and oversize_tx = 0.
 *
 * References: OMG DDSI-RTPS 2.5 §8.5 (discovery), §9.6.1.1 (port mapping);
 * ROS 2 topic name mangling (rt/ prefix, <pkg>::msg::dds_::<Type>_ names).
 */

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "board.h"
#include "rosette/net/board_network.hpp"
#include "rosette/platform/memory_report.hpp"
#include "rosette/platform/platform.hpp"
#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/participant_driver.hpp"
#include "rosette/rtps/transport.hpp"

namespace rtps = rosette::rtps;

namespace {

namespace net = rosette::net;
namespace pf = rosette::platform;

constexpr uint8_t     kDomainId = 0u;
constexpr uint8_t     kParticipantId = 0u;
constexpr uint32_t kSpdpGroup = 0xEFFF0001u;  // 239.255.0.1
constexpr uint32_t kPublishPeriodMs = 1000u;
constexpr uint32_t kReportPeriodMs = 5000u;
constexpr uint32_t    kRetryPeriodMs = 2000u;
constexpr char        kStringType[] = "std_msgs::msg::dds_::String_";
/// Depth of the chatter writer's history; matches ROSETTE_RTPS_MAX_WRITER_DEPTH in CMakeLists.txt.
constexpr uint8_t     kChatterDepth = 4u;
/// Room for the memory report's two lines: the main task, the IP stack's, and the kernel's.
constexpr std::size_t kMemoryTextBytes = 384u;

/// @brief DDSI-RTPS §9.6.1.1 default ports: PB=7400, DG=250, PG=2, d0/d1/d3=0/10/11.
constexpr uint16_t spdp_port(uint8_t domain) {
    return static_cast<uint16_t>(7400u + 250u * domain);
}

constexpr uint16_t meta_port(uint8_t domain, uint8_t id) {
    return static_cast<uint16_t>(spdp_port(domain) + 10u + 2u * id);
}

constexpr uint16_t user_port(uint8_t domain, uint8_t id) {
    return static_cast<uint16_t>(spdp_port(domain) + 11u + 2u * id);
}

alignas(8) rtps::Octet g_rx_buffer[ROSETTE_RTPS_MAX_DATAGRAM_SIZE];
uint32_t g_commands_received = 0u;

/// The participant's two user endpoints.
struct ChatterEndpoints {
    rtps::StatefulWriter* chatter = nullptr;
    rtps::StatefulReader* command = nullptr;
};

/**
 * @brief Encodes a std_msgs/String body in XCDR1 little-endian.
 *
 * The writer prepends the 4-octet CDR_LE encapsulation header; this writes
 * the body only: uint32 length (including the NUL), the characters, the NUL.
 * Every Rosette target (Cortex-M0+, M33, M7, and the host) is little-endian,
 * so the length is copied as it sits in memory.
 *
 * @param[in]  text Characters to send.
 * @param[out] out  Buffer receiving the body.
 * @param[in]  cap  Capacity of `out`.
 * @return Body length in octets, or 0 when it does not fit.
 */
uint32_t encode_string(const char* text, rtps::Octet* out, uint32_t cap) noexcept {
    const uint32_t len = static_cast<uint32_t>(std::strlen(text)) + 1u;
    if (4u + len > cap) {
        return 0u;
    }

    std::memcpy(out, &len, 4u);
    std::memcpy(out + 4u, text, len);
    return 4u + len;
}

/**
 * @brief Prints a received std_msgs/String (reader callback, driver lock held).
 *
 * The payload starts with the encapsulation header, then the XCDR1 string.
 * A length that overruns the payload is reported, not trusted.
 */
void on_command(const rtps::CacheChange& change, void*) noexcept {
    const rtps::Octet* p = change.payload.data;
    const uint32_t     n = change.payload.size;
    if (p == nullptr || n < 8u) {
        return;
    }

    uint32_t len = 0u;
    std::memcpy(&len, p + 4u, 4u);
    if (len == 0u || 8u + len > n) {
        std::printf("[chatter] malformed String (%lu octets)\n", static_cast<unsigned long>(n));
        return;
    }

    ++g_commands_received;
    std::printf("[chatter] /rosette/command: %.*s\n", static_cast<int>(len - 1u),
                reinterpret_cast<const char*>(p + 8u));
}

/// @brief Fills a writer or reader config for a std_msgs/String topic.
template<typename Config>
Config string_topic(const char* ros_topic) noexcept {
    Config cfg{};
    cfg.topicName.assign(ros_topic);
    cfg.typeName.assign(kStringType);
    cfg.reliability = rtps::ReliabilityKind::RELIABLE;
    cfg.durability = rtps::DurabilityKind::VOLATILE;
    return cfg;
}

/**
 * @brief Prints the discovery and datagram counters, then the memory in use.
 *
 * Runs with the driver lock held, so the counters and match counts are one
 * consistent snapshot. The memory lines are what step 4 of
 * docs/design/imxrt_memory_budget.md sizes the main task's stack and the
 * heaps from: each reservation's peak against its size.
 */
void report(const rtps::SocketUdpTransport& transport, const ChatterEndpoints& endpoints) noexcept {
    const rtps::TransportStats& st = transport.stats();
    std::printf(
        "[chatter] matched readers=%u writers=%u commands=%lu | limit=%lu tx=%lu rx=%lu "
        "max_tx=%lu max_rx=%lu oversize_tx=%lu oversize_rx=%lu truncated_rx=%lu "
        "arp_timeouts=%lu\n",
        endpoints.chatter->matched_reader_count(), endpoints.command->matched_writer_count(),
        static_cast<unsigned long>(g_commands_received),
        static_cast<unsigned long>(transport.max_datagram_size()),
        static_cast<unsigned long>(st.tx_datagrams), static_cast<unsigned long>(st.rx_datagrams),
        static_cast<unsigned long>(st.max_tx_datagram),
        static_cast<unsigned long>(st.max_rx_datagram), static_cast<unsigned long>(st.oversize_tx),
        static_cast<unsigned long>(st.oversize_rx), static_cast<unsigned long>(st.truncated_rx),
        static_cast<unsigned long>(net::board_send_timeouts()));

    const net::NetworkBufferUse buffers = net::board_network_buffer_use();
    if (buffers.count > 0u) {
        std::printf("[chatter] netbuf=%lu/%lu\n", static_cast<unsigned long>(buffers.peak),
                    static_cast<unsigned long>(buffers.count));
    }

    char text[kMemoryTextBytes];
    if (pf::format_memory_report(text, sizeof(text)) > 0u) {
        std::printf("[chatter] %s", text);
    }
}

/**
 * @brief Opens the RTPS ports on the board's sockets, or reports why not for ever.
 *
 * The transport binds the metatraffic and user unicast ports and the SPDP
 * multicast port, joining 239.255.0.1 on it. Failure means a port is taken
 * or the provider has no sockets left (a WIZnet chip has eight); nothing
 * further can run, so the reason repeats on the console.
 */
void open_transport(rtps::SocketUdpTransport& transport) noexcept {
    transport.set_max_datagram_size(ROSETTE_RTPS_MAX_DATAGRAM_SIZE);
    if (transport.open()) {
        return;
    }

    for (;;) {
        std::printf(
            "[chatter] transport open failed: ports %u/%u/%u (taken, or no sockets left?)\n",
            meta_port(kDomainId, kParticipantId), user_port(kDomainId, kParticipantId),
            spdp_port(kDomainId));
        pf::sleep_ms(kRetryPeriodMs);
    }
}

/**
 * @brief Creates /rosette/chatter's writer and /rosette/command's reader, or reports failure for
 * ever.
 *
 * Both endpoints fit the participant CMakeLists.txt sizes for them (one
 * writer of depth kChatterDepth, one reader, 128 octets of names); a failure
 * here means that sizing and this code disagree.
 */
ChatterEndpoints create_endpoints(rtps::RTPSParticipant&   participant,
                                  rtps::ParticipantDriver& driver) noexcept {
    ChatterEndpoints endpoints;
    {
        pf::LockGuard guard(driver.mutex());
        endpoints.chatter = participant.create_writer<kChatterDepth>(
            string_topic<rtps::WriterConfig>("rt/rosette/chatter"));
        endpoints.command = participant.create_reader(
            string_topic<rtps::ReaderConfig>("rt/rosette/command"), on_command, nullptr);
    }

    while (endpoints.chatter == nullptr || endpoints.command == nullptr) {
        std::printf("[chatter] endpoint creation failed\n");
        pf::sleep_ms(kRetryPeriodMs);
    }

    return endpoints;
}

/// @brief Publishes "hello from rosette #count" on /rosette/chatter.
void publish(rtps::ParticipantDriver& driver, rtps::StatefulWriter& writer,
             uint32_t count) noexcept {
    char text[48];
    std::snprintf(text, sizeof(text), "hello from rosette #%lu", static_cast<unsigned long>(count));
    rtps::Octet    body[64];
    const uint32_t n = encode_string(text, body, sizeof(body));
    pf::LockGuard  guard(driver.mutex());
    (void)writer.write_async(rtps::ChangeKind::ALIVE, body, n);
}

/**
 * @brief The application: bring the network up, join the domain, then publish and report for ever.
 *
 * Runs where the OS needs it (platform::run_main_task()): on a bare-metal
 * board straight from main(), under FreeRTOS as a task once the scheduler
 * runs, since network bring-up sleeps while the IP task works. The driver
 * spins the protocol between publications; the DHCP lease is renewed from
 * the same loop.
 */
void run_chatter() {
    pf::sleep_ms(1500u);  // give a USB console time to enumerate
    std::printf("\n[chatter] rosette RTPS chatter, datagram limit %u\n",
                static_cast<unsigned>(ROSETTE_RTPS_MAX_DATAGRAM_SIZE));

    (void)net::bring_up_board_network("rosette-chatter");
    net::SocketProvider& sockets = net::board_socket_provider();
    const uint32_t       addr = net::board_ipv4_address();

    static rtps::SocketUdpTransport transport(sockets, meta_port(kDomainId, kParticipantId),
                                              user_port(kDomainId, kParticipantId), kSpdpGroup,
                                              spdp_port(kDomainId));
    open_transport(transport);
    static rtps::RTPSParticipant participant(kDomainId, kParticipantId, transport);
    participant.add_ipv4_address(addr, net::board_address_snapshot().v4.primary().prefix_len);
    static rtps::ParticipantDriver driver(participant, transport, sockets, g_rx_buffer,
                                          sizeof(g_rx_buffer));
    const ChatterEndpoints         endpoints = create_endpoints(participant, driver);
    std::printf("[chatter] participant up: SPDP %u.%u.%u.%u -> 239.255.0.1:%u, meta %u, user %u\n",
                static_cast<unsigned>(addr >> 24), static_cast<unsigned>((addr >> 16) & 0xFFu),
                static_cast<unsigned>((addr >> 8) & 0xFFu), static_cast<unsigned>(addr & 0xFFu),
                spdp_port(kDomainId), meta_port(kDomainId, kParticipantId),
                user_port(kDomainId, kParticipantId));

    uint32_t count = 0u;
    uint32_t next_publish = pf::monotonic_ms();
    uint32_t next_report = next_publish + kReportPeriodMs;
    for (;;) {
        const uint32_t now = pf::monotonic_ms();
        const uint32_t wait = static_cast<int32_t>(next_publish - now) > 0 ? next_publish - now : 0u;
        (void)driver.spin_until([] { return false; }, wait);

        const uint32_t t = pf::monotonic_ms();
        net::poll_board_network(t);
        if (static_cast<int32_t>(t - next_publish) >= 0) {
            next_publish += kPublishPeriodMs;
            publish(driver, *endpoints.chatter, ++count);
        }

        if (static_cast<int32_t>(t - next_report) >= 0) {
            next_report += kReportPeriodMs;
            pf::LockGuard guard(driver.mutex());
            report(transport, endpoints);
        }
    }
}

}  // namespace

int main() {
    board_init();
    pf::run_main_task(&run_chatter);
    return 0;
}
