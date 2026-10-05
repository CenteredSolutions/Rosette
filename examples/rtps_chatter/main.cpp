/**
 * @file examples/rtps_chatter/main.cpp
 * @brief Agentless ROS 2 talker/listener on a WIZnet board, straight on RTPS.
 *
 * The board joins the ROS 2 domain as an ordinary DDS participant: SPDP
 * discovery, SEDP endpoint announcements, and reliable std_msgs/String
 * exchange with any stock middleware on the LAN, with no agent. It runs the
 * Rosette RTPS core on the WIZnet chip's hardware UDP sockets, so every
 * datagram stays within 1472 octets (ROSETTE_RTPS_MAX_DATAGRAM_SIZE).
 *
 *   - publishes "hello from rosette #N" on /rosette/chatter every second;
 *   - prints every string received on /rosette/command;
 *   - prints discovery and datagram counters every five seconds, including
 *     sends the chip abandoned because ARP for the destination failed.
 *
 * From a PC in domain 0 with stock Fast DDS (no configuration):
 *   ros2 topic echo /rosette/chatter std_msgs/msg/String
 *   ros2 topic pub /rosette/command std_msgs/msg/String "{data: hi}"
 *
 * Verification: the PC echoes the board's messages, the board prints the
 * PC's, and the counters show max_tx <= 1472 and oversize_tx = 0.
 *
 * References: OMG DDSI-RTPS 2.5 §8.5 (discovery), §9.6.1.1 (port mapping);
 * ROS 2 topic name mangling (rt/ prefix, <pkg>::msg::dds_::<Type>_ names).
 */

#include <cstdio>
#include <cstring>

#include "board.h"
#include "rosette/net/board_network.hpp"
#include "rosette/platform/platform.hpp"
#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/participant_driver.hpp"
#include "rosette/rtps/transport.hpp"

#include "pico/stdlib.h"

namespace {

namespace net = rosette::net;
namespace pf = rosette::platform;

constexpr uint8_t kDomainId = 0u;
constexpr uint8_t kParticipantId = 0u;
constexpr uint32_t kSpdpGroup = 0xEFFF0001u;  // 239.255.0.1
constexpr uint32_t kPublishPeriodMs = 1000u;
constexpr uint32_t kReportPeriodMs = 5000u;
constexpr char kStringType[] = "std_msgs::msg::dds_::String_";

/// @brief DDSI-RTPS §9.6.1.1 default ports: PB=7400, DG=250, PG=2, d0/d1/d3=0/10/11.
constexpr uint16_t spdp_port(uint8_t domain) { return static_cast<uint16_t>(7400u + 250u * domain); }
constexpr uint16_t meta_port(uint8_t domain, uint8_t id) {
    return static_cast<uint16_t>(spdp_port(domain) + 10u + 2u * id);
}
constexpr uint16_t user_port(uint8_t domain, uint8_t id) {
    return static_cast<uint16_t>(spdp_port(domain) + 11u + 2u * id);
}

alignas(8) rtps::Octet g_rx_buffer[ROSETTE_RTPS_MAX_DATAGRAM_SIZE];
uint32_t g_commands_received = 0u;

/**
 * @brief Encodes a std_msgs/String body in XCDR1 little-endian.
 *
 * The writer prepends the 4-octet CDR_LE encapsulation header; this writes
 * the body only: uint32 length (including the NUL), the characters, the NUL.
 *
 * @param[in]  text Characters to send.
 * @param[out] out  Buffer receiving the body.
 * @param[in]  cap  Capacity of `out`.
 * @return Body length in octets, or 0 when it does not fit.
 */
uint32_t encode_string(const char* text, rtps::Octet* out, uint32_t cap) noexcept {
    const uint32_t len = static_cast<uint32_t>(std::strlen(text)) + 1u;
    if (4u + len > cap) return 0u;
    std::memcpy(out, &len, 4u);  // RP2 is little-endian, as is CDR_LE
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
    const uint32_t n = change.payload.size;
    if (p == nullptr || n < 8u) return;
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

void report(const rtps::SocketUdpTransport& transport, rtps::StatefulWriter& writer,
            rtps::StatefulReader& reader, uint32_t send_timeouts) noexcept {
    const rtps::TransportStats& st = transport.stats();
    std::printf("[chatter] matched readers=%u writers=%u commands=%lu | limit=%lu tx=%lu rx=%lu "
                "max_tx=%lu max_rx=%lu oversize_tx=%lu oversize_rx=%lu truncated_rx=%lu arp_timeouts=%lu\n",
                writer.matched_reader_count(), reader.matched_writer_count(),
                static_cast<unsigned long>(g_commands_received),
                static_cast<unsigned long>(transport.max_datagram_size()),
                static_cast<unsigned long>(st.tx_datagrams), static_cast<unsigned long>(st.rx_datagrams),
                static_cast<unsigned long>(st.max_tx_datagram), static_cast<unsigned long>(st.max_rx_datagram),
                static_cast<unsigned long>(st.oversize_tx), static_cast<unsigned long>(st.oversize_rx),
                static_cast<unsigned long>(st.truncated_rx), static_cast<unsigned long>(send_timeouts));
}

}  // namespace

int main() {
    board_init();
    // Hold the boot log until a terminal attaches (up to 10 s).
    for (int i = 0; i < 100 && !stdio_usb_connected(); ++i) sleep_ms(100);
    std::printf("[chatter] rosette RTPS chatter, datagram limit %u\n",
                static_cast<unsigned>(ROSETTE_RTPS_MAX_DATAGRAM_SIZE));

    net::bring_up_board_network("rosette-chatter");
    net::SocketProvider& sockets = net::board_socket_provider();
    const uint32_t addr = net::board_ipv4_address();

    static rtps::SocketUdpTransport transport(sockets, meta_port(kDomainId, kParticipantId),
                                              user_port(kDomainId, kParticipantId), kSpdpGroup,
                                              spdp_port(kDomainId));
    transport.set_max_datagram_size(ROSETTE_RTPS_MAX_DATAGRAM_SIZE);
    if (!transport.open()) {
        for (;;) {
            std::printf("[chatter] transport open failed: ports %u/%u/%u (out of hardware sockets?)\n",
                        meta_port(kDomainId, kParticipantId), user_port(kDomainId, kParticipantId),
                        spdp_port(kDomainId));
            sleep_ms(2000);
        }
    }
    static rtps::RTPSParticipant participant(kDomainId, kParticipantId, transport);
    participant.add_ipv4_address(addr, net::board_address_snapshot().v4.primary().prefix_len);
    static rtps::ParticipantDriver driver(participant, transport, sockets, g_rx_buffer,
                                          sizeof(g_rx_buffer));

    rtps::StatefulWriter* writer = nullptr;
    rtps::StatefulReader* reader = nullptr;
    {
        pf::LockGuard guard(driver.mutex());
        writer = participant.create_writer<4>(string_topic<rtps::WriterConfig>("rt/rosette/chatter"));
        reader = participant.create_reader(string_topic<rtps::ReaderConfig>("rt/rosette/command"),
                                           on_command, nullptr);
    }
    std::printf("[chatter] participant up: SPDP %u.%u.%u.%u -> 239.255.0.1:%u, meta %u, user %u\n",
                static_cast<unsigned>(addr >> 24), static_cast<unsigned>((addr >> 16) & 0xFFu),
                static_cast<unsigned>((addr >> 8) & 0xFFu), static_cast<unsigned>(addr & 0xFFu),
                spdp_port(kDomainId),
                meta_port(kDomainId, kParticipantId), user_port(kDomainId, kParticipantId));
    if (writer == nullptr || reader == nullptr) {
        for (;;) {
            std::printf("[chatter] endpoint creation failed\n");
            sleep_ms(2000);
        }
    }

    uint32_t count = 0u;
    uint32_t next_publish = pf::monotonic_ms();
    uint32_t next_report = next_publish + kReportPeriodMs;
    for (;;) {
        const uint32_t now = pf::monotonic_ms();
        const uint32_t wait = static_cast<int32_t>(next_publish - now) > 0 ? next_publish - now : 0u;
        (void)driver.spin_until([] { return false; }, wait);

        const uint32_t t = pf::monotonic_ms();
        net::poll_board_network(t);  // DHCP lease renewal
        if (static_cast<int32_t>(t - next_publish) >= 0) {
            next_publish += kPublishPeriodMs;
            char text[48];
            std::snprintf(text, sizeof(text), "hello from rosette #%lu",
                          static_cast<unsigned long>(++count));
            rtps::Octet body[64];
            const uint32_t n = encode_string(text, body, sizeof(body));
            pf::LockGuard guard(driver.mutex());
            (void)writer->write_async(rtps::ChangeKind::ALIVE, body, n);
        }
        if (static_cast<int32_t>(t - next_report) >= 0) {
            next_report += kReportPeriodMs;
            pf::LockGuard guard(driver.mutex());
            report(transport, *writer, *reader, net::board_send_timeouts());
        }
    }
}
