#include "board_config.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IPv6.h"
#include "FreeRTOS_ND.h"
#include "FreeRTOS_Routing.h"
}

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "rosette/ros/ros.hpp"
#include "rosette/ros/cdr.h"
#include "rosette/net/freertos/socket.hpp"
#include "wiznet/macraw.h"

#include <array>
#include <cstdio>
#include <cstring>

extern "C" void board_init(void);
extern "C" void board_get_mac_address(uint8_t mac[6]);

namespace {

constexpr uint8_t kDomainId = 0u;
constexpr uint8_t kParticipantId = 0u;
constexpr uint32_t kTickPeriodMs = 10u;
constexpr uint32_t kPublishPeriodMs = 1000u;
constexpr uint32_t kNetworkPollDelayMs = 250u;
constexpr uint32_t kBlinkPeriodMs = 500u;

#ifndef RTPS_NET_MODE_STATIC
#define RTPS_NET_MODE_STATIC 0
#endif
#ifndef RTPS_NET_MODE_DHCP
#define RTPS_NET_MODE_DHCP 1
#endif
#ifndef RTPS_NET_MODE_SLAAC
#define RTPS_NET_MODE_SLAAC 2
#endif
#ifndef RTPS_NET_MODE_ZEROCONF
#define RTPS_NET_MODE_ZEROCONF 3
#endif
#ifndef RTPS_NET_MODE
#define RTPS_NET_MODE RTPS_NET_MODE_DHCP
#endif
#ifndef RTPS_NET_HOSTNAME
#define RTPS_NET_HOSTNAME "rosette"
#endif
#ifndef RTPS_NET_STATIC_IP
#define RTPS_NET_STATIC_IP {192u, 168u, 1u, 200u}
#endif
#ifndef RTPS_NET_STATIC_MASK
#define RTPS_NET_STATIC_MASK {255u, 255u, 255u, 0u}
#endif
#ifndef RTPS_NET_STATIC_GW
#define RTPS_NET_STATIC_GW {192u, 168u, 1u, 1u}
#endif
#ifndef RTPS_NET_STATIC_DNS
#define RTPS_NET_STATIC_DNS {8u, 8u, 8u, 8u}
#endif
#ifndef RTPS_NET_ZEROCONF_IP
#define RTPS_NET_ZEROCONF_IP {169u, 254u, 1u, 10u}
#endif

struct DemoStringMsg {
    char data[96];
};

class DemoContext : public milkros::Context {
public:
    DemoContext(rtps::ITransport& transport,
                rtps::ITransportRuntime& runtime) noexcept
        : milkros::Context(transport,
                           milkros::ContextOptions{
                               kDomainId,
                               kParticipantId,
                               1536u,
                           })
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

NetworkInterface_t g_network_interface{};
NetworkEndPoint_t g_ipv4_endpoint{};
NetworkEndPoint_t g_ipv6_endpoint{};
bool g_network_started = false;
uint32_t g_random_state = 0x5A17BEEFu;

constexpr std::array<uint8_t, 4> kStaticIp = RTPS_NET_STATIC_IP;
constexpr std::array<uint8_t, 4> kStaticMask = RTPS_NET_STATIC_MASK;
constexpr std::array<uint8_t, 4> kStaticGateway = RTPS_NET_STATIC_GW;
constexpr std::array<uint8_t, 4> kStaticDns = RTPS_NET_STATIC_DNS;
constexpr std::array<uint8_t, 4> kZeroconfIp = RTPS_NET_ZEROCONF_IP;
constexpr std::array<uint8_t, 16> kLinkLocalPrefix{
    0xFEu, 0x80u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
};

uint32_t mix_random(uint32_t value) noexcept
{
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    return value;
}

bool have_ipv4() noexcept
{
    return rtps::rtps_platform_get_ipv4_addr() != 0u;
}

bool have_ipv6(std::array<uint8_t, 16>& out) noexcept
{
    return rtps::rtps_platform_get_ipv6_addr(out.data());
}

bool network_ready() noexcept
{
    std::array<uint8_t, 16> ipv6{};
    return have_ipv4() || have_ipv6(ipv6);
}

bool start_board_network() noexcept
{
    if (g_network_started) {
        return true;
    }

    uint8_t mac[6]{};
    board_get_mac_address(mac);
    wiznet_macraw_set_mac_address(mac);

    const std::array<uint8_t, 4> ipv4 =
        (RTPS_NET_MODE == RTPS_NET_MODE_ZEROCONF) ? kZeroconfIp : kStaticIp;

    FreeRTOS_FillEndPoint(
        &g_network_interface,
        &g_ipv4_endpoint,
        ipv4.data(),
        kStaticMask.data(),
        kStaticGateway.data(),
        kStaticDns.data(),
        mac);

#if ( ipconfigUSE_DHCP != 0 )
    g_ipv4_endpoint.bits.bWantDHCP =
        (RTPS_NET_MODE == RTPS_NET_MODE_DHCP) ? pdTRUE_UNSIGNED : pdFALSE_UNSIGNED;
#endif

    IPv6_Address_t ipv6_addr{};
    IPv6_Address_t ipv6_prefix{};
    IPv6_Address_t ipv6_gateway{};
    std::memcpy(ipv6_prefix.ucBytes, kLinkLocalPrefix.data(), kLinkLocalPrefix.size());
    if (FreeRTOS_CreateIPv6Address(&ipv6_addr, &ipv6_prefix, 64u, pdTRUE) != pdPASS) {
        std::printf("FATAL: failed to create IPv6 link-local address\n");
        return false;
    }

    FreeRTOS_FillEndPoint_IPv6(
        &g_network_interface,
        &g_ipv6_endpoint,
        &ipv6_addr,
        &ipv6_prefix,
        64u,
        &ipv6_gateway,
        nullptr,
        mac);

#if ( ipconfigUSE_RA != 0 )
    g_ipv6_endpoint.bits.bWantRA =
        (RTPS_NET_MODE == RTPS_NET_MODE_SLAAC) ? pdTRUE_UNSIGNED : pdFALSE_UNSIGNED;
#endif

    if (FreeRTOS_IPInit_Multi() != pdPASS) {
        std::printf("FATAL: FreeRTOS_IPInit_Multi failed\n");
        return false;
    }

    g_network_started = true;
    return true;
}

void update_participant_addresses(rtps::RTPSParticipant& participant) noexcept
{
    const uint32_t ipv4 = rtps::rtps_platform_get_ipv4_addr();
    if (ipv4 != 0u) {
        (void)participant.add_ipv4_address(ipv4);
    }

    std::array<uint8_t, 16> ipv6{};
    if (rtps::rtps_platform_get_ipv6_addr(ipv6.data())) {
        (void)participant.add_ipv6_address(ipv6.data());
    }
}

void service_task(void*)
{
    if (!start_board_network()) {
        vTaskDelete(nullptr);
        return;
    }

    while (!network_ready()) {
        vTaskDelay(pdMS_TO_TICKS(kNetworkPollDelayMs));
    }

    static rosette::net::freertos::FreeRtosSocketProvider socket_provider;
    const uint16_t meta_port = static_cast<uint16_t>(rtps::spdp_multicast_port(kDomainId));
    const uint16_t user_port = static_cast<uint16_t>(rtps::spdp_unicast_port(kDomainId, kParticipantId));
    static rtps::SocketUdpTransport transport(
        socket_provider,
        meta_port,
        user_port,
        rtps::spdp_multicast_locator(kDomainId));
    static rtps::SocketTransportRuntime runtime(transport);
    static DemoContext context(transport, runtime);

    if (!context.start_runtime()) {
        std::printf("FATAL: transport start failed\n");
        vTaskDelete(nullptr);
        return;
    }

    update_participant_addresses(context.participant());

    if (!context.start()) {
        std::printf("FATAL: participant start failed\n");
        vTaskDelete(nullptr);
        return;
    }

    auto node = context.create_node("rosette_device", "");
    auto chatter = node.create_publisher<DemoStringMsg>("chatter");
    if (!chatter.valid()) {
        std::printf("FATAL: publisher creation failed\n");
        vTaskDelete(nullptr);
        return;
    }

    std::printf("[Rosette] device online, publishing std_msgs/msg/String on rt/chatter\n");

    uint32_t sequence = 0u;
    uint32_t last_publish_ms = 0u;
    for (;;) {
        const uint32_t now_ms = rtps_platform_get_ms();
        context.tick(now_ms);

        if ((now_ms - last_publish_ms) >= kPublishPeriodMs) {
            DemoStringMsg msg{};
            std::snprintf(msg.data,
                          sizeof(msg.data),
                          "Rosette hello seq=%lu uptime_ms=%lu",
                          static_cast<unsigned long>(sequence++),
                          static_cast<unsigned long>(now_ms));
            (void)chatter.publish(msg);
            last_publish_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(kTickPeriodMs));
    }
}

void blink_timer_cb(TimerHandle_t)
{
    static bool on = false;
    gpio_put(RTPS_PIN_LED, on ? 1u : 0u);
    on = !on;
}

}  // namespace

namespace milkros {

template<>
struct message_traits<DemoStringMsg> {
    static constexpr size_t max_serialized_size = 128u;

    static const char* dds_type_name() noexcept
    {
        return "std_msgs::msg::dds_::String_";
    }

    static size_t encode(uint8_t* out,
                         size_t out_cap,
                         const DemoStringMsg& message) noexcept
    {
        cdr_writer_t writer{};
        cdr_writer_init(&writer, out, out_cap);
        cdr_write_string(&writer, message.data);
        if (writer.overflow) {
            return 0u;
        }
        return cdr_writer_size(&writer);
    }

    static bool decode(const uint8_t* in,
                       size_t in_len,
                       DemoStringMsg& message) noexcept
    {
        cdr_reader_t reader{};
        if (!cdr_reader_init(&reader, in, in_len)) {
            return false;
        }
        (void)cdr_read_string(&reader, message.data, sizeof(message.data));
        return !reader.error;
    }

    static DemoStringMsg make_message() noexcept
    {
        return DemoStringMsg{};
    }
};

template<>
inline constexpr bool has_message_traits_v<DemoStringMsg> = true;

}  // namespace milkros

extern "C" const char* pcApplicationHostnameHook(void)
{
    return RTPS_NET_HOSTNAME;
}

extern "C" BaseType_t xApplicationGetRandomNumber(uint32_t* pulNumber)
{
    if (pulNumber == nullptr) {
        return pdFAIL;
    }

    g_random_state = mix_random(
        g_random_state ^
        static_cast<uint32_t>(time_us_64()) ^
        static_cast<uint32_t>(xTaskGetTickCount()));
    *pulNumber = g_random_state;
    return pdPASS;
}

extern "C" uint32_t ulApplicationGetNextSequenceNumber(uint32_t,
                                                        uint16_t,
                                                        uint32_t,
                                                        uint16_t)
{
    uint32_t value = 0u;
    if (xApplicationGetRandomNumber(&value) != pdPASS) {
        value = mix_random(static_cast<uint32_t>(time_us_64()));
    }
    return value;
}

extern "C" eDHCPCallbackAnswer_t xApplicationDHCPHook_Multi(
    eDHCPCallbackPhase_t,
    struct xNetworkEndPoint*,
    IP_Address_t*)
{
    return eDHCPContinue;
}

int main()
{
    board_init();

    gpio_put(RTPS_PIN_LED, 0u);
    TimerHandle_t blink = xTimerCreate("blink",
                                       pdMS_TO_TICKS(kBlinkPeriodMs),
                                       pdTRUE,
                                       nullptr,
                                       &blink_timer_cb);
    if (blink != nullptr) {
        (void)xTimerStart(blink, 0);
    }

    if (xTaskCreate(service_task,
                    "rosette",
                    3072,
                    nullptr,
                    3,
                    nullptr) != pdPASS) {
        for (;;) {
            tight_loop_contents();
        }
    }

    vTaskStartScheduler();

    for (;;) {
        tight_loop_contents();
    }
}
