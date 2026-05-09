#include "ros/micro_ros.hpp"
#include "ros/micro_ros/socket_transport.hpp"

#include "rtps/types.hpp"

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/detail/float32__functions.h>
#include <std_msgs/msg/detail/float32__type_support.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <array>

#ifdef RTPS_NO_FREERTOS
#include "net/posix/socket.hpp"

#include <chrono>
#include <fstream>
#include <thread>
#else
#include "board_config.h"
#include "net/freertos/socket.hpp"
#include "net/ntp_sync.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "timers.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IPv6.h"
#include "FreeRTOS_ND.h"
#include "FreeRTOS_Routing.h"
}

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

extern "C" void board_init(void);
extern "C" void board_get_mac_address(uint8_t mac[6]);
#endif

namespace {

constexpr char kNodeName[] = "rosette_micro_ros_smoke";
constexpr char kNodeNamespace[] = "";
constexpr char kTopicName[] = "mcu_temperature_c";
constexpr uint32_t kPublishPeriodMs = 1000u;

#ifndef ROSETTE_MICRO_ROS_AGENT_IP
#define ROSETTE_MICRO_ROS_AGENT_IP ""
#endif

#ifndef ROSETTE_MICRO_ROS_AGENT_PORT
#define ROSETTE_MICRO_ROS_AGENT_PORT "8888"
#endif

#ifndef ROSETTE_MICRO_ROS_DISCOVERY_GROUP
#define ROSETTE_MICRO_ROS_DISCOVERY_GROUP "239.255.0.2"
#endif

#ifndef ROSETTE_MICRO_ROS_DISCOVERY_PORT
#define ROSETTE_MICRO_ROS_DISCOVERY_PORT "7400"
#endif

#ifndef ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS
#define ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS 750
#endif

#ifndef ROSETTE_MICRO_ROS_DISCOVERY_ATTEMPTS
#define ROSETTE_MICRO_ROS_DISCOVERY_ATTEMPTS 3
#endif

#ifdef RTPS_NO_FREERTOS

using SocketProviderImpl = milkdds::net::posix::PosixSocketProvider;

uint64_t monotonic_time_us() noexcept
{
    using clock = std::chrono::steady_clock;
    static const auto start = clock::now();
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - start);
    return static_cast<uint64_t>(elapsed.count());
}

float read_posix_temperature_c() noexcept
{
    const char* candidates[] = {
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/hwmon/hwmon0/temp1_input",
        "/sys/class/hwmon/hwmon1/temp1_input",
    };

    for (const char* path : candidates) {
        std::ifstream input(path);
        long raw_milli_c = 0;
        if (input >> raw_milli_c) {
            return static_cast<float>(raw_milli_c) / 1000.0f;
        }
    }

    return std::numeric_limits<float>::quiet_NaN();
}

int run_posix_smoke()
{
    milkdds::net::posix::PosixSocketProvider socket_provider;

    milkros::micro_ros::ContextOptions options{};
    options.domain_id = 0u;
    options.executor_handles = 4u;
    options.spin_timeout_ns = 1000000ull;

#if ROSETTE_MICRO_ROS_TRANSPORT_CUSTOM
    const uint16_t agent_port =
        static_cast<uint16_t>(std::atoi(ROSETTE_MICRO_ROS_AGENT_PORT));
    const auto local = milkdds::net::make_socket_address_v4({0u, 0u, 0u, 0u}, 0u);
    milkdds::net::SocketAddress remote =
        milkdds::net::make_socket_address_v4({0u, 0u, 0u, 0u}, agent_port);

    const char* agent_ip = ROSETTE_MICRO_ROS_AGENT_IP;
    if ((agent_ip != nullptr) && (agent_ip[0] != '\0') &&
        !milkros::micro_ros::make_ipv4_socket_address(agent_ip, agent_port, remote)) {
        std::fprintf(stderr, "[micro_ros] invalid agent IP: %s\n", agent_ip);
        return 1;
    }

    milkros::micro_ros::UdpSocketTransport transport(socket_provider, remote, local, 100u);
    milkros::micro_ros::MulticastAgentLocator locator(
        socket_provider,
        milkros::micro_ros::MulticastAgentLocatorConfig{
            .multicast_group = ROSETTE_MICRO_ROS_DISCOVERY_GROUP,
            .multicast_port =
                static_cast<uint16_t>(std::atoi(ROSETTE_MICRO_ROS_DISCOVERY_PORT)),
            .default_agent_port = agent_port,
            .receive_timeout_ms = ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS,
            .attempts = ROSETTE_MICRO_ROS_DISCOVERY_ATTEMPTS,
        });
    options.use_custom_transport = true;
    options.custom_transport_framing = false;
    options.custom_transport = &transport;
    if ((agent_ip != nullptr) && (agent_ip[0] != '\0')) {
        options.udp_agent_address = agent_ip;
        options.udp_agent_port = ROSETTE_MICRO_ROS_AGENT_PORT;
    } else {
        options.agent_locator = &locator;
    }
#else
    if (ROSETTE_MICRO_ROS_AGENT_IP[0] != '\0') {
        options.udp_agent_address = ROSETTE_MICRO_ROS_AGENT_IP;
        options.udp_agent_port = ROSETTE_MICRO_ROS_AGENT_PORT;
        options.auto_discover_agent = false;
    } else {
        options.auto_discover_agent = true;
    }
#endif

    milkros::micro_ros::Context context(options);
    if (!context.start()) {
        std::fprintf(stderr, "[micro_ros] context start failed\n");
        return 1;
    }

    auto node = context.create_node(kNodeName, kNodeNamespace);
    if (!node.valid()) {
        std::fprintf(stderr, "[micro_ros] node creation failed\n");
        return 1;
    }

    auto publisher = node.create_publisher<std_msgs__msg__Float32>(kTopicName);
    if (!publisher.valid()) {
        std::fprintf(stderr, "[micro_ros] publisher creation failed\n");
        return 1;
    }

    std::printf("[micro_ros] posix smoke online, publishing %s\n", kTopicName);

    uint64_t last_publish_us = 0u;
    for (;;) {
        const uint64_t now_us = monotonic_time_us();
        (void)context.tick(static_cast<uint32_t>(now_us / 1000u));

        if ((now_us - last_publish_us) >= (static_cast<uint64_t>(kPublishPeriodMs) * 1000u)) {
            std_msgs__msg__Float32 msg{};
            msg.data = read_posix_temperature_c();
            (void)publisher.publish(msg);
            last_publish_us = now_us;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

#else

using SocketProviderImpl = milkdds::net::freertos::FreeRtosSocketProvider;

constexpr uint32_t kTickPeriodMs = 10u;
constexpr uint32_t kNetworkPollDelayMs = 250u;
constexpr uint32_t kBlinkPeriodMs = 500u;
constexpr EventBits_t kNetworkReadyBit = 1u << 0u;
constexpr EventBits_t kNtpSyncedBit = 1u << 1u;

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
#define RTPS_NET_HOSTNAME "rosette-micro-ros"
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

NetworkInterface_t g_network_interface{};
NetworkEndPoint_t g_ipv4_endpoint{};
NetworkEndPoint_t g_ipv6_endpoint{};
bool g_network_started = false;
uint32_t g_random_state = 0x51A7C0DEu;
EventGroupHandle_t g_network_events = nullptr;

constexpr std::array<uint8_t, 4> kStaticIp = RTPS_NET_STATIC_IP;
constexpr std::array<uint8_t, 4> kStaticMask = RTPS_NET_STATIC_MASK;
constexpr std::array<uint8_t, 4> kStaticGateway = RTPS_NET_STATIC_GW;
constexpr std::array<uint8_t, 4> kStaticDns = RTPS_NET_STATIC_DNS;
constexpr std::array<uint8_t, 4> kZeroconfIp = RTPS_NET_ZEROCONF_IP;
constexpr std::array<uint8_t, 16> kLinkLocalPrefix{
    0xFEu, 0x80u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
};

float read_mcu_temperature_c() noexcept
{
    adc_select_input(4u);
    const uint16_t raw = adc_read();
    const float conversion = 3.3f / 4096.0f;
    const float voltage = static_cast<float>(raw) * conversion;
    return 27.0f - ((voltage - 0.706f) / 0.001721f);
}

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

void blink_timer_cb(TimerHandle_t) noexcept
{
    static bool on = false;
    gpio_put(RTPS_PIN_LED, on ? 1u : 0u);
    on = !on;
}

void service_task(void*) noexcept
{
    if (!start_board_network()) {
        vTaskDelete(nullptr);
        return;
    }

    while (!network_ready()) {
        vTaskDelay(pdMS_TO_TICKS(kNetworkPollDelayMs));
    }

    xEventGroupSetBits(g_network_events, kNetworkReadyBit);

    static milkdds::net::freertos::FreeRtosSocketProvider socket_provider;
    static milkdds::net::NtpSyncService ntp_service(
        socket_provider,
        milkdds::net::NtpSyncConfig{
            .server = milkdds::net::make_socket_address_v4({216u, 239u, 35u, 0u}, 123u),
            .local_port = 12300u,
            .receive_timeout_ms = 5000u,
            .min_poll_ms = 64000u,
            .max_poll_ms = 1024000u,
            .step_threshold_us = 128000ll,
            .phase_gain = 1.0,
            .freq_gain = 0.0625,
            .max_freq_correction_us_per_s = 500.0,
            .filter_size = 8u,
            .task_stack_words = 1024u,
            .task_priority = tskIDLE_PRIORITY + 1u,
            .task_name = "ntp",
            .ready_events = g_network_events,
            .ready_mask = kNetworkReadyBit,
            .notify_events = g_network_events,
            .synced_bit = kNtpSyncedBit,
        });
    (void)ntp_service.start();

    (void)xEventGroupWaitBits(g_network_events,
                              kNtpSyncedBit,
                              pdFALSE,
                              pdFALSE,
                              pdMS_TO_TICKS(30000u));

    milkros::micro_ros::ContextOptions options{};
    options.domain_id = 0u;
    options.executor_handles = 4u;
    options.spin_timeout_ns = 1000000ull;

#if ROSETTE_MICRO_ROS_TRANSPORT_CUSTOM
    const uint16_t agent_port =
        static_cast<uint16_t>(std::atoi(ROSETTE_MICRO_ROS_AGENT_PORT));
    milkdds::net::SocketAddress remote =
        milkdds::net::make_socket_address_v4({0u, 0u, 0u, 0u}, agent_port);
    if (ROSETTE_MICRO_ROS_AGENT_IP[0] != '\0' &&
        !milkros::micro_ros::make_ipv4_socket_address(
            ROSETTE_MICRO_ROS_AGENT_IP,
            agent_port,
            remote)) {
        std::printf("FATAL: invalid agent IP '%s'\n", ROSETTE_MICRO_ROS_AGENT_IP);
        vTaskDelete(nullptr);
        return;
    }

    const auto local = milkdds::net::make_socket_address_v4({0u, 0u, 0u, 0u}, 0u);
    static milkros::micro_ros::UdpSocketTransport transport(
        socket_provider, remote, local, 100u);
    static milkros::micro_ros::MulticastAgentLocator locator(
        socket_provider,
        milkros::micro_ros::MulticastAgentLocatorConfig{
            .multicast_group = ROSETTE_MICRO_ROS_DISCOVERY_GROUP,
            .multicast_port =
                static_cast<uint16_t>(std::atoi(ROSETTE_MICRO_ROS_DISCOVERY_PORT)),
            .default_agent_port = agent_port,
            .receive_timeout_ms = ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS,
            .attempts = ROSETTE_MICRO_ROS_DISCOVERY_ATTEMPTS,
        });
    options.use_custom_transport = true;
    options.custom_transport_framing = false;
    options.custom_transport = &transport;
    if (ROSETTE_MICRO_ROS_AGENT_IP[0] != '\0') {
        options.udp_agent_address = ROSETTE_MICRO_ROS_AGENT_IP;
        options.udp_agent_port = ROSETTE_MICRO_ROS_AGENT_PORT;
    } else {
        options.agent_locator = &locator;
    }
#else
    if (ROSETTE_MICRO_ROS_AGENT_IP[0] != '\0') {
        options.udp_agent_address = ROSETTE_MICRO_ROS_AGENT_IP;
        options.udp_agent_port = ROSETTE_MICRO_ROS_AGENT_PORT;
        options.auto_discover_agent = false;
    } else {
        options.auto_discover_agent = true;
    }
#endif

    static milkros::micro_ros::Context context(options);
    if (!context.start()) {
        std::printf("FATAL: micro-ROS context start failed\n");
        vTaskDelete(nullptr);
        return;
    }

    auto node = context.create_node(kNodeName, kNodeNamespace);
    auto publisher = node.create_publisher<std_msgs__msg__Float32>(kTopicName);
    if (!node.valid() || !publisher.valid()) {
        std::printf("FATAL: micro-ROS node/publisher creation failed\n");
        vTaskDelete(nullptr);
        return;
    }

    std::printf("[micro_ros] board smoke online, publishing %s\n", kTopicName);

    uint32_t last_publish_ms = 0u;
    for (;;) {
        const uint32_t now_ms = rtps_platform_get_ms();
        (void)context.tick(now_ms);

        if ((now_ms - last_publish_ms) >= kPublishPeriodMs) {
            std_msgs__msg__Float32 msg{};
            msg.data = read_mcu_temperature_c();
            (void)publisher.publish(msg);
            last_publish_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(kTickPeriodMs));
    }
}

#endif

}  // namespace

namespace milkros {

template<>
struct message_traits<std_msgs__msg__Float32> {
    static const rosidl_message_type_support_t* rosidl_type_support() noexcept
    {
        return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
    }

    static void init_message(std_msgs__msg__Float32& message) noexcept
    {
        (void)std_msgs__msg__Float32__init(&message);
    }

    static void fini_message(std_msgs__msg__Float32& message) noexcept
    {
        std_msgs__msg__Float32__fini(&message);
    }
};

template<>
inline constexpr bool has_message_traits_v<std_msgs__msg__Float32> = true;

}  // namespace milkros

#ifdef RTPS_NO_FREERTOS
int main()
{
    return run_posix_smoke();
}
#else
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

extern "C" void vApplicationIPNetworkEventHook_Multi(
    eIPCallbackEvent_t eNetworkEvent,
    struct xNetworkEndPoint*)
{
    if (g_network_events == nullptr) {
        return;
    }

    if (eNetworkEvent == eNetworkUp) {
        xEventGroupSetBits(g_network_events, kNetworkReadyBit);
    } else {
        xEventGroupClearBits(g_network_events, kNetworkReadyBit);
    }
}

int main()
{
    board_init();

    adc_init();
    adc_set_temp_sensor_enabled(true);

    g_network_events = xEventGroupCreate();
    if (g_network_events == nullptr) {
        for (;;) {
            tight_loop_contents();
        }
    }

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
                    "micro_ros",
                    4096,
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
#endif
