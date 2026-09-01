/**
 * @file examples/rosette_smoke.cpp
 * @brief Minimal micro-ROS client smoke: publish a Float32 to an Agent.
 *
 * This is the smallest program that exercises the promoted `rosette::` wrapper
 * end to end against a real `micro_ros_agent`. It uses the stock XRCE-DDS UDP
 * transport, so it needs no board BSP and builds on the host
 * (`ROSETTE_BOARD=posix`).
 *
 * Run an Agent, then this program:
 *   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 *   ROSETTE_AGENT_IP=127.0.0.1 ./rosette_smoke
 *   ros2 topic echo /mcu_temperature_c std_msgs/msg/Float32
 *
 * The reconnect state machine, executor auto-sizing, and board transports land
 * in later phases of the micro-ROS refactor; this file stays deliberately flat.
 */

#include "rosette/ros/ros.hpp"

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/detail/float32__functions.h>
#include <std_msgs/msg/detail/float32__type_support.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace rosette {

/**
 * @brief Trait binding `std_msgs/msg/Float32` to its rosidl type support.
 *
 * The wrapper needs `rosidl_type_support()` for every published message type;
 * the optional init/fini hooks let it manage a message that owns dynamic
 * memory (Float32 does not, but the hooks are harmless and match the pattern
 * used for string- and sequence-bearing types).
 */
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

}  // namespace rosette

namespace {

constexpr char kNodeName[] = "rosette_smoke";
constexpr char kTopicName[] = "mcu_temperature_c";
constexpr uint32_t kPublishPeriodMs = 1000u;
constexpr uint32_t kStartRetryMs = 2000u;

/**
 * @brief Returns a monotonic millisecond counter for the tick loop.
 *
 * @return Milliseconds since program start.
 */
uint32_t now_ms() noexcept
{
    using clock = std::chrono::steady_clock;
    static const auto start = clock::now();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start).count());
}

/**
 * @brief Produces a slow synthetic "temperature" so the topic has motion.
 *
 * @param[in] t_ms Current millisecond timestamp.
 * @return A value oscillating gently around room temperature.
 */
float synthetic_temperature_c(uint32_t t_ms) noexcept
{
    const float phase = static_cast<float>(t_ms % 60000u) / 60000.0f;
    return 22.0f + 2.0f * (phase - 0.5f);
}

}  // namespace

int main()
{
    rosette::ContextOptions options{};
    const char* agent_ip = std::getenv("ROSETTE_AGENT_IP");
    options.udp_agent_address = (agent_ip != nullptr && agent_ip[0] != '\0') ? agent_ip : "127.0.0.1";
    options.udp_agent_port = 8888u;
    options.auto_discover_agent = false;

    rosette::Context context(options);

    while (true) {
        const rosette::ContextStartResult started = context.start_result();
        if (started.ok()) {
            break;
        }
        std::printf("[rosette_smoke] context start failed: %s — retrying\n",
                    rosette::context_start_error_string(context.last_start_error()));
        std::this_thread::sleep_for(std::chrono::milliseconds(kStartRetryMs));
    }

    auto node = context.create_node(kNodeName, "");
    auto publisher = node.create_publisher<std_msgs__msg__Float32>(kTopicName);
    if (!publisher.valid()) {
        std::printf("[rosette_smoke] publisher init failed\n");
        return 1;
    }

    std::printf("[rosette_smoke] online, publishing %s to %s:%u\n",
                kTopicName, options.udp_agent_address, options.udp_agent_port);

    uint32_t last_publish_ms = 0u;
    for (;;) {
        const uint32_t t_ms = now_ms();
        (void)context.tick(t_ms);

        if ((t_ms - last_publish_ms) >= kPublishPeriodMs) {
            std_msgs__msg__Float32 message{};
            message.data = synthetic_temperature_c(t_ms);
            (void)publisher.publish(message);
            last_publish_ms = t_ms;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
