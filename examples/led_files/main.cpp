/**
 * @file examples/led_files/main.cpp
 * @brief A minimal Rosette firmware: an LED service and an on-device file store.
 *
 * Shows the whole shape of a Rosette application:
 *   - `rosette::App` owns the micro-ROS session and the reconnect loop.
 *   - each `rosette::Module` groups one feature's entities and callbacks.
 *   - services are plain `(request, response)` callbacks.
 *   - entities live in `std::optional<...>` members so `App` can rebuild them
 *     across an Agent reconnect.
 *
 * Runs on the host (stock UDP transport) and on the W5100S-EVB-Pico2 (WIZnet
 * hardware sockets, no RTOS, superloop).
 *
 * Host:
 *   cmake -S . -B build -DROSETTE_BOARD=posix ; cmake --build build
 *   micro_ros_agent udp4 --port 8888
 *   ROSETTE_AGENT_IP=127.0.0.1 ./build/led_files
 *
 * W5100S-EVB-Pico2:
 *   cmake -S . -B build -DROSETTE_BOARD=w5100s-evb-pico2 \
 *         -DROSETTE_MICRO_ROS_AGENT_IP=192.168.1.10
 *   cmake --build build          # flash build/led_files.uf2
 *
 * Try it:
 *   ros2 service call /led_files/led/set   rosette_msgs/srv/SetLed   "{led: 0, state: true}"
 *   ros2 service call /led_files/files/put  rosette_msgs/srv/PutFile  "{key: k1, body: hello}"
 *   ros2 service call /led_files/files/get  rosette_msgs/srv/GetFile  "{key: k1}"
 *   ros2 service call /led_files/files/list rosette_msgs/srv/ListFiles "{}"
 */

#include "rosette_msgs_traits.hpp"

#include "rosette/ros/ros.hpp"
#include "rosette/ros/socket_transport.hpp"
#include "rosette/storage/file_store.hpp"
#include "rosette/storage/volume.hpp"

#include <rosidl_runtime_c/string_functions.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>

#if defined(ROSETTE_BOARD_PICO)
#include "board.h"
#include "rosette/net/wiznet/board.hpp"
#include "rosette/net/wiznet/link.hpp"
#endif

namespace {

// --------------------------------------------------------------------------
// LED module — rosette_msgs/srv/SetLed on ~/led/set
// --------------------------------------------------------------------------
class LedModule final : public rosette::Module {
public:
    const char* name() const noexcept override { return "led"; }
    uint8_t executor_handles() const noexcept override { return 1u; }

    void register_(rosette::Node& node) noexcept override
    {
        node.create_service<rosette::SetLedSrv>(
            service_, "led/set",
            [this](const Request& req, Response& resp) noexcept {
                set_led(req.led, req.state);
                resp.success = true;
            });
    }

    void on_disconnect() noexcept override { service_.reset(); }

private:
    using Request = rosette::service_traits<rosette::SetLedSrv>::Request;
    using Response = rosette::service_traits<rosette::SetLedSrv>::Response;

    void set_led(uint8_t index, bool on) noexcept
    {
        if (index != 0u) {
            return;
        }
#if defined(ROSETTE_BOARD_PICO)
        board_set_led(on);
#else
        std::printf("[led] LED %u -> %s\n", index, on ? "on" : "off");
#endif
    }

    std::optional<rosette::Service<rosette::SetLedSrv>> service_;
};

// --------------------------------------------------------------------------
// Files module — the rosette_msgs file-store services over a FileStore
// --------------------------------------------------------------------------
class FilesModule final : public rosette::Module {
public:
    explicit FilesModule(rosette::storage::Volume& volume) noexcept
        : store_(volume, "/files", "", 4096u)
    {}

    const char* name() const noexcept override { return "files"; }
    uint8_t executor_handles() const noexcept override { return 4u; }

    void register_(rosette::Node& node) noexcept override
    {
        node.create_service<rosette::PutFileSrv>(
            put_, "files/put",
            [this](const PutReq& rq, PutResp& rs) noexcept { on_put(rq, rs); });
        node.create_service<rosette::GetFileSrv>(
            get_, "files/get",
            [this](const GetReq& rq, GetResp& rs) noexcept { on_get(rq, rs); });
        node.create_service<rosette::ListFilesSrv>(
            list_, "files/list",
            [this](const ListReq&, ListResp& rs) noexcept { on_list(rs); });
        node.create_service<rosette::EraseFileSrv>(
            erase_, "files/erase",
            [this](const EraseReq& rq, EraseResp& rs) noexcept {
                rs.success = store_.erase(rq.key.data) == rosette::storage::FileStoreStatus::ok;
            });
    }

    void on_disconnect() noexcept override
    {
        put_.reset();
        get_.reset();
        list_.reset();
        erase_.reset();
    }

private:
    using PutReq = rosette::service_traits<rosette::PutFileSrv>::Request;
    using PutResp = rosette::service_traits<rosette::PutFileSrv>::Response;
    using GetReq = rosette::service_traits<rosette::GetFileSrv>::Request;
    using GetResp = rosette::service_traits<rosette::GetFileSrv>::Response;
    using ListReq = rosette::service_traits<rosette::ListFilesSrv>::Request;
    using ListResp = rosette::service_traits<rosette::ListFilesSrv>::Response;
    using EraseReq = rosette::service_traits<rosette::EraseFileSrv>::Request;
    using EraseResp = rosette::service_traits<rosette::EraseFileSrv>::Response;

    void on_put(const PutReq& rq, PutResp& rs) noexcept
    {
        const auto st = store_.put(rq.key.data, rq.body.data, rq.body.size);
        rs.success = st == rosette::storage::FileStoreStatus::ok;
        rosidl_runtime_c__String__assign(&rs.message,
                                         rosette::storage::file_store_status_string(st));
    }

    void on_get(const GetReq& rq, GetResp& rs) noexcept
    {
        static char buf[4096];
        size_t len = 0u;
        const auto st = store_.get(rq.key.data, buf, sizeof(buf), len);
        rs.success = st == rosette::storage::FileStoreStatus::ok;
        if (rs.success) {
            rosidl_runtime_c__String__assignn(&rs.body, buf, len);
        }
        rosidl_runtime_c__String__assign(&rs.message,
                                         rosette::storage::file_store_status_string(st));
    }

    void on_list(ListResp& rs) noexcept
    {
        static char keys[16][rosette::storage::kFileStoreKeyMax + 1];
        const uint8_t n = store_.list(keys, 16u);
        if (!rosidl_runtime_c__String__Sequence__init(&rs.keys, n)) {
            return;
        }
        for (uint8_t i = 0; i < n; ++i) {
            rosidl_runtime_c__String__assign(&rs.keys.data[i], keys[i]);
        }
    }

    rosette::storage::FileStore store_;
    std::optional<rosette::Service<rosette::PutFileSrv>> put_;
    std::optional<rosette::Service<rosette::GetFileSrv>> get_;
    std::optional<rosette::Service<rosette::ListFilesSrv>> list_;
    std::optional<rosette::Service<rosette::EraseFileSrv>> erase_;
};

void configure_app(rosette::AppOptions& options)
{
    options.context.udp_agent_port = 8888u;
    options.context.auto_discover_agent = false;
    options.node_namespace = "led_files";   // entities land under /led_files/...
    options.node_name = "board";
}

}  // namespace

#if defined(ROSETTE_BOARD_PICO)

int main()
{
    board_init();

    std::array<uint8_t, 6> mac{};
    board_get_mac_address(mac.data());

    namespace wiz = rosette::net::wiznet;
    auto bus = wiz::make_board_bus();
    wiz::BoardDevice device{bus};
    wiz::BoardSocketProvider sockets{device};

    wiz::BoardLink link{device, sockets, mac, wiz::LinkMode::dhcp};
    while (!link.begin()) {
        std::printf("[led_files] waiting for network link\n");
        rosette_platform_delay_ms(500u);
    }
    const auto ip = link.ipv4();
    std::printf("[led_files] link up: %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);

    rosette::storage::Volume volume;

    // WIZnet hardware-socket transport to the Agent.
    const char* agent_ip = ROSETTE_MICRO_ROS_AGENT_IP;
    if (agent_ip[0] == '\0') {
        std::printf("[led_files] no ROSETTE_MICRO_ROS_AGENT_IP configured\n");
        for (;;) { rosette_platform_delay_ms(1000u); }
    }

    rosette::net::SocketAddress agent{};
    (void)rosette::make_ipv4_socket_address(agent_ip, 8888u, agent);
    const auto local = rosette::net::make_socket_address_v4({0u, 0u, 0u, 0u}, 0u);
    rosette::UdpSocketTransport transport(sockets, agent, local, /*receive_timeout_ms=*/50u);

    rosette::AppOptions options{};
    configure_app(options);
    options.context.use_custom_transport = true;
    options.context.custom_transport = &transport;
    options.context.custom_transport_framing = false;   // UDP is packet-oriented
    // Hand the compiled-in Agent IP to the context: the custom-transport path
    // has no auto-discovery, so without a static endpoint Context::discover_agent
    // fails with kCustomTransportRequiresEndpoint.
    options.context.udp_agent_address = agent_ip;

    rosette::App app(options);
    static LedModule led;
    static FilesModule files(volume);
    app.add(led);
    app.add(files);

    std::printf("[led_files] agent %s:8888\n", agent_ip);
    bool was_connected = false;
    for (;;) {
        (void)app.spin_once(rosette_platform_get_ms());
        link.poll();
        if (app.connected() != was_connected) {
            was_connected = app.connected();
            std::printf("[led_files] %s\n",
                        was_connected ? "agent connected" : "agent lost, retrying");
        }
        rosette_platform_wait_for_event(app.connected() ? 5u : 200u);
    }
}

#else  // host

int main()
{
    rosette::storage::Volume volume;
    if (!volume.ok()) {
        std::printf("[led_files] storage volume not available\n");
    }

    rosette::AppOptions options{};
    configure_app(options);
    const char* agent_ip = std::getenv("ROSETTE_AGENT_IP");
    options.context.udp_agent_address =
        (agent_ip != nullptr && agent_ip[0] != '\0') ? agent_ip : "127.0.0.1";

    rosette::App app(options);
    static LedModule led;
    static FilesModule files(volume);
    app.add(led);
    app.add(files);

    std::printf("[led_files] agent %s:8888\n", options.context.udp_agent_address);
    app.spin();
    return 0;
}

#endif
