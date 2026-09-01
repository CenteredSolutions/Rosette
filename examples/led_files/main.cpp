/**
 * @file examples/led_files/main.cpp
 * @brief A minimal Rosette firmware: an LED service and an on-device file store.
 *
 * Shows the whole shape of a Rosette application:
 *   - `rosette::App` owns the micro-ROS session and the reconnect loop.
 *   - each `rosette::Module` groups one feature's entities and callbacks.
 *   - services are plain `(request, response)` callbacks.
 *
 * Entities live in `std::optional<...>` members so `App` can tear them down and
 * rebuild them across an Agent reconnect.
 *
 * Build (host):
 *   cmake -S . -B build -DROSETTE_BOARD=posix -DROSETTE_ENABLE_MICRO_ROS=ON \
 *     -DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF \
 *     -DROSETTE_MICRO_ROS_WORKSPACE_DIR=<ws with libmicroros.a>
 *   cmake --build build --target led_files
 *
 * Try it (host `rosette_msgs` must be on the ROS 2 path — `colcon build` it):
 *   <agent>/micro_ros_agent udp4 --port 8888
 *   ROSETTE_AGENT_IP=127.0.0.1 ./led_files
 *   ros2 service call /led_files/led/set   rosette_msgs/srv/SetLed   "{led: 0, state: true}"
 *   ros2 service call /led_files/files/put  rosette_msgs/srv/PutFile  "{key: k1, body: hello}"
 *   ros2 service call /led_files/files/get  rosette_msgs/srv/GetFile  "{key: k1}"
 *   ros2 service call /led_files/files/list rosette_msgs/srv/ListFiles "{}"
 */

#include "rosette_msgs_traits.hpp"

#include "rosette/ros/ros.hpp"
#include "rosette/storage/file_store.hpp"
#include "rosette/storage/volume.hpp"

#include <rosidl_runtime_c/string_functions.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>

#if defined(ROSETTE_BOARD_PICO)
#include "pico/stdlib.h"
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
#if defined(ROSETTE_BOARD_PICO)
        if (index == 0u) {
            gpio_put(PICO_DEFAULT_LED_PIN, on);
        }
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

}  // namespace

int main()
{
#if defined(ROSETTE_BOARD_PICO)
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

    rosette::storage::Volume volume;
    if (!volume.ok()) {
        std::printf("[led_files] storage volume not available\n");
    }

    rosette::AppOptions options{};
    const char* agent_ip = std::getenv("ROSETTE_AGENT_IP");
    options.context.udp_agent_address =
        (agent_ip != nullptr && agent_ip[0] != '\0') ? agent_ip : "127.0.0.1";
    options.context.udp_agent_port = 8888u;
    options.context.auto_discover_agent = false;
    options.node_namespace = "led_files";  // entities land under /led_files/...
    options.node_name = "board";

    rosette::App app(options);

    static LedModule led;
    static FilesModule files(volume);
    app.add(led);
    app.add(files);

    std::printf("[led_files] starting; agent %s:%u\n",
                options.context.udp_agent_address, options.context.udp_agent_port);
    app.spin();
    return 0;
}
