#pragma once

#include <array>
#include <memory>
#include <new>

#include "FreeRTOS.h"
#include "task.h"

#include "device.hpp"
#include "../socket.hpp"

namespace rosette::net::wiznet {

template<typename Chip, typename Bus>
class IolibrarySocketProvider;

template<typename Chip, typename Bus>
class IolibraryUdpSocket final : public Socket {
public:
    using device_type = WiznetDevice<Chip, Bus>;
    using socket_type = typename device_type::socket_type;

    IolibraryUdpSocket(device_type& device,
                       IolibrarySocketProvider<Chip, Bus>* provider,
                       socket_type socket) noexcept
        : device_(device)
        , provider_(provider)
        , socket_(socket) {}

    ~IolibraryUdpSocket() override
    {
        close();
    }

    bool is_open() const noexcept override
    {
        return open_;
    }

    int joinMulticastGroup(const SocketMulticastMembership& membership) noexcept override
    {
        if (open_) {
            last_error_ = SocketError::invalid_argument;
            return -1;
        }

        if (membership.domain != SocketDomain::ipv4) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        if ((membership.group[0] < 224u) || (membership.group[0] > 239u)) {
            last_error_ = SocketError::invalid_argument;
            return -1;
        }

        if (multicast_joined_ && !same_membership(membership, multicast_membership_)) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        multicast_membership_ = membership;
        multicast_joined_ = true;
        last_error_ = SocketError::none;
        return 0;
    }

    int leaveMulticastGroup(const SocketMulticastMembership& membership) noexcept override
    {
        if (!multicast_joined_) {
            last_error_ = SocketError::none;
            return 0;
        }

        if (!same_membership(membership, multicast_membership_)) {
            last_error_ = SocketError::invalid_argument;
            return -1;
        }

        if (open_) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        multicast_membership_ = {};
        multicast_joined_ = false;
        last_error_ = SocketError::none;
        return 0;
    }

    int bind(const SocketAddress& local) noexcept override
    {
        if (local.domain != SocketDomain::ipv4) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        if (multicast_joined_) {
            if (!device_.open_udp_multicast_socket(socket_,
                                                   local.port,
                                                   {multicast_membership_.group[0],
                                                    multicast_membership_.group[1],
                                                    multicast_membership_.group[2],
                                                    multicast_membership_.group[3]},
                                                   multicast_membership_.port != 0u
                                                       ? multicast_membership_.port
                                                       : local.port)) {
                last_error_ = SocketError::io;
                return -1;
            }
        } else if (!device_.open_udp_socket(socket_, local.port)) {
            last_error_ = SocketError::io;
            return -1;
        }

        open_ = true;
        last_error_ = SocketError::none;
        return 0;
    }

    int set_receive_timeout(uint32_t timeout_ms) noexcept override
    {
        receive_timeout_ms_ = timeout_ms;
        last_error_ = SocketError::none;
        return 0;
    }

    std::ptrdiff_t send_to(const void* data,
                           std::size_t length,
                           const SocketAddress& remote) noexcept override
    {
        if (!open_) {
            last_error_ = SocketError::closed;
            return -1;
        }

        if (remote.domain != SocketDomain::ipv4) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        if (device_.socket_tx_capacity(socket_) == 0u) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        // TODO: Wait for previous send to complete

        const auto* bytes = static_cast<const uint8_t*>(data);
        const std::ptrdiff_t rc =
            device_.udp_send_to(socket_,
                                bytes,
                                length,
                                {remote.address[0], remote.address[1],
                                 remote.address[2], remote.address[3]},
                                remote.port);
        last_error_ = rc < 0 ? SocketError::io : SocketError::none;
        return rc;
    }

    std::ptrdiff_t recv_from(void* data,
                             std::size_t capacity,
                             SocketAddress* remote) noexcept override
    {
        if (!open_) {
            last_error_ = SocketError::closed;
            return -1;
        }


        if (device_.socket_rx_capacity(socket_) == 0u) {
            std::printf("Socket %d RX capacity is 0\n", socket_);
            last_error_ = SocketError::not_supported;
            return -1;
        }

        const TickType_t start = xTaskGetTickCount();
        device_.register_socket_rx_task(socket_);
        for (;;) {
            std::array<uint8_t, 4> address{};
            uint16_t port = 0u;
            const std::ptrdiff_t rc =
                device_.udp_receive_from(socket_,
                                         static_cast<uint8_t*>(data),
                                         capacity,
                                         remote != nullptr ? &address : nullptr,
                                         remote != nullptr ? &port : nullptr);

            if (rc != 0) {
                if ((rc >= 0) && (remote != nullptr)) {
                    *remote = make_socket_address_v4(address, port);
                }

                last_error_ = rc < 0 ? SocketError::io : SocketError::none;
                return rc;
            }

            if (receive_timeout_ms_ == 0u) {
                last_error_ = SocketError::would_block;
                return -1;
            }

            const TickType_t elapsed = xTaskGetTickCount() - start;
            const TickType_t timeout_ticks = pdMS_TO_TICKS(receive_timeout_ms_);
            if (elapsed >= timeout_ticks) {
                last_error_ = SocketError::timed_out;
                return -1;
            }

            const TickType_t remaining = timeout_ticks - elapsed;
            if (ulTaskNotifyTake(pdTRUE, remaining) == 0u) {
                last_error_ = SocketError::timed_out;
                return -1;
            }
        }
    }

    void close() noexcept override
    {
        if (!claimed_) {
            return;
        }

        if (open_) {
            device_.close_socket(socket_);
        }

        device_.unregister_socket_rx_task(socket_);

        if (provider_ != nullptr) {
            provider_->release(socket_);
        }

        open_ = false;
        claimed_ = false;
        multicast_joined_ = false;
        multicast_membership_ = {};
        last_error_ = SocketError::none;
    }

    SocketError last_error() const noexcept override
    {
        return last_error_;
    }

private:
    static bool same_membership(const SocketMulticastMembership& lhs,
                                const SocketMulticastMembership& rhs) noexcept
    {
        return (lhs.domain == rhs.domain) &&
               (lhs.group == rhs.group) &&
               (lhs.interface == rhs.interface) &&
               (lhs.port == rhs.port) &&
               (lhs.interface_index == rhs.interface_index);
    }

    device_type& device_;
    IolibrarySocketProvider<Chip, Bus>* provider_ = nullptr;
    socket_type socket_ = 0u;
    uint32_t receive_timeout_ms_ = 0u;
    SocketMulticastMembership multicast_membership_{};
    SocketError last_error_ = SocketError::none;
    bool open_ = false;
    bool claimed_ = true;
    bool multicast_joined_ = false;
};

template<typename Chip, typename Bus>
class IolibrarySocketProvider final : public SocketProvider {
public:
    using device_type = WiznetDevice<Chip, Bus>;
    using socket_type = typename device_type::socket_type;

    explicit IolibrarySocketProvider(device_type& device) noexcept
        : device_(device) {}

    ~IolibrarySocketProvider() override
    {
        stop_dispatch_task();
    }

    std::unique_ptr<Socket> open(
        SocketDomain domain = SocketDomain::ipv4,
        SocketType type = SocketType::datagram,
        SocketProtocol protocol = SocketProtocol::udp) noexcept override
    {
        if ((domain != SocketDomain::ipv4) ||
            (type != SocketType::datagram) ||
            (protocol != SocketProtocol::udp)) {
            return {};
        }

        if (!ensure_dispatch_task_started()) {
            return {};
        }

        const socket_type socket = claim_socket();
        if (socket >= Chip::socket_count()) {
            return {};
        }

        auto instance = std::unique_ptr<Socket>(
            new (std::nothrow) IolibraryUdpSocket<Chip, Bus>(device_, this, socket));
        if (!instance) {
            release(socket);
        }
        return instance;
    }

    void release(socket_type socket) noexcept
    {
        if (socket < Chip::socket_count()) {
            taskENTER_CRITICAL();
            claimed_[socket] = false;
            taskEXIT_CRITICAL();
        }
    }

private:
    static void dispatch_task_entry(void* arg) noexcept
    {
        static_cast<IolibrarySocketProvider*>(arg)->dispatch_task();
    }

    bool ensure_dispatch_task_started() noexcept
    {
        if (!socket_mode_ready_) {
            const auto mac = device_.mac_address();
            auto result = device_.init_socket_mode(mac);
            if (std::holds_alternative<typename device_type::InvalidChipVersion>(result)) {
                return false;
            }
            socket_mode_ready_ = true;
        }

        taskENTER_CRITICAL();
        const bool already_started = dispatch_task_started_;
        taskEXIT_CRITICAL();
        if (already_started) {
            return true;
        }

        TaskHandle_t task = nullptr;
        const BaseType_t rc = xTaskCreate(&dispatch_task_entry,
                                          "wiz_irq",
                                          512u,
                                          this,
                                          configMAX_PRIORITIES - 2u,
                                          &task);
        if (rc != pdPASS) {
            return false;
        }

        taskENTER_CRITICAL();
        dispatch_task_handle_ = task;
        dispatch_task_started_ = true;
        taskEXIT_CRITICAL();
        return true;
    }

    void stop_dispatch_task() noexcept
    {
        TaskHandle_t task = nullptr;
        taskENTER_CRITICAL();
        task = dispatch_task_handle_;
        dispatch_task_handle_ = nullptr;
        dispatch_task_started_ = false;
        taskEXIT_CRITICAL();

        if (task != nullptr) {
            vTaskDelete(task);
        }
    }

    void dispatch_task() noexcept
    {
        device_.register_interrupt_dispatch_task();
        for (;;) {
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            device_.service_socket_interrupts();
        }
    }

    /**
     * claim_socket
     * 
     * Claims an available socket and returns its index. If no sockets are
     * available, returns an index equal to the total socket count, which should
     * be treated as an invalid socket indicator by the caller.
     */
    socket_type claim_socket() noexcept
    {
        taskENTER_CRITICAL();
        for (socket_type socket = 0; socket < Chip::socket_count(); ++socket) {
            if ((device_.mode() == DeviceMode::macraw) &&
                (socket == Chip::macraw_socket())) {
                continue;
            }

            if (!claimed_[socket]) {
                claimed_[socket] = true;
                taskEXIT_CRITICAL();
                return socket;
            }
        }

        taskEXIT_CRITICAL();
        return Chip::socket_count();
    }

    device_type& device_;
    std::array<bool, Chip::socket_count()> claimed_{};
    TaskHandle_t dispatch_task_handle_ = nullptr;
    bool dispatch_task_started_ = false;
    bool socket_mode_ready_ = false;
};

}  // namespace rosette::wiznet
