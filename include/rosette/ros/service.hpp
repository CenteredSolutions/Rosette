#pragma once

/**
 * @file ros/service.hpp
 * @brief micro-ROS service server and client adapters for the Rosette ROS API.
 *
 * Design goal
 * -----------
 * This module keeps the application-facing shape of `rosette::Service` and
 * `rosette::Client` while routing the wire protocol and request correlation
 * through `rcl` + `rclc`. It gives MCU code the same "create a typed service
 * or client from a node" experience of `rclcpp` without duplicating
 * ROS 2 service semantics in Rosette itself.
 *
 * Example
 * -------
 *   auto service = node.create_service<AddTwoInts>(
 *       "add_two_ints",
 *       [](const AddTwoIntsRequest& request,
 *          AddTwoIntsResponse& response) noexcept {
 *           response.sum = request.a + request.b;
 *       });
 *
 *   auto client = node.create_client<AddTwoInts>("add_two_ints");
 *   AddTwoIntsRequest request{};
 *   request.a = 2;
 *   request.b = 40;
 *   client.async_send_request(
 *       request,
 *       [](const AddTwoIntsResponse& response, void*) noexcept {
 *           (void)response;
 *       });
 *
 * Features
 * --------
 * - Typed request and response access based on `service_traits<ServiceT>`.
 * - `rclc_executor_t` integration for both service callbacks and async replies.
 * - Optional per-handle XRCE session timeout tuning through `ServiceOptions`.
 *
 * Caveats
 * -------
 * - The service callback executes on the shared `rclc_executor_t` thread of
 *   control. Keep it short and non-blocking.
 * - The current implementation uses local callback registries keyed by the
 *   executor-owned request/response message storage. This avoids global
 *   singleton state in the transport layer while remaining compatible with the
 *   C callback signatures provided by `rclc`.
 *
 * References
 * ----------
 * - ROS 2 services design: https://design.ros2.org/articles/ros_on_dds.html
 * - rclc service/client APIs: https://docs.ros.org/en/humble/p/rclc/
 */

#include "common.hpp"
#include "context.hpp"

#include "rosette/support/inplace_function.hpp"

namespace rosette {

template<typename ServiceT>
class Service {
public:
    using traits = ::rosette::service_traits<ServiceT>;
    using Request = typename traits::Request;
    using Response = typename traits::Response;

    /// Type-erased request handler: `handler(const Request&, Response&)`.
    using Handler = InplaceFunction<void(const Request&, Response&), 24u>;

    Service() noexcept = default;

    /**
     * @brief Constructs a typed micro-ROS service wrapper.
     *
     * The wrapper owns the `rcl_service_t` handle and the executor-owned
     * request/response storage associated with that service endpoint. Incoming
     * requests are delivered into `request_`; the callback registry maps that
     * storage address back to this object so the user-supplied C++ handler can
     * be invoked from the plain-C `rclc` callback.
     *
     * @param[in] node Initialized node that will own the service endpoint.
     * @param[in] context Running micro-ROS context whose executor dispatches the service.
     * @param[in] service_name ROS service name relative to the node namespace.
     * @param[in] handler Callable invoked as `handler(const Request&, Response&)`;
     * captures must fit `Handler`'s inline buffer.
     * @param[in] options QoS and session-timeout settings applied to the service.
     */
    template<typename F>
    Service(rcl_node_t* node,
            Context* context,
            const char* service_name,
            F&& handler,
            const ServiceOptions& options) noexcept
        : node_(node)
        , context_(context)
        , handler_(static_cast<F&&>(handler))
    {
        static_assert(has_micro_ros_service_traits_v<ServiceT>,
                      "service_traits<ServiceT>::rosidl_type_support() is required for micro-ROS services");

        if (node_ == nullptr || context_ == nullptr) {
            return;
        }

        init_request<ServiceT>(request_);
        init_response<ServiceT>(response_);

        service_ = rcl_get_zero_initialized_service();
        const rmw_qos_profile_t qos = make_qos_profile(options);
        if (!ros_ok(rclc_service_init(
                &service_,
                node_,
                traits::rosidl_type_support(),
                service_name,
                &qos))) {
            fini_request<ServiceT>(request_);
            fini_response<ServiceT>(response_);
            service_ = rcl_get_zero_initialized_service();
            return;
        }

        if (options.session_timeout_ms > 0) {
            rmw_service_t* rmw_service = rcl_service_get_rmw_handle(&service_);
            if (rmw_service != nullptr) {
                (void)ros_ok(rmw_uros_set_service_session_timeout(
                    rmw_service, options.session_timeout_ms));
            }
        }

        register_instance();
        if (!ros_ok(rclc_executor_add_service_with_request_id(
                &context_->executor(),
                &service_,
                &request_,
                &response_,
                &Service::on_request))) {
            unregister_instance();
            (void)ros_ok(rcl_service_fini(&service_, node_));
            fini_request<ServiceT>(request_);
            fini_response<ServiceT>(response_);
            service_ = rcl_get_zero_initialized_service();
            return;
        }

        valid_ = true;
    }

    Service(const Service&) = delete;
    Service& operator=(const Service&) = delete;
    Service(Service&&) = delete;
    Service& operator=(Service&&) = delete;

    ~Service()
    {
        fini();
    }

    bool valid() const noexcept { return valid_; }

private:
    /**
     * @brief Dispatches a service request from the executor to the matching wrapper.
     *
     * `rclc_executor_add_service_with_request_id()` exposes a pure C callback
     * without user context. The executor always passes the address of the
     * registered request storage, so the wrapper can recover `this` by looking
     * up the request pointer in a small intrusive registry for this template
     * instantiation. The response object is filled in place and returned
     * through the normal `rclc` service response path.
     *
     * @param[in] request Pointer to the executor-owned request storage.
     * @param[in] request_id Correlation header for the request.
     * @param[out] response Pointer to the executor-owned response storage.
     */
    static void on_request(const void* request,
                           rmw_request_id_t* request_id,
                           void* response) noexcept
    {
        (void)request_id;
        if (request == nullptr || response == nullptr) {
            return;
        }

        Service* self = find_by_request(request);
        if (self == nullptr) {
            return;
        }

        self->handler_(*static_cast<const Request*>(request),
                       *static_cast<Response*>(response));
    }

    /**
     * @brief Returns the wrapper whose registered request storage matches a callback pointer.
     *
     * The registry is intentionally per-template-instantiation, which keeps the
     * lookup logic simple and avoids type erasure across unrelated service
     * types. The executor uses stable request-storage addresses, so pointer
     * identity is a safe key while the wrapper remains alive.
     *
     * @param[in] request Pointer supplied by the `rclc` executor callback.
     * @return Matching wrapper instance, or `nullptr` if none is registered.
     */
    static Service* find_by_request(const void* request) noexcept
    {
        Service* current = instance_head_;
        while (current != nullptr) {
            if (request == &current->request_) {
                return current;
            }
            current = current->next_instance_;
        }
        return nullptr;
    }

    /**
     * @brief Adds this wrapper to the callback registry for its template specialization.
     *
     * The registry exists solely to bridge the gap between the context-free C
     * callback signature used by `rclc` services and the stateful C++ wrapper
     * API exposed by Rosette.
     */
    void register_instance() noexcept
    {
        next_instance_ = instance_head_;
        instance_head_ = this;
    }

    /**
     * @brief Removes this wrapper from the callback registry.
     *
     * Removal preserves the relative order of the remaining nodes and leaves
     * the registry in a valid state even when destruction happens out of order.
     */
    void unregister_instance() noexcept
    {
        Service** cursor = &instance_head_;
        while (*cursor != nullptr) {
            if (*cursor == this) {
                *cursor = next_instance_;
                next_instance_ = nullptr;
                return;
            }
            cursor = &((*cursor)->next_instance_);
        }
    }

    /**
     * @brief Releases executor and service resources owned by this wrapper.
     *
     * The request and response messages are finalized regardless of whether the
     * `rcl_service_t` handle successfully initialized, which keeps generated C
     * message lifetimes balanced when traits provide custom init/fini hooks.
     */
    void fini() noexcept
    {
        unregister_instance();
        if (valid_) {
            (void)ros_ok(rclc_executor_remove_service(&context_->executor(), &service_));
            (void)ros_ok(rcl_service_fini(&service_, node_));
        }
        fini_request<ServiceT>(request_);
        fini_response<ServiceT>(response_);
        service_ = rcl_get_zero_initialized_service();
        valid_ = false;
    }

    static Service* instance_head_;

    rcl_service_t service_ = rcl_get_zero_initialized_service();
    rcl_node_t* node_ = nullptr;
    Context* context_ = nullptr;
    Request request_{};
    Response response_{};
    Handler handler_{};
    Service* next_instance_ = nullptr;
    bool valid_ = false;
};

template<typename ServiceT>
Service<ServiceT>* Service<ServiceT>::instance_head_ = nullptr;

template<typename ServiceT, uint8_t MaxPending = 4>
class Client {
public:
    using traits = ::rosette::service_traits<ServiceT>;
    using Request = typename traits::Request;
    using Response = typename traits::Response;
    using ResponseCallback = void(*)(const Response&, void*) noexcept;

    Client() noexcept = default;

    /**
     * @brief Constructs a typed micro-ROS service client wrapper.
     *
     * The client owns an `rcl_client_t` and a single executor-owned response
     * message instance. Completed requests are matched back to the wrapper
     * through that response storage and then correlated to user callbacks by
     * sequence number.
     *
     * @param[in] node Initialized node that will own the client handle.
     * @param[in] context Running micro-ROS context whose executor dispatches replies.
     * @param[in] service_name ROS service name relative to the node namespace.
     * @param[in] options QoS and session-timeout settings applied to the client.
     */
    Client(rcl_node_t* node,
           Context* context,
           const char* service_name,
           const ServiceOptions& options) noexcept
        : node_(node)
        , context_(context)
    {
        static_assert(has_micro_ros_service_traits_v<ServiceT>,
                      "service_traits<ServiceT>::rosidl_type_support() is required for micro-ROS clients");
        static_assert(MaxPending >= 1u, "MaxPending must be at least 1");

        if (node_ == nullptr || context_ == nullptr) {
            return;
        }

        init_response<ServiceT>(response_);

        client_ = rcl_get_zero_initialized_client();
        const rmw_qos_profile_t qos = make_qos_profile(options);
        if (!ros_ok(rclc_client_init(
                &client_,
                node_,
                traits::rosidl_type_support(),
                service_name,
                &qos))) {
            fini_response<ServiceT>(response_);
            client_ = rcl_get_zero_initialized_client();
            return;
        }

        if (options.session_timeout_ms > 0) {
            rmw_client_t* rmw_client = rcl_client_get_rmw_handle(&client_);
            if (rmw_client != nullptr) {
                (void)ros_ok(rmw_uros_set_client_session_timeout(
                    rmw_client, options.session_timeout_ms));
            }
        }

        register_instance();
        if (!ros_ok(rclc_executor_add_client_with_request_id(
                &context_->executor(),
                &client_,
                &response_,
                &Client::on_response))) {
            unregister_instance();
            (void)ros_ok(rcl_client_fini(&client_, node_));
            fini_response<ServiceT>(response_);
            client_ = rcl_get_zero_initialized_client();
            return;
        }

        valid_ = true;
    }

    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;
    Client(Client&&) = delete;
    Client& operator=(Client&&) = delete;

    ~Client()
    {
        fini();
    }

    bool valid() const noexcept { return valid_; }

    /**
     * @brief Sends a request asynchronously and registers an optional completion callback.
     *
     * The micro-ROS client API returns a sequence number for each request. This
     * wrapper stores that sequence number in a fixed-size pending table so the
     * eventual response callback can be correlated without dynamic allocation.
     *
     * @param[in] request Request object to send.
     * @param[in] callback Optional completion callback invoked on response receipt.
     * @param[in] user_data Opaque pointer forwarded to the callback.
     * @return `true` if the request was queued successfully; otherwise `false`.
     */
    bool async_send_request(const Request& request,
                            ResponseCallback callback = nullptr,
                            void* user_data = nullptr) noexcept
    {
        if (!valid_) {
            return false;
        }

        for (uint8_t i = 0u; i < MaxPending; ++i) {
            if (!pending_[i].in_use) {
                int64_t sequence_number = 0;
                if (!ros_ok(rcl_send_request(&client_, &request, &sequence_number))) {
                    return false;
                }
                pending_[i].in_use = true;
                pending_[i].sequence_number = sequence_number;
                pending_[i].callback = callback;
                pending_[i].user_data = user_data;
                return true;
            }
        }
        return false;
    }

private:
    struct PendingRequest {
        bool in_use = false;
        int64_t sequence_number = 0;
        ResponseCallback callback = nullptr;
        void* user_data = nullptr;
    };

    /**
     * @brief Dispatches a client response from the executor to the matching pending request.
     *
     * The wrapper first identifies the owning client instance by response
     * storage address, then scans the fixed-size pending table for the returned
     * request sequence number. Matching callbacks are invoked exactly once and
     * the slot is released for reuse.
     *
     * @param[in] response Pointer to the executor-owned response storage.
     * @param[in] request_id RMW request header containing the correlation sequence number.
     */
    static void on_response(const void* response,
                            rmw_request_id_t* request_id) noexcept
    {
        if (response == nullptr || request_id == nullptr) {
            return;
        }

        Client* self = find_by_response(response);
        if (self == nullptr) {
            return;
        }

        for (uint8_t i = 0u; i < MaxPending; ++i) {
            PendingRequest& pending = self->pending_[i];
            if (!pending.in_use || pending.sequence_number != request_id->sequence_number) {
                continue;
            }

            if (pending.callback != nullptr) {
                pending.callback(*static_cast<const Response*>(response), pending.user_data);
            }

            pending = PendingRequest{};
            return;
        }
    }

    /**
     * @brief Returns the wrapper whose registered response storage matches a callback pointer.
     *
     * The response buffer is stable for the lifetime of the wrapper, which
     * makes pointer identity a safe and allocation-free lookup key.
     *
     * @param[in] response Pointer supplied by the `rclc` executor callback.
     * @return Matching wrapper instance, or `nullptr` if none is registered.
     */
    static Client* find_by_response(const void* response) noexcept
    {
        Client* current = instance_head_;
        while (current != nullptr) {
            if (response == &current->response_) {
                return current;
            }
            current = current->next_instance_;
        }
        return nullptr;
    }

    /**
     * @brief Adds this wrapper to the client callback registry for its template specialization.
     */
    void register_instance() noexcept
    {
        next_instance_ = instance_head_;
        instance_head_ = this;
    }

    /**
     * @brief Removes this wrapper from the client callback registry.
     */
    void unregister_instance() noexcept
    {
        Client** cursor = &instance_head_;
        while (*cursor != nullptr) {
            if (*cursor == this) {
                *cursor = next_instance_;
                next_instance_ = nullptr;
                return;
            }
            cursor = &((*cursor)->next_instance_);
        }
    }

    /**
     * @brief Releases executor and client resources owned by this wrapper.
     *
     * Pending callbacks are discarded because the underlying XRCE session is no
     * longer able to deliver their responses once the client handle is finalized.
     */
    void fini() noexcept
    {
        unregister_instance();
        if (valid_) {
            (void)ros_ok(rclc_executor_remove_client(&context_->executor(), &client_));
            (void)ros_ok(rcl_client_fini(&client_, node_));
        }
        fini_response<ServiceT>(response_);
        client_ = rcl_get_zero_initialized_client();
        for (uint8_t i = 0u; i < MaxPending; ++i) {
            pending_[i] = PendingRequest{};
        }
        valid_ = false;
    }

    static Client* instance_head_;

    rcl_client_t client_ = rcl_get_zero_initialized_client();
    rcl_node_t* node_ = nullptr;
    Context* context_ = nullptr;
    Response response_{};
    PendingRequest pending_[MaxPending] = {};
    Client* next_instance_ = nullptr;
    bool valid_ = false;
};

template<typename ServiceT, uint8_t MaxPending>
Client<ServiceT, MaxPending>* Client<ServiceT, MaxPending>::instance_head_ = nullptr;

}  // namespace rosette
