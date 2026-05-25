#pragma once

/**
 * @file ros/micro_ros/action.hpp
 * @brief micro-ROS action server and client adapters for the Rosette ROS API.
 *
 * Design goal
 * -----------
 * This module gives Rosette applications a C++ action surface that feels like
 * the in-tree RTPS backend while delegating the ROS 2 action protocol to
 * upstream `rcl_action` + `rclc`. The wrapper keeps goal acceptance,
 * cooperative execution, feedback publication, cancellation, and result
 * delivery inside the familiar `Context`/`Node` programming model.
 *
 * Example
 * -------
 *   auto server = node.create_action_server<Fibonacci>(
 *       "fibonacci",
 *       [](const milkros::micro_ros::GoalId& goal_id,
 *          const FibonacciGoal& goal) noexcept {
 *           (void)goal_id;
 *           return goal.order <= 32
 *               ? milkros::micro_ros::GoalResponse::ACCEPT
 *               : milkros::micro_ros::GoalResponse::REJECT;
 *       },
 *       [](milkros::micro_ros::GoalHandle<Fibonacci>& handle) noexcept {
 *           FibonacciFeedback feedback{};
 *           handle.publish_feedback(feedback);
 *           FibonacciResult result{};
 *           handle.succeed(result);
 *       });
 *
 *   auto client = node.create_action_client<Fibonacci>("fibonacci");
 *   FibonacciGoal goal{};
 *   milkros::micro_ros::GoalId goal_id{};
 *   client.send_goal(
 *       goal,
 *       goal_id,
 *       [](const milkros::micro_ros::GoalId&, bool accepted, void*) noexcept {
 *           (void)accepted;
 *       },
 *       [](const milkros::micro_ros::GoalId&,
 *          const FibonacciFeedback&,
 *          void*) noexcept {},
 *       [](const milkros::micro_ros::GoalId&,
 *          milkros::micro_ros::GoalStatus status,
 *          const FibonacciResult&,
 *          void*) noexcept {
 *           (void)status;
 *       });
 *
 * Features
 * --------
 * - Goal acceptance and cancellation callbacks routed through upstream `rclc`.
 * - Cooperative server execution via `Context::tick()`, matching Rosette's MCU
 *   execution model rather than spawning extra threads.
 * - Typed feedback and result callbacks on the client side.
 * - Compatibility shim for `request_result()` so application code can stay
 *   close to the RTPS backend even though `rclc` handles result requests for us.
 *
 * Caveats
 * -------
 * - This wrapper currently supports one active goal per action server
 *   instance. Additional goals are rejected while a goal is executing or
 *   waiting to flush its terminal result.
 * - Upstream `rclc` action APIs provide add-to-executor functions but no
 *   remove-from-executor counterpart. Action endpoints should therefore live at
 *   least as long as the `Context` executor that dispatches them.
 * - `action_traits<ActionT>` must expose the generated ROS action support types
 *   expected by this wrapper, as documented below.
 *
 * Required action_traits<ActionT> members
 * ---------------------------------------
 *   using Goal = <generated action goal struct>;
 *   using Result = <generated action result struct>;
 *   using Feedback = <generated action feedback struct>;
 *   using SendGoalRequest = <generated *_SendGoal_Request struct>;
 *   using GetResultResponse = <generated *_GetResult_Response struct>;
 *   using FeedbackMessage = <generated *_FeedbackMessage struct>;
 *   static const rosidl_action_type_support_t* rosidl_action_type_support() noexcept;
 *
 * Optional helpers:
 *   static void init_send_goal_request(SendGoalRequest&) noexcept;
 *   static void fini_send_goal_request(SendGoalRequest&) noexcept;
 *   static void init_result_response(GetResultResponse&) noexcept;
 *   static void fini_result_response(GetResultResponse&) noexcept;
 *   static void init_feedback_message(FeedbackMessage&) noexcept;
 *   static void fini_feedback_message(FeedbackMessage&) noexcept;
 *
 * References
 * ----------
 * - ROS 2 actions design: https://design.ros2.org/articles/actions.html
 * - rclc action APIs: https://docs.ros.org/en/humble/p/rclc/
 */

#include "common.hpp"
#include "context.hpp"

#if !ROSETTE_MICRO_ROS_HAS_ACTION_API
#error "Rosette micro-ROS action support requires rclc/action_client.h and rclc/action_server.h."
#endif

namespace milkros::micro_ros {

template<typename ActionT>
class GoalHandle {
public:
    using traits = ::milkros::action_traits<ActionT>;
    using Goal = typename traits::Goal;
    using Result = typename traits::Result;
    using Feedback = typename traits::Feedback;
    using GetResultResponse = typename traits::GetResultResponse;
    using FeedbackMessage = typename traits::FeedbackMessage;

    GoalHandle() noexcept = default;

    GoalStatus status() const noexcept
    {
        if (goal_handle_ == nullptr) {
            return GoalStatus::UNKNOWN;
        }
        return to_goal_status(goal_handle_->status);
    }

    const GoalId& goal_id() const noexcept
    {
        return goal_id_;
    }

    const Goal& goal() const noexcept
    {
        return *goal_;
    }

    bool is_active() const noexcept
    {
        const GoalStatus current = status();
        return current == GoalStatus::ACCEPTED || current == GoalStatus::EXECUTING;
    }

    bool is_canceling() const noexcept
    {
        return status() == GoalStatus::CANCELING;
    }

    /**
     * @brief Publishes an action feedback sample for the active goal.
     *
     * The wrapper reuses a preallocated generated `FeedbackMessage` instance so
     * the MCU does not need to allocate on the heap while publishing feedback.
     * The goal UUID is copied into the envelope on every publish so the action
     * client can correlate feedback even after multiple goals have passed
     * through the same action server.
     *
     * @param[in] feedback Generated ROS action feedback payload.
     */
    void publish_feedback(const Feedback& feedback) noexcept
    {
        if (goal_handle_ == nullptr || feedback_message_ == nullptr) {
            return;
        }

        copy_goal_id(goal_id_, feedback_message_->goal_id);
        feedback_message_->feedback = feedback;
        (void)rclc_action_publish_feedback(goal_handle_, feedback_message_);
    }

    /**
     * @brief Completes the goal successfully and stages the result for delivery.
     *
     * The actual transport-level result send happens in `ActionServer::tick()`
     * so the wrapper can retry the upstream `rclc_action_send_result()` call if
     * the client has not yet issued a result request.
     *
     * @param[in] result Generated ROS action result payload.
     */
    void succeed(const Result& result) noexcept
    {
        terminal(GoalStatus::SUCCEEDED, result);
    }

    /**
     * @brief Completes the goal with an aborted terminal state.
     *
     * @param[in] result Generated ROS action result payload.
     */
    void abort(const Result& result) noexcept
    {
        terminal(GoalStatus::ABORTED, result);
    }

    /**
     * @brief Completes a canceling goal with a canceled terminal state.
     *
     * @param[in] result Generated ROS action result payload.
     */
    void cancel(const Result& result) noexcept
    {
        terminal(GoalStatus::CANCELED, result);
    }

private:
    template<typename, typename, typename> friend class ActionServer;

    /**
     * @brief Wires this facade to an upstream `rclc_action_goal_handle_t`.
     *
     * The server binds a handle once a goal request is accepted. From that
     * point on the facade offers safe access to goal metadata, feedback
     * publication, and terminal transitions without exposing the raw `rclc`
     * structures to application code.
     *
     * @param[in] goal_handle Upstream goal handle for the active goal.
     * @param[in] goal_id Cached UUID copied from the request envelope.
     * @param[in] goal Pointer to the accepted goal payload.
     * @param[in,out] feedback_message Preallocated generated feedback envelope.
     * @param[in,out] result_response Preallocated generated result response envelope.
     * @param[in,out] terminal_status Storage for the staged terminal state.
     * @param[in,out] terminal_pending Flag set when a result is waiting to flush.
     */
    void bind(rclc_action_goal_handle_t* goal_handle,
              const GoalId& goal_id,
              const Goal* goal,
              FeedbackMessage* feedback_message,
              GetResultResponse* result_response,
              GoalStatus* terminal_status,
              bool* terminal_pending) noexcept
    {
        goal_handle_ = goal_handle;
        goal_id_ = goal_id;
        goal_ = goal;
        feedback_message_ = feedback_message;
        result_response_ = result_response;
        terminal_status_ = terminal_status;
        terminal_pending_ = terminal_pending;
    }

    /**
     * @brief Detaches this facade from the upstream goal handle after completion.
     *
     * Clearing the stored pointers ensures stale application references cannot
     * accidentally publish feedback or overwrite a completed result.
     */
    void unbind() noexcept
    {
        goal_handle_ = nullptr;
        goal_ = nullptr;
        feedback_message_ = nullptr;
        result_response_ = nullptr;
        terminal_status_ = nullptr;
        terminal_pending_ = nullptr;
        goal_id_ = GoalId{};
    }

    /**
     * @brief Records a terminal transition and copies the result payload into the staged response.
     *
     * `rclc_action_send_result()` may need to wait until the client requests
     * the result. This helper therefore records the desired terminal state and
     * response payload instead of sending immediately, allowing the server tick
     * loop to retry at a safe cadence.
     *
     * @param[in] status Terminal action state requested by the application.
     * @param[in] result Generated ROS action result payload.
     */
    void terminal(GoalStatus status, const Result& result) noexcept
    {
        if (goal_handle_ == nullptr ||
            result_response_ == nullptr ||
            terminal_status_ == nullptr ||
            terminal_pending_ == nullptr) {
            return;
        }

        result_response_->status = static_cast<int8_t>(status);
        result_response_->result = result;
        goal_handle_->status = to_rcl_goal_status(status);
        *terminal_status_ = status;
        *terminal_pending_ = true;
    }

    rclc_action_goal_handle_t* goal_handle_ = nullptr;
    GoalId goal_id_{};
    const Goal* goal_ = nullptr;
    FeedbackMessage* feedback_message_ = nullptr;
    GetResultResponse* result_response_ = nullptr;
    GoalStatus* terminal_status_ = nullptr;
    bool* terminal_pending_ = nullptr;
};

template<typename ActionT, typename GoalCbT, typename ExecuteCbT>
class ActionServer final : public IActionTickable {
public:
    using traits = ::milkros::action_traits<ActionT>;
    using Goal = typename traits::Goal;
    using Result = typename traits::Result;
    using SendGoalRequest = typename traits::SendGoalRequest;
    using GetResultResponse = typename traits::GetResultResponse;
    using FeedbackMessage = typename traits::FeedbackMessage;

    ActionServer() noexcept = default;

    /**
     * @brief Constructs a typed micro-ROS action server wrapper.
     *
     * The wrapper registers the upstream `rclc_action_server_t` with the shared
     * executor and then drives goal execution cooperatively from
     * `Context::tick()`. This mirrors Rosette's embedded execution style:
     * callbacks remain short, while the actual action work advances in a
     * deterministic application-controlled loop.
     *
     * @param[in] node Initialized node that will own the action server.
     * @param[in] context Running micro-ROS context whose executor dispatches action traffic.
     * @param[in] action_name ROS action name relative to the node namespace.
     * @param[in] goal_callback Goal accept/reject callback.
     * @param[in] execute_callback Cooperative execution callback driven by `Context::tick()`.
     */
    ActionServer(rcl_node_t* node,
                 Context* context,
                 const char* action_name,
                 GoalCbT goal_callback,
                 ExecuteCbT execute_callback) noexcept
        : node_(node)
        , context_(context)
        , goal_callback_(static_cast<GoalCbT&&>(goal_callback))
        , execute_callback_(static_cast<ExecuteCbT&&>(execute_callback))
    {
        static_assert(has_micro_ros_action_traits_v<ActionT>,
                      "action_traits<ActionT>::rosidl_action_type_support() is required for micro-ROS actions");

        if (node_ == nullptr || context_ == nullptr) {
            return;
        }

        init_send_goal_request<ActionT>(send_goal_request_);
        init_result_response<ActionT>(result_response_);
        init_feedback_message<ActionT>(feedback_message_);

        action_server_ = rclc_action_server_t{};
        if (rclc_action_server_init_default(
                &action_server_,
                node_,
                &context_->support(),
                traits::rosidl_action_type_support(),
                action_name) != RCL_RET_OK) {
            fini_send_goal_request<ActionT>(send_goal_request_);
            fini_result_response<ActionT>(result_response_);
            fini_feedback_message<ActionT>(feedback_message_);
            return;
        }

        if (rclc_executor_add_action_server(
                &context_->executor(),
                &action_server_,
                1u,
                &send_goal_request_,
                sizeof(send_goal_request_),
                &ActionServer::on_goal_request,
                &ActionServer::on_cancel_request,
                this) != RCL_RET_OK) {
            (void)rclc_action_server_fini(&action_server_, node_);
            fini_send_goal_request<ActionT>(send_goal_request_);
            fini_result_response<ActionT>(result_response_);
            fini_feedback_message<ActionT>(feedback_message_);
            action_server_ = rclc_action_server_t{};
            return;
        }

        if (!context_->register_action_tickable(this)) {
            (void)rclc_action_server_fini(&action_server_, node_);
            fini_send_goal_request<ActionT>(send_goal_request_);
            fini_result_response<ActionT>(result_response_);
            fini_feedback_message<ActionT>(feedback_message_);
            action_server_ = rclc_action_server_t{};
            return;
        }

        valid_ = true;
    }

    ActionServer(const ActionServer&) = delete;
    ActionServer& operator=(const ActionServer&) = delete;
    ActionServer(ActionServer&&) = delete;
    ActionServer& operator=(ActionServer&&) = delete;

    ~ActionServer()
    {
        fini();
    }

    bool valid() const noexcept { return valid_; }

    /**
     * @brief Advances the active goal and flushes terminal results when possible.
     *
     * The surrounding `Context` calls this after each executor spin. If an
     * active goal is executing or canceling, the user-provided execute callback
     * gets a chance to advance the state machine. Once the callback marks the
     * goal terminal, the wrapper repeatedly retries `rclc_action_send_result()`
     * until the result has been accepted by the upstream action machinery.
     */
    void tick() noexcept override
    {
        if (!valid_ || active_goal_handle_ == nullptr) {
            return;
        }

        const GoalStatus current_status = to_goal_status(active_goal_handle_->status);
        if (!terminal_result_pending_ &&
            (current_status == GoalStatus::ACCEPTED ||
             current_status == GoalStatus::EXECUTING ||
             current_status == GoalStatus::CANCELING)) {
            execute_callback_(goal_handle_);
        }

        if (terminal_result_pending_) {
            flush_result();
        }
    }

private:
    /**
     * @brief Handles an incoming goal request from the upstream `rclc` action server.
     *
     * The callback converts the generated send-goal request into the
     * application-facing `GoalId` and `Goal` objects, enforces the single-active-goal
     * policy, and records the accepted upstream goal handle for cooperative
     * execution. Rejection is reported through the `goal_accepted` flag exposed
     * by `rclc_action_goal_handle_t`.
     *
     * @param[in,out] goal_handle Upstream goal handle allocated by `rclc`.
     * @param[in] args Wrapper instance passed through the executor context.
     * @return `RCL_RET_OK` after updating the upstream acceptance flags.
     */
    static rcl_ret_t on_goal_request(rclc_action_goal_handle_t* goal_handle,
                                     void* args) noexcept
    {
        if (goal_handle == nullptr || args == nullptr) {
            return RCL_RET_INVALID_ARGUMENT;
        }

        auto* self = static_cast<ActionServer*>(args);
        auto* request = static_cast<SendGoalRequest*>(goal_handle->ros_goal_request);
        if (request == nullptr) {
            return RCL_RET_ERROR;
        }

        const bool busy = self->active_goal_handle_ != nullptr;
        if (busy) {
            goal_handle->goal_accepted = false;
            goal_handle->status = GOAL_STATE_UNKNOWN;
            return RCL_RET_OK;
        }

        const GoalId goal_id = to_goal_id(request->goal_id);
        const GoalResponse response = self->goal_callback_(goal_id, request->goal);
        const bool accepted = response == GoalResponse::ACCEPT;
        goal_handle->goal_accepted = accepted;
        goal_handle->goal_cancelled = false;
        goal_handle->status = accepted
                              ? static_cast<decltype(goal_handle->status)>(GOAL_STATE_EXECUTING)
                              : static_cast<decltype(goal_handle->status)>(GOAL_STATE_UNKNOWN);

        if (!accepted) {
            return RCL_RET_OK;
        }

        self->active_goal_handle_ = goal_handle;
        self->active_goal_id_ = goal_id;
        self->active_goal_ = request->goal;
        self->terminal_status_ = GoalStatus::UNKNOWN;
        self->terminal_result_pending_ = false;
        self->goal_handle_.bind(goal_handle,
                                self->active_goal_id_,
                                &self->active_goal_,
                                &self->feedback_message_,
                                &self->result_response_,
                                &self->terminal_status_,
                                &self->terminal_result_pending_);
        return RCL_RET_OK;
    }

    /**
     * @brief Handles an incoming cancel request for the active goal.
     *
     * The server accepts cancellation only for the currently active goal.
     * Accepted requests transition the upstream goal handle to the
     * `CANCELING` state so the cooperative execute callback can finish the goal
     * with `GoalHandle::cancel()`.
     *
     * @param[in,out] goal_handle Upstream goal handle targeted by the cancel request.
     * @param[in] args Wrapper instance passed through the executor context.
     * @return `true` when the cancel request is accepted; otherwise `false`.
     */
    static bool on_cancel_request(rclc_action_goal_handle_t* goal_handle,
                                  void* args) noexcept
    {
        if (goal_handle == nullptr || args == nullptr) {
            return false;
        }

        auto* self = static_cast<ActionServer*>(args);
        if (self->active_goal_handle_ != goal_handle) {
            return false;
        }

        goal_handle->status = GOAL_STATE_CANCELING;
        return true;
    }

    /**
     * @brief Attempts to flush a staged terminal result through upstream `rclc`.
     *
     * `rclc_action_send_result()` may report
     * `RCLC_RET_ACTION_WAIT_RESULT_REQUEST` until the client has issued its
     * result request. The wrapper preserves the staged result and retries on the
     * next `Context::tick()`. Once the send succeeds, the active goal is
     * released and the facade is reset for the next accepted request.
     */
    void flush_result() noexcept
    {
        if (active_goal_handle_ == nullptr) {
            terminal_result_pending_ = false;
            return;
        }

        const rcl_ret_t rc = rclc_action_send_result(
            active_goal_handle_,
            to_rcl_goal_status(terminal_status_),
            &result_response_);
        if (rc == RCL_RET_OK) {
            goal_handle_.unbind();
            active_goal_handle_ = nullptr;
            active_goal_id_ = GoalId{};
            active_goal_ = Goal{};
            terminal_status_ = GoalStatus::UNKNOWN;
            terminal_result_pending_ = false;
        }
    }

    /**
     * @brief Releases action server resources owned by this wrapper.
     *
     * The wrapper unregisters itself from the context tick list and finalizes
     * the generated message storage used by the upstream `rclc` action server.
     * Because `rclc` does not currently expose executor removal APIs for action
     * entities, applications should keep action wrappers alive until the owning
     * context is torn down.
     */
    void fini() noexcept
    {
        if (context_ != nullptr) {
            context_->unregister_action_tickable(this);
        }
        if (valid_) {
            (void)rclc_action_server_fini(&action_server_, node_);
        }
        goal_handle_.unbind();
        fini_send_goal_request<ActionT>(send_goal_request_);
        fini_result_response<ActionT>(result_response_);
        fini_feedback_message<ActionT>(feedback_message_);
        action_server_ = rclc_action_server_t{};
        active_goal_handle_ = nullptr;
        terminal_result_pending_ = false;
        valid_ = false;
    }

    rclc_action_server_t action_server_{};
    rcl_node_t* node_ = nullptr;
    Context* context_ = nullptr;
    SendGoalRequest send_goal_request_{};
    GetResultResponse result_response_{};
    FeedbackMessage feedback_message_{};
    GoalCbT goal_callback_{};
    ExecuteCbT execute_callback_{};
    rclc_action_goal_handle_t* active_goal_handle_ = nullptr;
    GoalHandle<ActionT> goal_handle_{};
    GoalId active_goal_id_{};
    Goal active_goal_{};
    GoalStatus terminal_status_ = GoalStatus::UNKNOWN;
    bool terminal_result_pending_ = false;
    bool valid_ = false;
};

template<typename ActionT, uint8_t MaxTrackedGoals = 4>
class ActionClient final {
public:
    using traits = ::milkros::action_traits<ActionT>;
    using Goal = typename traits::Goal;
    using Result = typename traits::Result;
    using Feedback = typename traits::Feedback;
    using SendGoalRequest = typename traits::SendGoalRequest;
    using GetResultResponse = typename traits::GetResultResponse;
    using FeedbackMessage = typename traits::FeedbackMessage;

    using GoalRespCb = void(*)(const GoalId&, bool, void*) noexcept;
    using FeedbackCb = void(*)(const GoalId&, const Feedback&, void*) noexcept;
    using ResultCb = void(*)(const GoalId&, GoalStatus, const Result&, void*) noexcept;

    ActionClient() noexcept = default;

    /**
     * @brief Constructs a typed micro-ROS action client wrapper.
     *
     * The wrapper registers upstream action callbacks with the shared executor
     * and tracks a fixed number of outstanding goals using the goal-handle
     * pointers assigned by `rclc_action_send_goal_request()`.
     *
     * @param[in] node Initialized node that will own the action client.
     * @param[in] context Running micro-ROS context whose executor dispatches action replies.
     * @param[in] action_name ROS action name relative to the node namespace.
     */
    ActionClient(rcl_node_t* node,
                 Context* context,
                 const char* action_name) noexcept
        : node_(node)
        , context_(context)
    {
        static_assert(has_micro_ros_action_traits_v<ActionT>,
                      "action_traits<ActionT>::rosidl_action_type_support() is required for micro-ROS actions");
        static_assert(MaxTrackedGoals >= 1u, "MaxTrackedGoals must be at least 1");

        if (node_ == nullptr || context_ == nullptr) {
            return;
        }

        init_send_goal_request<ActionT>(send_goal_request_);
        init_result_response<ActionT>(result_response_);
        init_feedback_message<ActionT>(feedback_message_);

        action_client_ = rclc_action_client_t{};
        if (rclc_action_client_init_default(
                &action_client_,
                node_,
                traits::rosidl_action_type_support(),
                action_name) != RCL_RET_OK) {
            fini_send_goal_request<ActionT>(send_goal_request_);
            fini_result_response<ActionT>(result_response_);
            fini_feedback_message<ActionT>(feedback_message_);
            return;
        }

        if (rclc_executor_add_action_client(
                &context_->executor(),
                &action_client_,
                MaxTrackedGoals,
                &result_response_,
                &feedback_message_,
                &ActionClient::on_goal_response,
                &ActionClient::on_feedback,
                &ActionClient::on_result,
                &ActionClient::on_cancel_response,
                this) != RCL_RET_OK) {
            (void)rclc_action_client_fini(&action_client_, node_);
            fini_send_goal_request<ActionT>(send_goal_request_);
            fini_result_response<ActionT>(result_response_);
            fini_feedback_message<ActionT>(feedback_message_);
            action_client_ = rclc_action_client_t{};
            return;
        }

        valid_ = true;
    }

    ActionClient(const ActionClient&) = delete;
    ActionClient& operator=(const ActionClient&) = delete;
    ActionClient(ActionClient&&) = delete;
    ActionClient& operator=(ActionClient&&) = delete;

    ~ActionClient()
    {
        fini();
    }

    bool valid() const noexcept { return valid_; }

    /**
     * @brief Sends an action goal and registers callbacks for the goal lifecycle.
     *
     * The wrapper synthesizes a UUID using the micro-ROS client key plus a
     * local monotonic sequence so callers do not need to construct goal IDs by
     * hand on constrained targets.
     *
     * @param[in] goal Generated ROS action goal payload.
     * @param[out] goal_id_out Receives the UUID used for the request.
     * @param[in] goal_response_callback Optional callback for accept/reject notifications.
     * @param[in] feedback_callback Optional callback for feedback samples.
     * @param[in] result_callback Optional callback for terminal results.
     * @param[in] user_data Opaque pointer forwarded to all callbacks.
     * @return `true` if the goal request was accepted for transmission; otherwise `false`.
     */
    bool send_goal(const Goal& goal,
                   GoalId& goal_id_out,
                   GoalRespCb goal_response_callback = nullptr,
                   FeedbackCb feedback_callback = nullptr,
                   ResultCb result_callback = nullptr,
                   void* user_data = nullptr) noexcept
    {
        if (!valid_) {
            return false;
        }

        TrackedGoal* slot = reserve_slot();
        if (slot == nullptr) {
            return false;
        }

        const GoalId goal_id = generate_goal_id();
        send_goal_request_.goal = goal;
        copy_goal_id(goal_id, send_goal_request_.goal_id);

        rclc_action_goal_handle_t* goal_handle = nullptr;
        const rcl_ret_t rc =
            rclc_action_send_goal_request(&action_client_, &send_goal_request_, &goal_handle);
        if (rc != RCL_RET_OK || goal_handle == nullptr) {
            *slot = TrackedGoal{};
            return false;
        }

        slot->goal_handle = goal_handle;
        slot->goal_id = goal_id;
        slot->goal_response_callback = goal_response_callback;
        slot->feedback_callback = feedback_callback;
        slot->result_callback = result_callback;
        slot->user_data = user_data;
        slot->in_use = true;

        goal_id_out = goal_id;
        return true;
    }

    /**
     * @brief Requests goal cancellation for a previously sent goal.
     *
     * The wrapper looks up the tracked goal by UUID and delegates the cancel
     * request to upstream `rclc`.
     *
     * @param[in] goal_id UUID returned from `send_goal()`.
     * @return `true` if the cancel request was handed to `rclc`; otherwise `false`.
     */
    bool cancel_goal(const GoalId& goal_id) noexcept
    {
        TrackedGoal* slot = find_by_goal_id(goal_id);
        if (slot == nullptr || slot->goal_handle == nullptr) {
            return false;
        }
        return rclc_action_send_cancel_request(slot->goal_handle) == RCL_RET_OK;
    }

    /**
     * @brief Compatibility shim for the RTPS backend's explicit result-request API.
     *
     * Upstream `rclc` manages action result requests internally after a goal is
     * accepted, so no separate wire operation is needed here. The method
     * returns whether the goal is still tracked, which preserves a similar call
     * site to the RTPS backend without misleading the application about the
     * underlying transport behavior.
     *
     * @param[in] goal_id UUID returned from `send_goal()`.
     * @return `true` if the goal is currently tracked; otherwise `false`.
     */
    bool request_result(const GoalId& goal_id) noexcept
    {
        return find_by_goal_id(goal_id) != nullptr;
    }

private:
    struct TrackedGoal {
        rclc_action_goal_handle_t* goal_handle = nullptr;
        GoalId goal_id{};
        GoalRespCb goal_response_callback = nullptr;
        FeedbackCb feedback_callback = nullptr;
        ResultCb result_callback = nullptr;
        void* user_data = nullptr;
        bool in_use = false;
    };

    /**
     * @brief Handles an upstream goal response callback for a tracked goal.
     *
     * The callback resolves the corresponding tracked goal slot by upstream
     * goal-handle pointer and forwards the acceptance outcome to the user
     * callback. Rejected goals are released immediately because no further
     * feedback or result traffic will arrive for them.
     *
     * @param[in] goal_handle Upstream goal handle referenced by the response.
     * @param[in] accepted Whether the action server accepted the goal.
     * @param[in] args Wrapper instance passed through the executor context.
     */
    static void on_goal_response(rclc_action_goal_handle_t* goal_handle,
                                 bool accepted,
                                 void* args) noexcept
    {
        if (goal_handle == nullptr || args == nullptr) {
            return;
        }

        auto* self = static_cast<ActionClient*>(args);
        TrackedGoal* slot = self->find_by_handle(goal_handle);
        if (slot == nullptr) {
            return;
        }

        if (slot->goal_response_callback != nullptr) {
            slot->goal_response_callback(slot->goal_id, accepted, slot->user_data);
        }

        if (!accepted) {
            *slot = TrackedGoal{};
        }
    }

    /**
     * @brief Handles an upstream feedback callback for a tracked goal.
     *
     * The generated feedback envelope includes both the goal UUID and the typed
     * feedback payload. The wrapper forwards only the payload and cached UUID to
     * the C++ callback so application code sees a tidy, backend-agnostic shape.
     *
     * @param[in] goal_handle Upstream goal handle referenced by the feedback.
     * @param[in] ros_feedback Pointer to the generated feedback envelope.
     * @param[in] args Wrapper instance passed through the executor context.
     */
    static void on_feedback(rclc_action_goal_handle_t* goal_handle,
                            void* ros_feedback,
                            void* args) noexcept
    {
        if (goal_handle == nullptr || ros_feedback == nullptr || args == nullptr) {
            return;
        }

        auto* self = static_cast<ActionClient*>(args);
        TrackedGoal* slot = self->find_by_handle(goal_handle);
        if (slot == nullptr || slot->feedback_callback == nullptr) {
            return;
        }

        const auto* feedback = static_cast<const FeedbackMessage*>(ros_feedback);
        slot->feedback_callback(slot->goal_id, feedback->feedback, slot->user_data);
    }

    /**
     * @brief Handles an upstream result callback for a tracked goal.
     *
     * The wrapper unwraps the generated result response into the portable
     * `(GoalId, GoalStatus, Result)` shape used by Rosette and then releases the
     * tracking slot because the goal lifecycle is complete from the client's
     * perspective.
     *
     * @param[in] goal_handle Upstream goal handle referenced by the result.
     * @param[in] ros_result_response Pointer to the generated result response.
     * @param[in] args Wrapper instance passed through the executor context.
     */
    static void on_result(rclc_action_goal_handle_t* goal_handle,
                          void* ros_result_response,
                          void* args) noexcept
    {
        if (goal_handle == nullptr || ros_result_response == nullptr || args == nullptr) {
            return;
        }

        auto* self = static_cast<ActionClient*>(args);
        TrackedGoal* slot = self->find_by_handle(goal_handle);
        if (slot == nullptr) {
            return;
        }

        if (slot->result_callback != nullptr) {
            const auto* response = static_cast<const GetResultResponse*>(ros_result_response);
            slot->result_callback(slot->goal_id,
                                  static_cast<GoalStatus>(response->status),
                                  response->result,
                                  slot->user_data);
        }

        *slot = TrackedGoal{};
    }

    /**
     * @brief Handles an upstream cancel-response callback for a tracked goal.
     *
     * The current Rosette C++ action facade exposes cancellation through the
     * terminal result callback rather than a dedicated cancel-response callback,
     * so this hook intentionally keeps only the upstream state synchronized.
     *
     * @param[in] goal_handle Upstream goal handle referenced by the cancel response.
     * @param[in] cancelled Whether the cancel request was accepted upstream.
     * @param[in] args Wrapper instance passed through the executor context.
     */
    static void on_cancel_response(rclc_action_goal_handle_t* goal_handle,
                                   bool cancelled,
                                   void* args) noexcept
    {
        if (goal_handle == nullptr || args == nullptr) {
            return;
        }

        auto* self = static_cast<ActionClient*>(args);
        TrackedGoal* slot = self->find_by_handle(goal_handle);
        if (slot == nullptr) {
            return;
        }

        if (cancelled && slot->goal_handle != nullptr) {
            slot->goal_handle->goal_cancelled = true;
        }
    }

    /**
     * @brief Reserves a free tracked-goal slot without allocating.
     *
     * @return Pointer to a free slot, or `nullptr` if the fixed table is full.
     */
    TrackedGoal* reserve_slot() noexcept
    {
        for (uint8_t i = 0u; i < MaxTrackedGoals; ++i) {
            if (!tracked_goals_[i].in_use) {
                return &tracked_goals_[i];
            }
        }
        return nullptr;
    }

    /**
     * @brief Finds a tracked goal by upstream goal-handle pointer.
     *
     * @param[in] goal_handle Upstream `rclc` goal-handle pointer.
     * @return Matching tracked slot, or `nullptr` if none is active.
     */
    TrackedGoal* find_by_handle(rclc_action_goal_handle_t* goal_handle) noexcept
    {
        for (uint8_t i = 0u; i < MaxTrackedGoals; ++i) {
            if (tracked_goals_[i].in_use && tracked_goals_[i].goal_handle == goal_handle) {
                return &tracked_goals_[i];
            }
        }
        return nullptr;
    }

    /**
     * @brief Finds a tracked goal by UUID.
     *
     * @param[in] goal_id Goal UUID returned from `send_goal()`.
     * @return Matching tracked slot, or `nullptr` if none is active.
     */
    TrackedGoal* find_by_goal_id(const GoalId& goal_id) noexcept
    {
        for (uint8_t i = 0u; i < MaxTrackedGoals; ++i) {
            if (tracked_goals_[i].in_use && tracked_goals_[i].goal_id == goal_id) {
                return &tracked_goals_[i];
            }
        }
        return nullptr;
    }

    /**
     * @brief Synthesizes a deterministic goal UUID for a new outbound request.
     *
     * The first four bytes carry the micro-ROS client key so different XRCE
     * clients are unlikely to collide, while the trailing bytes embed a local
     * monotonic sequence number.
     *
     * @return Fresh goal UUID for the next `send_goal()` call.
     */
    GoalId generate_goal_id() noexcept
    {
        GoalId goal_id{};
        const uint32_t client_key = context_->options().client_key;
        goal_id.uuid[0] = static_cast<uint8_t>(client_key >> 24u);
        goal_id.uuid[1] = static_cast<uint8_t>(client_key >> 16u);
        goal_id.uuid[2] = static_cast<uint8_t>(client_key >> 8u);
        goal_id.uuid[3] = static_cast<uint8_t>(client_key);

        const uint32_t sequence = next_goal_sequence_++;
        goal_id.uuid[12] = static_cast<uint8_t>(sequence >> 24u);
        goal_id.uuid[13] = static_cast<uint8_t>(sequence >> 16u);
        goal_id.uuid[14] = static_cast<uint8_t>(sequence >> 8u);
        goal_id.uuid[15] = static_cast<uint8_t>(sequence);
        return goal_id;
    }

    /**
     * @brief Releases action client resources owned by this wrapper.
     *
     * The tracked-goal table is cleared because callbacks can no longer arrive
     * once the underlying action client handle is finalized.
     */
    void fini() noexcept
    {
        if (valid_) {
            (void)rclc_action_client_fini(&action_client_, node_);
        }
        fini_send_goal_request<ActionT>(send_goal_request_);
        fini_result_response<ActionT>(result_response_);
        fini_feedback_message<ActionT>(feedback_message_);
        action_client_ = rclc_action_client_t{};
        for (uint8_t i = 0u; i < MaxTrackedGoals; ++i) {
            tracked_goals_[i] = TrackedGoal{};
        }
        valid_ = false;
    }

    rclc_action_client_t action_client_{};
    rcl_node_t* node_ = nullptr;
    Context* context_ = nullptr;
    SendGoalRequest send_goal_request_{};
    GetResultResponse result_response_{};
    FeedbackMessage feedback_message_{};
    TrackedGoal tracked_goals_[MaxTrackedGoals] = {};
    uint32_t next_goal_sequence_ = 1u;
    bool valid_ = false;
};

}  // namespace milkros::micro_ros
