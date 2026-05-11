/**
 * @file ros/action.hpp
 * @brief ROS 2-compatible embedded action server and client.
 *
 * Analogues
 * ---------
 *   milkros::ActionServer<T,G,E>  ≈  rclcpp_action::Server<T>
 *   milkros::ActionClient<T>      ≈  rclcpp_action::Client<T>
 *   milkros::GoalHandle<T>        ≈  rclcpp_action::ServerGoalHandle<T>
 *   milkros::action_traits<T>     ≈  rosidl_typesupport specialisation
 *
 * Protocol
 * --------
 * Follows the ROS 2 action protocol: 3 service pairs + 2 topics.
 *
 * Topics (server side — writers):
 *   rr/<ns>/<action>/_action/send_goalReply     — accept/reject response
 *   rr/<ns>/<action>/_action/get_resultReply    — result when goal is done
 *   rr/<ns>/<action>/_action/cancel_goalReply   — cancel acknowledgement
 *   rt/<ns>/<action>/_action/feedback            — feedback messages
 *   rt/<ns>/<action>/_action/status              — goal status updates
 *
 * Topics (server side — readers):
 *   rq/<ns>/<action>/_action/send_goalRequest
 *   rq/<ns>/<action>/_action/get_resultRequest
 *   rq/<ns>/<action>/_action/cancel_goalRequest
 *
 * Resource usage per ActionServer:  5 user writers + 3 user readers.
 * Resource usage per ActionClient:  3 user writers + 5 user readers.
 *
 * ⚠ Increase defaults before using actions:
 *   #define RTPS_MAX_USER_WRITERS 8
 *   #define RTPS_MAX_USER_READERS 8
 *
 * Wire payload layouts (embedded in payload, not InlineQoS — see service.hpp):
 *   send_goal request:   RequestId(24) + GoalId(16) + encoded_goal
 *   send_goal response:  RequestId(24) + accepted(1) + stamp_sec(4) + stamp_nsec(4)
 *   get_result request:  RequestId(24) + GoalId(16)
 *   get_result response: RequestId(24) + status(1) + encoded_result
 *   cancel request:      RequestId(24) + GoalId(16) + stamp_sec(4) + stamp_nsec(4)
 *   cancel response:     RequestId(24) + return_code(1) + GoalId(16)
 *   feedback topic:      GoalId(16) + encoded_feedback
 *   status topic:        GoalId(16) + status(1)
 *
 * Execute callback
 * ----------------
 * ExecuteCbT is called on every Context::tick() while the goal is EXECUTING.
 * The handler advances the goal each call, publishing intermediate feedback,
 * and calls handle.succeed() / handle.abort() / handle.cancel() to finish.
 * This is deliberately cooperative (no separate task needed) — suitable for
 * embedded systems.
 *
 * Defining action_traits
 * ----------------------
 *   struct Fibonacci {};
 *
 *   template<> struct milkros::action_traits<Fibonacci> {
 *       using Goal     = FibonacciGoal;
 *       using Result   = FibonacciResult;
 *       using Feedback = FibonacciFeedback;
 *
 *       // Base action type name (without _SendGoal_Request_ etc.).
 *       // The library appends sub-service suffixes automatically.
 *       static const char* action_type_name() noexcept {
 *           return "example_interfaces::action::dds_::Fibonacci";
 *       }
 *
 *       static constexpr size_t max_goal_serialized_size     = 8u;
 *       static constexpr size_t max_result_serialized_size   = 512u;
 *       static constexpr size_t max_feedback_serialized_size = 64u;
 *
 *       static Goal     make_goal()     noexcept { return {}; }
 *       static Result   make_result()   noexcept { return {}; }
 *       static Feedback make_feedback() noexcept { return {}; }
 *
 *       static size_t encode_goal(uint8_t* d, size_t c, const Goal& g)         noexcept { ... }
 *       static bool   decode_goal(const uint8_t* s, size_t n, Goal& g)          noexcept { ... }
 *       static size_t encode_result(uint8_t* d, size_t c, const Result& r)     noexcept { ... }
 *       static bool   decode_result(const uint8_t* s, size_t n, Result& r)      noexcept { ... }
 *       static size_t encode_feedback(uint8_t* d, size_t c, const Feedback& f) noexcept { ... }
 *       static bool   decode_feedback(const uint8_t* s, size_t n, Feedback& f)  noexcept { ... }
 *   };
 *   template<> inline constexpr bool milkros::has_action_traits_v<Fibonacci> = true;
 *
 * Pool sizes
 * ----------
 *   MILKROS_MAX_ACTIONS      (default 2) — action server/client slots in Context
 *   MILKROS_MAX_ACTION_SIZE  (default 512 B) — max sizeof(ActionServer<T,G,E>)
 */

#pragma once

#include "service.hpp"
#include "topic_names.hpp"
#include "rosette/rtps/participant.hpp"

#include <stdint.h>
#include <string.h>

#ifndef MILKROS_MAX_ACTIONS
#  define MILKROS_MAX_ACTIONS 2u
#endif
#ifndef MILKROS_MAX_ACTION_SIZE
/// Must be >= sizeof(ActionServer<ActionT, GoalCbT, ExecuteCbT>) for all types used.
#  define MILKROS_MAX_ACTION_SIZE 512u
#endif

namespace milkros {

// ============================================================
// GoalId — 16-byte UUID (matches unique_identifier_msgs/UUID)
// ============================================================

struct GoalId {
    uint8_t uuid[16] = {};

    bool operator==(const GoalId& o) const noexcept {
        return memcmp(uuid, o.uuid, 16u) == 0;
    }
    bool operator!=(const GoalId& o) const noexcept { return !(*this == o); }
    bool is_zero()   const noexcept {
        for (uint8_t b : uuid) { if (b) return false; }
        return true;
    }
};

// ============================================================
// GoalStatus — mirrors action_msgs/GoalStatus status codes
// ============================================================

enum class GoalStatus : int8_t {
    UNKNOWN   = 0,  ///< Status not yet known.
    ACCEPTED  = 1,  ///< Goal accepted, queued for execution.
    EXECUTING = 2,  ///< Goal is being actively executed.
    CANCELING = 3,  ///< Cancel requested; waiting for handler to acknowledge.
    SUCCEEDED = 4,  ///< Goal completed successfully.
    CANCELED  = 5,  ///< Goal was canceled.
    ABORTED   = 6,  ///< Goal was aborted (failure).
};

/// Return value from the goal-acceptance callback.
enum class GoalResponse : uint8_t {
    ACCEPT = 0,
    REJECT = 1,
};

// ============================================================
// action_traits<ActionT> — user specializes this
// ============================================================

template<typename ActionT>
struct action_traits;  ///< Must be specialised before instantiating ActionServer/ActionClient.

template<typename ActionT>
inline constexpr bool has_action_traits_v = false;

// ============================================================
// Internal helpers
// ============================================================

namespace detail {

/// Get current time in seconds + nanoseconds (uses RTPS_GET_MS()).
inline void now_stamp(uint32_t& sec_out, uint32_t& nsec_out) noexcept {
    const uint32_t ms = RTPS_GET_MS();
    sec_out  = ms / 1000u;
    nsec_out = (ms % 1000u) * 1000000u;
}

} // namespace detail

// ============================================================
// GoalHandle<ActionT> — passed to execute callback by reference
// ============================================================

/**
 * @brief Handle to an active action goal.
 *
 * Passed by reference to the execute callback.  The callback uses it to:
 *   - Query goal status and ID.
 *   - Publish intermediate feedback.
 *   - Complete the goal (succeed, abort, cancel).
 *
 * GoalHandle is owned by ActionServer and valid only while the goal is active.
 * Do not store a pointer to it across ticks.
 */
template<typename ActionT>
class GoalHandle {
    static_assert(has_action_traits_v<ActionT>,
                  "milkros::action_traits specialization required for ActionT");
public:
    using traits   = action_traits<ActionT>;
    using Feedback = typename traits::Feedback;
    using Result   = typename traits::Result;

    GoalHandle() noexcept = default;

    // ----------------------------------------------------------
    // State queries
    // ----------------------------------------------------------

    GoalStatus     status()      const noexcept { return *status_; }
    const GoalId&  goal_id()     const noexcept { return *goal_id_; }
    bool           is_active()   const noexcept { return status() == GoalStatus::EXECUTING; }
    bool           is_canceling()const noexcept { return status() == GoalStatus::CANCELING; }

    // ----------------------------------------------------------
    // Feedback
    // ----------------------------------------------------------

    /// Publish an intermediate feedback message.
    void publish_feedback(const Feedback& fb) noexcept {
        if (!feedback_writer_ || !goal_id_) return;

        constexpr size_t kMax = 16u + traits::max_feedback_serialized_size;
        uint8_t buf[kMax];

        memcpy(buf, goal_id_->uuid, 16u);
        const size_t encoded = traits::encode_feedback(buf + 16u, sizeof(buf) - 16u, fb);
        if (encoded == 0u) return;

        feedback_writer_->write_reliable(rtps::ChangeKind::ALIVE,
                                          buf,
                                          static_cast<uint32_t>(16u + encoded));
    }

    // ----------------------------------------------------------
    // Terminal transitions — call at most once per goal
    // ----------------------------------------------------------

    /// Signal successful completion with a result.
    void succeed(const Result& result) noexcept {
        terminal(GoalStatus::SUCCEEDED, result);
    }

    /// Signal aborted execution with a result.
    void abort(const Result& result) noexcept {
        terminal(GoalStatus::ABORTED, result);
    }

    /// Acknowledge cancellation with a result (call when is_canceling()).
    void cancel(const Result& result) noexcept {
        terminal(GoalStatus::CANCELED, result);
    }

    // Internal: called by ActionServer to wire up the handle.
    void bind(GoalId*               goal_id,
              GoalStatus*           status,
              rtps::StatefulWriter<1>* feedback_writer,
              rtps::StatefulWriter<1>* status_writer,
              uint8_t*              result_buf,
              size_t                result_buf_cap,
              size_t*               result_encoded_size) noexcept {
        goal_id_             = goal_id;
        status_              = status;
        feedback_writer_     = feedback_writer;
        status_writer_       = status_writer;
        result_buf_          = result_buf;
        result_buf_cap_      = result_buf_cap;
        result_encoded_size_ = result_encoded_size;
    }

private:
    void terminal(GoalStatus s, const Result& result) noexcept {
        if (!status_ || !goal_id_) return;
        if (*status_ != GoalStatus::EXECUTING &&
            *status_ != GoalStatus::CANCELING) return;  // already terminal

        *status_ = s;

        // Encode the result into the ActionServer's result buffer for later
        // retrieval via get_result service.
        if (result_buf_ && result_encoded_size_) {
            *result_encoded_size_ = traits::encode_result(result_buf_,
                                                           result_buf_cap_,
                                                           result);
        }

        // Publish updated status.
        publish_status();
    }

    void publish_status() noexcept {
        if (!status_writer_ || !goal_id_) return;

        uint8_t buf[17];
        memcpy(buf, goal_id_->uuid, 16u);
        buf[16] = static_cast<uint8_t>(*status_);
        status_writer_->write_reliable(rtps::ChangeKind::ALIVE, buf, 17u);
    }

    GoalId*                  goal_id_             = nullptr;
    GoalStatus*              status_              = nullptr;
    rtps::StatefulWriter<1>* feedback_writer_     = nullptr;
    rtps::StatefulWriter<1>* status_writer_       = nullptr;
    uint8_t*                 result_buf_          = nullptr;
    size_t                   result_buf_cap_      = 0u;
    size_t*                  result_encoded_size_ = nullptr;
};

// ============================================================
// IAction — type-erased base stored in Context pool
// ============================================================

class IAction {
public:
    virtual void tick(uint32_t now_ms) noexcept = 0;
protected:
    ~IAction() = default;
};

// ============================================================
// ActionServer<ActionT, GoalCbT, ExecuteCbT>
// ============================================================

/**
 * @brief ROS 2-compatible action server.
 *
 * Supports one active goal at a time (embedded constraint).
 *
 * @tparam ActionT     Action tag type with action_traits<ActionT> specialisation.
 * @tparam GoalCbT     Callable: GoalResponse(const GoalId&, const Goal&) noexcept
 * @tparam ExecuteCbT  Callable: void(GoalHandle<ActionT>&) noexcept
 *                     Called every tick while goal is EXECUTING or CANCELING.
 *                     Must call handle.succeed(), handle.abort(), or handle.cancel()
 *                     when finished.
 *
 * Instances are created via Node::create_action_server() and owned by the
 * Context pool. Never move or copy after creation.
 *
 * The server's tick() is registered as an ITimer with the Context so it is
 * driven automatically by Context::tick_timers().
 */
template<typename ActionT, typename GoalCbT, typename ExecuteCbT>
class ActionServer final : public IAction {
    static_assert(has_action_traits_v<ActionT>,
                  "milkros::action_traits specialization required for ActionT");

public:
    using traits   = action_traits<ActionT>;
    using Goal     = typename traits::Goal;
    using Result   = typename traits::Result;
    using Feedback = typename traits::Feedback;

    ActionServer() noexcept = default;

    ActionServer(// send_goal service endpoints
                 rtps::StatefulReader*    sg_req_reader,
                 rtps::StatefulWriter<1>* sg_resp_writer,
                 // get_result service endpoints
                 rtps::StatefulReader*    gr_req_reader,
                 rtps::StatefulWriter<1>* gr_resp_writer,
                 // cancel_goal service endpoints
                 rtps::StatefulReader*    cg_req_reader,
                 rtps::StatefulWriter<1>* cg_resp_writer,
                 // topic publishers
                 rtps::StatefulWriter<1>* feedback_writer,
                 rtps::StatefulWriter<1>* status_writer,
                 GoalCbT    goal_cb,
                 ExecuteCbT execute_cb) noexcept
        : sg_req_reader_ (sg_req_reader)
        , sg_resp_writer_(sg_resp_writer)
        , gr_req_reader_ (gr_req_reader)
        , gr_resp_writer_(gr_resp_writer)
        , cg_req_reader_ (cg_req_reader)
        , cg_resp_writer_(cg_resp_writer)
        , feedback_writer_(feedback_writer)
        , status_writer_  (status_writer)
        , goal_cb_   (static_cast<GoalCbT&&>(goal_cb))
        , execute_cb_(static_cast<ExecuteCbT&&>(execute_cb))
    {
        // Wire callbacks.
        if (sg_req_reader_) sg_req_reader_->set_on_change_available(&ActionServer::on_send_goal,   this);
        if (gr_req_reader_) gr_req_reader_->set_on_change_available(&ActionServer::on_get_result,  this);
        if (cg_req_reader_) cg_req_reader_->set_on_change_available(&ActionServer::on_cancel_goal, this);

        // Bind the goal handle.
        active_handle_.bind(&active_goal_id_,
                             &active_status_,
                             feedback_writer_,
                             status_writer_,
                             result_buf_,
                             sizeof(result_buf_),
                             &result_encoded_size_);
    }

    // Move-only
    ActionServer(const ActionServer&)            = delete;
    ActionServer& operator=(const ActionServer&) = delete;

    bool valid() const noexcept { return sg_req_reader_ != nullptr; }

    GoalStatus current_status() const noexcept { return active_status_; }

    // IAction: called every tick while a goal is active.
    void tick(uint32_t /*now_ms*/) noexcept override {
        if (active_status_ == GoalStatus::EXECUTING ||
            active_status_ == GoalStatus::CANCELING) {
            execute_cb_(active_handle_);

            // If the handler just made the goal terminal, deliver any pending result.
            if (is_terminal(active_status_) && pending_result_rid_.sequence_number >= 0) {
                send_result_response(pending_result_rid_);
                pending_result_rid_.sequence_number = -1;
            }
        }
    }

private:
    // ---- send_goal ----
    static void on_send_goal(const rtps::CacheChange& change, void* ud) noexcept {
        auto* self = static_cast<ActionServer*>(ud);
        const uint8_t* src = change.payload.data;
        const size_t   len = change.payload.size;

        // Minimum: RequestId(24) + GoalId(16) = 40 bytes.
        if (len < 40u) return;

        RequestId rid{};
        memcpy(rid.client_guid,      src,       16u);
        memcpy(&rid.sequence_number, src + 16u, 8u);

        GoalId goal_id{};
        memcpy(goal_id.uuid, src + 24u, 16u);

        Goal goal = traits::make_goal();
        if (len > 40u) {
            traits::decode_goal(src + 40u, len - 40u, goal);
        }

        const GoalResponse resp = self->goal_cb_(goal_id, goal);
        const bool accepted = (resp == GoalResponse::ACCEPT);

        if (accepted) {
            // Only accept if no goal is currently active.
            if (!is_terminal(self->active_status_) &&
                self->active_status_ != GoalStatus::UNKNOWN) {
                // Reject: already executing.
                // (A production server would queue goals; embedded allows one.)
                self->send_goal_response(rid, false);
                return;
            }

            self->active_goal_id_ = goal_id;
            self->active_status_  = GoalStatus::EXECUTING;
            // Store goal for execute callback via handle.
            self->active_goal_    = goal;
            self->result_encoded_size_ = 0u;
            self->pending_result_rid_.sequence_number = -1;
        }

        self->send_goal_response(rid, accepted);
    }

    void send_goal_response(const RequestId& rid, bool accepted) noexcept {
        if (!sg_resp_writer_) return;
        uint32_t sec = 0u, nsec = 0u;
        detail::now_stamp(sec, nsec);

        // Payload: RequestId(24) + accepted(1) + stamp_sec(4) + stamp_nsec(4) = 33 bytes.
        uint8_t buf[33];
        memcpy(buf,       rid.client_guid,      16u);
        memcpy(buf + 16u, &rid.sequence_number,  8u);
        buf[24]  = accepted ? 1u : 0u;
        memcpy(buf + 25u, &sec,  4u);
        memcpy(buf + 29u, &nsec, 4u);

        sg_resp_writer_->write_reliable(rtps::ChangeKind::ALIVE, buf, 33u);

        // Publish initial status.
        if (accepted) {
            uint8_t sbuf[17];
            memcpy(sbuf, active_goal_id_.uuid, 16u);
            sbuf[16] = static_cast<uint8_t>(active_status_);
            if (status_writer_) {
                status_writer_->write_reliable(rtps::ChangeKind::ALIVE, sbuf, 17u);
            }
        }
    }

    // ---- get_result ----
    static void on_get_result(const rtps::CacheChange& change, void* ud) noexcept {
        auto* self = static_cast<ActionServer*>(ud);
        const uint8_t* src = change.payload.data;
        const size_t   len = change.payload.size;

        // Minimum: RequestId(24) + GoalId(16) = 40 bytes.
        if (len < 40u) return;

        RequestId rid{};
        memcpy(rid.client_guid,      src,       16u);
        memcpy(&rid.sequence_number, src + 16u, 8u);

        GoalId goal_id{};
        memcpy(goal_id.uuid, src + 24u, 16u);

        if (!(goal_id == self->active_goal_id_)) {
            // Unknown goal — send UNKNOWN status with empty result.
            self->send_result_response_status(rid, GoalStatus::UNKNOWN);
            return;
        }

        if (is_terminal(self->active_status_)) {
            // Goal already done — respond immediately.
            self->send_result_response(rid);
        } else {
            // Goal still running — remember who asked; respond when done.
            self->pending_result_rid_ = rid;
        }
    }

    void send_result_response(const RequestId& rid) noexcept {
        send_result_response_status(rid, active_status_);
    }

    void send_result_response_status(const RequestId& rid, GoalStatus s) noexcept {
        if (!gr_resp_writer_) return;

        // Payload: RequestId(24) + status(1) + result(N).
        constexpr size_t kMax = 25u + traits::max_result_serialized_size;
        uint8_t buf[kMax];

        memcpy(buf,       rid.client_guid,      16u);
        memcpy(buf + 16u, &rid.sequence_number,  8u);
        buf[24] = static_cast<uint8_t>(s);

        size_t result_len = 0u;
        if (s != GoalStatus::UNKNOWN && result_encoded_size_ > 0u) {
            const size_t copy_len = (result_encoded_size_ < (sizeof(buf) - 25u))
                                    ? result_encoded_size_ : (sizeof(buf) - 25u);
            memcpy(buf + 25u, result_buf_, copy_len);
            result_len = copy_len;
        }

        gr_resp_writer_->write_reliable(rtps::ChangeKind::ALIVE,
                                         buf,
                                         static_cast<uint32_t>(25u + result_len));
    }

    // ---- cancel_goal ----
    static void on_cancel_goal(const rtps::CacheChange& change, void* ud) noexcept {
        auto* self = static_cast<ActionServer*>(ud);
        const uint8_t* src = change.payload.data;
        const size_t   len = change.payload.size;

        // Minimum: RequestId(24) + GoalId(16) = 40 bytes.
        if (len < 40u) return;

        RequestId rid{};
        memcpy(rid.client_guid,      src,       16u);
        memcpy(&rid.sequence_number, src + 16u, 8u);

        GoalId goal_id{};
        memcpy(goal_id.uuid, src + 24u, 16u);

        // return_code: 0=OK, 1=rejected, 2=unknown_goal_id, 3=goal_terminated
        uint8_t return_code = 0u;
        if (!(goal_id == self->active_goal_id_)) {
            return_code = 2u;  // unknown goal
        } else if (is_terminal(self->active_status_)) {
            return_code = 3u;  // already done
        } else {
            // Transition to CANCELING; execute_cb will call handle.cancel().
            self->active_status_ = GoalStatus::CANCELING;
        }

        // Payload: RequestId(24) + return_code(1) + GoalId(16) = 41 bytes.
        uint8_t buf[41];
        memcpy(buf,       rid.client_guid,      16u);
        memcpy(buf + 16u, &rid.sequence_number,  8u);
        buf[24] = return_code;
        memcpy(buf + 25u, goal_id.uuid, 16u);

        if (self->cg_resp_writer_) {
            self->cg_resp_writer_->write_reliable(rtps::ChangeKind::ALIVE, buf, 41u);
        }
    }

    static bool is_terminal(GoalStatus s) noexcept {
        return s == GoalStatus::SUCCEEDED ||
               s == GoalStatus::CANCELED  ||
               s == GoalStatus::ABORTED;
    }

    // ---- Members ----

    rtps::StatefulReader*    sg_req_reader_  = nullptr;
    rtps::StatefulWriter<1>* sg_resp_writer_ = nullptr;
    rtps::StatefulReader*    gr_req_reader_  = nullptr;
    rtps::StatefulWriter<1>* gr_resp_writer_ = nullptr;
    rtps::StatefulReader*    cg_req_reader_  = nullptr;
    rtps::StatefulWriter<1>* cg_resp_writer_ = nullptr;
    rtps::StatefulWriter<1>* feedback_writer_ = nullptr;
    rtps::StatefulWriter<1>* status_writer_   = nullptr;

    GoalCbT    goal_cb_{};
    ExecuteCbT execute_cb_{};

    GoalHandle<ActionT> active_handle_{};

    GoalId     active_goal_id_{};
    GoalStatus active_status_ = GoalStatus::UNKNOWN;
    Goal       active_goal_{};

    // Encoded result buffer — populated by GoalHandle::terminal().
    alignas(4) uint8_t result_buf_[traits::max_result_serialized_size] = {};
    size_t              result_encoded_size_ = 0u;

    // Deferred get_result requestor (seq_number = -1 means none pending).
    RequestId pending_result_rid_ = { {}, -1 };
};

// ============================================================
// ActionClient<ActionT>
// ============================================================

/**
 * @brief ROS 2-compatible action client.
 *
 * @tparam ActionT  Action tag type with action_traits<ActionT> specialisation.
 *
 * Response/feedback callbacks use plain function-pointer + void* for
 * zero-overhead embedded use.
 *
 * Instances are created via Node::create_action_client() and owned by the
 * Context pool. Never move or copy after creation.
 *
 * Typical usage:
 *   auto* client = node.create_action_client<Fibonacci>("fibonacci");
 *
 *   GoalId gid{};
 *   client->send_goal(goal, gid,
 *       [](const GoalId& id, bool ok, void*) { ... },   // goal_resp_cb
 *       [](const GoalId& id, const Feedback& fb, void*) { ... }, // feedback_cb
 *       [](const GoalId& id, GoalStatus s, const Result& r, void*) { ... }, // result_cb
 *       nullptr);
 *
 *   // Later, when ready to retrieve the result:
 *   client->request_result(gid);
 */
template<typename ActionT>
class ActionClient final : public IAction {
    static_assert(has_action_traits_v<ActionT>,
                  "milkros::action_traits specialization required for ActionT");

public:
    using traits   = action_traits<ActionT>;
    using Goal     = typename traits::Goal;
    using Result   = typename traits::Result;
    using Feedback = typename traits::Feedback;

    using GoalRespCb  = void(*)(const GoalId&, bool accepted,            void*) noexcept;
    using FeedbackCb  = void(*)(const GoalId&, const Feedback&,           void*) noexcept;
    using ResultCb    = void(*)(const GoalId&, GoalStatus, const Result&, void*) noexcept;

    ActionClient() noexcept = default;

    ActionClient(// send_goal service client endpoints
                 rtps::StatefulWriter<1>* sg_req_writer,
                 rtps::StatefulReader*    sg_resp_reader,
                 // get_result service client endpoints
                 rtps::StatefulWriter<1>* gr_req_writer,
                 rtps::StatefulReader*    gr_resp_reader,
                 // cancel_goal service client endpoints
                 rtps::StatefulWriter<1>* cg_req_writer,
                 rtps::StatefulReader*    cg_resp_reader,
                 // topic subscribers
                 rtps::StatefulReader*    feedback_reader,
                 rtps::StatefulReader*    status_reader) noexcept
        : sg_req_writer_ (sg_req_writer)
        , sg_resp_reader_(sg_resp_reader)
        , gr_req_writer_ (gr_req_writer)
        , gr_resp_reader_(gr_resp_reader)
        , cg_req_writer_ (cg_req_writer)
        , cg_resp_reader_(cg_resp_reader)
        , feedback_reader_(feedback_reader)
        , status_reader_  (status_reader)
    {
        // Capture writer GUID for send_goal correlation.
        if (sg_req_writer_) {
            const rtps::Guid& g = sg_req_writer_->guid();
            memcpy(sg_writer_guid_, g.prefix.value,   12u);
            memcpy(sg_writer_guid_ + 12u, g.entityId.entityKey, 3u);
            sg_writer_guid_[15] = g.entityId.entityKind;
        }
        // Capture writer GUID for get_result correlation.
        if (gr_req_writer_) {
            const rtps::Guid& g = gr_req_writer_->guid();
            memcpy(gr_writer_guid_, g.prefix.value,   12u);
            memcpy(gr_writer_guid_ + 12u, g.entityId.entityKey, 3u);
            gr_writer_guid_[15] = g.entityId.entityKind;
        }

        if (sg_resp_reader_) sg_resp_reader_->set_on_change_available(&ActionClient::on_sg_response,  this);
        if (gr_resp_reader_) gr_resp_reader_->set_on_change_available(&ActionClient::on_gr_response,  this);
        if (cg_resp_reader_) cg_resp_reader_->set_on_change_available(&ActionClient::on_cg_response,  this);
        if (feedback_reader_) feedback_reader_->set_on_change_available(&ActionClient::on_feedback,   this);
        if (status_reader_)   status_reader_->set_on_change_available(&ActionClient::on_status,       this);
    }

    // Move-only
    ActionClient(const ActionClient&)            = delete;
    ActionClient& operator=(const ActionClient&) = delete;

    bool valid() const noexcept { return sg_req_writer_ != nullptr; }

    // IAction — no periodic tick needed for client.
    void tick(uint32_t /*now_ms*/) noexcept override {}

    /**
     * @brief Send a goal to the action server.
     *
     * @param goal          Goal to send.
     * @param goal_id_out   Receives the generated GoalId (use for subsequent calls).
     * @param goal_resp_cb  Called when accept/reject response arrives (may be nullptr).
     * @param feedback_cb   Called when feedback messages arrive (may be nullptr).
     * @param result_cb     Called when the result is available (may be nullptr).
     * @param ud            User data passed to all callbacks.
     * @returns true if the goal was sent successfully.
     */
    bool send_goal(const Goal&  goal,
                   GoalId&      goal_id_out,
                   GoalRespCb   goal_resp_cb  = nullptr,
                   FeedbackCb   feedback_cb   = nullptr,
                   ResultCb     result_cb     = nullptr,
                   void*        ud            = nullptr) noexcept {
        if (!sg_req_writer_) return false;

        // Generate GoalId from writer GUID prefix + seqno (unique within participant).
        GoalId gid{};
        memcpy(gid.uuid, sg_writer_guid_, 12u);
        // Use seqno bytes for uniqueness: 4 bytes big-endian at end.
        const uint32_t s = static_cast<uint32_t>(sg_next_seqno_);
        gid.uuid[12] = static_cast<uint8_t>(s >> 24u);
        gid.uuid[13] = static_cast<uint8_t>(s >> 16u);
        gid.uuid[14] = static_cast<uint8_t>(s >>  8u);
        gid.uuid[15] = static_cast<uint8_t>(s);
        goal_id_out = gid;

        // Build RequestId for correlation.
        RequestId rid{};
        memcpy(rid.client_guid, sg_writer_guid_, 16u);
        rid.sequence_number = sg_next_seqno_++;

        // Store callbacks.
        pending_goal_rid_   = rid;
        pending_goal_id_    = gid;
        goal_resp_cb_       = goal_resp_cb;
        feedback_cb_        = feedback_cb;
        result_cb_          = result_cb;
        user_data_          = ud;
        pending_goal_active_= true;

        // Payload: RequestId(24) + GoalId(16) + encoded_goal.
        constexpr size_t kMax = 40u + traits::max_goal_serialized_size;
        uint8_t buf[kMax];

        memcpy(buf,       rid.client_guid,      16u);
        memcpy(buf + 16u, &rid.sequence_number,  8u);
        memcpy(buf + 24u, gid.uuid,             16u);

        const size_t encoded = traits::encode_goal(buf + 40u, sizeof(buf) - 40u, goal);
        return sg_req_writer_->write_reliable(rtps::ChangeKind::ALIVE,
                                               buf,
                                               static_cast<uint32_t>(40u + encoded));
    }

    /**
     * @brief Request the result from the server for an accepted goal.
     *
     * Call this after receiving a goal_accepted = true response, or at any
     * time after sending the goal.  The ResultCb registered in send_goal() is
     * called when the result arrives.
     */
    bool request_result(const GoalId& goal_id) noexcept {
        if (!gr_req_writer_) return false;

        RequestId rid{};
        memcpy(rid.client_guid, gr_writer_guid_, 16u);
        rid.sequence_number = gr_next_seqno_++;

        pending_result_rid_ = rid;

        // Payload: RequestId(24) + GoalId(16) = 40 bytes.
        uint8_t buf[40];
        memcpy(buf,       rid.client_guid,      16u);
        memcpy(buf + 16u, &rid.sequence_number,  8u);
        memcpy(buf + 24u, goal_id.uuid,         16u);

        return gr_req_writer_->write_reliable(rtps::ChangeKind::ALIVE, buf, 40u);
    }

    /**
     * @brief Send a cancel request for the given goal.
     */
    bool cancel_goal(const GoalId& goal_id) noexcept {
        if (!cg_req_writer_) return false;

        RequestId rid{};
        memcpy(rid.client_guid, sg_writer_guid_, 16u);  // reuse sg GUID
        rid.sequence_number = sg_next_seqno_++;

        uint32_t sec = 0u, nsec = 0u;
        detail::now_stamp(sec, nsec);

        // Payload: RequestId(24) + GoalId(16) + stamp_sec(4) + stamp_nsec(4) = 48 bytes.
        uint8_t buf[48];
        memcpy(buf,       rid.client_guid,      16u);
        memcpy(buf + 16u, &rid.sequence_number,  8u);
        memcpy(buf + 24u, goal_id.uuid,         16u);
        memcpy(buf + 40u, &sec,                  4u);
        memcpy(buf + 44u, &nsec,                 4u);

        return cg_req_writer_->write_reliable(rtps::ChangeKind::ALIVE, buf, 48u);
    }

private:
    // ---- send_goal response ----
    static void on_sg_response(const rtps::CacheChange& change, void* ud) noexcept {
        auto* self = static_cast<ActionClient*>(ud);
        const uint8_t* src = change.payload.data;
        const size_t   len = change.payload.size;
        if (len < 33u || !self->pending_goal_active_) return;  // RequestId(24)+accepted(1)+stamp(8)

        RequestId rid{};
        memcpy(rid.client_guid,      src,       16u);
        memcpy(&rid.sequence_number, src + 16u, 8u);
        if (rid != self->pending_goal_rid_) return;

        const bool accepted = (src[24] != 0u);
        if (self->goal_resp_cb_) {
            self->goal_resp_cb_(self->pending_goal_id_, accepted, self->user_data_);
        }
        if (!accepted) {
            self->pending_goal_active_ = false;
        }
    }

    // ---- get_result response ----
    static void on_gr_response(const rtps::CacheChange& change, void* ud) noexcept {
        auto* self = static_cast<ActionClient*>(ud);
        const uint8_t* src = change.payload.data;
        const size_t   len = change.payload.size;
        if (len < 25u) return;  // RequestId(24) + status(1)

        RequestId rid{};
        memcpy(rid.client_guid,      src,       16u);
        memcpy(&rid.sequence_number, src + 16u, 8u);
        if (rid != self->pending_result_rid_) return;

        const auto status = static_cast<GoalStatus>(src[24]);
        Result result = traits::make_result();
        if (len > 25u) {
            traits::decode_result(src + 25u, len - 25u, result);
        }

        if (self->result_cb_) {
            self->result_cb_(self->pending_goal_id_, status, result, self->user_data_);
        }
        self->pending_goal_active_ = false;
    }

    // ---- cancel_goal response (informational) ----
    static void on_cg_response(const rtps::CacheChange& /*change*/,
                                 void* /*ud*/) noexcept {
        // Acknowledged — application may inspect the cancel response payload
        // if needed. For now, the cancel is considered complete on receipt.
    }

    // ---- feedback topic ----
    static void on_feedback(const rtps::CacheChange& change, void* ud) noexcept {
        auto* self = static_cast<ActionClient*>(ud);
        const uint8_t* src = change.payload.data;
        const size_t   len = change.payload.size;
        if (len < 16u || !self->pending_goal_active_ || !self->feedback_cb_) return;

        GoalId gid{};
        memcpy(gid.uuid, src, 16u);
        if (!(gid == self->pending_goal_id_)) return;

        Feedback fb = traits::make_feedback();
        if (len > 16u && traits::decode_feedback(src + 16u, len - 16u, fb)) {
            self->feedback_cb_(gid, fb, self->user_data_);
        }
    }

    // ---- status topic ----
    static void on_status(const rtps::CacheChange& /*change*/,
                            void* /*ud*/) noexcept {
        // Status updates are informational; callbacks are not wired for now.
        // The result_cb fires via on_gr_response after request_result() is called.
    }

    // ---- Members ----

    rtps::StatefulWriter<1>* sg_req_writer_  = nullptr;
    rtps::StatefulReader*    sg_resp_reader_ = nullptr;
    rtps::StatefulWriter<1>* gr_req_writer_  = nullptr;
    rtps::StatefulReader*    gr_resp_reader_ = nullptr;
    rtps::StatefulWriter<1>* cg_req_writer_  = nullptr;
    rtps::StatefulReader*    cg_resp_reader_ = nullptr;
    rtps::StatefulReader*    feedback_reader_ = nullptr;
    rtps::StatefulReader*    status_reader_   = nullptr;

    uint8_t sg_writer_guid_[16] = {};
    uint8_t gr_writer_guid_[16] = {};
    int64_t sg_next_seqno_ = 0;
    int64_t gr_next_seqno_ = 0;

    // Pending goal state.
    RequestId  pending_goal_rid_   = {{}, -1};
    GoalId     pending_goal_id_    = {};
    bool       pending_goal_active_= false;
    GoalRespCb goal_resp_cb_       = nullptr;
    FeedbackCb feedback_cb_        = nullptr;
    ResultCb   result_cb_          = nullptr;
    void*      user_data_          = nullptr;

    RequestId  pending_result_rid_ = {{}, -1};
};

} // namespace milkros
