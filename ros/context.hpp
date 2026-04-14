#pragma once

#include "node.hpp"
#include "timer.hpp"
#include "service.hpp"
#include "action.hpp"
#include "topic_names.hpp"
#include "discovery.hpp"
#include "rtps/participant.hpp"
#include "rtps/spdp.hpp"
#include "rtps/transport.hpp"
#include "rtps/stateful_writer.hpp"

#include <stdint.h>
#include <new>

namespace milkros {

/**
 * @brief Configuration for a communication context.
 *
 * Closest embedded analogue of `rclcpp::Context`.  Owns the RTPS participant
 * and all endpoint/timer/service/action allocations for one ROS domain.
 *
 * Scheduling is intentionally left to the application: a FreeRTOS task or
 * bare-metal loop decides when to call tick().  Subclasses override tick()
 * and should call tick_participant(now_ms) and tick_timers(now_ms) from it.
 */
struct ContextOptions {
    uint8_t domain_id      = 0u;
    uint8_t participant_id = 0u;
    uint32_t rx_buffer_size = 1536u;
};

// ServiceOptions is defined in service.hpp (included above).

struct PublisherOptions {
    rtps::ReliabilityKind reliability = rtps::ReliabilityKind::BEST_EFFORT;
    rtps::DurabilityKind  durability  = rtps::DurabilityKind::VOLATILE;
    rtps::TopicKind       topic_kind  = rtps::TopicKind::NO_KEY;
};

struct SubscriptionOptions {
    rtps::ReliabilityKind reliability       = rtps::ReliabilityKind::BEST_EFFORT;
    rtps::DurabilityKind  durability        = rtps::DurabilityKind::VOLATILE;
    rtps::TopicKind       topic_kind        = rtps::TopicKind::NO_KEY;
    uint32_t              heartbeat_response_ms = RTPS_HEARTBEAT_RESPONSE_DELAY_MS;
};

class Context {
public:
    Context(rtps::ITransport& transport,
            const ContextOptions& options = {}) noexcept
        : options_(options)
        , transport_(transport)
        , participant_(options.domain_id, options.participant_id, transport_)
    {}

    virtual ~Context() = default;

    // ----------------------------------------------------------
    // Lifecycle
    // ----------------------------------------------------------

    bool start() noexcept {
        if (started_) return true;
        started_ = true;

        // Create the ros_discovery_info writer (RELIABLE + TRANSIENT_LOCAL).
        // TRANSIENT_LOCAL ensures late-joining readers automatically receive
        // the current graph state without requiring a SPDP hook.
        rtps::WriterConfig disco_cfg{};
        disco_cfg.entityId   = allocate_writer_entity_id();
        disco_cfg.topicName.assign("ros_discovery_info");  // no rt/ prefix — system topic
        disco_cfg.typeName.assign("rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_");
        disco_cfg.reliability = rtps::ReliabilityKind::RELIABLE;
        disco_cfg.durability  = rtps::DurabilityKind::TRANSIENT_LOCAL;
        disco_writer_ = participant_.create_writer<1>(disco_cfg);

        // Record our participant GID once — stays constant for the session.
        graph_.participant_gid = Gid::from_guid(participant_.participant_guid());

        participant_.announce();
        publish_discovery_info();
        return true;
    }

    virtual void tick(uint32_t now_ms) noexcept = 0;

    void announce() noexcept {
        if (started_) participant_.announce();
    }

    // ----------------------------------------------------------
    // Node factory
    // ----------------------------------------------------------

    Node create_node(const char* name,
                     const char* node_namespace = "") noexcept {
        const int idx = graph_.add_node(name, node_namespace);
        publish_discovery_info();
        return Node(*this, name, node_namespace, idx);
    }

    // ----------------------------------------------------------
    // Participant access
    // ----------------------------------------------------------

    rtps::RTPSParticipant& participant() noexcept { return participant_; }

    // ----------------------------------------------------------
    // EntityId allocation
    // ----------------------------------------------------------

    rtps::EntityId allocate_writer_entity_id() noexcept {
        return rtps::EntityId{{0x00, 0x10, next_writer_id_++}, 0x02};
    }

    rtps::EntityId allocate_reader_entity_id() noexcept {
        return rtps::EntityId{{0x00, 0x20, next_reader_id_++}, 0x07};
    }

    // ----------------------------------------------------------
    // Publisher / Subscription factories (unchanged from original)
    // ----------------------------------------------------------

    template<typename MessageT>
    Publisher<MessageT> create_publisher(const char* node_namespace,
                                         const char* topic,
                                         const PublisherOptions& options,
                                         int node_index = -1) noexcept {
        using traits = message_traits<MessageT>;
        static_assert(has_message_traits_v<MessageT>,
                      "milkros::message_traits specialization is required");

        rtps::WriterConfig cfg{};
        cfg.entityId = allocate_writer_entity_id();
        compose_topic_name(cfg.topicName, node_namespace, topic);
        cfg.typeName.assign(traits::dds_type_name());
        cfg.reliability = options.reliability;
        cfg.durability  = options.durability;
        cfg.topicKind   = options.topic_kind;

        auto* writer = participant_.create_writer<1>(cfg);
        if (writer && node_index >= 0 && node_index < static_cast<int>(graph_.node_count)) {
            graph_.nodes[node_index].add_writer(writer->guid());
            publish_discovery_info();
        }
        return Publisher<MessageT>(writer);
    }

    template<typename MessageT, typename CallbackT>
    Subscription<MessageT, CallbackT> create_subscription(
        const char* node_namespace,
        const char* topic,
        CallbackT callback,
        const SubscriptionOptions& options,
        int node_index = -1) noexcept {
        using traits = message_traits<MessageT>;
        static_assert(has_message_traits_v<MessageT>,
                      "milkros::message_traits specialization is required");

        rtps::ReaderConfig cfg{};
        cfg.entityId   = allocate_reader_entity_id();
        compose_topic_name(cfg.topicName, node_namespace, topic);
        cfg.typeName.assign(traits::dds_type_name());
        cfg.reliability = options.reliability;
        cfg.durability  = options.durability;
        cfg.topicKind   = options.topic_kind;
        cfg.hbResponseMs = options.heartbeat_response_ms;

        auto* reader = participant_.create_reader(cfg);
        if (reader && node_index >= 0 && node_index < static_cast<int>(graph_.node_count)) {
            graph_.nodes[node_index].add_reader(reader->guid());
            publish_discovery_info();
        }
        return Subscription<MessageT, CallbackT>(reader, callback);
    }

    // ----------------------------------------------------------
    // Timer factory
    // ----------------------------------------------------------

    /**
     * @brief Create and register a periodic timer.
     *
     * The timer is stored in the Context's pre-allocated timer pool and ticked
     * automatically by tick_timers().  The returned pointer is stable for the
     * lifetime of the Context.
     *
     * @param period_ms  Period in milliseconds.
     * @param callback   Zero-argument callable (lambda, function pointer, …).
     * @returns Pointer to the registered Timer, or nullptr if the pool is full
     *          or sizeof(Timer<CallbackT>) > MILKROS_MAX_TIMER_SIZE.
     */
    template<typename CallbackT>
    Timer<CallbackT>* alloc_timer(uint32_t period_ms, CallbackT callback) noexcept {
        using T = Timer<CallbackT>;
        static_assert(sizeof(T) <= MILKROS_MAX_TIMER_SIZE,
            "sizeof(Timer<CallbackT>) exceeds MILKROS_MAX_TIMER_SIZE. "
            "Increase MILKROS_MAX_TIMER_SIZE before including ros.hpp.");

        if (timer_count_ >= MILKROS_MAX_TIMERS) return nullptr;

        T* t = new (timer_storage_[timer_count_]) T(period_ms,
                                                     static_cast<CallbackT&&>(callback));
        timers_[timer_count_++] = t;
        return t;
    }

    // ----------------------------------------------------------
    // Service factory
    // ----------------------------------------------------------

    /**
     * @brief Create and register a service server.
     *
     * @param node_namespace  Node namespace (used for topic name composition).
     * @param service_name    Service name.
     * @param handler         Callable: void(const Request&, Response&) noexcept
     * @param options         Reliability / durability settings.
     * @returns Pointer to the registered Service, or nullptr on failure.
     */
    template<typename ServiceT, typename HandlerT>
    Service<ServiceT, HandlerT>* alloc_service(const char* node_namespace,
                                                const char* service_name,
                                                HandlerT    handler,
                                                const ServiceOptions& options = {},
                                                int node_index = -1) noexcept {
        using S = Service<ServiceT, HandlerT>;
        using traits = service_traits<ServiceT>;
        static_assert(has_service_traits_v<ServiceT>,
                      "milkros::service_traits specialization is required for ServiceT");
        static_assert(sizeof(S) <= MILKROS_MAX_SERVICE_SIZE,
            "sizeof(Service<ServiceT,HandlerT>) exceeds MILKROS_MAX_SERVICE_SIZE. "
            "Increase MILKROS_MAX_SERVICE_SIZE before including ros.hpp.");

        if (service_count_ >= MILKROS_MAX_SERVICES) return nullptr;

        // Create request reader.
        rtps::ReaderConfig req_cfg{};
        req_cfg.entityId = allocate_reader_entity_id();
        compose_service_request_topic(req_cfg.topicName, node_namespace, service_name);
        req_cfg.typeName.assign(traits::request_type_name());
        req_cfg.reliability  = options.reliability;
        req_cfg.durability   = options.durability;

        auto* req_reader = participant_.create_reader(req_cfg);
        if (!req_reader) return nullptr;

        // Create response writer.
        rtps::WriterConfig resp_cfg{};
        resp_cfg.entityId = allocate_writer_entity_id();
        compose_service_response_topic(resp_cfg.topicName, node_namespace, service_name);
        resp_cfg.typeName.assign(traits::response_type_name());
        resp_cfg.reliability = options.reliability;
        resp_cfg.durability  = options.durability;

        auto* resp_writer = participant_.create_writer<1>(resp_cfg);
        if (!resp_writer) return nullptr;

        S* s = new (service_storage_[service_count_]) S(req_reader,
                                                         resp_writer,
                                                         static_cast<HandlerT&&>(handler));
        services_[service_count_++] = s;
        if (node_index >= 0 && node_index < static_cast<int>(graph_.node_count)) {
            graph_.nodes[node_index].add_reader(req_reader->guid());
            graph_.nodes[node_index].add_writer(resp_writer->guid());
            publish_discovery_info();
        }
        return s;
    }

    // ----------------------------------------------------------
    // Client factory
    // ----------------------------------------------------------

    /**
     * @brief Create and register a service client.
     *
     * @tparam MaxPending  Maximum concurrent in-flight requests (default 4).
     * @param node_namespace  Node namespace.
     * @param service_name    Service name.
     * @param options         Reliability / durability settings.
     * @returns Pointer to the registered Client, or nullptr on failure.
     */
    template<typename ServiceT, uint8_t MaxPending = 4>
    Client<ServiceT, MaxPending>* alloc_client(const char* node_namespace,
                                                const char* service_name,
                                                const ServiceOptions& options = {},
                                                int node_index = -1) noexcept {
        using C = Client<ServiceT, MaxPending>;
        using traits = service_traits<ServiceT>;
        static_assert(has_service_traits_v<ServiceT>,
                      "milkros::service_traits specialization is required for ServiceT");
        static_assert(sizeof(C) <= MILKROS_MAX_CLIENT_SIZE,
            "sizeof(Client<ServiceT,MaxPending>) exceeds MILKROS_MAX_CLIENT_SIZE. "
            "Increase MILKROS_MAX_CLIENT_SIZE before including ros.hpp.");

        if (client_count_ >= MILKROS_MAX_CLIENTS) return nullptr;

        // Create request writer.
        rtps::WriterConfig req_cfg{};
        req_cfg.entityId = allocate_writer_entity_id();
        compose_service_request_topic(req_cfg.topicName, node_namespace, service_name);
        req_cfg.typeName.assign(traits::request_type_name());
        req_cfg.reliability = options.reliability;
        req_cfg.durability  = options.durability;

        auto* req_writer = participant_.create_writer<1>(req_cfg);
        if (!req_writer) return nullptr;

        // Create response reader.
        rtps::ReaderConfig resp_cfg{};
        resp_cfg.entityId = allocate_reader_entity_id();
        compose_service_response_topic(resp_cfg.topicName, node_namespace, service_name);
        resp_cfg.typeName.assign(traits::response_type_name());
        resp_cfg.reliability = options.reliability;
        resp_cfg.durability  = options.durability;

        auto* resp_reader = participant_.create_reader(resp_cfg);
        if (!resp_reader) return nullptr;

        C* c = new (client_storage_[client_count_]) C(req_writer, resp_reader);
        clients_[client_count_++] = c;
        if (node_index >= 0 && node_index < static_cast<int>(graph_.node_count)) {
            graph_.nodes[node_index].add_writer(req_writer->guid());
            graph_.nodes[node_index].add_reader(resp_reader->guid());
            publish_discovery_info();
        }
        return c;
    }

    // ----------------------------------------------------------
    // ActionServer factory
    // ----------------------------------------------------------

    /**
     * @brief Create and register an action server (full ROS 2 action protocol).
     *
     * Allocates 5 writers + 3 readers. Ensure RTPS_MAX_USER_WRITERS >= 8 and
     * RTPS_MAX_USER_READERS >= 8 when combining with other publishers/services.
     *
     * @param node_namespace  Node namespace.
     * @param action_name     Action name (e.g. "fibonacci").
     * @param goal_cb         GoalResponse(const GoalId&, const Goal&) — accept/reject.
     * @param execute_cb      void(GoalHandle<ActionT>&) — called every tick while executing.
     * @returns Pointer to ActionServer, or nullptr on failure.
     */
    template<typename ActionT, typename GoalCbT, typename ExecuteCbT>
    ActionServer<ActionT, GoalCbT, ExecuteCbT>*
    alloc_action_server(const char* node_namespace,
                         const char* action_name,
                         GoalCbT     goal_cb,
                         ExecuteCbT  execute_cb,
                         int         node_index = -1) noexcept {
        using AS = ActionServer<ActionT, GoalCbT, ExecuteCbT>;
        using traits = action_traits<ActionT>;
        static_assert(has_action_traits_v<ActionT>,
                      "milkros::action_traits specialization is required for ActionT");
        static_assert(sizeof(AS) <= MILKROS_MAX_ACTION_SIZE,
            "sizeof(ActionServer<...>) exceeds MILKROS_MAX_ACTION_SIZE. "
            "Increase MILKROS_MAX_ACTION_SIZE before including ros.hpp.");

        if (action_count_ >= MILKROS_MAX_ACTIONS) return nullptr;

        // Helper: build action sub-topic name into a TopicName.
        auto make_rq = [&](const char* suffix) -> rtps::TopicName {
            rtps::TopicName t{};
            compose_action_subtopic(t, node_namespace, action_name, suffix, true, false);
            return t;
        };
        auto make_rr = [&](const char* suffix) -> rtps::TopicName {
            rtps::TopicName t{};
            compose_action_subtopic(t, node_namespace, action_name, suffix, false, true);
            return t;
        };
        auto make_rt = [&](const char* suffix) -> rtps::TopicName {
            rtps::TopicName t{};
            compose_action_subtopic(t, node_namespace, action_name, suffix, false, false);
            return t;
        };

        // Helper: build action sub-service type name.
        auto type_name = [&](const char* suffix) -> rtps::TopicName {
            rtps::TopicName t{};
            detail::build_type_name(t.str, sizeof(t.str),
                                     traits::action_type_name(), suffix);
            return t;
        };

        const ServiceOptions svc_opts{};

        // --- send_goal reader (server listens for goal requests) ---
        rtps::ReaderConfig sg_req_cfg{};
        sg_req_cfg.entityId = allocate_reader_entity_id();
        sg_req_cfg.topicName = make_rq("send_goalRequest");
        sg_req_cfg.typeName  = type_name("_SendGoal_Request_");
        sg_req_cfg.reliability = svc_opts.reliability;
        auto* sg_req_reader = participant_.create_reader(sg_req_cfg);
        if (!sg_req_reader) return nullptr;

        // --- send_goal writer (server sends accept/reject) ---
        rtps::WriterConfig sg_resp_cfg{};
        sg_resp_cfg.entityId = allocate_writer_entity_id();
        sg_resp_cfg.topicName = make_rr("send_goalReply");
        sg_resp_cfg.typeName  = type_name("_SendGoal_Response_");
        sg_resp_cfg.reliability = svc_opts.reliability;
        auto* sg_resp_writer = participant_.create_writer<1>(sg_resp_cfg);
        if (!sg_resp_writer) return nullptr;

        // --- get_result reader ---
        rtps::ReaderConfig gr_req_cfg{};
        gr_req_cfg.entityId = allocate_reader_entity_id();
        gr_req_cfg.topicName = make_rq("get_resultRequest");
        gr_req_cfg.typeName  = type_name("_GetResult_Request_");
        gr_req_cfg.reliability = svc_opts.reliability;
        auto* gr_req_reader = participant_.create_reader(gr_req_cfg);
        if (!gr_req_reader) return nullptr;

        // --- get_result writer ---
        rtps::WriterConfig gr_resp_cfg{};
        gr_resp_cfg.entityId = allocate_writer_entity_id();
        gr_resp_cfg.topicName = make_rr("get_resultReply");
        gr_resp_cfg.typeName  = type_name("_GetResult_Response_");
        gr_resp_cfg.reliability = svc_opts.reliability;
        auto* gr_resp_writer = participant_.create_writer<1>(gr_resp_cfg);
        if (!gr_resp_writer) return nullptr;

        // --- cancel_goal reader ---
        rtps::ReaderConfig cg_req_cfg{};
        cg_req_cfg.entityId = allocate_reader_entity_id();
        cg_req_cfg.topicName = make_rq("cancel_goalRequest");
        cg_req_cfg.typeName.assign("action_msgs::srv::dds_::CancelGoal_Request_");
        cg_req_cfg.reliability = svc_opts.reliability;
        auto* cg_req_reader = participant_.create_reader(cg_req_cfg);
        if (!cg_req_reader) return nullptr;

        // --- cancel_goal writer ---
        rtps::WriterConfig cg_resp_cfg{};
        cg_resp_cfg.entityId = allocate_writer_entity_id();
        cg_resp_cfg.topicName = make_rr("cancel_goalReply");
        cg_resp_cfg.typeName.assign("action_msgs::srv::dds_::CancelGoal_Response_");
        cg_resp_cfg.reliability = svc_opts.reliability;
        auto* cg_resp_writer = participant_.create_writer<1>(cg_resp_cfg);
        if (!cg_resp_writer) return nullptr;

        // --- feedback topic writer ---
        rtps::WriterConfig fb_cfg{};
        fb_cfg.entityId = allocate_writer_entity_id();
        fb_cfg.topicName = make_rt("_action/feedback");
        fb_cfg.typeName  = type_name("_FeedbackMessage_");
        fb_cfg.reliability = rtps::ReliabilityKind::BEST_EFFORT;
        auto* feedback_writer = participant_.create_writer<1>(fb_cfg);
        if (!feedback_writer) return nullptr;

        // --- status topic writer ---
        rtps::WriterConfig st_cfg{};
        st_cfg.entityId = allocate_writer_entity_id();
        st_cfg.topicName = make_rt("_action/status");
        st_cfg.typeName.assign("action_msgs::msg::dds_::GoalStatusArray_");
        st_cfg.reliability = rtps::ReliabilityKind::BEST_EFFORT;
        auto* status_writer = participant_.create_writer<1>(st_cfg);
        if (!status_writer) return nullptr;

        AS* as = new (action_storage_[action_count_]) AS(
            sg_req_reader, sg_resp_writer,
            gr_req_reader, gr_resp_writer,
            cg_req_reader, cg_resp_writer,
            feedback_writer, status_writer,
            static_cast<GoalCbT&&>(goal_cb),
            static_cast<ExecuteCbT&&>(execute_cb));

        action_ifaces_[action_count_++] = as;
        if (node_index >= 0 && node_index < static_cast<int>(graph_.node_count)) {
            NodeEntitiesInfo& n = graph_.nodes[node_index];
            n.add_reader(sg_req_reader->guid());
            n.add_writer(sg_resp_writer->guid());
            n.add_reader(gr_req_reader->guid());
            n.add_writer(gr_resp_writer->guid());
            n.add_reader(cg_req_reader->guid());
            n.add_writer(cg_resp_writer->guid());
            n.add_writer(feedback_writer->guid());
            n.add_writer(status_writer->guid());
            publish_discovery_info();
        }
        return as;
    }

    // ----------------------------------------------------------
    // ActionClient factory
    // ----------------------------------------------------------

    /**
     * @brief Create and register an action client (full ROS 2 action protocol).
     *
     * Allocates 3 writers + 5 readers. Ensure RTPS_MAX_USER_WRITERS >= 8 and
     * RTPS_MAX_USER_READERS >= 8 when combining with other subscriptions/services.
     *
     * @param node_namespace  Node namespace.
     * @param action_name     Action name (must match the server's action_name).
     * @returns Pointer to ActionClient, or nullptr on failure.
     */
    template<typename ActionT>
    ActionClient<ActionT>* alloc_action_client(const char* node_namespace,
                                               const char* action_name,
                                               int         node_index = -1) noexcept {
        using AC = ActionClient<ActionT>;
        using traits = action_traits<ActionT>;
        static_assert(has_action_traits_v<ActionT>,
                      "milkros::action_traits specialization is required for ActionT");
        static_assert(sizeof(AC) <= MILKROS_MAX_ACTION_SIZE,
            "sizeof(ActionClient<ActionT>) exceeds MILKROS_MAX_ACTION_SIZE. "
            "Increase MILKROS_MAX_ACTION_SIZE before including ros.hpp.");

        if (action_count_ >= MILKROS_MAX_ACTIONS) return nullptr;

        auto make_rq = [&](const char* suffix) -> rtps::TopicName {
            rtps::TopicName t{};
            compose_action_subtopic(t, node_namespace, action_name, suffix, true, false);
            return t;
        };
        auto make_rr = [&](const char* suffix) -> rtps::TopicName {
            rtps::TopicName t{};
            compose_action_subtopic(t, node_namespace, action_name, suffix, false, true);
            return t;
        };
        auto make_rt = [&](const char* suffix) -> rtps::TopicName {
            rtps::TopicName t{};
            compose_action_subtopic(t, node_namespace, action_name, suffix, false, false);
            return t;
        };
        auto type_name = [&](const char* suffix) -> rtps::TopicName {
            rtps::TopicName t{};
            detail::build_type_name(t.str, sizeof(t.str),
                                     traits::action_type_name(), suffix);
            return t;
        };

        const ServiceOptions svc_opts{};

        // --- send_goal writer (client sends goal) ---
        rtps::WriterConfig sg_req_cfg{};
        sg_req_cfg.entityId = allocate_writer_entity_id();
        sg_req_cfg.topicName = make_rq("send_goalRequest");
        sg_req_cfg.typeName  = type_name("_SendGoal_Request_");
        sg_req_cfg.reliability = svc_opts.reliability;
        auto* sg_req_writer = participant_.create_writer<1>(sg_req_cfg);
        if (!sg_req_writer) return nullptr;

        // --- send_goal reader (client receives accept/reject) ---
        rtps::ReaderConfig sg_resp_cfg{};
        sg_resp_cfg.entityId = allocate_reader_entity_id();
        sg_resp_cfg.topicName = make_rr("send_goalReply");
        sg_resp_cfg.typeName  = type_name("_SendGoal_Response_");
        sg_resp_cfg.reliability = svc_opts.reliability;
        auto* sg_resp_reader = participant_.create_reader(sg_resp_cfg);
        if (!sg_resp_reader) return nullptr;

        // --- get_result writer ---
        rtps::WriterConfig gr_req_cfg{};
        gr_req_cfg.entityId = allocate_writer_entity_id();
        gr_req_cfg.topicName = make_rq("get_resultRequest");
        gr_req_cfg.typeName  = type_name("_GetResult_Request_");
        gr_req_cfg.reliability = svc_opts.reliability;
        auto* gr_req_writer = participant_.create_writer<1>(gr_req_cfg);
        if (!gr_req_writer) return nullptr;

        // --- get_result reader ---
        rtps::ReaderConfig gr_resp_cfg{};
        gr_resp_cfg.entityId = allocate_reader_entity_id();
        gr_resp_cfg.topicName = make_rr("get_resultReply");
        gr_resp_cfg.typeName  = type_name("_GetResult_Response_");
        gr_resp_cfg.reliability = svc_opts.reliability;
        auto* gr_resp_reader = participant_.create_reader(gr_resp_cfg);
        if (!gr_resp_reader) return nullptr;

        // --- cancel_goal writer ---
        rtps::WriterConfig cg_req_cfg{};
        cg_req_cfg.entityId = allocate_writer_entity_id();
        cg_req_cfg.topicName = make_rq("cancel_goalRequest");
        cg_req_cfg.typeName.assign("action_msgs::srv::dds_::CancelGoal_Request_");
        cg_req_cfg.reliability = svc_opts.reliability;
        auto* cg_req_writer = participant_.create_writer<1>(cg_req_cfg);
        if (!cg_req_writer) return nullptr;

        // --- cancel_goal reader ---
        rtps::ReaderConfig cg_resp_cfg{};
        cg_resp_cfg.entityId = allocate_reader_entity_id();
        cg_resp_cfg.topicName = make_rr("cancel_goalReply");
        cg_resp_cfg.typeName.assign("action_msgs::srv::dds_::CancelGoal_Response_");
        cg_resp_cfg.reliability = svc_opts.reliability;
        auto* cg_resp_reader = participant_.create_reader(cg_resp_cfg);
        if (!cg_resp_reader) return nullptr;

        // --- feedback reader ---
        rtps::ReaderConfig fb_cfg{};
        fb_cfg.entityId = allocate_reader_entity_id();
        fb_cfg.topicName = make_rt("_action/feedback");
        fb_cfg.typeName  = type_name("_FeedbackMessage_");
        fb_cfg.reliability = rtps::ReliabilityKind::BEST_EFFORT;
        auto* feedback_reader = participant_.create_reader(fb_cfg);
        if (!feedback_reader) return nullptr;

        // --- status reader ---
        rtps::ReaderConfig st_cfg{};
        st_cfg.entityId = allocate_reader_entity_id();
        st_cfg.topicName = make_rt("_action/status");
        st_cfg.typeName.assign("action_msgs::msg::dds_::GoalStatusArray_");
        st_cfg.reliability = rtps::ReliabilityKind::BEST_EFFORT;
        auto* status_reader = participant_.create_reader(st_cfg);
        if (!status_reader) return nullptr;

        AC* ac = new (action_storage_[action_count_]) AC(
            sg_req_writer, sg_resp_reader,
            gr_req_writer, gr_resp_reader,
            cg_req_writer, cg_resp_reader,
            feedback_reader, status_reader);

        action_ifaces_[action_count_++] = ac;
        if (node_index >= 0 && node_index < static_cast<int>(graph_.node_count)) {
            NodeEntitiesInfo& n = graph_.nodes[node_index];
            n.add_writer(sg_req_writer->guid());
            n.add_reader(sg_resp_reader->guid());
            n.add_writer(gr_req_writer->guid());
            n.add_reader(gr_resp_reader->guid());
            n.add_writer(cg_req_writer->guid());
            n.add_reader(cg_resp_reader->guid());
            n.add_reader(feedback_reader->guid());
            n.add_reader(status_reader->guid());
            publish_discovery_info();
        }
        return ac;
    }

protected:
    // ----------------------------------------------------------
    // Helpers for subclass tick() implementations
    // ----------------------------------------------------------

    void spin_transport(rtps::ITransportRuntime& runtime) noexcept {
        runtime.spin_some(rx_buffer_, options_.rx_buffer_size, participant_);
    }

    void tick_participant(uint32_t now_ms) noexcept {
        participant_.tick(now_ms);
    }

    /// Drive all registered timers and action server ticks.
    /// Call this from your tick() override.
    void tick_timers(uint32_t now_ms) noexcept {
        for (uint8_t i = 0u; i < timer_count_; ++i) {
            timers_[i]->tick(now_ms);
        }
        for (uint8_t i = 0u; i < action_count_; ++i) {
            action_ifaces_[i]->tick(now_ms);
        }
    }

private:
    // ----------------------------------------------------------
    // publish_discovery_info() — encode and enqueue a ros_discovery_info update.
    // ----------------------------------------------------------
    void publish_discovery_info() noexcept {
        if (!disco_writer_) return;

        rtps::BufRef payload;
        rtps::CacheChange* ch = disco_writer_->new_change(
            rtps::ChangeKind::ALIVE, payload);
        if (!ch) return;

        const uint32_t written = encode_participant_entities_info(
            graph_, ch->payload.data, ch->payload.capacity());
        if (written == 0u) {
            // Encoded size exceeds pool block — graph_ state is too large.
            // The staging buffer will be abandoned; tick() will not transmit it.
            return;
        }
        ch->payload.size = written;
        disco_writer_->add_change(static_cast<rtps::CacheChange&&>(*ch));
    }

    // ----------------------------------------------------------
    // Data members
    // ----------------------------------------------------------

    ContextOptions options_{};
    rtps::ITransport& transport_;
    rtps::RTPSParticipant participant_;
    bool    started_        = false;
    uint8_t next_writer_id_ = 1u;
    uint8_t next_reader_id_ = 1u;
    uint8_t rx_buffer_[1536] = {};

    // ---- ros_discovery_info graph state ----
    ParticipantEntitiesInfo        graph_        = {};
    rtps::StatefulWriter<1>*       disco_writer_ = nullptr;

    // ---- Timer pool ----
    alignas(alignof(max_align_t))
    uint8_t timer_storage_[MILKROS_MAX_TIMERS][MILKROS_MAX_TIMER_SIZE] = {};
    ITimer* timers_[MILKROS_MAX_TIMERS] = {};
    uint8_t timer_count_ = 0u;

    // ---- Service pool ----
    alignas(alignof(max_align_t))
    uint8_t service_storage_[MILKROS_MAX_SERVICES][MILKROS_MAX_SERVICE_SIZE] = {};
    IService* services_[MILKROS_MAX_SERVICES] = {};
    uint8_t service_count_ = 0u;

    // ---- Client pool ----
    alignas(alignof(max_align_t))
    uint8_t client_storage_[MILKROS_MAX_CLIENTS][MILKROS_MAX_CLIENT_SIZE] = {};
    IService* clients_[MILKROS_MAX_CLIENTS] = {};
    uint8_t client_count_ = 0u;

    // ---- Action pool (servers + clients share the same pool) ----
    alignas(alignof(max_align_t))
    uint8_t action_storage_[MILKROS_MAX_ACTIONS][MILKROS_MAX_ACTION_SIZE] = {};
    IAction* action_ifaces_[MILKROS_MAX_ACTIONS] = {};
    uint8_t action_count_ = 0u;
};

// ----------------------------------------------------------
// Node inline method implementations (deferred — Node sees Context)
// ----------------------------------------------------------

inline Node::Node(Context& context,
                  const char* name,
                  const char* node_namespace,
                  int         node_index) noexcept
    : context_(context)
    , node_index_(node_index) {
    if (name) {
        strncpy(name_, name, sizeof(name_) - 1u);
        name_[sizeof(name_) - 1u] = '\0';
    }
    if (node_namespace) {
        strncpy(namespace_, node_namespace, sizeof(namespace_) - 1u);
        namespace_[sizeof(namespace_) - 1u] = '\0';
    }
}

template<typename MessageT>
Publisher<MessageT> Node::create_publisher(const char* topic,
                                           const PublisherOptions& options) noexcept {
    return context_.create_publisher<MessageT>(namespace_, topic, options, node_index_);
}

template<typename MessageT, typename CallbackT>
Subscription<MessageT, CallbackT> Node::create_subscription(
    const char* topic,
    CallbackT callback,
    const SubscriptionOptions& options) noexcept {
    return context_.create_subscription<MessageT>(namespace_,
                                                  topic,
                                                  callback,
                                                  options,
                                                  node_index_);
}

template<typename CallbackT>
Timer<CallbackT>* Node::create_timer(uint32_t period_ms, CallbackT callback) noexcept {
    return context_.alloc_timer(period_ms, static_cast<CallbackT&&>(callback));
}

template<typename ServiceT, typename HandlerT>
Service<ServiceT, HandlerT>*
Node::create_service(const char* service_name,
                      HandlerT    handler,
                      const ServiceOptions& options) noexcept {
    return context_.alloc_service<ServiceT>(namespace_, service_name,
                                             static_cast<HandlerT&&>(handler),
                                             options,
                                             node_index_);
}

template<typename ServiceT, uint8_t MaxPending>
Client<ServiceT, MaxPending>*
Node::create_client(const char* service_name,
                     const ServiceOptions& options) noexcept {
    return context_.alloc_client<ServiceT, MaxPending>(namespace_, service_name, options,
                                                        node_index_);
}

template<typename ActionT, typename GoalCbT, typename ExecuteCbT>
ActionServer<ActionT, GoalCbT, ExecuteCbT>*
Node::create_action_server(const char* action_name,
                             GoalCbT    goal_cb,
                             ExecuteCbT execute_cb) noexcept {
    return context_.alloc_action_server<ActionT>(namespace_, action_name,
                                                  static_cast<GoalCbT&&>(goal_cb),
                                                  static_cast<ExecuteCbT&&>(execute_cb),
                                                  node_index_);
}

template<typename ActionT>
ActionClient<ActionT>*
Node::create_action_client(const char* action_name) noexcept {
    return context_.alloc_action_client<ActionT>(namespace_, action_name, node_index_);
}

} // namespace milkros
