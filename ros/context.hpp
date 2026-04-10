#pragma once

#include "node.hpp"
#include "rtps/participant.hpp"
#include "rtps/spdp.hpp"
#include "rtps/transport.hpp"

#include <stdint.h>
#include <string.h>

namespace milkros {

/**
 * @brief Configuration for a communication context.
 *
 * Conceptually this is closest to a very small embedded analogue of
 * `rclcpp::Context`: it owns the RTPS participant and endpoint allocation
 * for one ROS communication domain.
 *
 * It is intentionally not a full ROS 2 context implementation. There is
 * no init/shutdown graph, allocator plumbing, or executor coupling yet.
 * Scheduling is intentionally left to the application so an RTOS task or
 * bare-metal loop can decide when transport polling and participant timers
 * should run.
 */
struct ContextOptions {
    uint8_t domain_id = 0u;
    uint8_t participant_id = 0u;
    uint32_t rx_buffer_size = 1536u;
};

struct PublisherOptions {
    rtps::ReliabilityKind reliability = rtps::ReliabilityKind::BEST_EFFORT;
    rtps::DurabilityKind durability = rtps::DurabilityKind::VOLATILE;
    rtps::TopicKind topic_kind = rtps::TopicKind::NO_KEY;
};

struct SubscriptionOptions {
    rtps::ReliabilityKind reliability = rtps::ReliabilityKind::BEST_EFFORT;
    rtps::DurabilityKind durability = rtps::DurabilityKind::VOLATILE;
    rtps::TopicKind topic_kind = rtps::TopicKind::NO_KEY;
    uint32_t heartbeat_response_ms = RTPS_HEARTBEAT_RESPONSE_DELAY_MS;
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

    bool start() noexcept {
        if (started_) return true;
        started_ = true;
        participant_.announce();
        return true;
    }

    virtual void tick(uint32_t now_ms) noexcept = 0;

    void announce() noexcept {
        if (started_) participant_.announce();
    }

    Node create_node(const char* name,
                     const char* node_namespace = "") noexcept {
        return Node(*this, name, node_namespace);
    }

    rtps::RTPSParticipant& participant() noexcept { return participant_; }

    rtps::EntityId allocate_writer_entity_id() noexcept {
        return rtps::EntityId{{0x00, 0x10, next_writer_id_++}, 0x02};
    }

    rtps::EntityId allocate_reader_entity_id() noexcept {
        return rtps::EntityId{{0x00, 0x20, next_reader_id_++}, 0x07};
    }

    template<typename MessageT>
    Publisher<MessageT> create_publisher(const char* node_namespace,
                                         const char* topic,
                                         const PublisherOptions& options) noexcept {
        using traits = message_traits<MessageT>;
        static_assert(has_message_traits_v<MessageT>,
                      "milkros::message_traits specialization is required");

        rtps::WriterConfig cfg{};
        cfg.entityId = allocate_writer_entity_id();
        compose_topic_name(cfg.topicName, node_namespace, topic);
        cfg.typeName.assign(traits::dds_type_name());
        cfg.reliability = options.reliability;
        cfg.durability = options.durability;
        cfg.topicKind = options.topic_kind;

        auto* writer = participant_.create_writer<1>(cfg);
        return Publisher<MessageT>(writer);
    }

    template<typename MessageT, typename CallbackT>
    Subscription<MessageT, CallbackT> create_subscription(
        const char* node_namespace,
        const char* topic,
        CallbackT callback,
        const SubscriptionOptions& options) noexcept {
        using traits = message_traits<MessageT>;
        static_assert(has_message_traits_v<MessageT>,
                      "milkros::message_traits specialization is required");

        rtps::ReaderConfig cfg{};
        cfg.entityId = allocate_reader_entity_id();
        compose_topic_name(cfg.topicName, node_namespace, topic);
        cfg.typeName.assign(traits::dds_type_name());
        cfg.reliability = options.reliability;
        cfg.durability = options.durability;
        cfg.topicKind = options.topic_kind;
        cfg.hbResponseMs = options.heartbeat_response_ms;

        auto* reader = participant_.create_reader(cfg);
        return Subscription<MessageT, CallbackT>(reader, callback);
    }

protected:
    void spin_transport(rtps::ITransportRuntime& runtime) noexcept {
        runtime.spin_some(rx_buffer_, options_.rx_buffer_size, participant_);
    }

    void tick_participant(uint32_t now_ms) noexcept {
        participant_.tick(now_ms);
    }

private:
    static void append_segment(char* dst,
                               size_t cap,
                               size_t& pos,
                               const char* src) noexcept {
        if (!dst || cap == 0u || !src) return;
        while (pos + 1u < cap && *src) {
            dst[pos++] = *src++;
        }
        dst[pos] = '\0';
    }

    static void append_normalized_path(char* dst,
                                       size_t cap,
                                       size_t& pos,
                                       const char* src) noexcept {
        if (!src) return;

        while (*src == '/') ++src;
        while (*src && pos + 1u < cap) {
            if (*src == '/' && (pos == 0u || dst[pos - 1u] == '/')) {
                ++src;
                continue;
            }
            dst[pos++] = *src++;
        }

        while (pos > 0u && dst[pos - 1u] == '/') {
            --pos;
        }
        dst[pos] = '\0';
    }

    static void compose_topic_name(rtps::TopicName& out,
                                   const char* node_namespace,
                                   const char* topic) noexcept {
        if (!topic) {
            out.assign("rt/unnamed");
            return;
        }

        if (strncmp(topic, "rt/", 3u) == 0) {
            out.assign(topic);
            return;
        }

        char full[RTPS_MAX_TOPIC_NAME_LEN] = {};
        size_t pos = 0u;
        append_segment(full, sizeof(full), pos, "rt/");

        if (node_namespace && node_namespace[0] != '\0' &&
            !(node_namespace[0] == '/' && node_namespace[1] == '\0')) {
            append_normalized_path(full, sizeof(full), pos, node_namespace);
            if (pos + 1u < sizeof(full) && pos > 0u && full[pos - 1u] != '/') {
                full[pos++] = '/';
                full[pos] = '\0';
            }
        }

        append_normalized_path(full, sizeof(full), pos, topic);
        out.assign(full);
    }

    ContextOptions options_{};
    rtps::ITransport& transport_;
    rtps::RTPSParticipant participant_;
    bool started_ = false;
    uint8_t next_writer_id_ = 1u;
    uint8_t next_reader_id_ = 1u;
    uint8_t rx_buffer_[1536] = {};
};

inline Node::Node(Context& context,
                  const char* name,
                  const char* node_namespace) noexcept
    : context_(context) {
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
    return context_.create_publisher<MessageT>(namespace_, topic, options);
}

template<typename MessageT, typename CallbackT>
Subscription<MessageT, CallbackT> Node::create_subscription(
    const char* topic,
    CallbackT callback,
    const SubscriptionOptions& options) noexcept {
    return context_.create_subscription<MessageT>(namespace_,
                                                  topic,
                                                  callback,
                                                  options);
}

} // namespace milkros
