/**
 * @file test_service_reply_depth.cpp
 * @brief A service reply writer holds RMW_ROSETTE_REPLY_DEPTH replies in flight.
 *
 * A service and its clients (as many as RMW_ROSETTE_REPLY_DEPTH and
 * RMW_ROSETTE_MAX_CLIENTS allow) share one rmw_rosette context over real UDP
 * loopback, using the generated std_srvs/SetBool type support. Answering
 * RMW_ROSETTE_REPLY_DEPTH requests back to back puts up to that many replies
 * in flight at once, as concurrent requests to a busy service do. Where the
 * executor runs the participant (no pump thread), replies sent between two
 * waits stay unacknowledged until the next one, so all of them are in flight
 * together; on a host the pump thread may deliver and acknowledge some before
 * the last is sent, and the history then holds the rest.
 *
 * Verification:
 * - Correct means: the reply writer's depth and the depth it reports are
 *   RMW_ROSETTE_REPLY_DEPTH whatever the service's QoS requests, and that
 *   many replies in flight at once all reach their clients and then leave
 *   the history.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_send_response`,
 *   `rmw_service_response_publisher_get_actual_qos`
 * - OMG DDSI-RTPS 2.5 §8.3.3.1 (KEEP_LAST), §8.4.7 (reliable delivery)
 */

#include <gtest/gtest.h>

#include "rcl_default_init_options.hpp"

#include <algorithm>

#include <rmw/rmw.h>

#include <std_srvs/srv/set_bool.h>

#include "rmw_rosette/types.hpp"

namespace {

/// Replies the service holds at once: one request each, so every reply is in
/// flight before any is delivered.
constexpr int kRequests = RMW_ROSETTE_REPLY_DEPTH;

/// Clients of the service. Client entities are a separate capacity
/// (RMW_ROSETTE_MAX_CLIENTS), so a reply depth above it is reached by
/// sending several requests from each client rather than by adding clients.
constexpr int kClients = kRequests < RMW_ROSETTE_MAX_CLIENTS ? kRequests : RMW_ROSETTE_MAX_CLIENTS;

/// One context with a SetBool service and kClients clients of it.
class ServiceReplyDepth : public ::testing::Test {
protected:
    void SetUp() override {
        rmw_init_options_t options = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&options), RMW_RET_OK);
        ASSERT_EQ(rmw_init(&options, &context_), RMW_RET_OK);
        ASSERT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
        node_ = rmw_create_node(&context_, "reply_depth_test", "/");
        ASSERT_NE(node_, nullptr);
        wait_set_ = rmw_create_wait_set(&context_, kClients + 1);
        ASSERT_NE(wait_set_, nullptr);
    }

    void TearDown() override {
        for (rmw_client_t* client : clients_) {
            if (client != nullptr) {
                EXPECT_EQ(rmw_destroy_client(node_, client), RMW_RET_OK);
            }
        }

        if (service_ != nullptr) {
            EXPECT_EQ(rmw_destroy_service(node_, service_), RMW_RET_OK);
        }

        EXPECT_EQ(rmw_destroy_wait_set(wait_set_), RMW_RET_OK);
        EXPECT_EQ(rmw_destroy_node(node_), RMW_RET_OK);
        EXPECT_EQ(rmw_shutdown(&context_), RMW_RET_OK);
        EXPECT_EQ(rmw_context_fini(&context_), RMW_RET_OK);
    }

    const rosidl_service_type_support_t* type_support() const {
        return ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);
    }

    /// Runs discovery and delivery for 100 ms.
    void pump() {
        rmw_time_t      t{0, 100000000};
        const rmw_ret_t ret = rmw_wait(nullptr, nullptr, nullptr, nullptr, nullptr, wait_set_, &t);
        EXPECT_TRUE(ret == RMW_RET_OK || ret == RMW_RET_TIMEOUT);
    }

    /// The service's rmw_rosette state.
    const rmw_rosette::Service& service() const {
        return *static_cast<const rmw_rosette::Service*>(service_->data);
    }

    /// Requests the service's request ring keeps between takes: the KEEP_LAST
    /// depth of its actual QoS, at most RMW_ROSETTE_MAX_PENDING_CHANGES.
    int request_ring_depth() const { return static_cast<int>(service().requests.depth()); }

    /// Replies the service's writer holds, read under the participant lock
    /// because the pump thread (on a host) changes the history concurrently.
    size_t replies_held() const {
        rosette::platform::LockGuard guard(service().node->context->driver->mutex());
        return service().reply_writer->cache().size();
    }

    /// Whether the participant pump thread runs this context: built in, and
    /// not turned off by RMW_ROSETTE_PUMP_THREAD=0 in the environment.
    bool pump_running() const {
#if RMW_ROSETTE_PUMP_THREAD
        return service().node->context->pump.running();
#else
        return false;
#endif
    }

    /// The reply writer's history capacity; fixed when the service is created.
    size_t reply_capacity() const { return service().reply_writer->cache().capacity(); }

    rmw_context_t   context_ = rmw_get_zero_initialized_context();
    rmw_node_t*     node_ = nullptr;
    rmw_service_t*  service_ = nullptr;
    rmw_client_t*   clients_[kClients] = {};
    rmw_wait_set_t* wait_set_ = nullptr;
};

}  // namespace

TEST_F(ServiceReplyDepth, ReplyDepthIgnoresRequestedQos) {
    rmw_qos_profile_t qos = rmw_qos_profile_services_default;
    qos.depth = 1;
    service_ = rmw_create_service(node_, type_support(), "/reply_depth_qos", &qos);
    ASSERT_NE(service_, nullptr);

    rmw_qos_profile_t actual{};
    ASSERT_EQ(rmw_service_response_publisher_get_actual_qos(service_, &actual), RMW_RET_OK);
    EXPECT_EQ(actual.depth, static_cast<size_t>(RMW_ROSETTE_REPLY_DEPTH));
    EXPECT_EQ(actual.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    EXPECT_EQ(actual.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    EXPECT_EQ(reply_capacity(), RMW_ROSETTE_REPLY_DEPTH);
}

TEST_F(ServiceReplyDepth, EveryReplyInFlightArrives) {
    const rmw_qos_profile_t qos = rmw_qos_profile_services_default;
    service_ = rmw_create_service(node_, type_support(), "/reply_depth", &qos);
    ASSERT_NE(service_, nullptr);
    for (rmw_client_t*& client : clients_) {
        client = rmw_create_client(node_, type_support(), "/reply_depth", &qos);
        ASSERT_NE(client, nullptr);
    }

    for (rmw_client_t* client : clients_) {
        bool available = false;
        for (int i = 0; i < 50 && !available; ++i) {
            pump();
            ASSERT_EQ(rmw_service_server_is_available(node_, client, &available), RMW_RET_OK);
        }

        ASSERT_TRUE(available) << "a client and the service never matched";
    }

    // Request r goes out from client r % kClients, so the clients share the
    // kRequests replies the service will hold, and each client's count of
    // replies is known. The service's request reader keeps only as many
    // requests between takes as its request ring is deep (the depth of the
    // service's QoS, within RMW_ROSETTE_MAX_PENDING_CHANGES), so they go out
    // and are taken in waves of that size; no reply is sent until every
    // request is taken, which is what puts them all in flight together.
    int                            expected[kClients] = {};
    std_srvs__srv__SetBool_Request req{};
    std_srvs__srv__SetBool_Request__init(&req);
    rmw_service_info_t headers[kRequests] = {};
    int                taken_requests = 0;
    const int          wave_size = request_ring_depth();
    ASSERT_GE(wave_size, 1);
    for (int sent = 0; sent < kRequests;) {
        const int wave_end = std::min(kRequests, sent + wave_size);
        for (; sent < wave_end; ++sent) {
            int64_t sequence = 0;
            ASSERT_EQ(rmw_send_request(clients_[sent % kClients], &req, &sequence), RMW_RET_OK);
            ++expected[sent % kClients];
        }

        for (int i = 0; i < 50 && taken_requests < wave_end; ++i) {
            pump();
            bool taken = true;
            while (taken && taken_requests < wave_end) {
                ASSERT_EQ(rmw_take_request(service_, &headers[taken_requests], &req, &taken),
                          RMW_RET_OK);
                taken_requests += taken ? 1 : 0;
            }
        }
    }

    std_srvs__srv__SetBool_Request__fini(&req);
    ASSERT_EQ(taken_requests, kRequests) << "not every request arrived";

    std_srvs__srv__SetBool_Response res{};
    std_srvs__srv__SetBool_Response__init(&res);
    res.success = true;
    for (rmw_service_info_t& header : headers) {
        ASSERT_EQ(rmw_send_response(service_, &header.request_id, &res), RMW_RET_OK);
    }

    // With the pump thread running, any number of the replies may already be
    // acknowledged here; the delivery count below is what shows that none was
    // evicted. Without it nothing runs the participant between these calls.
    if (!pump_running()) {
        EXPECT_EQ(replies_held(), RMW_ROSETTE_REPLY_DEPTH)
            << "every reply is held until its client acknowledges it";
    }

    int received[kClients] = {};
    int replies = 0;
    for (int i = 0; i < 50 && replies < kRequests; ++i) {
        pump();
        for (int c = 0; c < kClients; ++c) {
            bool taken = true;
            while (taken) {
                rmw_service_info_t reply_header{};
                ASSERT_EQ(rmw_take_response(clients_[c], &reply_header, &res, &taken), RMW_RET_OK);
                if (taken) {
                    EXPECT_TRUE(res.success);
                    ++received[c];
                    ++replies;
                }
            }
        }
    }

    std_srvs__srv__SetBool_Response__fini(&res);
    EXPECT_EQ(replies, kRequests) << "a reply in flight was lost";
    for (int c = 0; c < kClients; ++c) {
        EXPECT_EQ(received[c], expected[c]) << "client " << c << " got the wrong number of replies";
    }

    for (int i = 0; i < 20 && replies_held() != 0u; ++i) {
        pump();
    }

    EXPECT_EQ(replies_held(), 0u) << "acknowledged replies stay in the history";
}
