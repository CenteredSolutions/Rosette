// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_service_departed_client.cpp
 * @brief rmw_send_response() replies only to a client that is still there.
 *
 * A service and a client share one rmw_rosette context over real UDP
 * loopback, using the generated std_srvs/SetBool type support. The client's
 * request is taken, then the response is sent either with the client still
 * matched or after the client is destroyed, the order an rclcpp deferred
 * response or a multithreaded executor allows.
 *
 * Verification:
 * - Correct means: a live client receives its reply; a reply to a client
 *   whose request writer has unmatched succeeds and leaves nothing in the
 *   reply writer's history, so it is neither held nor replayed to a later
 *   client.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_send_response`
 * - OMG DDS-RPC 1.0 §7.8.2 (related sample identity)
 */

#include <gtest/gtest.h>

#include "rcl_default_init_options.hpp"

#include <rmw/rmw.h>

#include <std_srvs/srv/set_bool.h>

#include "rmw_rosette/types.hpp"

namespace {

/// One context with a SetBool service and a client of it, matched to each other.
class ServiceDepartedClient : public ::testing::Test {
protected:
    void SetUp() override {
        rmw_init_options_t options = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&options), RMW_RET_OK);
        ASSERT_EQ(rmw_init(&options, &context_), RMW_RET_OK);
        ASSERT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
        node_ = rmw_create_node(&context_, "departed_client_test", "/");
        ASSERT_NE(node_, nullptr);

        const rosidl_service_type_support_t* ts =
            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);
        const rmw_qos_profile_t qos = rmw_qos_profile_services_default;
        service_ = rmw_create_service(node_, ts, "/departed_client", &qos);
        ASSERT_NE(service_, nullptr);
        client_ = rmw_create_client(node_, ts, "/departed_client", &qos);
        ASSERT_NE(client_, nullptr);
        wait_set_ = rmw_create_wait_set(&context_, 4);
        ASSERT_NE(wait_set_, nullptr);

        bool available = false;
        for (int i = 0; i < 50 && !available; ++i) {
            pump();
            ASSERT_EQ(rmw_service_server_is_available(node_, client_, &available), RMW_RET_OK);
        }

        ASSERT_TRUE(available) << "client and service never matched";
    }

    void TearDown() override {
        if (client_ != nullptr) {
            EXPECT_EQ(rmw_destroy_client(node_, client_), RMW_RET_OK);
        }

        EXPECT_EQ(rmw_destroy_wait_set(wait_set_), RMW_RET_OK);
        EXPECT_EQ(rmw_destroy_service(node_, service_), RMW_RET_OK);
        EXPECT_EQ(rmw_destroy_node(node_), RMW_RET_OK);
        EXPECT_EQ(rmw_shutdown(&context_), RMW_RET_OK);
        EXPECT_EQ(rmw_context_fini(&context_), RMW_RET_OK);
    }

    /// Runs discovery and delivery for 100 ms.
    void pump() {
        rmw_time_t t{0, 100000000};
        const rmw_ret_t ret = rmw_wait(nullptr, nullptr, nullptr, nullptr, nullptr, wait_set_, &t);
        EXPECT_TRUE(ret == RMW_RET_OK || ret == RMW_RET_TIMEOUT);
    }

    /// Sends a request from the client and takes it at the service.
    void request(rmw_service_info_t& header) {
        std_srvs__srv__SetBool_Request req{};
        std_srvs__srv__SetBool_Request__init(&req);
        req.data = true;
        int64_t sequence = 0;
        ASSERT_EQ(rmw_send_request(client_, &req, &sequence), RMW_RET_OK);

        void* services[] = {service_->data};
        rmw_services_t ready{1, services};
        rmw_time_t timeout{5, 0};
        ASSERT_EQ(rmw_wait(nullptr, nullptr, &ready, nullptr, nullptr, wait_set_, &timeout),
                  RMW_RET_OK);
        ASSERT_NE(ready.services[0], nullptr) << "the request never arrived";

        std_srvs__srv__SetBool_Request taken_req{};
        std_srvs__srv__SetBool_Request__init(&taken_req);
        bool taken = false;
        ASSERT_EQ(rmw_take_request(service_, &header, &taken_req, &taken), RMW_RET_OK);
        ASSERT_TRUE(taken);
        std_srvs__srv__SetBool_Request__fini(&taken_req);
        std_srvs__srv__SetBool_Request__fini(&req);
    }

    /// Sends the response to the request `header` identifies.
    rmw_ret_t respond(rmw_service_info_t& header) {
        std_srvs__srv__SetBool_Response res{};
        std_srvs__srv__SetBool_Response__init(&res);
        res.success = true;
        const rmw_ret_t ret = rmw_send_response(service_, &header.request_id, &res);
        std_srvs__srv__SetBool_Response__fini(&res);
        return ret;
    }

    /// The service's request reader matches this many client writers, read
    /// under the participant lock because the pump thread (on a host) removes
    /// a departed writer concurrently.
    size_t matched_request_writers() const {
        const auto* impl = static_cast<const rmw_rosette::Service*>(service_->data);
        rosette::platform::LockGuard guard(impl->node->context->driver->mutex());
        return impl->request_reader->matched_writer_count();
    }

    /// Whether the service's reply writer holds no reply, read under the
    /// participant lock for the same reason.
    bool reply_history_empty() const {
        const auto* impl = static_cast<const rmw_rosette::Service*>(service_->data);
        rosette::platform::LockGuard guard(impl->node->context->driver->mutex());
        return impl->reply_writer->cache().is_empty();
    }

    rmw_context_t context_ = rmw_get_zero_initialized_context();
    rmw_node_t* node_ = nullptr;
    rmw_service_t* service_ = nullptr;
    rmw_client_t* client_ = nullptr;
    rmw_wait_set_t* wait_set_ = nullptr;
};

}  // namespace

TEST_F(ServiceDepartedClient, LiveClientReceivesItsReply) {
    rmw_service_info_t header{};
    request(header);
    ASSERT_EQ(respond(header), RMW_RET_OK);

    void* clients[] = {client_->data};
    rmw_clients_t ready{1, clients};
    rmw_time_t timeout{5, 0};
    ASSERT_EQ(rmw_wait(nullptr, nullptr, nullptr, &ready, nullptr, wait_set_, &timeout),
              RMW_RET_OK);
    ASSERT_NE(ready.clients[0], nullptr) << "the reply never arrived";

    std_srvs__srv__SetBool_Response res{};
    std_srvs__srv__SetBool_Response__init(&res);
    rmw_service_info_t reply_header{};
    bool taken = false;
    ASSERT_EQ(rmw_take_response(client_, &reply_header, &res, &taken), RMW_RET_OK);
    EXPECT_TRUE(taken);
    EXPECT_TRUE(res.success);
    std_srvs__srv__SetBool_Response__fini(&res);
}

TEST_F(ServiceDepartedClient, ReplyToDepartedClientLeavesNothing) {
    rmw_service_info_t header{};
    request(header);

    ASSERT_EQ(rmw_destroy_client(node_, client_), RMW_RET_OK);
    client_ = nullptr;
    for (int i = 0; i < 50 && matched_request_writers() > 0u; ++i) {
        pump();
    }

    ASSERT_EQ(matched_request_writers(), 0u) << "the client never unmatched";

    EXPECT_EQ(respond(header), RMW_RET_OK);
    EXPECT_TRUE(reply_history_empty()) << "a reply to a departed client is held";
}
