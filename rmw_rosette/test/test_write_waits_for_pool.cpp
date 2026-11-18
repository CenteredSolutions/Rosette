// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_write_waits_for_pool.cpp
 * @brief A reliable write waits for a pool block instead of failing at once.
 *
 * A SetBool service and client share one rmw_rosette context over real UDP
 * loopback. The test takes every pool block a SetBool sample could use, as
 * unacknowledged large replies do on a board with few large blocks, and
 * then writes:
 *
 * | Case | Writer                 | Blocks come back       | Write returns            |
 * |------|------------------------|------------------------|--------------------------|
 * | 001  | reply (RELIABLE)       | after kHoldMs          | OK before the deadline   |
 * | 002  | reply (RELIABLE)       | never during the call  | ERROR after the deadline |
 * | 003  | request (BEST_EFFORT)  | never during the call  | ERROR at once            |
 *
 * Verification:
 * - Correct means: as serialize_for_write(); case 001 also delivers the
 *   reply to the client, so the request that found the pool exhausted is
 *   answered.
 *
 * References:
 * - OMG DDS 1.4 §2.2.2.4.2.11, §2.2.3.14 (RELIABILITY `max_blocking_time`)
 * - rmw/rmw.h (Humble), `rmw_send_response`, `rmw_send_request`
 */

#include <gtest/gtest.h>

#include "rcl_default_init_options.hpp"

#include <chrono>
#include <thread>
#include <vector>

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <std_srvs/srv/set_bool.h>

#include "rmw_rosette/serialize_for_write.hpp"
#include "rmw_rosette/types.hpp"

namespace {

namespace rtps = rosette::rtps;

/// How long case 001 keeps the pool exhausted; well inside the deadline.
constexpr int kHoldMs = 30;

/// Milliseconds since `start`.
int64_t elapsed_ms(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
                                                                 - start)
        .count();
}

/// Every pool block alloc() hands out for a small sample, held until release().
class PoolHoard {
public:
    /// Takes blocks until the pool refuses one big enough for a SetBool sample.
    void take_all() {
        for (;;) {
            rtps::BufRef block = rtps::buf_alloc(1u);
            if (!block.valid()) {
                break;
            }

            blocks_.push_back(static_cast<rtps::BufRef&&>(block));
        }

        ASSERT_FALSE(rtps::buf_alloc(256u).valid()) << "the pool still has a block to encode into";
    }

    void release() { blocks_.clear(); }

private:
    std::vector<rtps::BufRef> blocks_;
};

/// One context with a SetBool service and one client of it.
class WriteWaitsForPool : public ::testing::Test {
protected:
    void SetUp() override {
        rmw_init_options_t options = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&options), RMW_RET_OK);
        ASSERT_EQ(rmw_init(&options, &context_), RMW_RET_OK);
        ASSERT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
        node_ = rmw_create_node(&context_, "write_waits_for_pool_test", "/");
        ASSERT_NE(node_, nullptr);
        wait_set_ = rmw_create_wait_set(&context_, 2);
        ASSERT_NE(wait_set_, nullptr);
        std_srvs__srv__SetBool_Request__init(&request_);
        std_srvs__srv__SetBool_Response__init(&response_);
    }

    void TearDown() override {
        hoard_.release();
        std_srvs__srv__SetBool_Request__fini(&request_);
        std_srvs__srv__SetBool_Response__fini(&response_);
        if (client_ != nullptr) {
            EXPECT_EQ(rmw_destroy_client(node_, client_), RMW_RET_OK);
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

    /// Creates the service and a client with `client_qos`.
    void create(const char* name, const rmw_qos_profile_t& client_qos) {
        const rmw_qos_profile_t qos = rmw_qos_profile_services_default;
        service_ = rmw_create_service(node_, type_support(), name, &qos);
        ASSERT_NE(service_, nullptr);
        client_ = rmw_create_client(node_, type_support(), name, &client_qos);
        ASSERT_NE(client_, nullptr);
    }

    /// Creates a reliable service and client and waits for them to match.
    void create_matched(const char* name) {
        create(name, rmw_qos_profile_services_default);
        bool available = false;
        for (int i = 0; i < 50 && !available; ++i) {
            pump();
            ASSERT_EQ(rmw_service_server_is_available(node_, client_, &available), RMW_RET_OK);
        }

        ASSERT_TRUE(available) << "the client and the service never matched";
    }

    /// Sends one request and takes it at the service.
    void take_one_request(rmw_service_info_t& header) {
        int64_t sequence = 0;
        ASSERT_EQ(rmw_send_request(client_, &request_, &sequence), RMW_RET_OK);
        bool taken = false;
        for (int i = 0; i < 50 && !taken; ++i) {
            pump();
            ASSERT_EQ(rmw_take_request(service_, &header, &request_, &taken), RMW_RET_OK);
        }

        ASSERT_TRUE(taken) << "the request never arrived";
    }

    rmw_context_t                   context_ = rmw_get_zero_initialized_context();
    rmw_node_t*                     node_ = nullptr;
    rmw_service_t*                  service_ = nullptr;
    rmw_client_t*                   client_ = nullptr;
    rmw_wait_set_t*                 wait_set_ = nullptr;
    std_srvs__srv__SetBool_Request  request_{};
    std_srvs__srv__SetBool_Response response_{};
    PoolHoard                       hoard_;
};

}  // namespace

// TC-RMW-WRITE-001: a reply that finds the pool exhausted is sent once a block
// comes back, and the client receives it.
TEST_F(WriteWaitsForPool, ReplyWaitsForReleasedBlock) {
    create_matched("/write_waits_reply");
    rmw_service_info_t header{};
    take_one_request(header);

    hoard_.take_all();
    const auto  start = std::chrono::steady_clock::now();
    std::thread releaser([this] {
        std::this_thread::sleep_for(std::chrono::milliseconds(kHoldMs));
        hoard_.release();
        // Released outside a protocol pass, as by another thread's take, so
        // the waiting write sees it only once the driver is woken.
        context_.impl->rosette_context.driver->wake();
    });
    response_.success = true;
    const rmw_ret_t ret = rmw_send_response(service_, &header.request_id, &response_);
    const int64_t   waited_ms = elapsed_ms(start);
    releaser.join();

    ASSERT_EQ(ret, RMW_RET_OK) << rmw_get_error_string().str;
    EXPECT_LT(waited_ms, static_cast<int64_t>(RMW_ROSETTE_MAX_BLOCKING_TIME_MS));

    bool taken = false;
    for (int i = 0; i < 50 && !taken; ++i) {
        pump();
        rmw_service_info_t reply_header{};
        response_.success = false;
        ASSERT_EQ(rmw_take_response(client_, &reply_header, &response_, &taken), RMW_RET_OK);
    }

    ASSERT_TRUE(taken) << "the reply that waited never reached the client";
    EXPECT_TRUE(response_.success);
}

// TC-RMW-WRITE-002: a reliable write fails only after
// RMW_ROSETTE_MAX_BLOCKING_TIME_MS without a block.
TEST_F(WriteWaitsForPool, ReliableWriteFailsAfterDeadline) {
    create_matched("/write_waits_deadline");
    rmw_service_info_t header{};
    take_one_request(header);

    hoard_.take_all();
    const auto      start = std::chrono::steady_clock::now();
    const rmw_ret_t ret = rmw_send_response(service_, &header.request_id, &response_);
    const int64_t   waited_ms = elapsed_ms(start);
    rmw_reset_error();

    EXPECT_EQ(ret, RMW_RET_ERROR);
    EXPECT_GE(waited_ms, static_cast<int64_t>(RMW_ROSETTE_MAX_BLOCKING_TIME_MS) - 1);
}

// TC-RMW-WRITE-003: a best-effort write never waits for the pool.
TEST_F(WriteWaitsForPool, BestEffortWriteFailsAtOnce) {
    rmw_qos_profile_t qos = rmw_qos_profile_services_default;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    create("/write_waits_best_effort", qos);

    hoard_.take_all();
    int64_t         sequence = 0;
    const auto      start = std::chrono::steady_clock::now();
    const rmw_ret_t ret = rmw_send_request(client_, &request_, &sequence);
    const int64_t   waited_ms = elapsed_ms(start);
    rmw_reset_error();

    EXPECT_EQ(ret, RMW_RET_ERROR);
    EXPECT_LT(waited_ms, static_cast<int64_t>(RMW_ROSETTE_MAX_BLOCKING_TIME_MS) / 2);
}
