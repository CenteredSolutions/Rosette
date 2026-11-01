// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_client_gid.cpp
 * @brief rmw_get_gid_for_client() names the client's request writer.
 *
 * rclcpp and service introspection identify a client by its GID, and a
 * service sees the same identity as `rmw_request_id_t::writer_guid` on every
 * request it takes. A SetBool service and two clients share one rmw_rosette
 * context over real UDP loopback.
 *
 * Verification:
 * - Correct means: the GID carries this implementation's identifier, its
 *   first 16 bytes equal the writer_guid of a request that client sent, the
 *   rest is zero, two clients have different GIDs, and null or foreign
 *   arguments are refused.
 *
 * References:
 * - rmw/rmw.h (Jazzy), `rmw_get_gid_for_client`
 * - rmw/types.h (Jazzy), `rmw_gid_t`, `rmw_request_id_t`
 */

#include <gtest/gtest.h>

#include "rcl_default_init_options.hpp"

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <std_srvs/srv/set_bool.h>

#include <cstring>

namespace {

/// One context with a SetBool service and two clients of it.
class ClientGid : public ::testing::Test {
protected:
    void SetUp() override {
        rmw_init_options_t options = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&options), RMW_RET_OK);
        ASSERT_EQ(rmw_init(&options, &context_), RMW_RET_OK);
        ASSERT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
        node_ = rmw_create_node(&context_, "client_gid_test", "/");
        ASSERT_NE(node_, nullptr);

        const rosidl_service_type_support_t* ts =
            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);
        const rmw_qos_profile_t qos = rmw_qos_profile_services_default;
        service_ = rmw_create_service(node_, ts, "/client_gid", &qos);
        ASSERT_NE(service_, nullptr);
        client_ = rmw_create_client(node_, ts, "/client_gid", &qos);
        ASSERT_NE(client_, nullptr);
        other_client_ = rmw_create_client(node_, ts, "/client_gid", &qos);
        ASSERT_NE(other_client_, nullptr);
        wait_set_ = rmw_create_wait_set(&context_, 4);
        ASSERT_NE(wait_set_, nullptr);
    }

    void TearDown() override {
        EXPECT_EQ(rmw_destroy_wait_set(wait_set_), RMW_RET_OK);
        EXPECT_EQ(rmw_destroy_client(node_, other_client_), RMW_RET_OK);
        EXPECT_EQ(rmw_destroy_client(node_, client_), RMW_RET_OK);
        EXPECT_EQ(rmw_destroy_service(node_, service_), RMW_RET_OK);
        EXPECT_EQ(rmw_destroy_node(node_), RMW_RET_OK);
        EXPECT_EQ(rmw_shutdown(&context_), RMW_RET_OK);
        EXPECT_EQ(rmw_context_fini(&context_), RMW_RET_OK);
        rmw_reset_error();
    }

    /// Runs discovery and delivery until `client_` sees the service, for at most 5 s.
    void wait_for_service() {
        bool available = false;
        for (int i = 0; i < 50 && !available; ++i) {
            rmw_time_t      t{0, 100000000};
            const rmw_ret_t ret =
                rmw_wait(nullptr, nullptr, nullptr, nullptr, nullptr, wait_set_, &t);
            ASSERT_TRUE(ret == RMW_RET_OK || ret == RMW_RET_TIMEOUT);
            ASSERT_EQ(rmw_service_server_is_available(node_, client_, &available), RMW_RET_OK);
        }

        ASSERT_TRUE(available) << "client and service never matched";
    }

    rmw_context_t   context_ = rmw_get_zero_initialized_context();
    rmw_node_t*     node_ = nullptr;
    rmw_service_t*  service_ = nullptr;
    rmw_client_t*   client_ = nullptr;
    rmw_client_t*   other_client_ = nullptr;
    rmw_wait_set_t* wait_set_ = nullptr;
};

// TC-RMW-CLIENTGID-001: the GID is stamped with this rmw and differs between clients.
TEST_F(ClientGid, EachClientHasItsOwnGid) {
    rmw_gid_t gid{};
    rmw_gid_t other{};
    ASSERT_EQ(rmw_get_gid_for_client(client_, &gid), RMW_RET_OK);
    ASSERT_EQ(rmw_get_gid_for_client(other_client_, &other), RMW_RET_OK);

    EXPECT_STREQ(gid.implementation_identifier, rmw_get_implementation_identifier());
    EXPECT_NE(std::memcmp(gid.data, other.data, sizeof(gid.data)), 0);

    bool compared = false;
    EXPECT_EQ(rmw_compare_gids_equal(&gid, &gid, &compared), RMW_RET_OK);
    EXPECT_TRUE(compared);
}

// TC-RMW-CLIENTGID-002: a service sees the client's GID as the request's writer_guid.
TEST_F(ClientGid, GidMatchesTheRequestWriterGuid) {
    wait_for_service();

    std_srvs__srv__SetBool_Request req{};
    std_srvs__srv__SetBool_Request__init(&req);
    int64_t sequence = 0;
    ASSERT_EQ(rmw_send_request(client_, &req, &sequence), RMW_RET_OK);

    void*          services[] = {service_->data};
    rmw_services_t ready{1, services};
    rmw_time_t     timeout{5, 0};
    ASSERT_EQ(rmw_wait(nullptr, nullptr, &ready, nullptr, nullptr, wait_set_, &timeout),
              RMW_RET_OK);
    ASSERT_NE(ready.services[0], nullptr) << "the request never arrived";

    rmw_service_info_t header{};
    bool               taken = false;
    ASSERT_EQ(rmw_take_request(service_, &header, &req, &taken), RMW_RET_OK);
    ASSERT_TRUE(taken);
    std_srvs__srv__SetBool_Request__fini(&req);

    rmw_gid_t gid{};
    ASSERT_EQ(rmw_get_gid_for_client(client_, &gid), RMW_RET_OK);
    EXPECT_EQ(std::memcmp(gid.data, header.request_id.writer_guid, 16), 0);
    for (size_t i = 16; i < sizeof(gid.data); ++i) {
        EXPECT_EQ(gid.data[i], 0u) << "byte " << i;
    }
}

// TC-RMW-CLIENTGID-003: null arguments and clients of another rmw are refused.
TEST_F(ClientGid, RefusesInvalidArguments) {
    rmw_gid_t gid{};
    EXPECT_EQ(rmw_get_gid_for_client(nullptr, &gid), RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();
    EXPECT_EQ(rmw_get_gid_for_client(client_, nullptr), RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();

    const char* const own_identifier = client_->implementation_identifier;
    client_->implementation_identifier = "rmw_other_implementation";
    EXPECT_EQ(rmw_get_gid_for_client(client_, &gid), RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    client_->implementation_identifier = own_identifier;
}

}  // namespace
