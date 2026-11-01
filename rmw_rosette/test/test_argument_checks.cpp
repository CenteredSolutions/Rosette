// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_argument_checks.cpp
 * @brief Unit tests for the argument checks every rmw_* entry point makes.
 *
 * The checks in argument_checks.hpp decide the return code of every refused
 * call: INVALID_ARGUMENT for a null, uninitialized or invalid argument, and
 * INCORRECT_RMW_IMPLEMENTATION for another rmw's handle. These tests pin the
 * order the faults are checked in and the error text, without a context or a
 * participant; test_rmw_implementation exercises the same checks through the
 * entry points.
 */

#include "rmw_rosette/argument_checks.hpp"
#include "rmw_rosette/identifier.h"
#include "rmw_rosette/ros_features.h"
#include "rmw_rosette/types.hpp"

#include <gtest/gtest.h>

#include <rcutils/error_handling.h>
#include <rmw/qos_profiles.h>
#include <rmw/rmw.h>

#include <string>

namespace {

using rmw_rosette::ArgumentFault;

/// @brief The current rcutils error message, then cleared for the next check.
std::string take_error() {
    std::string message = rcutils_get_error_string().str;
    rcutils_reset_error();
    return message;
}

/// @brief The error message without the " at <file>:<line>" rcutils appends.
std::string message_of(const std::string& error) {
    return error.substr(0, error.find(", at "));
}

int g_object = 0;

// TC-RMW-ARG-001: the message reads "rmw_rosette: <function>: <argument> <fault>",
// and only a foreign handle maps to INCORRECT_RMW_IMPLEMENTATION.
TEST(ArgumentChecks, RefuseArgumentFormatsTheMessageAndPicksTheCode) {
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT,
              rmw_rosette::refuse_argument("rmw_publish", "ros_message", ArgumentFault::null));
    EXPECT_EQ("rmw_rosette: rmw_publish: ros_message is null", message_of(take_error()));

    EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION,
              rmw_rosette::refuse_argument("rmw_publish", "publisher", ArgumentFault::foreign));
    EXPECT_EQ("rmw_rosette: rmw_publish: publisher belongs to another rmw implementation",
              message_of(take_error()));

    for (ArgumentFault fault : {ArgumentFault::uninitialized, ArgumentFault::already_in_use,
                                ArgumentFault::invalid, ArgumentFault::shut_down}) {
        EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_rosette::refuse_argument("f", "a", fault));
        rcutils_reset_error();
    }
}

// TC-RMW-ARG-002: null, then a zero-initialized handle, then foreign before
// uninitialized, then the required pointers in order; a usable handle passes
// and sets no error.
TEST(ArgumentChecks, CheckHandleRefusesInOrder) {
    rmw_publisher_t publisher{};
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT,
              rmw_rosette::check_handle<rmw_publisher_t>(nullptr, "f", "publisher"));
    EXPECT_EQ("rmw_rosette: f: publisher is null", message_of(take_error()));

    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_rosette::check_handle(&publisher, "f", "publisher"));
    EXPECT_EQ("rmw_rosette: f: publisher is not initialized", message_of(take_error()));

    publisher.implementation_identifier = "not-an-rmw-implementation-identifier";
    EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION,
              rmw_rosette::check_handle(&publisher, "f", "publisher"));
    rcutils_reset_error();

    const std::string copied_identifier = rmw_rosette_identifier;
    publisher.implementation_identifier = copied_identifier.c_str();
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_rosette::check_handle(&publisher, "f", "publisher"));
    EXPECT_EQ("rmw_rosette: f: publisher is not initialized", message_of(take_error()));

    publisher.data = &g_object;
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT,
              rmw_rosette::check_handle(&publisher, "f", "publisher",
                                        {{&g_object, "gid"}, {nullptr, "qos"}}));
    EXPECT_EQ("rmw_rosette: f: qos is null", message_of(take_error()));

    EXPECT_EQ(RMW_RET_OK,
              rmw_rosette::check_handle(&publisher, "f", "publisher", {{&g_object, "gid"}}));
    EXPECT_FALSE(rcutils_error_is_set());
}

// TC-RMW-ARG-003: a context is refused when null, zero-initialized, foreign or
// without impl, in that order, and one that was shut down only when the caller
// requires a live context.
TEST(ArgumentChecks, CheckContextRefusesInOrder) {
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_rosette::check_context(nullptr, "f", false));
    rcutils_reset_error();

    rmw_context_t context = rmw_get_zero_initialized_context();
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_rosette::check_context(&context, "f", false));
    EXPECT_EQ("rmw_rosette: f: context is not initialized", message_of(take_error()));

    context.implementation_identifier = "not-an-rmw-implementation-identifier";
    EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION,
              rmw_rosette::check_context(&context, "f", false));
    rcutils_reset_error();

    context.implementation_identifier = rmw_rosette_identifier;
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_rosette::check_context(&context, "f", false));
    EXPECT_EQ("rmw_rosette: f: context is not initialized", message_of(take_error()));

    rmw_context_impl_t impl{};
    context.impl = &impl;
    EXPECT_EQ(RMW_RET_OK, rmw_rosette::check_context(&context, "f", true));

    impl.rosette_context.shut_down.store(true);
    EXPECT_EQ(RMW_RET_OK, rmw_rosette::check_context(&context, "f", false));
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_rosette::check_context(&context, "f", true));
    EXPECT_EQ("rmw_rosette: f: context has been shut down", message_of(take_error()));
}

// TC-RMW-ARG-004: the endpoint factories' shared checks, in the stock order,
// with ROS name rules waived by avoid_ros_namespace_conventions, QoS policies
// held to the distribution's enumerations, and no endpoint for a node whose
// context was shut down.
TEST(ArgumentChecks, CheckEndpointArgumentsRefusesInOrder) {
    rmw_context_impl_t impl{};
    rmw_context_t      context = rmw_get_zero_initialized_context();
    context.implementation_identifier = rmw_rosette_identifier;
    context.impl = &impl;
    rmw_node_t node{};
    node.implementation_identifier = rmw_rosette_identifier;
    node.data = &g_object;
    node.context = &context;
    const void*       ts = &g_object;
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    const auto        check = [&](const char* name, const rmw_qos_profile_t* profile) {
        const rmw_ret_t ret = rmw_rosette::check_endpoint_arguments(&node, ts, name, profile, "f");
        rcutils_reset_error();
        return ret;
    };

    EXPECT_EQ(RMW_RET_OK, check("/chatter", &qos));
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check(nullptr, &qos));
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("", &qos));
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("chatter", &qos));
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("/foo bar", &qos));
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("/chatter", nullptr));
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("/chatter", &rmw_qos_profile_unknown));
    rmw_qos_profile_t out_of_range = qos;
    out_of_range.history = static_cast<rmw_qos_history_policy_t>(42);
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("/chatter", &out_of_range));
    out_of_range = qos;
    out_of_range.reliability = static_cast<rmw_qos_reliability_policy_t>(42);
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("/chatter", &out_of_range));
    out_of_range = qos;
    out_of_range.durability = static_cast<rmw_qos_durability_policy_t>(42);
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("/chatter", &out_of_range));
    out_of_range = qos;
    out_of_range.liveliness = static_cast<rmw_qos_liveliness_policy_t>(42);
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("/chatter", &out_of_range));
#if RMW_ROSETTE_HAS(QOS_BEST_AVAILABLE)
    EXPECT_EQ(RMW_RET_OK, check("/chatter", &rmw_qos_profile_best_available));
#endif

    qos.avoid_ros_namespace_conventions = true;
    EXPECT_EQ(RMW_RET_OK, check("chatter", &qos));
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("", &qos));

    qos.avoid_ros_namespace_conventions = false;
    impl.rosette_context.shut_down.store(true);
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("/chatter", &qos));

    impl.rosette_context.shut_down.store(false);
    ts = nullptr;
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, check("/chatter", &qos));

    node.implementation_identifier = "not-an-rmw-implementation-identifier";
    EXPECT_EQ(RMW_RET_INCORRECT_RMW_IMPLEMENTATION, check("/chatter", &qos));
}

}  // namespace
