// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_graph_cache.cpp
 * @brief Unit tests for GraphCache and the rmw name queries it answers.
 *
 * The expected names follow what rmw_fastrtps_cpp reports for the same DDS
 * endpoints. The live case drives a real participant, so the observer's
 * registration and the path from create_publisher to the listing are checked.
 */

#include <gtest/gtest.h>

#include <rmw/get_service_names_and_types.h>
#include <rmw/get_topic_names_and_types.h>
#include <rmw/names_and_types.h>
#include <rmw/rmw.h>

#include <std_msgs/msg/string.h>

#include <cstring>
#include <string>

#include "rcl_default_init_options.hpp"
#include "rmw_rosette/graph_cache.hpp"

namespace rtps = rosette::rtps;

namespace {

rtps::Guid make_guid(uint8_t participant, uint8_t entity) {
    rtps::Guid guid{};
    guid.prefix.value[11] = participant;
    guid.entityId.entityKey[2] = entity;
    return guid;
}

constexpr char kStringType[] = "std_msgs::msg::dds_::String_";

}  // namespace

TEST(GraphCache, DemanglesRosTopics) {
    rmw_rosette::GraphCache cache;
    cache.add(make_guid(1, 1), "rt/chatter", kStringType, true);
    cache.add(make_guid(2, 1), "rt/chatter", kStringType, false);
    cache.add(make_guid(1, 2), "ros_discovery_info", "rmw_dds_common::msg::dds_::Info_", true);
    cache.add(make_guid(1, 3), "rq/addRequest", "pkg::srv::dds_::Add_Request_", false);

    const auto topics = cache.topic_names_and_types(false);
    ASSERT_EQ(topics.size(), 1u);
    ASSERT_EQ(topics.count("/chatter"), 1u);
    EXPECT_EQ(*topics.at("/chatter").begin(), "std_msgs/msg/String");

    const auto raw = cache.topic_names_and_types(true);
    EXPECT_EQ(raw.size(), 3u);
    EXPECT_EQ(*raw.at("rt/chatter").begin(), kStringType);
}

TEST(GraphCache, DemanglesServices) {
    rmw_rosette::GraphCache cache;
    cache.add(make_guid(1, 1), "rq/ns/addRequest", "pkg::srv::dds_::Add_Request_", false);
    cache.add(make_guid(1, 2), "rr/ns/addReply", "pkg::srv::dds_::Add_Response_", true);
    cache.add(make_guid(1, 3), "rt/not_a_service", kStringType, true);

    const auto services = cache.service_names_and_types();
    ASSERT_EQ(services.size(), 1u);
    ASSERT_EQ(services.at("/ns/add").size(), 1u);
    EXPECT_EQ(*services.at("/ns/add").begin(), "pkg/srv/Add");
}

TEST(GraphCache, RepeatedAnnouncementIsIdempotentAndRemovalForgets) {
    rmw_rosette::GraphCache cache;
    const rtps::Guid        guid = make_guid(1, 1);
    cache.add(guid, "rt/a", kStringType, true);
    cache.add(guid, "rt/a", kStringType, true);
    EXPECT_EQ(cache.topic_names_and_types(false).size(), 1u);

    cache.remove(guid);
    EXPECT_TRUE(cache.topic_names_and_types(false).empty());
}

TEST(GraphCache, RemovesOnlyTheDepartedParticipant) {
    rmw_rosette::GraphCache cache;
    cache.add(make_guid(1, 1), "rt/a", kStringType, true);
    cache.add(make_guid(1, 2), "rt/b", kStringType, true);
    cache.add(make_guid(2, 1), "rt/c", kStringType, true);

    cache.remove_participant(make_guid(1, 0).prefix);

    const auto topics = cache.topic_names_and_types(false);
    ASSERT_EQ(topics.size(), 1u);
    EXPECT_EQ(topics.count("/c"), 1u);
}

// The graph guard wakes exactly when a listing could differ: a new endpoint, a
// re-announcement with other names, a withdrawal, a departure with endpoints.
TEST(GraphCache, ReportsEveryChangeAndNothingElse) {
    rmw_rosette::GraphCache cache;
    int                     changes = 0;
    cache.set_on_change([](void* user) noexcept { ++*static_cast<int*>(user); }, &changes);
    const rtps::Guid guid = make_guid(1, 1);

    EXPECT_TRUE(cache.add(guid, "rt/a", kStringType, true));
    EXPECT_FALSE(cache.add(guid, "rt/a", kStringType, true));
    EXPECT_EQ(changes, 1);

    EXPECT_TRUE(cache.add(guid, "rt/a", "std_msgs::msg::dds_::Int32_", true));
    EXPECT_TRUE(cache.add(guid, "rt/b", "std_msgs::msg::dds_::Int32_", true));
    EXPECT_TRUE(cache.add(guid, "rt/b", "std_msgs::msg::dds_::Int32_", false));
    EXPECT_EQ(changes, 4);

    EXPECT_TRUE(cache.remove(guid));
    EXPECT_FALSE(cache.remove(guid));
    EXPECT_EQ(changes, 5);

    cache.add(make_guid(2, 1), "rt/c", kStringType, true);
    cache.add(make_guid(2, 2), "rt/d", kStringType, true);
    EXPECT_EQ(changes, 7);
    EXPECT_TRUE(cache.remove_participant(make_guid(2, 0).prefix));
    EXPECT_FALSE(cache.remove_participant(make_guid(2, 0).prefix));
    EXPECT_EQ(changes, 8);
}

// The match table hands the observer every endpoint, including one it has no
// room for and so never reports as its own change; the cache's callback is
// what wakes the graph guard for it.
TEST(GraphCache, ObserverForwardsToTheCache) {
    rmw_rosette::GraphCache            cache;
    int                                changes = 0;
    const rtps::GraphEndpointObserver& observer = cache.observer();
    const rtps::Guid                   guid = make_guid(3, 1);
    cache.set_on_change([](void* user) noexcept { ++*static_cast<int*>(user); }, &changes);

    observer.on_endpoint(observer.user, guid, "rt/seen", kStringType, true);
    EXPECT_EQ(cache.topic_names_and_types(false).count("/seen"), 1u);
    EXPECT_EQ(changes, 1);

    observer.on_endpoint_gone(observer.user, guid);
    EXPECT_TRUE(cache.topic_names_and_types(false).empty());
    EXPECT_EQ(changes, 2);
}

TEST(RmwGraphNames, ListsALocalPublisherAndForgetsItWhenDestroyed) {
    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&init_options), RMW_RET_OK);

    rmw_context_t context = rmw_get_zero_initialized_context();
    ASSERT_EQ(rmw_init(&init_options, &context), RMW_RET_OK);
    ASSERT_EQ(rmw_init_options_fini(&init_options), RMW_RET_OK);

    rmw_node_t* node = rmw_create_node(&context, "graph_names_node", "/");
    ASSERT_NE(node, nullptr);

    rmw_publisher_options_t options = rmw_get_default_publisher_options();
    rmw_publisher_t*        pub =
        rmw_create_publisher(node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                             "/graph_names_topic", &rmw_qos_profile_default, &options);
    ASSERT_NE(pub, nullptr);

    rcutils_allocator_t   allocator = rcutils_get_default_allocator();
    rmw_names_and_types_t listing = rmw_get_zero_initialized_names_and_types();
    ASSERT_EQ(rmw_get_topic_names_and_types(node, &allocator, false, &listing), RMW_RET_OK);

    bool found = false;
    for (size_t i = 0; i < listing.names.size; ++i) {
        if (std::strcmp(listing.names.data[i], "/graph_names_topic") == 0) {
            found = true;
            ASSERT_EQ(listing.types[i].size, 1u);
            EXPECT_STREQ(listing.types[i].data[0], "std_msgs/msg/String");
        }
    }

    EXPECT_TRUE(found);
    ASSERT_EQ(rmw_names_and_types_fini(&listing), RMW_RET_OK);

    ASSERT_EQ(rmw_destroy_publisher(node, pub), RMW_RET_OK);
    listing = rmw_get_zero_initialized_names_and_types();
    ASSERT_EQ(rmw_get_topic_names_and_types(node, &allocator, false, &listing), RMW_RET_OK);
    for (size_t i = 0; i < listing.names.size; ++i) {
        EXPECT_STRNE(listing.names.data[i], "/graph_names_topic");
    }

    ASSERT_EQ(rmw_names_and_types_fini(&listing), RMW_RET_OK);

    ASSERT_EQ(rmw_destroy_node(node), RMW_RET_OK);
    ASSERT_EQ(rmw_shutdown(&context), RMW_RET_OK);
    ASSERT_EQ(rmw_context_fini(&context), RMW_RET_OK);
}
