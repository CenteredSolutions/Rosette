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

#include <rcutils/types/string_array.h>
#include <rmw/get_service_names_and_types.h>
#include <rmw/get_topic_names_and_types.h>
#include <rmw/names_and_types.h>
#include <rmw/rmw.h>

#include <std_msgs/msg/string.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "rcl_default_init_options.hpp"
#include "rmw_rosette/graph_cache.hpp"
#include "rmw_rosette/ros_discovery_info_reader.hpp"
#include "rmw_rosette/types.hpp"

namespace rtps = rosette::rtps;

namespace {

rtps::Guid make_guid(uint8_t participant, uint8_t entity) {
    rtps::Guid guid{};
    guid.prefix.value[11] = participant;
    guid.entityId.entityKey[2] = entity;
    return guid;
}

constexpr char kStringType[] = "std_msgs::msg::dds_::String_";

/// @brief Counts the listed nodes with this name and namespace; SIZE_MAX when the query or its
/// cleanup fails.
size_t count_nodes(const rmw_node_t* node, const char* name, const char* namespace_) {
    rcutils_string_array_t names = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t namespaces = rcutils_get_zero_initialized_string_array();
    if (rmw_get_node_names(node, &names, &namespaces) != RMW_RET_OK) {
        return SIZE_MAX;
    }

    size_t count = 0;
    for (size_t i = 0; i < names.size; ++i) {
        if (std::strcmp(names.data[i], name) == 0
            && std::strcmp(namespaces.data[i], namespace_) == 0) {
            ++count;
        }
    }

    const bool released = rcutils_string_array_fini(&names) == RCUTILS_RET_OK;
    if (rcutils_string_array_fini(&namespaces) != RCUTILS_RET_OK || !released) {
        return SIZE_MAX;
    }

    return count;
}

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

// A plain DDS application on a ROS topic name is listed with the type it
// announced, as rmw_fastrtps_cpp's _demangle_if_ros_type leaves it.
TEST(GraphCache, KeepsForeignTypesOnRosTopics) {
    rmw_rosette::GraphCache cache;
    cache.add(make_guid(1, 1), "rt/foo", "MyModule::MyType", true);
    cache.add(make_guid(2, 1), "rt/foo", kStringType, false);

    const auto topics = cache.topic_names_and_types(false);
    ASSERT_EQ(topics.count("/foo"), 1u);
    const std::set<std::string> expected{"MyModule::MyType", "std_msgs/msg/String"};
    EXPECT_EQ(topics.at("/foo"), expected);
}

// Counts are taken from the same records as the listings: by DDS topic and
// kind, whatever the type, and a refreshed participant's endpoints count
// until its replay completes without them.
TEST(GraphCache, CountsMatchListings) {
    rmw_rosette::GraphCache cache;
    cache.add(make_guid(1, 1), "rt/chatter", kStringType, true);
    cache.add(make_guid(2, 1), "rt/chatter", "MyModule::MyType", true);
    cache.add(make_guid(2, 2), "rt/chatter", kStringType, false);
    cache.add(make_guid(2, 3), "rr/addReply", "pkg::srv::dds_::Add_Response_", true);

    EXPECT_EQ(cache.count("rt/chatter", true), 2u);
    EXPECT_EQ(cache.count("rt/chatter", false), 1u);
    EXPECT_EQ(cache.count("rr/addReply", true), 1u);
    EXPECT_EQ(cache.count("rt/absent", true), 0u);

    const rtps::GuidPrefix refreshed = make_guid(2, 0).prefix;
    cache.begin_refresh(refreshed);
    EXPECT_EQ(cache.count("rt/chatter", true), 2u);
    cache.end_refresh(refreshed);
    EXPECT_EQ(cache.count("rt/chatter", true), 1u);
    EXPECT_EQ(cache.count("rt/chatter", false), 0u);
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

// A reactivated participant replays its live endpoints; the refresh forgets
// only those of that participant the replay did not confirm, and reports the
// change once.
TEST(GraphCache, RefreshForgetsOnlyUnconfirmed) {
    rmw_rosette::GraphCache cache;
    int                     changes = 0;
    cache.add(make_guid(1, 1), "rt/replayed", kStringType, true);
    cache.add(make_guid(1, 2), "rt/withdrawn", kStringType, true);
    cache.add(make_guid(2, 1), "rt/other_participant", kStringType, true);
    cache.set_on_change([](void* user) noexcept { ++*static_cast<int*>(user); }, &changes);

    const rtps::GuidPrefix refreshed = make_guid(1, 0).prefix;
    cache.begin_refresh(refreshed);
    EXPECT_EQ(cache.topic_names_and_types(false).size(), 3u);
    EXPECT_FALSE(cache.add(make_guid(1, 1), "rt/replayed", kStringType, true));
    EXPECT_TRUE(cache.add(make_guid(1, 3), "rt/new", kStringType, true));
    EXPECT_EQ(changes, 1);

    EXPECT_TRUE(cache.end_refresh(refreshed));
    EXPECT_EQ(changes, 2);
    const auto topics = cache.topic_names_and_types(false);
    EXPECT_EQ(topics.size(), 3u);
    EXPECT_EQ(topics.count("/replayed"), 1u);
    EXPECT_EQ(topics.count("/new"), 1u);
    EXPECT_EQ(topics.count("/other_participant"), 1u);

    EXPECT_FALSE(cache.end_refresh(refreshed));
    EXPECT_EQ(changes, 2);
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

/// @brief How many reader and writer GIDs one fixture node owns.
struct EndpointCounts {
    uint32_t readers = 0;
    uint32_t writers = 0;
};

/**
 * @brief A serialized ParticipantEntitiesInfo with `nodes` of (namespace, name).
 *
 * Node `i` owns `endpoints[i]` reader and writer GIDs, and none past the end of `endpoints`.
 * Each GID's octets are nonzero and differ from the next one's, so a decoder that skips the
 * wrong number of octets reads GID bytes as a string length and fails or misnames a node.
 */
static std::vector<uint8_t> encode_nodes(const std::vector<rmw_rosette::NodeName>& nodes,
                                         const std::vector<EndpointCounts>&        endpoints = {}) {
    std::vector<uint8_t> out = {0x00u, 0x01u, 0x00u, 0x00u};
    out.resize(out.size() + rmw_rosette::kGidSize);

    const auto put_u32 = [&out](uint32_t value) {
        while ((out.size() - 4u) % 4u != 0u) {
            out.push_back(0u);
        }

        for (int shift = 0; shift < 32; shift += 8) {
            out.push_back(static_cast<uint8_t>(value >> shift));
        }
    };
    const auto put_string = [&](const std::string& text) {
        put_u32(static_cast<uint32_t>(text.size() + 1u));
        out.insert(out.end(), text.begin(), text.end());
        out.push_back(0u);
    };

    uint8_t    gid_fill = 0x40u;
    const auto put_gids = [&](uint32_t count) {
        put_u32(count);
        for (uint32_t i = 0; i < count; ++i) {
            out.insert(out.end(), rmw_rosette::kGidSize, gid_fill++);
        }
    };

    put_u32(static_cast<uint32_t>(nodes.size()));
    for (size_t i = 0; i < nodes.size(); ++i) {
        const EndpointCounts counts = i < endpoints.size() ? endpoints[i] : EndpointCounts{};
        put_string(nodes[i].namespace_);
        put_string(nodes[i].name);
        put_gids(counts.readers);
        put_gids(counts.writers);
    }

    return out;
}

// A participant's latest node list replaces its earlier one and lasts until it
// leaves; the graph guard wakes only when node_names() could differ.
TEST(GraphCache, ReplacesAndForgetsNodes) {
    rmw_rosette::GraphCache cache;
    int                     changes = 0;
    cache.set_on_change([](void* user) noexcept { ++*static_cast<int*>(user); }, &changes);
    const rtps::GuidPrefix peer = make_guid(1, 0).prefix;
    const rtps::GuidPrefix other = make_guid(2, 0).prefix;

    EXPECT_TRUE(cache.set_participant_nodes(peer, {{"/", "first"}}));
    EXPECT_TRUE(cache.set_participant_nodes(other, {{"/robot", "kept"}}));
    EXPECT_FALSE(cache.set_participant_nodes(other, {{"/robot", "kept"}}));
    EXPECT_TRUE(cache.set_participant_nodes(peer, {{"/", "second"}}));
    EXPECT_FALSE(cache.set_participant_nodes(make_guid(3, 0).prefix, {}));
    ASSERT_EQ(cache.node_names().size(), 2u);
    EXPECT_EQ(changes, 3);

    // A participant with nodes and no endpoints still changes the graph by leaving.
    EXPECT_TRUE(cache.remove_participant(peer));
    EXPECT_EQ(changes, 4);
    const std::vector<rmw_rosette::NodeName> left = cache.node_names();
    ASSERT_EQ(left.size(), 1u);
    EXPECT_EQ(left[0].namespace_, "/robot");
    EXPECT_EQ(left[0].name, "kept");

    EXPECT_TRUE(cache.set_participant_nodes(other, {}));
    EXPECT_TRUE(cache.node_names().empty());
    EXPECT_EQ(changes, 5);
}

TEST(GraphCache, DecodesNodeNames) {
    const std::vector<uint8_t> bytes = encode_nodes({{"/", "talker"}, {"/robot", "listener"}});

    std::vector<rmw_rosette::NodeName> nodes;
    ASSERT_TRUE(rmw_rosette::decode_participant_node_names(bytes.data(), bytes.size(), nodes));
    ASSERT_EQ(nodes.size(), 2u);
    EXPECT_EQ(nodes[0].namespace_, "/");
    EXPECT_EQ(nodes[0].name, "talker");
    EXPECT_EQ(nodes[1].namespace_, "/robot");
    EXPECT_EQ(nodes[1].name, "listener");
}

TEST(GraphCache, RefusesTruncatedNodeMessages) {
    const std::vector<uint8_t> bytes = encode_nodes({{"/", "talker"}});

    for (size_t size = 0; size < bytes.size(); ++size) {
        std::vector<rmw_rosette::NodeName> nodes = {{"/", "unchanged"}};
        EXPECT_FALSE(rmw_rosette::decode_participant_node_names(bytes.data(), size, nodes)) << size;
        ASSERT_EQ(nodes.size(), 1u);
        EXPECT_EQ(nodes[0].name, "unchanged");
    }
}

TEST(GraphCache, RefusesBigEndianAndHugeCounts) {
    std::vector<uint8_t> big_endian = encode_nodes({{"/", "talker"}});
    big_endian[1] = 0x00u;

    std::vector<rmw_rosette::NodeName> nodes;
    EXPECT_FALSE(
        rmw_rosette::decode_participant_node_names(big_endian.data(), big_endian.size(), nodes));

    std::vector<uint8_t> huge = encode_nodes({});
    for (size_t i = huge.size() - 4u; i < huge.size(); ++i) {
        huge[i] = 0xFFu;
    }

    EXPECT_FALSE(rmw_rosette::decode_participant_node_names(huge.data(), huge.size(), nodes));
}

// Stock nodes own endpoints (a /rosout writer, parameter services), so a real sample
// carries GID sequences between node names; each skip must land on the next name.
TEST(GraphCache, DecodesNodeNamesPastEndpointGids) {
    const std::vector<uint8_t> bytes =
        encode_nodes({{"/", "talker"}, {"/robot", "listener"}, {"/robot", "params"}},
                     {{2u, 3u}, {0u, 1u}, {4u, 0u}});

    std::vector<rmw_rosette::NodeName> nodes;
    ASSERT_TRUE(rmw_rosette::decode_participant_node_names(bytes.data(), bytes.size(), nodes));
    ASSERT_EQ(nodes.size(), 3u);
    EXPECT_EQ(nodes[0].namespace_, "/");
    EXPECT_EQ(nodes[0].name, "talker");
    EXPECT_EQ(nodes[1].namespace_, "/robot");
    EXPECT_EQ(nodes[1].name, "listener");
    EXPECT_EQ(nodes[2].namespace_, "/robot");
    EXPECT_EQ(nodes[2].name, "params");
}

// A sample cut anywhere, including inside a GID, is refused and leaves the output alone.
TEST(GraphCache, RefusesNodeMessagesTruncatedInsideGids) {
    const std::vector<uint8_t> bytes =
        encode_nodes({{"/", "talker"}, {"/robot", "listener"}}, {{2u, 1u}, {1u, 2u}});

    for (size_t size = 0; size < bytes.size(); ++size) {
        std::vector<rmw_rosette::NodeName> nodes = {{"/", "unchanged"}};
        EXPECT_FALSE(rmw_rosette::decode_participant_node_names(bytes.data(), size, nodes)) << size;
        ASSERT_EQ(nodes.size(), 1u);
        EXPECT_EQ(nodes[0].name, "unchanged");
    }
}

// A GID count is refused one past what the sample holds and accepted at exactly what it holds.
TEST(GraphCache, BoundsGidCountsBySampleSize) {
    // The fixture ends with the last node's writer GID count, which is zero.
    const std::vector<uint8_t> base = encode_nodes({{"/", "talker"}}, {{2u, 0u}});
    const auto                 with_writer_count = [&base](uint32_t count) {
        std::vector<uint8_t> out = base;
        for (size_t i = 0; i < 4u; ++i) {
            out[out.size() - 4u + i] = static_cast<uint8_t>(count >> (8u * i));
        }

        return out;
    };

    // The last count's octet total wraps a 32-bit size_t to a few octets, as on a microcontroller.
    for (const uint32_t count : {1u, 0xFFFFFFFFu, 0xFFFFFFFFu / rmw_rosette::kGidSize + 1u}) {
        const std::vector<uint8_t>         bytes = with_writer_count(count);
        std::vector<rmw_rosette::NodeName> nodes = {{"/", "unchanged"}};
        EXPECT_FALSE(rmw_rosette::decode_participant_node_names(bytes.data(), bytes.size(), nodes))
            << count;
        ASSERT_EQ(nodes.size(), 1u);
        EXPECT_EQ(nodes[0].name, "unchanged");
    }

    std::vector<uint8_t> exact = with_writer_count(1u);
    exact.insert(exact.end(), rmw_rosette::kGidSize, 0x7Fu);
    std::vector<rmw_rosette::NodeName> nodes;
    ASSERT_TRUE(rmw_rosette::decode_participant_node_names(exact.data(), exact.size(), nodes));
    ASSERT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes[0].name, "talker");
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

// Whether `observer`'s context lists a node named `name` in `ns`.
static bool lists_node(const rmw_node_t* observer, const char* name, const char* ns) {
    rcutils_string_array_t names = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t namespaces = rcutils_get_zero_initialized_string_array();
    bool                   found = false;
    if (rmw_get_node_names(observer, &names, &namespaces) == RMW_RET_OK) {
        for (size_t i = 0; i < names.size && i < namespaces.size; ++i) {
            found = found
                    || (std::strcmp(names.data[i], name) == 0
                        && std::strcmp(namespaces.data[i], ns) == 0);
        }
    }

    EXPECT_EQ(rcutils_string_array_fini(&names), RCUTILS_RET_OK);
    EXPECT_EQ(rcutils_string_array_fini(&namespaces), RCUTILS_RET_OK);
    return found;
}

TEST(RmwNodeNames, ListsALocalNodeUntilDestroyed) {
    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&init_options), RMW_RET_OK);

    rmw_context_t context = rmw_get_zero_initialized_context();
    ASSERT_EQ(rmw_init(&init_options, &context), RMW_RET_OK);
    ASSERT_EQ(rmw_init_options_fini(&init_options), RMW_RET_OK);

    rmw_node_t* listed = rmw_create_node(&context, "graph_listed_node", "/robot");
    ASSERT_NE(listed, nullptr);
    rmw_node_t* observer = rmw_create_node(&context, "graph_observer_node", "/");
    ASSERT_NE(observer, nullptr);
    EXPECT_TRUE(lists_node(observer, "graph_listed_node", "/robot"));
    EXPECT_TRUE(lists_node(observer, "graph_observer_node", "/"));

    // The output arrays must start zero-initialized, as for every rmw.
    rcutils_string_array_t names = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t namespaces = rcutils_get_zero_initialized_string_array();
    ASSERT_EQ(rmw_get_node_names(observer, &names, &namespaces), RMW_RET_OK);
    EXPECT_EQ(rmw_get_node_names(observer, &names, &namespaces), RMW_RET_INVALID_ARGUMENT);
    rcutils_reset_error();
    ASSERT_EQ(rcutils_string_array_fini(&names), RCUTILS_RET_OK);
    ASSERT_EQ(rcutils_string_array_fini(&namespaces), RCUTILS_RET_OK);

    ASSERT_EQ(rmw_destroy_node(listed), RMW_RET_OK);
    EXPECT_FALSE(lists_node(observer, "graph_listed_node", "/robot"));
    EXPECT_TRUE(lists_node(observer, "graph_observer_node", "/"));

    ASSERT_EQ(rmw_destroy_node(observer), RMW_RET_OK);
    ASSERT_EQ(rmw_shutdown(&context), RMW_RET_OK);
    ASSERT_EQ(rmw_context_fini(&context), RMW_RET_OK);
}

// The context's ros_discovery_info sample would loop back to its own reader if
// the two paired; waiting on the graph guard drives the participant long
// enough for that sample to arrive, and wakes as soon as it changes the cache.
TEST(RmwNodeNames, ListsALocalNodeOnceAfterItsSampleIsSent) {
    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&init_options), RMW_RET_OK);

    rmw_context_t context = rmw_get_zero_initialized_context();
    ASSERT_EQ(rmw_init(&init_options, &context), RMW_RET_OK);
    ASSERT_EQ(rmw_init_options_fini(&init_options), RMW_RET_OK);

    rmw_node_t* node = rmw_create_node(&context, "listed_once_node", "/robot");
    ASSERT_NE(node, nullptr);
    rmw_wait_set_t* wait_set = rmw_create_wait_set(&context, 1);
    ASSERT_NE(wait_set, nullptr);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
        void*                  guard = rmw_node_get_graph_guard_condition(node)->data;
        rmw_guard_conditions_t guards{1, &guard};
        rmw_time_t             timeout{0, 100000000};
        const rmw_ret_t        waited =
            rmw_wait(nullptr, &guards, nullptr, nullptr, nullptr, wait_set, &timeout);
        ASSERT_TRUE(waited == RMW_RET_OK || waited == RMW_RET_TIMEOUT);
    }

    EXPECT_EQ(count_nodes(node, "listed_once_node", "/robot"), 1u);

    ASSERT_EQ(rmw_destroy_wait_set(wait_set), RMW_RET_OK);
    ASSERT_EQ(rmw_destroy_node(node), RMW_RET_OK);
    ASSERT_EQ(rmw_shutdown(&context), RMW_RET_OK);
    ASSERT_EQ(rmw_context_fini(&context), RMW_RET_OK);
}
