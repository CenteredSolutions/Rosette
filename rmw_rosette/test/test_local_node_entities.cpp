// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_local_node_entities.cpp
 * @brief Unit tests for LocalNodeEntities (local_node_entities.hpp).
 *
 * The encoder's output is decoded through rmw_dds_common's own introspection
 * type support, the definition every stock DDS rmw reads `ros_discovery_info`
 * with, so these tests check wire compatibility rather than restating the
 * encoder's own assumptions.
 */

#include <gtest/gtest.h>

#include <rmw_dds_common/msg/participant_entities_info.h>

#include <cstring>
#include <vector>

#include "rmw_rosette/local_node_entities.hpp"
#include "rmw_rosette/message_codec.hpp"
#include "rmw_rosette/ros_discovery_info_writer.hpp"
#include "rmw_rosette/types.hpp"

namespace rtps = rosette::rtps;

namespace {

const rosidl_message_type_support_t* entities_ts() {
    return ROSIDL_GET_MSG_TYPE_SUPPORT(rmw_dds_common, msg, ParticipantEntitiesInfo);
}

rtps::Guid make_guid(uint8_t prefix_seed, uint8_t key, uint8_t kind) {
    rtps::Guid g{};
    for (uint8_t i = 0; i < 12; ++i) g.prefix.value[i] = static_cast<uint8_t>(prefix_seed + i);
    g.entityId.entityKey[0] = 0;
    g.entityId.entityKey[1] = 0;
    g.entityId.entityKey[2] = key;
    g.entityId.entityKind = kind;
    return g;
}

void set_node(rmw_rosette::Node& node, const char* ns, const char* name) {
    std::strncpy(node.namespace_, ns, sizeof(node.namespace_) - 1);
    std::strncpy(node.name, name, sizeof(node.name) - 1);
}

/// Checks a decoded Gid against the GUID it was encoded from: 16 GUID octets, then zeros.
void expect_gid(const rmw_dds_common__msg__Gid& gid, const rtps::Guid& guid) {
    uint8_t want[rmw_rosette::kGidSize] = {};
    std::memcpy(want, guid.prefix.value, 12);
    std::memcpy(want + 12, guid.entityId.entityKey, 3);
    want[15] = guid.entityId.entityKind;
    EXPECT_EQ(0, std::memcmp(gid.data, want, sizeof(want)));
}

/// Encodes `table`, prepends CDR_LE, and decodes it as ParticipantEntitiesInfo.
bool round_trip(const rmw_rosette::LocalNodeEntities& table, const rtps::Guid& participant,
                rmw_dds_common__msg__ParticipantEntitiesInfo& msg) {
    std::vector<uint8_t> buf(2048);
    buf[1] = 0x01;
    const uint32_t n =
        table.encode(participant, buf.data() + 4, static_cast<uint32_t>(buf.size() - 4));
    if (n == 0) return false;
    rmw_rosette::MessageCodec codec;
    if (!rmw_rosette::resolve_message_codec(entities_ts(), codec)) return false;
    return rmw_rosette::deserialize_message(codec, buf.data(), n + 4, &msg);
}

}  // namespace

TEST(LocalNodeEntities, TypeNameMatchesRmwDdsCommon) {
    rmw_rosette::MessageCodec codec;
    ASSERT_TRUE(rmw_rosette::resolve_message_codec(entities_ts(), codec));
    char name[128];
    ASSERT_TRUE(rmw_rosette::compose_codec_type_name(codec, name, sizeof(name)));
    EXPECT_STREQ(name, rmw_rosette::kParticipantEntitiesInfoType);
}

TEST(LocalNodeEntities, EncodesNodesAndTheirEndpoints) {
    rmw_rosette::Node talker{};
    rmw_rosette::Node listener{};
    set_node(talker, "/demo", "talker");
    set_node(listener, "/", "listener");

    const rtps::Guid participant = make_guid(0x10, 0x01, 0xC1);
    const rtps::Guid chatter_writer = make_guid(0x10, 0x11, 0x03);
    const rtps::Guid rosout_writer = make_guid(0x10, 0x12, 0x03);
    const rtps::Guid chatter_reader = make_guid(0x10, 0x13, 0x04);

    rmw_rosette::LocalNodeEntities table;
    ASSERT_TRUE(table.add_node(talker));
    ASSERT_TRUE(table.add_node(listener));
    ASSERT_TRUE(table.associate(talker, chatter_writer, false));
    ASSERT_TRUE(table.associate(listener, chatter_reader, true));
    ASSERT_TRUE(table.associate(talker, rosout_writer, false));

    rmw_dds_common__msg__ParticipantEntitiesInfo msg;
    ASSERT_TRUE(rmw_dds_common__msg__ParticipantEntitiesInfo__init(&msg));
    ASSERT_TRUE(round_trip(table, participant, msg));

    expect_gid(msg.gid, participant);
    ASSERT_EQ(msg.node_entities_info_seq.size, 2u);
    const auto& t = msg.node_entities_info_seq.data[0];
    const auto& l = msg.node_entities_info_seq.data[1];
    EXPECT_STREQ(t.node_namespace.data, "/demo");
    EXPECT_STREQ(t.node_name.data, "talker");
    EXPECT_EQ(t.reader_gid_seq.size, 0u);
    ASSERT_EQ(t.writer_gid_seq.size, 2u);
    expect_gid(t.writer_gid_seq.data[0], chatter_writer);
    expect_gid(t.writer_gid_seq.data[1], rosout_writer);
    EXPECT_STREQ(l.node_namespace.data, "/");
    EXPECT_STREQ(l.node_name.data, "listener");
    ASSERT_EQ(l.reader_gid_seq.size, 1u);
    expect_gid(l.reader_gid_seq.data[0], chatter_reader);
    EXPECT_EQ(l.writer_gid_seq.size, 0u);
    rmw_dds_common__msg__ParticipantEntitiesInfo__fini(&msg);
}

TEST(LocalNodeEntities, RemovalDropsNodeAndItsEndpoints) {
    rmw_rosette::Node a{};
    rmw_rosette::Node b{};
    set_node(a, "/", "a");
    set_node(b, "/", "b");
    const rtps::Guid participant = make_guid(0x20, 0x01, 0xC1);
    const rtps::Guid a_writer = make_guid(0x20, 0x11, 0x03);
    const rtps::Guid b_reader = make_guid(0x20, 0x12, 0x04);
    const rtps::Guid b_writer = make_guid(0x20, 0x13, 0x03);

    rmw_rosette::LocalNodeEntities table;
    ASSERT_TRUE(table.add_node(a));
    ASSERT_TRUE(table.add_node(b));
    ASSERT_TRUE(table.associate(a, a_writer, false));
    ASSERT_TRUE(table.associate(b, b_reader, true));
    ASSERT_TRUE(table.associate(b, b_writer, false));
    table.dissociate(b_writer);
    table.remove_node(a);

    rmw_dds_common__msg__ParticipantEntitiesInfo msg;
    ASSERT_TRUE(rmw_dds_common__msg__ParticipantEntitiesInfo__init(&msg));
    ASSERT_TRUE(round_trip(table, participant, msg));
    ASSERT_EQ(msg.node_entities_info_seq.size, 1u);
    const auto& only = msg.node_entities_info_seq.data[0];
    EXPECT_STREQ(only.node_name.data, "b");
    ASSERT_EQ(only.reader_gid_seq.size, 1u);
    expect_gid(only.reader_gid_seq.data[0], b_reader);
    EXPECT_EQ(only.writer_gid_seq.size, 0u);
    rmw_dds_common__msg__ParticipantEntitiesInfo__fini(&msg);
}

TEST(LocalNodeEntities, RejectsEndpointOfUnknownNodeAndTooManyNodes) {
    rmw_rosette::Node nodes[RMW_ROSETTE_MAX_NODES + 1] = {};
    rmw_rosette::LocalNodeEntities table;
    EXPECT_FALSE(table.associate(nodes[0], make_guid(0x30, 0x11, 0x03), false));
    for (uint32_t i = 0; i < RMW_ROSETTE_MAX_NODES; ++i) EXPECT_TRUE(table.add_node(nodes[i]));
    EXPECT_FALSE(table.add_node(nodes[RMW_ROSETTE_MAX_NODES]));
}

TEST(LocalNodeEntities, ReportsABufferTooSmall) {
    rmw_rosette::Node node{};
    set_node(node, "/", "node");
    rmw_rosette::LocalNodeEntities table;
    ASSERT_TRUE(table.add_node(node));
    const rtps::Guid participant = make_guid(0x40, 0x01, 0xC1);

    uint8_t buf[256];
    const uint32_t n = table.encode(participant, buf, sizeof(buf));
    ASSERT_GT(n, 0u);
    EXPECT_EQ(table.encode(participant, buf, n - 1), 0u);
    EXPECT_EQ(table.encode(participant, buf, n), n);
}

TEST(LocalNodeEntities, MeasuresExactlyWhatItEncodes) {
    rmw_rosette::Node nodes[2] = {};
    set_node(nodes[0], "/mach", "b_67751d2a");
    set_node(nodes[1], "/", "n");
    rmw_rosette::LocalNodeEntities table;
    const rtps::Guid               participant = make_guid(0x50, 0x01, 0xC1);
    uint8_t                        buf[2048];
    EXPECT_EQ(table.encoded_size(participant), table.encode(participant, buf, sizeof(buf)));

    ASSERT_TRUE(table.add_node(nodes[0]));
    ASSERT_TRUE(table.add_node(nodes[1]));
    // Enough endpoints to pass 1024 octets with either Gid size (24 or 16 octets).
    for (uint8_t i = 0; table.encoded_size(participant) <= 1024u; ++i) {
        ASSERT_TRUE(table.associate(nodes[i % 2], make_guid(0x50, i, 0x03), i % 3 == 0));
    }

    const uint32_t n = table.encoded_size(participant);
    ASSERT_GT(n, 1024u);
    EXPECT_EQ(table.encode(participant, buf, n), n);
    EXPECT_EQ(table.encode(participant, buf, n - 1), 0u);
}
