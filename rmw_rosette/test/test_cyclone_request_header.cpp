// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_cyclone_request_header.cpp
 * @brief The rmw_cyclonedds_cpp service dialect: request header, request identity, and
 *   the endpoint USER_DATA that pairs a service's endpoints.
 *
 * Verification:
 * - Correct means: a request payload laid out as rmw_cyclonedds_cpp's
 *   serialize_top_level() writes it yields its header and message; a body
 *   encoded with a reserved header carries the message exactly as without
 *   one; a packed Cyclone request identity names its request writer while
 *   that writer is matched and nothing once it is removed; a service's two
 *   endpoints announce one `serviceid=` USER_DATA string in the form
 *   csid_to_string() writes.
 *
 * References:
 * - rmw_cyclonedds_cpp 1.3.5 serdata.hpp (cdds_request_header_t),
 *   Serialization.cpp (serialize_top_level), rmw_node.cpp (rmw_send_response,
 *   get_unique_csid, csid_to_string, rmw_init_cs)
 */

#include <gtest/gtest.h>

#include "rcl_default_init_options.hpp"

#include <rmw/rmw.h>
#include <rosidl_runtime_c/string_functions.h>

#include <std_srvs/srv/set_bool.h>

#include "rmw_rosette/compat/cyclone_request_header.hpp"
#include "rmw_rosette/compat/cyclone_service_user_data.hpp"
#include "rmw_rosette/message_codec.hpp"
#include "rmw_rosette/types.hpp"

#include <cstring>
#include <vector>

namespace rtps = rosette::rtps;

namespace {

const rosidl_service_type_support_t* set_bool() {
    return ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);
}

TEST(CycloneRequestHeader, DecodesARequestAsCycloneWritesIt) {
    // CDR_LE encapsulation, client id 0x1122334455667788, sequence 7, then
    // SetBool_Request{data: true}.
    const uint8_t payload[] = {0x00, 0x01, 0x00, 0x00, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22,
                               0x11, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
    rmw_rosette::CycloneRequestHeader header{};
    ASSERT_TRUE(rmw_rosette::read_cyclone_request_header(payload, sizeof(payload), header));
    EXPECT_EQ(header.client_id, 0x1122334455667788ull);
    EXPECT_EQ(header.sequence, 7);

    rmw_rosette::MessageCodec request{};
    rmw_rosette::MessageCodec response{};
    ASSERT_TRUE(rmw_rosette::resolve_service_codecs(set_bool(), request, response));
    std_srvs__srv__SetBool_Request req{};
    std_srvs__srv__SetBool_Request__init(&req);
    EXPECT_TRUE(rmw_rosette::deserialize_message(request, payload, sizeof(payload), &req,
                                                 rmw_rosette::kCycloneRequestHeaderSize));
    EXPECT_TRUE(req.data);
    std_srvs__srv__SetBool_Request__fini(&req);
}

TEST(CycloneRequestHeader, RejectsShortAndBigEndianPayloads) {
    rmw_rosette::CycloneRequestHeader header{};
    const uint8_t                     short_payload[19] = {0x00, 0x01};
    EXPECT_FALSE(
        rmw_rosette::read_cyclone_request_header(short_payload, sizeof(short_payload), header));

    const uint8_t big_endian[20] = {0x00, 0x00};
    EXPECT_FALSE(rmw_rosette::read_cyclone_request_header(big_endian, sizeof(big_endian), header));
}

TEST(CycloneRequestHeader, ReservedHeaderLeavesTheMessageEncodingUnchanged) {
    rmw_rosette::MessageCodec request{};
    rmw_rosette::MessageCodec response{};
    ASSERT_TRUE(rmw_rosette::resolve_service_codecs(set_bool(), request, response));
    std_srvs__srv__SetBool_Response res{};
    std_srvs__srv__SetBool_Response__init(&res);
    res.success = true;
    ASSERT_TRUE(rosidl_runtime_c__String__assign(&res.message, "done"));

    uint8_t        plain[64] = {};
    const uint32_t plain_len = rmw_rosette::serialize_message(response, &res, plain, sizeof(plain));
    ASSERT_GT(plain_len, 0u);

    rtps::BufRef body =
        rmw_rosette::serialize_to_pool(response, &res, rmw_rosette::kCycloneRequestHeaderSize);
    ASSERT_TRUE(body.valid());
    rmw_rosette::write_cyclone_request_header({0x0102030405060708ull, -2}, body.data);
    ASSERT_EQ(body.size, rmw_rosette::kCycloneRequestHeaderSize + plain_len);
    const uint8_t expected_header[] = {0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
                                       0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    EXPECT_EQ(std::memcmp(body.data, expected_header, sizeof(expected_header)), 0);
    EXPECT_EQ(std::memcmp(body.data + rmw_rosette::kCycloneRequestHeaderSize, plain, plain_len), 0);

    // The writer prepends the encapsulation; decode what a Cyclone client receives.
    std::vector<uint8_t> payload = {0x00, 0x01, 0x00, 0x00};
    payload.insert(payload.end(), body.data, body.data + body.size);
    std_srvs__srv__SetBool_Response decoded{};
    std_srvs__srv__SetBool_Response__init(&decoded);
    EXPECT_TRUE(rmw_rosette::deserialize_message(response, payload.data(),
                                                 static_cast<uint32_t>(payload.size()), &decoded,
                                                 rmw_rosette::kCycloneRequestHeaderSize));
    EXPECT_TRUE(decoded.success);
    EXPECT_STREQ(decoded.message.data, "done");
    std_srvs__srv__SetBool_Response__fini(&decoded);
    std_srvs__srv__SetBool_Response__fini(&res);
}

// The id is the GUID prefix and a big-endian counter in lowercase hex, dot
// separated, without leading zeros; the first octet is padded to two
// characters with a space, as std::setw(2) pads it.
TEST(CycloneServiceUserData, MatchesCycloneForm) {
    rtps::GuidPrefix prefix{};
    const uint8_t    octets[12] = {0x01, 0x0f, 0x10, 0xab, 0, 0, 0, 0, 0, 0, 0, 0xff};
    std::memcpy(prefix.value, octets, sizeof(octets));
    char out[rmw_rosette::kCycloneServiceUserDataSize];

    rmw_rosette::compose_cyclone_service_user_data(rmw_rosette::ServiceRole::Client, prefix,
                                                   0x00010203u, out);
    EXPECT_STREQ(out, "clientid= 1.f.10.ab.0.0.0.0.0.0.0.ff.0.1.2.3;");

    std::memset(prefix.value, 0xff, sizeof(prefix.value));
    rmw_rosette::compose_cyclone_service_user_data(rmw_rosette::ServiceRole::Server, prefix,
                                                   0xffffffffu, out);
    EXPECT_STREQ(out, "serviceid=ff.ff.ff.ff.ff.ff.ff.ff.ff.ff.ff.ff.ff.ff.ff.ff;");
    EXPECT_EQ(std::strlen(out) + 1u, sizeof(out));
}

/// A SetBool service in its own context, whose request reader the test matches by hand.
class CycloneRequester : public ::testing::Test {
protected:
    void SetUp() override {
        rmw_init_options_t options = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&options), RMW_RET_OK);
        ASSERT_EQ(rmw_init(&options, &context_), RMW_RET_OK);
        ASSERT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
        node_ = rmw_create_node(&context_, "cyclone_requester_test", "/");
        ASSERT_NE(node_, nullptr);
        const rmw_qos_profile_t qos = rmw_qos_profile_services_default;
        service_ = rmw_create_service(node_, set_bool(), "/cyclone_requester", &qos);
        ASSERT_NE(service_, nullptr);
    }

    void TearDown() override {
        EXPECT_EQ(rmw_destroy_service(node_, service_), RMW_RET_OK);
        EXPECT_EQ(rmw_destroy_node(node_), RMW_RET_OK);
        EXPECT_EQ(rmw_shutdown(&context_), RMW_RET_OK);
        EXPECT_EQ(rmw_context_fini(&context_), RMW_RET_OK);
    }

    rmw_rosette::Service& impl() { return *static_cast<rmw_rosette::Service*>(service_->data); }

    rmw_context_t  context_ = rmw_get_zero_initialized_context();
    rmw_node_t*    node_ = nullptr;
    rmw_service_t* service_ = nullptr;
};

TEST_F(CycloneRequester, NamesItsWriterOnlyWhileMatched) {
    rtps::Guid writer{};
    for (uint8_t i = 0; i < 12; ++i) {
        writer.prefix.value[i] = static_cast<uint8_t>(0xa0 + i);
    }

    writer.entityId = {{0x00, 0x00, 0x12}, 0x03};
    rtps::Guid other = writer;
    other.entityId.entityKey[2] = 0x13;
    EXPECT_NE(rmw_rosette::request_writer_key(writer), rmw_rosette::request_writer_key(other));

    rmw_request_id_t id{};
    rmw_rosette::cyclone_request_to_request_id({0x5566778899aabbccull, 42}, writer, id);
    EXPECT_EQ(id.sequence_number, 42);

    rosette::platform::LockGuard      guard(impl().node->context->driver->mutex());
    rmw_rosette::CycloneRequestHeader header{};
    EXPECT_FALSE(rmw_rosette::find_cyclone_requester(*impl().request_reader, id, header));

    const rtps::Locator locator = rtps::make_udpv4_locator(0x7f000001u, 7411);
    ASSERT_TRUE(impl().request_reader->matched_writer_add(writer, &locator, 1u));
    ASSERT_TRUE(rmw_rosette::find_cyclone_requester(*impl().request_reader, id, header));
    EXPECT_EQ(header.client_id, 0x5566778899aabbccull);
    EXPECT_EQ(header.sequence, 42);

    // A DDS-RPC lookup by GUID must not mistake the packed identity for a writer.
    EXPECT_EQ(impl().request_reader->matched_writer_lookup(
                  rmw_rosette::guid_from_request_writer(id.writer_guid)),
              nullptr);

    impl().request_reader->matched_writer_remove(writer);
    EXPECT_FALSE(rmw_rosette::find_cyclone_requester(*impl().request_reader, id, header));
}

// A service announces one `serviceid=` string, naming its own participant, on
// its request reader and its reply writer.
TEST_F(CycloneRequester, AnnouncesOneServiceIdOnBothEndpoints) {
    rtps::RTPSParticipant&     participant = *impl().node->context->participant;
    rtps::DiscoveredReaderData reader{};
    rtps::DiscoveredWriterData writer{};
    ASSERT_TRUE(participant.match_table().local_reader_data(impl().request_reader->guid(), reader));
    ASSERT_TRUE(participant.match_table().local_writer_data(impl().reply_writer->guid(), writer));
    ASSERT_NE(reader.userData, nullptr);
    ASSERT_NE(writer.userData, nullptr);
    EXPECT_STREQ(reader.userData, writer.userData);

    char expected[rmw_rosette::kCycloneServiceUserDataSize];
    rmw_rosette::compose_cyclone_service_user_data(
        rmw_rosette::ServiceRole::Server, participant.guid_prefix(),
        impl().node->context->service_ids_issued, expected);
    EXPECT_STREQ(reader.userData, expected);
}

}  // namespace
