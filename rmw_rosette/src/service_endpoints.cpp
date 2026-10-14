/**
 * @file service_endpoints.cpp
 * @brief Creation and teardown of the RTPS endpoint pair behind a service or client.
 */

#include "rmw_rosette/service_endpoints.hpp"

#include "rmw_rosette/capacity.hpp"
#include "rmw_rosette/qos.hpp"
#include "rmw_rosette/topic_naming.hpp"

namespace rmw_rosette {

ServiceEndpointsStatus create_service_endpoints(Node& node, ServiceRole role,
                                                const char* service,
                                                const rmw_qos_profile_t& qos,
                                                const MessageCodec& request,
                                                const MessageCodec& response,
                                                rtps::StatefulReader::OnChangeAvailable on_change,
                                                void* user_data, rtps::StatefulWriter*& writer,
                                                rtps::StatefulReader*& reader) noexcept {
    writer = nullptr;
    reader = nullptr;
    rtps::RTPSParticipant& participant = *node.context->participant;
    const bool server = (role == ServiceRole::Server);

    char request_topic[RTPS_MAX_TOPIC_NAME_LEN];
    char reply_topic[RTPS_MAX_TOPIC_NAME_LEN];
    char request_type[RTPS_MAX_TOPIC_NAME_LEN];
    char reply_type[RTPS_MAX_TOPIC_NAME_LEN];
    if (!compose_service_request_topic(request_topic, sizeof(request_topic), service) ||
        !compose_service_reply_topic(reply_topic, sizeof(reply_topic), service) ||
        !compose_codec_type_name(request, request_type, sizeof(request_type)) ||
        !compose_codec_type_name(response, reply_type, sizeof(reply_type))) {
        return ServiceEndpointsStatus::NameTooLong;
    }

    rtps::WriterConfig wcfg{};
    fill_writer_config(wcfg, qos);
    // A client may see the service as available (its endpoints matched ours)
    // before this reply writer has matched the client's reply reader. A
    // TRANSIENT_LOCAL reply writer delivers retained replies to readers that
    // match late, so a reply sent in that window still arrives; clients drop
    // replies correlated with other clients' requests. Each reply leaves the
    // history once its requester has it, so the history holds only replies
    // still in flight rather than the last depth replies. Its depth,
    // RMW_ROSETTE_REPLY_DEPTH, bounds the replies in flight at once whatever
    // `qos` requests.
    if (server) {
        wcfg.durability = rtps::DurabilityKind::TRANSIENT_LOCAL;
        wcfg.releaseRepliesOnDelivery = true;
    }

    wcfg.topicName.assign(server ? reply_topic : request_topic);
    wcfg.typeName.assign(server ? reply_type : request_type);

    rtps::ReaderConfig rcfg{};
    fill_reader_config(rcfg, qos);
    rcfg.topicName.assign(server ? request_topic : reply_topic);
    rcfg.typeName.assign(server ? request_type : reply_type);

    writer = server ? participant.create_writer<RMW_ROSETTE_REPLY_DEPTH>(wcfg)
                    : create_writer_for_qos(participant, wcfg, qos);
    if (!writer) {
        return ServiceEndpointsStatus::TablesFull;
    }

    reader = participant.create_reader(rcfg, on_change, user_data);
    if (!reader) {
        participant.delete_writer(writer);
        writer = nullptr;
        return ServiceEndpointsStatus::TablesFull;
    }
    if (!node.context->discovery_info.add_endpoints(node, &reader->guid(), &writer->guid())) {
        participant.delete_reader(reader);
        participant.delete_writer(writer);
        reader = nullptr;
        writer = nullptr;
        return ServiceEndpointsStatus::NotAnnounced;
    }
    return ServiceEndpointsStatus::Ok;
}

void destroy_service_endpoints(Node& node, rtps::StatefulWriter*& writer,
                               rtps::StatefulReader*& reader) noexcept {
    rtps::RTPSParticipant& participant = *node.context->participant;
    node.context->discovery_info.remove_endpoints(reader ? &reader->guid() : nullptr,
                                                  writer ? &writer->guid() : nullptr);
    if (reader) participant.delete_reader(reader);
    if (writer) participant.delete_writer(writer);
    reader = nullptr;
    writer = nullptr;
}

}  // namespace rmw_rosette
