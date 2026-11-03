// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/service_endpoints.hpp
 * @brief RTPS endpoint pairs behind ROS 2 services and clients.
 *
 * A ROS 2 service is two DDS topics (ROS 2 design, "Topic and Service name
 * mapping to DDS"):
 *   request  rq/<service>Request   type <pkg>::srv::dds_::<Srv>_Request_
 *   reply    rr/<service>Reply     type <pkg>::srv::dds_::<Srv>_Response_
 * A server reads requests and writes replies; a client does the opposite.
 * This header creates and destroys that pair for either role, so
 * rmw_service.cpp and rmw_client.cpp contain only their correlation logic.
 *
 * Correlation follows rmw_fastrtps_cpp, the Humble default, through
 * PID_RELATED_SAMPLE_IDENTITY in the DATA inline QoS:
 *   - a request carries {client reply-reader GUID, unknown SN};
 *   - a reply carries {request writer GUID, request SN}, which is exactly
 *     the rmw_request_id_t the server took the request with. rmw_fastrtps_cpp
 *     servers substitute the client's reply-reader GUID from the request, so
 *     clients accept replies naming either of their two endpoints.
 */

#pragma once

#include "types.hpp"

#include <rmw/types.h>

namespace rmw_rosette {

/// Which side of a service an endpoint pair serves.
enum class ServiceRole : uint8_t {
    Server,  ///< Reads requests, writes replies.
    Client,  ///< Writes requests, reads replies.
};

/// Outcome of create_service_endpoints().
enum class ServiceEndpointsStatus : uint8_t {
    Ok,           ///< Both endpoints exist.
    NameTooLong,  ///< A topic or type name exceeds RTPS_MAX_TOPIC_NAME_LEN - 1.
    TablesFull,   ///< The participant has no free writer or reader slot.
    NotAnnounced, ///< The updated `ros_discovery_info` message could not be written.
};

/**
 * @brief Creates the RTPS writer and reader for one service or client.
 *
 * Composes the request and reply topic and type names, creates the writer
 * and the reader with `on_change` as its delivery callback. A client's
 * request writer has a depth chosen from `qos` (create_writer_for_qos()). A
 * server's reply writer is TRANSIENT_LOCAL with depth RMW_ROSETTE_REPLY_DEPTH
 * whatever `qos` requests: a reply written before the calling client's reply
 * reader has matched is still delivered once it does, and the history holds
 * the replies in flight at once (capacity.hpp).
 * Both endpoints are then associated with `node` on `ros_discovery_info`
 * (ros_discovery_info_writer.hpp), which is how stock peers attribute the
 * service or client to its node. On any failure, whatever was created is
 * deleted again, so the participant tables are left unchanged.
 *
 * @param[in]  node        Node the entity belongs to; supplies the participant.
 * @param[in]  role        Server or client.
 * @param[in]  service     Fully-qualified service name, as rcl passes it.
 * @param[in]  qos         Requested QoS for both endpoints.
 * @param[in]  request     Codec of the request type (for its DDS type name).
 * @param[in]  response    Codec of the response type.
 * @param[in]  on_change   Reader callback; receives `user_data`.
 * @param[in]  user_data   Owning Service or Client.
 * @param[out] writer      Receives the created writer.
 * @param[out] reader      Receives the created reader.
 * @return ServiceEndpointsStatus::Ok when both endpoints exist; otherwise
 *   `writer` and `reader` are null and the status says why.
 *
 * Verification:
 * - Correct means: the endpoints' SEDP announcements carry the names
 *   rmw_fastrtps_cpp uses, so a stock client matches a rosette server and
 *   the reverse.
 * - Covered by: tests/ros2 service interop (stock rclpy parameter services).
 *
 * References:
 * - ROS 2 design, "Topic and Service name mapping to DDS"
 * - rmw_fastrtps_shared_cpp (Humble), rmw_service.cpp / rmw_client.cpp
 */
ServiceEndpointsStatus create_service_endpoints(
    Node& node, ServiceRole role, const char* service, const rmw_qos_profile_t& qos,
    const MessageCodec& request, const MessageCodec& response,
    rosette::rtps::StatefulReader::OnChangeAvailable on_change, void* user_data,
    rosette::rtps::StatefulWriter*& writer, rosette::rtps::StatefulReader*& reader) noexcept;

/**
 * @brief Deletes a service's or client's endpoints from the participant.
 *
 * The endpoints are dissociated from `node` on `ros_discovery_info`, and each
 * deletion withdraws the endpoint through SEDP, so peers unmatch it.
 *
 * @param[in]     node   Node the entity belongs to.
 * @param[in,out] writer Writer to delete; set to null.
 * @param[in,out] reader Reader to delete; set to null.
 */
void destroy_service_endpoints(Node& node, rosette::rtps::StatefulWriter*& writer,
                               rosette::rtps::StatefulReader*& reader) noexcept;

}  // namespace rmw_rosette
