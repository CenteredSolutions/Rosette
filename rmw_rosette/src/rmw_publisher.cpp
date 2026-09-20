/**
 * @file rmw_publisher.cpp
 * @brief rmw_create_publisher / rmw_publish / matched-subscription queries.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/qos.hpp"
#include "rmw_rosette/topic_naming.hpp"
#include "rmw_rosette/type_support.hpp"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

rmw_publisher_t* rmw_create_publisher(
    const rmw_node_t* node,
    const rosidl_message_type_support_t* type_support,
    const char* topic_name,
    const rmw_qos_profile_t* qos_policies,
    const rmw_publisher_options_t* publisher_options) {
    (void)node; (void)type_support; (void)topic_name;
    (void)qos_policies; (void)publisher_options;
    // TODO:
    //   1. rmw_rosette::compose_topic_name() into a stack TopicName.
    //   2. rmw_rosette::get_message_typesupport() to resolve the
    //      microxrcedds callbacks + type name string.
    //   3. rtps::WriterConfig cfg; fill_writer_config(cfg, *qos_policies);
    //      cfg.topicName = ...; cfg.typeName = ...;
    //   4. rmw_rosette::create_writer_for_qos(*participant, cfg, *qos_policies)
    //      — resolves the runtime depth request against a small closed
    //      set of StatefulWriter<D,...> instantiations and hands back the
    //      type-erased rtps::IStatefulWriter* (see qos.hpp, types.hpp).
    //   5. Allocate rmw_rosette::Publisher, allocate + fill rmw_publisher_t.
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_publisher not implemented");
    return nullptr;
}

rmw_ret_t rmw_destroy_publisher(rmw_node_t* node, rmw_publisher_t* publisher) {
    (void)node; (void)publisher;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_destroy_publisher not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publish(const rmw_publisher_t* publisher, const void* ros_message,
                       rmw_publisher_allocation_t* allocation) {
    (void)publisher; (void)ros_message; (void)allocation;
    // TODO:
    //   1. Look up the Publisher's type_support callbacks.
    //   2. Serialize ros_message into a stack buffer via the
    //      microxrcedds _cdr_serialize callback (no new codegen needed —
    //      see type_support.hpp).
    //   3. publisher_impl->writer->write_reliable(ChangeKind::ALIVE, buf, len)
    //      (or new_change()+add_change() for best-effort/async — see
    //      stateful_writer.hpp).
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_publish not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publish_serialized_message(const rmw_publisher_t* publisher,
                                          const rmw_serialized_message_t* serialized_message,
                                          rmw_publisher_allocation_t* allocation) {
    (void)publisher; (void)serialized_message; (void)allocation;
    // Same as rmw_publish() minus the serialize step — the caller already
    // has CDR bytes. Trivial once rmw_publish() exists.
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_publish_serialized_message not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publisher_count_matched_subscriptions(const rmw_publisher_t* publisher,
                                                     size_t* subscription_count) {
    (void)publisher;
    if (!subscription_count) return RMW_RET_INVALID_ARGUMENT;
    // TODO: IStatefulWriter has no public matched-reader count accessor
    // today (see stateful_writer.hpp's matched_reader_add/remove) — either
    // add one or track it alongside GraphCache.
    *subscription_count = 0;
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t* publisher, rmw_gid_t* gid) {
    (void)publisher; (void)gid;
    // Trivial once Publisher stores its writer's guid() — see guid.hpp.
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_get_gid_for_publisher not implemented");
    return RMW_RET_UNSUPPORTED;
}

// Deferred, not yet stubbed: rmw_publisher_assert_liveliness (no
// LIVELINESS QoS in rtps yet), rmw_publisher_wait_for_all_acked (no
// direct equivalent — write_reliable() already blocks internally, but
// that's a different contract), rmw_publisher_get_actual_qos,
// rmw_borrow_loaned_message / rmw_publish_loaned_message (loaned-message
// support assumes a memory model this stack doesn't have),
// rmw_publisher_get_network_flow_endpoints.

}  // extern "C"
