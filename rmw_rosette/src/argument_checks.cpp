/**
 * @file argument_checks.cpp
 * @brief The one place an rmw_* entry point's argument refusal is reported.
 *
 * See rmw_rosette/argument_checks.hpp for the checks and the return codes.
 */

#include "rmw_rosette/argument_checks.hpp"

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/types.hpp"

#include <rcutils/error_handling.h>
#include <rcutils/snprintf.h>
#include <rmw/validate_full_topic_name.h>

#include <cstring>

namespace rmw_rosette {

namespace {

/// @brief The message text for a fault, read after the argument's name.
const char* describe(ArgumentFault fault) noexcept {
    switch (fault) {
        case ArgumentFault::null:
            return "is null";
        case ArgumentFault::foreign:
            return "belongs to another rmw implementation";
        case ArgumentFault::uninitialized:
            return "is not initialized";
        case ArgumentFault::already_in_use:
            return "is already initialized";
        case ArgumentFault::invalid:
            return "is invalid";
        case ArgumentFault::shut_down:
            return "has been shut down";
    }

    return "is invalid";
}

/// @brief Whether a profile leaves a policy the endpoint needs UNKNOWN.
bool has_unknown_policy(const rmw_qos_profile_t& qos) noexcept {
    return qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN
           || qos.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN
           || qos.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN
           || qos.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
}

}  // namespace

rmw_ret_t refuse_argument(const char* function, const char* argument,
                          ArgumentFault fault) noexcept {
    char message[128];
    (void)rcutils_snprintf(message, sizeof(message), "rmw_rosette: %s: %s %s", function, argument,
                           describe(fault));
    rcutils_set_error_state(message, __FILE__, __LINE__);
    return fault == ArgumentFault::foreign ? RMW_RET_INCORRECT_RMW_IMPLEMENTATION
                                           : RMW_RET_INVALID_ARGUMENT;
}

bool is_rosette_identifier(const char* identifier) noexcept {
    return identifier != nullptr
           && (identifier == rmw_rosette_identifier
               || std::strcmp(identifier, rmw_rosette_identifier) == 0);
}

rmw_ret_t check_context(const rmw_context_t* context, const char* function,
                        bool require_live) noexcept {
    if (context == nullptr) {
        return refuse_argument(function, "context", ArgumentFault::null);
    }

    if (context->implementation_identifier == nullptr || context->impl == nullptr) {
        return refuse_argument(function, "context", ArgumentFault::uninitialized);
    }

    if (!is_rosette_identifier(context->implementation_identifier)) {
        return refuse_argument(function, "context", ArgumentFault::foreign);
    }

    if (require_live && context->impl->rosette_context.shut_down) {
        return refuse_argument(function, "context", ArgumentFault::shut_down);
    }

    return RMW_RET_OK;
}

rmw_ret_t check_endpoint_arguments(const rmw_node_t* node, const void* type_support,
                                   const char* name, const rmw_qos_profile_t* qos,
                                   const char* function) noexcept {
    const rmw_ret_t node_ret = check_handle(node, function, "node");
    if (node_ret != RMW_RET_OK) {
        return node_ret;
    }

    if (type_support == nullptr) {
        return refuse_argument(function, "type_support", ArgumentFault::null);
    }

    if (name == nullptr || name[0] == '\0') {
        return refuse_argument(function, "name",
                               name == nullptr ? ArgumentFault::null : ArgumentFault::invalid);
    }

    if (qos == nullptr) {
        return refuse_argument(function, "qos_profile", ArgumentFault::null);
    }

    if (!qos->avoid_ros_namespace_conventions) {
        int validation = RMW_TOPIC_VALID;
        if (rmw_validate_full_topic_name(name, &validation, nullptr) != RMW_RET_OK) {
            return RMW_RET_ERROR;
        }

        if (validation != RMW_TOPIC_VALID) {
            return refuse_argument(function, "name", ArgumentFault::invalid);
        }
    }

    if (has_unknown_policy(*qos)) {
        return refuse_argument(function, "qos_profile", ArgumentFault::invalid);
    }

    return RMW_RET_OK;
}

}  // namespace rmw_rosette
