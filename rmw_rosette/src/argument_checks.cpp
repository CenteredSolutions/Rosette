/**
 * @file argument_checks.cpp
 * @brief The one place an rmw_* entry point's argument refusal is reported.
 *
 * See rmw_rosette/argument_checks.hpp for the checks and the return codes.
 */

#include "rmw_rosette/argument_checks.hpp"

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/ros_features.h"
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

/// @brief Whether `history` names a history policy an endpoint can have.
bool is_endpoint_history(rmw_qos_history_policy_t history) noexcept {
    switch (history) {
        case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
        case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
        case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
            return true;
        default:
            return false;
    }
}

/// @brief Whether `reliability` names a reliability policy an endpoint can have.
bool is_endpoint_reliability(rmw_qos_reliability_policy_t reliability) noexcept {
    switch (reliability) {
        case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
        case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
        case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
#if RMW_ROSETTE_HAS(QOS_BEST_AVAILABLE)
        case RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE:
#endif
            return true;
        default:
            return false;
    }
}

/// @brief Whether `durability` names a durability policy an endpoint can have.
bool is_endpoint_durability(rmw_qos_durability_policy_t durability) noexcept {
    switch (durability) {
        case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
        case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
        case RMW_QOS_POLICY_DURABILITY_VOLATILE:
#if RMW_ROSETTE_HAS(QOS_BEST_AVAILABLE)
        case RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE:
#endif
            return true;
        default:
            return false;
    }
}

/**
 * @brief Whether `liveliness` names a liveliness policy an endpoint can have.
 *
 * MANUAL_BY_NODE is deprecated but still part of the enumeration, so it is
 * accepted; it is compared by value to stay clear of the deprecation warning
 * its enumerator carries.
 */
bool is_endpoint_liveliness(rmw_qos_liveliness_policy_t liveliness) noexcept {
    constexpr int kManualByNode = 2;
    switch (liveliness) {
        case RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT:
        case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
        case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
#if RMW_ROSETTE_HAS(QOS_BEST_AVAILABLE)
        case RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE:
#endif
            return true;
        default:
            return static_cast<int>(liveliness) == kManualByNode;
    }
}

/**
 * @brief Whether every policy of a profile names one an endpoint can have.
 *
 * UNKNOWN (rmw_qos_profile_unknown) and values outside the enumerations the
 * paired distribution defines describe no endpoint; BEST_AVAILABLE is
 * accepted where that distribution defines it.
 */
bool is_endpoint_profile(const rmw_qos_profile_t& qos) noexcept {
    return is_endpoint_history(qos.history) && is_endpoint_reliability(qos.reliability)
           && is_endpoint_durability(qos.durability) && is_endpoint_liveliness(qos.liveliness);
}

}  // namespace

rmw_ret_t refuse_argument(const char* function, const char* argument,
                          ArgumentFault fault) noexcept {
    // A message longer than the buffer is truncated, which still names the
    // function and argument; only an encoding failure leaves no message.
    char      message[128];
    const int written = rcutils_snprintf(message, sizeof(message), "rmw_rosette: %s: %s %s",
                                         function, argument, describe(fault));
    if (written < 0) {
        message[0] = '\0';
    }

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

    if (context->implementation_identifier == nullptr) {
        return refuse_argument(function, "context", ArgumentFault::uninitialized);
    }

    if (!is_rosette_identifier(context->implementation_identifier)) {
        return refuse_argument(function, "context", ArgumentFault::foreign);
    }

    if (context->impl == nullptr) {
        return refuse_argument(function, "context", ArgumentFault::uninitialized);
    }

    if (require_live && context->impl->rosette_context.shut_down.load()) {
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

    const rmw_ret_t context_ret = check_context(node->context, function, true);
    if (context_ret != RMW_RET_OK) {
        return context_ret;
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

    if (!is_endpoint_profile(*qos)) {
        return refuse_argument(function, "qos_profile", ArgumentFault::invalid);
    }

    return RMW_RET_OK;
}

}  // namespace rmw_rosette
