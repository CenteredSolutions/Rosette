/**
 * @file rmw_rosette/argument_checks.hpp
 * @brief The argument and handle checks every rmw_* entry point makes before it acts.
 *
 * rcl validates most arguments before it calls into the rmw layer, but the
 * rmw API is public: rmw_implementation forwards to it, and
 * test_rmw_implementation calls it directly with null pointers, handles of
 * another implementation, zero-initialized handles and invalid names. The
 * rmw header documents the result for each, and the stock implementations
 * (rmw_cyclonedds_cpp, rmw_fastrtps_cpp) return it:
 *
 *   null argument                        RMW_RET_INVALID_ARGUMENT
 *   handle of another implementation     RMW_RET_INCORRECT_RMW_IMPLEMENTATION
 *   handle not initialized (zero-init)   RMW_RET_INVALID_ARGUMENT
 *   invalid name, QoS or allocator       RMW_RET_INVALID_ARGUMENT
 *
 * A check runs before anything is acquired or changed, so a refused call
 * leaves every pool and every handle exactly as it was. Without that, a
 * create that should have been refused takes a pool slot nobody releases,
 * and with pools of a few slots each every later create in the process
 * fails.
 *
 * Every refusal is reported through one out-of-line function, which formats
 * "rmw_rosette: <function>: <argument> <fault>" into the rcutils error state.
 * The function, argument and fault strings are shared by every call site, so
 * the checks cost a call and a few short literals each in a firmware image,
 * not a distinct message string per check.
 */

#pragma once

#include <rmw/init.h>
#include <rmw/ret_types.h>
#include <rmw/types.h>

#include <initializer_list>

namespace rmw_rosette {

/// @brief Why an argument was refused; selects the message and the return code.
enum class ArgumentFault {
    null,            ///< The pointer is null.
    foreign,         ///< The handle belongs to another rmw implementation.
    uninitialized,   ///< The handle has no implementation data (zero-initialized or finalized).
    already_in_use,  ///< An output handle that must be zero-initialized is not.
    invalid,         ///< The value is outside what the rmw API accepts.
    shut_down,       ///< The context was shut down.
};

/**
 * @brief Records why an rmw_* call refused an argument and returns the code to report.
 *
 * Called by the entry points through the check helpers below, and directly
 * for refusals they cannot express (an invalid name, a QoS value). It
 * replaces the rcutils error state, as RMW_SET_ERROR_MSG does.
 *
 * @param[in] function The refusing entry point, normally `__func__`; static storage.
 * @param[in] argument The parameter's name as the rmw header spells it; static storage.
 * @param[in] fault    Why it was refused.
 * @return RMW_RET_INCORRECT_RMW_IMPLEMENTATION for `foreign`, RMW_RET_INVALID_ARGUMENT
 *   for every other fault.
 *
 * Verification:
 * - Correct means: the error string reads "rmw_rosette: <function>: <argument> <fault>"
 *   and the return code follows the table in the file comment.
 * - Covered by: rmw_rosette/test/test_argument_checks.cpp (TC-RMW-ARG-001).
 *
 * References:
 * - rmw/error_handling.h, RMW_SET_ERROR_MSG
 * - rmw/impl/cpp/macros.hpp, RMW_CHECK_TYPE_IDENTIFIERS_MATCH
 */
rmw_ret_t refuse_argument(const char* function, const char* argument, ArgumentFault fault) noexcept;

/**
 * @brief Whether an implementation identifier is rmw_rosette's.
 *
 * Compares by content, as RMW_CHECK_TYPE_IDENTIFIERS_MATCH does, so a handle
 * whose identifier was copied rather than pointed at still matches.
 *
 * @param[in] identifier The handle's `implementation_identifier`; may be null.
 * @return true when it names rmw_rosette; false when it is null or names another.
 */
bool is_rosette_identifier(const char* identifier) noexcept;

/// @brief A pointer argument that must not be null, and its name for the error message.
struct RequiredArgument {
    const void* pointer;
    const char* name;
};

/**
 * @brief Checks a handle argument: present, rmw_rosette's, and initialized.
 *
 * Fits every rmw handle type that carries `implementation_identifier` and
 * `data` (node, publisher, subscription, client, service, guard condition,
 * wait set, event). rmw_context_t keeps its state in `impl` and uses
 * check_context() instead. The entry point's other pointer arguments are
 * checked in the same call, after the handle, so an operation states every
 * precondition in one line.
 *
 * @param[in] handle   The handle; may be null.
 * @param[in] function The checking entry point, normally `__func__`.
 * @param[in] argument The parameter's name.
 * @param[in] required The entry point's other pointer arguments that must not be null.
 * @return RMW_RET_OK when the handle is usable; otherwise the refusal's code, with the
 *   error state set.
 *
 * Verification:
 * - Correct means: null is refused before the identifier is read; a null
 *   identifier (a zero-initialized handle) is refused as uninitialized; a
 *   foreign identifier is refused before the data pointer is read; and a
 *   usable handle is not touched.
 * - Covered by: rmw_rosette/test/test_argument_checks.cpp (TC-RMW-ARG-002), and
 *   test_rmw_implementation's *_with_bad_arguments and *_of_another_impl cases.
 *
 * References:
 * - rmw/rmw.h, the INVALID_ARGUMENT and INCORRECT_RMW_IMPLEMENTATION results of each entry point
 */
template <typename Handle>
rmw_ret_t check_handle(const Handle* handle, const char* function, const char* argument,
                       std::initializer_list<RequiredArgument> required = {}) noexcept {
    if (handle == nullptr) {
        return refuse_argument(function, argument, ArgumentFault::null);
    }

    if (handle->implementation_identifier == nullptr) {
        return refuse_argument(function, argument, ArgumentFault::uninitialized);
    }

    if (!is_rosette_identifier(handle->implementation_identifier)) {
        return refuse_argument(function, argument, ArgumentFault::foreign);
    }

    if (handle->data == nullptr) {
        return refuse_argument(function, argument, ArgumentFault::uninitialized);
    }

    for (const RequiredArgument& other : required) {
        if (other.pointer == nullptr) {
            return refuse_argument(function, other.name, ArgumentFault::null);
        }
    }

    return RMW_RET_OK;
}

/**
 * @brief Checks a context argument: present, rmw_rosette's, initialized, and optionally live.
 *
 * rmw_context_t keeps its state in `impl` rather than `data`, and an entity
 * may only be created in a context that has not been shut down
 * (rmw_shutdown()), while finalizing entities is allowed after shutdown.
 *
 * @param[in] context        The context; may be null.
 * @param[in] function       The checking entry point, normally `__func__`.
 * @param[in] require_live   true to also refuse a context that was shut down.
 * @return RMW_RET_OK when the context is usable; otherwise the refusal's code, with the
 *   error state set.
 *
 * Verification:
 * - Correct means: the faults are checked in the order null, null identifier
 *   (uninitialized), foreign identifier, null impl (uninitialized), shut
 *   down, and a usable context is not touched.
 * - Covered by: rmw_rosette/test/test_argument_checks.cpp (TC-RMW-ARG-003), and
 *   test_rmw_implementation's test_create_destroy_node.
 *
 * References:
 * - rmw/rmw.h, rmw_create_node; rmw/init.h, rmw_shutdown
 */
rmw_ret_t check_context(const rmw_context_t* context, const char* function,
                        bool require_live) noexcept;

/**
 * @brief Checks the arguments every endpoint factory shares, before anything is created.
 *
 * rmw_create_publisher(), rmw_create_subscription(), rmw_create_client()
 * and rmw_create_service() take the same four arguments in the same roles,
 * and the stock rmws check them in the same order; this is that check, so
 * the four factories refuse identically and a refused call takes no pool
 * slot. The name must be a fully qualified ROS name unless the QoS profile
 * sets `avoid_ros_namespace_conventions`, in which case it is passed to
 * DDS as given and only has to be non-empty. A profile whose history,
 * reliability, durability or liveliness is UNKNOWN (rmw_qos_profile_unknown),
 * or a value outside the enumeration the paired distribution's rmw/types.h
 * defines, describes no endpoint and is refused; BEST_AVAILABLE is accepted
 * where that distribution defines it.
 *
 * @param[in] node         The node to create in; must be rmw_rosette's, initialized, and in a
 *   context that has not been shut down.
 * @param[in] type_support The message or service type support; must be non-null.
 *   Whether it offers a typesupport family this rmw reads is the factory's check.
 * @param[in] name         The topic or service name.
 * @param[in] qos          The requested QoS profile.
 * @param[in] function     The checking factory, normally `__func__`.
 * @return RMW_RET_OK when the endpoint may be created; otherwise the refusal's code
 *   (INCORRECT_RMW_IMPLEMENTATION for a foreign node, INVALID_ARGUMENT for the rest),
 *   with the error state set; RMW_RET_ERROR if the name validator itself fails.
 *
 * Verification:
 * - Correct means: every create_with_bad_arguments case of test_rmw_implementation
 *   returns nullptr and leaves the entity pools as they were, and a node kept
 *   past rmw_shutdown() creates no endpoint.
 * - Covered by: rmw_rosette/test/test_argument_checks.cpp (TC-RMW-ARG-004),
 *   rmw_rosette/test/test_live_context_refusals.cpp (TC-RMW-ARG-005), and
 *   test_rmw_implementation's TestPublisher, TestSubscription, TestClient and
 *   TestService create_with_bad_arguments.
 *
 * References:
 * - rmw/rmw.h (Humble, Jazzy), `rmw_create_publisher` and the other factories
 * - rmw/validate_full_topic_name.h, `rmw_validate_full_topic_name`
 * - rmw/qos_profiles.h, `rmw_qos_profile_unknown`
 * - rmw/types.h, `rmw_qos_history_policy_e` and the other policy enumerations
 */
rmw_ret_t check_endpoint_arguments(const rmw_node_t* node, const void* type_support,
                                   const char* name, const rmw_qos_profile_t* qos,
                                   const char* function) noexcept;

/**
 * @brief Checks an entry point's node and endpoint handle arguments, node first.
 *
 * rmw_destroy_publisher(), rmw_destroy_subscription(), rmw_destroy_client(),
 * rmw_destroy_service() and rmw_service_server_is_available() take the node
 * that created an endpoint and the endpoint. Both are checked as handles
 * before anything is read or released, so a destroy the rmw refuses leaves
 * the endpoint to the caller.
 *
 * @param[in] node     The node argument.
 * @param[in] handle   The endpoint argument.
 * @param[in] function The checking entry point, normally `__func__`.
 * @param[in] argument The endpoint parameter's name.
 * @param[in] required The entry point's other pointer arguments that must not be null.
 * @return What check_handle() returns for the node, or else for the endpoint
 *   and `required`.
 *
 * Verification:
 * - Correct means: destroy with a foreign node or endpoint returns
 *   INCORRECT_RMW_IMPLEMENTATION, and destroying again with the identifier
 *   restored succeeds.
 * - Covered by: test_rmw_implementation's destroy_with_bad_arguments and
 *   destroy_*_of_another_impl cases.
 *
 * References:
 * - rmw/rmw.h (Humble, Jazzy), `rmw_destroy_publisher` and the other destroy calls
 */
template <typename Handle>
rmw_ret_t check_node_and_handle(const rmw_node_t* node, const Handle* handle, const char* function,
                                const char*                             argument,
                                std::initializer_list<RequiredArgument> required = {}) noexcept {
    const rmw_ret_t node_ret = check_handle(node, function, "node");
    return node_ret != RMW_RET_OK ? node_ret : check_handle(handle, function, argument, required);
}

/**
 * @brief Checks a pointer argument that is not a handle: present.
 *
 * @param[in] pointer  The argument; may be null.
 * @param[in] function The checking entry point, normally `__func__`.
 * @param[in] argument The parameter's name.
 * @return RMW_RET_OK when non-null; RMW_RET_INVALID_ARGUMENT otherwise, with the error state set.
 */
inline rmw_ret_t check_not_null(const void* pointer, const char* function,
                                const char* argument) noexcept {
    return pointer != nullptr ? RMW_RET_OK
                              : refuse_argument(function, argument, ArgumentFault::null);
}

}  // namespace rmw_rosette
