/**
 * @file rmw_loaned_message.cpp
 * @brief Loaned-message entry points, which rmw_rosette does not offer.
 *
 * Message loaning lets middleware hand the application memory it can
 * publish from or take into without a copy (zero-copy shared memory in
 * Fast DDS and Iceoryx-backed Cyclone DDS). rmw_rosette serializes every
 * sample into an RTPS datagram, so there is no middleware-owned message
 * memory to lend. Publishers and subscriptions advertise this with
 * `can_loan_messages = false`, which keeps rcl from calling any function in
 * this file; each still answers `RMW_RET_UNSUPPORTED` if called directly.
 *
 * References:
 * - rmw/rmw.h (Humble), "loaned message" API
 * - ROS 2 design, "Zero Copy via Loaned Messages"
 */

#include <rmw/rmw.h>

extern "C" {

/// @brief Unsupported: no middleware-owned message memory (see file comment).
rmw_ret_t rmw_borrow_loaned_message(const rmw_publisher_t* publisher,
                                    const rosidl_message_type_support_t* type_support,
                                    void** ros_message) {
    (void)publisher; (void)type_support; (void)ros_message;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: no middleware-owned message memory (see file comment).
rmw_ret_t rmw_return_loaned_message_from_publisher(const rmw_publisher_t* publisher,
                                                   void* loaned_message) {
    (void)publisher; (void)loaned_message;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: no middleware-owned message memory (see file comment).
rmw_ret_t rmw_publish_loaned_message(const rmw_publisher_t* publisher, void* ros_message,
                                     rmw_publisher_allocation_t* allocation) {
    (void)publisher; (void)ros_message; (void)allocation;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: no middleware-owned message memory (see file comment).
rmw_ret_t rmw_take_loaned_message(const rmw_subscription_t* subscription, void** loaned_message,
                                  bool* taken, rmw_subscription_allocation_t* allocation) {
    (void)subscription; (void)loaned_message; (void)taken; (void)allocation;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: no middleware-owned message memory (see file comment).
rmw_ret_t rmw_take_loaned_message_with_info(const rmw_subscription_t* subscription,
                                            void** loaned_message, bool* taken,
                                            rmw_message_info_t* message_info,
                                            rmw_subscription_allocation_t* allocation) {
    (void)subscription; (void)loaned_message; (void)taken; (void)message_info; (void)allocation;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: no middleware-owned message memory (see file comment).
rmw_ret_t rmw_return_loaned_message_from_subscription(const rmw_subscription_t* subscription,
                                                      void* loaned_message) {
    (void)subscription; (void)loaned_message;
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
