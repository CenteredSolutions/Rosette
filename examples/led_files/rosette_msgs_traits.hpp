/**
 * @file rosette_msgs_traits.hpp
 * @brief `rosette::service_traits` for this example's `rosette_msgs` package.
 *
 * Binding generated ROS types to the `rosette::` wrappers is a thin, per-app
 * concern, so it lives with the example rather than in the framework. It needs
 * `libmicroros` built with `examples/led_files/rosette_msgs/` — see the example
 * CMakeLists, which adds that directory to `ROSETTE_MICRO_ROS_INTERFACES_DIRS`.
 *
 * The `_Request` / `_Response` structs hold `rosidl_runtime_c__String` fields,
 * which own heap storage, so each trait provides `init_*` / `fini_*` hooks the
 * executor-owned message instances need.
 */

#pragma once

#include "rosette/ros/common.hpp"

#include <rosette_msgs/srv/set_led.h>
#include <rosette_msgs/srv/put_file.h>
#include <rosette_msgs/srv/get_file.h>
#include <rosette_msgs/srv/list_files.h>
#include <rosette_msgs/srv/erase_file.h>

#include <rosette_msgs/srv/detail/set_led__functions.h>
#include <rosette_msgs/srv/detail/put_file__functions.h>
#include <rosette_msgs/srv/detail/get_file__functions.h>
#include <rosette_msgs/srv/detail/list_files__functions.h>
#include <rosette_msgs/srv/detail/erase_file__functions.h>

#include <rosidl_runtime_c/service_type_support_struct.h>

namespace rosette {

#define ROSETTE_DEFINE_SRV_TRAITS(Tag, pkg, ns, Name)                            \
    template<>                                                                  \
    struct service_traits<Tag> {                                                \
        using Request = pkg##__##ns##__##Name##_Request;                        \
        using Response = pkg##__##ns##__##Name##_Response;                      \
        static const rosidl_service_type_support_t* rosidl_type_support() noexcept \
        {                                                                      \
            return ROSIDL_GET_SRV_TYPE_SUPPORT(pkg, ns, Name);                  \
        }                                                                      \
        static void init_request(Request& r) noexcept                           \
        {                                                                      \
            (void)pkg##__##ns##__##Name##_Request__init(&r);                    \
        }                                                                      \
        static void fini_request(Request& r) noexcept                           \
        {                                                                      \
            pkg##__##ns##__##Name##_Request__fini(&r);                          \
        }                                                                      \
        static void init_response(Response& r) noexcept                         \
        {                                                                      \
            (void)pkg##__##ns##__##Name##_Response__init(&r);                   \
        }                                                                      \
        static void fini_response(Response& r) noexcept                         \
        {                                                                      \
            pkg##__##ns##__##Name##_Response__fini(&r);                         \
        }                                                                      \
    }

/// Empty tag types name each service for the wrappers.
struct SetLedSrv {};
struct PutFileSrv {};
struct GetFileSrv {};
struct ListFilesSrv {};
struct EraseFileSrv {};

ROSETTE_DEFINE_SRV_TRAITS(SetLedSrv, rosette_msgs, srv, SetLed);
ROSETTE_DEFINE_SRV_TRAITS(PutFileSrv, rosette_msgs, srv, PutFile);
ROSETTE_DEFINE_SRV_TRAITS(GetFileSrv, rosette_msgs, srv, GetFile);
ROSETTE_DEFINE_SRV_TRAITS(ListFilesSrv, rosette_msgs, srv, ListFiles);
ROSETTE_DEFINE_SRV_TRAITS(EraseFileSrv, rosette_msgs, srv, EraseFile);

#undef ROSETTE_DEFINE_SRV_TRAITS

}  // namespace rosette
