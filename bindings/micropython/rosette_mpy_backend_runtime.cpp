/**
 * @file bindings/micropython/rosette_mpy_backend_runtime.cpp
 * @brief Runtime bridge from MicroPython objects into Rosette's micro-ROS backend.
 *
 * This translation unit owns the C++ runtime objects that live behind the
 * MicroPython-facing `Context` and `Node` wrappers. The binding keeps those
 * runtime objects out of the object-layout header so the Python-facing code
 * can stay small and so the build can fall back to a stub backend when a real
 * `libmicroros.a` is not available.
 *
 * The host unix-port smoke path uses static-agent UDP first because that is
 * the shortest route to prove the binding can start a real XRCE session. The
 * same object model also supports Rosette's custom transport path when the
 * generated micro-ROS library was built with custom-transport support.
 */

#include "rosette_mpy_backend.hpp"

extern "C" {
#include "py/nlr.h"
#include "py/objlist.h"
#include "py/objstr.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/gc.h"
#include "py/misc.h"
}

#if ROSETTE_MICROPY_HAS_MICRO_ROS
#include "rosette/ros/micro_ros/context.hpp"
#include "rosette/ros/micro_ros/custom_transport.hpp"
#include "rosette/ros/micro_ros/node.hpp"

#include <rcl/publisher.h>
#include <rcl/types.h>
#include <rclc/publisher.h>
// newlib (arm-none-eabi) does not reliably inject C string/stdio symbols into
// std:: via <cstring>/<cstdio>.  Include the plain C headers so that
// unqualified and std::-qualified names both resolve.
#include <new>
#include <string.h>
#include <stdio.h>
#include <rosidl_typesupport_introspection_c/identifier.h>
#include <rosidl_typesupport_introspection_c/message_introspection.h>
#include <std_msgs/msg/float32.h>

#endif

namespace rosette::bindings::micropython {

namespace {

void CopyStartDetail(mp_rosette_context_obj_t* self, const char* detail) noexcept
{
    if (self == nullptr) {
        return;
    }
    if (detail == nullptr) {
        self->backend_start_detail[0] = '\0';
        return;
    }
    snprintf(self->backend_start_detail,
             sizeof(self->backend_start_detail),
             "%s",
             detail);
}

void AppendStartDetail(mp_rosette_context_obj_t* self, const char* suffix) noexcept
{
    if (self == nullptr || suffix == nullptr) {
        return;
    }

    size_t used = strlen(self->backend_start_detail);
    if (used >= sizeof(self->backend_start_detail) - 1u) {
        return;
    }

    const size_t remaining = sizeof(self->backend_start_detail) - used;
    size_t write = 0u;
    while (suffix[write] != '\0' && write + 1u < remaining) {
        self->backend_start_detail[used + write] = suffix[write];
        ++write;
    }
    self->backend_start_detail[used + write] = '\0';
}

#if ROSETTE_MICROPY_HAS_MICRO_ROS

// ---------------------------------------------------------------------------
// MicroPython GC-backed rcl allocator
//
// rcl_get_default_allocator() returns malloc/free which is unavailable or
// maps to a tiny heap on MicroPython's RP2040 build. Route all micro-ROS
// internal allocations through the MicroPython GC instead.
// ---------------------------------------------------------------------------

static void* mpy_rcl_allocate(size_t size, void* /*state*/) {
    return gc_alloc(size, 0);
}
static void mpy_rcl_deallocate(void* ptr, void* /*state*/) {
    gc_free(ptr);
}
static void* mpy_rcl_reallocate(void* ptr, size_t size, void* /*state*/) {
    return gc_realloc(ptr, size, true);
}
static void* mpy_rcl_zero_allocate(size_t count, size_t size, void* /*state*/) {
    void* p = gc_alloc(count * size, 0);
    if (p) memset(p, 0, count * size);
    return p;
}

static rcl_allocator_t make_mpy_allocator() {
    rcl_allocator_t a{};
    a.allocate      = mpy_rcl_allocate;
    a.deallocate    = mpy_rcl_deallocate;
    a.reallocate    = mpy_rcl_reallocate;
    a.zero_allocate = mpy_rcl_zero_allocate;
    a.state         = nullptr;
    return a;
}

void CaptureStartDetail(mp_rosette_context_obj_t* self,
                        const milkros::micro_ros::Context& context) noexcept
{
    if (self == nullptr) {
        return;
    }

    const char* summary =
        milkros::micro_ros::context_start_error_string(context.last_start_error());
    const char* detail = context.last_start_detail();

    if (detail != nullptr && detail[0] != '\0') {
        CopyStartDetail(self, summary);
        AppendStartDetail(self, " (");
        AppendStartDetail(self, detail);
        AppendStartDetail(self, ")");
        return;
    }

    CopyStartDetail(self, summary);
}

/**
 * @brief Heap-owned micro-ROS context runtime stored behind `Context.backend`.
 *
 * The MicroPython object stores opaque `void*` backend handles so its layout
 * remains stable across stub and real-backend builds. This runtime object owns
 * the durable strings referenced by `milkros::micro_ros::ContextOptions` and
 * the `milkros::micro_ros::Context` itself.
 */
struct MicroPythonContextBackend {
    char agent_address[40];  // IPv4, IPv6, or short hostname
    milkros::micro_ros::ContextOptions options;
    milkros::micro_ros::Context context;

    explicit MicroPythonContextBackend(mp_rosette_context_obj_t* owner)
    {
        const char* ip = owner->agent_ip_obj == mp_const_none
            ? ""
            : mp_obj_str_get_str(owner->agent_ip_obj);
        strncpy(agent_address, ip, sizeof(agent_address) - 1);
        agent_address[sizeof(agent_address) - 1] = '\0';
        options = BuildOptions(owner, agent_address);
        new (&context) milkros::micro_ros::Context(options);
    }

    /**
     * @brief Builds Rosette micro-ROS context options from one binding context.
     *
     * The binding keeps Python-facing constructor knobs intentionally small, so
     * this function is the one place that translates those knobs into the
     * richer `milkros::micro_ros::ContextOptions` structure. Host smoke builds
     * default to stock UDP with a static agent endpoint, while embedded builds
     * can select the custom transport path through a compile-time switch.
     *
     * @param[in] owner MicroPython context that owns configuration state.
     * @param[in] agent_address Null-terminated agent IP address string.
     * @param[in] agent_port Null-terminated agent UDP port string.
     * @return Populated Rosette context options ready for `Context` startup.
     *
     * References:
     * - micro-ROS RMW transport configuration: `rmw_uros_options_set_udp_address()`.
     * - micro-ROS custom transport configuration: `rmw_uros_options_set_custom_transport()`.
     */
    static milkros::micro_ros::ContextOptions BuildOptions(
        mp_rosette_context_obj_t* owner,
        const char* agent_address)
    {
        milkros::micro_ros::ContextOptions options{};
        options.allocator = make_mpy_allocator();
        options.domain_id = owner->domain_id;
        options.client_key = owner->client_key;
        options.executor_handles = owner->executor_handles;
        options.spin_timeout_ns = static_cast<uint64_t>(owner->spin_timeout_us) * 1000ull;
        options.auto_discover_agent = false;
        options.udp_agent_address = agent_address;
        options.udp_agent_port = owner->agent_port;
#if ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM
        options.use_custom_transport = true;
        options.custom_transport_framing = false;  // UDP datagrams have natural boundaries
        {
            mp_obj_t transport_native = mp_obj_cast_to_native_base(
                owner->transport_obj, MP_OBJ_FROM_PTR(&rosette_transport_type));
            if (transport_native == MP_OBJ_NULL) {
                return options;
            }
            auto* transport = static_cast<mp_rosette_transport_obj_t*>(
                MP_OBJ_TO_PTR(transport_native));
            options.custom_transport =
                static_cast<milkros::micro_ros::CustomTransport*>(
                    transport->implementation);
        }
#else
        options.use_custom_transport = false;
        options.custom_transport = nullptr;
#endif
        return options;
    }
};

/**
 * @brief Heap-owned micro-ROS node runtime stored behind `Node.backend`.
 *
 * Node wrappers in MicroPython own Python objects for name and namespace while
 * the actual ROS node handle lives in C++. This runtime object owns the
 * `milkros::micro_ros::Node` so node shutdown cleanly mirrors the C++ backend
 * lifetime rules.
 */
struct MicroPythonNodeBackend {
    milkros::micro_ros::Node node;

    explicit MicroPythonNodeBackend(milkros::micro_ros::Node&& node_in) noexcept
        : node(static_cast<milkros::micro_ros::Node&&>(node_in))
    {}
};

/**
 * @brief Heap-owned micro-ROS publisher runtime stored behind `Publisher.backend`.
 *
 * The MicroPython publisher keeps Python references to the schema, topic, and
 * parent node. This runtime owns the native `rcl_publisher_t` created from
 * those references and publishes generated C message instances.
 */
struct MicroPythonPublisherBackend {
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    rcl_node_t* node = nullptr;
    uint8_t message_kind = ROSETTE_MPY_MESSAGE_UNSUPPORTED;
    bool valid = false;

    ~MicroPythonPublisherBackend()
    {
        fini();
    }

    void fini() noexcept
    {
        if (valid) {
            [[maybe_unused]] const rcl_ret_t fini_ret =
                rcl_publisher_fini(&publisher, node);
        }
        publisher = rcl_get_zero_initialized_publisher();
        node = nullptr;
        valid = false;
    }
};

/**
 * @brief Heap-owned publisher backend for schemas parsed from `.msg` text.
 *
 * Dynamic schemas have no statically-linked ROSIDL generated type. This backend
 * constructs a minimal rosidl_typesupport_introspection type support at runtime
 * so that `rclc_publisher_init` can register the topic with the correct type
 * name string.  The actual serialization is handled by the generic CDR encoder
 * in rosette_mpy_schema.cpp, and the payload is sent via
 * `rcl_publish_serialized_message` so the RMW never tries to serialize a
 * generated C struct.
 *
 * The type support objects (MessageMembers, MessageMember array, and the
 * rosidl_message_type_support_t wrapper) are owned by this struct and must
 * outlive the `rcl_publisher_t`.
 */
struct MicroPythonDynamicPublisherBackend {
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    rcl_node_t* node = nullptr;
    bool valid = false;

    // Runtime-allocated type support structures.
    rosidl_typesupport_introspection_c__MessageMembers members{};
    rosidl_typesupport_introspection_c__MessageMember  member_placeholder{};
    rosidl_message_type_support_t type_support{};

    // Fixed-size buffers for the type name strings (members_ fields point here).
    // Sized to hold the longest plausible ROS package/message name plus "__msg".
    static constexpr size_t kNameBufSize = 64u;
    char namespace_str[kNameBufSize];
    char name_str[kNameBufSize];

    // Scratch encode buffer.  Resized on first use; stays allocated for the
    // lifetime of the publisher to avoid per-publish heap churn.
    static constexpr size_t kDefaultBufSize = 256u;
    uint8_t* encode_buf = nullptr;
    size_t   encode_buf_size = 0u;

    ~MicroPythonDynamicPublisherBackend()
    {
        fini();
        if (encode_buf != nullptr) {
            m_del(uint8_t, encode_buf, encode_buf_size);
        }
        encode_buf = nullptr;
        encode_buf_size = 0u;
    }

    void fini() noexcept
    {
        if (valid) {
            [[maybe_unused]] const rcl_ret_t fini_ret =
                rcl_publisher_fini(&publisher, node);
        }
        publisher = rcl_get_zero_initialized_publisher();
        node = nullptr;
        valid = false;
    }

    /**
     * @brief Builds the runtime type support from a parsed schema's name parts.
     *
     * `rclc_publisher_init` uses the type support to register the DDS topic
     * with the correct ROS type name (e.g. "my_pkg/msg/MyMsg").  The member
     * array is never used for serialization because we call
     * `rcl_publish_serialized_message` instead of `rcl_publish`.
     *
     * @param[in] schema Schema whose full_name drives the type name strings.
     */
    void build_type_support(const mp_rosette_schema_obj_t* schema)
    {
        // ROS 2 type support namespace convention: "pkg__msg" (double underscore).
        // When no package component exists, use an empty namespace.
        const char* full_name = qstr_str(schema->full_name);
        const char* slash = strchr(full_name, '/');
        if (slash != nullptr) {
            const size_t pkg_len = static_cast<size_t>(slash - full_name);
            const size_t copy_len = pkg_len < (kNameBufSize - 6u)
                                    ? pkg_len : (kNameBufSize - 6u);
            memcpy(namespace_str, full_name, copy_len);
            memcpy(namespace_str + copy_len, "__msg", 6u);  // includes NUL
            strncpy(name_str, slash + 1, kNameBufSize - 1u);
            name_str[kNameBufSize - 1u] = '\0';
        } else {
            strncpy(namespace_str, "unknown__msg", kNameBufSize - 1u);
            namespace_str[kNameBufSize - 1u] = '\0';
            const char* src = (full_name[0] != '\0') ? full_name : "Unknown";
            strncpy(name_str, src, kNameBufSize - 1u);
            name_str[kNameBufSize - 1u] = '\0';
        }

        // A placeholder member is required by some RMW implementations that
        // inspect member_count_ before accepting the type support. We provide
        // one minimal entry so the publisher is accepted.
        member_placeholder = {};
        member_placeholder.name_    = name_str;
        member_placeholder.type_id_ = 0u;  // FIELD_TYPE_NOT_SET / placeholder

        members = {};
        members.message_namespace_ = namespace_str;
        members.message_name_      = name_str;
        members.member_count_      = 1u;
        members.size_of_           = 0u;
        members.members_           = &member_placeholder;
        members.init_function      = nullptr;
        members.fini_function      = nullptr;

        type_support = {};
        type_support.typesupport_identifier = rosidl_typesupport_introspection_c__identifier;
        type_support.data = &members;
        // Identity handle function: return self when identifier matches.
        type_support.func = [](const rosidl_message_type_support_t* handle,
                               const char* id) -> const rosidl_message_type_support_t* {
            if (strcmp(id, rosidl_typesupport_introspection_c__identifier) == 0) {
                return handle;
            }
            return nullptr;
        };
    }

    /**
     * @brief Ensures the encode scratch buffer is at least `needed` bytes.
     *
     * @param[in] needed Required buffer size.
     * @return true when the buffer is ready; false on allocation failure.
     */
    bool ensure_buf(size_t needed)
    {
        if (encode_buf != nullptr && encode_buf_size >= needed) return true;
        if (encode_buf != nullptr) {
            m_del(uint8_t, encode_buf, encode_buf_size);
        }
        encode_buf = nullptr;
        encode_buf_size = 0u;
        const size_t alloc = (needed < kDefaultBufSize) ? kDefaultBufSize : needed;
        encode_buf = m_new_maybe(uint8_t, alloc);
        if (encode_buf == nullptr) return false;
        encode_buf_size = alloc;
        return true;
    }
};

/**
 * @brief Heap-owned subscription backend that receives serialized samples.
 *
 * MicroPython subscriptions use runtime schemas, so the receive path keeps the
 * RCL entity native but takes messages as CDR bytes. The context pump drains
 * this subscription after each executor spin and either calls the Python
 * callback or stores the frame in the subscription ring.
 *
 * The serialized buffer is allocated with the RCL allocator so
 * `rcl_take_serialized_message()` can grow it safely if the middleware needs
 * more room than the subscription's initial payload budget.
 */
struct MicroPythonSubscriptionBackend {
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    rcl_node_t* node = nullptr;
    rcl_serialized_message_t serialized{};
    bool valid = false;
    bool dynamic_type_support = false;

    rosidl_typesupport_introspection_c__MessageMembers members{};
    rosidl_typesupport_introspection_c__MessageMember member_placeholder{};
    rosidl_message_type_support_t type_support{};
    // Fixed-size buffers for the type name strings (members_ fields point here).
    static constexpr size_t kNameBufSize = 64u;
    char namespace_str[kNameBufSize];
    char name_str[kNameBufSize];

    ~MicroPythonSubscriptionBackend()
    {
        fini();
        release_serialized_buffer();
    }

    /**
     * @brief Builds minimal runtime type support for a dynamic schema.
     *
     * The type support names the DDS topic type while serialized take keeps the
     * middleware from needing generated C message storage.
     *
     * @param[in] schema Runtime schema whose full name identifies the ROS type.
     */
    void build_type_support(const mp_rosette_schema_obj_t* schema)
    {
        const char* full_name = qstr_str(schema->full_name);
        const char* slash = strchr(full_name, '/');
        if (slash != nullptr) {
            const size_t pkg_len = static_cast<size_t>(slash - full_name);
            const size_t copy_len = pkg_len < (kNameBufSize - 6u)
                                    ? pkg_len : (kNameBufSize - 6u);
            memcpy(namespace_str, full_name, copy_len);
            memcpy(namespace_str + copy_len, "__msg", 6u);  // includes NUL
            strncpy(name_str, slash + 1, kNameBufSize - 1u);
            name_str[kNameBufSize - 1u] = '\0';
        } else {
            strncpy(namespace_str, "unknown__msg", kNameBufSize - 1u);
            namespace_str[kNameBufSize - 1u] = '\0';
            const char* src = (full_name[0] != '\0') ? full_name : "Unknown";
            strncpy(name_str, src, kNameBufSize - 1u);
            name_str[kNameBufSize - 1u] = '\0';
        }

        member_placeholder = {};
        member_placeholder.name_ = name_str;
        member_placeholder.type_id_ = 0u;

        members = {};
        members.message_namespace_ = namespace_str;
        members.message_name_ = name_str;
        members.member_count_ = 1u;
        members.size_of_ = 0u;
        members.members_ = &member_placeholder;
        members.init_function = nullptr;
        members.fini_function = nullptr;

        type_support = {};
        type_support.typesupport_identifier = rosidl_typesupport_introspection_c__identifier;
        type_support.data = &members;
        type_support.func = [](const rosidl_message_type_support_t* handle,
                               const char* id) -> const rosidl_message_type_support_t* {
            return strcmp(id, rosidl_typesupport_introspection_c__identifier) == 0
                ? handle
                : nullptr;
        };
        dynamic_type_support = true;
    }

    /**
     * @brief Allocates the reusable serialized receive buffer.
     *
     * @param[in] capacity Number of bytes to reserve for incoming CDR payloads.
     * @return true when the buffer is ready for `rcl_take_serialized_message()`.
     */
    bool init_serialized_buffer(size_t capacity) noexcept
    {
        serialized = {};
        serialized.allocator = rcl_get_default_allocator();
        serialized.buffer = static_cast<uint8_t*>(
            serialized.allocator.allocate(capacity, serialized.allocator.state));
        if (serialized.buffer == nullptr) {
            serialized.buffer_capacity = 0u;
            return false;
        }
        serialized.buffer_length = 0u;
        serialized.buffer_capacity = capacity;
        return true;
    }

    void release_serialized_buffer() noexcept
    {
        if (serialized.buffer != nullptr) {
            serialized.allocator.deallocate(
                serialized.buffer, serialized.allocator.state);
        }
        serialized.buffer = nullptr;
        serialized.buffer_length = 0u;
        serialized.buffer_capacity = 0u;
    }

    void fini() noexcept
    {
        if (valid) {
            rcl_ret_t _fini_rc = rcl_subscription_fini(&subscription, node);
            (void)_fini_rc;
        }
        subscription = rcl_get_zero_initialized_subscription();
        node = nullptr;
        valid = false;
    }
};

/**
 * @brief Returns the typed micro-ROS context backend for one binding context.
 *
 * @param[in] self MicroPython context object whose backend should be decoded.
 * @return Typed backend pointer, or `nullptr` when startup has not completed.
 */
MicroPythonContextBackend* GetContextBackend(mp_rosette_context_obj_t* self) noexcept
{
    return self == nullptr
           ? nullptr
           : static_cast<MicroPythonContextBackend*>(self->backend);
}

/**
 * @brief Returns the typed micro-ROS node backend for one binding node.
 *
 * @param[in] self MicroPython node object whose backend should be decoded.
 * @return Typed backend pointer, or `nullptr` when node initialization failed.
 */
MicroPythonNodeBackend* GetNodeBackend(mp_rosette_node_obj_t* self) noexcept
{
    return self == nullptr ? nullptr : static_cast<MicroPythonNodeBackend*>(self->backend);
}

/**
 * @brief Tag stored in the low bit of the backend pointer to distinguish
 *        static (generated type) from dynamic (runtime schema) publishers.
 *
 * Both backend structs are heap-allocated with `new`, so their natural
 * alignment is at least 2 bytes.  We steal the low bit as a tag and mask it
 * off before casting.
 */
static constexpr uintptr_t kDynamicPublisherTag = 1u;

/**
 * @brief Returns the static publisher backend pointer, or nullptr if the
 *        backend is tagged as dynamic.
 */
MicroPythonPublisherBackend* GetPublisherBackend(mp_rosette_publisher_obj_t* self) noexcept
{
    if (self == nullptr || self->backend == nullptr) return nullptr;
    const uintptr_t raw = reinterpret_cast<uintptr_t>(self->backend);
    if (raw & kDynamicPublisherTag) return nullptr;
    return reinterpret_cast<MicroPythonPublisherBackend*>(raw);
}

/**
 * @brief Returns the dynamic publisher backend pointer, or nullptr if the
 *        backend is tagged as static.
 */
MicroPythonDynamicPublisherBackend*
GetDynamicPublisherBackend(mp_rosette_publisher_obj_t* self) noexcept
{
    if (self == nullptr || self->backend == nullptr) return nullptr;
    const uintptr_t raw = reinterpret_cast<uintptr_t>(self->backend);
    if (!(raw & kDynamicPublisherTag)) return nullptr;
    return reinterpret_cast<MicroPythonDynamicPublisherBackend*>(raw & ~kDynamicPublisherTag);
}

/**
 * @brief Stores a dynamic backend pointer with the tag bit set.
 */
[[maybe_unused]] void SetDynamicPublisherBackend(mp_rosette_publisher_obj_t* self,
                                MicroPythonDynamicPublisherBackend* backend) noexcept
{
    const uintptr_t raw = reinterpret_cast<uintptr_t>(backend);
    self->backend = reinterpret_cast<void*>(raw | kDynamicPublisherTag);
}

MicroPythonSubscriptionBackend*
GetSubscriptionBackend(mp_rosette_subscription_obj_t* self) noexcept
{
    return self == nullptr
        ? nullptr
        : static_cast<MicroPythonSubscriptionBackend*>(self->backend);
}

const rosidl_message_type_support_t*
StaticMessageTypeSupport(const mp_rosette_schema_obj_t* schema) noexcept
{
    if (schema == nullptr) {
        return nullptr;
    }
    if (schema->message_kind == ROSETTE_MPY_MESSAGE_STD_MSGS_FLOAT32) {
        return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
    }
    return nullptr;
}

void PrintCallbackException(mp_obj_t exception) noexcept
{
    mp_printf(&mp_plat_print, "rosette subscription callback error:\n");
    mp_obj_print_exception(&mp_plat_print, exception);
}

/**
 * @brief Extracts the numeric `data` field for `std_msgs/Float32` publishing.
 *
 * The first publisher slice accepts a bare Python number so async examples can
 * stay compact. A one-field dictionary is also accepted to match the shape
 * `recv()` will eventually return for structured messages.
 *
 * @param[in] payload Python float/int value or dictionary containing `data`.
 * @return Floating-point value to write into `std_msgs__msg__Float32::data`.
 */
float ExtractFloat32Payload(mp_obj_t payload)
{
    if (mp_obj_is_dict_or_ordereddict(payload)) {
        mp_map_t* map = mp_obj_dict_get_map(payload);
        mp_map_elem_t* elem = mp_map_lookup(
            map,
            MP_OBJ_NEW_QSTR(MP_QSTR_data),
            MP_MAP_LOOKUP);
        if (elem == nullptr) {
            mp_raise_type_arg(&mp_type_KeyError, MP_OBJ_NEW_QSTR(MP_QSTR_data));
        }
        payload = elem->value;
    }
    return static_cast<float>(mp_obj_get_float(payload));
}

#endif  // ROSETTE_MICROPY_HAS_MICRO_ROS

}  // namespace

/**
 * @brief Starts the Rosette backend for one MicroPython context.
 *
 * Real micro-ROS builds allocate a typed runtime object, translate the
 * Python-facing configuration into `milkros::micro_ros::ContextOptions`, and
 * start the underlying XRCE session. Stub builds keep the same call shape so
 * the MicroPython module remains compileable without a generated micro-ROS
 * library.
 *
 * @param[in,out] self MicroPython context object that owns the socket and
 * transport configuration.
 * @return `true` when the backend is running and ready to create nodes.
 */
bool ContextStart(mp_rosette_context_obj_t* self) noexcept
{
    if (self == nullptr || self->closed || self->transport_obj == mp_const_none) {
        return false;
    }
    CopyStartDetail(self, "");
    {
        mp_obj_t transport_native = mp_obj_cast_to_native_base(
            self->transport_obj, MP_OBJ_FROM_PTR(&rosette_transport_type));
        if (transport_native == MP_OBJ_NULL) {
            return false;
        }
        auto* transport = static_cast<mp_rosette_transport_obj_t*>(
            MP_OBJ_TO_PTR(transport_native));
        if (!transport->socket_ready) {
            return false;
        }
    }

#if ROSETTE_MICROPY_HAS_MICRO_ROS
    if (self->backend != nullptr) {
        return true;
    }

    auto* allocd = m_new_maybe(uint8_t, sizeof(MicroPythonContextBackend));
    if (allocd == nullptr) {
        mp_raise_type(&mp_type_MemoryError);
    }
    auto* backend = new (allocd) MicroPythonContextBackend(self);
    if (backend->options.use_custom_transport && backend->options.custom_transport == nullptr) {
        self->backend_start_error = static_cast<uint8_t>(
            milkros::micro_ros::ContextStartError::kCustomTransportMissing);
        CopyStartDetail(
            self,
            milkros::micro_ros::context_start_error_string(
                milkros::micro_ros::ContextStartError::kCustomTransportMissing));
        backend->~MicroPythonContextBackend();
        m_del(uint8_t, allocd, sizeof(MicroPythonContextBackend));
        return false;
    }
    const auto result = backend->context.start_result();
    self->backend_start_error =
        static_cast<uint8_t>(backend->context.last_start_error());
    CaptureStartDetail(self, backend->context);
    if (!result.ok()) {
        backend->~MicroPythonContextBackend();
        m_del(uint8_t, allocd, sizeof(MicroPythonContextBackend));
        return false;
    }
    self->backend = backend;
    return true;
#else
    self->backend_start_error = 0u;
    CopyStartDetail(self, "ok");
    self->backend = self;
    return true;
#endif
}

/**
 * @brief Returns diagnostic text for the last backend startup attempt.
 *
 * The MicroPython exception layer calls this after `ContextStart()` fails so
 * users see the backend phase that failed. The status byte lives on the
 * Python-facing context object because a failed startup deletes the transient
 * C++ backend before returning.
 *
 * @param[in] self MicroPython context object to inspect.
 * @return Static diagnostic text for the latest startup status.
 */
const char* ContextStartErrorMessage(const mp_rosette_context_obj_t* self) noexcept
{
#if ROSETTE_MICROPY_HAS_MICRO_ROS
    if (self == nullptr) {
        return "context object is null";
    }
    if (self->backend_start_detail[0] != '\0') {
        return self->backend_start_detail;
    }
    return milkros::micro_ros::context_start_error_string(
        static_cast<milkros::micro_ros::ContextStartError>(
            self->backend_start_error));
#else
    (void)self;
    return "ok";
#endif
}

/**
 * @brief Finalizes the Rosette backend owned by one MicroPython context.
 *
 * @param[in,out] self MicroPython context object whose backend should be
 * released.
 */
void ContextFini(mp_rosette_context_obj_t* self) noexcept
{
    if (self == nullptr) {
        return;
    }

#if ROSETTE_MICROPY_HAS_MICRO_ROS
    auto* backend = GetContextBackend(self);
    if (backend != nullptr) {
        backend->~MicroPythonContextBackend();
        m_del(uint8_t, backend, sizeof(MicroPythonContextBackend));
    }
#endif
    self->backend = nullptr;
}

/**
 * @brief Advances the Rosette backend owned by one MicroPython context.
 *
 * Real micro-ROS builds delegate to the shared `rclc_executor_t` tick path.
 * Stub builds preserve the same boolean semantics for smoke testing.
 *
 * @param[in,out] self MicroPython context object whose backend should advance.
 * @return `true` when the backend tick completed successfully.
 */
bool ContextTick(mp_rosette_context_obj_t* self) noexcept
{
#if ROSETTE_MICROPY_HAS_MICRO_ROS
    if (self == nullptr) {
        return false;
    }
    auto* backend = GetContextBackend(self);
    if (backend == nullptr || !backend->context.tick()) {
        return false;
    }
    size_t count = 0u;
    mp_obj_t* items = nullptr;
    mp_obj_get_array(self->subscriptions, &count, &items);
    for (size_t i = 0u; i < count; ++i) {
        auto* sub = static_cast<mp_rosette_subscription_obj_t*>(
            MP_OBJ_TO_PTR(items[i]));
        if (!SubscriptionDispatchPending(sub)) {
            return false;
        }
    }
    return true;
#else
    return self != nullptr && self->backend != nullptr && self->started && !self->closed;
#endif
}

/**
 * @brief Creates the backend node owned by one MicroPython `Node` object.
 *
 * @param[in,out] self MicroPython node object to initialize.
 * @return `true` when the node backend was created successfully.
 */
bool NodeInit(mp_rosette_node_obj_t* self) noexcept
{
    if (self == nullptr) {
        return false;
    }

#if ROSETTE_MICROPY_HAS_MICRO_ROS
    auto* context =
        static_cast<mp_rosette_context_obj_t*>(MP_OBJ_TO_PTR(self->context_obj));
    auto* context_backend = GetContextBackend(context);
    if (context_backend == nullptr) {
        return false;
    }

    const char* name = mp_obj_str_get_str(self->name_obj);
    const char* node_namespace = mp_obj_str_get_str(self->namespace_obj);
    auto node = context_backend->context.create_node(name, node_namespace);
    if (!node.valid()) {
        return false;
    }

    auto* allocd = m_new_maybe(uint8_t, sizeof(MicroPythonNodeBackend));
    if (allocd == nullptr) {
        mp_raise_type(&mp_type_MemoryError);
    }
    self->backend = new (allocd) MicroPythonNodeBackend(static_cast<milkros::micro_ros::Node&&>(node));
    return true;
#else
    self->backend = self;
    return true;
#endif
}

/**
 * @brief Finalizes the backend node owned by one MicroPython `Node` object.
 *
 * @param[in,out] self MicroPython node object whose backend should be
 * released.
 */
void NodeFini(mp_rosette_node_obj_t* self) noexcept
{
    if (self == nullptr) {
        return;
    }

#if ROSETTE_MICROPY_HAS_MICRO_ROS
    auto* backend = GetNodeBackend(self);
    if (backend != nullptr) {
        backend->~MicroPythonNodeBackend();
        m_del(uint8_t, backend, sizeof(MicroPythonNodeBackend));
    }
#endif
    self->backend = nullptr;
}

/**
 * @brief Creates the backend publisher owned by one MicroPython `Publisher`.
 *
 * The publisher factory validates that the schema maps to a generated ROSIDL C
 * message already linked into the micro-ROS library, then initializes an
 * `rcl_publisher_t` from the parent node. This keeps Python-facing schema
 * construction lightweight while refusing unsupported message types before
 * any publish loop starts.
 *
 * @param[in,out] self MicroPython publisher object to initialize.
 * @param[in] options QoS depth and reliability translated from Python.
 * @return `true` when the native publisher was created successfully.
 *
 * References:
 * - ROS 2 RCL publisher API: `rcl_publish()`.
 * - ROSIDL C type support lookup: `ROSIDL_GET_MSG_TYPE_SUPPORT()`.
 */
bool PublisherInit(mp_rosette_publisher_obj_t* self,
                   const EndpointOptions& options) noexcept
{
    if (self == nullptr) {
        return false;
    }
    self->depth    = options.depth;
    self->reliable = options.reliable;

#if ROSETTE_MICROPY_HAS_MICRO_ROS
    auto* node_obj =
        static_cast<mp_rosette_node_obj_t*>(MP_OBJ_TO_PTR(self->node_obj));
    auto* schema =
        static_cast<mp_rosette_schema_obj_t*>(MP_OBJ_TO_PTR(self->schema_obj));
    auto* node_backend = GetNodeBackend(node_obj);
    if (node_backend == nullptr || schema == nullptr) {
        return false;
    }

    milkros::micro_ros::PublisherOptions publisher_options{};
    publisher_options.depth = options.depth;
    publisher_options.reliability = options.reliable
                                    ? milkros::micro_ros::Reliability::RELIABLE
                                    : milkros::micro_ros::Reliability::BEST_EFFORT;
    const rmw_qos_profile_t qos =
        milkros::micro_ros::make_qos_profile(publisher_options);
    const char* topic_name = mp_obj_str_get_str(self->topic_obj);
    rcl_node_t* node_handle = node_backend->node.native_handle();
    if (node_handle == nullptr) {
        return false;
    }

    // ----------------------------------------------------------------
    // Dynamic path: schema was parsed from .msg text at runtime.
    // ----------------------------------------------------------------
    if (schema->is_dynamic) {
        auto* allocd = m_new_maybe(uint8_t, sizeof(MicroPythonDynamicPublisherBackend));
        if (allocd == nullptr) {
            return false;
        }
        auto* backend = new (allocd) MicroPythonDynamicPublisherBackend();

        backend->node = node_handle;
        backend->build_type_support(schema);

        if (rclc_publisher_init(
                &backend->publisher,
                backend->node,
                &backend->type_support,
                topic_name,
                &qos) != RCL_RET_OK) {
            backend->~MicroPythonDynamicPublisherBackend();
            m_del(uint8_t, allocd, sizeof(MicroPythonDynamicPublisherBackend));
            return false;
        }
        backend->valid = true;
        SetDynamicPublisherBackend(self, backend);
        return true;
    }

    // ----------------------------------------------------------------
    // Static path: schema maps to a statically-linked ROSIDL type.
    // ----------------------------------------------------------------
    if (schema->message_kind == ROSETTE_MPY_MESSAGE_UNSUPPORTED) {
        return false;
    }

    auto* allocd = m_new_maybe(uint8_t, sizeof(MicroPythonPublisherBackend));
    if (allocd == nullptr) {
        return false;
    }
    auto* backend = new (allocd) MicroPythonPublisherBackend();

    backend->node         = node_handle;
    backend->message_kind = schema->message_kind;

    const rosidl_message_type_support_t* type_support = nullptr;
    if (schema->message_kind == ROSETTE_MPY_MESSAGE_STD_MSGS_FLOAT32) {
        type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
    }
    if (type_support == nullptr ||
        rclc_publisher_init(
            &backend->publisher,
            backend->node,
            type_support,
            topic_name,
            &qos) != RCL_RET_OK) {
        backend->~MicroPythonPublisherBackend();
        m_del(uint8_t, allocd, sizeof(MicroPythonPublisherBackend));
        return false;
    }
    backend->valid = true;
    self->backend  = backend;
    return true;
#else
    // Only emit the hard error during a real compilation pass, not during the
    // MicroPython qstr-collection pass (which defines NO_QSTR and does not
    // pass target compile-definitions, so ROSETTE_MICROPY_HAS_MICRO_ROS
    // appears undefined/0 even when it will be 1 at compile time).
    #ifndef NO_QSTR
    #error "Micro-ROS backend is required to publish messages from MicroPython"
    #endif
    self->backend = self;
    return true;
#endif
}

/**
 * @brief Finalizes the backend publisher owned by one MicroPython object.
 *
 * @param[in,out] self Publisher object whose native handle should be released.
 */
void PublisherFini(mp_rosette_publisher_obj_t* self) noexcept
{
    if (self == nullptr) {
        return;
    }
#if ROSETTE_MICROPY_HAS_MICRO_ROS
    if (auto* dyn = GetDynamicPublisherBackend(self)) {
        dyn->~MicroPythonDynamicPublisherBackend();
        m_del(uint8_t, dyn, sizeof(MicroPythonDynamicPublisherBackend));
    } else if (auto* pub = GetPublisherBackend(self)) {
        pub->~MicroPythonPublisherBackend();
        m_del(uint8_t, pub, sizeof(MicroPythonPublisherBackend));
    }
#endif
    self->backend = nullptr;
}

/**
 * @brief Publishes one Python payload through a generated-message backend.
 *
 * The binding converts the Python object into the generated C message selected
 * by the publisher schema and then calls `rcl_publish()`. The first supported
 * schema is `std_msgs/Float32`, which accepts either a bare number or a
 * dictionary containing a numeric `data` field.
 *
 * @param[in,out] self Publisher object that owns the native publisher handle.
 * @param[in] payload Python payload to convert and publish.
 * @return `true` when the backend accepted the message.
 */
bool PublisherPublish(mp_rosette_publisher_obj_t* self,
                      mp_obj_t payload)
{
    if (self == nullptr || self->backend == nullptr) {
        return false;
    }

#if ROSETTE_MICROPY_HAS_MICRO_ROS

    // ----------------------------------------------------------------
    // Dynamic path: encode via generic CDR codec, send serialized.
    // ----------------------------------------------------------------
    if (auto* dyn = GetDynamicPublisherBackend(self)) {
        if (!dyn->valid) return false;

        auto* schema =
            static_cast<const mp_rosette_schema_obj_t*>(
                MP_OBJ_TO_PTR(self->schema_obj));

        // Estimate size: use max_serialized_size when set, else a generous
        // initial scratch buffer (the encoder will return 0 on overflow and
        // we retry with a larger buffer).
        size_t needed = schema->max_serialized_size > 0u
                        ? static_cast<size_t>(schema->max_serialized_size)
                        : MicroPythonDynamicPublisherBackend::kDefaultBufSize;

        for (int attempt = 0; attempt < 2; ++attempt) {
            if (!dyn->ensure_buf(needed)) return false;
            const size_t written =
                SchemaCdrEncode(schema, payload, dyn->encode_buf, dyn->encode_buf_size);
            if (written == 0u) {
                // Overflow — double the buffer and retry once.
                needed = dyn->encode_buf_size * 2u;
                continue;
            }

            rcl_serialized_message_t serialized{};
            serialized.buffer          = dyn->encode_buf;
            serialized.buffer_length   = written;
            serialized.buffer_capacity = dyn->encode_buf_size;
            serialized.allocator       = rcl_get_default_allocator();

            return rcl_publish_serialized_message(
                       &dyn->publisher, &serialized, nullptr) == RCL_RET_OK;
        }
        return false;
    }

    // ----------------------------------------------------------------
    // Static path: use generated C message structs.
    // ----------------------------------------------------------------
    auto* backend = GetPublisherBackend(self);
    if (backend == nullptr || !backend->valid) {
        return false;
    }
    if (backend->message_kind == ROSETTE_MPY_MESSAGE_STD_MSGS_FLOAT32) {
        std_msgs__msg__Float32 message{};
        message.data = ExtractFloat32Payload(payload);
        return rcl_publish(&backend->publisher, &message, nullptr) == RCL_RET_OK;
    }
    return false;
#else
    (void)payload;
    return true;
#endif
}

/**
 * @brief Creates the native subscription handle for a MicroPython subscription.
 *
 * The binding initializes an `rcl_subscription_t` directly instead of adding it
 * to the rclc executor because runtime schemas are consumed as serialized CDR
 * bytes. The context pump performs non-blocking serialized takes after each
 * executor spin and routes decoded messages into either the Python callback or
 * the manual receive queue.
 *
 * @param[in,out] self Subscription object that owns callback and queue state.
 * @param[in] options QoS settings selected by the Python factory.
 * @return true when the native subscription can receive messages.
 *
 * References:
 * - ROS 2 RCL subscription API: `rcl_subscription_init()`.
 * - ROS 2 RCL serialized take API: `rcl_take_serialized_message()`.
 */
bool SubscriptionBackendInit(mp_rosette_subscription_obj_t* self,
                             const EndpointOptions& options) noexcept
{
    if (self == nullptr) {
        return false;
    }

#if ROSETTE_MICROPY_HAS_MICRO_ROS
    auto* node_obj =
        static_cast<mp_rosette_node_obj_t*>(MP_OBJ_TO_PTR(self->node_obj));
    auto* schema =
        static_cast<mp_rosette_schema_obj_t*>(MP_OBJ_TO_PTR(self->schema_obj));
    auto* node_backend = GetNodeBackend(node_obj);
    if (node_backend == nullptr || schema == nullptr) {
        return false;
    }

    auto* allocd = m_new_maybe(uint8_t, sizeof(MicroPythonSubscriptionBackend));
    if (allocd == nullptr) {
        return false;
    }
    auto* backend = new (allocd) MicroPythonSubscriptionBackend();
    backend->node = node_backend->node.native_handle();
    if (backend->node == nullptr) {
        backend->~MicroPythonSubscriptionBackend();
        m_del(uint8_t, allocd, sizeof(MicroPythonSubscriptionBackend));
        return false;
    }

    const rosidl_message_type_support_t* type_support = nullptr;
    if (schema->is_dynamic) {
        backend->build_type_support(schema);
        type_support = &backend->type_support;
    } else {
        type_support = StaticMessageTypeSupport(schema);
    }
    if (type_support == nullptr) {
        backend->~MicroPythonSubscriptionBackend();
        m_del(uint8_t, allocd, sizeof(MicroPythonSubscriptionBackend));
        return false;
    }

    milkros::micro_ros::SubscriptionOptions subscription_options{};
    subscription_options.depth = options.depth;
    subscription_options.reliability = options.reliable
        ? milkros::micro_ros::Reliability::RELIABLE
        : milkros::micro_ros::Reliability::BEST_EFFORT;
    rcl_subscription_options_t rcl_options = rcl_subscription_get_default_options();
    rcl_options.qos = milkros::micro_ros::make_qos_profile(subscription_options);

    const char* topic_name = mp_obj_str_get_str(self->topic_obj);
    if (rcl_subscription_init(
            &backend->subscription,
            backend->node,
            type_support,
            topic_name,
            &rcl_options) != RCL_RET_OK) {
        backend->~MicroPythonSubscriptionBackend();
        m_del(uint8_t, allocd, sizeof(MicroPythonSubscriptionBackend));
        return false;
    }

    if (!backend->init_serialized_buffer(512u)) {
        rcl_ret_t _fini_rc2 = rcl_subscription_fini(&backend->subscription, backend->node);
        (void)_fini_rc2;
        backend->~MicroPythonSubscriptionBackend();
        m_del(uint8_t, allocd, sizeof(MicroPythonSubscriptionBackend));
        return false;
    }

    backend->valid = true;
    self->backend = backend;
    return true;
#else
    (void)options;
    self->backend = self;
    return true;
#endif
}

/**
 * @brief Releases the native subscription handle for a binding subscription.
 *
 * @param[in,out] self Subscription whose backend state should be released.
 */
void SubscriptionBackendFini(mp_rosette_subscription_obj_t* self) noexcept
{
    if (self == nullptr || self->backend == nullptr) {
        return;
    }
#if ROSETTE_MICROPY_HAS_MICRO_ROS
    auto* backend = GetSubscriptionBackend(self);
    if (backend != nullptr) {
        backend->~MicroPythonSubscriptionBackend();
        m_del(uint8_t, backend, sizeof(MicroPythonSubscriptionBackend));
    }
#endif
    self->backend = nullptr;
}

/**
 * @brief Drains currently available serialized samples for one subscription.
 *
 * The function performs non-blocking serialized takes until the middleware
 * reports that no more samples are ready. Each payload is decoded through the
 * subscription schema and delivered to the Python callback immediately.
 * Messages with no callback registered are silently dropped.
 * Callback exceptions are printed and swallowed so one handler failure does
 * not permanently stop the cooperative ROS pump.
 *
 * @param[in,out] self Subscription to drain.
 * @return true when the drain completed without a hard backend error.
 */
bool SubscriptionDispatchPending(mp_rosette_subscription_obj_t* self) noexcept
{
    if (self == nullptr || self->backend == nullptr) {
        return true;
    }

#if ROSETTE_MICROPY_HAS_MICRO_ROS
    auto* backend = GetSubscriptionBackend(self);
    if (backend == nullptr || !backend->valid) {
        return false;
    }

    while (true) {
        backend->serialized.buffer_length = 0u;
        rmw_message_info_t info{};
        const rcl_ret_t ret = rcl_take_serialized_message(
            &backend->subscription,
            &backend->serialized,
            &info,
            nullptr);
        if (ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
            return true;
        }
        if (ret != RCL_RET_OK) {
            return false;
        }

        if (self->callback_obj == mp_const_none) {
            continue;  /* no callback — drop message */
        }

        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            const auto* schema = static_cast<const mp_rosette_schema_obj_t*>(
                MP_OBJ_TO_PTR(self->schema_obj));
            mp_obj_t message = SchemaCdrDecode(
                schema,
                backend->serialized.buffer,
                backend->serialized.buffer_length);
            if (message != mp_const_none) {
                mp_call_function_1(self->callback_obj, message);
            }
            nlr_pop();
        } else {
            PrintCallbackException(MP_OBJ_FROM_PTR(nlr.ret_val));
        }
    }
#else
    return true;
#endif
}

}  // namespace rosette::bindings::micropython
