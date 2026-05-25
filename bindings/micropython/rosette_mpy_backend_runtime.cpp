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
#include "py/objstr.h"
#include "py/runtime.h"
}

#if ROSETTE_MICROPY_HAS_MICRO_ROS
#include "rosette/ros/micro_ros/context.hpp"
#include "rosette/ros/micro_ros/custom_transport.hpp"
#include "rosette/ros/micro_ros/node.hpp"

#include <rcl/publisher.h>
#include <rcl/types.h>
#include <rclc/publisher.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_typesupport_introspection_c/identifier.h>
#include <rosidl_typesupport_introspection_c/message_introspection.h>
#include <std_msgs/msg/float32.h>

#include <new>
#endif

namespace rosette::bindings::micropython {

namespace {

#if ROSETTE_MICROPY_HAS_MICRO_ROS

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
        const char* ip = mp_obj_str_get_str(owner->agent_ip_obj);
        std::strncpy(agent_address, ip, sizeof(agent_address) - 1);
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
        options.domain_id = owner->domain_id;
        options.client_key = owner->client_key;
        options.executor_handles = owner->executor_handles;
        options.spin_timeout_ns = owner->spin_timeout_ns;
        options.auto_discover_agent = false;
        options.udp_agent_address = agent_address;
        options.udp_agent_port = owner->agent_port;
#if ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM
        options.use_custom_transport = true;
        options.custom_transport =
            static_cast<milkros::micro_ros::CustomTransport*>(
                owner->transport.implementation);
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

    // Owned copies of the type name strings (members_ fields point here).
    std::string namespace_str;
    std::string name_str;

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
        const char* slash = std::strchr(full_name, '/');
        if (slash != nullptr) {
            namespace_str.assign(full_name, static_cast<size_t>(slash - full_name));
            namespace_str += "__msg";
            name_str.assign(slash + 1);
        } else {
            namespace_str = "unknown__msg";
            name_str = (full_name[0] != '\0') ? full_name : "Unknown";
        }

        // A placeholder member is required by some RMW implementations that
        // inspect member_count_ before accepting the type support. We provide
        // one minimal entry so the publisher is accepted.
        member_placeholder = {};
        member_placeholder.name_    = name_str.c_str();
        member_placeholder.type_id_ = 0u;  // FIELD_TYPE_NOT_SET / placeholder

        members = {};
        members.message_namespace_ = namespace_str.c_str();
        members.message_name_      = name_str.c_str();
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
            if (std::strcmp(id, rosidl_typesupport_introspection_c__identifier) == 0) {
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
void SetDynamicPublisherBackend(mp_rosette_publisher_obj_t* self,
                                MicroPythonDynamicPublisherBackend* backend) noexcept
{
    const uintptr_t raw = reinterpret_cast<uintptr_t>(backend);
    self->backend = reinterpret_cast<void*>(raw | kDynamicPublisherTag);
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
    if (mp_obj_is_type(
            payload,
            reinterpret_cast<const mp_obj_type_t*>(&rosette_message_type))) {
        auto* message =
            static_cast<mp_rosette_message_obj_t*>(MP_OBJ_TO_PTR(payload));
        payload = message->values_obj;
    }
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

/**
 * @brief Returns the value for one dictionary key or a fallback object.
 *
 * Message conversion accepts both schema-created messages and plain
 * dictionaries. This helper keeps optional-field extraction compact while
 * using MicroPython's dictionary map directly instead of adding another
 * parser-like lookup layer.
 *
 * @param[in] dict Python dictionary-like object to inspect.
 * @param[in] key Field key qstr.
 * @param[in] fallback Value returned when the key is absent.
 * @return Field value or `fallback`.
 */
mp_obj_t DictGetOr(mp_obj_t dict, qstr key, mp_obj_t fallback)
{
    if (!mp_obj_is_dict_or_ordereddict(dict)) {
        return fallback;
    }
    mp_map_t* map = mp_obj_dict_get_map(dict);
    mp_map_elem_t* elem = mp_map_lookup(map, MP_OBJ_NEW_QSTR(key), MP_MAP_LOOKUP);
    return elem == nullptr ? fallback : elem->value;
}

/**
 * @brief Normalizes a Python payload into a field dictionary when possible.
 *
 * @param[in] payload Schema-created message or dictionary.
 * @return Dictionary containing message fields, or `MP_OBJ_NULL`.
 */
mp_obj_t PayloadValues(mp_obj_t payload) noexcept
{
    if (mp_obj_is_type(
            payload,
            reinterpret_cast<const mp_obj_type_t*>(&rosette_message_type))) {
        auto* message =
            static_cast<mp_rosette_message_obj_t*>(MP_OBJ_TO_PTR(payload));
        return message->values_obj;
    }
    return mp_obj_is_dict_or_ordereddict(payload) ? payload : MP_OBJ_NULL;
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
    if (self == nullptr || self->closed || !self->transport.socket_ready ||
        self->socket_obj == mp_const_none) {
        return false;
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
        backend->~MicroPythonContextBackend();
        m_del(uint8_t, allocd, sizeof(MicroPythonContextBackend));
        return false;
    }
    const auto result = backend->context.start_result();
    self->backend_start_error =
        static_cast<uint8_t>(backend->context.last_start_error());
    if (!result.ok()) {
        backend->~MicroPythonContextBackend();
        m_del(uint8_t, allocd, sizeof(MicroPythonContextBackend));
        return false;
    }
    self->backend = backend;
    return true;
#else
    self->backend_start_error = 0u;
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
    auto* backend = GetContextBackend(self);
    return backend != nullptr && backend->context.tick();
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

}  // namespace rosette::bindings::micropython
