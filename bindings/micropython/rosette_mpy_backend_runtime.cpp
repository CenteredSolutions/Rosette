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
#include <rclc/publisher.h>
#include <std_msgs/msg/float32.h>

#include <string>
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
    std::string agent_address;
    std::string agent_port;
    milkros::micro_ros::ContextOptions options;
    milkros::micro_ros::Context context;

    explicit MicroPythonContextBackend(mp_rosette_context_obj_t* owner)
        : agent_address(mp_obj_str_get_str(owner->agent_ip_obj))
        , agent_port(std::to_string(owner->agent_port))
        , options(BuildOptions(owner, agent_address.c_str(), agent_port.c_str()))
        , context(options)
    {}

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
        const char* agent_address,
        const char* agent_port)
    {
        milkros::micro_ros::ContextOptions options{};
        options.domain_id = owner->domain_id;
        options.client_key = owner->client_key;
        options.executor_handles = owner->executor_handles;
        options.spin_timeout_ns = owner->spin_timeout_ns;
        options.auto_discover_agent = false;
        options.udp_agent_address = agent_address;
        options.udp_agent_port = agent_port;
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

    /**
     * @brief Releases the native publisher handle when initialized.
     *
     * The publisher must be finalized before its parent node and context are
     * destroyed. Binding objects keep parent references, so normal MicroPython
     * lifetime rules preserve that ordering for user-created objects.
     */
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
 * @brief Returns the typed micro-ROS publisher backend for one binding publisher.
 *
 * @param[in] self MicroPython publisher object whose backend should be decoded.
 * @return Typed backend pointer, or `nullptr` when publisher initialization failed.
 */
MicroPythonPublisherBackend* GetPublisherBackend(mp_rosette_publisher_obj_t* self) noexcept
{
    return self == nullptr
           ? nullptr
           : static_cast<MicroPythonPublisherBackend*>(self->backend);
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

    auto* backend = new MicroPythonContextBackend(self);
    if (backend->options.use_custom_transport && backend->options.custom_transport == nullptr) {
        self->backend_start_error = static_cast<uint8_t>(
            milkros::micro_ros::ContextStartError::kCustomTransportMissing);
        delete backend;
        return false;
    }
    const auto result = backend->context.start_result();
    self->backend_start_error =
        static_cast<uint8_t>(backend->context.last_start_error());
    if (!result.ok()) {
        delete backend;
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
    delete backend;
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

    self->backend = new MicroPythonNodeBackend(static_cast<milkros::micro_ros::Node&&>(node));
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
    delete backend;
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
    self->depth = options.depth;
    self->reliable = options.reliable;

#if ROSETTE_MICROPY_HAS_MICRO_ROS
    auto* node_obj =
        static_cast<mp_rosette_node_obj_t*>(MP_OBJ_TO_PTR(self->node_obj));
    auto* schema =
        static_cast<mp_rosette_schema_obj_t*>(MP_OBJ_TO_PTR(self->schema_obj));
    auto* node_backend = GetNodeBackend(node_obj);
    if (node_backend == nullptr || schema == nullptr ||
        schema->message_kind == ROSETTE_MPY_MESSAGE_UNSUPPORTED) {
        return false;
    }

    auto* backend = new MicroPythonPublisherBackend();
    backend->node = node_backend->node.native_handle();
    backend->message_kind = schema->message_kind;
    if (backend->node == nullptr) {
        delete backend;
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
        delete backend;
        return false;
    }

    backend->valid = true;
    self->backend = backend;
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
    auto* backend = GetPublisherBackend(self);
    delete backend;
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
