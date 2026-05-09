# micro-ROS Backend Surface

This directory adds a micro-ROS-oriented backend that mirrors the application
shape of Rosette's existing `milkros` facade:

- `Context`
- `Node`
- `Publisher<T>`
- `Subscription<T, CB>`
- `Service<T, H>`
- `Client<T, N>`
- `ActionServer<T, G, E>`
- `ActionClient<T, N>`
- `Timer<CB>`

## Design

The adapter uses `rcl` + `rclc` for entity management and callback execution,
and `rmw_microxrcedds` for MCU-to-Agent transport. The important architectural
shift versus the in-tree RTPS backend is that the MCU no longer participates as
an RTPS peer directly. Instead:

1. The MCU boots a micro-ROS XRCE client.
2. The client discovers or targets a micro-ROS Agent.
3. The Agent joins the ROS 2 DDS graph on the MCU's behalf.

That is why agent discovery is handled at `Context::start()` time rather than
inside topic/service creation.

The key split is:

- `rmw_microxrcedds` is not a POSIX-only idea. micro-ROS supports MCU clients,
  RTOS targets, custom transports, and Ethernet UDP on embedded boards.
- automatic Agent discovery is documented as a UDP feature. If Rosette uses its
  own socket/driver layer or a fully custom transport, discovery belongs in a
  Rosette shim that resolves the Agent address and feeds it into the context.

## Trait contract

The micro-ROS adapter keeps using the existing `milkros::message_traits` and
`milkros::service_traits` templates, but asks them for ROSIDL type support
instead of raw CDR encode/decode functions.

For a topic message, define:

```cpp
template<>
struct milkros::message_traits<my_pkg__msg__Status> {
    static const rosidl_message_type_support_t* rosidl_type_support() noexcept;
    static void init_message(my_pkg__msg__Status& msg) noexcept;
    static void fini_message(my_pkg__msg__Status& msg) noexcept;
};
```

For a service tag, define:

```cpp
struct ResetDeviceService {};

template<>
struct milkros::service_traits<ResetDeviceService> {
    using Request = custom__srv__ResetDevice_Request;
    using Response = custom__srv__ResetDevice_Response;

    static const rosidl_service_type_support_t* rosidl_type_support() noexcept;
    static void init_request(Request& req) noexcept;
    static void fini_request(Request& req) noexcept;
    static void init_response(Response& resp) noexcept;
    static void fini_response(Response& resp) noexcept;
};
```

For an action tag, define:

```cpp
struct MoveArmAction {};

template<>
struct milkros::action_traits<MoveArmAction> {
    using Goal = custom__action__MoveArm_Goal;
    using Result = custom__action__MoveArm_Result;
    using Feedback = custom__action__MoveArm_Feedback;
    using SendGoalRequest = custom__action__MoveArm_SendGoal_Request;
    using GetResultResponse = custom__action__MoveArm_GetResult_Response;
    using FeedbackMessage = custom__action__MoveArm_FeedbackMessage;

    static const rosidl_action_type_support_t* rosidl_action_type_support() noexcept;
    static void init_send_goal_request(SendGoalRequest& request) noexcept;
    static void fini_send_goal_request(SendGoalRequest& request) noexcept;
    static void init_result_response(GetResultResponse& response) noexcept;
    static void fini_result_response(GetResultResponse& response) noexcept;
    static void init_feedback_message(FeedbackMessage& feedback) noexcept;
    static void fini_feedback_message(FeedbackMessage& feedback) noexcept;
};
```

## Discovery model

micro-ROS Agent discovery is supported through `rmw_uros_discover_agent()` when
UDP transport is selected. That is the documented micro-ROS discovery path and
depends on the Agent being launched with discovery enabled.

For custom transports, Rosette should provide a `CustomTransport` adapter and
either:

- set a static Agent address, or
- implement an out-of-band discovery mechanism through `AgentLocator` before
  `Context::start()`.

The important caveat is platform-specific: upstream Micro XRCE-DDS documents
its built-in discovery profile as Linux-only, even though the wider micro-ROS
stack and `rmw_uros_ping_agent_options()` API are usable on MCU targets. In
practice that means:

- `posix` builds can rely on stock UDP discovery when the Agent is started with
  discovery enabled.
- MCU boards should treat Agent resolution as a Rosette concern and feed the
  resolved endpoint into the custom transport through `AgentLocator`.

Rosette now includes a transport-agnostic multicast locator in
[multicast_agent_locator.hpp](path/to/Rosette/ros/micro_ros/multicast_agent_locator.hpp).
It reuses the upstream Micro XRCE-DDS discovery request and reply format on top
of Rosette's socket abstraction. The default settings match the upstream
documentation and client implementation:

- multicast group: `239.255.0.2`
- multicast port: `7400`
- Agent UDP port: `8888`

That means a stock discovery-capable Agent on Linux and Rosette's own MCU
locator now speak the same `GET_INFO` / `INFO` exchange on the wire.

The matching host-side responder lives at
[tools/micro_ros/agent_multicast_responder.py](path/to/Rosette/tools/micro_ros/agent_multicast_responder.py).
It is useful as a reference endpoint and as a fallback helper when the host
environment does not expose the upstream discovery server behavior directly.
Run it on the host that also runs the micro-ROS Agent:

```bash
python3 tools/micro_ros/agent_multicast_responder.py --agent-port 8888
```

If the responder is not on the same host or interface as the Agent, advertise
an explicit Agent IPv4 address:

```bash
python3 tools/micro_ros/agent_multicast_responder.py \
  --agent-port 8888 \
  --agent-address 192.168.1.50
```

## Board Smoke Target

Rosette exposes a board-aware smoke executable named
`rtps_micro_ros_board_smoke`. It is selected with the same `RTPS_BOARD`
mechanism as the rest of the project.

The smoke path is:

1. Bring up the selected board runtime.
2. Start networking.
3. On MCU boards, wait for DHCP/network readiness and start NTP sync.
4. Start a micro-ROS node.
5. Resolve or discover the Agent.
6. Publish MCU temperature on `mcu_temperature_c`.

On `RTPS_BOARD=posix`, the topic value comes from native host thermal sensors
when available and falls back to `NaN` when the host OS does not expose one.

For custom transports, the smoke target follows this policy:

- if `ROSETTE_MICRO_ROS_AGENT_IP` is set, use that static endpoint
- otherwise, use upstream-compatible XRCE multicast discovery

## Actions

The micro-ROS backend now includes a typed action wrapper built on upstream
`rclc` action support. The surface mirrors the RTPS backend's cooperative
execution model:

- `Node::create_action_server()` accepts a goal callback and an execute callback.
- `Context::tick()` advances accepted goals and retries terminal result sends
  until the upstream action stack is ready to deliver them.
- `Node::create_action_client()` provides typed goal-response, feedback, and
  result callbacks.

Important caveats:

- each `ActionServer` currently supports one active goal at a time
- action entities should live at least as long as the owning `Context`, because
  upstream `rclc` provides add-to-executor APIs without matching remove APIs
- action wrappers require the generated ROSIDL C action support types listed in
  the trait contract above
