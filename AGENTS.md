# Working on Rosette

Rosette puts a microcontroller on a ROS 2 network. It supplies networking (socket backends, the WIZnet driver,
address providers) and storage, and two ROS 2 backends; each firmware project picks one:

- **`rmw_rosette`**: a DDSI-RTPS stack (`src/rtps`, `include/rosette/rtps`) and an rmw on top (`rmw_rosette/`). The
  board is a DDS participant and needs no extra process. Hosts run it with `RMW_IMPLEMENTATION=rmw_rosette`.
- **micro-ROS agent**: `rmw_microxrcedds` over XRCE-DDS to a `micro_ros_agent` on a host. Less flash and RAM; needs
  the agent on the network.

## Ground rules

- Application code is stock `rclc` and builds against either backend; only session setup differs. Anything Rosette
  adds above rmw works on both backends, or it is a defect.
- No Rosette-specific ROS client API and no `rcl` emulation. The `milkros` name and its `Context` API are retired.
- Interoperability is proven against stock `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`, and `micro_ros_agent`, never
  against Rosette alone.

## Code

Details and examples: [.agents/code-style.md](.agents/code-style.md).

- Google style for C++ and Python; the rules here win where they differ.
- Small files with specific names that say what they implement.
- A function is at most 100 lines. Split only into pieces a reader would recognize by name.
- Every `if`/`else`/`for`/`while`/`do` body in braces on its own line; a blank line after a `}` that ends a block;
  wrapped expressions break before the operator.
- `tools/style/run_style_checks.sh` passes before you push; CI runs the same script.
- When editing, improve the surrounding code toward these rules where practical.

## Documentation

Tiers, template, and diagrams: [.agents/doc-comments.md](.agents/doc-comments.md).

- `@brief` on every function. Public API, documented in the header: purpose, every `@param`, and `@return`.
- `Verification:` and `References:` only where an external standard defines correct behavior: DDSI-RTPS, the ROS 2
  rmw API, an RFC, a datasheet. Code without such an authority (private helpers, style checkers, build scripts)
  carries neither.
- Comments say why a line exists, not what it does. Draw wire formats and state machines instead of describing them.
- Write code and docs as finished. When code changes, rewrite its docs to fit; never describe them as interim,
  changed, or migrated.
