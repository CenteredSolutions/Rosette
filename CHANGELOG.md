# Changelog

All notable changes to Rosette are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html). Before 1.0, a minor
version may change the public API.

## [Unreleased]

## [0.1.0]

First public release, under the Apache License 2.0.

### Added

- `rosette::rtps`: a DDSI-RTPS 2.5 stack with SPDP and SEDP discovery, stateful
  reliable writers and readers, and fragmentation, which allocates nothing at
  runtime. A two-tier peer table keeps a board discoverable on networks larger
  than its peer table, and shared proxy pools size per-match state by the
  matches that exist.
- `rmw_rosette`: a ROS 2 middleware implementation over it, for stock `rclc`
  firmware and for `rclcpp` / `rclpy` on POSIX hosts, built and tested in CI on
  ROS 2 Humble and Jazzy.
- A micro-ROS agent backend (`rmw_microxrcedds`) for firmware built against the
  same `rclc` application code.
- A register-level WIZnet driver for the W5100S, W6100 and W6300, with DHCP,
  RFC 3927 link-local addressing, and IPv6 link-local and SLAAC on the W6100 and
  W6300.
- Ports for RP2040 and RP2350 (bare metal) and POSIX hosts, and the boards
  `posix`, `w5100s-evb-pico2`, `w6100-evb-pico`, `w6100-evb-pico2` and
  `w6300-evb-pico2`. `PORTING.md` describes the port contracts.
- A LittleFS volume and file store for on-board configuration.
- Conformance tests keyed to DDSI-RTPS spec sections, fuzzing of the network
  parsers, and interoperability tests against stock `rmw_fastrtps_cpp`.

### Notes

- The history before this release was prepared for publication: internal
  project names and links to private tooling were removed from commit messages
  and earlier file versions, author addresses were unified, and commits were
  signed off. Dates and code are otherwise as written.

[Unreleased]: https://github.com/CenteredSolutions/Rosette/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/CenteredSolutions/Rosette/releases/tag/v0.1.0
