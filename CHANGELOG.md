# Changelog

All notable changes to Rosette are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html). Before 1.0, a minor
version may change the public API.

## [Unreleased]

## [0.1.0] - 2026-11-02

First public release, under the Apache License 2.0.

### Added

- `rosette::rtps`: a DDSI-RTPS 2.5 stack with SPDP and SEDP discovery, stateful
  reliable writers and readers, and fragmentation, which allocates nothing at
  runtime. A two-tier peer table keeps a board discoverable on networks larger
  than its peer table, and shared proxy pools size per-match state by the
  matches that exist.
- `rmw_rosette`: a ROS 2 middleware implementation over it, for stock `rclc`
  firmware and for `rcl`, `rclc` and `rclpy` programs on POSIX hosts, built and
  tested in CI on ROS 2 Humble and Jazzy. It has C type support only, so
  `rclcpp` programs reach Rosette nodes over another rmw; the README lists this
  and the other known limitations.
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
  parsers, and interoperability tests against stock `rmw_fastrtps_cpp` and
  `rmw_cyclonedds_cpp` using the upstream ROS 2 test suites.

### Notes

- The development history is published in full. For publication, references
  to unrelated private projects were removed, author addresses were unified,
  and commits were signed off; dates are as recorded.

[Unreleased]: https://github.com/CenteredSolutions/Rosette/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/CenteredSolutions/Rosette/releases/tag/v0.1.0
