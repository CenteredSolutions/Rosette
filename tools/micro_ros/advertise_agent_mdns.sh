#!/usr/bin/env bash
#
# Advertise a running micro-ROS Agent over mDNS / DNS-SD so a Rosette board can
# find it with MdnsAgentLocator (see include/rosette/ros/agent_locator_mdns.hpp).
#
# This is a thin wrapper around avahi-publish: it registers the DNS-SD service
# type that the board browses by default and holds the registration until you
# press Ctrl-C. The board then resolves the Agent by the source address of the
# mDNS reply, which is routable even when the Agent runs behind Docker.
#
# Usage:
#   tools/micro_ros/advertise_agent_mdns.sh [PORT] [SERVICE_TYPE] [INSTANCE_NAME]
#
#   PORT           UDP port the Agent listens on         (default: 8888)
#   SERVICE_TYPE   DNS-SD service type without ".local"   (default: _microrosagent._udp)
#   INSTANCE_NAME  Human-readable instance label          (default: "micro-ROS Agent")
#
# Requires the avahi-utils package (avahi-publish) and a running avahi-daemon.
# For a container, either run this on the host or run avahi-daemon in the
# container with the host network.

set -euo pipefail

PORT="${1:-8888}"
SERVICE_TYPE="${2:-_microrosagent._udp}"
INSTANCE_NAME="${3:-micro-ROS Agent}"

if ! command -v avahi-publish >/dev/null 2>&1; then
    echo "error: avahi-publish not found (install avahi-utils / avahi-tools)" >&2
    exit 1
fi

echo "Advertising '${INSTANCE_NAME}' as ${SERVICE_TYPE}.local on port ${PORT}"
echo "Press Ctrl-C to stop."
exec avahi-publish -s "${INSTANCE_NAME}" "${SERVICE_TYPE}" "${PORT}"
