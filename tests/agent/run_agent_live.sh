#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
# Runs agent_live_probe (stock rclc on the micro-ROS agent backend, over
# Rosette's POSIX sockets) against a stock micro_ros_agent in Docker and a
# stock rclpy echo peer. The probe exchanges traffic, checks its clock against
# this host's, then stops the agent, checks the loss is detected, restarts it,
# and checks the session is rebuilt. Passes only when the probe and the peer
# both pass.
#
# Prerequisites: Docker with the microros/micro-ros-agent image, a sourced
# ROS 2 environment (for the peer), and the probe built:
#   cmake -S . -B build-host-agent -DROSETTE_BOARD=posix -DROSETTE_ENABLE_MICRO_ROS=ON
#   cmake --build build-host-agent --target agent_live_probe -j
#
# Environment:
#   ROSETTE_PROBE        probe binary      (default: build-host-agent/agent_live_probe)
#   ROSETTE_AGENT_IMAGE  agent image       (default: microros/micro-ros-agent:humble)
#   ROSETTE_AGENT_PORT   agent UDP port    (default: 8888)
#   ROSETTE_PEER_RMW     peer's stock RMW  (default: rmw_fastrtps_cpp)
#   ROSETTE_DOMAIN       DDS domain id     (default: 42)
#   ROSETTE_COUNT        round trips and calls per traffic phase (default: 5)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PROBE="${ROSETTE_PROBE:-${REPO_DIR}/build-host-agent/agent_live_probe}"
IMAGE="${ROSETTE_AGENT_IMAGE:-microros/micro-ros-agent:humble}"
PORT="${ROSETTE_AGENT_PORT:-8888}"
PEER_RMW="${ROSETTE_PEER_RMW:-rmw_fastrtps_cpp}"
DOMAIN="${ROSETTE_DOMAIN:-42}"
COUNT="${ROSETTE_COUNT:-5}"
SESSION="agent_live_$(date +%s)"
CONTAINER="rosette_${SESSION}"
LOG_DIR="${TMPDIR:-/tmp}/rosette-agent-live-${SESSION}"
mkdir -p "${LOG_DIR}"

# --ipc=host shares /dev/shm, so the agent's Fast DDS participant and a
# same-host peer can use the shared-memory transport they both advertise.
AGENT_START="docker run -d --rm --name ${CONTAINER} --net=host --ipc=host ${IMAGE} udp4 --port ${PORT} >/dev/null"
AGENT_STOP="docker stop -t 1 ${CONTAINER} >/dev/null"

cleanup() {
  docker stop -t 1 "${CONTAINER}" >/dev/null 2>&1 || true
  [[ -n "${PEER_PID:-}" ]] && kill "${PEER_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "[agent-live] image=${IMAGE} port=${PORT} peer=${PEER_RMW} domain=${DOMAIN} logs=${LOG_DIR}"
eval "${AGENT_START}"

RMW_IMPLEMENTATION="${PEER_RMW}" python3 "${SCRIPT_DIR}/agent_echo_peer.py" \
  --session "${SESSION}" --domain "${DOMAIN}" --duration-sec 75 \
  --expect-rx "$((COUNT * 2))" --expect-calls "$((COUNT * 2))" \
  --expect-rmw "${PEER_RMW}" >"${LOG_DIR}/peer.log" 2>&1 &
PEER_PID=$!

PROBE_RC=0
"${PROBE}" --agent-ip 127.0.0.1 --agent-port "${PORT}" --session "${SESSION}" \
  --domain "${DOMAIN}" --count "${COUNT}" \
  --stop-cmd "${AGENT_STOP}" --start-cmd "${AGENT_START}" \
  2>&1 | tee "${LOG_DIR}/probe.log" || PROBE_RC=$?

# The peer's verdict needs the probe's traffic only; stop it early once the probe is done.
kill -TERM "${PEER_PID}" 2>/dev/null || true
PEER_RC=0
wait "${PEER_PID}" || PEER_RC=$?
PEER_PID=""
cat "${LOG_DIR}/peer.log"

echo "[agent-live] probe=${PROBE_RC} peer=${PEER_RC}"
[[ "${PROBE_RC}" == 0 && "${PEER_RC}" == 0 ]]
