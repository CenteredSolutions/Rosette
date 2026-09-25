#!/usr/bin/env bash
# Runs rcl_graph_wait_probe (RMW under test) against rmw_graph_peer.py (stock
# RMW) on one host. The probe blocks on its node's graph guard condition,
# waiting for a publisher and then a subscription, while the peer creates one
# of each after a delay; it then checks both counts are 1, and sees both fall
# to 0 when the peer destroys them. Each wait must return within ROSETTE_MAX_WAIT_MS, well under its 10 s
# timeout, which it can only do if the graph guard condition is triggered.
#
# Prerequisite: a sourced ROS 2 environment plus the colcon overlay that
# contains rmw_rosette. The probe is compiled here against the sourced rcl.
#
# Environment:
#   ROSETTE_RMW          RMW under test (observer)  (default: rmw_rosette)
#   ROSETTE_PEER_RMW     stock RMW (peer)           (default: rmw_fastrtps_cpp)
#   ROSETTE_DOMAIN       DDS domain id              (default: 42)
#   ROSETTE_MAX_WAIT_MS  bound on each graph wait   (default: 7000)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RMW_UNDER_TEST="${ROSETTE_RMW:-rmw_rosette}"
PEER_RMW="${ROSETTE_PEER_RMW:-rmw_fastrtps_cpp}"
DOMAIN="${ROSETTE_DOMAIN:-42}"
MAX_WAIT_MS="${ROSETTE_MAX_WAIT_MS:-7000}"
SESSION="graph_$(date +%s)_$$"
TOPIC="/rosette_graph_${SESSION}"
LOG_DIR="${TMPDIR:-/tmp}/rosette-rmw-graph-${SESSION}"
mkdir -p "${LOG_DIR}"

PROBE_BIN="${LOG_DIR}/rcl_graph_wait_probe"
ROS_INC="${AMENT_PREFIX_PATH%%:*}/include"
[[ -d "/opt/ros/${ROS_DISTRO:-humble}/include" ]] && ROS_INC="/opt/ros/${ROS_DISTRO:-humble}/include"
INCLUDES=()
for pkg in rcl rcutils rmw rcl_yaml_param_parser rosidl_runtime_c rosidl_typesupport_interface; do
  INCLUDES+=("-I${ROS_INC}/${pkg}")
done
g++ -std=c++17 -O1 "${SCRIPT_DIR}/rcl_graph_wait_probe.cpp" "${INCLUDES[@]}" \
  -L"$(dirname "${ROS_INC}")/lib" -lrcl -lrmw_implementation -lrmw -lrcutils -o "${PROBE_BIN}"

echo "[rmw-graph] observer=${RMW_UNDER_TEST} peer=${PEER_RMW} topic=${TOPIC} logs=${LOG_DIR}"

RMW_IMPLEMENTATION="${RMW_UNDER_TEST}" "${PROBE_BIN}" "${TOPIC}" "${DOMAIN}" "${MAX_WAIT_MS}" \
  >"${LOG_DIR}/observer.log" 2>&1 &
OBS_PID=$!
RMW_IMPLEMENTATION="${PEER_RMW}" python3 "${SCRIPT_DIR}/rmw_graph_peer.py" \
  --topic "${TOPIC}" --domain "${DOMAIN}" --expect-rmw "${PEER_RMW}" \
  >"${LOG_DIR}/peer.log" 2>&1 &
PEER_PID=$!

status=0
wait "${OBS_PID}" || { echo "[rmw-graph] observer (${RMW_UNDER_TEST}) FAILED"; status=1; }
wait "${PEER_PID}" || { echo "[rmw-graph] peer (${PEER_RMW}) FAILED"; status=1; }

for role in observer peer; do
  echo "----- ${role}.log (tail)"
  tail -n 6 "${LOG_DIR}/${role}.log"
done

if [[ ${status} -eq 0 ]]; then
  echo "[rmw-graph] PASS: ${RMW_UNDER_TEST} observes ${PEER_RMW}"
fi
exit ${status}
