#!/usr/bin/env bash
# Runs one device under test inside a simulated ROS 2 network on one host.
#
# The device is one participant of the topology (ROSETTE_DEVICE, default
# board:0), run on the RMW under test. Every other participant of the
# topology runs on a stock RMW inside one simulator process
# (discovery_scale_sim.py), each in its own rclpy context. The run passes when
# every simulated participant that the topology says matches the device
# matched it, every participant lists the device's node, and the device's
# probed services answered.
#
# Prerequisite: a sourced ROS 2 environment, plus the colcon overlay that
# provides the RMW under test.
#
# Environment:
#   ROSETTE_TOPOLOGY      topology file          (default: example_topology.yaml here)
#   ROSETTE_DEVICE        device's participant   (default: board:0)
#   ROSETTE_RMW           RMW under test         (default: rmw_rosette)
#   ROSETTE_PEER_RMW      RMW of the network     (default: rmw_fastrtps_cpp)
#   ROSETTE_DEVICE_START  first | last           (default: first; last joins a running network)
#   ROSETTE_STAGGER_MS    pause between network participants (default: 0, all at once)
#   ROSETTE_DOMAIN        DDS domain id          (default: 44)
#   ROSETTE_DURATION_SEC  network run time       (default: 60)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPOLOGY="${ROSETTE_TOPOLOGY:-${HERE}/example_topology.yaml}"
DEVICE="${ROSETTE_DEVICE:-board:0}"
RMW_UNDER_TEST="${ROSETTE_RMW:-rmw_rosette}"
PEER_RMW="${ROSETTE_PEER_RMW:-rmw_fastrtps_cpp}"
DEVICE_START="${ROSETTE_DEVICE_START:-first}"
STAGGER_MS="${ROSETTE_STAGGER_MS:-0}"
DOMAIN="${ROSETTE_DOMAIN:-44}"
DURATION_SEC="${ROSETTE_DURATION_SEC:-60}"
LOG_DIR="${TMPDIR:-/tmp}/rosette-discovery-scale-$(date +%s)-$$"
mkdir -p "${LOG_DIR}"

# Stock participants exchange everything over UDP, as on separate hosts.
export FASTRTPS_DEFAULT_PROFILES_FILE="${HERE}/fastdds_udp_only.xml"
watch_node="$(python3 "${HERE}/discovery_topology.py" "${TOPOLOGY}" --node-of "${DEVICE}")"
python3 "${HERE}/discovery_topology.py" "${TOPOLOGY}" --focus "${DEVICE}" | tee "${LOG_DIR}/analysis.txt"
echo "[scale] device=${DEVICE} on ${RMW_UNDER_TEST}, network on ${PEER_RMW}," \
  "device starts ${DEVICE_START}, logs=${LOG_DIR}"

start_device() {
  RMW_IMPLEMENTATION="${RMW_UNDER_TEST}" python3 "${HERE}/discovery_scale_sim.py" "${TOPOLOGY}" \
    --domain "${DOMAIN}" --only "${DEVICE}" --duration-sec $((DURATION_SEC + 30)) \
    >"${LOG_DIR}/device.log" 2>&1 &
  device_pid=$!
}

network_status=0
if [[ "${DEVICE_START}" == "first" ]]; then
  start_device
  sleep 2
fi

RMW_IMPLEMENTATION="${PEER_RMW}" python3 "${HERE}/discovery_scale_sim.py" "${TOPOLOGY}" \
  --domain "${DOMAIN}" --exclude "${DEVICE}" --watch-node "${watch_node}" \
  --stagger-ms "${STAGGER_MS}" --duration-sec "${DURATION_SEC}" \
  --report "${LOG_DIR}/network.json" >"${LOG_DIR}/network.log" 2>&1 &
network_pid=$!

if [[ "${DEVICE_START}" == "last" ]]; then
  # Long enough for the whole network to be up and to know each other.
  sleep 20
  start_device
fi

wait "${network_pid}" || network_status=1
kill -INT "${device_pid}" 2>/dev/null || true
wait "${device_pid}" 2>/dev/null || true

grep '^\[sim\]' "${LOG_DIR}/network.log" || true
dropped=$(grep -c "peer table full" "${LOG_DIR}/device.log" || true)
echo "[scale] device dropped ${dropped} participant announcements (peer table full)"
if [[ ${network_status} -eq 0 ]]; then
  echo "[scale] PASS: every peer that matches the device reached it"
else
  echo "[scale] FAIL: see ${LOG_DIR}"
fi

exit ${network_status}
