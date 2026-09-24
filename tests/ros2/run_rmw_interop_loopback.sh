#!/usr/bin/env bash
# Runs a single-host rmw interop session: rmw_interop_probe.py as role alpha on
# the RMW under test and as role beta on a stock RMW, in the same DDS domain.
# Passes only when both sides load their requested RMW, each receives at
# least EXPECT_RX samples from the other, and each completes at least
# EXPECT_CALLS correlated calls to each of the other's services.
#
# Prerequisite: a sourced ROS 2 environment plus the colcon overlay that
# contains rmw_rosette.
#
# Environment:
#   ROSETTE_RMW          RMW under test         (default: rmw_rosette)
#   ROSETTE_PEER_RMW     stock reference RMW    (default: rmw_fastrtps_cpp)
#   ROSETTE_DOMAIN       DDS domain id          (default: 42, away from 0's traffic)
#   ROSETTE_DURATION_SEC session length         (default: 20)
#   ROSETTE_EXPECT_RX    min samples per side   (default: 3)
#   ROSETTE_EXPECT_CALLS min calls per peer service per side (default: 3)
#   ROSETTE_CAPTURE      1 = alpha also records a pcap (needs tcpdump privileges)
#   ROSETTE_CAPTURE_IFACE  capture interface    (default: any on Linux)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROBE="${SCRIPT_DIR}/rmw_interop_probe.py"

RMW_UNDER_TEST="${ROSETTE_RMW:-rmw_rosette}"
PEER_RMW="${ROSETTE_PEER_RMW:-rmw_fastrtps_cpp}"
DOMAIN="${ROSETTE_DOMAIN:-42}"
DURATION_SEC="${ROSETTE_DURATION_SEC:-20}"
EXPECT_RX="${ROSETTE_EXPECT_RX:-3}"
EXPECT_CALLS="${ROSETTE_EXPECT_CALLS:-3}"
SESSION="interop_$(date +%s)"
LOG_DIR="${TMPDIR:-/tmp}/rosette-rmw-interop-${SESSION}"
mkdir -p "${LOG_DIR}"

CAPTURE_ARGS=()
if [[ "${ROSETTE_CAPTURE:-0}" == "1" ]]; then
  CAPTURE_ARGS=(--capture --pcap-file "${LOG_DIR}/session.pcap")
  if [[ -n "${ROSETTE_CAPTURE_IFACE:-}" ]]; then
    CAPTURE_ARGS+=(--capture-iface "${ROSETTE_CAPTURE_IFACE}")
  fi
fi

echo "[rmw-interop] alpha=${RMW_UNDER_TEST} beta=${PEER_RMW} domain=${DOMAIN} logs=${LOG_DIR}"

run_probe() {
  local role="$1" rmw="$2"
  shift 2
  RMW_IMPLEMENTATION="${rmw}" python3 "${PROBE}" \
    --role "${role}" --domain "${DOMAIN}" --session "${SESSION}" \
    --duration-sec "${DURATION_SEC}" --expect-rx "${EXPECT_RX}" \
    --expect-calls "${EXPECT_CALLS}" "$@" \
    >"${LOG_DIR}/${role}.log" 2>&1
}

run_probe beta "${PEER_RMW}" &
BETA_PID=$!
run_probe alpha "${RMW_UNDER_TEST}" "${CAPTURE_ARGS[@]+"${CAPTURE_ARGS[@]}"}" &
ALPHA_PID=$!

status=0
wait "${ALPHA_PID}" || { echo "[rmw-interop] alpha (${RMW_UNDER_TEST}) FAILED"; status=1; }
wait "${BETA_PID}"  || { echo "[rmw-interop] beta (${PEER_RMW}) FAILED"; status=1; }

for role in alpha beta; do
  echo "----- ${role}.log (tail)"
  tail -n 5 "${LOG_DIR}/${role}.log"
done

if [[ ${status} -eq 0 ]]; then
  echo "[rmw-interop] PASS: ${RMW_UNDER_TEST} <-> ${PEER_RMW}"
fi
exit ${status}
