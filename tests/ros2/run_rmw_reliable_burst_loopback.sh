#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
# Runs rmw_reliable_burst_probe.py on one host: a stock-RMW publisher sends one
# reliable burst while the rmw_rosette subscription is not taking, and the
# subscription must then take exactly the newest min(count, depth) samples, in
# order. The subscriber's windows start when the publisher has published, so
# slow discovery under packet loss does not cut short the repair.
# Passes only when both roles pass.
#
# Prerequisite: a sourced ROS 2 environment plus the colcon overlay that
# contains rmw_rosette.
#
# Environment:
#   ROSETTE_RMW       RMW under test (subscriber)   (default: rmw_rosette)
#   ROSETTE_PEER_RMW  stock RMW (publisher)         (default: rmw_fastrtps_cpp)
#   ROSETTE_DOMAIN    DDS domain id                 (default: 42)
#   ROSETTE_COUNT     samples in the burst          (default: 5)
#   ROSETTE_DEPTH     KEEP_LAST depth, both sides   (default: 10)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROBE="${SCRIPT_DIR}/rmw_reliable_burst_probe.py"

RMW_UNDER_TEST="${ROSETTE_RMW:-rmw_rosette}"
PEER_RMW="${ROSETTE_PEER_RMW:-rmw_fastrtps_cpp}"
DOMAIN="${ROSETTE_DOMAIN:-42}"
COUNT="${ROSETTE_COUNT:-5}"
DEPTH="${ROSETTE_DEPTH:-10}"
# The random part keeps sessions apart when several start in the same second
# with the same PID, as in parallel containers.
SESSION="burst_$(date +%s)_$$_${RANDOM}"
LOG_DIR="${TMPDIR:-/tmp}/rosette-rmw-burst-${SESSION}"
mkdir -p "${LOG_DIR}"

echo "[rmw-burst] subscriber=${RMW_UNDER_TEST} publisher=${PEER_RMW} count=${COUNT} depth=${DEPTH} logs=${LOG_DIR}"

run_probe() {
  local role="$1" rmw="$2"
  RMW_IMPLEMENTATION="${rmw}" python3 "${PROBE}" \
    --role "${role}" --domain "${DOMAIN}" --session "${SESSION}" \
    --count "${COUNT}" --depth "${DEPTH}" --published-marker "${LOG_DIR}/published" \
    >"${LOG_DIR}/${role}.log" 2>&1
}

run_probe subscriber "${RMW_UNDER_TEST}" &
SUB_PID=$!
run_probe publisher "${PEER_RMW}" &
PUB_PID=$!

status=0
wait "${SUB_PID}" || { echo "[rmw-burst] subscriber (${RMW_UNDER_TEST}) FAILED"; status=1; }
wait "${PUB_PID}" || { echo "[rmw-burst] publisher (${PEER_RMW}) FAILED"; status=1; }

for role in subscriber publisher; do
  echo "----- ${role}.log (tail)"
  tail -n 4 "${LOG_DIR}/${role}.log"
done

if [[ ${status} -eq 0 ]]; then
  echo "[rmw-burst] PASS: ${RMW_UNDER_TEST} <- ${PEER_RMW}"
fi
exit ${status}
