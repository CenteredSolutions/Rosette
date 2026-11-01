#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
# Runs rmw_lease_rematch_probe.py as role board on the RMW under test and as
# role watch on a stock RMW, on one host. The board's participant announces
# itself less often than its lease lasts, so the stock watcher expires it and
# re-discovers it on the next announcement, while the board keeps the
# watcher. Passes only when the watcher lists the board's node, loses it,
# and lists it again with its services: the board republished its
# ros_discovery_info message for the watcher's re-matched reader.
#
# Prerequisite: a sourced ROS 2 environment plus a colcon overlay whose
# rmw_rosette is built to announce less often than its 20 s lease:
#
#   colcon build --packages-select rmw_rosette \
#     --cmake-args "-DCMAKE_CXX_FLAGS=-DRTPS_SPDP_RESEND_PERIOD_MS=30000u"
#
# Environment:
#   ROSETTE_RMW          RMW under test (board)     (default: rmw_rosette)
#   ROSETTE_PEER_RMW     stock RMW (watcher)        (default: rmw_fastrtps_cpp)
#   ROSETTE_DOMAIN       DDS domain id              (default: 42)
#   ROSETTE_DURATION_SEC session length             (default: 45; must cover one
#                        lease expiry and the next announcement)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROBE="${SCRIPT_DIR}/rmw_lease_rematch_probe.py"
RMW_UNDER_TEST="${ROSETTE_RMW:-rmw_rosette}"
PEER_RMW="${ROSETTE_PEER_RMW:-rmw_fastrtps_cpp}"
DOMAIN="${ROSETTE_DOMAIN:-42}"
DURATION_SEC="${ROSETTE_DURATION_SEC:-45}"
SESSION="lease_$(date +%s)_$$"
NODE="rosette_${SESSION}"
LOG_DIR="${TMPDIR:-/tmp}/rosette-rmw-lease-${SESSION}"
mkdir -p "${LOG_DIR}"

echo "[rmw-lease] board=${RMW_UNDER_TEST} watch=${PEER_RMW} domain=${DOMAIN} logs=${LOG_DIR}"

RMW_IMPLEMENTATION="${PEER_RMW}" python3 "${PROBE}" --role watch --node "${NODE}" \
  --domain "${DOMAIN}" --duration-sec "${DURATION_SEC}" --expect-rmw "${PEER_RMW}" \
  >"${LOG_DIR}/watch.log" 2>&1 &
WATCH_PID=$!
sleep 2
RMW_IMPLEMENTATION="${RMW_UNDER_TEST}" python3 "${PROBE}" --role board --node "${NODE}" \
  --domain "${DOMAIN}" --duration-sec "${DURATION_SEC}" --expect-rmw "${RMW_UNDER_TEST}" \
  >"${LOG_DIR}/board.log" 2>&1 &
BOARD_PID=$!

status=0
wait "${WATCH_PID}" || { echo "[rmw-lease] watcher (${PEER_RMW}) FAILED"; status=1; }
wait "${BOARD_PID}" || { echo "[rmw-lease] board (${RMW_UNDER_TEST}) FAILED"; status=1; }

for role in watch board; do
  echo "----- ${role}.log (tail)"
  tail -n 8 "${LOG_DIR}/${role}.log"
done

if [[ ${status} -eq 0 ]]; then
  echo "[rmw-lease] PASS: ${PEER_RMW} lists ${RMW_UNDER_TEST}'s node again after its lease expired"
fi
exit ${status}
