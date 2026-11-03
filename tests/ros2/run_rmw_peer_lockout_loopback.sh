#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
# Runs rmw_peer_lockout_probe.py on one host: a board on the RMW under test,
# built with two active peer slots, then ROSETTE_FILLERS stock filler
# participants that share nothing with it, then one late stock participant
# that calls the board's service. Passes only when the late participant
# reaches the board's service: a board whose active tier is full of peers it
# matches nothing with must still serve a peer it does.
#
# Prerequisite: a sourced ROS 2 environment plus a colcon overlay whose
# rmw_rosette has two active slots and room to know every participant:
#
#   colcon build --packages-select rmw_rosette \
#     --cmake-args -DRMW_ROSETTE_MAX_REMOTE_PARTICIPANTS=2 \
#     -DRMW_ROSETTE_MAX_KNOWN_PARTICIPANTS=8
#
# With ROSETTE_FILLERS=0 the same run is the control: the late participant
# then has a free slot, so a failure there is not about the table.
#
# Environment:
#   ROSETTE_RMW          RMW under test (board)           (default: rmw_rosette)
#   ROSETTE_PEER_RMW     stock RMW (fillers, late peer)   (default: rmw_fastrtps_cpp)
#   ROSETTE_FILLERS      filler participants              (default: 2)
#   ROSETTE_DOMAIN       DDS domain id                    (default: 43)
#   ROSETTE_WAIT_SEC     late peer's wait for the service (default: 20)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROBE="${SCRIPT_DIR}/rmw_peer_lockout_probe.py"
RMW_UNDER_TEST="${ROSETTE_RMW:-rmw_rosette}"
PEER_RMW="${ROSETTE_PEER_RMW:-rmw_fastrtps_cpp}"
FILLERS="${ROSETTE_FILLERS:-2}"
DOMAIN="${ROSETTE_DOMAIN:-43}"
WAIT_SEC="${ROSETTE_WAIT_SEC:-20}"
FILL_SETTLE_SEC=5
DURATION_SEC=$((WAIT_SEC + FILL_SETTLE_SEC + 20))
SESSION="lockout_$(date +%s)_$$"
LOG_DIR="${TMPDIR:-/tmp}/rosette-rmw-lockout-${SESSION}"
mkdir -p "${LOG_DIR}"

echo "[rmw-lockout] board=${RMW_UNDER_TEST} peers=${PEER_RMW} fillers=${FILLERS}" \
  "domain=${DOMAIN} logs=${LOG_DIR}"

common=(--session "${SESSION}" --domain "${DOMAIN}" --duration-sec "${DURATION_SEC}")
RMW_IMPLEMENTATION="${RMW_UNDER_TEST}" python3 "${PROBE}" --role board "${common[@]}" \
  --expect-rmw "${RMW_UNDER_TEST}" >"${LOG_DIR}/board.log" 2>&1 &
pids=($!)
sleep 2

for ((i = 1; i <= FILLERS; ++i)); do
  RMW_IMPLEMENTATION="${PEER_RMW}" python3 "${PROBE}" --role filler --index "${i}" \
    "${common[@]}" --expect-rmw "${PEER_RMW}" >"${LOG_DIR}/filler${i}.log" 2>&1 &
  pids+=($!)
done

# Every filler has announced itself to the board several times by now.
sleep "${FILL_SETTLE_SEC}"

status=0
RMW_IMPLEMENTATION="${PEER_RMW}" python3 "${PROBE}" --role late "${common[@]}" \
  --wait-sec "${WAIT_SEC}" --expect-rmw "${PEER_RMW}" >"${LOG_DIR}/late.log" 2>&1 \
  || status=1

kill -INT "${pids[@]}" 2>/dev/null || true
wait "${pids[@]}" 2>/dev/null || true

echo "----- late.log (tail)"
tail -n 4 "${LOG_DIR}/late.log"
dropped=$(grep -c "peer table full" "${LOG_DIR}/board.log" || true)
echo "[rmw-lockout] board dropped ${dropped} participant announcements (peer table full)"

if [[ ${status} -eq 0 ]]; then
  echo "[rmw-lockout] PASS: the late participant reached the board"
elif [[ ${dropped} -gt 0 ]]; then
  echo "[rmw-lockout] FAIL: the board's full participant table locked the late participant out"
else
  echo "[rmw-lockout] FAIL: the late participant failed with the table not full; see ${LOG_DIR}"
fi
exit ${status}
