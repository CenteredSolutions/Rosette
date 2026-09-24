#!/usr/bin/env bash
# Fragmentation acceptance test at a small datagram limit (default 1472, the
# UDP payload of a 1500-byte Ethernet MTU: what a WIZnet hardware-UDP link
# carries without IP fragmentation).
#
# rmw_rosette (role alpha) runs with RMW_ROSETTE_MAX_DATAGRAM_SIZE=LIMIT; the
# stock peer (role beta) runs with a Fast DDS profile capping maxMessageSize to
# LIMIT. rmw_frag_probe.py exchanges images up to 60 KB and fragmented
# get_parameters replies, verifying every octet. The session passes only when
# both probes pass AND rosette's transport counters show that no datagram
# exceeded LIMIT in either direction and none was truncated.
#
# The same counters are the on-hardware check: a board built with a 1472
# limit reports them over its console.
#
# Environment:
#   ROSETTE_FRAG_LIMIT     datagram limit in octets     (default: 1472)
#   ROSETTE_PEER_RMW       stock reference RMW          (default: rmw_fastrtps_cpp)
#   ROSETTE_DOMAIN         DDS domain id                (default: 43)
#   ROSETTE_DURATION_SEC   session length               (default: 20)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROBE="${SCRIPT_DIR}/rmw_frag_probe.py"
LIMIT="${ROSETTE_FRAG_LIMIT:-1472}"
PEER_RMW="${ROSETTE_PEER_RMW:-rmw_fastrtps_cpp}"
DOMAIN="${ROSETTE_DOMAIN:-43}"
DURATION_SEC="${ROSETTE_DURATION_SEC:-20}"
SESSION="frag_$(date +%s)"
LOG_DIR="${TMPDIR:-/tmp}/rosette-rmw-frag-${SESSION}"
mkdir -p "${LOG_DIR}"

# Fast DDS: UDPv4 only (no shared memory), datagrams capped at LIMIT.
PROFILE="${LOG_DIR}/fastdds_capped.xml"
cat >"${PROFILE}" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_capped</transport_id>
      <type>UDPv4</type>
      <maxMessageSize>${LIMIT}</maxMessageSize>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="capped" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>udp_capped</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF

echo "[rmw-frag] limit=${LIMIT} alpha=rmw_rosette beta=${PEER_RMW} domain=${DOMAIN} logs=${LOG_DIR}"

ROS_DOMAIN_ID="${DOMAIN}" RMW_IMPLEMENTATION="${PEER_RMW}" \
  FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE}" RMW_FASTRTPS_USE_QOS_FROM_XML=0 \
  python3 "${PROBE}" --role beta --session "${SESSION}" --duration-sec "${DURATION_SEC}" \
  >"${LOG_DIR}/beta.log" 2>&1 &
BETA_PID=$!
ROS_DOMAIN_ID="${DOMAIN}" RMW_IMPLEMENTATION=rmw_rosette \
  RMW_ROSETTE_MAX_DATAGRAM_SIZE="${LIMIT}" RMW_ROSETTE_TRANSPORT_STATS=1 \
  python3 "${PROBE}" --role alpha --session "${SESSION}" --duration-sec "${DURATION_SEC}" \
  >"${LOG_DIR}/alpha.log" 2>&1 &
ALPHA_PID=$!

status=0
wait "${ALPHA_PID}" || { echo "[rmw-frag] alpha (rmw_rosette) probe FAILED"; status=1; }
wait "${BETA_PID}"  || { echo "[rmw-frag] beta (${PEER_RMW}) probe FAILED"; status=1; }

grep -h "\[frag-probe\]" "${LOG_DIR}/alpha.log" "${LOG_DIR}/beta.log" || true

stats="$(grep -h "rmw_rosette transport:" "${LOG_DIR}/alpha.log" | tail -n 1 || true)"
echo "${stats:-[rmw-frag] no transport stats reported}"
field() { sed -n "s/.* $1=\([0-9]*\).*/\1/p" <<<"${stats}"; }
if [[ -z "${stats}" ]]; then
  status=1
else
  max_tx="$(field max_tx)"; max_rx="$(field max_rx)"
  for counter in oversize_tx oversize_rx truncated_rx; do
    if [[ "$(field "${counter}")" != "0" ]]; then
      echo "[rmw-frag] ${counter} is $(field "${counter}"), expected 0"; status=1
    fi
  done
  (( max_tx <= LIMIT )) || { echo "[rmw-frag] rosette sent a ${max_tx}-octet datagram"; status=1; }
  (( max_rx <= LIMIT )) || { echo "[rmw-frag] rosette received a ${max_rx}-octet datagram"; status=1; }
fi

if [[ ${status} -eq 0 ]]; then
  echo "[rmw-frag] PASS: every datagram <= ${LIMIT} octets, all frames and replies intact"
fi
exit ${status}
