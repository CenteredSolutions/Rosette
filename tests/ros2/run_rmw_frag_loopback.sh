#!/usr/bin/env bash
# Fragmentation acceptance test at a small datagram limit (default 1472, the
# UDP payload of a 1500-byte Ethernet MTU: what a WIZnet hardware-UDP link
# carries without IP fragmentation).
#
# rmw_rosette (role alpha) runs with RMW_ROSETTE_MAX_DATAGRAM_SIZE=LIMIT and
# RMW_ROSETTE_STRICT_LINK, so every datagram it receives above LIMIT is
# discarded exactly as such a link would lose it. The peer (role beta) is a
# stock RMW. rmw_frag_probe.py exchanges images and get_parameters replies,
# verifying every octet.
#
# Peer modes (ROSETTE_PEER_MODE):
#   stock   (default) The peer runs with no configuration at all, so this
#           proves an unmodified ROS 2 system interoperates with a board.
#           Rosette -> peer carries every size (Rosette fragments to LIMIT).
#           Peer -> Rosette carries samples that fit one datagram; a stock
#           peer sends anything larger as a single IP-fragmented datagram,
#           which cannot cross a link without IP reassembly, and no RTPS
#           parameter lets a reader ask for smaller datagrams. Discovery must
#           survive the peer's packed multi-kilobyte SEDP datagrams.
#   capped  The peer's Fast DDS profile caps maxMessageSize at LIMIT (the
#           configuration a system integrator can apply to reach a board with
#           large samples), and both directions carry every size.
#
# The session passes only when both probes pass AND rosette's transport
# counters show it never sent a datagram above LIMIT and never truncated one.
# In capped mode it must also never have received one.
#
# Environment:
#   ROSETTE_PEER_MODE      stock | capped                (default: stock)
#   ROSETTE_FRAG_LIMIT     datagram limit in octets     (default: 1472)
#   ROSETTE_PEER_RMW       stock reference RMW          (default: rmw_fastrtps_cpp)
#   ROSETTE_DOMAIN         DDS domain id                (default: 43)
#   ROSETTE_DURATION_SEC   session length               (default: 20)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROBE="${SCRIPT_DIR}/rmw_frag_probe.py"
MODE="${ROSETTE_PEER_MODE:-stock}"
LIMIT="${ROSETTE_FRAG_LIMIT:-1472}"
PEER_RMW="${ROSETTE_PEER_RMW:-rmw_fastrtps_cpp}"
DOMAIN="${ROSETTE_DOMAIN:-43}"
DURATION_SEC="${ROSETTE_DURATION_SEC:-20}"
SESSION="frag_$(date +%s)"
LOG_DIR="${TMPDIR:-/tmp}/rosette-rmw-frag-${SESSION}"
mkdir -p "${LOG_DIR}"

ALL_SIZES="16,1400,1500,4000,20000,60000"
declare -a PEER_ENV=()
case "${MODE}" in
  stock)
    # Largest Image payload whose DATA, with RTPS and submessage headers,
    # still fits LIMIT; the peer's parameter reply is kept within it too.
    PEER_SIZES="16,$((LIMIT - 200))"
    PEER_PARAM_CHARS=$((LIMIT - 400))
    ;;
  capped)
    PEER_SIZES="${ALL_SIZES}"
    PEER_PARAM_CHARS=20000
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
    PEER_ENV=(FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE}" RMW_FASTRTPS_USE_QOS_FROM_XML=0)
    ;;
  *)
    echo "[rmw-frag] unknown ROSETTE_PEER_MODE=${MODE}"; exit 2 ;;
esac

echo "[rmw-frag] mode=${MODE} limit=${LIMIT} alpha=rmw_rosette beta=${PEER_RMW}" \
     "domain=${DOMAIN} logs=${LOG_DIR}"

env ROS_DOMAIN_ID="${DOMAIN}" RMW_IMPLEMENTATION="${PEER_RMW}" "${PEER_ENV[@]}" \
  python3 "${PROBE}" --role beta --session "${SESSION}" --duration-sec "${DURATION_SEC}" \
    --frame-sizes "${PEER_SIZES}" --param-chars "${PEER_PARAM_CHARS}" \
    --peer-frame-sizes "${ALL_SIZES}" --peer-param-chars 20000 \
  >"${LOG_DIR}/beta.log" 2>&1 &
BETA_PID=$!
env ROS_DOMAIN_ID="${DOMAIN}" RMW_IMPLEMENTATION=rmw_rosette \
  RMW_ROSETTE_MAX_DATAGRAM_SIZE="${LIMIT}" RMW_ROSETTE_STRICT_LINK=1 \
  RMW_ROSETTE_TRANSPORT_STATS=1 \
  python3 "${PROBE}" --role alpha --session "${SESSION}" --duration-sec "${DURATION_SEC}" \
    --frame-sizes "${ALL_SIZES}" --param-chars 20000 \
    --peer-frame-sizes "${PEER_SIZES}" --peer-param-chars "${PEER_PARAM_CHARS}" \
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
  counters=(oversize_tx truncated_rx)
  [[ "${MODE}" == capped ]] && counters+=(oversize_rx)
  for counter in "${counters[@]}"; do
    if [[ "$(field "${counter}")" != "0" ]]; then
      echo "[rmw-frag] ${counter} is $(field "${counter}"), expected 0"; status=1
    fi
  done
  max_tx="$(field max_tx)"
  (( max_tx <= LIMIT )) || { echo "[rmw-frag] rosette sent a ${max_tx}-octet datagram"; status=1; }
fi

if [[ ${status} -eq 0 ]]; then
  echo "[rmw-frag] PASS (${MODE} peer): rosette never sent above ${LIMIT} octets," \
       "every deliverable frame and reply intact"
fi
exit ${status}
