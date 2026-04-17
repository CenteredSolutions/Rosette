#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 /path/to/ros2_posix_chatter_smoke"
  exit 2
fi

BIN="$1"
if [[ ! -x "$BIN" ]]; then
  echo "Binary not executable: $BIN"
  exit 2
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "docker not found; skipping"
  exit 0
fi

IMAGE="${MILKDDS_ROS2_IMAGE:-osrf/ros:humble-desktop}"
TOPIC="${MILKDDS_ROS2_TOPIC:-/chatter}"
DOMAIN="${MILKDDS_ROS2_DOMAIN:-0}"

echo "[ros2-docker] using image: $IMAGE"

docker rm -f milkdds_ros2_echo >/dev/null 2>&1 || true

docker run --rm --name milkdds_ros2_echo --network host \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  "$IMAGE" \
  bash -lc "source /opt/ros/humble/setup.bash && timeout 25s ros2 topic echo ${TOPIC} std_msgs/msg/String --once" \
  > /tmp/milkdds_ros2_echo.log 2>&1 &
ECHO_PID=$!

cleanup() {
  docker rm -f milkdds_ros2_echo >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 2

ROS_DOMAIN_ID="$DOMAIN" "$BIN" --domain "$DOMAIN" --duration-ms 12000 --publish-ms 500 --topic "${TOPIC#/}"

wait "$ECHO_PID" || {
  echo "[ros2-docker] echo process failed"
  cat /tmp/milkdds_ros2_echo.log
  exit 1
}

if ! grep -q "milkDDS POSIX hello" /tmp/milkdds_ros2_echo.log; then
  echo "[ros2-docker] expected payload not observed"
  cat /tmp/milkdds_ros2_echo.log
  exit 1
fi

echo "[ros2-docker] success"
