#!/usr/bin/env bash
set -euo pipefail

RUN_ID="${1:-${EXPERIMENT_RUN_ID:-}}"
if [[ -z "${RUN_ID}" ]]; then
  echo "Usage: $0 RUN_ID" >&2
  exit 2
fi

OUT_DIR="/app/experiments/logs/${RUN_ID}"
mkdir -p "${OUT_DIR}"

echo "Recording ROS topics to ${OUT_DIR}/ros_topics.bag"
echo "Stop with Ctrl+C; the bag is closed by rosbag on exit."

exec rosbag record --lz4 -O "${OUT_DIR}/ros_topics.bag" \
  /odom \
  /odom_enc \
  /cmd_vel \
  /tf \
  /tf_static \
  /robot_pose \
  /robot_state \
  /mir_status_msg \
  /rosout

