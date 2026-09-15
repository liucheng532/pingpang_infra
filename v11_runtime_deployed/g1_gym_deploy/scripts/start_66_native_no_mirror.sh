#!/usr/bin/env bash
set -euo pipefail

mode="${1:-shadow}"
case "${mode}" in
  shadow) mode_args=(--shadow) ;;
  active) mode_args=() ;;
  *) echo "usage: $0 [shadow|active]" >&2; exit 2 ;;
esac

deploy_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_MASTER_URI=http://192.168.123.165:11311
export ROS_IP=192.168.123.164
unset ROS_HOSTNAME
export LCM_DEFAULT_URL='udpm://239.255.76.66:7667?ttl=0'

command=(
  python3 -u scripts/deploy_policy.py
  --policy "${deploy_root}/policy/v9_model19000/student_v9_m14500_timedhandoff_1666_model19000.onnx"
  --external-planner
  --robot-id table_right
  --racket-hand right
  --table-right-move-mode native-no-mirror
  --planner-home-y -0.20
  --episode-start-x 0.2
  --lcm-url "${LCM_DEFAULT_URL}"
  --record-dir "${deploy_root}/logs/table_right_native_v9_i19000"
  --external-hit-command-mode stream
  --hit-policy-source teacher
  --hit-teacher-history distill-reset
  --hit-reference-lead-steps 1
  --hit-arm7-residual-mode off
  --hit-arm7-residual-scale 1.0
  --startup-home-current
  "${mode_args[@]}"
)

if [[ "${NATIVE_NO_MIRROR_DRY_RUN:-0}" == 1 ]]; then
  echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
  echo "ROS_IP=${ROS_IP}"
  echo "LCM_DEFAULT_URL=${LCM_DEFAULT_URL}"
  printf '%q ' "${command[@]}"
  printf '\n'
  exit 0
fi

set +u
source /opt/ros/noetic/setup.bash
set -u
cd "${deploy_root}/g1_gym_deploy"
exec "${command[@]}"
