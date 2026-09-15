#!/usr/bin/env bash
set -euo pipefail

robot_id="${1:?usage: $0 table_left|table_right [shadow|active]}"
mode="${2:-shadow}"
case "${robot_id}" in
  table_left)
    ros_ip=192.168.124.164
    lcm_suffix=198
    mirror_args=()
    planner_home_y=0.20
    bootstrap_args=(--bootstrap-outward-hold)
    ;;
  table_right)
    ros_ip=192.168.123.164
    lcm_suffix=66
    mirror_args=()
    planner_home_y=-0.20
    bootstrap_args=(--startup-home-current)
    ;;
  *) echo "unknown robot_id: ${robot_id}" >&2; exit 2 ;;
esac
case "${mode}" in
  shadow) mode_args=(--shadow) ;;
  active) mode_args=() ;;
  *) echo "mode must be shadow or active" >&2; exit 2 ;;
esac

deploy_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_MASTER_URI=http://192.168.123.165:11311
export ROS_IP="${ros_ip}"
unset ROS_HOSTNAME
export LCM_DEFAULT_URL="udpm://239.255.76.${lcm_suffix}:7667?ttl=0"
diagnostics_dir="${deploy_root}/logs/${robot_id}/v01"
diagnostics_path="${diagnostics_dir}/v01_active_$(date +%Y%m%d_%H%M%S).jsonl"
command=(
  python3 -u scripts/deploy_policy.py
  --policy "${deploy_root}/policy/v10move_i21500/student_v10move18500_xrecovery_1666_model21500.onnx"
  --external-planner
  --robot-id "${robot_id}"
  --racket-hand right
  --planner-home-y "${planner_home_y}"
  --episode-start-x 0.2
  --lcm-url "${LCM_DEFAULT_URL}"
  --record-dir "${deploy_root}/logs/${robot_id}"
  --external-hit-command-mode stream
  --hit-policy-source teacher
  --hit-teacher-history distill-reset
  --hit-reference-lead-steps 1
  --hit-arm7-residual-mode off
  --hit-arm7-residual-scale 1.0
  --hit-arm7-diagnostics-jsonl "${diagnostics_path}"
  "${mirror_args[@]}"
  "${bootstrap_args[@]}"
  "${mode_args[@]}"
)

if [[ "${DUAL_RIGHT_HAND_DRY_RUN:-0}" == 1 ]]; then
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
mkdir -p "${diagnostics_dir}"

cd "${deploy_root}/g1_gym_deploy"
exec "${command[@]}"
