#!/usr/bin/env bash
set -euo pipefail

mode="${1:-shadow}"
case "${mode}" in
  shadow) mode_args=(--shadow) ;;
  active) mode_args=() ;;
  *) echo "usage: $0 [shadow|active] [normal|hit_home] [off|shadow|active] [scale] [v01|v11-teacher]" >&2; exit 2 ;;
esac

profile="${2:-normal}"
case "${profile}" in
  normal) stationary_args=(); startup_args=(--bootstrap-outward-hold); record_suffix="" ;;
  hit_home) stationary_args=(--stationary-hit-test hit_home); startup_args=(--startup-home-current); record_suffix="_hit_home" ;;
  *) echo "usage: $0 [shadow|active] [normal|hit_home] [off|shadow|active] [scale] [v01|v11-teacher]" >&2; exit 2 ;;
esac

residual_mode="${3:-shadow}"
case "${residual_mode}" in
  off|shadow|active) ;;
  *) echo "usage: $0 [shadow|active] [normal|hit_home] [off|shadow|active] [scale] [v01|v11-teacher]" >&2; exit 2 ;;
esac
residual_scale="${4:-1.0}"
residual_family="${5:-v01}"
case "${residual_family}" in
  v01|v11-teacher) ;;
  *) echo "residual family must be v01 or v11-teacher (fifth argument)" >&2; exit 2 ;;
esac
if [[ ! ${residual_scale} =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] \
  || ! awk -v value="${residual_scale}" 'BEGIN { exit !(value >= 0.0 && value <= 1.0) }'; then
  echo "residual scale must be within [0, 1]" >&2
  exit 2
fi

deploy_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_master_uri=http://192.168.123.165:11311
ros_ip=192.168.124.164
lcm_url='udpm://239.255.76.198:7667?ttl=0'

command=(
  python3 -u scripts/deploy_policy.py
  --policy "${deploy_root}/policy/v11_resume_i42500/student_v11_r2i299_commonhold_1666_resume42500.onnx"
  --external-planner
  --robot-id table_left
  --racket-hand right
  --torso-topic /doubles/table_left/torso_pose_origin
  --planner-home-y 0.20
  --lcm-url "${lcm_url}"
  --record-dir "${deploy_root}/logs/table_left_v11_resume42500_canonical${record_suffix}"
  --record-mode circular
  --record-duration 600
  "${startup_args[@]}"
  --startup-relative-x
  --external-hit-command-mode stream
  --hit-policy-source teacher
  --hit-teacher-history distill-reset
  --hit-reference-lead-steps 1
  --hit-arm7-residual-mode "${residual_mode}"
  --hit-arm7-residual-family "${residual_family}"
  --hit-arm7-residual-scale "${residual_scale}"
  --hit-arm7-diagnostics-jsonl "${deploy_root}/logs/table_left_v11_hit_arm7_${residual_mode}.jsonl"
  "${stationary_args[@]}"
  "${mode_args[@]}"
)

if [[ "${V11_RESUME42500_198_DRY_RUN:-0}" == 1 ]]; then
  echo "ROS_MASTER_URI=${ros_master_uri}"
  echo "ROS_IP=${ros_ip}"
  echo "LCM_DEFAULT_URL=${lcm_url}"
  printf '%q ' "${command[@]}"
  printf '\n'
  exit 0
fi

if pgrep -f '[p]ython3 .*scripts/deploy_policy.py' >/dev/null; then
  echo "deploy_policy.py is already running" >&2
  exit 3
fi
if [[ "${mode}" == active ]] && ! pgrep -x g1_control >/dev/null; then
  echo "active deployment requires g1_control on the 198 multicast group" >&2
  exit 3
fi

set +u
source /opt/ros/noetic/setup.bash
set -u
export ROS_MASTER_URI="${ros_master_uri}"
export ROS_IP="${ros_ip}"
unset ROS_HOSTNAME
export LCM_DEFAULT_URL="${lcm_url}"

cd "${deploy_root}/g1_gym_deploy"
exec "${command[@]}"
