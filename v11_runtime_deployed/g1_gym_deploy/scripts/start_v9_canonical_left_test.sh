#!/usr/bin/env bash
set -euo pipefail

mode="${1:-shadow}"
case "${mode}" in
  shadow) mode_args=(--shadow) ;;
  active) mode_args=() ;;
  *) echo "usage: $0 [shadow|active]" >&2; exit 2 ;;
esac

deploy_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ros_master_uri=http://192.168.123.165:11311
ros_ip=192.168.123.164
lcm_url='udpm://239.255.76.66:7667?ttl=0'
torso_topic=/doubles/table_right/torso_pose_origin

command=(
  python3 scripts/deploy_policy.py
  --torso-topic "${torso_topic}"
  --lcm-url "${lcm_url}"
  --record-dir "${deploy_root}/logs/table_right_canonical"
  --record-mode triggered
  --record-duration 15
  --record-pre-trigger 2
  --record-post-trigger 3
  --startup-home-current
  "${mode_args[@]}"
)

if [[ "${V9_CANONICAL_TEST_DRY_RUN:-0}" == 1 ]]; then
  echo "ROS_MASTER_URI=${ros_master_uri}"
  echo "ROS_IP=${ros_ip}"
  echo "LCM_DEFAULT_URL=${lcm_url}"
  printf '%q ' "${command[@]}"
  printf '\n'
  exit 0
fi

if pgrep -f '[d]eploy_policy.py' >/dev/null; then
  echo "deploy_policy.py is already running; stop it before canonical test" >&2
  exit 3
fi
if [[ "${mode}" == active ]] && ! pgrep -f '[g]1_control' >/dev/null; then
  echo "active canonical test requires g1_control on the 66 multicast group" >&2
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
