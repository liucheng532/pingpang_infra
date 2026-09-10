#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-active}"
case "$mode" in
  active) args=(--start --active) ;;
  shadow) args=(--start) ;;
  dry-run) args=(--active) ;;
  -h|--help)
    echo 'Usage: bash scripts/start_yichao_workstation.sh [active|shadow|dry-run]'
    echo 'Default active: original Predictor + Yichao Planner + new Monitor on 8090.'
    echo 'Ctrl-C stops only these workstation children, not robot Controllers.'
    exit 0 ;;
  *) echo "Unknown mode: $mode" >&2; exit 2 ;;
esac
if (( $# > 1 )); then echo 'Too many arguments' >&2; exit 2; fi
if [[ "$mode" != dry-run ]]; then
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
fi
export ROS_MASTER_URI=http://192.168.123.165:11311
export ROS_IP=192.168.123.165
unset ROS_HOSTNAME
cd "$root"
exec "$root/.venv/bin/python" scripts/run_yichao_stack.py "${args[@]}" --with-predictor --monitor-port 8090
