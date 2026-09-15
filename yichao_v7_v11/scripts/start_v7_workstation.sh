#!/usr/bin/env bash
set -Eeuo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
configured_python=/home/odl/miniconda3/envs/tabletennis-active-v1/bin/python
if [[ -x $configured_python ]]; then
  python=$configured_python
else
  python=${PYTHON:-python3}
fi
action=${1:-dry-run}
mode=${2:-shadow}
shift $(( $# > 0 ? 1 : 0 ))
shift $(( $# > 0 ? 1 : 0 ))
export ROS_MASTER_URI=http://192.168.123.165:11311
export ROS_IP=192.168.123.165
unset ROS_HOSTNAME
export PYTHONPATH="$root/src:/opt/ros/noetic/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
exec "$python" -B "$root/scripts/run_workstation.py" "$action" --mode "$mode" "$@"
