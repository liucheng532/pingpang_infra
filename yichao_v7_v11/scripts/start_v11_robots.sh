#!/usr/bin/env bash
set -Eeuo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
config="$root/config/deployment.json"
launcher=/home/odl/codebase/pingpang_doubles_v9_runtime/scripts/start_v11_dual_robots.sh
python=${PYTHON:-python3}

action=${1:-dry-run}
mode=${2:-shadow}
confirmation=${3:-}
case "$action" in
  dry-run|check|start) ;;
  status|stop)
    exec "$launcher" "$action"
    ;;
  *) printf 'Usage: %s dry-run|check|start|status|stop [shadow|active]\n' "$0" >&2; exit 64 ;;
esac
case "$mode" in shadow|active) ;; *) printf 'Mode must be shadow or active\n' >&2; exit 64 ;; esac

if [[ $mode == active && $action != dry-run ]]; then
  if [[ $confirmation != --site-safety-confirmed ]]; then
    printf 'V11 active requires a new explicit --site-safety-confirmed invocation\n' >&2
    exit 64
  fi
  if [[ ${V7_ACTIVE_SITE_CONFIRMED:-} != YES ]]; then
    printf 'V11 active requires V7_ACTIVE_SITE_CONFIRMED=YES\n' >&2
    exit 64
  fi
fi

if [[ $action == dry-run ]]; then
  printf '{"delegate":"%s","action":"start","controller_mode":"%s","profile":"normal","residual_mode":"off","residual_scale":1.0,"residual_family":"v11-teacher","active_requires_site_confirmation":true,"remote_contacted":false}\n' \
    "$launcher" "$mode"
  exit 0
fi

set +u
source /opt/ros/noetic/setup.bash
set -u
export ROS_MASTER_URI=http://192.168.123.165:11311
export ROS_IP=192.168.123.165
unset ROS_HOSTNAME

require_live_torso_sample() {
  local topic=$1
  if ! timeout 3 rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
    printf 'V7 preflight FAILED: no live torso sample within 3s: %s\n' "$topic" >&2
    return 1
  fi
}

require_live_torso_sample /doubles/table_left/torso_pose_origin
require_live_torso_sample /doubles/table_right/torso_pose_origin

if PYTHONPATH="$root/src${PYTHONPATH:+:$PYTHONPATH}" "$python" -B \
  "$root/scripts/create_v11_attestation.py" \
  --config "$config" --mode "$mode" >/dev/null; then
  :
else
  status=$?
  printf 'V11 pre-start asset attestation failed; no Controller was started.\n' >&2
  exit "$status"
fi

"$launcher" "$action" "$mode" normal off 1.0 v11-teacher
if [[ $action == start ]]; then
  if PYTHONPATH="$root/src${PYTHONPATH:+:$PYTHONPATH}" "$python" -B "$root/scripts/create_v11_attestation.py" \
    --config "$config" --mode "$mode" --require-running; then
    :
  else
    status=$?
    printf 'V11 attestation failed; stopping the newly started controller session.\n' >&2
    "$launcher" stop || true
    exit "$status"
  fi
fi
