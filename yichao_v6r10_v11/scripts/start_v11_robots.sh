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
  if [[ ${V6R10_ACTIVE_SITE_CONFIRMED:-} != YES ]]; then
    printf 'V11 active requires V6R10_ACTIVE_SITE_CONFIRMED=YES\n' >&2
    exit 64
  fi
fi

if [[ $action == dry-run ]]; then
  printf '{"delegate":"%s","action":"start","controller_mode":"%s","profile":"normal","residual_mode":"off","residual_scale":1.0,"residual_family":"v11-teacher","active_requires_site_confirmation":true,"remote_contacted":false}\n' \
    "$launcher" "$mode"
  exit 0
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
