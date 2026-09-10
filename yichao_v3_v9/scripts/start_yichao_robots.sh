#!/usr/bin/env bash
set -euo pipefail
launcher=/home/odl/codebase/pingpang_doubles_v9_runtime/scripts/start_v11_dual_robots.sh
action="${1:-start}"
if (( $# > 1 )); then echo 'Too many arguments' >&2; exit 2; fi
case "$action" in
  start|check|dry-run) args=("$action" active normal active 1.0 v11-teacher) ;;
  status|stop) args=("$action") ;;
  -h|--help)
    echo 'Usage: bash scripts/start_yichao_robots.sh [start|check|dry-run|status|stop]'
    echo 'Uses the ORIGINAL Fixed V11 launcher: active normal active 1.0 v11-teacher.'
    echo 'Start workstation inputs first. Each robot keeps its original two R2 steps.'
    exit 0 ;;
  *) echo "Unknown action: $action" >&2; exit 2 ;;
esac
if [[ "$action" == dry-run ]]; then
  printf '%q ' bash "$launcher" "${args[@]}"
  printf '\n'
  exit 0
fi
exec bash "$launcher" "${args[@]}"
