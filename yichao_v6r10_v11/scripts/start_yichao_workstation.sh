#!/usr/bin/env bash
set -Eeuo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
action=${1:-start}
if (( $# > 1 )); then printf 'Too many arguments\n' >&2; exit 2; fi
case "$action" in
  start|status|stop|dry-run) ;;
  -h|--help)
    printf 'Usage: bash scripts/start_yichao_workstation.sh [start|status|stop|dry-run]\n'
    printf 'Default start: Predictor + command-free bootstrap, then automatic V6R10 active switch.\n'
    exit 0 ;;
  *) printf 'Unknown action: %s\n' "$action" >&2; exit 2 ;;
esac
python=${YICHAO_PYTHON:-python3}
exec "$python" -B "$root/scripts/run_active_bootstrap.py" "$action"
