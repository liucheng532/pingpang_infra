#!/usr/bin/env bash
set -Eeuo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
action=${1:-start}
if (( $# > 1 )); then printf 'Too many arguments\n' >&2; exit 2; fi
case "$action" in
  start)
    if [[ ${V6R10_ACTIVE_SITE_CONFIRMED:-} != YES ]]; then
      if [[ ! -t 0 ]]; then
        printf 'Interactive site confirmation required; run this command in a terminal.\n' >&2
        exit 64
      fi
      printf 'Confirm BOTH robots have power, correct placement/tethers, clear area, and working e-stop.\n'
      read -r -p 'Type YES to start both V11 Controllers in active mode: ' answer
      if [[ $answer != YES ]]; then
        printf 'Cancelled; no Controller was started.\n' >&2
        exit 64
      fi
    fi
    export V6R10_ACTIVE_SITE_CONFIRMED=YES
    exec "$root/scripts/start_v11_robots.sh" start active --site-safety-confirmed
    ;;
  check)
    export V6R10_ACTIVE_SITE_CONFIRMED=YES
    exec "$root/scripts/start_v11_robots.sh" check active --site-safety-confirmed
    ;;
  dry-run)
    exec "$root/scripts/start_v11_robots.sh" dry-run active
    ;;
  status|stop)
    exec "$root/scripts/start_v11_robots.sh" "$action"
    ;;
  -h|--help)
    printf 'Usage: bash scripts/start_yichao_robots.sh [start|check|dry-run|status|stop]\n'
    printf 'Default start: confirmed V11 active; start the workstation entry first.\n'
    exit 0 ;;
  *) printf 'Unknown action: %s\n' "$action" >&2; exit 2 ;;
esac
