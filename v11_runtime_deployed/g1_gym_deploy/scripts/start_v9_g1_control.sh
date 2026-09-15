#!/usr/bin/env bash
set -euo pipefail

robot_id="${1:?usage: $0 table_left|table_right}"
case "${robot_id}" in
  table_left)  lcm_suffix=198 ;;
  table_right) lcm_suffix=66 ;;
  *) echo "unknown robot_id: ${robot_id}" >&2; exit 2 ;;
esac

deploy_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export LCM_DEFAULT_URL="udpm://239.255.76.${lcm_suffix}:7667?ttl=0"
exec "${deploy_root}/unitree_sdk2/build/bin/g1_control" eth0

