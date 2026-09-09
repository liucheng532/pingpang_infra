#!/usr/bin/env bash
set -euo pipefail
PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PACKAGE_ROOT"
exec nice -n 19 ionice -c 3 "$PACKAGE_ROOT/run_offline.sh" "$@"
