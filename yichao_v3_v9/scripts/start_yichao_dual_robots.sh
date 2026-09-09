#!/usr/bin/env bash
set -euo pipefail
YICHAO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$YICHAO_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$YICHAO_ROOT/.venv/bin/python" -B -m yichao_v3_v9.two_terminal robots "$@"
