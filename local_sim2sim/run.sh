#!/usr/bin/env bash
set -euo pipefail
SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM_PYTHON="${YICHAO_SIM_PYTHON:-/home/lyz/miniconda3/envs/yichao_sim2sim/bin/python}"
export PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=4
exec "$SIM_PYTHON" -B -u "$SIM_ROOT/run.py" "$@"
