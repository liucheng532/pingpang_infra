#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONPATH="$PWD/src"
export ROS_MASTER_URI=http://127.0.0.1:11311
# Capture requires an explicit, already assigned workstation subscriber address.
exec nice -n 19 ionice -c 3 .venv/bin/python -B -m yichao_v3_v9.observer "$@"
