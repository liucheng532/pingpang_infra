"""Yichao positioning and inference on a frozen copy of the Fixed runtime."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
HANDOFF = ROOT / 'vendor/handoff/20260906_v3_planner_190_199'
BASELINE = ROOT / 'baseline'
sys.path.insert(0, str(BASELINE))
