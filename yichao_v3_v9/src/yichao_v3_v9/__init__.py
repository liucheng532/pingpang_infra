"""Offline V3/V9 integration and input-only diagnostics. No control transport."""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
HANDOFF = ROOT / 'vendor/handoff/20260906_v3_planner_190_199'
# Only frozen, private snapshots; never import from a live checkout.
sys.path.insert(0, str(ROOT / 'vendor'))
sys.path.insert(0, str(ROOT / 'vendor/deploy/g1_gym_deploy'))
