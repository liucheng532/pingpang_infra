#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
if str(DEPLOY_ROOT) not in sys.path:
    sys.path.insert(0, str(DEPLOY_ROOT))

from utils.ball_racket_diagnostics import analyze_hit_frames


def finite_summary(values):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"count": 0, "median": None, "p90": None, "max": None}
    return {
        "count": int(len(values)),
        "median": float(np.median(values)),
        "p90": float(np.quantile(values, 0.90)),
        "max": float(np.max(values)),
    }


def analyze_session(path, central_only=False):
    path = Path(path).expanduser().resolve()
    metadata = json.loads((path / "metadata.json").read_text())
    results = []
    for clip_path in sorted(path.glob("hit_*.npy")):
        frames = np.load(clip_path, mmap_mode="r")
        required = {"raw_ball_valid", "raw_ball_position", "raw_ball_velocity"}
        missing = required.difference(frames.dtype.names or ())
        if missing:
            raise RuntimeError(
                f"{clip_path} predates raw-ball diagnostics; missing fields: {sorted(missing)}"
            )
        hit = frames[np.isin(frames["phase_id"], (0, 1))]
        if not len(hit):
            continue
        entry = hit[0]
        observation = np.asarray(entry["teacher_policy_obs_history"], dtype=np.float64)
        target = observation[67:70].copy()
        if central_only and not (0.95 <= target[2] <= 1.10 and abs(target[1]) <= 0.20):
            continue
        result = analyze_hit_frames(frames)
        result.update(
            {
                "clip": clip_path.name,
                "robot_id": metadata.get("robot_id"),
                "mirror_left_hand": bool(metadata.get("mirror_left_hand", False)),
                "entry_tts_s": float(entry["corrected_tts"]),
                "motion_index": int(entry["hit_motion_index"]),
                "reference_step": int(entry["reference_step"]),
                "prediction_age_ms": float(entry["planner_prediction_age_ms"]),
                "command_age_ms": float(entry["planner_command_age_ms"]),
                "target_entry_canonical_m": target.tolist(),
            }
        )
        crossing = result.get("ball_strike_plane_position_canonical_m")
        if crossing is not None:
            result["strike_prediction_error_m"] = (
                np.asarray(crossing, dtype=np.float64) - target
            ).tolist()
        results.append(result)
    classifications = Counter(value["classification"] for value in results)
    summary = {
        "session": str(path),
        "robot_id": metadata.get("robot_id"),
        "clip_count": len(results),
        "classifications": dict(classifications),
        "center_distance_m": finite_summary(
            [value.get("center_distance_m", np.nan) for value in results]
        ),
        "closest_tts_s": finite_summary(
            [value.get("closest_tts_s", np.nan) for value in results]
        ),
        "raw_ball_valid_fraction": finite_summary(
            [value.get("valid_fraction", np.nan) for value in results]
        ),
        "prediction_age_ms": finite_summary(
            [value.get("prediction_age_ms", np.nan) for value in results]
        ),
        "command_age_ms": finite_summary(
            [value.get("command_age_ms", np.nan) for value in results]
        ),
    }
    return summary, results


def main():
    parser = argparse.ArgumentParser(description="Reconstruct real ball-to-racket contact geometry.")
    parser.add_argument("sessions", nargs="+", type=Path)
    parser.add_argument("--central-only", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    report = {"schema": "doubles_ball_racket_diagnostics_v1", "sessions": []}
    for session in args.sessions:
        summary, hits = analyze_session(session, central_only=args.central_only)
        report["sessions"].append({"summary": summary, "hits": hits})
        print(json.dumps(summary, sort_keys=True))
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
