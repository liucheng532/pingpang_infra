#!/usr/bin/env python3
"""Assess a completed local rollout, independently of the simulator exit code."""
import argparse
import json
import math
from pathlib import Path


def assess(directory: Path):
    summary = json.loads((directory / "v0_safe_rl_rollout.json").read_text())
    manifest = json.loads((directory / "local_manifest.json").read_text())
    metrics = summary["metrics"]
    requested = summary["shots_requested"]
    measured = summary["strikes_completed"]
    error_stats = summary["strike_error_statistics"]
    thresholds = {
        "position_error_m": 0.08, "velocity_error_mps": 1.0,
        "orientation_error_rad": 0.10, "timing_error_s": 0.04,
    }
    events = summary["strike_error_events"]
    passes = sum(all(math.isfinite(event[k]) and event[k] <= v
                     for k, v in thresholds.items()) for event in events)
    critical = (
        "physical_collision", "simultaneous_hit", "terminated", "timeout",
        "v0_any_unsafe", "v0_forced_timeout",
    )
    checks = {
        "requested_strikes_completed": requested is not None and measured >= requested,
        "handoffs_completed": metrics["handoff_completed"] >= measured,
        "both_robots_hit": set(summary["commit_sequence"]) == {"left", "right"},
        "strict_alternation": summary["strict_commit_alternation"],
        "complete_strike_events": len(events) == measured,
        "finite_measured_errors": all(
            v["count"] == measured and all(
                isinstance(v[k], (int, float)) and math.isfinite(v[k])
                for k in ("mean", "max")
            ) for v in error_stats.values()
        ),
        **{f"zero_{key}": metrics[key] == 0 for key in critical},
    }
    result = {
        "local_control_chain_pass": all(checks.values()),
        "checks": checks,
        "measured_strikes": measured,
        "requested_strikes": requested,
        "failed_strike_quality_count": metrics["v0_failed_strike_quality"],
        "quality_thresholds": thresholds,
        "quality_passes": passes,
        "quality_pass_rate": passes / measured if measured else None,
        "all_strikes_pass_quality": measured > 0 and passes == measured,
        "exact_training_patch_available": manifest["exact_training_patch_available"],
        "real_robot_acceptance": False,
        "ball_contact_and_outgoing_flight_simulated": False,
        "minimum_distances_m": summary["minimum_distances_m"],
        "strike_error_statistics": error_stats,
        "video": summary["video"],
    }
    (directory / "assessment.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    result = assess(parser.parse_args().directory)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["local_control_chain_pass"] else 1)
