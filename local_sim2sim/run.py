#!/usr/bin/env python3
"""Local-only V3/V9 Isaac evaluation; no ROS, DDS or hardware transport."""
from pathlib import Path
import hashlib
import json
import os
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
sys.path.insert(0, str(ROOT / "planner"))


def main():
    import run_isaac_doubles_relay as relay
    import evaluate_isaac_doubles_ppo as evaluator

    # Override only the asset location, using the user's supplied archive.
    relay.DEFAULT_UNITREE_DESCRIPTION_ROOT = ROOT / "assets/unitree_description"
    handoff = ROOT / "planner/handoff/20260906_v3_planner_190_199"
    output = ROOT / "outputs" / time.strftime("%Y%m%dT%H%M%S")
    args = [
        "--controller-root", str(ROOT / "controller"),
        "--checkpoint", str(handoff / "checkpoints/v9_student_iteration_19000.pt"),
        "--move-motion-data", str(PROJECT / "pingpang_assets/0718-move-160-80hz"),
        "--hit-motion-data", str(PROJECT / "pingpang_assets/0302_combined"),
        "--planner-version", "v1", "--action-mode", "lateral-y",
        "--runner-checkpoint", str(handoff / "checkpoints/rl_planner_model_199.pt"),
        "--runner-algorithm", "cbf-rl",
        "--v0-barrier-checkpoint", str(handoff / "checkpoints/safe_filter_v3.pt"),
        "--v0-barrier-candidate-radius", "0.75",
        "--v0-barrier-candidate-grid-points", "7",
        "--safety-mode", "hard-guard", "--steps", "64", "--shots", "24",
        "--low-level-steps", "8", "--warmup-steps", "150",
        "--shot-interval", "1.2", "--initial-hitter", "alternating",
        "--crossing-yield-probability", "0.75", "--ball-speed-scale", "1.25",
        "--ball-target-y-range-m", "-0.90", "0.90",
        "--seed", "10000", "--device", "cuda:0",
        "--output-dir", str(output),
    ] + sys.argv[1:]
    parsed = evaluator.parse_args(args)
    output = parsed.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    sources = [
        parsed.checkpoint, parsed.runner_checkpoint, parsed.v0_barrier_checkpoint,
        ROOT / "controller/source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp/distill_commands.py",
        ROOT / "assets/unitree_description/urdf/g1/main.urdf",
    ]
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    manifest = {
        "purpose": "local physics closed-loop diagnostic",
        "hardware_transport": False,
        "exact_training_patch_available": hashes[str(sources[3])] == "835a9c6930979acb0876fadf55602f49e6c2f181a258f2019733adb4dfe05706",
        "argv": args, "python": sys.executable, "sha256": hashes,
    }
    (output / "local_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("LOCAL_SIM_OUTPUT=" + str(output), flush=True)
    print("EXACT_TRAINING_PATCH=" + str(manifest["exact_training_patch_available"]), flush=True)
    os.environ.setdefault("PINGPANG_SKIP_APP_CLOSE", "1")
    code = evaluator.main(args)
    (output / "run_completion.json").write_text(json.dumps({
        "evaluator_return_code": code,
        "summary_exists": (output / "v0_safe_rl_rollout.json").is_file(),
        "completed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }, indent=2) + "\n")
    return code


if __name__ == "__main__":
    # The evaluator closes its environment and flushes its video/JSON. Isaac
    # Sim 5's remaining Kit plugin destructors can hang during interpreter
    # shutdown on this laptop; bypass only that final interpreter teardown.
    try:
        exit_code = main()
    except KeyboardInterrupt:
        exit_code = 130
    except SystemExit as error:
        exit_code = error.code if isinstance(error.code, int) else 1
    except BaseException:
        traceback.print_exc()
        exit_code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)
