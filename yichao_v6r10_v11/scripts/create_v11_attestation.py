#!/usr/bin/env python3
"""Read-only remote audit of the exact V11 processes and frozen assets."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from yichao_v6r10_v11 import PLANNER_IDENTITY
from yichao_v6r10_v11.attestation import ATTESTATION_SCHEMA, file_sha256


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True, choices=("shadow", "active"))
    parser.add_argument("--output", type=Path, default=ROOT / "output/current_v11_attestation.json")
    parser.add_argument("--config", type=Path, default=ROOT / "config/deployment.json")
    parser.add_argument("--require-running", action="store_true")
    return parser.parse_args()


def remote(host: str, key: str, command: str) -> str:
    completed = subprocess.run(
        [
            "ssh", "-i", key, "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
            "-o", "ConnectTimeout=5", f"unitree@{host}", command,
        ],
        text=True,
        capture_output=True,
        timeout=20,
    )
    if completed.returncode:
        raise RuntimeError(f"remote audit failed for {host}: {completed.stderr.strip()}")
    return completed.stdout


def process_record(host: str, key: str, pattern: str) -> dict:
    command = (
        "set -eu; pids=$(pgrep -f " + shlex.quote(pattern) + "); "
        "test $(printf '%s\\n' $pids | wc -l) -eq 1; pid=$pids; "
        "start=$(sed 's/.*) //' /proc/$pid/stat | awk '{print $20}'); "
        "argv=$(tr '\\0' ' ' </proc/$pid/cmdline); "
        "cwd=$(readlink -f /proc/$pid/cwd); exe=$(readlink -f /proc/$pid/exe); "
        "comm=$(cat /proc/$pid/comm); "
        "lcm=$(tr '\\0' '\\n' </proc/$pid/environ | "
        "sed -n 's/^LCM_DEFAULT_URL=//p'); "
        "printf '%s\\t%s\\t%s\\t%s\\t%s\\t%s\\t%s\\n' "
        "\"$pid\" \"$start\" \"$argv\" \"$cwd\" \"$exe\" \"$comm\" \"$lcm\""
    )
    fields = remote(host, key, command).rstrip("\n").split("\t", 6)
    if len(fields) != 7:
        raise RuntimeError(f"cannot parse process identity for {host}")
    return {
        "pid": int(fields[0]), "starttime": int(fields[1]), "argv": fields[2].strip(),
        "cwd": fields[3], "exe": fields[4], "comm": fields[5],
        "environment": {"LCM_DEFAULT_URL": fields[6]},
    }


def option(argv, name):
    if argv.count(name) != 1:
        raise RuntimeError(f"controller argv requires exactly one {name}")
    index = argv.index(name)
    if index + 1 >= len(argv):
        raise RuntimeError(f"controller argv has no value for {name}")
    return argv[index + 1]


def audit_robot(robot: str, config: dict, deployment: dict, mode: str, require_running: bool):
    host = config["ssh_host"]
    key = deployment["ssh_key"]
    root = deployment["controller_root"]
    assets = deployment["controller_assets"]
    absolute_assets = {root + "/" + path: digest for path, digest in assets.items()}
    absolute_assets.update(deployment["external_controller_assets"])
    quoted_paths = " ".join(shlex.quote(path) for path in absolute_assets)
    output = remote(
        host,
        key,
        "set -eu; cat /proc/sys/kernel/random/boot_id; sha256sum " + quoted_paths,
    ).splitlines()
    if len(output) != len(absolute_assets) + 1:
        raise RuntimeError(f"incomplete asset audit for {robot}")
    actual_absolute_assets = {}
    for line in output[1:]:
        digest, path = line.split(maxsplit=1)
        actual_absolute_assets[path] = digest
    if actual_absolute_assets != absolute_assets:
        raise RuntimeError(f"V11 asset hash mismatch for {robot}")

    sidecar_path = root + "/policy/v11_resume_i42500/student_v11_r2i299_commonhold_1666_resume42500.onnx.json"
    sidecar = json.loads(remote(host, key, "cat " + shlex.quote(sidecar_path)))
    contract_names = tuple(deployment["embedded_controller_contract"])
    embedded = {name: sidecar.get(name) for name in contract_names}
    if embedded != deployment["embedded_controller_contract"]:
        raise RuntimeError(f"V11 embedded teacher/motion contract mismatch for {robot}")

    item = {
        "host_label": config["host_label"],
        "ssh_host": host,
        "boot_id": output[0].strip(),
        "assets": {path: actual_absolute_assets[root + "/" + path] for path in assets},
        "external_assets": {
            path: actual_absolute_assets[path]
            for path in deployment["external_controller_assets"]
        },
        "embedded_controller_contract": embedded,
    }
    if require_running:
        item["g1_control"] = process_record(host, key, "[g]1_control")
        item["policy"] = process_record(host, key, "[p]ython3 .*scripts/deploy_policy.py")
        argv = item["policy"]["argv"].split()
        if ("--shadow" in argv) != (mode == "shadow"):
            raise RuntimeError(f"controller mode mismatch for {robot}")
        if argv.count("--external-planner") != 1:
            raise RuntimeError(f"controller argv contract mismatch for {robot}")
        if option(argv, "--robot-id") != robot:
            raise RuntimeError(f"controller identity mismatch for {robot}")
        expected_policy = root + "/policy/v11_resume_i42500/student_v11_r2i299_commonhold_1666_resume42500.onnx"
        expected = {
            "--policy": expected_policy,
            "--hit-policy-source": "teacher",
            "--hit-arm7-residual-mode": "off",
            "--hit-arm7-residual-scale": "1.0",
            "--hit-arm7-residual-family": "v11-teacher",
            "--racket-hand": config["racket_hand"],
        }
        for name, value in expected.items():
            if option(argv, name) != value:
                raise RuntimeError(f"controller argv mismatch for {robot}: {name}")
        if ("--mirror-left-hand" in argv) != config["mirror_left_hand"]:
            raise RuntimeError(f"controller mirror mismatch for {robot}")
        if "--stationary-hit-test" in argv:
            raise RuntimeError(f"controller profile is not normal for {robot}")
        if item["policy"]["cwd"] != root + "/g1_gym_deploy":
            raise RuntimeError(f"controller cwd mismatch for {robot}")
        if item["policy"]["comm"] not in {"python3", "python"}:
            raise RuntimeError(f"controller executable mismatch for {robot}")
        native = item["g1_control"]
        if native["comm"] != "g1_control" or native["exe"] != root + "/unitree_sdk2/build/bin/g1_control":
            raise RuntimeError(f"native controller identity mismatch for {robot}")
        if shlex.split(native["argv"]) != [
            root + "/unitree_sdk2/build/bin/g1_control", "eth0"
        ]:
            raise RuntimeError(f"native controller argv mismatch for {robot}")
        expected_lcm = (
            f"udpm://239.255.76.{config['host_label']}:7667?ttl=0"
        )
        if native.get("environment", {}).get("LCM_DEFAULT_URL") != expected_lcm:
            raise RuntimeError(f"native controller LCM identity mismatch for {robot}")
    return item


def main() -> int:
    args = arguments()
    deployment = json.loads(args.config.read_text(encoding="utf-8"))
    launcher = Path(deployment["original_v11_launcher"])
    launcher_sha256 = file_sha256(launcher)
    if launcher_sha256 != deployment["original_v11_launcher_sha256"]:
        raise RuntimeError("original V11 launcher SHA256 mismatch")
    record = {
        "schema": ATTESTATION_SCHEMA,
        "attestation_id": "v11_" + time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8],
        "planner_identity": PLANNER_IDENTITY,
        "created_at": time.time(),
        "controller_mode": args.mode,
        "profile": "normal",
        "residual_mode": "off",
        "residual_scale": 1.0,
        "residual_family": "v11-teacher",
        "launcher": str(launcher),
        "launcher_sha256": launcher_sha256,
        "live_processes_verified": bool(args.require_running),
        "robots": {},
    }
    for robot, config in deployment["robots"].items():
        record["robots"][robot] = audit_robot(
            robot, config, deployment, args.mode, args.require_running
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, args.output)
    print(json.dumps(record, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
