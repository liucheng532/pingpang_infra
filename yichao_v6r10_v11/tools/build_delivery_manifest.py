#!/usr/bin/env python3
"""Build the explicit immutable file list used for workstation sync."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "DEPLOY_MANIFEST.json"
TOP_FILES = {"README.md", "DEPLOY.md", "requirements.lock"}
TOP_DIRS = {"config", "models", "scripts", "src", "tests", "tools"}
EVIDENCE = "output/offline_replay_20260913/"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def included(path):
    relative = path.relative_to(ROOT).as_posix()
    if path == MANIFEST or "__pycache__" in path.parts or path.suffix == ".pyc":
        return False
    return relative in TOP_FILES or path.parts[len(ROOT.parts)] in TOP_DIRS or relative.startswith(EVIDENCE)


def main():
    files = []
    for path in sorted(item for item in ROOT.rglob("*") if item.is_file() and included(item)):
        files.append(
            {
                "path": path.relative_to(ROOT).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": digest(path),
                "executable": bool(path.stat().st_mode & 0o111),
            }
        )
    record = {
        "schema": "v6r10-v11-delivery-manifest-v1",
        "target": "/home/odl/codebase/yichao_v6r10_v11_adapter",
        "file_count": len(files),
        "total_bytes": sum(item["bytes"] for item in files),
        "files": files,
    }
    MANIFEST.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({key: record[key] for key in ("schema", "target", "file_count", "total_bytes")}, indent=2))


if __name__ == "__main__":
    main()

