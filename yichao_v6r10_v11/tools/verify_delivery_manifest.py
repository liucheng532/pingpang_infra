#!/usr/bin/env python3
"""Verify every file in DEPLOY_MANIFEST.json without modifying the tree."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    manifest = json.loads((ROOT / "DEPLOY_MANIFEST.json").read_text(encoding="utf-8"))
    if manifest.get("schema") != "v6r10-v11-delivery-manifest-v1":
        raise ValueError("wrong manifest schema")
    failures = []
    for item in manifest["files"]:
        path = ROOT / item["path"]
        if path.resolve().parent != ROOT.resolve() and ROOT.resolve() not in path.resolve().parents:
            failures.append(item["path"] + ":outside_root")
            continue
        if not path.is_file():
            failures.append(item["path"] + ":missing")
            continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        executable = bool(path.stat().st_mode & 0o111)
        if (
            digest != item["sha256"]
            or path.stat().st_size != item["bytes"]
            or executable != item["executable"]
        ):
            failures.append(item["path"] + ":identity")
    if failures:
        raise ValueError("delivery verification failed: " + ", ".join(failures))
    print(json.dumps({"verified": True, "file_count": len(manifest["files"]), "failures": []}))


if __name__ == "__main__":
    main()
