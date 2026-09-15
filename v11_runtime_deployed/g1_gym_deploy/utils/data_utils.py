from __future__ import annotations

import os
import re
from collections.abc import Sequence
import torch
import numpy as np


class MotionLoader:
    def __init__(self, motion_file: str, device: str = "cpu"):
        assert os.path.isfile(motion_file), f"Invalid file path: {motion_file}"
        data = np.load(motion_file)
        self.fps = float(data["fps"]) if "fps" in data else 30.0
        self.joint_pos = torch.tensor(data["joint_pos"], dtype=torch.float32, device=device)
        self.joint_vel = torch.tensor(data["joint_vel"], dtype=torch.float32, device=device)
        self.body_pos_w = torch.tensor(data["body_pos_w"], dtype=torch.float32, device=device)
        self.body_quat_w = torch.tensor(data["body_quat_w"], dtype=torch.float32, device=device)
        self.time_step_total = int(self.joint_pos.shape[0])


class MotionCommand:
    """
    Load multiple motion npz files indexed by localdata/<data_path>/dataindex.csv,
    pad to same length, and stack.

    Outputs:
        self.joint_pos_all: (M, T_max, J)
        self.joint_vel_all: (M, T_max, J)
        self.motion_targets: (M, 3)
        self.backhand_flags: (M,) bool
    """

    def __init__(self, multirange_root: str, device: str = "cpu"):
        self.device = device
        # Normalize the provided root so callers can pass "1220" or "1220/" or a full path.
        multirange_root = os.path.normpath(multirange_root)
        # basename is used when constructing folder names like "1220-00:v0"
        root_name = os.path.basename(multirange_root)

        index_csv = os.path.join(multirange_root, "dataindex.csv")
        if not os.path.isfile(index_csv):
            raise FileNotFoundError(f"dataindex.csv not found: {index_csv}")

        motion_files: list[str] = []
        motion_targets: list[list[float]] = []
        backhand_flags: list[bool] = []

        with open(index_csv, "r") as f:
            lines = f.readlines()
        if not lines:
            raise RuntimeError(f"Empty CSV: {index_csv}")

        # Parse rows like:
        # index   backhand    target
        # 00          1        0.6442, 0.5193, 1.1631
        for line_number, line in enumerate(lines[1:], start=2):
            line = line.strip()
            if not line:
                continue

            m = re.match(r"^\s*(\S+)\s+([01])\s+(.+)$", line)
            if not m:
                raise ValueError(f"Invalid hit manifest row {line_number}: {line!r}")

            index_str, backhand_str, target_str = m.group(1), m.group(2), m.group(3)

            # Resolve motion folder name. The CSV may contain either
            #  - an index like "00" (we expect folder names like "1220-00:v0" under multirange_root)
            #  - or a full folder name already including the root prefix like "1220-00:v1"
            # Use root_name (basename) so callers can pass multirange_root with or without trailing slash
            folder = index_str if index_str.endswith(":v0") else f"{index_str}:v0"
            if not folder.startswith(f"{root_name}-"):
                folder = f"{root_name}-{folder}"

            npz_path = os.path.join(multirange_root, folder, "motion.npz")
            if not os.path.isfile(npz_path):
                raise FileNotFoundError(f"Hit motion from row {line_number} not found: {npz_path}")

            # Parse target triple
            target_str = target_str.strip().replace("[", "").replace("]", "")
            try:
                tx, ty, tz = [float(s) for s in target_str.split(",")]
            except Exception:
                parts = target_str.split()
                if len(parts) < 3:
                    raise ValueError(f"Invalid hit target on row {line_number}: {line!r}")
                tx = float(parts[0].rstrip(","))
                ty = float(parts[1].rstrip(","))
                tz = float(parts[2].rstrip(","))

            motion_files.append(npz_path)
            motion_targets.append([tx, ty, tz])
            backhand_flags.append(bool(int(backhand_str)))
        # Load motions
        self.motions: list[MotionLoader] = [MotionLoader(p, device=self.device) for p in motion_files]
        self.motion_files = tuple(motion_files)
        self.num_motions: int = len(self.motions)
        if self.num_motions == 0:
            raise RuntimeError(f"No valid motion files found under: {multirange_root}")

        # Pad each motion to the same length (repeat last step), then stack
        T_max = max(m.time_step_total for m in self.motions)

        def pad_last(x: torch.Tensor, T_max_: int) -> torch.Tensor:
            T_i = x.shape[0]
            if T_i == T_max_:
                return x
            pad_len = T_max_ - T_i
            return torch.cat([x, x[-1:].expand(pad_len, *x.shape[1:])], dim=0)

        joint_pos_padded = [pad_last(m.joint_pos, T_max) for m in self.motions]  # (T_max, J)
        joint_vel_padded = [pad_last(m.joint_vel, T_max) for m in self.motions]  # (T_max, J)

        body_pos_padded  = [pad_last(m.body_pos_w, T_max) for m in self.motions]              # (T_max, B, 3)
        body_quat_padded = [pad_last(m.body_quat_w, T_max) for m in self.motions]             # (T_max, B, 4)
        motion_anchor_index = 9
        # Stacked caches for fast, loop-free gather:
        self.joint_pos_all = torch.stack(joint_pos_padded, dim=0)  # (M, T_max, J)
        self.joint_vel_all = torch.stack(joint_vel_padded, dim=0)  # (M, T_max, J)

        self.body_pos_all = torch.stack(body_pos_padded, dim=0)  # (M, T_max, B, 3)
        self.body_quat_all = torch.stack(body_quat_padded, dim=0)  # (M, T_max, B, 4)
        self.torso_pos_all = self.body_pos_all[:, :, motion_anchor_index, :]  # (M, T_max,  3)
        self.torso_quat_all = self.body_quat_all[:, :, motion_anchor_index, :]  # (M, T_max, 4)
        # Also keep targets and flags aligned with motion index
        self.motion_targets = torch.tensor(motion_targets, dtype=torch.float32, device=self.device)  # (M, 3)
        self.backhand_flags = torch.tensor(backhand_flags, dtype=torch.bool, device=self.device)    # (M,)


class MoveMotionBank:
    """Load the scalar-target 0718 move-motion manifest into NumPy arrays."""

    DISABLED_INDICES = frozenset({41, 47})

    def __init__(
        self,
        multirange_root: str,
        expected_count: int | None = 155,
        expected_fps: float = 80.0,
        excluded_source_ids: Sequence[int] | None = None,
        expected_active_count: int | None = None,
    ):
        self.root = os.path.normpath(multirange_root)
        index_csv = os.path.join(self.root, "dataindex.csv")
        if not os.path.isfile(index_csv):
            raise FileNotFoundError(f"dataindex.csv not found: {index_csv}")

        root_name = os.path.basename(self.root)
        indices: list[str] = []
        source_ids: list[int] = []
        labels: list[int] = []
        targets_y: list[float] = []
        joint_pos: list[np.ndarray] = []
        joint_vel: list[np.ndarray] = []
        fps: list[float] = []

        with open(index_csv, "r", encoding="utf-8") as stream:
            lines = stream.readlines()
        if not lines:
            raise RuntimeError(f"Empty CSV: {index_csv}")

        for line_number, line in enumerate(lines[1:], start=2):
            fields = line.strip().split()
            if not fields:
                continue
            if len(fields) != 3:
                raise ValueError(f"Invalid move manifest row {line_number}: {line.rstrip()}")
            index, label_text, target_text = fields
            source_match = re.search(r"(?:^|-)(\d+)(?::v0)?$", index)
            if source_match is None:
                raise ValueError(f"Move source ID is missing on row {line_number}: {index!r}")
            source_id = int(source_match.group(1))
            try:
                label = int(label_text)
                target_y = float(target_text)
            except ValueError as exc:
                raise ValueError(f"Invalid move manifest row {line_number}: {line.rstrip()}") from exc
            if label not in (0, 1):
                raise ValueError(f"Move label must be 0 or 1 on row {line_number}, got {label}.")

            folder = index if index.startswith(f"{root_name}-") else f"{root_name}-{index}:v0"
            if ":" not in folder:
                folder = f"{folder}:v0"
            motion_path = os.path.join(self.root, folder, "motion.npz")
            if not os.path.isfile(motion_path):
                raise FileNotFoundError(f"Move motion file not found: {motion_path}")

            with np.load(motion_path) as data:
                if "joint_pos" not in data or "joint_vel" not in data:
                    raise ValueError(f"Move motion lacks joint_pos/joint_vel: {motion_path}")
                q = np.asarray(data["joint_pos"], dtype=np.float32)
                qd = np.asarray(data["joint_vel"], dtype=np.float32)
                motion_fps = float(np.asarray(data["fps"]).reshape(-1)[0]) if "fps" in data else 30.0
            if q.ndim != 2 or q.shape[1] != 29:
                raise ValueError(f"Move joint_pos must have shape [T, 29], got {q.shape}: {motion_path}")
            if qd.shape != q.shape:
                raise ValueError(f"Move joint_vel shape {qd.shape} does not match joint_pos {q.shape}: {motion_path}")
            if q.shape[0] == 0:
                raise ValueError(f"Move motion contains no frames: {motion_path}")
            if not np.isclose(motion_fps, expected_fps):
                raise ValueError(f"Move motion fps must be {expected_fps}, got {motion_fps}: {motion_path}")

            indices.append(index)
            source_ids.append(source_id)
            labels.append(label)
            targets_y.append(target_y)
            joint_pos.append(q)
            joint_vel.append(qd)
            fps.append(motion_fps)

        if expected_count is not None and len(indices) != expected_count:
            raise ValueError(f"Expected {expected_count} move motions, loaded {len(indices)} from {self.root}.")
        if not indices:
            raise RuntimeError(f"No move motions found under: {self.root}")
        if set(labels) != {0, 1}:
            raise ValueError("Move bank must contain both label-0 and label-1 motions.")

        self.indices = tuple(indices)
        self.source_ids = np.asarray(source_ids, dtype=np.int64)
        self.excluded_source_ids = (
            None
            if excluded_source_ids is None
            else frozenset(int(value) for value in excluded_source_ids)
        )
        self.active_mask = (
            np.ones(len(indices), dtype=bool)
            if self.excluded_source_ids is None
            else ~np.isin(self.source_ids, tuple(self.excluded_source_ids))
        )
        self.active_count = int(self.active_mask.sum())
        if expected_active_count is not None and self.active_count != expected_active_count:
            raise ValueError(
                f"Expected {expected_active_count} active move motions, got {self.active_count}."
            )
        self.labels = np.asarray(labels, dtype=np.int64)
        self.targets_y = np.asarray(targets_y, dtype=np.float32)
        self.joint_pos = tuple(joint_pos)
        self.joint_vel = tuple(joint_vel)
        self.fps = np.asarray(fps, dtype=np.float32)
        self.lengths = np.asarray([q.shape[0] for q in joint_pos], dtype=np.int64)
        self.num_motions = len(indices)

    def nearest_index(self, target_displacement_y: float, label: int) -> int:
        candidates = np.flatnonzero(self.labels == int(label))
        if self.excluded_source_ids is None:
            candidates = candidates[~np.isin(candidates, tuple(self.DISABLED_INDICES))]
        else:
            candidates = candidates[self.active_mask[candidates]]
        if candidates.size == 0:
            raise ValueError(f"No move motions with label {label}.")
        errors = np.abs(self.targets_y[candidates] - float(target_displacement_y))
        return int(candidates[int(np.argmin(errors))])

    def frame(self, motion_index: int, frame_index: int) -> tuple[np.ndarray, np.ndarray, int]:
        motion_index = int(motion_index)
        last = int(self.lengths[motion_index] - 1)
        step = int(np.clip(frame_index, 0, last))
        return self.joint_pos[motion_index][step], self.joint_vel[motion_index][step], step
