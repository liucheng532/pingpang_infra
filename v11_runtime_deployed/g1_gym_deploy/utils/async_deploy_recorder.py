from __future__ import annotations

import json
import multiprocessing as mp
import queue
import re
import time
from datetime import datetime
from pathlib import Path

import numpy as np


RECORD_RATE_HZ = 50
DEFAULT_QUEUE_SIZE = 256


RECORD_DTYPE = np.dtype(
    [
        ("sequence", "<u8"),
        ("wall_time_ns", "<u8"),
        ("monotonic_time_ns", "<u8"),
        ("loop_period_s", "<f4"),
        ("inference_s", "<f4"),
        ("command_seq", "<u8"),
        ("command_publish_monotonic_raw_ns", "<u8"),
        ("trace_command_seq", "<u8"),
        ("command_lcm_receive_monotonic_raw_ns", "<u8"),
        ("command_control_monotonic_raw_ns", "<u8"),
        ("command_dds_write_monotonic_raw_ns", "<u8"),
        ("body_source_monotonic_raw_ns", "<u8"),
        ("body_receive_monotonic_raw_ns", "<u8"),
        ("imu_source_monotonic_raw_ns", "<u8"),
        ("imu_receive_monotonic_raw_ns", "<u8"),
        ("enqueue_s", "<f4"),
        ("queue_depth", "<u2"),
        ("dropped_frames", "<u4"),
        ("shadow", "u1"),
        ("mirror_left_hand", "u1"),
        ("racket_hand", "u1"),
        ("right_side_canonicalization", "u1"),
        ("arm_transform_applied", "u1"),
        ("table_right_move_mode", "u1"),
        ("observation_transform_applied", "u1"),
        ("action_transform_applied", "u1"),
        ("phase_id", "i1"),
        ("phase", "<f4", (6,)),
        ("outward_hold_elapsed_s", "<f4"),
        ("outward_hold_duration_s", "<f4"),
        ("outward_hold_entry_reason", "S16"),
        ("raw_tts", "<f4"),
        ("corrected_tts", "<f4"),
        ("torso_pos", "<f4", (3,)),
        ("torso_quat_wxyz", "<f4", (4,)),
        ("pelvis_pos", "<f4", (3,)),
        ("pelvis_quat_wxyz", "<f4", (4,)),
        ("torso_minus_pelvis", "<f4", (3,)),
        ("waist_joint_pos", "<f4", (3,)),
        ("torso_source_stamp_s", "<f8"),
        ("torso_receive_monotonic_ns", "<u8"),
        ("target_source_stamp_s", "<f8"),
        ("target_receive_monotonic_ns", "<u8"),
        ("velocity_source_stamp_s", "<f8"),
        ("velocity_receive_monotonic_ns", "<u8"),
        ("tts_source_stamp_s", "<f8"),
        ("tts_receive_monotonic_ns", "<u8"),
        ("joint_pos", "<f4", (29,)),
        ("joint_vel", "<f4", (29,)),
        ("body_rpy", "<f4", (3,)),
        ("body_quat", "<f4", (4,)),
        ("body_angular_vel", "<f4", (3,)),
        ("body_receive_monotonic_ns", "<u8"),
        ("imu_receive_monotonic_ns", "<u8"),
        ("obs", "<f4", (172,)),
        ("obs_history", "<f4", (1666,)),
        ("policy_obs_history", "<f4", (1666,)),
        ("teacher_obs_history", "<f4", (1670,)),
        ("teacher_policy_obs_history", "<f4", (1670,)),
        ("active_policy_source", "u1"),
        ("control_policy_sequence", "<u8"),
        ("policy_source_switch", "u1"),
        ("action_jump_abs_max", "<f4"),
        ("action_jump_rms", "<f4"),
        ("q_des_jump_abs_max", "<f4"),
        ("q_des_jump_rms", "<f4"),
        ("hit_residual_mode", "u1"),
        ("hit_residual_backend", "u1"),
        ("hit_residual_ball_valid", "u1"),
        ("hit_residual_ball_age_ms", "<f4"),
        ("hit_residual_ball_reason", "S24"),
        ("hit_residual_phase_gate", "<f4"),
        ("hit_teacher_canonical_action", "<f4", (29,)),
        ("hit_combined_canonical_action", "<f4", (29,)),
        ("hit_output_canonical_action", "<f4", (29,)),
        ("hit_teacher_physical_action", "<f4", (29,)),
        ("hit_combined_physical_action", "<f4", (29,)),
        ("hit_output_physical_action", "<f4", (29,)),
        ("hit_residual_applied_physical", "<f4", (29,)),
        ("hit_residual_abs_max", "<f4"),
        ("hit_residual_rms", "<f4"),
        ("hit_residual_inference_ms", "<f4"),
        ("external_hit_command_mode", "u1"),
        ("planner_command_sequence", "<u8"),
        ("planner_commit_token", "S64"),
        ("planner_command_age_ms", "<f4"),
        ("planner_raw_tts", "<f4"),
        ("planner_streamed_tts", "<f4"),
        ("planner_stream_update", "u1"),
        ("planner_prediction_source_timestamp_s", "<f8"),
        ("planner_prediction_age_ms", "<f4"),
        ("raw_ball_valid", "u1"),
        ("raw_ball_reason", "S24"),
        ("raw_ball_source_time_s", "<f8"),
        ("raw_ball_frame", "<i8"),
        ("raw_ball_rigid_body_id", "<i4"),
        ("raw_ball_position", "<f4", (3,)),
        ("raw_ball_velocity", "<f4", (3,)),
        ("raw_ball_age_ms", "<f4"),
        ("raw_ball_receive_monotonic_ns", "<u8"),
        ("raw_ball_sample_count", "<u2"),
        ("reference_joint_pos", "<f4", (29,)),
        ("reference_joint_vel", "<f4", (29,)),
        ("canonical_action_lab", "<f4", (29,)),
        ("action_lab", "<f4", (29,)),
        ("action_gym", "<f4", (29,)),
        ("q_des", "<f4", (29,)),
        ("target_base", "<f4", (2,)),
        ("physical_target_base", "<f4", (2,)),
        ("policy_target_base", "<f4", (2,)),
        ("episode_start_x_override", "<f4"),
        ("episode_start_x", "<f4"),
        ("target_base_x", "<f4"),
        ("hit_motion_index", "<i4"),
        ("move_motion_index", "<i4"),
        ("move_motion_source_label", "i1"),
        ("move_semantic_phase", "i1"),
        ("reference_step", "<i4"),
        ("home_y", "<f4"),
        ("outward_target_y", "<f4"),
        ("monitor_receive_wall_time_ns", "<u8"),
        ("monitor_tracker_sensor", "<i4"),
        ("monitor_tracker_callback_count", "<u8"),
        ("monitor_ball_callback_count", "<u8"),
        ("monitor_tracker_age_ms", "<f4"),
        ("monitor_tracker_pos_world", "<f4", (3,)),
        ("monitor_planner_state", "S16"),
        ("monitor_cross_detected", "u1"),
        ("monitor_planning_armed", "u1"),
        ("monitor_tts", "<f4"),
    ]
)


_TRACKER_STATUS_RE = re.compile(
    r"age_ms=(?P<age>[-+0-9.eE]+).*?pos_world=\[(?P<position>[^\]]+)\]"
)


def _json_write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_planner_monitor_payload(payload: dict) -> dict:
    mocap = payload.get("mocap", {})
    planner = payload.get("planner", {})
    tracker_sensor = int(mocap.get("tracker_sensor", -1))
    ball_sensor = int(mocap.get("ball_sensor", -1))
    callback_stats = mocap.get("callback_stats", {})
    tracker_stats = callback_stats.get(str(tracker_sensor), callback_stats.get(tracker_sensor, {}))
    ball_stats = callback_stats.get(str(ball_sensor), callback_stats.get(ball_sensor, {}))
    tracker_age = np.nan
    tracker_position = np.full(3, np.nan, dtype=np.float32)
    match = _TRACKER_STATUS_RE.search(str(mocap.get("tracker_status", "")))
    if match:
        tracker_age = float(match.group("age"))
        values = np.fromstring(match.group("position"), sep=" ", dtype=np.float32)
        if values.shape == (3,):
            tracker_position = values
    return {
        "monitor_receive_wall_time_ns": time.time_ns(),
        "monitor_tracker_sensor": tracker_sensor,
        "monitor_tracker_callback_count": int(tracker_stats.get("count", 0)),
        "monitor_ball_callback_count": int(ball_stats.get("count", 0)),
        "monitor_tracker_age_ms": tracker_age,
        "monitor_tracker_pos_world": tracker_position,
        "monitor_planner_state": str(planner.get("state", ""))[:16].encode("utf-8"),
        "monitor_cross_detected": int(bool(planner.get("cross_detected", False))),
        "monitor_planning_armed": int(bool(planner.get("planning_armed", False))),
        "monitor_tts": float(planner.get("time_to_strike", np.nan)),
    }


def _planner_monitor_subscriber(latest: list[dict]) -> object | None:
    try:
        import rospy
        from std_msgs.msg import String
    except Exception:
        return None

    try:
        if not rospy.core.is_initialized():
            rospy.init_node("doubles_deploy_recorder", anonymous=True, disable_signals=True)
    except Exception:
        return None

    def callback(msg: String) -> None:
        try:
            latest[0] = parse_planner_monitor_payload(json.loads(msg.data))
        except Exception:
            return

    return rospy.Subscriber("/table_tennis_planner_monitor", String, callback, queue_size=1)


def _append_event(stream, event: str, **payload) -> None:
    record = {"wall_time_ns": time.time_ns(), "event": event, **payload}
    stream.write(json.dumps(record, separators=(",", ":")) + "\n")
    stream.flush()


def _save_ring_clip(
    frames,
    session_path: Path,
    capacity: int,
    start_sequence: int,
    end_sequence: int,
    trigger_sequence: int,
    partial: bool = False,
) -> Path | None:
    if end_sequence < start_sequence:
        return None
    sequences = np.arange(start_sequence, end_sequence + 1, dtype=np.uint64)
    clip = np.asarray(frames[sequences % capacity]).copy()
    if not np.array_equal(clip["sequence"], sequences):
        return None
    suffix = "_partial" if partial else ""
    path = session_path / f"hit_{trigger_sequence:08d}{suffix}.npy"
    np.save(path, clip, allow_pickle=False)
    return path


def _writer_main(
    record_queue,
    session_dir: str,
    capacity: int,
    metadata: dict,
    capacity_event,
    subscribe_planner_monitor: bool,
    mode: str,
    pre_trigger_frames: int,
    post_trigger_frames: int,
) -> None:
    session_path = Path(session_dir)
    session_path.mkdir(parents=True, exist_ok=False)
    frames_path = session_path / "frames.npy"
    metadata_path = session_path / "metadata.json"
    events_path = session_path / "events.jsonl"
    metadata = dict(metadata)
    metadata.update(
        {
            "capacity_frames": int(capacity),
            "record_dtype": RECORD_DTYPE.descr,
            "record_rate_hz": RECORD_RATE_HZ,
            "record_mode": mode,
            "pre_trigger_frames": int(pre_trigger_frames),
            "post_trigger_frames": int(post_trigger_frames),
            "status": "recording",
            "valid_frames": 0,
            "total_frames": 0,
            "write_index": 0,
        }
    )
    _json_write(metadata_path, metadata)
    frames = np.lib.format.open_memmap(frames_path, mode="w+", dtype=RECORD_DTYPE, shape=(capacity,))
    latest_monitor = [{}]
    monitor_subscriber = _planner_monitor_subscriber(latest_monitor) if subscribe_planner_monitor else None
    circular = mode in {"circular", "triggered"}
    total_frames = 0
    valid_frames = 0
    last_phase = None
    last_policy_source = None
    last_action_lab = None
    last_q_des = None
    last_flush = time.monotonic()
    pending_clips = []

    with events_path.open("a", encoding="utf-8", buffering=1) as events:
        _append_event(events, "recorder_started", capacity_frames=capacity)
        try:
            while True:
                try:
                    snapshot = record_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if snapshot is None:
                    break
                if not circular and total_frames >= capacity:
                    if not capacity_event.is_set():
                        capacity_event.set()
                        _append_event(events, "capacity_reached", valid_frames=valid_frames)
                    continue

                record = np.zeros((), dtype=RECORD_DTYPE)
                record["monitor_tracker_age_ms"] = np.nan
                record["monitor_tracker_pos_world"] = np.nan
                record["monitor_tts"] = np.nan
                record["hit_residual_ball_age_ms"] = np.nan
                record["planner_command_age_ms"] = np.nan
                record["planner_raw_tts"] = np.nan
                record["planner_streamed_tts"] = np.nan
                record["planner_prediction_source_timestamp_s"] = np.nan
                record["planner_prediction_age_ms"] = np.nan
                record["episode_start_x_override"] = np.nan
                record["raw_ball_source_time_s"] = np.nan
                record["raw_ball_frame"] = -1
                record["raw_ball_rigid_body_id"] = -1
                record["raw_ball_position"] = np.nan
                record["raw_ball_velocity"] = np.nan
                record["raw_ball_age_ms"] = np.nan
                for name, value in snapshot.items():
                    if name in RECORD_DTYPE.fields:
                        record[name] = value
                for name, value in latest_monitor[0].items():
                    if name in RECORD_DTYPE.fields:
                        record[name] = value
                policy_source = int(record["active_policy_source"])
                policy_source_switch = (
                    last_policy_source is not None and policy_source != last_policy_source
                )
                if policy_source_switch:
                    action_delta = np.asarray(record["action_lab"]) - last_action_lab
                    q_des_delta = np.asarray(record["q_des"]) - last_q_des
                    record["policy_source_switch"] = 1
                    record["action_jump_abs_max"] = np.max(np.abs(action_delta))
                    record["action_jump_rms"] = np.sqrt(np.mean(np.square(action_delta)))
                    record["q_des_jump_abs_max"] = np.max(np.abs(q_des_delta))
                    record["q_des_jump_rms"] = np.sqrt(np.mean(np.square(q_des_delta)))
                write_index = total_frames % capacity if circular else total_frames
                frames[write_index] = record
                total_frames += 1
                valid_frames = min(total_frames, capacity)

                phase = int(record["phase_id"])
                if mode == "triggered" and phase == 0 and last_phase != 0:
                    trigger_sequence = int(record["sequence"])
                    pending_clips.append(
                        {
                            "trigger": trigger_sequence,
                            "start": max(0, trigger_sequence - pre_trigger_frames),
                            "end": trigger_sequence + post_trigger_frames,
                        }
                    )
                    _append_event(
                        events,
                        "hit_triggered",
                        trigger_sequence=trigger_sequence,
                    )
                if last_phase is not None and phase != last_phase:
                    _append_event(
                        events,
                        "phase_transition",
                        sequence=int(record["sequence"]),
                        previous_phase=last_phase,
                        phase=phase,
                    )
                last_phase = phase
                if policy_source_switch:
                    _append_event(
                        events,
                        "policy_source_transition",
                        sequence=int(record["sequence"]),
                        control_policy_sequence=int(record["control_policy_sequence"]),
                        previous_source=int(last_policy_source),
                        source=policy_source,
                        action_jump_abs_max=float(record["action_jump_abs_max"]),
                        action_jump_rms=float(record["action_jump_rms"]),
                        q_des_jump_abs_max=float(record["q_des_jump_abs_max"]),
                        q_des_jump_rms=float(record["q_des_jump_rms"]),
                    )
                last_policy_source = policy_source
                last_action_lab = np.asarray(record["action_lab"], dtype=np.float32).copy()
                last_q_des = np.asarray(record["q_des"], dtype=np.float32).copy()

                current_sequence = int(record["sequence"])
                completed = []
                for clip in pending_clips:
                    if current_sequence < clip["end"]:
                        continue
                    path = _save_ring_clip(
                        frames,
                        session_path,
                        capacity,
                        clip["start"],
                        clip["end"],
                        clip["trigger"],
                    )
                    _append_event(
                        events,
                        "hit_clip_saved" if path is not None else "hit_clip_overwritten",
                        trigger_sequence=clip["trigger"],
                        path=None if path is None else path.name,
                    )
                    completed.append(clip)
                for clip in completed:
                    pending_clips.remove(clip)

                if time.monotonic() - last_flush >= 1.0:
                    frames.flush()
                    metadata["valid_frames"] = int(valid_frames)
                    metadata["total_frames"] = int(total_frames)
                    metadata["write_index"] = int(total_frames % capacity)
                    _json_write(metadata_path, metadata)
                    last_flush = time.monotonic()
        except Exception as exc:
            metadata["status"] = "writer_error"
            metadata["writer_error"] = repr(exc)
            _append_event(events, "writer_error", error=repr(exc))
        finally:
            last_sequence = total_frames - 1
            for clip in pending_clips:
                path = _save_ring_clip(
                    frames,
                    session_path,
                    capacity,
                    clip["start"],
                    min(clip["end"], last_sequence),
                    clip["trigger"],
                    partial=True,
                )
                _append_event(
                    events,
                    "hit_clip_partial",
                    trigger_sequence=clip["trigger"],
                    path=None if path is None else path.name,
                )
            frames.flush()
            metadata["valid_frames"] = int(valid_frames)
            metadata["total_frames"] = int(total_frames)
            metadata["write_index"] = int(total_frames % capacity)
            if metadata.get("status") == "recording":
                metadata["status"] = "complete"
            metadata["monitor_subscription_active"] = monitor_subscriber is not None
            _json_write(metadata_path, metadata)
            _append_event(events, "recorder_stopped", valid_frames=valid_frames, status=metadata["status"])


class AsyncDeployRecorder:
    def __init__(
        self,
        output_root: str | Path,
        duration_s: float = 600.0,
        metadata: dict | None = None,
        queue_size: int = DEFAULT_QUEUE_SIZE,
        subscribe_planner_monitor: bool = True,
        mode: str = "bounded",
        pre_trigger_s: float = 2.0,
        post_trigger_s: float = 3.0,
    ):
        duration_s = float(duration_s)
        if duration_s <= 0.0:
            raise ValueError(f"Recorder duration must be positive, got {duration_s}.")
        output_root = Path(output_root).expanduser().resolve()
        output_root.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.session_dir = output_root / f"session_{timestamp}"
        self.capacity = int(round(duration_s * RECORD_RATE_HZ))
        if self.capacity <= 0:
            raise ValueError("Recorder duration must contain at least one frame.")
        mode = str(mode)
        if mode not in {"bounded", "circular", "triggered"}:
            raise ValueError(f"Unknown recorder mode: {mode!r}.")
        pre_trigger_frames = int(round(float(pre_trigger_s) * RECORD_RATE_HZ))
        post_trigger_frames = int(round(float(post_trigger_s) * RECORD_RATE_HZ))
        if pre_trigger_frames < 0 or post_trigger_frames < 0:
            raise ValueError("Recorder trigger windows must not be negative.")
        if mode == "triggered" and pre_trigger_frames + post_trigger_frames + 1 > self.capacity:
            raise ValueError("Triggered recorder window must fit inside the circular buffer.")
        self.mode = mode
        self._ctx = mp.get_context("spawn")
        self._queue = self._ctx.Queue(maxsize=int(queue_size))
        self._capacity_event = self._ctx.Event()
        self._process = self._ctx.Process(
            target=_writer_main,
            args=(
                self._queue,
                str(self.session_dir),
                self.capacity,
                metadata or {},
                self._capacity_event,
                bool(subscribe_planner_monitor),
                mode,
                pre_trigger_frames,
                post_trigger_frames,
            ),
            name="doubles-deploy-recorder",
            daemon=True,
        )
        self._process.start()
        self.sequence = 0
        self.dropped_frames = 0
        self.last_enqueue_s = 0.0
        self.failed = False

    def enqueue(self, snapshot: dict) -> bool:
        if self.failed or self._capacity_event.is_set():
            return False
        if not self._process.is_alive():
            self.failed = True
            return False
        snapshot["sequence"] = self.sequence
        snapshot["dropped_frames"] = self.dropped_frames
        snapshot["enqueue_s"] = self.last_enqueue_s
        try:
            snapshot["queue_depth"] = max(0, self._queue.qsize())
        except (NotImplementedError, AttributeError):
            snapshot["queue_depth"] = 0
        started = time.perf_counter_ns()
        try:
            self._queue.put_nowait(snapshot)
        except queue.Full:
            self.dropped_frames += 1
            return False
        finally:
            self.last_enqueue_s = (time.perf_counter_ns() - started) * 1.0e-9
        self.sequence += 1
        return True

    def close(self, timeout_s: float = 2.0) -> None:
        if self._process is None:
            return
        try:
            self._queue.put(None, timeout=0.5)
        except queue.Full:
            pass
        self._process.join(timeout=max(0.0, float(timeout_s)))
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=0.5)
        self._queue.close()
        self._process = None
