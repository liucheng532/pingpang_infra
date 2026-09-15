from __future__ import annotations

from collections import deque
from numbers import Integral
import threading
import time

import numpy as np


RAW_BALL_TOPIC = "/residual/mocap_ball_state"
BALL_RIGID_BODY_ID = 30300


class RawBallStateBuffer:
    """Thread-safe latest raw mocap ball sample with a short velocity fit."""

    def __init__(self, max_age_s=0.06, velocity_window_s=0.06, max_samples=32,
                 expected_rigid_body_id=BALL_RIGID_BODY_ID):
        if (isinstance(expected_rigid_body_id, bool)
                or not isinstance(expected_rigid_body_id, Integral)
                or expected_rigid_body_id < 0):
            raise ValueError("expected_rigid_body_id must be a non-negative integer")
        self.expected_rigid_body_id = int(expected_rigid_body_id)
        self.max_age_s = float(max_age_s)
        self.velocity_window_s = float(velocity_window_s)
        self._lock = threading.Lock()
        self._history = deque(maxlen=int(max_samples))
        self._latest = None
        self._last_received_rigid_body_id = -1
        self._reason = "no_data"

    def ingest_payload(self, values, receive_monotonic_ns=None):
        try:
            if len(values) != 6:
                raise ValueError
            source_time = float(values[0])
            frame_value = float(values[1])
            rigid_value = float(values[2])
            position = np.asarray(values[3:6], dtype=np.float64)
        except (TypeError, ValueError):
            with self._lock:
                self._reason = "malformed"
            return False
        if not np.isfinite((source_time, frame_value, rigid_value)).all() or not np.isfinite(position).all():
            with self._lock:
                self._reason = "nonfinite"
            return False
        frame = int(round(frame_value))
        rigid_body_id = int(round(rigid_value))
        if source_time <= 0.0 or abs(frame_value - frame) > 1.0e-6:
            with self._lock:
                self._reason = "bad_timestamp"
            return False
        if rigid_body_id != self.expected_rigid_body_id or abs(rigid_value - rigid_body_id) > 1.0e-6:
            with self._lock:
                self._last_received_rigid_body_id = rigid_body_id
                self._reason = "wrong_rigid"
            return False
        receive_ns = time.monotonic_ns() if receive_monotonic_ns is None else int(receive_monotonic_ns)
        with self._lock:
            if self._latest is not None:
                previous_time, previous_frame = self._latest[:2]
                if source_time <= previous_time or frame <= previous_frame:
                    if receive_ns - self._latest[4] <= 250_000_000:
                        self._reason = "out_of_order"
                        return False
                    self._history.clear()
                elif source_time - previous_time > 0.25:
                    self._history.clear()
            self._history.append((source_time, position.copy()))
            cutoff = source_time - self.velocity_window_s
            while self._history and self._history[0][0] < cutoff:
                self._history.popleft()
            self._latest = (source_time, frame, rigid_body_id, position.copy(), receive_ns)
            self._last_received_rigid_body_id = rigid_body_id
            self._reason = "ok"
        return True

    @staticmethod
    def _fit_velocity(history):
        if len(history) < 2:
            return np.zeros(3, dtype=np.float32), False
        times = np.asarray([value[0] for value in history], dtype=np.float64)
        positions = np.asarray([value[1] for value in history], dtype=np.float64)
        relative = times - times[-1]
        if float(np.ptp(relative)) < 1.0e-4:
            return np.zeros(3, dtype=np.float32), False
        design = np.stack((np.ones_like(relative), relative), axis=-1)
        velocity = np.linalg.lstsq(design, positions, rcond=None)[0][1]
        return velocity.astype(np.float32), bool(np.isfinite(velocity).all())

    def sample(self, now_monotonic_ns=None):
        now_ns = time.monotonic_ns() if now_monotonic_ns is None else int(now_monotonic_ns)
        with self._lock:
            latest = self._latest
            history = list(self._history)
            received_rigid_body_id = self._last_received_rigid_body_id
            reason = self._reason
        if latest is None:
            return {
                "valid": False,
                "reason": reason,
                "source_time_s": np.nan,
                "frame": -1,
                "rigid_body_id": received_rigid_body_id,
                "position": np.full(3, np.nan, dtype=np.float32),
                "velocity": np.full(3, np.nan, dtype=np.float32),
                "age_ms": np.nan,
                "receive_monotonic_ns": 0,
                "sample_count": 0,
            }
        source_time, frame, rigid_body_id, position, receive_ns = latest
        age_s = max(0.0, (now_ns - receive_ns) * 1.0e-9)
        velocity, velocity_valid = self._fit_velocity(history)
        valid = age_s <= self.max_age_s and velocity_valid and reason == "ok"
        if reason != "ok":
            pass
        elif age_s > self.max_age_s:
            reason = "stale"
        elif not velocity_valid:
            reason = "velocity_warmup"
        return {
            "valid": bool(valid),
            "reason": reason,
            "source_time_s": float(source_time),
            "frame": int(frame),
            "rigid_body_id": int(received_rigid_body_id if reason == "wrong_rigid" else rigid_body_id),
            "position": np.asarray(position, dtype=np.float32),
            "velocity": velocity,
            "age_ms": 1000.0 * age_s,
            "receive_monotonic_ns": int(receive_ns),
            "sample_count": len(history),
        }


class RawBallStateSource:
    def __init__(self, topic=RAW_BALL_TOPIC, max_age_s=0.06,
                 expected_rigid_body_id=BALL_RIGID_BODY_ID):
        try:
            import rospy
            from std_msgs.msg import Float64MultiArray
        except ImportError as error:  # pragma: no cover - ROS is unavailable in unit tests
            raise RuntimeError("Raw ball diagnostics require rospy and std_msgs") from error
        self.buffer = RawBallStateBuffer(max_age_s=max_age_s,
                                         expected_rigid_body_id=expected_rigid_body_id)
        self.subscriber = rospy.Subscriber(
            topic,
            Float64MultiArray,
            lambda message: self.buffer.ingest_payload(message.data),
            queue_size=1,
            tcp_nodelay=True,
        )

    def sample(self, now_monotonic_ns=None):
        return self.buffer.sample(now_monotonic_ns=now_monotonic_ns)

    def close(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.subscriber = None
