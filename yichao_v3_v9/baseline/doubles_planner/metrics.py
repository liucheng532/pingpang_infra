from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class MetricThresholds:
    base_distance_m: float = 0.47
    hand_distance_m: float = 0.20
    racket_distance_m: float = 0.25
    base_ttc_s: float = 0.35
    hand_ttc_s: float = 0.15
    racket_ttc_s: float = 0.15
    strike_position_error_m: float = 0.04
    strike_velocity_error_mps: float = 0.50
    strike_orientation_error_rad: float = 0.05
    strike_timing_error_s: float = 0.02
    strike_sample_window_s: float = 0.12
    simultaneous_strike_window_s: float = 0.08

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class PairKinematics:
    distance_m: float
    clearance_m: float
    closing_speed_mps: float
    time_to_contact_s: float | None
    time_to_threshold_s: float | None


@dataclass(frozen=True)
class SetKinematics:
    minimum_distance_m: float
    minimum_clearance_m: float
    maximum_closing_speed_mps: float
    minimum_time_to_contact_s: float | None
    minimum_time_to_threshold_s: float | None
    closest_pair: tuple[int, int]
    critical_ttc_pair: tuple[int, int] | None


def pair_kinematics(
    position_a: Sequence[float] | np.ndarray,
    velocity_a: Sequence[float] | np.ndarray,
    position_b: Sequence[float] | np.ndarray,
    velocity_b: Sequence[float] | np.ndarray,
    threshold_m: float,
) -> PairKinematics:
    if threshold_m < 0.0:
        raise ValueError("threshold_m must not be negative")
    position_a_array = np.asarray(position_a, dtype=float)
    position_b_array = np.asarray(position_b, dtype=float)
    velocity_a_array = np.asarray(velocity_a, dtype=float)
    velocity_b_array = np.asarray(velocity_b, dtype=float)
    if (
        position_a_array.shape != position_b_array.shape
        or position_a_array.shape != velocity_a_array.shape
        or position_a_array.shape != velocity_b_array.shape
        or position_a_array.ndim != 1
    ):
        raise ValueError("positions and velocities must be equal-width vectors")
    if not all(
        np.isfinite(values).all()
        for values in (position_a_array, position_b_array, velocity_a_array, velocity_b_array)
    ):
        raise ValueError("positions and velocities must be finite")

    displacement = position_b_array - position_a_array
    distance = float(np.linalg.norm(displacement))
    clearance = distance - threshold_m
    if distance <= 1.0e-9:
        closing_speed = float(np.linalg.norm(velocity_b_array - velocity_a_array))
    else:
        distance_rate = float(
            np.dot(displacement / distance, velocity_b_array - velocity_a_array)
        )
        closing_speed = max(0.0, -distance_rate)
    if clearance <= 0.0:
        time_to_threshold = 0.0
    elif closing_speed > 1.0e-9:
        time_to_threshold = clearance / closing_speed
    else:
        time_to_threshold = None
    time_to_contact = distance / closing_speed if closing_speed > 1.0e-9 else None
    return PairKinematics(
        distance_m=distance,
        clearance_m=clearance,
        closing_speed_mps=closing_speed,
        time_to_contact_s=time_to_contact,
        time_to_threshold_s=time_to_threshold,
    )


def set_kinematics(
    positions_a: Sequence[Sequence[float]] | np.ndarray,
    velocities_a: Sequence[Sequence[float]] | np.ndarray,
    positions_b: Sequence[Sequence[float]] | np.ndarray,
    velocities_b: Sequence[Sequence[float]] | np.ndarray,
    threshold_m: float,
) -> SetKinematics:
    positions_a_array = np.asarray(positions_a, dtype=float)
    positions_b_array = np.asarray(positions_b, dtype=float)
    velocities_a_array = np.asarray(velocities_a, dtype=float)
    velocities_b_array = np.asarray(velocities_b, dtype=float)
    if (
        positions_a_array.ndim != 2
        or positions_b_array.ndim != 2
        or positions_a_array.shape != velocities_a_array.shape
        or positions_b_array.shape != velocities_b_array.shape
        or positions_a_array.shape[1] != positions_b_array.shape[1]
        or positions_a_array.shape[0] == 0
        or positions_b_array.shape[0] == 0
    ):
        raise ValueError("point sets and velocity sets must be non-empty matching matrices")

    pairs: list[tuple[tuple[int, int], PairKinematics]] = []
    for index_a in range(positions_a_array.shape[0]):
        for index_b in range(positions_b_array.shape[0]):
            pairs.append(
                (
                    (index_a, index_b),
                    pair_kinematics(
                        positions_a_array[index_a],
                        velocities_a_array[index_a],
                        positions_b_array[index_b],
                        velocities_b_array[index_b],
                        threshold_m,
                    ),
                )
            )
    closest_pair, closest = min(pairs, key=lambda item: item[1].distance_m)
    finite_ttc = [
        (pair, values)
        for pair, values in pairs
        if values.time_to_threshold_s is not None
    ]
    if finite_ttc:
        critical_pair, critical = min(
            finite_ttc,
            key=lambda item: float(item[1].time_to_threshold_s),
        )
        minimum_ttc = critical.time_to_threshold_s
    else:
        critical_pair = None
        minimum_ttc = None
    contact_times = [
        values.time_to_contact_s
        for _, values in pairs
        if values.time_to_contact_s is not None
    ]
    return SetKinematics(
        minimum_distance_m=closest.distance_m,
        minimum_clearance_m=closest.clearance_m,
        maximum_closing_speed_mps=max(values.closing_speed_mps for _, values in pairs),
        minimum_time_to_contact_s=min(contact_times) if contact_times else None,
        minimum_time_to_threshold_s=minimum_ttc,
        closest_pair=closest_pair,
        critical_ttc_pair=critical_pair,
    )


def _finite_quantile(values: Sequence[float], quantile: float) -> float | None:
    finite = np.asarray([value for value in values if np.isfinite(value)], dtype=float)
    return float(np.quantile(finite, quantile)) if finite.size else None


def _sample_summary(
    values: Sequence[float],
    *,
    unit: str,
) -> dict[str, float | int | None]:
    suffix = f"_{unit}" if unit else ""
    finite = np.asarray([value for value in values if np.isfinite(value)], dtype=float)
    return {
        "samples": int(finite.size),
        f"minimum{suffix}": float(np.min(finite)) if finite.size else None,
        f"p05{suffix}": float(np.quantile(finite, 0.05)) if finite.size else None,
        f"mean{suffix}": float(np.mean(finite)) if finite.size else None,
        f"p95{suffix}": float(np.quantile(finite, 0.95)) if finite.size else None,
        f"maximum{suffix}": float(np.max(finite)) if finite.size else None,
    }


@dataclass
class _RelayShotRecord:
    shot_id: str | int
    hitter: str
    reserved_at_s: float | None = None
    reservation_tts_s: float | None = None
    committed_at_s: float | None = None
    commit_tts_s: float | None = None
    commit_slack_s: float | None = None
    peer_clear_at_commit: bool | None = None
    peer_clear_started_at_s: float | None = None
    commit_base_gap_m: float | None = None
    commit_base_ttc_s: float | None = None
    commit_hand_gap_m: float | None = None
    verified_strike_at_s: float | None = None
    handoff_at_s: float | None = None

    def to_dict(self) -> dict[str, object]:
        reservation_to_commit_s = None
        if self.reserved_at_s is not None and self.committed_at_s is not None:
            reservation_to_commit_s = self.committed_at_s - self.reserved_at_s
        peer_clear_start_latency_s = None
        if self.committed_at_s is not None and self.peer_clear_started_at_s is not None:
            peer_clear_start_latency_s = (
                self.peer_clear_started_at_s - self.committed_at_s
            )
        return {
            "shot_id": self.shot_id,
            "hitter": self.hitter,
            "reserved_at_s": self.reserved_at_s,
            "reservation_lead_s": self.reservation_tts_s,
            "committed_at_s": self.committed_at_s,
            "reservation_to_commit_s": reservation_to_commit_s,
            "commit_tts_s": self.commit_tts_s,
            "commit_slack_s": self.commit_slack_s,
            "peer_clear_at_commit": self.peer_clear_at_commit,
            "peer_clear_started_at_s": self.peer_clear_started_at_s,
            "peer_clear_start_latency_s": peer_clear_start_latency_s,
            "commit_base_gap_m": self.commit_base_gap_m,
            "commit_base_ttc_s": self.commit_base_ttc_s,
            "commit_hand_gap_m": self.commit_hand_gap_m,
            "verified_strike_at_s": self.verified_strike_at_s,
            "handoff_at_s": self.handoff_at_s,
        }


@dataclass
class RelayStageMetrics:
    """Event and phase metrics for a strict two-player relay supervisor.

    Reservations may be made ahead of the current turn token, so reservation
    order and commit-token ownership are checked independently.  The token only
    advances after :meth:`record_strike` receives a verified strike.
    """

    robots: tuple[str, str] = ("left", "right")
    initial_hitter: str | None = "left"
    minimum_reservation_lead_s: float = 0.0
    minimum_commit_lead_s: float = 0.0
    sample_period_s: float = 0.02
    retarget_deadband_m: float = 0.01
    protected_phases: frozenset[str] = frozenset({"HIT", "POST_DELAY"})
    _shots: dict[str | int, _RelayShotRecord] = field(
        default_factory=dict,
        init=False,
        repr=False,
    )
    _reservation_order: list[str | int] = field(
        default_factory=list,
        init=False,
        repr=False,
    )
    _turn_token: str | None = field(default=None, init=False, repr=False)
    _token_transitions: int = field(default=0, init=False, repr=False)
    _token_violations: Counter[str] = field(
        default_factory=Counter,
        init=False,
        repr=False,
    )
    _lifecycle_violations: Counter[str] = field(
        default_factory=Counter,
        init=False,
        repr=False,
    )
    _phase_interruptions: Counter[str] = field(
        default_factory=Counter,
        init=False,
        repr=False,
    )
    _phase_transitions: Counter[str] = field(
        default_factory=Counter,
        init=False,
        repr=False,
    )
    _retarget_by_robot: Counter[str] = field(
        default_factory=Counter,
        init=False,
        repr=False,
    )
    _retarget_by_phase: Counter[str] = field(
        default_factory=Counter,
        init=False,
        repr=False,
    )
    _protected_retarget_by_robot: Counter[str] = field(
        default_factory=Counter,
        init=False,
        repr=False,
    )
    _last_phases: dict[str, str] = field(default_factory=dict, init=False, repr=False)
    _last_commands_y: dict[str, float] = field(
        default_factory=dict,
        init=False,
        repr=False,
    )
    _last_command_directions: dict[str, int] = field(
        default_factory=dict,
        init=False,
        repr=False,
    )
    _phase_samples: int = field(default=0, init=False, repr=False)
    _simultaneous_hit_steps: int = field(default=0, init=False, repr=False)
    _simultaneous_hit_entries: int = field(default=0, init=False, repr=False)
    _last_simultaneous_hit: bool = field(default=False, init=False, repr=False)
    _retarget_events: int = field(default=0, init=False, repr=False)
    _protected_retarget_events: int = field(default=0, init=False, repr=False)
    _retarget_reversals: int = field(default=0, init=False, repr=False)

    def __post_init__(self) -> None:
        if len(self.robots) != 2 or len(set(self.robots)) != 2:
            raise ValueError("robots must contain two unique names")
        if self.initial_hitter is not None and self.initial_hitter not in self.robots:
            raise ValueError("initial_hitter must be one of robots or None")
        for name, value in (
            ("minimum_reservation_lead_s", self.minimum_reservation_lead_s),
            ("minimum_commit_lead_s", self.minimum_commit_lead_s),
            ("retarget_deadband_m", self.retarget_deadband_m),
        ):
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if not np.isfinite(self.sample_period_s) or self.sample_period_s <= 0.0:
            raise ValueError("sample_period_s must be finite and positive")
        if not self.protected_phases:
            raise ValueError("protected_phases must not be empty")
        self._turn_token = self.initial_hitter

    @property
    def turn_token(self) -> str | None:
        return self._turn_token

    def _other(self, robot: str) -> str:
        self._validate_robot(robot)
        first, second = self.robots
        return second if robot == first else first

    def _validate_robot(self, robot: str) -> None:
        if robot not in self.robots:
            raise ValueError(f"robot must be one of {self.robots}")

    @staticmethod
    def _finite(value: float, name: str) -> float:
        result = float(value)
        if not np.isfinite(result):
            raise ValueError(f"{name} must be finite")
        return result

    @classmethod
    def _nonnegative(cls, value: float | None, name: str) -> float | None:
        if value is None:
            return None
        result = cls._finite(value, name)
        if result < 0.0:
            raise ValueError(f"{name} must be non-negative")
        return result

    def _validate_pair_mapping(self, values: Mapping[str, object], name: str) -> None:
        if set(values) != set(self.robots):
            raise ValueError(f"{name} must contain exactly {self.robots}")

    def record_reservation(
        self,
        shot_id: str | int,
        hitter: str,
        *,
        now_s: float,
        time_to_strike_s: float,
    ) -> None:
        """Record the one-shot ownership decision made before commitment."""

        self._validate_robot(hitter)
        now = self._finite(now_s, "now_s")
        time_to_strike = self._finite(time_to_strike_s, "time_to_strike_s")
        existing = self._shots.get(shot_id)
        if existing is not None:
            self._lifecycle_violations["duplicate_reservation"] += 1
            if existing.hitter != hitter:
                self._token_violations["reservation_owner_changed"] += 1
            return

        if self._reservation_order:
            expected = self._other(self._shots[self._reservation_order[-1]].hitter)
        else:
            expected = self.initial_hitter
        if expected is not None and hitter != expected:
            self._token_violations["reservation_order"] += 1
        if self._turn_token is None:
            self._turn_token = hitter
        self._shots[shot_id] = _RelayShotRecord(
            shot_id=shot_id,
            hitter=hitter,
            reserved_at_s=now,
            reservation_tts_s=time_to_strike,
        )
        self._reservation_order.append(shot_id)

    def record_commit(
        self,
        shot_id: str | int,
        hitter: str,
        *,
        now_s: float,
        time_to_strike_s: float,
        peer_clear_at_commit: bool,
        base_gap_m: float | None = None,
        base_ttc_s: float | None = None,
        hand_gap_m: float | None = None,
    ) -> None:
        """Record the atomic hitter-HIT and teammate-CLEAR commitment."""

        self._validate_robot(hitter)
        if not isinstance(peer_clear_at_commit, bool):
            raise TypeError("peer_clear_at_commit must be a bool")
        now = self._finite(now_s, "now_s")
        time_to_strike = self._finite(time_to_strike_s, "time_to_strike_s")
        record = self._shots.get(shot_id)
        if record is None:
            self._lifecycle_violations["commit_without_reservation"] += 1
            record = _RelayShotRecord(shot_id=shot_id, hitter=hitter)
            self._shots[shot_id] = record
            self._reservation_order.append(shot_id)
        elif record.hitter != hitter:
            self._token_violations["commit_owner_changed"] += 1
        if record.committed_at_s is not None:
            self._lifecycle_violations["duplicate_commit"] += 1
            return
        if self._turn_token is None:
            self._turn_token = hitter
        if hitter != self._turn_token:
            self._token_violations["commit_token"] += 1
        if any(
            other.committed_at_s is not None
            and other.verified_strike_at_s is None
            and other.shot_id != shot_id
            for other in self._shots.values()
        ):
            self._token_violations["overlapping_commit"] += 1
        if record.reserved_at_s is not None and now < record.reserved_at_s:
            self._lifecycle_violations["commit_before_reservation"] += 1

        record.committed_at_s = now
        record.commit_tts_s = time_to_strike
        record.commit_slack_s = time_to_strike - self.minimum_commit_lead_s
        record.peer_clear_at_commit = peer_clear_at_commit
        record.commit_base_gap_m = self._nonnegative(base_gap_m, "base_gap_m")
        record.commit_base_ttc_s = self._nonnegative(base_ttc_s, "base_ttc_s")
        record.commit_hand_gap_m = self._nonnegative(hand_gap_m, "hand_gap_m")

    def record_peer_clear_started(self, shot_id: str | int, *, now_s: float) -> None:
        """Record observed CLEAR motion onset, which may lag the command."""

        now = self._finite(now_s, "now_s")
        record = self._shots.get(shot_id)
        if record is None or record.committed_at_s is None:
            self._lifecycle_violations["peer_clear_without_commit"] += 1
            return
        if record.peer_clear_started_at_s is not None:
            self._lifecycle_violations["duplicate_peer_clear_start"] += 1
            return
        if now < record.committed_at_s:
            self._lifecycle_violations["peer_clear_before_commit"] += 1
        record.peer_clear_started_at_s = now

    def record_strike(
        self,
        shot_id: str | int,
        hitter: str,
        *,
        now_s: float,
        verified: bool,
    ) -> None:
        """Record contact/release evidence and advance the strict turn token."""

        self._validate_robot(hitter)
        if not isinstance(verified, bool):
            raise TypeError("verified must be a bool")
        now = self._finite(now_s, "now_s")
        record = self._shots.get(shot_id)
        if record is None:
            self._lifecycle_violations["strike_without_reservation"] += 1
            return
        if record.committed_at_s is None:
            self._lifecycle_violations["strike_without_commit"] += 1
        if record.hitter != hitter:
            self._token_violations["strike_owner"] += 1
        if not verified:
            self._lifecycle_violations["unverified_strike"] += 1
            return
        if record.verified_strike_at_s is not None:
            self._lifecycle_violations["duplicate_verified_strike"] += 1
            return
        record.verified_strike_at_s = now
        if hitter != self._turn_token:
            self._token_violations["strike_token"] += 1
            return
        self._turn_token = self._other(hitter)
        self._token_transitions += 1

    def record_handoff(self, shot_id: str | int, *, now_s: float) -> None:
        now = self._finite(now_s, "now_s")
        record = self._shots.get(shot_id)
        if record is None:
            self._lifecycle_violations["handoff_without_reservation"] += 1
            return
        if record.handoff_at_s is not None:
            self._lifecycle_violations["duplicate_handoff"] += 1
            return
        if record.verified_strike_at_s is None:
            self._lifecycle_violations["handoff_without_verified_strike"] += 1
        elif now < record.verified_strike_at_s:
            self._lifecycle_violations["handoff_before_strike"] += 1
        record.handoff_at_s = now

    def record_phase_interruption(
        self,
        robot: str,
        *,
        from_phase: str,
        to_phase: str,
        reason: str = "external",
    ) -> None:
        """Record a semantic interruption reported by the controller bridge."""

        self._validate_robot(robot)
        if not from_phase or not to_phase or not reason:
            raise ValueError("phase names and reason must not be empty")
        self._phase_interruptions[
            f"{robot}:{from_phase}->{to_phase}:{reason}"
        ] += 1

    def add_phase_sample(
        self,
        *,
        now_s: float,
        phases: Mapping[str, str],
        command_y: Mapping[str, float] | None = None,
        base_y: Mapping[str, float] | None = None,
    ) -> dict[str, object]:
        """Sample controller phases and optionally infer command retargets."""

        self._finite(now_s, "now_s")
        self._validate_pair_mapping(phases, "phases")
        if command_y is not None:
            self._validate_pair_mapping(command_y, "command_y")
        if base_y is not None:
            self._validate_pair_mapping(base_y, "base_y")
        if base_y is not None and command_y is None:
            raise ValueError("base_y requires command_y")

        normalized_phases = {robot: str(phases[robot]).upper() for robot in self.robots}
        interruption_count = 0
        retarget_count = 0
        reversal_count = 0
        for robot in self.robots:
            phase = normalized_phases[robot]
            if not phase:
                raise ValueError("phase names must not be empty")
            previous_phase = self._last_phases.get(robot)
            if previous_phase is not None and previous_phase != phase:
                self._phase_transitions[f"{robot}:{previous_phase}->{phase}"] += 1
                expected_exit = (
                    previous_phase == "HIT" and phase == "POST_DELAY"
                ) or (
                    previous_phase == "POST_DELAY" and phase == "OUTWARD"
                )
                if previous_phase in self.protected_phases and not expected_exit:
                    self._phase_interruptions[
                        f"{robot}:{previous_phase}->{phase}:observed"
                    ] += 1
                    interruption_count += 1

            if command_y is not None:
                command = self._finite(command_y[robot], f"command_y[{robot}]")
                previous_command = self._last_commands_y.get(robot)
                changed = (
                    previous_command is not None
                    and abs(command - previous_command) > self.retarget_deadband_m
                )
                direction = 0
                if base_y is not None:
                    base = self._finite(base_y[robot], f"base_y[{robot}]")
                    displacement = command - base
                    if abs(displacement) > self.retarget_deadband_m:
                        direction = 1 if displacement > 0.0 else -1
                previous_direction = self._last_command_directions.get(robot, 0)
                if changed:
                    self._retarget_events += 1
                    self._retarget_by_robot[robot] += 1
                    self._retarget_by_phase[f"{robot}:{phase}"] += 1
                    retarget_count += 1
                    protected = (
                        phase in self.protected_phases
                        and previous_phase in self.protected_phases
                    )
                    if protected:
                        self._protected_retarget_events += 1
                        self._protected_retarget_by_robot[robot] += 1
                    if (
                        direction
                        and previous_direction
                        and direction != previous_direction
                        and previous_phase == phase
                    ):
                        self._retarget_reversals += 1
                        reversal_count += 1
                self._last_commands_y[robot] = command
                if direction:
                    self._last_command_directions[robot] = direction
            self._last_phases[robot] = phase

        both_hit = all(normalized_phases[robot] == "HIT" for robot in self.robots)
        if both_hit and not self._last_simultaneous_hit:
            self._simultaneous_hit_entries += 1
        self._simultaneous_hit_steps += int(both_hit)
        self._last_simultaneous_hit = both_hit
        self._phase_samples += 1
        return {
            "simultaneous_hit": both_hit,
            "phase_interruptions": interruption_count,
            "retargets": retarget_count,
            "retarget_reversals": reversal_count,
        }

    def summary(self) -> dict[str, object]:
        rows = [self._shots[shot_id].to_dict() for shot_id in self._reservation_order]
        committed = [row for row in rows if row["committed_at_s"] is not None]
        reserved = [row for row in rows if row["reserved_at_s"] is not None]

        def present(rows_to_read: Sequence[dict[str, object]], key: str) -> list[float]:
            return [
                float(row[key])
                for row in rows_to_read
                if row[key] is not None
            ]

        peer_clear_values = [
            bool(row["peer_clear_at_commit"])
            for row in committed
            if row["peer_clear_at_commit"] is not None
        ]
        reservation_late = sum(
            float(row["reservation_lead_s"]) < self.minimum_reservation_lead_s
            for row in reserved
        )
        commit_late = sum(float(row["commit_slack_s"]) < 0.0 for row in committed)
        return {
            "shots_reserved": len(reserved),
            "shots_committed": len(committed),
            "shots_verified_strike": sum(
                row["verified_strike_at_s"] is not None for row in rows
            ),
            "shots_handed_off": sum(row["handoff_at_s"] is not None for row in rows),
            "turn_token": self._turn_token,
            "token_transitions": self._token_transitions,
            "minimum_reservation_lead_s": self.minimum_reservation_lead_s,
            "minimum_commit_lead_s": self.minimum_commit_lead_s,
            "reservation_lead": _sample_summary(
                present(reserved, "reservation_lead_s"),
                unit="s",
            ),
            "reservation_to_commit": _sample_summary(
                present(committed, "reservation_to_commit_s"),
                unit="s",
            ),
            "commit_tts": _sample_summary(
                present(committed, "commit_tts_s"),
                unit="s",
            ),
            "commit_slack": _sample_summary(
                present(committed, "commit_slack_s"),
                unit="s",
            ),
            "reservation_lead_violation_count": int(reservation_late),
            "reservation_lead_violation_rate": (
                float(reservation_late / len(reserved)) if reserved else 0.0
            ),
            "late_commit_count": int(commit_late),
            "late_commit_rate": (
                float(commit_late / len(committed)) if committed else 0.0
            ),
            "peer_clear_at_commit_count": sum(peer_clear_values),
            "peer_clear_at_commit_rate": (
                float(np.mean(peer_clear_values)) if peer_clear_values else 0.0
            ),
            "peer_clear_start_latency": _sample_summary(
                present(committed, "peer_clear_start_latency_s"),
                unit="s",
            ),
            "commit_base_gap": _sample_summary(
                present(committed, "commit_base_gap_m"),
                unit="m",
            ),
            "commit_base_ttc": _sample_summary(
                present(committed, "commit_base_ttc_s"),
                unit="s",
            ),
            "commit_hand_gap": _sample_summary(
                present(committed, "commit_hand_gap_m"),
                unit="m",
            ),
            "token_violation_count": int(sum(self._token_violations.values())),
            "token_violations": dict(sorted(self._token_violations.items())),
            "lifecycle_violation_count": int(
                sum(self._lifecycle_violations.values())
            ),
            "lifecycle_violations": dict(sorted(self._lifecycle_violations.items())),
            "phase_samples": self._phase_samples,
            "simultaneous_hit_steps": self._simultaneous_hit_steps,
            "simultaneous_hit_duration_s": float(
                self._simultaneous_hit_steps * self.sample_period_s
            ),
            "simultaneous_hit_entries": self._simultaneous_hit_entries,
            "phase_interruption_count": int(sum(self._phase_interruptions.values())),
            "phase_interruptions": dict(sorted(self._phase_interruptions.items())),
            "phase_transitions": dict(sorted(self._phase_transitions.items())),
            "retarget_count": self._retarget_events,
            "protected_phase_retarget_count": self._protected_retarget_events,
            "retarget_reversal_count": self._retarget_reversals,
            "retarget_by_robot": dict(sorted(self._retarget_by_robot.items())),
            "protected_phase_retarget_by_robot": dict(
                sorted(self._protected_retarget_by_robot.items())
            ),
            "retarget_by_phase": dict(sorted(self._retarget_by_phase.items())),
            "shots": rows,
        }


@dataclass
class DoublesSafetyMetrics:
    thresholds: MetricThresholds = field(default_factory=MetricThresholds)
    sample_period_s: float = 0.02
    steps: int = 0
    base_distance_samples: list[float] = field(default_factory=list)
    hand_distance_samples: list[float] = field(default_factory=list)
    racket_distance_samples: list[float] = field(default_factory=list)
    base_closing_speed_samples: list[float] = field(default_factory=list)
    hand_closing_speed_samples: list[float] = field(default_factory=list)
    racket_closing_speed_samples: list[float] = field(default_factory=list)
    base_contact_time_samples: list[float] = field(default_factory=list)
    hand_contact_time_samples: list[float] = field(default_factory=list)
    racket_contact_time_samples: list[float] = field(default_factory=list)
    base_ttc_samples: list[float] = field(default_factory=list)
    hand_ttc_samples: list[float] = field(default_factory=list)
    racket_ttc_samples: list[float] = field(default_factory=list)
    base_distance_violations: int = 0
    hand_distance_violations: int = 0
    racket_distance_violations: int = 0
    base_ttc_violations: int = 0
    hand_ttc_violations: int = 0
    racket_ttc_violations: int = 0
    simultaneous_hit_steps: int = 0
    simultaneous_strike_window_steps: int = 0
    terminal_command_gap_samples: list[float] = field(default_factory=list)
    safety_command_gap_samples: list[float] = field(default_factory=list)
    controller_command_gap_samples: list[float] = field(default_factory=list)
    motion_reference_gap_samples: list[float] = field(default_factory=list)
    command_tracking_errors: dict[str, list[float]] = field(
        default_factory=lambda: {"left": [], "right": []}
    )
    controller_tracking_errors: dict[str, list[float]] = field(
        default_factory=lambda: {"left": [], "right": []}
    )
    motion_reference_tracking_errors: dict[str, list[float]] = field(
        default_factory=lambda: {"left": [], "right": []}
    )
    planner_controller_errors: dict[str, list[float]] = field(
        default_factory=lambda: {"left": [], "right": []}
    )
    phase_pairs: Counter[str] = field(default_factory=Counter)
    closest_hand_pair_counts: Counter[str] = field(default_factory=Counter)

    def __post_init__(self) -> None:
        if self.sample_period_s <= 0.0:
            raise ValueError("sample_period_s must be positive")

    def add_sample(
        self,
        *,
        base_positions_xy: Mapping[str, Sequence[float] | np.ndarray],
        base_velocities_xy: Mapping[str, Sequence[float] | np.ndarray],
        hand_positions: Mapping[str, Sequence[Sequence[float]] | np.ndarray],
        hand_velocities: Mapping[str, Sequence[Sequence[float]] | np.ndarray],
        racket_positions: Mapping[str, Sequence[float] | np.ndarray],
        racket_velocities: Mapping[str, Sequence[float] | np.ndarray],
        phases: Mapping[str, str],
        time_to_strike_s: Mapping[str, float],
        terminal_command_y: Mapping[str, float],
        safety_command_y: Mapping[str, float],
        controller_command_y: Mapping[str, float] | None = None,
        motion_reference_y: Mapping[str, float] | None = None,
    ) -> dict[str, float | bool | str | None]:
        names = ("left", "right")
        for mapping_name, mapping in (
            ("base_positions_xy", base_positions_xy),
            ("base_velocities_xy", base_velocities_xy),
            ("hand_positions", hand_positions),
            ("hand_velocities", hand_velocities),
            ("racket_positions", racket_positions),
            ("racket_velocities", racket_velocities),
            ("phases", phases),
            ("time_to_strike_s", time_to_strike_s),
            ("terminal_command_y", terminal_command_y),
            ("safety_command_y", safety_command_y),
        ):
            if set(mapping) != set(names):
                raise ValueError(f"{mapping_name} must contain exactly left and right")
        if controller_command_y is None:
            controller_command_y = terminal_command_y
        if motion_reference_y is None:
            motion_reference_y = controller_command_y
        for mapping_name, mapping in (
            ("controller_command_y", controller_command_y),
            ("motion_reference_y", motion_reference_y),
        ):
            if set(mapping) != set(names):
                raise ValueError(f"{mapping_name} must contain exactly left and right")

        base = pair_kinematics(
            base_positions_xy["left"],
            base_velocities_xy["left"],
            base_positions_xy["right"],
            base_velocities_xy["right"],
            self.thresholds.base_distance_m,
        )
        hands = set_kinematics(
            hand_positions["left"],
            hand_velocities["left"],
            hand_positions["right"],
            hand_velocities["right"],
            self.thresholds.hand_distance_m,
        )
        rackets = pair_kinematics(
            racket_positions["left"],
            racket_velocities["left"],
            racket_positions["right"],
            racket_velocities["right"],
            self.thresholds.racket_distance_m,
        )
        self.steps += 1
        self.base_distance_samples.append(base.distance_m)
        self.hand_distance_samples.append(hands.minimum_distance_m)
        self.racket_distance_samples.append(rackets.distance_m)
        self.base_closing_speed_samples.append(base.closing_speed_mps)
        self.hand_closing_speed_samples.append(hands.maximum_closing_speed_mps)
        self.racket_closing_speed_samples.append(rackets.closing_speed_mps)
        for value, target in (
            (base.time_to_contact_s, self.base_contact_time_samples),
            (hands.minimum_time_to_contact_s, self.hand_contact_time_samples),
            (rackets.time_to_contact_s, self.racket_contact_time_samples),
        ):
            if value is not None:
                target.append(float(value))
        for values, target in (
            (base, self.base_ttc_samples),
            (hands, self.hand_ttc_samples),
            (rackets, self.racket_ttc_samples),
        ):
            time_to_threshold = values.time_to_threshold_s if isinstance(values, PairKinematics) else values.minimum_time_to_threshold_s
            if time_to_threshold is not None:
                target.append(float(time_to_threshold))

        self.base_distance_violations += int(base.distance_m < self.thresholds.base_distance_m)
        self.hand_distance_violations += int(
            hands.minimum_distance_m < self.thresholds.hand_distance_m
        )
        self.racket_distance_violations += int(
            rackets.distance_m < self.thresholds.racket_distance_m
        )
        self.base_ttc_violations += int(
            base.time_to_threshold_s is not None
            and base.time_to_threshold_s < self.thresholds.base_ttc_s
        )
        self.hand_ttc_violations += int(
            hands.minimum_time_to_threshold_s is not None
            and hands.minimum_time_to_threshold_s < self.thresholds.hand_ttc_s
        )
        self.racket_ttc_violations += int(
            rackets.time_to_threshold_s is not None
            and rackets.time_to_threshold_s < self.thresholds.racket_ttc_s
        )

        both_hit = phases["left"] == "HIT" and phases["right"] == "HIT"
        both_in_strike_window = both_hit and all(
            abs(float(time_to_strike_s[name]))
            <= self.thresholds.simultaneous_strike_window_s
            for name in names
        )
        self.simultaneous_hit_steps += int(both_hit)
        self.simultaneous_strike_window_steps += int(both_in_strike_window)
        self.phase_pairs[f"{phases['left']}|{phases['right']}"] += 1
        hand_labels = ("left_hand", "right_hand")
        closest_hand_pair = (
            f"left.{hand_labels[hands.closest_pair[0]]}-"
            f"right.{hand_labels[hands.closest_pair[1]]}"
        )
        self.closest_hand_pair_counts[closest_hand_pair] += 1

        terminal_gap = float(terminal_command_y["right"] - terminal_command_y["left"])
        safety_gap = float(safety_command_y["right"] - safety_command_y["left"])
        controller_gap = float(
            controller_command_y["right"] - controller_command_y["left"]
        )
        motion_reference_gap = float(
            motion_reference_y["right"] - motion_reference_y["left"]
        )
        self.terminal_command_gap_samples.append(terminal_gap)
        self.safety_command_gap_samples.append(safety_gap)
        self.controller_command_gap_samples.append(controller_gap)
        self.motion_reference_gap_samples.append(motion_reference_gap)
        for name in names:
            base_y = float(np.asarray(base_positions_xy[name], dtype=float)[1])
            self.command_tracking_errors[name].append(
                abs(base_y - float(terminal_command_y[name]))
            )
            self.controller_tracking_errors[name].append(
                abs(base_y - float(controller_command_y[name]))
            )
            self.motion_reference_tracking_errors[name].append(
                abs(base_y - float(motion_reference_y[name]))
            )
            self.planner_controller_errors[name].append(
                abs(float(terminal_command_y[name]) - float(controller_command_y[name]))
            )

        return {
            "base_distance_m": base.distance_m,
            "base_closing_speed_mps": base.closing_speed_mps,
            "base_ttc_s": base.time_to_threshold_s,
            "base_time_to_contact_s": base.time_to_contact_s,
            "hand_distance_m": hands.minimum_distance_m,
            "hand_closing_speed_mps": hands.maximum_closing_speed_mps,
            "hand_ttc_s": hands.minimum_time_to_threshold_s,
            "hand_time_to_contact_s": hands.minimum_time_to_contact_s,
            "closest_hand_pair": closest_hand_pair,
            "racket_distance_m": rackets.distance_m,
            "racket_closing_speed_mps": rackets.closing_speed_mps,
            "racket_ttc_s": rackets.time_to_threshold_s,
            "racket_time_to_contact_s": rackets.time_to_contact_s,
            "simultaneous_hit": both_hit,
            "simultaneous_strike_window": both_in_strike_window,
            "terminal_command_gap_m": terminal_gap,
            "safety_command_gap_m": safety_gap,
            "controller_command_gap_m": controller_gap,
            "motion_reference_gap_m": motion_reference_gap,
        }

    def summary(self) -> dict[str, object]:
        def rate(count: int) -> float:
            return float(count / self.steps) if self.steps else 0.0

        def distance_summary(values: Sequence[float]) -> dict[str, float | None]:
            return {
                "minimum_m": min(values) if values else None,
                "p01_m": _finite_quantile(values, 0.01),
                "p05_m": _finite_quantile(values, 0.05),
                "mean_m": float(np.mean(values)) if values else None,
            }

        def ttc_summary(values: Sequence[float]) -> dict[str, float | None]:
            return {
                "minimum_s": min(values) if values else None,
                "p05_s": _finite_quantile(values, 0.05),
            }

        def closing_speed_summary(values: Sequence[float]) -> dict[str, float | None]:
            return {
                "maximum_mps": max(values) if values else None,
                "p95_mps": _finite_quantile(values, 0.95),
                "mean_mps": float(np.mean(values)) if values else None,
            }

        def duration(count: int) -> float:
            return float(count * self.sample_period_s)

        return {
            "steps": self.steps,
            "thresholds": vars(self.thresholds),
            "base_distance": distance_summary(self.base_distance_samples),
            "hand_distance": distance_summary(self.hand_distance_samples),
            "racket_distance": distance_summary(self.racket_distance_samples),
            "base_closing_speed": closing_speed_summary(self.base_closing_speed_samples),
            "hand_closing_speed": closing_speed_summary(self.hand_closing_speed_samples),
            "racket_closing_speed": closing_speed_summary(self.racket_closing_speed_samples),
            "base_time_to_contact": ttc_summary(self.base_contact_time_samples),
            "hand_time_to_contact": ttc_summary(self.hand_contact_time_samples),
            "racket_time_to_contact": ttc_summary(self.racket_contact_time_samples),
            "base_ttc": ttc_summary(self.base_ttc_samples),
            "hand_ttc": ttc_summary(self.hand_ttc_samples),
            "racket_ttc": ttc_summary(self.racket_ttc_samples),
            "base_distance_violation_rate": rate(self.base_distance_violations),
            "hand_distance_violation_rate": rate(self.hand_distance_violations),
            "racket_distance_violation_rate": rate(self.racket_distance_violations),
            "base_ttc_violation_rate": rate(self.base_ttc_violations),
            "hand_ttc_violation_rate": rate(self.hand_ttc_violations),
            "racket_ttc_violation_rate": rate(self.racket_ttc_violations),
            "base_distance_violation_duration_s": duration(
                self.base_distance_violations
            ),
            "hand_distance_violation_duration_s": duration(
                self.hand_distance_violations
            ),
            "racket_distance_violation_duration_s": duration(
                self.racket_distance_violations
            ),
            "base_ttc_violation_duration_s": duration(self.base_ttc_violations),
            "hand_ttc_violation_duration_s": duration(self.hand_ttc_violations),
            "racket_ttc_violation_duration_s": duration(
                self.racket_ttc_violations
            ),
            "simultaneous_hit_steps": self.simultaneous_hit_steps,
            "simultaneous_hit_duration_s": duration(self.simultaneous_hit_steps),
            "simultaneous_strike_window_steps": self.simultaneous_strike_window_steps,
            "minimum_terminal_command_gap_m": min(self.terminal_command_gap_samples)
            if self.terminal_command_gap_samples
            else None,
            "minimum_safety_command_gap_m": min(self.safety_command_gap_samples)
            if self.safety_command_gap_samples
            else None,
            "minimum_controller_command_gap_m": min(
                self.controller_command_gap_samples
            )
            if self.controller_command_gap_samples
            else None,
            "minimum_motion_reference_gap_m": min(
                self.motion_reference_gap_samples
            )
            if self.motion_reference_gap_samples
            else None,
            "command_tracking": {
                name: {
                    "mean_absolute_error_m": float(np.mean(values)) if values else None,
                    "p95_absolute_error_m": _finite_quantile(values, 0.95),
                    "maximum_absolute_error_m": max(values) if values else None,
                }
                for name, values in self.command_tracking_errors.items()
            },
            "controller_command_tracking": {
                name: {
                    "mean_absolute_error_m": float(np.mean(values)) if values else None,
                    "p95_absolute_error_m": _finite_quantile(values, 0.95),
                    "maximum_absolute_error_m": max(values) if values else None,
                }
                for name, values in self.controller_tracking_errors.items()
            },
            "motion_reference_tracking": {
                name: {
                    "mean_absolute_error_m": float(np.mean(values)) if values else None,
                    "p95_absolute_error_m": _finite_quantile(values, 0.95),
                    "maximum_absolute_error_m": max(values) if values else None,
                }
                for name, values in self.motion_reference_tracking_errors.items()
            },
            "planner_to_controller_command": {
                name: {
                    "mean_absolute_error_m": float(np.mean(values)) if values else None,
                    "p95_absolute_error_m": _finite_quantile(values, 0.95),
                    "maximum_absolute_error_m": max(values) if values else None,
                }
                for name, values in self.planner_controller_errors.items()
            },
            "phase_pair_steps": dict(sorted(self.phase_pairs.items())),
            "closest_hand_pair_steps": dict(sorted(self.closest_hand_pair_counts.items())),
        }


@dataclass(frozen=True)
class StrikeSample:
    time_offset_s: float
    position_error_m: float
    velocity_error_mps: float
    orientation_error_rad: float


@dataclass
class StrikeMetrics:
    thresholds: MetricThresholds = field(default_factory=MetricThresholds)
    samples: dict[int, list[StrikeSample]] = field(default_factory=dict)
    hitters: dict[int, str] = field(default_factory=dict)

    def add_sample(
        self,
        shot: int,
        hitter: str,
        *,
        time_offset_s: float,
        position_error_m: float,
        velocity_error_mps: float,
        orientation_error_rad: float,
    ) -> None:
        values = np.asarray(
            [time_offset_s, position_error_m, velocity_error_mps, orientation_error_rad],
            dtype=float,
        )
        if not np.isfinite(values).all():
            raise ValueError("strike samples must be finite")
        if abs(time_offset_s) > self.thresholds.strike_sample_window_s + 1.0e-9:
            return
        if hitter not in {"left", "right"}:
            raise ValueError("hitter must be left or right")
        previous = self.hitters.setdefault(shot, hitter)
        if previous != hitter:
            raise ValueError("a shot cannot change hitter")
        self.samples.setdefault(shot, []).append(
            StrikeSample(
                time_offset_s=float(time_offset_s),
                position_error_m=float(position_error_m),
                velocity_error_mps=float(velocity_error_mps),
                orientation_error_rad=float(orientation_error_rad),
            )
        )

    def shot_summary(self, shot: int) -> dict[str, object]:
        samples = self.samples.get(shot, [])
        if not samples:
            raise ValueError(f"shot {shot} has no strike-window samples")
        scheduled = min(samples, key=lambda sample: abs(sample.time_offset_s))
        closest = min(
            samples,
            key=lambda sample: (
                sample.position_error_m,
                sample.velocity_error_mps,
                abs(sample.time_offset_s),
            ),
        )
        timing_error = abs(closest.time_offset_s)
        success = (
            scheduled.position_error_m <= self.thresholds.strike_position_error_m
            and scheduled.velocity_error_mps <= self.thresholds.strike_velocity_error_mps
            and scheduled.orientation_error_rad
            <= self.thresholds.strike_orientation_error_rad
            and timing_error <= self.thresholds.strike_timing_error_s
        )
        return {
            "shot": shot,
            "hitter": self.hitters[shot],
            "scheduled_sample_time_offset_s": scheduled.time_offset_s,
            "scheduled_position_error_m": scheduled.position_error_m,
            "scheduled_velocity_error_mps": scheduled.velocity_error_mps,
            "scheduled_orientation_error_rad": scheduled.orientation_error_rad,
            "closest_approach_time_offset_s": closest.time_offset_s,
            "timing_error_s": timing_error,
            "minimum_position_error_m": closest.position_error_m,
            "strike_success": success,
        }

    def summary(self, expected_shots: int | None = None) -> dict[str, object]:
        shot_ids = sorted(self.samples)
        if expected_shots is not None and shot_ids != list(range(expected_shots)):
            raise ValueError(
                f"strike samples must cover shots 0..{expected_shots - 1}, got {shot_ids}"
            )
        shots = [self.shot_summary(shot) for shot in shot_ids]

        def values(key: str) -> np.ndarray:
            return np.asarray([float(shot[key]) for shot in shots], dtype=float)

        return {
            "thresholds": {
                "position_error_m": self.thresholds.strike_position_error_m,
                "velocity_error_mps": self.thresholds.strike_velocity_error_mps,
                "orientation_error_rad": self.thresholds.strike_orientation_error_rad,
                "timing_error_s": self.thresholds.strike_timing_error_s,
            },
            "shots": shots,
            "success_rate": float(
                np.mean([bool(shot["strike_success"]) for shot in shots])
            )
            if shots
            else 0.0,
            "mean_scheduled_position_error_m": float(
                values("scheduled_position_error_m").mean()
            )
            if shots
            else None,
            "p95_scheduled_position_error_m": float(
                np.quantile(values("scheduled_position_error_m"), 0.95)
            )
            if shots
            else None,
            "mean_scheduled_velocity_error_mps": float(
                values("scheduled_velocity_error_mps").mean()
            )
            if shots
            else None,
            "p95_scheduled_velocity_error_mps": float(
                np.quantile(values("scheduled_velocity_error_mps"), 0.95)
            )
            if shots
            else None,
            "mean_scheduled_orientation_error_rad": float(
                values("scheduled_orientation_error_rad").mean()
            )
            if shots
            else None,
            "p95_scheduled_orientation_error_rad": float(
                np.quantile(values("scheduled_orientation_error_rad"), 0.95)
            )
            if shots
            else None,
            "mean_timing_error_s": float(values("timing_error_s").mean())
            if shots
            else None,
            "p95_timing_error_s": float(np.quantile(values("timing_error_s"), 0.95))
            if shots
            else None,
        }


__all__ = [
    "DoublesSafetyMetrics",
    "MetricThresholds",
    "PairKinematics",
    "RelayStageMetrics",
    "SetKinematics",
    "StrikeMetrics",
    "StrikeSample",
    "pair_kinematics",
    "set_kinematics",
]
