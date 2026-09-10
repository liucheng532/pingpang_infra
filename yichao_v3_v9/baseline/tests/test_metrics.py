from __future__ import annotations

import numpy as np
import pytest

from doubles_planner.metrics import (
    DoublesSafetyMetrics,
    MetricThresholds,
    StrikeMetrics,
    pair_kinematics,
    set_kinematics,
)


def test_pair_kinematics_uses_clearance_to_threshold_for_ttc() -> None:
    values = pair_kinematics(
        [0.0, 0.0],
        [0.0, 0.4],
        [0.0, 1.0],
        [0.0, -0.2],
        threshold_m=0.4,
    )
    assert values.distance_m == pytest.approx(1.0)
    assert values.clearance_m == pytest.approx(0.6)
    assert values.closing_speed_mps == pytest.approx(0.6)
    assert values.time_to_contact_s == pytest.approx(1.0 / 0.6)
    assert values.time_to_threshold_s == pytest.approx(1.0)


def test_pair_kinematics_reports_no_ttc_while_separating() -> None:
    values = pair_kinematics(
        [0.0, 0.0],
        [0.0, -0.4],
        [0.0, 1.0],
        [0.0, 0.2],
        threshold_m=0.4,
    )
    assert values.closing_speed_mps == 0.0
    assert values.time_to_contact_s is None
    assert values.time_to_threshold_s is None


def test_set_kinematics_finds_distance_and_ttc_pairs_independently() -> None:
    values = set_kinematics(
        [[0.0, 0.0], [0.0, 0.5]],
        [[0.0, 0.0], [0.0, 0.8]],
        [[0.0, 1.0], [0.0, 1.8]],
        [[0.0, 0.0], [0.0, -0.8]],
        threshold_m=0.2,
    )
    assert values.closest_pair == (1, 0)
    assert values.minimum_distance_m == pytest.approx(0.5)
    assert values.critical_ttc_pair == (1, 0)
    assert values.minimum_time_to_contact_s == pytest.approx(0.625)
    assert values.minimum_time_to_threshold_s == pytest.approx(0.375)


def _safety_sample(metrics: DoublesSafetyMetrics) -> dict[str, object]:
    return metrics.add_sample(
        base_positions_xy={"left": [0.0, -0.20], "right": [0.0, 0.20]},
        base_velocities_xy={"left": [0.0, 0.4], "right": [0.0, -0.4]},
        hand_positions={
            "left": [[0.0, -0.05, 1.0], [0.0, -0.40, 1.0]],
            "right": [[0.0, 0.05, 1.0], [0.0, 0.40, 1.0]],
        },
        hand_velocities={
            "left": [[0.0, 0.2, 0.0], [0.0, 0.0, 0.0]],
            "right": [[0.0, -0.2, 0.0], [0.0, 0.0, 0.0]],
        },
        racket_positions={"left": [0.0, -0.12, 1.0], "right": [0.0, 0.12, 1.0]},
        racket_velocities={"left": [0.0, 0.2, 0.0], "right": [0.0, -0.2, 0.0]},
        phases={"left": "HIT", "right": "HIT"},
        time_to_strike_s={"left": 0.01, "right": -0.01},
        terminal_command_y={"left": -0.30, "right": 0.30},
        safety_command_y={"left": -0.45, "right": 0.45},
        controller_command_y={"left": -0.28, "right": 0.32},
        motion_reference_y={"left": -0.22, "right": 0.22},
    )


def test_safety_metrics_cover_hands_ttc_commands_and_phase_overlap() -> None:
    metrics = DoublesSafetyMetrics()
    sample = _safety_sample(metrics)
    assert sample["base_distance_m"] == pytest.approx(0.4)
    assert sample["base_time_to_contact_s"] == pytest.approx(0.5)
    assert sample["base_ttc_s"] == 0.0
    assert sample["hand_distance_m"] == pytest.approx(0.1)
    assert sample["simultaneous_hit"] is True
    assert sample["simultaneous_strike_window"] is True

    summary = metrics.summary()
    assert summary["base_distance_violation_rate"] == 1.0
    assert summary["hand_distance_violation_rate"] == 1.0
    assert summary["simultaneous_hit_steps"] == 1
    assert summary["minimum_terminal_command_gap_m"] == pytest.approx(0.6)
    assert summary["minimum_safety_command_gap_m"] == pytest.approx(0.9)
    assert summary["minimum_controller_command_gap_m"] == pytest.approx(0.6)
    assert summary["minimum_motion_reference_gap_m"] == pytest.approx(0.44)
    assert summary["planner_to_controller_command"]["left"][
        "mean_absolute_error_m"
    ] == pytest.approx(0.02)
    assert summary["base_distance_violation_duration_s"] == pytest.approx(0.02)
    assert summary["base_closing_speed"]["maximum_mps"] == pytest.approx(0.8)
    assert summary["phase_pair_steps"] == {"HIT|HIT": 1}


def test_strike_metrics_separate_scheduled_accuracy_and_timing() -> None:
    metrics = StrikeMetrics(MetricThresholds(strike_sample_window_s=0.10))
    metrics.add_sample(
        0,
        "left",
        time_offset_s=-0.02,
        position_error_m=0.08,
        velocity_error_mps=0.40,
        orientation_error_rad=0.04,
    )
    metrics.add_sample(
        0,
        "left",
        time_offset_s=0.0,
        position_error_m=0.03,
        velocity_error_mps=0.45,
        orientation_error_rad=0.04,
    )
    metrics.add_sample(
        0,
        "left",
        time_offset_s=0.02,
        position_error_m=0.01,
        velocity_error_mps=0.70,
        orientation_error_rad=0.04,
    )
    summary = metrics.summary(expected_shots=1)
    shot = summary["shots"][0]
    assert shot["scheduled_position_error_m"] == pytest.approx(0.03)
    assert shot["timing_error_s"] == pytest.approx(0.02)
    assert shot["minimum_position_error_m"] == pytest.approx(0.01)
    assert shot["strike_success"] is True


def test_strike_samples_outside_window_are_ignored() -> None:
    metrics = StrikeMetrics()
    metrics.add_sample(
        0,
        "left",
        time_offset_s=0.13,
        position_error_m=0.0,
        velocity_error_mps=0.0,
        orientation_error_rad=0.0,
    )
    with pytest.raises(ValueError, match="no strike-window samples"):
        metrics.shot_summary(0)
