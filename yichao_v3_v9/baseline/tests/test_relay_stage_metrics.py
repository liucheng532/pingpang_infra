from __future__ import annotations

import pytest

from doubles_planner.metrics import RelayStageMetrics


def test_stage_metrics_measure_early_reservation_atomic_clear_and_handoff() -> None:
    metrics = RelayStageMetrics(
        minimum_reservation_lead_s=0.80,
        minimum_commit_lead_s=0.35,
        sample_period_s=0.02,
    )
    metrics.record_reservation(
        "shot-0",
        "left",
        now_s=1.0,
        time_to_strike_s=1.20,
    )
    # A pending reservation is legal before the active token flips.
    metrics.record_reservation(
        "shot-1",
        "right",
        now_s=1.1,
        time_to_strike_s=2.10,
    )
    metrics.record_commit(
        "shot-0",
        "left",
        now_s=1.4,
        time_to_strike_s=0.80,
        peer_clear_at_commit=True,
        base_gap_m=0.92,
        base_ttc_s=None,
        hand_gap_m=0.48,
    )
    metrics.record_peer_clear_started("shot-0", now_s=1.42)
    metrics.add_phase_sample(
        now_s=1.4,
        phases={"left": "HIT", "right": "OUTWARD"},
        command_y={"left": -0.35, "right": 0.68},
        base_y={"left": -0.35, "right": 0.52},
    )
    metrics.add_phase_sample(
        now_s=2.2,
        phases={"left": "POST_DELAY", "right": "OUTWARD"},
        command_y={"left": -0.35, "right": 0.68},
        base_y={"left": -0.35, "right": 0.66},
    )
    metrics.add_phase_sample(
        now_s=2.22,
        phases={"left": "OUTWARD", "right": "OUTWARD"},
        command_y={"left": -0.55, "right": 0.68},
        base_y={"left": -0.35, "right": 0.67},
    )
    metrics.record_strike("shot-0", "left", now_s=2.2, verified=True)
    metrics.record_handoff("shot-0", now_s=2.3)

    summary = metrics.summary()
    assert summary["shots_reserved"] == 2
    assert summary["shots_committed"] == 1
    assert summary["turn_token"] == "right"
    assert summary["token_transitions"] == 1
    assert summary["token_violation_count"] == 0
    assert summary["lifecycle_violation_count"] == 0
    assert summary["reservation_lead"]["minimum_s"] == pytest.approx(1.2)
    assert summary["reservation_to_commit"]["mean_s"] == pytest.approx(0.4)
    assert summary["commit_tts"]["mean_s"] == pytest.approx(0.8)
    assert summary["commit_slack"]["minimum_s"] == pytest.approx(0.45)
    assert summary["peer_clear_at_commit_rate"] == 1.0
    assert summary["peer_clear_start_latency"]["mean_s"] == pytest.approx(0.02)
    assert summary["commit_base_gap"]["minimum_m"] == pytest.approx(0.92)
    assert summary["commit_base_ttc"]["samples"] == 0
    assert summary["commit_hand_gap"]["minimum_m"] == pytest.approx(0.48)
    assert summary["phase_interruption_count"] == 0
    assert summary["protected_phase_retarget_count"] == 0
    assert summary["simultaneous_hit_steps"] == 0


def test_stage_metrics_expose_token_clear_phase_and_retarget_failures() -> None:
    metrics = RelayStageMetrics(
        minimum_reservation_lead_s=0.80,
        minimum_commit_lead_s=0.40,
        sample_period_s=0.02,
        retarget_deadband_m=0.01,
    )
    metrics.record_reservation(
        0,
        "right",
        now_s=0.0,
        time_to_strike_s=0.30,
    )
    metrics.record_commit(
        0,
        "right",
        now_s=0.1,
        time_to_strike_s=0.10,
        peer_clear_at_commit=False,
        base_gap_m=0.44,
        base_ttc_s=0.08,
        hand_gap_m=0.18,
    )
    metrics.add_phase_sample(
        now_s=0.1,
        phases={"left": "HIT", "right": "HOME_HOLD"},
        command_y={"left": -0.40, "right": 0.40},
        base_y={"left": -0.50, "right": 0.50},
    )
    live = metrics.add_phase_sample(
        now_s=0.12,
        phases={"left": "HIT", "right": "HIT"},
        command_y={"left": -0.60, "right": 0.40},
        base_y={"left": -0.50, "right": 0.50},
    )
    interrupted = metrics.add_phase_sample(
        now_s=0.14,
        phases={"left": "OUTWARD", "right": "HIT"},
        command_y={"left": -0.60, "right": 0.40},
        base_y={"left": -0.50, "right": 0.50},
    )

    summary = metrics.summary()
    assert live["simultaneous_hit"] is True
    assert live["retargets"] == 1
    assert live["retarget_reversals"] == 1
    assert interrupted["phase_interruptions"] == 1
    assert summary["token_violations"] == {
        "commit_token": 1,
        "reservation_order": 1,
    }
    assert summary["late_commit_count"] == 1
    assert summary["reservation_lead_violation_count"] == 1
    assert summary["peer_clear_at_commit_rate"] == 0.0
    assert summary["simultaneous_hit_steps"] == 1
    assert summary["simultaneous_hit_entries"] == 1
    assert summary["simultaneous_hit_duration_s"] == pytest.approx(0.02)
    assert summary["phase_interruptions"] == {
        "left:HIT->OUTWARD:observed": 1,
    }
    assert summary["protected_phase_retarget_count"] == 1
    assert summary["retarget_reversal_count"] == 1
    assert summary["commit_base_gap"]["minimum_m"] == pytest.approx(0.44)
    assert summary["commit_base_ttc"]["minimum_s"] == pytest.approx(0.08)
    assert summary["commit_hand_gap"]["minimum_m"] == pytest.approx(0.18)


def test_stage_token_advances_only_after_verified_strike() -> None:
    metrics = RelayStageMetrics()
    metrics.record_reservation(0, "left", now_s=0.0, time_to_strike_s=1.0)
    metrics.record_commit(
        0,
        "left",
        now_s=0.2,
        time_to_strike_s=0.8,
        peer_clear_at_commit=True,
    )
    metrics.record_strike(0, "left", now_s=1.0, verified=False)
    assert metrics.turn_token == "left"

    metrics.record_strike(0, "left", now_s=1.02, verified=True)
    assert metrics.turn_token == "right"
    assert metrics.summary()["lifecycle_violations"] == {"unverified_strike": 1}


def test_stage_metric_inputs_are_validated_without_affecting_safety_metrics() -> None:
    metrics = RelayStageMetrics()
    with pytest.raises(ValueError, match="phases must contain exactly"):
        metrics.add_phase_sample(now_s=0.0, phases={"left": "HIT"})
    with pytest.raises(TypeError, match="peer_clear_at_commit"):
        metrics.record_commit(
            0,
            "left",
            now_s=0.0,
            time_to_strike_s=0.5,
            peer_clear_at_commit=1,  # type: ignore[arg-type]
        )
