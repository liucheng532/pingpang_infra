from __future__ import annotations

import numpy as np
import pytest

from doubles_planner import BallPrediction, PlannerConfig, RelayPlanner, RobotFeedback


FAR_ORIGIN_WORLD = np.asarray([3.54, 0.0, 0.0], dtype=float)
RZ_PI = np.diag([-1.0, -1.0, 1.0])


def far_world_position_to_local(position_world: np.ndarray) -> np.ndarray:
    return RZ_PI @ (np.asarray(position_world, dtype=float) - FAR_ORIGIN_WORLD)


def far_world_vector_to_local(vector_world: np.ndarray) -> np.ndarray:
    return RZ_PI @ np.asarray(vector_world, dtype=float)


def prediction(
    shot_id: str,
    time_to_strike: float,
    now: float,
    position: np.ndarray,
    velocity: np.ndarray,
) -> BallPrediction:
    return BallPrediction(
        position=position,
        velocity=velocity,
        time_to_strike=time_to_strike,
        timestamp=now,
        racket_normal=[0.93, 0.0, 0.36],
        racket_velocity=[1.9, 0.0, 0.7],
        shot_id=shot_id,
    )


def feedback(
    robots: tuple[str, str],
    now: float,
    first_base_xy: np.ndarray,
    second_base_xy: np.ndarray,
) -> dict[str, RobotFeedback]:
    return {
        robots[0]: RobotFeedback(robots[0], first_base_xy, timestamp=now),
        robots[1]: RobotFeedback(robots[1], second_base_xy, timestamp=now),
    }


@pytest.mark.parametrize(
    ("robots", "first_position", "first_velocity", "first_base", "second_base"),
    [
        (
            ("A1", "A2"),
            np.asarray([0.45, -0.40, 1.05]),
            np.asarray([-3.2, 0.10, -0.25]),
            np.asarray([0.0, -0.35]),
            np.asarray([0.0, 0.35]),
        ),
        (
            ("B1", "B2"),
            far_world_position_to_local(np.asarray([3.09, 0.40, 1.05])),
            far_world_vector_to_local(np.asarray([3.2, -0.10, -0.25])),
            far_world_position_to_local(np.asarray([3.54, 0.35, 0.0]))[:2],
            far_world_position_to_local(np.asarray([3.54, -0.35, 0.0]))[:2],
        ),
    ],
)
def test_each_team_releases_a_shot_before_rotating_to_teammate(
    robots: tuple[str, str],
    first_position: np.ndarray,
    first_velocity: np.ndarray,
    first_base: np.ndarray,
    second_base: np.ndarray,
) -> None:
    config = PlannerConfig(robots=robots)
    planner = RelayPlanner(config=config, strategy="relay_heuristic")

    now = 1.0
    states = feedback(robots, now, first_base, second_base)
    first = planner.plan(
        prediction("shot-1", 0.50, now, first_position, first_velocity),
        states,
        now,
    )
    assert first.hitter == robots[0]
    assert first.next_hitter == robots[1]

    now += 0.1
    states = feedback(robots, now, first_base, second_base)
    before_release = planner.plan(
        prediction(
            "shot-1",
            config.shot_release_time + 0.001,
            now,
            first_position,
            first_velocity,
        ),
        states,
        now,
    )
    assert before_release.hitter == robots[0]
    assert before_release.shot_id == "shot-1"

    now += 0.1
    states = feedback(robots, now, first_base, second_base)
    released = planner.plan(
        prediction(
            "shot-1",
            config.shot_release_time,
            now,
            first_position,
            first_velocity,
        ),
        states,
        now,
    )
    assert released.hitter == robots[0]
    assert released.shot_id == "shot-1"

    now += 0.1
    states = feedback(robots, now, first_base, second_base)
    second = planner.plan(
        prediction("shot-2", 0.50, now, first_position, first_velocity),
        states,
        now,
    )
    assert second.hitter == robots[1]
    assert second.next_hitter == robots[0]
    assert second.shot_id == "shot-2"


def test_far_side_rigid_frame_maps_world_y_to_negative_local_y() -> None:
    position_world = np.asarray([3.09, 0.42, 1.08])
    velocity_world = np.asarray([3.2, -0.3, -0.2])

    position_local = far_world_position_to_local(position_world)
    velocity_local = far_world_vector_to_local(velocity_world)

    np.testing.assert_allclose(position_local, [0.45, -0.42, 1.08], atol=1.0e-12)
    np.testing.assert_allclose(velocity_local, [-3.2, 0.3, -0.2], atol=1.0e-12)
    assert position_local[1] == pytest.approx(-position_world[1])
    assert velocity_local[1] == pytest.approx(-velocity_world[1])
