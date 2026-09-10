"""Small, versioned planner delivery surface for the real-robot adapter.

The transport layer remains outside this package.  This module only fixes the
student identity used by the fallback delivery and gives callers one factory
for the four supported positioning planners.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from .config import PlannerConfig
from .core import RelayPlanner
from .rl import FrozenQPolicy
from .strategies import RLStrategy, make_strategy


REAL_PLANNER_NAMES = ("heuristic", "cbf", "reachability", "rl")
V6_PHASE_ORDER = (
    "HIT",
    "POST_DELAY",
    "OUTWARD",
    "OUTWARD_HOLD",
    "RETURN",
    "HOME_HOLD",
)


@dataclass(frozen=True)
class StudentDeliveryProfile:
    """Identity and ABI metadata for a frozen low-level student."""

    name: str
    checkpoint: str
    sha256: str
    iteration: int
    observation_dim: int
    action_dim: int
    phase_order: tuple[str, ...]
    controller_mode: str
    move_teacher_contract: str


V6_STUDENT_PROFILE = StudentDeliveryProfile(
    name="v6_m73000_hold6",
    checkpoint="model_23000.pt",
    sha256="f2d31b802a37c014971bf83a0b9d1a61e2b8727b42ffbed3dd153b7e4d67127d",
    iteration=23000,
    observation_dim=1666,
    action_dim=29,
    phase_order=V6_PHASE_ORDER,
    controller_mode="readiness_gated",
    move_teacher_contract="m73000_hold6",
)

V9_STUDENT_PROFILE = StudentDeliveryProfile(
    name="v9_m14500_timedhandoff",
    checkpoint="handoff/20260819_v9_i19000/checkpoints/student/v9_student_iteration_019000.pt",
    sha256="2c70e278b3d5d4c96e26c20b3d7a6c4943c486c5c30c599b6a25b04ba358eb8a",
    iteration=19000,
    observation_dim=1666,
    action_dim=29,
    phase_order=V6_PHASE_ORDER,
    controller_mode="external_timed",
    move_teacher_contract="v9_widehome_armfix_model14500",
)

ACTIVE_STUDENT_PROFILE = V6_STUDENT_PROFILE

_ALIASES = {
    "heuristic": "heuristic",
    "relay_heuristic": "heuristic",
    "cbf": "cbf",
    "reach": "reachability",
    "viability": "reachability",
    "reachability": "reachability",
    "q": "rl",
    "q_learning": "rl",
    "rl": "rl",
}


def _canonical_strategy(name: str) -> str:
    if not isinstance(name, str):
        raise TypeError("planner strategy name must be a string")
    normalized = name.strip().lower().replace("-", "_")
    try:
        return _ALIASES[normalized]
    except KeyError as error:
        supported = ", ".join(REAL_PLANNER_NAMES)
        raise ValueError(
            f"unknown real-robot planner {name!r}; choose one of: {supported}"
        ) from error


def make_real_robot_planner(
    strategy: str,
    config: PlannerConfig | None = None,
    *,
    q_policy: FrozenQPolicy | None = None,
    minimum_visits: int | None = None,
) -> RelayPlanner:
    """Construct one of the four planners used by the real-input adapter."""

    canonical = _canonical_strategy(strategy)
    if canonical == "rl":
        implementation = RLStrategy(policy=q_policy, minimum_visits=minimum_visits)
    else:
        implementation = make_strategy(canonical)
    return RelayPlanner(config=config, strategy=implementation)


def make_real_robot_planners(
    config: PlannerConfig | None = None,
    *,
    q_policy: FrozenQPolicy | None = None,
    minimum_visits: int | None = None,
) -> Mapping[str, RelayPlanner]:
    """Construct all supported planners with one shared immutable config."""

    return {
        name: make_real_robot_planner(
            name,
            config=config,
            q_policy=q_policy,
            minimum_visits=minimum_visits,
        )
        for name in REAL_PLANNER_NAMES
    }


__all__ = [
    "ACTIVE_STUDENT_PROFILE",
    "REAL_PLANNER_NAMES",
    "StudentDeliveryProfile",
    "V6_PHASE_ORDER",
    "V6_STUDENT_PROFILE",
    "V9_STUDENT_PROFILE",
    "make_real_robot_planner",
    "make_real_robot_planners",
]
