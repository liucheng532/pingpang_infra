from __future__ import annotations

import numpy as np

from .config import PlannerConfig


def desired_base_y(config: PlannerConfig, robot: str, strike_y: float) -> float:
    """Return the base center that places ``robot``'s racket at ``strike_y``."""
    try:
        robot_index = config.robots.index(robot)
    except ValueError as error:
        raise ValueError(f"unknown robot: {robot}") from error
    return float(
        np.clip(
            float(strike_y) - config.racket_reach_y[robot_index],
            config.workspace_y[0],
            config.workspace_y[1],
        )
    )
