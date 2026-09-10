from __future__ import annotations


OUTPUT_SUFFIXES = (
    "predicted_ball_position",
    "predicted_ball_velocity",
    "predicted_ball_predict_time",
    "predicted_racket_normal",
    "predicted_racket_velocity",
    "desired_base_position",
)


def output_topics(robot: str) -> dict[str, str]:
    name = robot.strip("/")
    if not name:
        raise ValueError("robot namespace must not be empty")
    return {suffix: f"/{name}/{suffix}" for suffix in OUTPUT_SUFFIXES}


def feedback_topic(robot: str) -> str:
    name = robot.strip("/")
    if not name:
        raise ValueError("robot namespace must not be empty")
    return f"/{name}/torso_pose_origin"
