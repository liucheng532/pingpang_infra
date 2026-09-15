"""Independent V7 model190 + frozen CBF adapter for V11 i42500."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / "models"
CONFIG_DIR = ROOT / "config"

PLANNER_IDENTITY = "v7-model190-cbf-v11-i42500-f052ebf"
HANDOFF_COMMIT = "f052ebfc51239e83ff58c2067bbb30dd77292e59"
TRAINING_AUDIT_COMMIT = "1a8e81d0c9ed2a0a027074d3ab2ddfb44e0f4fbc"
STATUS_TOPIC = "/doubles/yichao_v7/status"
COMMAND_SCHEMA = "v9-planner-command-v1"
STATE_SCHEMA = "v9-robot-state-v1"

# Never derive these identities from the Fixed runtime's ROBOT_ORDER.  Its
# transport order is right/left, while the actor checkpoint is left/right.
MODEL_ORDER = ("left", "right")
MODEL_TO_ROS = {"left": "table_left", "right": "table_right"}
ROS_TO_MODEL = {value: key for key, value in MODEL_TO_ROS.items()}
MODEL_TO_HOST = {"left": "198", "right": "66"}
ACTION_INDEX = {"left": 0, "right": 1}

HOME_PAIR = (0.35, -0.35)
WORKSPACE_Y = (-0.9, 0.9)
MINIMUM_GAP_M = 0.50
CBF_RISK_THRESHOLD = 0.3027352380752564
CBF_CANDIDATE_RADIUS_NORMALIZED = 1.25
CBF_GRID_POINTS = 51
CBF_HITTER_WEIGHT = 100.0
CBF_NON_HITTER_WEIGHT = 1.0
