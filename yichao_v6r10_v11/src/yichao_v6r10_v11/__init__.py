"""Independent V6R10 model630 planner adapter for the V11 i42500 controller."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / "models"
CONFIG_DIR = ROOT / "config"

PLANNER_IDENTITY = "v6r10-model630-v11-i42500"
STATUS_TOPIC = "/doubles/yichao_v6r10/status"
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

