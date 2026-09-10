"""Transport-neutral doubles relay planner.

The package deliberately sits above the frozen single-player policy ABI.  It
only produces the planner commands that the existing deploy adapter already
consumes.
"""

from .config import PlannerConfig
from .core import RelayPlanner
from .fixed_relay import FixedRelayPlanner
from .models import BallPrediction, PlanResult, RelayStage, RobotCommand, RobotFeedback
from .metrics import (
    DoublesSafetyMetrics,
    MetricThresholds,
    RelayStageMetrics,
    StrikeMetrics,
)
from .centralized import (
    ACTION_VERSION,
    CENTRALIZED_OBSERVATION_SCHEMA_HASH,
    OBSERVATION_NAMES_V2,
    BallPolicyState,
    CentralizedRelaySupervisor,
    DoublesCommandAction,
    DoublesCommandBatch,
    DoublesCommandConfig,
    DoublesObservationConfig,
    DoublesPolicyObservation,
    DoublesRobotCommand,
    HitRequest,
    RobotPolicyState,
    centralized_observation_schema,
    guard_doubles_action,
)
from .real_inputs import (
    CONTROLLER_PHASE_NAMES,
    G1BaseEstimate,
    G1ControllerTelemetry,
    G1_DOF,
    G1_IMU_SIZE,
    G1RobotInput,
    REAL_INPUT_CONTRACT_VERSION,
    RealBallPrediction,
    RealInputAdapter,
    UnitreeG1LowState,
)
from .deployment import (
    ACTIVE_STUDENT_PROFILE,
    REAL_PLANNER_NAMES,
    StudentDeliveryProfile,
    V6_PHASE_ORDER,
    V6_STUDENT_PROFILE,
    V9_STUDENT_PROFILE,
    make_real_robot_planner,
    make_real_robot_planners,
)

__all__ = [
    "BallPrediction",
    "DoublesSafetyMetrics",
    "MetricThresholds",
    "PlanResult",
    "PlannerConfig",
    "RelayStageMetrics",
    "RelayPlanner",
    "FixedRelayPlanner",
    "RelayStage",
    "RobotCommand",
    "RobotFeedback",
    "StrikeMetrics",
    "ACTION_VERSION",
    "CENTRALIZED_OBSERVATION_SCHEMA_HASH",
    "OBSERVATION_NAMES_V2",
    "BallPolicyState",
    "CentralizedRelaySupervisor",
    "DoublesCommandAction",
    "DoublesCommandBatch",
    "DoublesCommandConfig",
    "DoublesObservationConfig",
    "DoublesPolicyObservation",
    "DoublesRobotCommand",
    "HitRequest",
    "RobotPolicyState",
    "centralized_observation_schema",
    "guard_doubles_action",
    "CONTROLLER_PHASE_NAMES",
    "G1BaseEstimate",
    "G1ControllerTelemetry",
    "G1_DOF",
    "G1_IMU_SIZE",
    "G1RobotInput",
    "REAL_INPUT_CONTRACT_VERSION",
    "RealBallPrediction",
    "RealInputAdapter",
    "UnitreeG1LowState",
    "ACTIVE_STUDENT_PROFILE",
    "REAL_PLANNER_NAMES",
    "StudentDeliveryProfile",
    "V6_PHASE_ORDER",
    "V6_STUDENT_PROFILE",
    "V9_STUDENT_PROFILE",
    "make_real_robot_planner",
    "make_real_robot_planners",
]
