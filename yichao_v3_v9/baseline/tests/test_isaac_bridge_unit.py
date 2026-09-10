from __future__ import annotations

import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import torch

from doubles_planner.isaac_bridge import IsaacControllerBridge, mirror_command_sources
from doubles_planner.models import BallPrediction, PlanResult, RobotCommand
from doubles_planner.mirror import G1_JOINT_NAMES


class _CommandStub:
    def __init__(self) -> None:
        self.device = "cpu"
        self.robot_pelvis_pos_origin = np.asarray([[0.0, 0.0, 0.0]])
        self.HIT = 0
        self.POST_DELAY = 1
        self.OUTWARD = 2
        self.OUTWARD_HOLD = 3
        self.RETURN = 4
        self.HOME_HOLD = 5
        self.doubles_state = torch.tensor([self.HOME_HOLD])
        self.state_elapsed_s = torch.tensor([1.0])
        self.cfg = SimpleNamespace(
            home_target_y_range=(0.30, 0.40),
            outward_target_y=0.78,
            stable_position_error_m=0.06,
        )
        self.target_y = torch.tensor([0.0])
        self.home_target_y = torch.tensor([0.0])
        self.hit_arguments = None
        self.base_arguments = None
        self.outward_arguments = None

    def set_external_hit(self, **arguments: object) -> None:
        self.hit_arguments = arguments

    def set_external_base_target(self, **arguments: object) -> None:
        self.base_arguments = arguments

    def set_external_outward_target(self, **arguments: object) -> None:
        self.outward_arguments = arguments


class IsaacBridgeUnitTests(unittest.TestCase):
    @staticmethod
    def _robot_command(
        robot: str,
        desired_y: float,
        trajectory_y: float,
    ) -> RobotCommand:
        return RobotCommand(
            robot=robot,
            active=False,
            predicted_ball_position=[0.45, 0.0, 1.0],
            predicted_ball_velocity=[0.0, 0.0, 0.0],
            predicted_ball_predict_time=-0.5,
            predicted_racket_normal=[1.0, 0.0, 0.0],
            predicted_racket_velocity=[0.0, 0.0, 0.0],
            desired_base_position=[0.0, desired_y],
            trajectory_base_position=[0.0, trajectory_y],
        )

    @classmethod
    def _committed_plan(cls, token: str) -> PlanResult:
        return PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="relay_heuristic",
            commands={
                "left": replace(
                    cls._robot_command("left", -0.4, -0.4),
                    active=True,
                    role="hit",
                ),
                "right": replace(
                    cls._robot_command("right", 0.7, 0.7),
                    role="clear",
                ),
            },
            diagnostics={"teammate_stage_mode": "outward", "preview_y": 0.35},
            relay_stage="committed",
            commit_token=token,
            commit_requested=True,
        )

    @staticmethod
    def _prediction() -> BallPrediction:
        return BallPrediction(
            position=[0.45, 0.0, 1.0],
            velocity=[-3.2, 0.0, 0.0],
            time_to_strike=0.5,
            timestamp=1.0,
            racket_normal=[1.0, 0.0, 0.0],
            racket_velocity=[1.9, 0.0, 0.0],
        )

    @staticmethod
    def _add_feedback_state(command: _CommandStub) -> None:
        command.robot_pelvis_pos_origin = torch.zeros(1, 3)
        command.robot_pelvis_body_index = 0
        command.robot = SimpleNamespace(
            data=SimpleNamespace(body_lin_vel_w=torch.zeros(1, 1, 3))
        )

    def test_mirrored_move_bank_preserves_direction_classes(self) -> None:
        body_names = ("pelvis", "left_hand", "right_hand", "racket")
        joint_values = torch.arange(2 * 2 * 29, dtype=torch.float32).reshape(2, 2, 29)
        body_vectors = torch.zeros(2, 2, len(body_names), 3)
        body_quaternions = torch.zeros(2, 2, len(body_names), 4)
        body_quaternions[..., 0] = 1.0
        hit = SimpleNamespace(
            joint_pos_all=joint_values.clone(),
            joint_vel_all=joint_values.clone(),
            body_pos_all=body_vectors.clone(),
            body_quat_all=body_quaternions.clone(),
            motion_targets=torch.tensor([[0.0, -0.4, 0.0], [0.0, 0.4, 0.0]]),
            motion_labels=torch.tensor([False, True]),
        )
        command = SimpleNamespace(
            device="cpu",
            joint_pos_all=joint_values.clone(),
            joint_vel_all=joint_values.clone(),
            body_pos_all=body_vectors.clone(),
            body_quat_all=body_quaternions.clone(),
            body_lin_vel_all=body_vectors.clone(),
            body_ang_vel_all=body_vectors.clone(),
            motion_targets_t=torch.tensor([[0.0, -0.4, 0.0], [0.0, 0.4, 0.0]]),
            motion_labels_t=torch.tensor([False, True]),
            _motion_time_totals_all=torch.tensor([2, 2]),
            motion_pelvis_body_index=0,
            hit=hit,
        )

        mirror_command_sources(command, G1_JOINT_NAMES, body_names)

        torch.testing.assert_close(command.motion_targets_t[:, 1], torch.tensor([0.4, -0.4]))
        torch.testing.assert_close(command.motion_labels_t, torch.tensor([False, True]))
        torch.testing.assert_close(command._outbound_motion_indices, torch.tensor([0]))
        torch.testing.assert_close(command._return_motion_indices, torch.tensor([1]))
        self.assertLess(command.motion_targets_t[command._return_motion_indices, 1].item(), 0.0)
        self.assertGreater(command.motion_targets_t[command._outbound_motion_indices, 1].item(), 0.0)

    def test_numpy_vectors_are_batched_before_tensor_conversion(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        bridge = IsaacControllerBridge({"left": left, "right": right})
        prediction = BallPrediction(
            position=[0.4, -0.2, 1.1],
            velocity=[-3.2, 0.1, -0.3],
            time_to_strike=0.45,
            timestamp=1.0,
            racket_normal=[0.9, 0.0, 0.4],
            racket_velocity=[1.8, -0.2, 0.6],
        )

        converted: list[object] = []

        def fake_tensor(value: object, device: object, dtype: str | None = None) -> object:
            del device, dtype
            converted.append(value)
            return value

        with patch("doubles_planner.isaac_bridge._tensor", side_effect=fake_tensor):
            bridge.inject_hit("left", prediction)
            bridge.inject_base_target(
                "left",
                np.asarray([0.0, -0.35]),
                return_target_y=-0.4,
            )

        self.assertIsNotNone(left.hit_arguments)
        self.assertIsNotNone(left.base_arguments)
        for argument in ("racket_target", "target_velocity", "ball_velocity"):
            value = left.hit_arguments[argument]
            self.assertIsInstance(value, np.ndarray)
            self.assertEqual(value.shape, (1, 3))
        target_xy = left.base_arguments["target_xy"]
        self.assertIsInstance(target_xy, np.ndarray)
        self.assertEqual(target_xy.shape, (1, 2))
        self.assertFalse(
            any(
                isinstance(value, list) and value and isinstance(value[0], np.ndarray)
                for value in converted
            )
        )

    def test_commit_is_exactly_once_and_starts_peer_clear_first(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        bridge = IsaacControllerBridge({"left": left, "right": right})
        prediction = BallPrediction(
            position=[0.45, -0.1, 1.0],
            velocity=[-3.2, 0.0, -0.2],
            time_to_strike=0.5,
            timestamp=1.0,
            racket_normal=[0.93, 0.0, 0.36],
            racket_velocity=[1.9, 0.0, 0.7],
        )
        commands = {
            "left": replace(
                self._robot_command("left", -0.4, -0.4),
                active=True,
                role="hit",
            ),
            "right": replace(
                self._robot_command("right", 0.7, 0.7),
                role="clear",
            ),
        }
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="relay_heuristic",
            commands=commands,
            diagnostics={"teammate_stage_mode": "outward", "preview_y": 0.35},
            relay_stage="committed",
            commit_token="token-1",
            commit_requested=True,
        )

        accepted = bridge.commit(plan, prediction)
        duplicate = bridge.commit(plan, prediction)

        self.assertTrue(accepted.accepted)
        self.assertFalse(accepted.duplicate)
        self.assertTrue(duplicate.accepted)
        self.assertTrue(duplicate.duplicate)
        self.assertIsNotNone(left.hit_arguments)
        self.assertIsNotNone(right.base_arguments)

    def test_commit_rejects_when_peer_is_still_hit_locked(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        right.doubles_state[0] = right.POST_DELAY
        bridge = IsaacControllerBridge({"left": left, "right": right})
        prediction = BallPrediction(
            position=[0.45, 0.0, 1.0],
            velocity=[-3.2, 0.0, 0.0],
            time_to_strike=0.5,
            timestamp=1.0,
            racket_normal=[1.0, 0.0, 0.0],
            racket_velocity=[1.9, 0.0, 0.0],
        )
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="relay_heuristic",
            commands={
                "left": replace(
                    self._robot_command("left", -0.4, -0.4),
                    active=True,
                    role="hit",
                ),
                "right": replace(
                    self._robot_command("right", 0.7, 0.7),
                    role="clear",
                ),
            },
            relay_stage="committed",
            commit_token="token-2",
            commit_requested=True,
        )

        result = bridge.commit(plan, prediction)

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "peer_hit_locked")
        self.assertIsNone(left.hit_arguments)

    def test_locomotion_preemption_defaults_to_v6_home_hold_only(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        self._add_feedback_state(left)
        self._add_feedback_state(right)
        bridge = IsaacControllerBridge({"left": left, "right": right})

        for phase in (left.OUTWARD, left.OUTWARD_HOLD, left.RETURN):
            with self.subTest(phase=phase):
                left.doubles_state[0] = phase
                self.assertFalse(bridge.feedback(now=1.0)["left"].ready)
                result = bridge.commit(self._committed_plan(f"v6-{phase}"), self._prediction())
                self.assertFalse(result.accepted)
                self.assertEqual(result.reason, "hitter_not_home_hold")

        left.doubles_state[0] = left.HOME_HOLD
        self.assertTrue(bridge.feedback(now=1.0)["left"].ready)

    def test_locomotion_preemption_accepts_released_phases_but_not_hit_lock(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        self._add_feedback_state(left)
        self._add_feedback_state(right)
        bridge = IsaacControllerBridge(
            {"left": left, "right": right},
            allow_locomotion_preemption=True,
        )
        released = (left.OUTWARD, left.OUTWARD_HOLD, left.RETURN, left.HOME_HOLD)

        for phase in released:
            with self.subTest(side="hitter", phase=phase):
                left.doubles_state[0] = phase
                right.doubles_state[0] = right.HOME_HOLD
                self.assertTrue(bridge.feedback(now=1.0)["left"].ready)
                result = bridge.commit(
                    self._committed_plan(f"preempt-hitter-{phase}"),
                    self._prediction(),
                )
                self.assertTrue(result.accepted)

        for phase in released:
            with self.subTest(side="peer", phase=phase):
                left.doubles_state[0] = left.HOME_HOLD
                right.doubles_state[0] = phase
                self.assertTrue(bridge.feedback(now=1.0)["right"].ready)
                result = bridge.commit(
                    self._committed_plan(f"preempt-peer-{phase}"),
                    self._prediction(),
                )
                self.assertTrue(result.accepted)

        for phase in (left.HIT, left.POST_DELAY):
            with self.subTest(side="hitter", locked_phase=phase):
                left.doubles_state[0] = phase
                right.doubles_state[0] = right.HOME_HOLD
                self.assertFalse(bridge.feedback(now=1.0)["left"].ready)
                result = bridge.commit(
                    self._committed_plan(f"locked-hitter-{phase}"),
                    self._prediction(),
                )
                self.assertFalse(result.accepted)
                self.assertEqual(result.reason, "hitter_not_home_hold")

            with self.subTest(side="peer", locked_phase=phase):
                left.doubles_state[0] = left.HOME_HOLD
                right.doubles_state[0] = phase
                self.assertFalse(bridge.feedback(now=1.0)["right"].ready)
                result = bridge.commit(
                    self._committed_plan(f"locked-peer-{phase}"),
                    self._prediction(),
                )
                self.assertFalse(result.accepted)
                self.assertEqual(result.reason, "peer_hit_locked")

    def test_non_emergency_cbf_streams_waypoint_only_when_active(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        left.robot_pelvis_pos_origin[0, 1] = -0.35
        right.robot_pelvis_pos_origin[0, 1] = 0.35
        bridge = IsaacControllerBridge({"left": left, "right": right})
        commands = {
            "left": self._robot_command("left", -0.345, -0.70),
            "right": self._robot_command("right", 0.355, 0.70),
        }
        nominal_plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="cbf_mpc",
            commands=commands,
            diagnostics={"cbf_active": False},
        )

        bridge.apply_positioning(nominal_plan)

        self.assertIsNotNone(right.base_arguments)
        torch.testing.assert_close(
            right.base_arguments["target_xy"],
            torch.tensor([[0.0, 0.70]]),
        )

        right.base_arguments = None
        filtered_plan = PlanResult(
            sequence=2,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="cbf_mpc",
            commands=commands,
            diagnostics={
                "cbf_active": True,
                "cbf_controlled_robot": "right",
            },
        )
        bridge.apply_positioning(filtered_plan)
        self.assertIsNotNone(right.base_arguments)
        torch.testing.assert_close(
            right.base_arguments["target_xy"],
            torch.tensor([[0.0, 0.40]]),
        )
        self.assertTrue(right.base_arguments["safety_override"])
        torch.testing.assert_close(
            right.base_arguments["return_target_y"],
            torch.tensor([0.40]),
        )

    def test_non_emergency_safety_does_not_interrupt_hit_phase(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        left.doubles_state[0] = left.HIT
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="cbf",
            commands={
                "left": self._robot_command("left", -0.50, -0.70),
                "right": self._robot_command("right", 0.50, 0.70),
            },
            diagnostics={
                "safety_active": True,
                "safety_controlled_robot": "left",
            },
        )

        bridge.apply_positioning(plan)

        self.assertIsNone(left.base_arguments)

    def test_safety_retarget_reversal_is_rolled_back_and_counted(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        right.doubles_state[0] = right.OUTWARD
        right.target_y[0] = 0.72
        right.home_target_y[0] = 0.35

        def reject_reversal(**arguments: object) -> None:
            right.target_y[0] = 0.10
            right.home_target_y[0] = 0.10
            raise ValueError(
                "External base target would reverse the selected motion reference"
            )

        right.set_external_base_target = reject_reversal
        bridge = IsaacControllerBridge({"left": left, "right": right})

        result = bridge.inject_base_target(
            "right",
            np.asarray([0.0, 0.50]),
            return_target_y=0.40,
            safety_override=True,
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "selected_motion_reference_reversal")
        torch.testing.assert_close(right.target_y, torch.tensor([0.72]))
        torch.testing.assert_close(right.home_target_y, torch.tensor([0.35]))
        self.assertEqual(
            bridge.base_target_diagnostics(),
            {
                "rejection_count": 1,
                "rejection_reasons": {"selected_motion_reference_reversal": 1},
                "rejection_by_robot": {"right": 1},
                "rejection_by_phase": {"right:OUTWARD": 1},
            },
        )

    def test_nominal_retarget_reversal_remains_fatal(self) -> None:
        left = _CommandStub()
        right = _CommandStub()

        def reject_reversal(**arguments: object) -> None:
            raise ValueError(
                "External base target would reverse the selected motion reference"
            )

        right.set_external_base_target = reject_reversal
        bridge = IsaacControllerBridge({"left": left, "right": right})

        with self.assertRaisesRegex(ValueError, "right base target failed"):
            bridge.inject_base_target(
                "right",
                np.asarray([0.0, 0.50]),
                return_target_y=0.40,
                safety_override=False,
            )

    def test_pre_hit_outward_stage_streams_safety_waypoint(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        right.doubles_state[0] = right.RETURN
        right.robot_pelvis_pos_origin[0, 1] = 0.44
        right.target_y[0] = 0.78
        right.home_target_y[0] = 0.35
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="cbf",
            commands={
                "left": self._robot_command("left", -0.40, -0.40),
                "right": self._robot_command("right", 0.46, 0.65),
            },
            diagnostics={
                "safety_active": True,
                "safety_controlled_robot": "right",
                "teammate_stage_mode": "outward",
                "preview_y": 0.18,
            },
        )

        bridge.apply_positioning(plan)

        self.assertIsNotNone(right.base_arguments)
        torch.testing.assert_close(
            right.base_arguments["target_xy"],
            torch.tensor([[0.0, 0.46]]),
        )
        torch.testing.assert_close(
            right.base_arguments["return_target_y"],
            torch.tensor([0.40]),
        )
        self.assertTrue(right.base_arguments["safety_override"])

    def test_pre_hit_nominal_outward_stage_preserves_active_return(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        right.doubles_state[0] = right.RETURN
        right.robot_pelvis_pos_origin[0, 1] = 0.44
        right.target_y[0] = 0.78
        right.home_target_y[0] = 0.35
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="relay_heuristic",
            commands={
                "left": self._robot_command("left", -0.40, -0.40),
                "right": self._robot_command("right", 0.65, 0.65),
            },
            diagnostics={
                "teammate_stage_mode": "outward",
                "preview_y": 0.18,
            },
        )

        bridge.apply_positioning(plan)

        self.assertIsNone(right.base_arguments)

    def test_pre_hit_outward_stage_queues_target_during_post_delay(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        right.doubles_state[0] = right.POST_DELAY
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="relay_heuristic",
            commands={
                "left": self._robot_command("left", -0.40, -0.40),
                "right": self._robot_command("right", 0.65, 0.65),
            },
            diagnostics={
                "teammate_stage_mode": "outward",
                "preview_y": 0.18,
            },
        )

        bridge.apply_positioning(plan)

        self.assertIsNotNone(right.outward_arguments)
        torch.testing.assert_close(
            right.outward_arguments["target_y"],
            torch.tensor([0.65]),
        )
        self.assertIsNone(right.base_arguments)

    def test_post_hit_positioning_uses_outward_hook(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        left.doubles_state[0] = left.POST_DELAY
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="post_hit",
            strategy="cbf_mpc",
            commands={
                "left": self._robot_command("left", -0.36, -0.78),
                "right": self._robot_command("right", 0.36, 0.40),
            },
            diagnostics={"cbf_active": False},
        )

        bridge.apply_positioning(plan)

        self.assertIsNotNone(left.outward_arguments)
        torch.testing.assert_close(
            left.outward_arguments["target_y"],
            torch.tensor([-0.78]),
        )
        self.assertIsNotNone(right.base_arguments)

    def test_post_hit_positioning_retargets_active_outward_motion(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        left.doubles_state[0] = left.OUTWARD
        left.robot_pelvis_pos_origin[0, 1] = -0.45
        left.target_y[0] = -0.35
        left.home_target_y[0] = -0.40
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="post_hit",
            strategy="cbf_mpc",
            commands={
                "left": self._robot_command("left", -0.50, -0.78),
                "right": self._robot_command("right", 0.35, 0.35),
            },
            diagnostics={"cbf_active": False},
        )

        bridge.apply_positioning(plan)

        self.assertIsNone(left.outward_arguments)
        self.assertIsNotNone(left.base_arguments)
        torch.testing.assert_close(
            left.base_arguments["target_xy"],
            torch.tensor([[0.0, -0.78]]),
        )
        torch.testing.assert_close(
            left.base_arguments["return_target_y"],
            torch.tensor([-0.40]),
        )

    def test_post_hit_next_hitter_streams_safety_during_return(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        left.doubles_state[0] = left.RETURN
        left.robot_pelvis_pos_origin[0, 1] = -0.44
        left.target_y[0] = -0.78
        left.home_target_y[0] = -0.60
        left.cfg.outward_target_y = -0.78
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="right",
            next_hitter="left",
            phase="post_hit",
            strategy="cbf_mpc",
            commands={
                "left": self._robot_command("left", -0.34, -0.20),
                "right": self._robot_command("right", 0.40, 0.78),
            },
            diagnostics={
                "cbf_active": True,
                "cbf_controlled_robot": "both",
            },
        )

        bridge.apply_positioning(plan)

        self.assertIsNotNone(left.base_arguments)
        torch.testing.assert_close(
            left.base_arguments["target_xy"],
            torch.tensor([[0.0, -0.40]]),
        )
        torch.testing.assert_close(
            left.base_arguments["return_target_y"],
            torch.tensor([-0.40]),
        )
        self.assertTrue(left.base_arguments["safety_override"])

    def test_return_safety_waypoint_stays_on_selected_reference_side(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        right.doubles_state[0] = right.RETURN
        right.robot_pelvis_pos_origin[0, 1] = 0.70
        right.target_y[0] = 0.65
        right.home_target_y[0] = 0.40
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="post_hit",
            strategy="cbf",
            commands={
                "left": self._robot_command("left", -0.50, -0.78),
                "right": self._robot_command("right", 0.683, 0.40),
            },
            diagnostics={
                "safety_active": True,
                "safety_controlled_robot": "both",
            },
        )

        bridge.apply_positioning(plan)

        self.assertIsNotNone(right.base_arguments)
        torch.testing.assert_close(
            right.base_arguments["target_xy"],
            torch.tensor([[0.0, 0.65]]),
        )
        torch.testing.assert_close(
            right.base_arguments["return_target_y"],
            torch.tensor([0.40]),
        )
        self.assertTrue(right.base_arguments["safety_override"])

    def test_return_reference_direction_survives_collapsed_home_target(self) -> None:
        command = _CommandStub()
        command.doubles_state[0] = command.RETURN
        command.target_y[0] = 0.58
        command.home_target_y[0] = 0.58
        command.return_motion_index = torch.tensor([0])
        command.motion_targets_t = torch.tensor([[0.0, -0.42, 0.0]])

        goal = IsaacControllerBridge._return_reference_feasible_goal(
            command,
            np.asarray([0.0, 0.66]),
        )

        np.testing.assert_allclose(goal, np.asarray([0.0, 0.58]))

    def test_post_hit_hitter_does_not_reverse_return(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        left.doubles_state[0] = left.RETURN
        left.robot_pelvis_pos_origin[0, 1] = -0.55
        left.home_target_y[0] = -0.35
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="post_hit",
            strategy="cbf_mpc",
            commands={
                "left": self._robot_command("left", -0.50, -0.78),
                "right": self._robot_command("right", 0.35, 0.35),
            },
            diagnostics={"cbf_active": False},
        )

        bridge.apply_positioning(plan)

        self.assertIsNone(left.outward_arguments)
        self.assertIsNone(left.base_arguments)

    def test_next_hitter_does_not_restart_outward_hold(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        right.doubles_state[0] = right.OUTWARD_HOLD
        right.robot_pelvis_pos_origin[0, 1] = 0.68
        right.target_y[0] = 0.70
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="cbf_mpc",
            commands={
                "left": self._robot_command("left", -0.35, -0.35),
                "right": self._robot_command("right", 0.35, 0.90),
            },
            diagnostics={"cbf_active": False},
        )

        bridge.apply_positioning(plan)

        self.assertIsNone(right.base_arguments)

    def test_next_hitter_waits_for_home_hold_settling(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        right.robot_pelvis_pos_origin[0, 1] = 0.44
        right.state_elapsed_s[0] = 0.0
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="cbf_mpc",
            commands={
                "left": self._robot_command("left", -0.35, -0.35),
                "right": self._robot_command("right", 0.70, 0.70),
            },
            diagnostics={"cbf_active": False},
        )

        bridge.apply_positioning(plan)
        self.assertIsNone(right.base_arguments)

        right.state_elapsed_s[0] = 0.30
        bridge.apply_positioning(plan)
        self.assertIsNotNone(right.base_arguments)

    def test_next_hitter_goal_stays_in_mirrored_policy_home_basin(self) -> None:
        for next_hitter, goal_y, expected_y in (
            ("left", -0.285, -0.40),
            ("right", 0.285, 0.40),
        ):
            with self.subTest(next_hitter=next_hitter):
                left = _CommandStub()
                right = _CommandStub()
                left.cfg.outward_target_y = -0.78
                left.robot_pelvis_pos_origin[0, 1] = -0.55
                right.robot_pelvis_pos_origin[0, 1] = 0.55
                hitter = "right" if next_hitter == "left" else "left"
                bridge = IsaacControllerBridge({"left": left, "right": right})
                plan = PlanResult(
                    sequence=1,
                    shot_id="shot",
                    hitter=hitter,
                    next_hitter=next_hitter,
                    phase="pre_hit",
                    strategy="cbf_mpc",
                    commands={
                        "left": self._robot_command("left", goal_y, goal_y),
                        "right": self._robot_command("right", goal_y, goal_y),
                    },
                    diagnostics={"cbf_active": False},
                )

                bridge.apply_positioning(plan)

                arguments = bridge.commands[next_hitter].base_arguments
                self.assertIsNotNone(arguments)
                torch.testing.assert_close(
                    arguments["target_xy"],
                    torch.tensor([[0.0, expected_y]]),
                )

    def test_post_hit_hitter_streams_safety_during_outward_hold(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        left.doubles_state[0] = left.OUTWARD_HOLD
        left.robot_pelvis_pos_origin[0, 1] = -0.76
        left.target_y[0] = -0.78
        left.home_target_y[0] = -0.40
        left.cfg.outward_target_y = -0.78
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="post_hit",
            strategy="cbf_mpc",
            commands={
                "left": self._robot_command("left", -0.95, -0.60),
                "right": self._robot_command("right", 0.35, 0.35),
            },
            diagnostics={
                "cbf_active": True,
                "cbf_controlled_robot": "both",
            },
        )

        bridge.apply_positioning(plan)

        self.assertIsNotNone(left.base_arguments)
        torch.testing.assert_close(
            left.base_arguments["target_xy"],
            torch.tensor([[0.0, -0.95]]),
        )
        torch.testing.assert_close(
            left.base_arguments["return_target_y"],
            torch.tensor([-0.40]),
        )
        self.assertTrue(left.base_arguments["safety_override"])

    def test_cbf_emergency_overrides_hit_phase_for_both_robots(self) -> None:
        for phase in range(6):
            with self.subTest(phase=phase):
                left = _CommandStub()
                right = _CommandStub()
                left.doubles_state[0] = phase
                right.doubles_state[0] = phase
                left.home_target_y[0] = -0.35
                right.home_target_y[0] = 0.35
                bridge = IsaacControllerBridge({"left": left, "right": right})
                plan = PlanResult(
                    sequence=1,
                    shot_id="shot",
                    hitter="left",
                    next_hitter="right",
                    phase="pre_hit",
                    strategy="cbf_mpc",
                    commands={
                        "left": self._robot_command("left", -0.95, -0.50),
                        "right": self._robot_command("right", 0.95, 0.50),
                    },
                    diagnostics={"cbf_active": True, "cbf_emergency": True},
                )

                bridge.apply_positioning(plan)

                if phase == left.OUTWARD_HOLD:
                    self.assertIsNone(left.base_arguments)
                    self.assertIsNone(right.base_arguments)
                else:
                    self.assertTrue(left.base_arguments["safety_override"])
                    self.assertTrue(right.base_arguments["safety_override"])
                    torch.testing.assert_close(
                        left.base_arguments["return_target_y"],
                        torch.tensor([-0.35]),
                    )
                    torch.testing.assert_close(
                        right.base_arguments["return_target_y"],
                        torch.tensor([0.35]),
                    )

    def test_cbf_emergency_return_uses_home_terminal_not_workspace_rail(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        left.doubles_state[0] = left.RETURN
        left.cfg.outward_target_y = -0.78
        left.robot_pelvis_pos_origin[0, 1] = -0.52
        left.target_y[0] = -0.95
        left.home_target_y[0] = -0.70
        left.return_motion_index = torch.tensor([0])
        left.motion_targets_t = torch.tensor([[0.0, 0.42, 0.0]])

        def reject_workspace_rail(**arguments: object) -> None:
            target_y = float(arguments["target_xy"][0, 1])
            if target_y < -0.80:
                raise ValueError(
                    "External base target would reverse the selected motion reference"
                )
            left.base_arguments = arguments

        left.set_external_base_target = reject_workspace_rail
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="right",
            next_hitter="left",
            phase="post_hit",
            strategy="cbf",
            commands={
                "left": self._robot_command("left", -0.95, -0.95),
                "right": self._robot_command("right", 0.70, 0.70),
            },
            diagnostics={
                "safety_active": True,
                "safety_emergency": True,
                "safety_controlled_robot": "left",
            },
        )

        bridge.apply_positioning(plan)

        self.assertIsNotNone(left.base_arguments)
        torch.testing.assert_close(
            left.base_arguments["target_xy"],
            torch.tensor([[0.0, -0.70]]),
        )
        torch.testing.assert_close(
            left.base_arguments["return_target_y"],
            torch.tensor([-0.70]),
        )

    def test_cbf_emergency_outward_is_limited_to_trained_lane(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        left.doubles_state[0] = left.OUTWARD
        left.cfg.outward_target_y = -0.78
        left.robot_pelvis_pos_origin[0, 1] = -0.68
        left.target_y[0] = -0.70
        left.home_target_y[0] = -0.40
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="right",
            next_hitter="left",
            phase="post_hit",
            strategy="cbf",
            commands={
                "left": self._robot_command("left", -0.95, -0.70),
                "right": self._robot_command("right", 0.70, 0.70),
            },
            diagnostics={
                "safety_active": True,
                "safety_emergency": True,
                "safety_controlled_robot": "left",
            },
        )

        bridge.apply_positioning(plan)

        self.assertIsNotNone(left.base_arguments)
        torch.testing.assert_close(
            left.base_arguments["target_xy"],
            torch.tensor([[0.0, -0.78]]),
        )

    def test_pre_hit_emergency_only_moves_controlled_teammate(self) -> None:
        left = _CommandStub()
        right = _CommandStub()
        left.doubles_state[0] = left.HIT
        right.doubles_state[0] = right.HIT
        left.home_target_y[0] = -0.35
        right.home_target_y[0] = 0.35
        bridge = IsaacControllerBridge({"left": left, "right": right})
        plan = PlanResult(
            sequence=1,
            shot_id="shot",
            hitter="left",
            next_hitter="right",
            phase="pre_hit",
            strategy="cbf_mpc",
            commands={
                "left": self._robot_command("left", -0.95, -0.50),
                "right": self._robot_command("right", 0.95, 0.50),
            },
            diagnostics={
                "cbf_active": True,
                "cbf_emergency": True,
                "cbf_controlled_robot": "right",
            },
        )

        bridge.apply_positioning(plan)

        self.assertIsNone(left.base_arguments)
        self.assertTrue(right.base_arguments["safety_override"])


if __name__ == "__main__":
    unittest.main()
