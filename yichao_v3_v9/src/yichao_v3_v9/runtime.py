"""Fixed relay lifecycle with Yichao position selection.

This module owns neither R2 nor whole-body inference. The existing Fixed
state/command schemas, commit tokens, phase acknowledgements and RETURN retry
logic remain the transport and lifecycle contract.
"""
from dataclasses import replace
import time
import numpy as np

from doubles_planner.fixed_relay import FixedRelayPlanner
from doubles_planner.real_ros_runtime import ROBOT_ORDER, V9RealFixedRelayRuntime
from doubles_planner.strategies import StrategyDecision
from .inputs import FixedStateFeatures
from .inference import Pipeline
from .joint_normalization import JointNormalizer


class YichaoPositionStrategy:
    name = 'yichao_actor199_filter_v3'

    def __init__(self):
        self.decision = None
        self.reason = 'warming_inputs'

    def decide(self, context):
        decision = self.decision
        if decision is None or not decision['valid']:
            return StrategyDecision(
                {name: state.base_xy.copy() for name, state in context.feedback.items()},
                fallbacks=(self.reason,))
        return StrategyDecision(
            {name: np.array([context.feedback[name].base_xy[0], decision['target_y'][i]])
             for i, name in enumerate(ROBOT_ORDER)},
            diagnostics={'yichao': decision})


class YichaoRelayPlanner(FixedRelayPlanner):
    def __init__(self, config, strategy):
        super().__init__(config)
        self.strategy = strategy

    def _commit_admission(self, slot, prediction, feedback, missing):
        result = super()._commit_admission(slot, prediction, feedback, missing)
        if self.strategy.decision is None or not self.strategy.decision['valid']:
            result['admitted'] = False
            result['reasons'] = (*result['reasons'], self.strategy.reason)
        return result

    def _pending_result(self, slot, feedback, feedback_fallbacks, admission):
        result = super()._pending_result(slot, feedback, feedback_fallbacks, admission)
        # Fixed retains ownership of reservations and exclusivity. Position only
        # when neither robot is hit-locked and the model supplied a valid pair.
        decision = self.strategy.decision
        if (self._active_shot is not None or feedback_fallbacks or decision is None
                or not decision['valid'] or any(not s.valid or s.controller_phase in
                                                {'HIT', 'POST_DELAY'} for s in feedback.values())):
            return result
        commands = {}
        for i, name in enumerate(ROBOT_ORDER):
            goal = np.array([feedback[name].base_xy[0], decision['target_y'][i]])
            commands[name] = replace(result.commands[name], role='stage',
                                     desired_base_position=goal, trajectory_base_position=goal)
        return replace(result, commands=commands)



class YichaoRuntime(V9RealFixedRelayRuntime):
    def __init__(self, *, shadow=True, session_id=None, pipeline=None):
        super().__init__(shadow=shadow, session_id=session_id)
        self.position_strategy = YichaoPositionStrategy()
        self.planner = YichaoRelayPlanner(self.config, self.position_strategy)
        self.pipeline = pipeline if pipeline is not None else Pipeline(199, workspace=self.config.workspace_y)
        self.features = FixedStateFeatures(JointNormalizer())
        self.last_features = None
        self.last_inference = None
        self._decision_shot = None
        self._shot_decision = None
        # Keep Fixed's internal mode for its readiness and RETURN behaviour.
        # The wire mode is tagged below so the adapted bridge can route stage.

    def update_robot_state(self, robot, payload, receive_monotonic=None):
        received = time.monotonic() if receive_monotonic is None else float(receive_monotonic)
        accepted = super().update_robot_state(robot, payload, received)
        if accepted:
            self.features.ingest(robot, self._states[robot], received)
        return accepted

    def tick(self, now=None):
        current = time.monotonic() if now is None else float(now)
        self.position_strategy.decision = None
        self.last_features = None
        self.last_inference = None
        try:
            features = self.features.build(self._states, self._ball, current)
            self.last_features = features
            shot = self._ball['shot_id']
            if shot != self._decision_shot or self._shot_decision is None:
                decision = self.pipeline.decide(features)
                if decision['valid']:
                    # Do not silently clip a learned target to Fixed's lanes.
                    # The delivered filter's workspace is wider than Fixed's
                    # accepted feedback window; reject and expose this mismatch.
                    low, high = self.config.workspace_y
                    if any(not low <= y <= high for y in decision['target_y']):
                        decision = {**decision, 'valid': False, 'reason': 'model_target_outside_deploy_workspace'}
                if decision['valid']:
                    self._decision_shot, self._shot_decision = shot, decision
            else:
                decision = self._shot_decision
            self.last_features = features
            self.last_inference = decision
            self.position_strategy.decision = decision
            self.position_strategy.reason = decision.get('reason') or 'ready'
        except (KeyError, TypeError, ValueError) as error:
            self.position_strategy.reason = str(error)
        outputs = super().tick(current)
        for name, payload in outputs.items():
            if self._return_dispatched and payload['command']['role'] == 'stage':
                payload['command']['role'] = 'hold'
                payload['planned_active'] = False
                payload['command']['active'] = False
            # planner_mode is a routing selector in the EXISTING controller.
            # Its fixed_relay branch handles hold/return; stage uses the
            # existing generic base-target branch. No robot changes required.
            payload['planner_mode'] = ('fixed_relay' if payload['command']['role'] in {'hold', 'return'}
                                       else 'yichao_relay')
            payload['position_strategy'] = 'yichao_actor199_filter_v3'
            payload['yichao'] = {'reason': ','.join(payload['admission_reasons']) or self.position_strategy.reason,
                                 'inference_reason': self.position_strategy.reason,
                                 'decision': self.position_strategy.decision}
        return outputs
