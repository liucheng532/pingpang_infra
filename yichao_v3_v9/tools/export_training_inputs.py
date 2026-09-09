"""Export metadata from an ALREADY CREATED IsaacV1ShotPlannerEnv on the training host.

Call export_training_inputs(v3_env, '/new/path/training_inputs.json') at a paused
Python breakpoint between environment steps. This helper does not construct an
environment, advance simulation, reset, or call observation builders (those can
mutate histories). It needs no Isaac imports itself.
"""
import datetime
import hashlib
import inspect
import json
import math
from pathlib import Path


def values(tensor):
    if hasattr(tensor, 'detach'):
        tensor = tensor.detach().cpu()
    return tensor.tolist() if hasattr(tensor, 'tolist') else tensor


def export_training_inputs(v3_env, output):
    env = v3_env.env
    data = {'schema': 'yichao-training-input-export-v1',
            'exported_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'real_interface_accepted': False, 'robots': {}, 'source_files': {}}
    dt = float(env._step_dt())
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('invalid actual environment step dt')
    data['actual_step_dt_s'] = dt
    for obj in (v3_env, env):
        path = inspect.getsourcefile(type(obj))
        if path:
            data['source_files'][path] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    for role in ('left', 'right'):
        robot = env.robots[role]
        names = list(robot.joint_names[:29])
        if len(names) != 29 or len(set(names)) != 29:
            raise ValueError('expected 29 distinct actual joint names')
        limits = robot.data.soft_joint_pos_limits
        if limits.ndim == 3:
            limits = limits[0]
        limits = values(limits[:29])
        if len(limits) != 29 or any(len(pair) != 2 or not all(math.isfinite(x) for x in pair)
                                   or pair[0] >= pair[1] for pair in limits):
            raise ValueError('invalid actual soft joint limits')
        data['robots'][role] = {'joint_names': names, 'soft_joint_pos_limits': limits,
                               'joint_pos_sample': values(robot.data.joint_pos[0, :29]),
                               'base_position_history_sample': values(env.base_position_history[role][0])}
    data['ball_position_history_sample'] = values(env.ball_position_history[0])
    data['previous_applied_targets_sample'] = values(v3_env.previous_applied_targets[0])
    with Path(output).open('x') as f:
        json.dump(data, f, indent=2, allow_nan=False)
        f.write('\n')
    return data
