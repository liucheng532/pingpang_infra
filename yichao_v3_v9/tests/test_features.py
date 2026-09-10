import json
import numpy as np
from yichao_v3_v9 import ROOT
from yichao_v3_v9.inputs import build_vectors
from yichao_v3_v9.inference import Pipeline


def test_frozen_training_feature_vectors():
    fixture = json.loads((ROOT/'fixtures/reference_vectors.json').read_text())
    record = fixture['input']
    cfg = json.loads((ROOT/'fixtures/normalization_fixture.json').read_text())
    limits = np.asarray(cfg['fixture_joint_limits'], dtype=np.float32)
    joints = []
    histories = []
    for slot in ('66', '198'):
        state = record['robots'][slot]
        histories.append(state['base_history'])
        joints.append(np.clip((np.asarray(state['q'], dtype=np.float32)-limits.mean(axis=1)) /
                              ((limits[:,1]-limits[:,0])/2), -1, 1))
    ball = {**record['ball'], 'predicted_strike_position': record['ball']['strike_position']}
    features = build_vectors(histories, joints, ball, fixture['previous_targets'],
                             acceleration=ball['acceleration'])
    np.testing.assert_allclose(features.actor, fixture['actor_reference'], atol=2e-7, rtol=0)
    np.testing.assert_allclose(features.safe, fixture['safe_reference'], atol=2e-7, rtol=0)


def test_actor_matches_delivered_torchscript():
    import torch
    from yichao_v3_v9 import HANDOFF
    torch.set_num_threads(1)
    rng = np.random.default_rng(19)
    observation = rng.normal(0, .3, (32, 36)).astype(np.float32)
    for iteration in (190, 199):
        pipe = Pipeline(iteration)
        reference = torch.jit.load(str(HANDOFF/f'onnx/rl_planner_actor_model_{iteration}.pt'), map_location='cpu')
        expected = reference(torch.from_numpy(observation)).detach().numpy()
        actual = pipe.actor.run(['action'], {'observation': observation})[0]
        np.testing.assert_allclose(actual, expected, atol=3e-6, rtol=1e-5)


def test_filter_selection_matches_frozen_training_implementation():
    import torch
    from yichao_v3_v9 import HANDOFF
    from training_reference.v0 import FrozenV0SafetyFilter, FrozenV0SafetyConfig
    reference = FrozenV0SafetyFilter(FrozenV0SafetyConfig(
        checkpoint=HANDOFF/'checkpoints/safe_filter_v3.pt', enabled=True,
        candidate_radius=.75, candidate_grid_points=7), device='cpu')
    pipeline = Pipeline()
    rng = np.random.default_rng(11)
    for _ in range(12):
        state = rng.normal(0,.4,85).astype(np.float32)
        action = rng.uniform(-1,1,2).astype(np.float32)
        actual = pipeline.select(state, action)
        expected = reference.filter(torch.tensor(state[None]), torch.tensor(action[None]),
                                    home_action=torch.tensor(pipeline.home[None]), minimum_normalized_gap=.375)
        np.testing.assert_allclose(actual['filtered'], expected.action.detach().numpy()[0], atol=2e-6, rtol=0)


def test_joint_order_matches_existing_controller():
    import sys
    sys.path.insert(0, str(ROOT/'baseline/controller_reference/g1_gym_deploy'))
    from utils.joint_mapping import LAB_JOINT_NAMES
    from yichao_v3_v9.joint_normalization import JointNormalizer
    assert JointNormalizer().profile['joint_names'] == list(LAB_JOINT_NAMES)
