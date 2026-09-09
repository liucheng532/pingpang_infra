from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
import numpy as np
import torch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from yichao_v3_v9.joint_normalization import JointNormalizer
from utils.joint_mapping import LAB_JOINT_NAMES
from doubles_planner.v0 import IsaacV0ShotPlannerEnv


class NormalizationTests(unittest.TestCase):
    def test_matches_frozen_training_feature_function(self):
        normalizer = JointNormalizer()
        vectors = [normalizer.midpoint, normalizer.soft[:,0], normalizer.soft[:,1],
                   np.linspace(-3.,3.,29,dtype=np.float32)]
        for q in vectors:
            robot = SimpleNamespace(data=SimpleNamespace(joint_pos=torch.tensor(q[None,:]),
                soft_joint_pos_limits=torch.tensor(normalizer.soft[None,:,:])))
            fake = SimpleNamespace(env=SimpleNamespace(robots={'left':robot}),num_envs=1,device='cpu')
            reference = IsaacV0ShotPlannerEnv._joint_position_normalized(fake,'left').numpy()[0]
            np.testing.assert_allclose(normalizer.normalize(q,LAB_JOINT_NAMES),reference,atol=1e-7,rtol=0)

    def test_midpoint_and_soft_boundaries(self):
        n=JointNormalizer()
        np.testing.assert_allclose(n.normalize(n.midpoint,LAB_JOINT_NAMES),0,atol=1e-7)
        np.testing.assert_allclose(n.normalize(n.soft[:,0],LAB_JOINT_NAMES),-1,atol=1e-7)
        np.testing.assert_allclose(n.normalize(n.soft[:,1],LAB_JOINT_NAMES),1,atol=1e-7)

    def test_wrong_order_nonfinite_or_dimension_rejected(self):
        n=JointNormalizer()
        with self.assertRaises(ValueError):n.normalize([0.]*29,list(reversed(LAB_JOINT_NAMES)))
        with self.assertRaises(ValueError):n.normalize([0.]*28,LAB_JOINT_NAMES)
        with self.assertRaises(ValueError):n.normalize([float('nan')]*29,LAB_JOINT_NAMES)


if __name__=='__main__':unittest.main()
