"""V3 joint-position preprocessing from the user-designated G1 URDF."""
import hashlib
import json
import xml.etree.ElementTree as ET
import numpy as np

from . import ROOT
from .inputs import array
from utils.joint_mapping import LAB_JOINT_NAMES


class JointNormalizer:
    def __init__(self):
        self.profile = json.loads((ROOT/'config/joint_normalization.json').read_text())
        p = self.profile
        if p['version'] != 'yichao-v3-joint-normalization-v1' or p['joint_names'] != list(LAB_JOINT_NAMES):
            raise ValueError('joint_normalization_schema_mismatch')
        blob = (ROOT/p['urdf_relative_path']).read_bytes()
        if hashlib.sha256(blob).hexdigest() != p['urdf_sha256']:
            raise ValueError('normalization_urdf_identity_mismatch')
        joints = {j.attrib['name']:j for j in ET.fromstring(blob).findall('joint') if j.attrib['type'] != 'fixed'}
        if set(joints) != set(LAB_JOINT_NAMES):
            raise ValueError('unexpected_urdf_joints')
        hard = np.asarray([[float(joints[name].find('limit').attrib[k]) for k in ('lower','upper')]
                           for name in LAB_JOINT_NAMES], dtype=np.float32)
        if not np.array_equal(hard, array(p['hard_joint_limits_rad'], (29,2), 'hard_limits')):
            raise ValueError('profile_limits_differ_from_urdf')
        if p['soft_joint_pos_limit_factor'] != .9:
            raise ValueError('factor_differ_from_frozen_controller')
        # Same midpoint/range operation used by Isaac Lab articulation setup.
        midpoint = (hard[:,0]+hard[:,1])*.5
        half = (hard[:,1]-hard[:,0])*.5*np.float32(.9)
        self.soft = np.stack((midpoint-half, midpoint+half),axis=1)
        if not np.allclose(self.soft, array(p['soft_joint_limits_rad'],(29,2),'soft_limits'),atol=5e-7,rtol=0):
            raise ValueError('profile_soft_limits_mismatch')
        self.midpoint = (self.soft[:,0]+self.soft[:,1])*.5
        self.half = (self.soft[:,1]-self.soft[:,0])*.5
        if np.any(self.half <= 1e-5):
            raise ValueError('degenerate_joint_limit')

    def normalize(self, q_lab_rad, joint_names):
        if list(joint_names) != list(LAB_JOINT_NAMES):
            raise ValueError('joint_order_mismatch')
        q = array(q_lab_rad, (29,), 'q_lab_rad')
        return np.clip((q-self.midpoint)/self.half,-1,1).astype(np.float32)
