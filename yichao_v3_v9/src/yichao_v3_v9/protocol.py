"""Source-backed V3 motion target projection, without creating an Isaac environment.

The unmodified planner's pure projection methods operate on frozen motion arrays.
Robot geometry and live soft-limit acceptance remain outside this offline bridge.
"""
import json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import torch
from . import ROOT,HANDOFF
from .inputs import SLOTS
from doubles_planner.isaac_rl import IsaacDoublesHighLevelEnv


class V3Protocol:
    def __init__(self):
        schema=json.loads((HANDOFF/'config/schema.json').read_text())
        cfg=schema['environment']['observation']['planner']
        self.locomotion_preemption=schema['environment']['locomotion_preemption']
        self.reservation_duration_s=schema['environment']['reservation_duration_s']
        self.return_pairs=np.asarray(cfg['next_hitter_return_y'],dtype=np.float32)
        self.outward=np.asarray(cfg['outward_y'],dtype=np.float32)
        self.post_delay=max(schema['frozen_low_level_policy']['training_scenario_config']['post_delay_range_s'])
        # Source controller commands.py uses cfg.body_names.index('pelvis');
        # controller@47359de flat_env_cfg.py places pelvis at motion index zero.
        bank=ROOT/'assets/0718-move-160-80hz'
        labels=[];targets=[];actual=[]
        for line in (bank/'dataindex.csv').read_text().splitlines()[1:]:
            if not line.strip():continue
            index,label,target=line.split()
            folder=index if index.startswith(bank.name+'-') else bank.name+'-'+index+':v0'
            if ':' not in folder:folder+=':v0'
            with np.load(bank/folder/'motion.npz',allow_pickle=False) as motion:
                pelvis=motion['body_pos_w'][:,0,1]
                actual.append(float(pelvis[-1]-pelvis[0]))
            labels.append(int(label));targets.append(float(target))
        labels=np.asarray(labels,dtype=bool);targets=np.asarray(targets,np.float32);actual=np.asarray(actual,np.float32)
        reference=object.__new__(IsaacDoublesHighLevelEnv)
        reference.device='cpu';reference.config=NS(observation=NS(planner=NS(**cfg)))
        reference._step_dt=lambda:.02
        reference.commands={}
        selectable=np.arange(len(labels));selectable=selectable[~np.isin(selectable,[41,47])]
        for name,sign in (('left',-1.),('right',1.)):
            tensor_targets=np.zeros((len(labels),3),np.float32);tensor_targets[:,1]=targets*sign
            reference.commands[name]=NS(cfg=NS(outward_target_y=sign*.9125,external_target_deadband_m=.01),
                motion_targets_t=torch.tensor(tensor_targets),motion_labels_t=torch.tensor(labels),
                _motion_pelvis_displacement_y=torch.tensor(actual*sign),
                _outbound_motion_indices=torch.tensor(selectable[labels[selectable]]),
                _return_motion_indices=torch.tensor(selectable[~labels[selectable]]))
        self.reference=reference

    def commit(self,hitter,record):
        index=SLOTS.index(hitter);peer=1-index
        robot=record['robots'][hitter]
        current=float(robot['base_position'][1]);velocity=float(robot['base_velocity'][1])
        sign=-1. if index==0 else 1.
        with torch.no_grad():
            outward=self.reference._physical_outward_target(index,torch.tensor([current]),
                outward_speed=torch.tensor([velocity*sign]),
                transition_horizon_s=torch.tensor([record['ball']['time_to_strike_s']+self.post_delay]))
        return {'hitter_outward_y':float(outward[0]),'peer_clear_y':float(self.outward[peer]),
            'return_pair':self.return_pairs[peer].tolist(),'next_hitter':SLOTS[peer],
            'source':'frozen V3 _physical_outward_target + next_hitter_return_y',
            'training_post_delay_guard_s':self.post_delay}

    def return_targets(self,requested,record):
        result=[]
        with torch.no_grad():
            for index,slot in enumerate(SLOTS):
                target,feasible=self.reference._project_controller_motion_target(index,
                    torch.tensor([float(record['robots'][slot]['base_position'][1])]),
                    torch.tensor([float(requested[index])]),expected_class=False,allow_stationary=True)
                if not bool(feasible[0]):raise ValueError('V3_return_reference_infeasible:'+slot)
                result.append(float(target[0]))
        return result
