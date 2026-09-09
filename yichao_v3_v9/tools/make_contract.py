import json, sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src'))
from yichao_v3_v9 import HANDOFF
from utils.joint_mapping import LAB_JOINT_NAMES
import torch
schema=json.loads((HANDOFF/'config/schema.json').read_text())
checkpoint=torch.load(HANDOFF/'checkpoints/safe_filter_v3.pt',map_location='cpu',weights_only=True)
contract={'version':'yichao-v3-v9-offline-v1','real_input_accepted':False,
'actor_iteration':199,'calibrated_threshold':float(checkpoint['calibrated_threshold']),
'fixture_history_dt_s':.02,'history_dt_evidence':'synthetic assumption; exact training step_dt not yet accepted',
'fixture_joint_order':list(LAB_JOINT_NAMES),'fixture_joint_limits':[[-2.,2.]]*29,
'joint_evidence':'synthetic limits only; LAB names are deploy order, training order/soft limits pending',
'initial_memory_evidence':'synthetic scheduler bootstrap only; real restart requires reconciled protocol targets',
'coordinate_evidence':'synthetic training_common_fixture only; no live transform accepted',
'clock_evidence':'single replay monotonic domain only; no cross-machine conversion accepted',
'slots':[{'index':0,'training_name':'left','robot':'66','role':'table_right','mirrored':True},{'index':1,'training_name':'right','robot':'198','role':'table_left','mirrored':False}],
'workspace_y':[-1.2,1.2],'minimum_gap_m':.45,'training_home_y':schema['environment']['observation']['planner']['home_y'],
'feedback_timeout_s':.25,'prediction_timeout_s':.30,'decision_budget_s':.02,'queue_capacity':1,
'candidate_grid':{'points_per_axis':7,'radius_normalized':.75,'include_nominal':True,'include_home':True},
'fields':[],'unresolved':['training joint order and per-robot soft limits','training exact step_dt and base history z/origin','real coordinate transform/heading/slot evidence','real source clocks and uncertainty','initial applied-target memory','exact dirty controller patch','real motion array equivalence and runtime configuration','physical safety watchdogs and resource/control ownership']}
for abi,names in [('actor',schema['observation_names']),('safe',schema['cbf_input_names'][:85])]:
    for i,name in enumerate(names):
        if abi=='actor':
            source='model-reconstructed ball minus each historical base' if i<30 else ('current base XY' if i<34 else 'confirmed protocol target memory')
            scale='XYZ/[3,1.5,1.5], clip ±3' if i<30 else ('XY/[3,1.2], clip ±3' if i<34 else 'Y/1.2')
        else:
            source='base history per slot' if i<20 else ('joint positions per slot in training order' if i<78 else ('predicted strike point' if i<81 else ('racket velocity (not ball velocity)' if i<84 else 'TTS at decision time')))
            scale='XY/[3,1.2], clip ±3' if i<20 else ('(q-mid)/half, clip ±1' if i<78 else ('XYZ/[3,1.5,1.5], clip ±3' if i<81 else ('velocity/4, clip ±3' if i<84 else 'seconds, clip ±3')))
        contract['fields'].append({'abi':abi,'index':i,'name':name,'source':source,'units':'dimensionless except TTS seconds','frame':'common training world; joints training order','transform':scale,'sample_time':'history index oldest to newest; otherwise source projected to decision time','validity':'finite, strict source freshness and ABI; real acceptance gate closed','missing':'reject entire decision; no new HIT'})
(root/'config/interface.json').write_text(json.dumps(contract,indent=2)+'\n')
