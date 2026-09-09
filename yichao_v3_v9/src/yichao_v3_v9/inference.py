"""CPU-only ONNX actor, frozen risk scorer, candidate search and analytic guard."""
import json
import numpy as np
import onnxruntime as ort
from . import HANDOFF
from .inputs import array


def project(action):
    physical = np.clip(array(action,(2,),'action'),-1,1)*np.float32(1.2)
    changed = physical[1]-physical[0] < .45
    if changed:
        left = np.clip(physical.mean()-.225,-1.2,.75)
        physical = np.array([left,left+.45],np.float32)
    return physical/np.float32(1.2), bool(changed)


class Pipeline:
    def __init__(self, actor=199):
        if actor not in (190,199):
            raise ValueError('actor must be 190 or 199')
        options=ort.SessionOptions()
        options.intra_op_num_threads=1
        options.inter_op_num_threads=1
        options.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
        self.actor=ort.InferenceSession(str(HANDOFF/f'onnx/rl_planner_actor_model_{actor}.onnx'),options,providers=['CPUExecutionProvider'])
        self.scorer=ort.InferenceSession(str(HANDOFF/'onnx/safe_filter_v3.onnx'),options,providers=['CPUExecutionProvider'])
        # Extraction is verified against the frozen PT in the acceptance tests.
        from . import ROOT
        cfg=json.loads((ROOT/'config/interface.json').read_text())
        self.threshold=cfg['calibrated_threshold']
        self.home=np.array(cfg['training_home_y'],np.float32)/1.2
        self.actor_iteration=actor

    def risks(self,state,actions):
        actions=np.asarray(actions,dtype=np.float32)
        states=np.repeat(array(state,(85,),'safe_state')[None,:],len(actions),axis=0)
        risks=self.scorer.run(['conservative_risk'],{'safe_observation':states,'candidate_action':actions})[0]
        if risks.shape != (len(actions),) or not np.isfinite(risks).all() or np.any((risks<0)|(risks>1)):
            raise ValueError('invalid_filter_output')
        return risks

    def select(self,state,raw):
        nominal=np.clip(array(raw,(2,),'actor_output'),-1,1)
        offsets=np.linspace(-.75,.75,7,dtype=np.float32)
        grid=np.array([(a,b) for a in offsets for b in offsets],np.float32)
        candidates=np.concatenate((nominal[None,:],np.clip(nominal+grid,-1,1),self.home[None,:]))
        risks=self.risks(state,candidates)
        safe=((candidates[:,1]-candidates[:,0])>=np.float32(.45/1.2)) & (risks<=self.threshold)
        scores=np.where(safe,((candidates-nominal)**2).sum(axis=1)+1e-3*risks,np.inf)
        index=int(np.argmin(scores)) if safe.any() else len(candidates)-1
        selected=candidates[index]
        projected,intervention=project(selected)
        # Any subsequent projection must be scored; home is never assumed safe.
        projected_risk=float(self.risks(state,projected[None,:])[0]) if intervention else float(risks[index])
        valid=bool(safe.any() and projected_risk<=self.threshold)
        return {'valid':valid,'reason':None if valid else 'no_safe_candidate',
            'actor_iteration':self.actor_iteration,'raw_nominal':np.asarray(raw).tolist(),'nominal':nominal.tolist(),
            'filtered':selected.tolist(),'projected':projected.tolist(),'target_y':(projected*1.2).tolist(),
            'nominal_risk':float(risks[0]),'selected_risk':float(risks[index]),'projected_risk':projected_risk,
            'candidate_index':index,'candidate_risks':risks.tolist(),'filter_intervened':bool(np.max(np.abs(selected-nominal))>1e-6),
            'analytic_intervened':intervention,'fallback':not bool(safe.any()),'threshold':self.threshold}

    def decide(self,features):
        raw=self.actor.run(['action'],{'observation':features.actor[None,:]})[0]
        return self.select(features.safe,array(raw,(1,2),'actor_result')[0])
