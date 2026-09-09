"""Generate a clearly synthetic prescribed-pelvis replay, not a physics/controller simulation."""
import json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'src'),str(root/'tests')]
from common import record,CONTRACT
from yichao_v3_v9.inputs import InputAdapter
from yichao_v3_v9.inference import Pipeline
from yichao_v3_v9.executor import OfflineExecutor
from yichao_v3_v9.relay import Relay
pipeline=Pipeline();adapter=InputAdapter(CONTRACT);executor=OfflineExecutor('fixture');relay=Relay('fixture',[-.9125,.2])
pipeline.actor.run(None,{'observation':np.zeros((1,36),np.float32)});pipeline.risks(np.zeros(85,np.float32),np.zeros((51,2),np.float32))
rows=[];shot=1
histories={s:[p.copy() for _ in range(5)] for s,p in executor.positions.items()}
for i in range(450):
    now=10+i*.02
    positions={s:p.copy() for s,p in executor.positions.items()}
    for s,scheduler in executor.schedulers.items():
        ref=scheduler.output()
        if ref.state.name not in ('HIT','POST_DELAY'):
            positions[s][1]+=np.clip(ref.target_base[1],-.03,.03)
        histories[s]=histories[s][1:]+[positions[s].copy()]
    r=record(now,f'shot-{shot}',{s:p.tolist() for s,p in positions.items()},{s:x.state.name for s,x in executor.schedulers.items()})
    for s in positions:
        r['robots'][s]['base_history']=[p.tolist() for p in histories[s]]
        r['robots'][s]['q']=executor.schedulers[s].output().joint_pos.tolist()
    f=adapter.build(r,relay.previous_targets)
    acks=executor.tick(now,positions);relay.observe(acks.values())
    # Match recorded phases to the source scheduler feedback for this decision.
    for s in positions:r['robots'][s]['phase']=executor.schedulers[s].state.name
    f.record=r
    commands=relay.step(f,pipeline)
    if commands:relay.observe(executor.apply_batch(commands,now))
    rows.append(r)
    if relay.state=='FAULT':raise RuntimeError(relay.snapshot())
    if relay.state=='COMPLETE':
        if shot==2:break
        shot+=1
path=root/'fixtures/two_shots.jsonl'
path.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in rows))
print(json.dumps({'records':len(rows),'hits':executor.hit_count,'state':relay.state,'provenance':'synthetic prescribed pelvis follower (1.5 m/s), rolling 0.5s TTS; no dynamics or real controller feedback'},indent=2))
