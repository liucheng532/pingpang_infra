"""Only file input/output. No ROS, LCM, sockets, subprocess or active option."""
import argparse
import json
import sys
import time
from pathlib import Path
from . import ROOT
from .isolation import install_guard
install_guard()
from .inputs import InputAdapter
from .inference import Pipeline
from .executor import OfflineExecutor
from .relay import Relay
from .manifest import verify, sha
from .v9_wire import encode as encode_v9


def run(input_path,output_path,actor=199):
    import stat
    if not stat.S_ISREG(Path(input_path).stat().st_mode):
        raise ValueError("offline input must be a regular file")
    manifest=verify()
    release_hash=sha(ROOT/'config/assets.lock.json')
    contract=json.loads((ROOT/'config/interface.json').read_text())
    adapter=InputAdapter(contract);pipeline=Pipeline(actor)
    # Warm CPU ONNX before the 20 ms decision budget begins.
    import numpy as np
    pipeline.actor.run(None,{'observation':np.zeros((1,36),np.float32)})
    pipeline.risks(np.zeros(85,np.float32),np.zeros((51,2),np.float32))
    session='offline-'+str(time.time_ns())
    executor=OfflineExecutor(session)
    relay=Relay(session,[-.9125,.2])  # explicit synthetic bootstrap: scheduler targets only
    stats={'session':session,'release_manifest_sha256':release_hash,'input_contract_version':contract['version'],
        'planner_commit':manifest['planner_commit'],'deploy_commit':manifest['deploy_commit'],'actor_iteration':actor,
        'records':0,'invalid':0,'deadline_drops':0,'queue_drops':0,'queue_capacity':1,'latencies_ms':[],
        'evidence':'synthetic/reference logic and simulated scheduler only','real_interface_accepted':False}
    # Exclusive creation prevents overwriting a previous run's evidence.
    with Path(input_path).open() as source,Path(output_path).open('x') as sink:
        for line in source:
            start=time.perf_counter();entry={'record_index':stats['records'],'input_contract_version':contract['version'],'release_manifest_sha256':release_hash}
            try:
                record=json.loads(line)
                features=adapter.build(record,relay.previous_targets)
                if record['provenance']!='synthetic': raise ValueError('offline_executor_requires_synthetic_provenance')
                feedback=executor.tick(record['now'],{s:record['robots'][s]['base_position'] for s in executor.positions})
                relay.observe(feedback.values())
                commands=relay.step(features,pipeline)
                v9_preview=[encode_v9(c,record['now'],float(record['robots'][c['robot']]['base_position'][0]),allow_hit=True)
                            for c in commands]
                # Include feature building and scheduler feedback handling in the budget.
                elapsed=time.perf_counter()-start
                if elapsed>.02 and commands:
                    stats['deadline_drops']+=1;commands=relay.fail('end_to_end_deadline_exceeded');v9_preview=[]
                acks=executor.apply_batch(commands,record['now']) if commands else []
                relay.observe(acks)
                entry.update(actor_features=features.actor.tolist(),safe_features=features.safe.tolist(),
                    source_record=record,commands=commands,v9_wire_preview=v9_preview,
                    v9_wire_published=False,acknowledgements=acks,completion_feedback=feedback,valid=True)
            except (KeyError,TypeError,ValueError,RuntimeError) as error:
                stats['invalid']+=1;relay.fail('input_or_execution_invalid:'+str(error))
                entry.update(valid=False,error=str(error),commands=[])
            elapsed=(time.perf_counter()-start)*1000
            stats['latencies_ms'].append(elapsed);stats['records']+=1
            entry.update(relay=relay.snapshot(),processing_ms=elapsed,queue_wait_ms=0.,evidence='simulated_scheduler')
            sink.write(json.dumps(entry,allow_nan=False)+'\n')
    stats['hit_count']=executor.hit_count
    stats['final_state']=relay.state
    stats['fault']=relay.fault
    values=stats.pop('latencies_ms')
    stats['latency_ms']={k:float(np.percentile(values,p)) if values else None for k,p in [('p50',50),('p95',95),('max',100)]}
    Path(str(output_path)+'.summary.json').write_text(json.dumps(stats,indent=2)+'\n')
    return stats


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--actor',type=int,choices=(190,199),default=199)
    args=p.parse_args();print(json.dumps(run(args.input,args.output,args.actor),indent=2))

if __name__=='__main__': main()
