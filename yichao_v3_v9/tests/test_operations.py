import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
from urllib.request import Request, urlopen
import pytest
from yichao_v3_v9 import ROOT
from yichao_v3_v9.journal import SessionJournal
from yichao_v3_v9.monitor.server import TelemetryStore, MonitorHttpServer, DEFAULT_TOPICS
from yichao_v3_v9.monitor_recording import Library


def script(name):
    spec=importlib.util.spec_from_file_location('test_'+name,ROOT/'scripts'/f'{name}.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def test_launcher_only_owns_planner_and_monitor(tmp_path):
    launcher=script('run_yichao_stack')
    components=launcher.components(tmp_path)
    assert [c.name for c in components]==['planner','monitor']
    assert not any('TableTennis' in arg or 'g1_control' in arg or 'deploy_policy.py' in arg
                   for c in components for arg in c.command)
    assert launcher.existing_planners('42 python3 scripts/run_v9_real_fixed_relay.py')
    assert not launcher.existing_planners('43 python3 TableTennis.py\n44 /robot/g1_control')


def test_journal_rotates_without_deleting_events(tmp_path):
    journal=SessionJournal(tmp_path,segment_bytes=200)
    for i in range(40):journal.write('test',sequence=i,invalid=float('nan') if i==3 else None)
    journal.close()
    files=sorted(tmp_path.glob('events.*.jsonl'))
    assert len(files)>1
    rows=[json.loads(line) for p in files for line in p.read_text().splitlines()]
    assert [r['sequence'] for r in rows]==list(range(40))
    assert rows[3]['invalid']=={'nonfinite':'nan'}
    assert journal.error is None and journal.dropped==0
    with pytest.raises(ValueError):SessionJournal(tmp_path)


def test_monitor_recording_download_import_and_seek(tmp_path):
    store=TelemetryStore(DEFAULT_TOPICS);store.recordings=Library(tmp_path,store)
    server=MonitorHttpServer(('127.0.0.1',0),store)
    serving=threading.Thread(target=server.serve_forever,daemon=True);serving.start()
    base=f'http://127.0.0.1:{server.server_address[1]}'
    def request(path,data=None):
        with urlopen(Request(base+path,data=data),timeout=3) as response:return response.read()
    try:
        started=json.loads(request('/api/recordings/start',b''))
        store.ingest('ball',{'valid':False,'position':[0,0,0]})
        time.sleep(.12)
        stopped=json.loads(request('/api/recordings/stop',b''))
        assert stopped['frames']>=2 and stopped['error'] is None
        body=request('/api/recordings/download?id='+started['id'])
        imported=json.loads(request('/api/recordings/import',body))
        replay=json.loads(request('/api/replay?id='+imported['id']))
        assert replay['frames'][-1]['snapshot']['topics']['ball']['data']['valid'] is False
        assert replay['metadata']['status']=='imported'
    finally:
        server.shutdown();server.server_close();serving.join()
        if store.recordings.recorder:store.recordings.stop()


@pytest.mark.parametrize('active',[False,True])
def test_ros_entry_never_publishes_shadow_commands_or_replaces_fixed(monkeypatch,tmp_path,active):
    module=script('run_yichao_planner');publishers=[];shutdown=[]
    fake=SimpleNamespace(
        init_node=lambda *a,**k:None,
        get_master=lambda:SimpleNamespace(getSystemState=lambda:(1,'',[[['/doubles/table_right/command',['/existing_fixed']]],[],[]])),
        Publisher=lambda topic,*a,**k:publishers.append(topic),
        Subscriber=lambda *a,**k:None,Timer=lambda *a:None,Duration=lambda v:v,
        on_shutdown=shutdown.append,loginfo=lambda *a:None,spin=lambda:[f() for f in shutdown])
    monkeypatch.setitem(sys.modules,'rospy',fake)
    monkeypatch.setitem(sys.modules,'std_msgs',SimpleNamespace())
    monkeypatch.setitem(sys.modules,'std_msgs.msg',SimpleNamespace(String=object))
    monkeypatch.setattr(sys,'argv',['planner','--session-dir',str(tmp_path),*(['--active'] if active else [])])
    if active:
        with pytest.raises(RuntimeError,match='Existing command publishers'):module.main()
        assert publishers==[]
    else:
        module.main()
        assert publishers==['/doubles/yichao/status']


def test_convenience_launchers_preserve_fixed_components_and_no_io_dry_run(tmp_path):
    import subprocess
    launcher=script('run_yichao_stack')
    items=launcher.components(tmp_path,active=True,predictor=True,monitor_port=8090)
    assert [c.name for c in items]==['predictor','planner','monitor']
    original=launcher.fixed.build_components()[0]
    assert items[0]==original
    assert '--active' in items[1].command
    assert items[2].command[items[2].command.index('--port')+1]=='8090'
    assert launcher.existing_planners('42 python3 scripts/run_predictor.py',predictor=True)
    assert launcher.environment()['PINGPANG_CALIBRATION_CONFIG']==str(launcher.fixed.CALIBRATION_CONFIG)
    work=subprocess.run(['bash',str(ROOT/'scripts/start_yichao_workstation.sh'),'dry-run'],capture_output=True,text=True,check=True)
    plan=json.loads(work.stdout)
    assert plan['start'] is False and plan['active'] is True
    assert [c['name'] for c in plan['components']]==['predictor','planner','monitor']
    robots=subprocess.run(['bash',str(ROOT/'scripts/start_yichao_robots.sh'),'dry-run'],capture_output=True,text=True,check=True)
    assert 'start_v11_dual_robots.sh dry-run active normal active 1.0 v11-teacher' in robots.stdout
