"""Portable monitor JSONL recordings and per-browser, indexed playback."""
import bisect
from datetime import datetime
import json
import math
import os
from pathlib import Path
import queue
import re
import threading
import time
import uuid

SCHEMA = 'yichao-monitor-recording-v1'
ID = re.compile(r'rec_[0-9]{8}_[0-9]{6}_[a-f0-9]{8}')
SESSION = re.compile(r'active_[a-z0-9_]{1,70}')
MAX_LINE = 8*1024*1024


def new_id():
    return 'rec_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]


def encode(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))+'\n').encode()


def atomic_json(path, value):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_bytes(encode(value));tmp.replace(path)


class Summary:
    def __init__(self):
        self.shots=set();self.hits=set();self.acks=set();self.completed=set();self.sessions=set()

    def topic(self, key, data, session=None):
        if key=='ball' and data.get('valid') is True and data.get('shot_id') is not None:
            self.shots.add((str(session),str(data['shot_id'])))
        if key.endswith('_command') and data.get('kind')=='hit':
            self.hits.add((str(data.get('session')),str(data.get('token')),str(data.get('robot'))))
        if key.endswith('_state'):
            relay=data.get('yichao_relay',{});ack=relay.get('feedback') or {}
            if ack.get('kind')=='hit' and ack.get('status') in ('accepted','completed'):
                self.acks.add((str(relay.get('session')),str(ack.get('token')),str(relay.get('robot'))))
        if key.endswith('_command') and data.get('schema_version') == 'v9-planner-command-v1':
            if data.get('valid') and data.get('command', {}).get('active') and data['command'].get('role') == 'hit':
                self.hits.add((str(data.get('session_id')), str(data.get('commit_token')), str(data.get('robot'))))
        if key.endswith('_state') and data.get('last_commit_token'):
            token = (str(data.get('last_planner_session_id')), str(data['last_commit_token']), str(data.get('robot')))
            if data.get('valid') and data.get('phase') in ('HIT', 'POST_DELAY'):
                self.acks.add(token)
            if token in self.acks and data.get('valid') and data.get('phase') in ('OUTWARD', 'OUTWARD_HOLD', 'RETURN', 'HOME_HOLD'):
                self.completed.add(token)
        if key=='planner':
            relay=data.get('relay',{});session=relay.get('session')
            if session:self.sessions.add(session)
            for shot in data.get('completed_shots',[]):self.completed.add((str(session),str(shot)))

    def result(self):
        return {'valid_shots':len(self.shots),'hit_commands':len(self.hits),'hit_acks':len(self.acks),
                'completed_shots':len(self.completed),'sessions':sorted(self.sessions)}


class Journal:
    def __init__(self, directory, label, source):
        self.directory=directory;directory.mkdir(parents=True)
        self.stream=(directory/'recording.jsonl').open('xb')
        self.index=[];self.summary=Summary();self.events=0;self.closed=False
        self.meta={'id':directory.name,'schema':SCHEMA,'label':label[:120],
                   'created_at':datetime.now().astimezone().isoformat(),'source':source,'status':'recording',
                   'duration_s':0.,'frames':0,'events':0,'dropped_events':0,'error':None}
        self.write({'kind':'header','schema':SCHEMA,'metadata':self.meta.copy()})
        self.save_meta()

    def save_meta(self):
        atomic_json(self.directory/'metadata.json',self.meta)

    def write(self, row):
        offset=self.stream.tell();self.stream.write(encode(row));return offset

    def topic(self, t, key, data, session=None, wall=None):
        self.summary.topic(key,data,session);self.events+=1
        self.write({'kind':'topic','t':t,'key':key,'data':data,'session':session,'source_wall_s':wall})

    def frame(self, t, snapshot):
        # Full checkpoints make seeking bounded and preserve calibration changes.
        offset=self.write({'kind':'frame','t':t,'snapshot':snapshot})
        self.index.append([t,offset]);self.meta.update(duration_s=t,frames=len(self.index),events=self.events)
        if snapshot.get('session',{}).get('session'):self.summary.sessions.add(snapshot['session']['session'])

    def close(self, status='saved', error=None, dropped=0):
        if self.closed:return self.meta.copy()
        self.meta.update(status=status,error=error,dropped_events=dropped,summary=self.summary.result())
        try:
            self.write({'kind':'summary','metadata':self.meta.copy()})
            self.stream.flush();os.fsync(self.stream.fileno())
        finally:
            self.stream.close();self.closed=True
        atomic_json(self.directory/'index.json',self.index);self.save_meta()
        return self.meta.copy()


class Recorder:
    def __init__(self, store, journal):
        self.store=store;self.journal=journal;self.started=time.monotonic();self.queue=queue.Queue(maxsize=8192)
        self.done=threading.Event();self.dropped=0;self.error=None
        self.worker=threading.Thread(target=self.run,name='monitor-recording',daemon=True)
        self.worker.start()

    def topic(self,key,data,session,wall):
        if self.done.is_set():return
        row=(max(0.,time.monotonic()-self.started),key,data,session or self.store.session.get('session'),wall)
        try:self.queue.put_nowait(row)
        except queue.Full:self.dropped+=1

    def run(self):
        next_frame=0.;next_flush=0.
        try:
            while not self.done.is_set() or not self.queue.empty():
                for _ in range(1000):
                    try:row=self.queue.get_nowait()
                    except queue.Empty:break
                    self.journal.topic(*row)
                t=time.monotonic()-self.started
                if t>=next_frame:
                    self.journal.frame(t,self.store.snapshot());next_frame=t+.1
                if t>=next_flush:
                    self.journal.stream.flush();self.journal.save_meta();next_flush=t+1
                if not self.done.is_set():self.done.wait(.02)
            self.journal.frame(time.monotonic()-self.started,self.store.snapshot())
            self.journal.close(dropped=self.dropped)
        except Exception as exc:
            self.error=str(exc);self.done.set()
            try:self.journal.close('interrupted',self.error,self.dropped)
            except OSError:self.journal.stream.close()

    def stop(self):
        self.done.set();self.worker.join(timeout=15)
        if self.worker.is_alive():raise RuntimeError('recording is still saving')
        return self.journal.meta.copy()

    def status(self):
        return {**self.journal.meta,'duration_s':(time.monotonic()-self.started if self.worker.is_alive() else self.journal.meta['duration_s']),
                'dropped_events':self.dropped,'error':self.error,'active':not self.done.is_set()}


class Library:
    def __init__(self, root, store):
        self.root=Path(root);self.directory=self.root/'output/monitor_recordings';self.directory.mkdir(parents=True,exist_ok=True)
        self.store=store;self.lock=threading.RLock();self.recorder=None;self.job=None;self.index_cache={}

    def path(self, name):
        if not isinstance(name,str) or not ID.fullmatch(name):raise ValueError('invalid recording id')
        p=self.directory/name
        if p.is_symlink() or p.resolve().parent!=self.directory.resolve():raise ValueError('invalid recording path')
        return p

    def topic(self,key,data,session,wall):
        r=self.recorder
        if r:r.topic(key,data,session,wall)

    def start(self,label):
        if not isinstance(label,str) or len(label)>120:raise ValueError('recording name must be at most 120 characters')
        with self.lock:
            if self.recorder and self.recorder.worker.is_alive():raise ValueError('已有录制正在进行')
            journal=Journal(self.directory/new_id(),label or '乒乓球测试','monitor')
            self.recorder=Recorder(self.store,journal)
            return self.recorder.status()

    def stop(self,name=None):
        with self.lock:
            if not self.recorder:raise ValueError('没有正在进行的录制')
            if name and name!=self.recorder.journal.meta['id']:raise ValueError('recording changed; refresh the page')
            return self.recorder.stop()

    def status(self):
        with self.lock:
            return {'recording':self.recorder.status() if self.recorder else None,'job':dict(self.job) if self.job else None}

    def list(self):
        records=[]
        for p in sorted(self.directory.glob('rec_*/metadata.json'),reverse=True):
            try:
                self.path(p.parent.name);d=json.loads(p.read_text())
                if d.get('status')=='recording' and not self.writing(d['id']):
                    d['status']='interrupted';d['error']='录制服务曾退出，可回放已保存部分'
                records.append(d)
            except (OSError,ValueError,KeyError):continue
        return {'items':records,**self.status()}

    def writing(self,name):
        return bool((self.recorder and self.recorder.journal.meta['id']==name and self.recorder.worker.is_alive()) or
                    (self.job and self.job.get('id')==name and self.job['status']=='running'))

    def _index(self,name):
        p=self.path(name)
        with self.lock:
            if self.writing(name):
                raise ValueError('请先停止保存，再打开回放')
            if name not in self.index_cache:
                index_path=p/'index.json'
                if index_path.exists():index=json.loads(index_path.read_text())
                else:
                    index=[]
                    with (p/'recording.jsonl').open('rb') as f:
                        while True:
                            offset=f.tell();line=f.readline(MAX_LINE+1)
                            if not line or not line.endswith(b'\n'):break
                            row=json.loads(line)
                            if row.get('kind')=='frame':index.append([row['t'],offset])
                    atomic_json(index_path,index)
                self.index_cache[name]=index
            return self.index_cache[name]

    def frames(self,name,start=0.,seconds=5.):
        if not math.isfinite(start) or not math.isfinite(seconds) or start<0 or not 0<seconds<=35:
            raise ValueError('invalid replay range')
        p=self.path(name);index=self._index(name);times=[r[0] for r in index]
        begin=max(0,bisect.bisect_right(times,start)-1);end=bisect.bisect_right(times,start+seconds)
        frames=[]
        with (p/'recording.jsonl').open('rb') as f:
            for t,offset in index[begin:end]:
                f.seek(offset);row=json.loads(f.readline(MAX_LINE+1));frames.append(row)
        return {'id':name,'duration_s':times[-1] if times else 0.,'frames':frames,
                'metadata':json.loads((p/'metadata.json').read_text())}

    def upload(self, stream, length):
        if not 0<length<=2*1024**3:raise ValueError('日志文件须小于 2 GiB')
        name=new_id();directory=self.directory/name;directory.mkdir()
        path=directory/'recording.jsonl';index=[];header=None;summary=None;remaining=length;previous=-1.;events=0
        try:
            with path.open('xb') as target:
                while remaining:
                    line=stream.readline(min(MAX_LINE+1,remaining));remaining-=len(line)
                    if not line:raise ValueError('文件上传中断')
                    if len(line)>MAX_LINE or not line.endswith(b'\n'):raise ValueError('日志行不完整或过大')
                    row=json.loads(line)
                    if not isinstance(row,dict):raise ValueError('日志记录须为 JSON 对象')
                    if row.get('kind') in ('header','summary') and not isinstance(row.get('metadata',{}),dict):raise ValueError('无效日志元数据')
                    if header is None:
                        if row.get('kind')!='header' or row.get('schema')!=SCHEMA:raise ValueError('请选择 Monitor 导出的 JSONL 文件')
                        header=row
                    if row.get('kind')=='frame':
                        t=row['t'];s=row['snapshot']
                        if type(t) not in (float,int) or not math.isfinite(t) or t<0 or t<previous or not isinstance(s,dict):raise ValueError('无效时间轴')
                        if not all(isinstance(s.get(k),dict) for k in ('topics','session','server')):raise ValueError('无效快照')
                        if any(not isinstance(v,dict) or not isinstance(v.get('data'),dict) for v in s['topics'].values()):raise ValueError('无效 topic 快照')
                        previous=t;index.append([t,target.tell()])
                    if row.get('kind')=='summary':summary=row.get('metadata')
                    if row.get('kind')=='topic':events+=1
                    target.write(line)
            if not index:raise ValueError('日志中没有回放帧')
            meta={**header.get('metadata',{}),**(summary or {}),'schema':SCHEMA,'id':name,'status':'imported',
                  'source':'uploaded','duration_s':index[-1][0],'frames':len(index),'events':events}
            atomic_json(directory/'metadata.json',meta);atomic_json(directory/'index.json',index);return meta
        except Exception:
            path.unlink(missing_ok=True);directory.rmdir();raise
