"""Asynchronous append-only session records, without deleting old segments."""
import copy
import json
import math
from pathlib import Path
import queue
import threading
import time


def serializable(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite": str(value)}
    if isinstance(value, dict): return {k:serializable(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [serializable(v) for v in value]
    return value


class SessionJournal:
    def __init__(self, directory, segment_bytes=16*1024*1024):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        if any(self.directory.glob("events.*.jsonl")):
            raise ValueError("session directory already contains a journal")
        self.segment_bytes = segment_bytes
        self.pending = queue.Queue(maxsize=8192)
        self.dropped = 0
        self.error = None
        self.done = threading.Event()
        self.worker = threading.Thread(target=self._run, name='session-journal', daemon=True)
        self.worker.start()

    def write(self, kind, **payload):
        record = copy.deepcopy({'kind': kind, 'wall_s': time.time(), 'monotonic_s': time.monotonic(), **payload})
        try:
            self.pending.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        stream = None
        try:
            index = 0
            while not self.done.is_set() or not self.pending.empty():
                try:
                    record = self.pending.get(timeout=.1)
                except queue.Empty:
                    if stream: stream.flush()
                    continue
                if stream is None or stream.tell() >= self.segment_bytes:
                    if stream: stream.close()
                    stream = (self.directory/f'events.{index:05d}.jsonl').open('x', buffering=1)
                    index += 1
                stream.write(json.dumps(serializable(record), allow_nan=False, separators=(',', ':'))+'\n')
        except Exception as error:
            self.error = str(error)
        finally:
            if stream: stream.close()

    def close(self):
        self.done.set()
        self.worker.join(timeout=15)
        if self.worker.is_alive():
            raise RuntimeError('session journal still saving')
        (self.directory/'journal_status.json').write_text(json.dumps(
            {'dropped_events': self.dropped, 'write_error': self.error,
             'unwritten_events': self.pending.qsize()}, indent=2)+'\n')
