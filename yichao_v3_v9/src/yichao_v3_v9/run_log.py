"""One durable, segmented JSONL journal per Planner invocation."""
import os
import copy
from pathlib import Path
import queue
import sys
import threading
import time
import traceback

from .observer import SegmentedWriter


class RunLog:
    def __init__(self, path, metadata, *, segment_bytes=16 * 1024 * 1024):
        self.path = Path(path)
        self.metadata = metadata
        self.segment_bytes = segment_bytes
        self.summary = {}
        self.pending = queue.Queue(maxsize=4096)
        self.dropped = 0
        self.write_errors = 0
        self.last_error = None

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x')
        self.writer = SegmentedWriter(self.stream, segment_bytes=self.segment_bytes)
        self.started = time.monotonic()
        self.worker = threading.Thread(target=self._drain, name='planner-log', daemon=True)
        self.worker.start()
        self({'kind': 'run_start', 'pid': os.getpid(), 'log_path': str(self.path.resolve()),
              **self.metadata})
        return self

    def __call__(self, record):
        # Detach from mutable relay/ACK dictionaries before the next control tick.
        item = copy.deepcopy({'log_monotonic_s': time.monotonic(), 'log_wall_s': time.time(), **record})
        try:
            self.pending.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def _drain(self):
        while True:
            item = self.pending.get()
            try:
                if item is None:
                    return
                self.writer(item)
            except (OSError, ValueError, TypeError) as exc:
                self.write_errors += 1
                self.last_error = str(exc)
                if self.write_errors == 1:
                    print('Planner log write failed; control continues: '+str(exc), file=sys.stderr, flush=True)
            finally:
                self.pending.task_done()

    def __exit__(self, exc_type, exc, tb):
        try:
            reason = 'finished'
            if exc_type:
                reason = 'interrupted' if issubclass(exc_type, (KeyboardInterrupt, SystemExit)) else 'error'
                self({'kind': 'run_exception', 'exception_type': exc_type.__name__,
                      'reason': str(exc), 'traceback': ''.join(traceback.format_exception(exc_type, exc, tb))})
            self.pending.join()
            self({'kind': 'summary', **self.metadata, **self.summary, 'exit_reason': reason,
                  'log_dropped_records':self.dropped,'log_write_errors':self.write_errors,
                  'log_last_error':self.last_error,
                  'elapsed_s': time.monotonic() - self.started})
        finally:
            self.pending.put(None)
            self.worker.join()
            self.writer.close()
            self.stream.close()
        return False
