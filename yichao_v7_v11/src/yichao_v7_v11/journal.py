"""Asynchronous append-only session journal with explicit loss/error status."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import queue
import threading
import time


def _serializable(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite": str(value)}
    if isinstance(value, dict):
        return {str(key): _serializable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_serializable(item) for item in value]
    return value


class SessionJournal:
    def __init__(self, directory, *, segment_bytes=32 * 1024 * 1024):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.segment_bytes = int(segment_bytes)
        self.pending = queue.Queue(maxsize=20000)
        self.dropped = 0
        self.error = None
        self.done = threading.Event()
        self.worker = threading.Thread(target=self._run, daemon=True, name="v7-journal")
        self.worker.start()

    def write(self, kind, **payload):
        record = copy.deepcopy(
            {"kind": kind, "wall_s": time.time(), "monotonic_s": time.monotonic(), **payload}
        )
        try:
            self.pending.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        stream = None
        index = 0
        try:
            while not self.done.is_set() or not self.pending.empty():
                try:
                    record = self.pending.get(timeout=0.1)
                except queue.Empty:
                    if stream:
                        stream.flush()
                    continue
                if stream is None or stream.tell() >= self.segment_bytes:
                    if stream:
                        stream.close()
                    stream = (self.directory / f"events.{index:05d}.jsonl").open(
                        "x", encoding="utf-8", buffering=1
                    )
                    index += 1
                stream.write(
                    json.dumps(_serializable(record), separators=(",", ":"), allow_nan=False)
                    + "\n"
                )
        except Exception as error:  # pragma: no cover - surfaced in status
            self.error = str(error)
        finally:
            if stream:
                stream.close()

    def close(self):
        self.done.set()
        self.worker.join(timeout=20.0)
        if self.worker.is_alive():
            raise RuntimeError("session journal did not finish")
        (self.directory / "journal_status.json").write_text(
            json.dumps(
                {
                    "dropped_events": self.dropped,
                    "write_error": self.error,
                    "unwritten_events": self.pending.qsize(),
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

