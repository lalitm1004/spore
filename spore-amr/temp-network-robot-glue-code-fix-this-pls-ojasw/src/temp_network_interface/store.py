"""Durable storage for the network layer's global state.

An append-only JSONL journal: one record per line, written with a single flush,
so a crash can only ever truncate the last line, which replay skips. This is the
durable source of truth; the in-memory `Fleet` is rebuilt from it on startup via
`Fleet.load`.

Records are the two events the fleet undergoes:

    {"type": "status",  "bot_id": 5, "status":  {...}}   # a robot reported
    {"type": "command", "bot_id": 5, "command": {...}}   # a command was issued

Stdlib + JSON only, mirroring the webots implementation's file conventions.

Pure: no grpc.
"""

from __future__ import annotations

import json
import threading
from pathlib import Path


class Journal:
    """An append-only, flush-per-line JSONL file."""

    def __init__(self, path):
        self.path = Path(path)
        self._handle = None
        self._lock = threading.Lock()

    def open(self) -> "Journal":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("a")
        return self

    def append(self, record: dict) -> None:
        line = json.dumps(record, separators=(",", ":"))
        with self._lock:
            self._handle.write(line + "\n")
            self._handle.flush()

    def read(self):
        """Yield records in order, skipping a trailing partial line."""
        if not self.path.exists():
            return
        with self.path.open("r") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    yield json.loads(line)
                except ValueError:
                    # The last line may be truncated by a crash; drop it.
                    continue

    def close(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None

    def __enter__(self) -> "Journal":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()
