# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/truth_recorder.py

3.151: the truth body's recorder made to hold under memory pressure
(3.135.B's F2 had 0.69-3.01 s holes in the truth log at 15-99 MB
`MemAvailable`, with the body at `oom_score_adj` -900: stalled, not killed).

* `Recorder(sample, file, period_s)`: a sampler thread takes `sample()`
  every `period_s` and hands its JSON line to a writer thread, so a write
  that blocks (a page-cache allocation in reclaim) never delays the next
  sample; the writer writes what it holds, in order, and `stop()` drains it.
  A failed write is reported on stderr and those lines are lost (a hole in
  the log, never a dead recorder) -- as the loop before.
* `lock_memory()`: `mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT)` --
  every page the process has touched or touches stays resident (no major
  fault on its code or heap under reclaim), without populating what it
  only maps (1.1 GB of stacks and reservations against 72 MB resident;
  `MCL_CURRENT` alone would fault all of it in). Answers what happened; a
  refusal is reported, never retried.
"""

import collections
import ctypes
import errno as errno_mod
import json
import resource
import sys
import threading
import time

MCL_CURRENT, MCL_FUTURE, MCL_ONFAULT = 1, 2, 4          # <sys/mman.h> on Linux


class Recorder:
    def __init__(self, sample, file, period_s: float = 0.02):
        self.sample, self.file, self.period_s = sample, file, period_s
        self.latest: dict = {}
        self._lines = collections.deque()
        self._cv = threading.Condition()
        self._stop = threading.Event()
        self._sampler = threading.Thread(target=self._sample_loop, daemon=True, name="fmea-truth")
        self._writer = threading.Thread(target=self._write_loop, daemon=True, name="fmea-truth-writer")

    def start(self) -> "Recorder":
        self._writer.start()
        self._sampler.start()
        return self

    def _sample_loop(self) -> None:
        nxt = time.monotonic()
        while not self._stop.is_set():
            row = self.sample()
            self.latest = row                   # one name rebound: a reader never sees half a row
            line = json.dumps(row) + "\n"
            with self._cv:
                self._lines.append(line)
                self._cv.notify()
            nxt += self.period_s
            time.sleep(max(0.0, nxt - time.monotonic()))

    def _take(self) -> str:
        with self._cv:
            while not self._lines and not self._stop.is_set():
                self._cv.wait(0.5)
            out = "".join(self._lines)
            self._lines.clear()
        return out

    def _write(self, text: str) -> None:
        if not text:
            return
        try:
            self.file.write(text)
            self.file.flush()
        except OSError as e:                    # a hole in the log, never a dead recorder
            print(f"truth write failed: {e}", file=sys.stderr, flush=True)

    def _write_loop(self) -> None:
        while not self._stop.is_set():
            self._write(self._take())

    def stop(self, timeout_s: float = 5.0) -> None:
        """Stop sampling, then write everything taken (the writer may be in a
        blocked write: it finishes that first)."""
        self._stop.set()
        self._sampler.join(timeout_s)
        with self._cv:
            self._cv.notify_all()
        self._writer.join(timeout_s)
        self._write(self._take())


def _libc():
    return ctypes.CDLL(None, use_errno=True)


def _kb(v):
    return None if v == resource.RLIM_INFINITY else v // 1024


def _status_kb(field: str):
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith(field + ":"):
                    return int(line.split()[1])
    except OSError:
        pass
    return None


def lock_memory(dry: bool = False) -> dict:
    """Lock this process's touched pages (see the module docstring). `dry`
    only reports the limits (never locks the caller). `limit_*_kb` is
    "unlimited" for no limit."""
    soft, hard = resource.getrlimit(resource.RLIMIT_MEMLOCK)
    out = {"locked": None, "errno": None, "error": None,
           "limit_soft_kb": "unlimited" if soft == resource.RLIM_INFINITY else _kb(soft),
           "limit_hard_kb": "unlimited" if hard == resource.RLIM_INFINITY else _kb(hard)}
    if not dry:
        rc = _libc().mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT)
        e = ctypes.get_errno() if rc != 0 else 0
        out.update(locked=rc == 0, errno=e or None,
                   error=(errno_mod.errorcode.get(e, str(e)) if e else None))
    out.update(vm_lck_kb=_status_kb("VmLck"), vm_rss_kb=_status_kb("VmRSS"))
    return out
