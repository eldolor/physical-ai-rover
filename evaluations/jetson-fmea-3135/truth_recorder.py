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
  sample; the writer writes what it holds, in order, one line a write, and
  is the only thread that writes (`stop()` lets it drain). A failed write
  is reported on stderr and that line is lost (a hole in the log, never a
  dead recorder) -- as the loop before; so is a sample that raises. The
  backlog waiting for the writer is capped at `BACKLOG_MAX` lines (10 min
  at 50 Hz), the oldest dropped and counted; with the batch a stalled
  writer already holds, at most twice that is resident, never unbounded.
* `lock_memory()`: `mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT)` --
  every page the process has touched or touches stays resident (no major
  fault on its code or heap under reclaim), without populating what it
  only maps (1.1 GB of stacks and reservations against 72 MB resident;
  `MCL_CURRENT` alone would fault all of it in). Only with no memlock
  limit: under a finite one `MCL_FUTURE` would make a later thread stack or
  heap mapping fail mid-run, so the lock is then not tried. Answers what
  happened; a refusal is reported, never retried.
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
BACKLOG_MAX = 30000


class Recorder:
    def __init__(self, sample, file, period_s: float = 0.02):
        self.sample, self.file, self.period_s = sample, file, period_s
        self.latest: dict = {}
        self.dropped = 0
        self._stopped = None                    # stop()'s answer, once
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
            try:
                row = self.sample()
                line = json.dumps(row) + "\n"
            except Exception as e:              # noqa: BLE001 -- a hole, never a dead sampler
                print(f"truth sample failed: {e!r}", file=sys.stderr, flush=True)
            else:
                self.latest = row               # one name rebound: a reader never sees half a row
                with self._cv:
                    if self._stop.is_set():     # after the end: the writer's last look is done
                        break
                    if len(self._lines) >= BACKLOG_MAX:
                        self._lines.popleft()
                        self.dropped += 1
                        if self.dropped == 1 or self.dropped % 1000 == 0:
                            print(f"truth backlog full: {self.dropped} rows dropped",
                                  file=sys.stderr, flush=True)
                    self._lines.append(line)
                    self._cv.notify()
            nxt += self.period_s
            time.sleep(max(0.0, nxt - time.monotonic()))

    def _take(self) -> list:
        with self._cv:
            while not self._lines and not self._stop.is_set():
                self._cv.wait(0.5)
            out = list(self._lines)
            self._lines.clear()
        return out

    def _write(self, lines: list) -> None:
        for line in lines:                      # one line a write: a failure loses that line only
            try:
                self.file.write(line)
            except ValueError as e:             # the file closed under us: nothing more can land
                print(f"truth write failed, file closed: {e!r}", file=sys.stderr, flush=True)
                return
            except Exception as e:              # noqa: BLE001 -- a hole in the log, never a dead writer
                print(f"truth write failed: {e!r}", file=sys.stderr, flush=True)
        try:
            self.file.flush()
        except Exception as e:                  # noqa: BLE001
            print(f"truth flush failed: {e!r}", file=sys.stderr, flush=True)

    def _write_loop(self) -> None:
        while True:
            lines = self._take()
            self._write(lines)
            with self._cv:
                if self._stop.is_set() and not self._lines:
                    return

    def stop(self, timeout_s: float = 3.0) -> bool:
        """Stop sampling and let the writer drain what it holds; True if it
        finished within `timeout_s` (a writer still blocked keeps the file to
        itself and finishes on its own: no second thread ever writes).
        Once: a second call answers the first's."""
        if self._stopped is not None:
            return self._stopped
        self._stop.set()
        self._sampler.join(1.0)
        with self._cv:
            self._cv.notify_all()
        self._writer.join(timeout_s)
        done = self._stopped = not self._writer.is_alive()
        print(f"truth recorder stopped: drained={done}, dropped={self.dropped}",
              file=sys.stderr, flush=True)
        return done


def _libc():
    return ctypes.CDLL(None, use_errno=True)


def _kb(v):
    return "unlimited" if v == resource.RLIM_INFINITY else v // 1024


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
           "limit_soft_kb": _kb(soft), "limit_hard_kb": _kb(hard)}
    if not dry and soft != resource.RLIM_INFINITY:
        out.update(locked=False, error="limit_not_raised")   # MCL_FUTURE under a limit: not tried
    elif not dry:
        rc = _libc().mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT)
        e = ctypes.get_errno() if rc != 0 else 0
        out.update(locked=rc == 0, errno=e or None,
                   error=(errno_mod.errorcode.get(e, str(e)) if e else None))
    out.update(vm_lck_kb=_status_kb("VmLck"), vm_rss_kb=_status_kb("VmRSS"))
    return out
