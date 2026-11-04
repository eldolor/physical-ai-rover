# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/wheel-loop-3141/tracer/sitecustomize.py

3.141's stall tracer for the robot server: what holds its event loop (the
loop `wheel_loop()` and the watchdog run on) when a tick runs late. Put this
directory on PYTHONPATH with TRACE_3141_LOG set -- first, or right after
3.136's shim, which chains it (`steady.sh STEADY_TRACER=1`; never before the
shim, which would chain it again); it acts only in the
process started on `robot.server` and edits nothing in `robot/`.

One JSON line per event (epoch seconds):

| k      | what                                                                 |
|--------|----------------------------------------------------------------------|
| gap    | the 5 ms probe on the loop woke `gap_ms` late (over GAP_MS)          |
| tick   | a 50 ms probe shaped as `wheel_loop()` (sleep, then dt): dt > TICK_MS |
| stall  | the sampler thread saw the loop's heartbeat older than STALL_MS: every thread's stack at first sight, and each thread's CPU (ms, /proc) from a baseline at most BASELINE_S before it to the stall's end |
| gc     | one collection over GC_MS: generation, duration, collected           |
| vet    | `SafetyController.vet_wheel_velocity` over VET_MS                     |
| req    | an HTTP request over REQ_MS, end to end                              |
| heap   | every HEAP_S: gc counts, tracked objects per generation (only with TRACE_3141_HEAP_OBJECTS=1), frozen, RSS, the event loop thread's CPU (from the process's start: only diffs mean anything), GC rows dropped, the tracer's own threads' CPU and its probes' work on the loop (ms, cumulative from the tracer's start: diff two rows for a window) |

The sampler needs the GIL to look, so a C call holding it (a `json.dumps`)
is seen only after it returns: the per-thread CPU over the stall is what
says who ran. Ubuntu's own sitecustomize is chained, as 3.136's shim does.
"""

import asyncio
import collections
import gc
import importlib.abc
import importlib.util
import json
import os
import sys
import threading
import time
import traceback

_LOG = os.environ.get("TRACE_3141_LOG", "")
GAP_MS = float(os.environ.get("TRACE_3141_GAP_MS", "15"))
TICK_MS = float(os.environ.get("TRACE_3141_TICK_MS", "70"))
STALL_MS = float(os.environ.get("TRACE_3141_STALL_MS", "25"))
GC_MS = float(os.environ.get("TRACE_3141_GC_MS", "2"))
VET_MS = float(os.environ.get("TRACE_3141_VET_MS", "5"))
REQ_MS = float(os.environ.get("TRACE_3141_REQ_MS", "20"))
HEAP_S = float(os.environ.get("TRACE_3141_HEAP_S", "30"))
HEAP_OBJECTS = os.environ.get("TRACE_3141_HEAP_OBJECTS") == "1"
CLK = os.sysconf("SC_CLK_TCK")

_lock = threading.Lock()
_fh = None
_hb = [time.monotonic()]          # the loop's heartbeat (monotonic)


def _write(rec: dict) -> None:
    global _fh
    try:
        with _lock:
            if _fh is None:
                # Truncated: this process's trace only, whoever launched it.
                _fh = open(_LOG, "w", buffering=1)
            _fh.write(json.dumps(rec) + "\n")
    except OSError:
        pass


def _thread_cpu() -> dict:
    """native tid -> (comm, cpu ticks) for every thread of this process."""
    out = {}
    try:
        tids = os.listdir("/proc/self/task")
    except OSError:
        return out
    for tid in tids:
        try:
            with open(f"/proc/self/task/{tid}/stat") as f:
                s = f.read()
        except OSError:
            continue
        comm = s[s.index("(") + 1:s.rindex(")")]
        fields = s[s.rindex(")") + 2:].split()
        out[int(tid)] = (comm, int(fields[11]) + int(fields[12]))
    return out


def _stacks() -> list:
    names = {t.ident: (t.name, t.native_id) for t in threading.enumerate()}
    me = threading.get_ident()
    out = []
    for ident, frame in sys._current_frames().items():
        if ident == me:
            continue
        name, nid = names.get(ident, ("?", None))
        st = traceback.extract_stack(frame)[-10:]
        out.append({"name": name, "tid": nid,
                    "stack": [f"{os.path.basename(fr.filename)}:{fr.lineno} {fr.name}" for fr in st]})
    return out


_own = []                         # native ids of the tracer's own threads
_probe_s = [0.0]                  # the probes' own work on the loop, s (not their sleeps)
BASELINE_S = 1.0                  # how old a stall's CPU baseline may be
SAMPLE_S = 0.01                   # the sampler's look at the heartbeat (STALL_MS is 25)


_loop_tid = []                    # the event loop's thread, once it runs


def _cpu_ms_of(tids) -> int:
    ticks = 0
    for tid in list(tids):
        try:
            with open(f"/proc/self/task/{tid}/stat") as f:
                st = f.read()
        except OSError:
            continue
        fields = st[st.rindex(")") + 2:].split()
        ticks += int(fields[11]) + int(fields[12])
    return round(ticks * 1000 / CLK)


def _own_cpu_ms() -> int:
    return _cpu_ms_of(_own)


def _sampler() -> None:
    """Each SAMPLE_S: only the heartbeat's age; every BASELINE_S, every thread's
    CPU (/proc), so a stall's CPU is measured from a baseline taken before it
    began -- a C call holding the GIL keeps this thread out until it returns.
    Reading /proc every 5 ms cost ~17% of a core on the desktop."""
    _own.append(threading.get_native_id())
    base, base_t = _thread_cpu(), time.monotonic()
    in_stall, start_cpu, rec = False, None, None
    while True:
        time.sleep(SAMPLE_S)
        now = time.monotonic()
        _flush_gc()
        age = (now - _hb[0]) * 1000
        if not in_stall and age > STALL_MS:
            in_stall, start_cpu = True, base
            rec = {"k": "stall", "t": time.time(), "age_ms": round(age, 1),
                   "baseline_age_ms": round((now - base_t) * 1000), "threads": _stacks()}
        elif in_stall and age <= STALL_MS:
            cpu = {}
            for tid, (comm, ticks) in _thread_cpu().items():
                d = ticks - start_cpu.get(tid, (comm, 0))[1]
                if d:
                    cpu[f"{comm}/{tid}"] = round(d * 1000 / CLK)
            rec["cpu_ms"] = cpu
            rec["t_end"] = time.time()
            _write(rec)
            in_stall = False
            base, base_t = _thread_cpu(), time.monotonic()
        elif in_stall:
            rec["age_ms"] = round(max(rec["age_ms"], age), 1)
        elif now - base_t >= BASELINE_S:
            base, base_t = _thread_cpu(), now


def _heap() -> None:
    _own.append(threading.get_native_id())
    while True:
        time.sleep(HEAP_S)
        t0 = time.perf_counter()
        sizes = None
        if HEAP_OBJECTS:
            # Walks every tracked object with the GIL held: a pause of its own
            # on a big heap, so off unless asked (TRACE_3141_HEAP_OBJECTS=1).
            try:
                sizes = [len(gc.get_objects(g)) for g in range(3)]
            except Exception:  # noqa: BLE001 -- generation numbering differs by version
                pass
        cost = (time.perf_counter() - t0) * 1000 if HEAP_OBJECTS else None
        try:
            with open("/proc/self/statm") as f:
                rss = int(f.read().split()[1]) * os.sysconf("SC_PAGE_SIZE") // 2**20
        except OSError:
            rss = None
        _write({"k": "heap", "t": time.time(), "count": gc.get_count(), "sizes": sizes,
                "cost_ms": None if cost is None else round(cost, 1), "frozen": gc.get_freeze_count(), "rss_mb": rss,
                "tracer_cpu_ms": _own_cpu_ms(), "probe_ms": round(_probe_s[0] * 1000),
                "loop_cpu_ms": _cpu_ms_of(_loop_tid), "gc_dropped": _gc_dropped[0],
                "stats": gc.get_stats(), "threads": threading.active_count()})


_gc_t0 = [0.0]
# A collection can start inside `_write` (json.dumps allocates) with its lock
# held; a callback that wrote there would wait on that lock for ever on its
# own thread -- the event loop's, as like as not. So it only appends here
# (never blocks, takes no lock), and the sampler writes the rows out.
_gc_rows: collections.deque = collections.deque(maxlen=10000)
_gc_dropped = [0]


def _gc_cb(phase, info):
    if phase == "start":
        _gc_t0[0] = time.perf_counter()
        return
    ms = (time.perf_counter() - _gc_t0[0]) * 1000
    if ms >= GC_MS:
        if len(_gc_rows) == _gc_rows.maxlen:
            _gc_dropped[0] += 1
        _gc_rows.append({"k": "gc", "t": time.time(), "ms": round(ms, 2),
                         "gen": info.get("generation"), "collected": info.get("collected"),
                         "thread": threading.current_thread().name})


def _flush_gc() -> None:
    while _gc_rows:
        _write(_gc_rows.popleft())


async def _probe5():
    last = time.monotonic()
    while True:
        await asyncio.sleep(0.005)
        now = time.monotonic()
        _hb[0] = now
        late = (now - last) * 1000 - 5
        if late > GAP_MS:
            _write({"k": "gap", "t": time.time(), "gap_ms": round(late, 1)})
        last = now
        _probe_s[0] += time.monotonic() - now


async def _probe50():
    last = time.monotonic()
    while True:
        await asyncio.sleep(0.05)
        now = time.monotonic()
        dt = (now - last) * 1000
        if dt > TICK_MS:
            _write({"k": "tick", "t": time.time(), "dt_ms": round(dt, 1)})
        last = now
        _probe_s[0] += time.monotonic() - now


class _Traced:
    def __init__(self, app):
        self.app = app
        self.started = False

    def __getattr__(self, name):
        return getattr(self.app, name)

    async def __call__(self, scope, receive, send):
        if not self.started:
            self.started = True
            loop = asyncio.get_running_loop()
            _hb[0] = time.monotonic()      # not import time: no stall before the first beat
            _loop_tid.append(threading.get_native_id())
            loop.create_task(_probe5())
            loop.create_task(_probe50())
            threading.Thread(target=_sampler, daemon=True, name="trace-sampler").start()
            threading.Thread(target=_heap, daemon=True, name="trace-heap").start()
            gc.callbacks.append(_gc_cb)
            _write({"k": "start", "t": time.time(), "py": sys.version, "pid": os.getpid(),
                    "gc_threshold": gc.get_threshold()})
        if scope["type"] != "http":
            return await self.app(scope, receive, send)
        t0 = time.time()
        status = [None]

        async def snd(msg):
            if msg["type"] == "http.response.start":
                status[0] = msg["status"]
            await send(msg)
        try:
            return await self.app(scope, receive, snd)
        finally:
            ms = (time.time() - t0) * 1000
            if ms > REQ_MS:
                _write({"k": "req", "t0": t0, "ms": round(ms, 1), "path": scope.get("path"),
                        "method": scope.get("method"), "status": status[0]})


def _timed_vet(fn):
    def record(t0, cmd, reason):
        try:
            ms = (time.perf_counter() - t0) * 1000
            if ms > VET_MS:
                _write({"k": "vet", "t": time.time(), "ms": round(ms, 2),
                        "thread": threading.current_thread().name,
                        "cmd": [round(float(x), 2) for x in cmd], "reason": reason[:100]})
        except Exception:  # noqa: BLE001 -- a record must never change a vet
            pass

    def wrapper(*a, **k):
        t0 = time.perf_counter()
        cmd = a[1:3] or [k.get("left_rad_s", 0.0), k.get("right_rad_s", 0.0)]
        try:
            out = fn(*a, **k)              # the safety answer, untouched
        except BaseException:
            record(t0, cmd, "raised")
            raise
        record(t0, cmd, str(out[2] or ""))
        return out
    wrapper.__wrapped__ = fn
    return wrapper


def _patch(name: str, mod) -> None:
    if name == "robot.safety":
        mod.SafetyController.vet_wheel_velocity = _timed_vet(mod.SafetyController.vet_wheel_velocity)
    elif name == "robot.server":
        mod.app = _Traced(mod.app)


TARGETS = ("robot.safety", "robot.server")


class _Loader(importlib.abc.Loader):
    def __init__(self, inner, name):
        self.inner, self.name = inner, name

    def create_module(self, spec):
        return self.inner.create_module(spec)

    def exec_module(self, module):
        self.inner.exec_module(module)
        _patch(self.name, module)


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path, target=None):
        if name not in TARGETS:
            return None
        for f in sys.meta_path:
            if f is self:
                continue
            spec = f.find_spec(name, path, target) if hasattr(f, "find_spec") else None
            if spec is not None and spec.loader is not None:
                spec.loader = _Loader(spec.loader, name)
                return spec
        return None


def _chain_system_sitecustomize() -> None:
    # Only what comes AFTER this directory: 3.136's shim, first on the path,
    # chains this file, and looking before it would load the shim again.
    here = os.path.dirname(os.path.realpath(__file__))
    paths = [os.path.realpath(d) for d in sys.path if d]
    after = paths[paths.index(here) + 1:] if here in paths else paths
    for d in after:
        if d == here:
            continue
        p = os.path.join(d, "sitecustomize.py")
        if os.path.isfile(p):
            try:
                spec = importlib.util.spec_from_file_location("_system_sitecustomize", p)
                spec.loader.exec_module(importlib.util.module_from_spec(spec))
            except Exception:  # noqa: BLE001 -- the system hook is optional
                pass
            return


_chain_system_sitecustomize()
if _LOG and any("robot.server" in a for a in sys.argv):
    sys.meta_path.insert(0, _Finder())
