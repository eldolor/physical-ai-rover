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
| stall  | the sampler thread saw the loop's heartbeat older than STALL_MS: every thread's stack at first sight, the drain's time just before it, the longest age seen and its end (no per-thread CPU: amendment 4) |
| gc     | one collection over GC_MS: generation, duration, collected           |
| vet    | `SafetyController.vet_wheel_velocity` over VET_MS; with TRACE_3141_VET_PARTS=1 its `parts`: per call path (`pivot_scale/get_scan`), [exclusive ms, calls] -- HardwareRobot's reads and SafetyController's stages, so they add up (3.141.E0) |
| req    | an HTTP request over REQ_MS, end to end                              |
| heap   | every HEAP_S: gc counts, tracked objects per generation (only with TRACE_3141_HEAP_OBJECTS=1), frozen, RSS, the event loop thread's CPU (from the process's start: only diffs mean anything), rows dropped from the queue, the tracer's own threads' CPU and its probes' work on the loop (ms, cumulative from the tracer's start: diff two rows for a window) |

The sampler needs the GIL to look, so a C call holding it (a `json.dumps`)
is seen only after it returns: what names a stall is the `gc` and `vet` rows
that overlap it, and its stacks at first sight. Ubuntu's own sitecustomize
is chained, as 3.136's shim does.
"""

import asyncio
import atexit
import collections
import gc
import importlib.abc
import importlib.util
import json
import math
import os
import sys
import threading
import time
import traceback

_LOG = os.environ.get("TRACE_3141_LOG", "")
GAP_MS = float(os.environ.get("TRACE_3141_GAP_MS", "15"))
TICK_MS = float(os.environ.get("TRACE_3141_TICK_MS", "60"))
STALL_MS = float(os.environ.get("TRACE_3141_STALL_MS", "25"))
GC_MS = float(os.environ.get("TRACE_3141_GC_MS", "2"))
VET_MS = float(os.environ.get("TRACE_3141_VET_MS", "5"))
REQ_MS = float(os.environ.get("TRACE_3141_REQ_MS", "20"))
HEAP_S = float(os.environ.get("TRACE_3141_HEAP_S", "30"))
HEAP_OBJECTS = os.environ.get("TRACE_3141_HEAP_OBJECTS") == "1"
# 3.141.E0: each slow vet row splits its time into the reads and stages below.
VET_PARTS = os.environ.get("TRACE_3141_VET_PARTS") == "1"
CLK = os.sysconf("SC_CLK_TCK")

_PID = os.getpid()                   # a fork's child has no sampler: its rows are never written
_rows: collections.deque = collections.deque(maxlen=20000)
_rows_dropped = [0]
_sampler_ticks = [0]                 # the sampler's passes: a heap row's proof it is alive
_sampler_errors = [0]
_heap_errors = [0]
_fh = None
_hb = [time.monotonic()]          # the loop's heartbeat (monotonic)


def _write(rec: dict) -> None:
    """Queue a row; never blocks, takes no lock, does no I/O: the event loop,
    a vet and a GC callback call it. Only the sampler thread writes rows out
    (`_drain`), so nothing the loop does waits on the file or on a lock the
    sampler holds while encoding a large row."""
    if len(_rows) == _rows.maxlen:
        _rows_dropped[0] += 1
    _rows.append({**rec, "pid": _PID})


_drain_lock = threading.Lock()       # the sampler's and the exit drain's; never the loop's


def _drain() -> None:
    """The sampler's: every queued row out to the file, appended (a
    restarted robot server keeps its predecessor's rows; a caller clears a
    reused file, as steady.sh does)."""
    with _drain_lock:
        _drain_locked()


def _drain_locked() -> None:
    global _fh
    while True:
        try:
            rec = _rows.popleft()
        except IndexError:                 # empty (the exit drain may race the sampler)
            return
        try:
            if _fh is None:
                _fh = open(_LOG, "a", buffering=1)
            _fh.write(json.dumps(rec) + "\n")
        except (OSError, TypeError, ValueError):
            _rows_dropped[0] += 1          # lost, and counted as lost


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
    """Each SAMPLE_S: drain the queue, then only the heartbeat's age; a stall
    row holds every thread's stack at first sight and the drain's time just
    before it. No other thread's CPU is read: 3.141.D's D1 on the board
    (2026-11-04) found a full /proc read cost ~20 ms -- periodic, it delayed
    the loop; at a stall, it lengthened the stall and measured the wrong
    window (amendment 4)."""
    _own.append(threading.get_native_id())
    in_stall, rec = False, None
    while True:
        time.sleep(SAMPLE_S)
        try:
            in_stall, rec = _sample(in_stall, rec)
        except Exception:  # noqa: BLE001 -- the sampler must outlive one odd read
            _sampler_errors[0] += 1
            if in_stall:
                _rows_dropped[0] += 1          # that stall's row is lost
            in_stall, rec = False, None


def _sample(in_stall, rec):
    """One look: drain the queue, then the heartbeat's age."""
    d0 = time.monotonic()
    _drain()
    now = time.monotonic()
    drain_ms = (now - d0) * 1000           # an encode holds the GIL: a stall right
    _sampler_ticks[0] += 1                 # after a long drain may be the tracer's
    age = (now - _hb[0]) * 1000
    if not in_stall and age > STALL_MS:
        rec = {"k": "stall", "t": time.time(), "age_ms": round(age, 1),
               "drain_ms": round(drain_ms, 1), "threads": _stacks()}
        return True, rec
    if in_stall and age <= STALL_MS:
        rec["t_end"] = time.time()
        _write(rec)
        return False, None
    if in_stall:
        rec["age_ms"] = round(max(rec["age_ms"], age), 1)
    return in_stall, rec


def _heap() -> None:
    _own.append(threading.get_native_id())
    while True:
        time.sleep(HEAP_S)
        try:
            _heap_row()
        except Exception:  # noqa: BLE001 -- the next row must still come
            _heap_errors[0] += 1


def _heap_row() -> None:
    sizes = cost = None
    if HEAP_OBJECTS:
        t0 = time.perf_counter()
        # Walks every tracked object with the GIL held: a pause of its own
        # on a big heap, so off unless asked (TRACE_3141_HEAP_OBJECTS=1).
        try:
            sizes = [len(gc.get_objects(g)) for g in range(3)]
        except Exception:  # noqa: BLE001 -- generation numbering differs by version
            pass
        cost = (time.perf_counter() - t0) * 1000
    try:
        with open("/proc/self/statm") as f:
            rss = int(f.read().split()[1]) * os.sysconf("SC_PAGE_SIZE") // 2**20
    except OSError:
        rss = None
    _write({"k": "heap", "t": time.time(), "count": gc.get_count(), "sizes": sizes,
            "cost_ms": None if cost is None else round(cost, 1), "frozen": gc.get_freeze_count(), "rss_mb": rss,
            "tracer_cpu_ms": _own_cpu_ms(), "probe_ms": round(_probe_s[0] * 1000),
            "loop_cpu_ms": _cpu_ms_of(_loop_tid), "rows_dropped": _rows_dropped[0],
            "sampler_ticks": _sampler_ticks[0], "sampler_errors": _sampler_errors[0],
            "heap_errors": _heap_errors[0],
            "tracer_tids": list(_own), "loop_tid": _loop_tid[:1],
            "stats": gc.get_stats(), "threads": threading.active_count()})


_gc_t0 = [0.0]


def _gc_cb(phase, info):
    if phase == "start":
        _gc_t0[0] = time.perf_counter()
        return
    ms = (time.perf_counter() - _gc_t0[0]) * 1000
    if ms >= GC_MS:
        _write({"k": "gc", "t": time.time(), "ms": round(ms, 2), "gen": info.get("generation"),
                "collected": info.get("collected"), "tid": threading.get_native_id()})


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


_in_vet = threading.local()          # this thread's open vet: {part: ms}
ROBOT_PARTS = ("get_wheel_state", "get_depth_grid", "get_scan", "sensor_age_s", "turn_since_scan")
SAFETY_PARTS = ("forward_clearance", "reverse_clearance", "pivot_scale")


def _timed_part(name, fn):
    """Inside a vet on this thread, add this call's EXCLUSIVE time (its own,
    less the timed calls inside it) under its call path ("pivot_scale/
    get_scan"), and count the call; outside a vet, nothing but the call. The
    parts then add up to the time spent in timed calls, each counted once.
    Never changes the call's result."""
    def wrapper(*a, **k):
        parts = getattr(_in_vet, "parts", None)
        if parts is None:
            return fn(*a, **k)
        stack = _in_vet.stack
        frame = [name, 0.0]                # [name, time in timed children]
        stack.append(frame)
        t0 = time.perf_counter()
        try:
            return fn(*a, **k)
        finally:
            try:
                ms = (time.perf_counter() - t0) * 1000
                key = "/".join(f[0] for f in stack)
                stack.pop()
                if stack:
                    stack[-1][1] += ms
                cell = parts.setdefault(key, [0.0, 0])
                cell[0] += ms - frame[1]
                cell[1] += 1
            except Exception:  # noqa: BLE001 -- a record must never change a call
                pass
    wrapper.__wrapped__ = fn
    return wrapper


def _timed_vet(fn):
    def record(t0, a, k, out, exc, parts=None):
        try:
            ms = (time.perf_counter() - t0) * 1000
            if ms > VET_MS:
                cmd = [a[1] if len(a) > 1 else k.get("left_rad_s"),
                       a[2] if len(a) > 2 else k.get("right_rad_s")]
                reason = (f"raised {type(exc).__name__}" if exc is not None
                          else str(out[2] or ""))
                _write({"k": "vet", "t": time.time(), "ms": round(ms, 2),
                        "thread": threading.current_thread().name,
                        "cmd": [round(float(x), 2) if x is not None and math.isfinite(float(x)) else None
                                for x in cmd],
                        "reason": reason[:100],
                        **({"parts": {n: [round(v[0], 2), v[1]] for n, v in parts.items()}}
                           if parts is not None else {})})
        except Exception:  # noqa: BLE001 -- a record must never change a vet
            pass

    def wrapper(*a, **k):
        t0 = time.perf_counter()
        outer = getattr(_in_vet, "parts", None)
        if VET_PARTS and outer is None:
            _in_vet.parts, _in_vet.stack = {}, []
        parts = None
        try:
            out = fn(*a, **k)              # the safety answer, untouched
        except BaseException as e:
            if outer is None:
                parts, _in_vet.parts = getattr(_in_vet, "parts", None), None
            record(t0, a, k, None, e, parts)
            raise
        if outer is None:
            parts, _in_vet.parts = getattr(_in_vet, "parts", None), None
        record(t0, a, k, out, None, parts)
        return out
    wrapper.__wrapped__ = fn
    return wrapper


def _patch(name: str, mod) -> None:
    if name == "robot.safety":
        if VET_PARTS:
            for n in SAFETY_PARTS:
                if hasattr(mod.SafetyController, n):
                    setattr(mod.SafetyController, n, _timed_part(n, getattr(mod.SafetyController, n)))
        mod.SafetyController.vet_wheel_velocity = _timed_vet(mod.SafetyController.vet_wheel_velocity)
    elif name == "robot.hardware_robot" and VET_PARTS:
        for n in ROBOT_PARTS:
            if hasattr(mod.HardwareRobot, n):
                setattr(mod.HardwareRobot, n, _timed_part(n, getattr(mod.HardwareRobot, n)))
    elif name == "robot.server":
        mod.app = _Traced(mod.app)


TARGETS = ("robot.safety", "robot.server", "robot.hardware_robot")


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


def _drain_at_exit() -> None:
    try:
        _drain()
    except Exception:  # noqa: BLE001 -- exiting anyway
        pass


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
    atexit.register(_drain_at_exit)    # rows still queued when the server stops
