# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/shim/sitecustomize.py

3.136.B's timing shim for the brain (P4's overlap count, P12's room work).
Put this directory first on PYTHONPATH with PERF_SHIM_LOG set and every
Python process started under it imports this file at start-up; it does
nothing until one of these modules is imported, then wraps, on the module
or class (so every caller sees the wrapper, `from x import f` included):

| kind    | what is timed                                   | P   |
|---------|-------------------------------------------------|-----|
| segment | `brain.room_labels.segment()` (the room cut)    | P12 |
| walk    | `brain.room_labels._walk_dist()` (spacing's walk)| P12 |
| map     | `control.remote_world.RemoteWorld.get_map()`    | P4  |
| vlm     | `brain.local_vlm.LocalVLMClient._post()` (a frame question to VE) | P4 |

One JSON line per call: `{"k", "t0", "t1", "tid", "pid", "caller"}` (epoch
seconds, the calling thread, the calling file's name). Nothing in brain/ or control/ is edited.

Criterion 3: PERF_SHIM_PLANT="segment,walk" (any kinds) busy-waits
PERF_SHIM_PLANT_S (0.1) inside the wrapped call, before the real one, so
`plant_room.py` can check that the timing reads it back.

Ubuntu ships its own sitecustomize (apport's hook); this one shadows it on
the path, so it runs that one too.
"""

import importlib.abc
import importlib.util
import json
import os
import sys
import threading
import time

_LOG = os.environ.get("PERF_SHIM_LOG", "")
_PLANT = {k.strip() for k in os.environ.get("PERF_SHIM_PLANT", "").split(",") if k.strip()}
_PLANT_S = float(os.environ.get("PERF_SHIM_PLANT_S", "0.1"))
_lock = threading.Lock()
_fh = None


def _write(kind: str, t0: float, t1: float, caller: str) -> None:
    global _fh
    try:
        with _lock:
            if _fh is None:
                _fh = open(_LOG, "a", buffering=1)
            _fh.write(json.dumps({"k": kind, "t0": round(t0, 6), "t1": round(t1, 6),
                                  "tid": threading.get_ident(), "pid": os.getpid(),
                                  "caller": caller}) + "\n")
    except OSError:
        pass                     # a full disk never breaks the brain


def _timed(kind: str, fn):
    if getattr(fn, "_perf_shim", False):
        return fn

    def wrapper(*a, **k):
        # The calling module: the room labeller's cut and the rooms policy's
        # cut of its coarse map are both `segment()` (P12 reports them apart).
        f = sys._getframe(1)
        caller = os.path.basename(f.f_code.co_filename)
        t0 = time.time()
        try:
            if kind in _PLANT:
                # A busy wait, not a sleep (amendment 3): a sleeping core
                # drops its clock, and the real call after it ran slower.
                end = time.perf_counter() + _PLANT_S
                while time.perf_counter() < end:
                    pass
            return fn(*a, **k)
        finally:
            _write(kind, t0, time.time(), caller)
    wrapper._perf_shim = True
    wrapper.__wrapped__ = fn
    wrapper.__name__ = getattr(fn, "__name__", kind)
    wrapper.__doc__ = getattr(fn, "__doc__", None)
    return wrapper


def _patch(name: str, mod) -> None:
    if name == "brain.room_labels":
        mod.segment = _timed("segment", mod.segment)
        mod._walk_dist = _timed("walk", mod._walk_dist)
    elif name == "control.remote_world":
        mod.RemoteWorld.get_map = _timed("map", mod.RemoteWorld.get_map)
    elif name == "brain.local_vlm":
        mod.LocalVLMClient._post = _timed("vlm", mod.LocalVLMClient._post)


TARGETS = ("brain.room_labels", "control.remote_world", "brain.local_vlm")


class _Loader(importlib.abc.Loader):
    def __init__(self, inner, name):
        self.inner, self.name = inner, name

    def create_module(self, spec):
        return self.inner.create_module(spec)

    def exec_module(self, module):
        self.inner.exec_module(module)
        _patch(self.name, module)   # before any importer binds its names


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
    here = os.path.dirname(os.path.abspath(__file__))
    for d in sys.path:
        if not d or os.path.abspath(d) == here:
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
if _LOG:
    for _n in TARGETS:
        if _n in sys.modules:      # imported before us (never at start-up)
            _patch(_n, sys.modules[_n])
    sys.meta_path.insert(0, _Finder())
