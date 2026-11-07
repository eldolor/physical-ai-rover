# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/shim_lat/sitecustomize.py

3.136.C's shim for the robot server and the brain (P5, P6, P7). Put this
directory first on PYTHONPATH with LAT_SHIM_LOG set; it does nothing until
one of these modules is imported, then wraps on the module or class (the
pattern of 3.136.B's `shim/sitecustomize.py`). Nothing in robot/, brain/ or
control/ is edited.

| module            | what                                               | for |
|-------------------|----------------------------------------------------|-----|
| robot.lidar_ld19  | `Ld19Scanner.feed`: each revolution it completes, one row `{"k": "scan", "t", "fwd_m"}` -- `t` the monotonic instant the robot server holds the new scan, `fwd_m` its nearest return within +-25 deg of straight ahead | P5's "first scan showing it" |
| robot.server      | a middleware: each `POST .../stop`, a row `{"k": "stop_in", "t"}` on arrival | P6 (information) |

Criterion 3's plants, read per call from the file `LAT_PLANT_FILE` names
(its words): `stop` -- the robot server holds a `POST .../stop` 100 ms
before handing it on (P6's both ends move); `verdict` -- the brain's
`TieredVision.confirm_local` (the arrival's question to the local tier)
busy-waits 100 ms before the real call (P7's `verdict_s` moves). P5's plant
is the body's (`lat_body.py`, `zero`).

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

_LOG = os.environ.get("LAT_SHIM_LOG", "")
_PLANT_FILE = os.environ.get("LAT_PLANT_FILE", "")
_PLANT_S = 0.1
_FWD_DEG = 25
_lock = threading.Lock()
_fh = None


def _planted(word: str) -> bool:
    if not _PLANT_FILE:
        return False
    try:
        with open(_PLANT_FILE) as f:
            return word in f.read().split()
    except OSError:
        return False


def _write(row: dict) -> None:
    global _fh
    try:
        with _lock:
            if _fh is None:
                _fh = open(_LOG, "a", buffering=1)
            _fh.write(json.dumps({**row, "pid": os.getpid()}) + "\n")
    except OSError:
        pass                     # a full disk never breaks the robot


def _patch_lidar(mod) -> None:
    feed = mod.Ld19Scanner.feed

    def wrapped(self, data, t=None):
        n = feed(self, data, t)
        if n:
            now = time.monotonic()
            with self._lock:
                scan = self._scan
            fwd = None
            if scan is not None:
                rs = scan["ranges_m"][180 - _FWD_DEG:180 + _FWD_DEG + 1]
                vals = [r for r in rs if r is not None]
                fwd = round(min(vals), 4) if vals else None
            _write({"k": "scan", "t": round(now, 4), "fwd_m": fwd})
        return n
    mod.Ld19Scanner.feed = wrapped


def _patch_server(mod) -> None:
    import asyncio

    @mod.app.middleware("http")
    async def lat_stop(request, call_next):
        if request.method == "POST" and request.url.path.endswith("/stop"):
            _write({"k": "stop_in", "t": round(time.monotonic(), 4)})
            if _planted("stop"):
                await asyncio.sleep(_PLANT_S)
        return await call_next(request)


def _patch_tiered(mod) -> None:
    confirm = mod.TieredVision.confirm_local

    def wrapped(self, *a, **k):
        if _planted("verdict"):
            end = time.perf_counter() + _PLANT_S
            while time.perf_counter() < end:
                pass
        return confirm(self, *a, **k)
    mod.TieredVision.confirm_local = wrapped


_PATCH = {"robot.lidar_ld19": _patch_lidar, "robot.server": _patch_server,
          "brain.tiered": _patch_tiered}


class _Loader(importlib.abc.Loader):
    def __init__(self, inner, name):
        self.inner, self.name = inner, name

    def create_module(self, spec):
        return self.inner.create_module(spec)

    def exec_module(self, module):
        self.inner.exec_module(module)
        _PATCH[self.name](module)   # before any importer binds its names


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path, target=None):
        if name not in _PATCH:
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
    for _n in _PATCH:
        if _n in sys.modules:      # imported before us (never at start-up)
            _PATCH[_n](sys.modules[_n])
    sys.meta_path.insert(0, _Finder())
