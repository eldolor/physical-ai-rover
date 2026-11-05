# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141.F1 criterion 1: one `/world/map` reply's cost, split into its parts,
on the desktop (or the board) -- the fix is chosen from this.

The robot server's real route (`robot.server`'s app, `drive`-agnostic) over
a real `RosWorld` whose bridge is an in-process fake serving a SLAM grid of
the given size (ROS convention, an ETag per version, a 304 on a match, as
`picar_bridge` does since 3.126). Parts, each timed alone, many times:

* `fetch_full`: `RosWorld._fetch_map()` of a new version (the bridge's JSON
  decoded: `httpx`'s `r.json()`);
* `fetch_304`: the same with nothing new (revalidation only);
* `resample`: `RosWorld._resample()` (3.87's numpy);
* `encode`: `JSONResponse(m).body`, the route's encode before 3.141.F1;
* `encode_sliced`: `robot.server.map_json(m)`, the same bytes in slices;
* `route_full` / `route_304`: the whole route through FastAPI's TestClient,
  a new version / an `If-None-Match` hit;
* `route_same`: a new request, no `If-None-Match`, version unchanged (the
  kept body).

Beside each, a probe thread sleeping 5 ms records how late it woke: its
worst lateness is how long the part held the GIL against another thread
(the wheel loop's position), to the probe's resolution.

    python evaluations/wheel-loop-3141/map_reply_profile.py [--cells 400] [--n 30]
"""

import argparse
import json
import os
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import httpx  # noqa: E402

os.environ.setdefault("APP_SHARED_SECRET", "")


class Bridge:
    """A SLAM grid of `cells` x `cells` at 0.05 m: walls on the border, a few
    rooms, unknown outside a disc -- real-looking runs for the encoders."""

    def __init__(self, cells):
        self.n, self.version, self.session = cells, 1, "s1"
        g = []
        c = cells / 2
        for j in range(cells):
            for i in range(cells):
                r = ((i - c) ** 2 + (j - c) ** 2) ** 0.5
                if r > c * 0.95:
                    g.append(-1)
                elif i % 60 == 0 or j % 60 == 0 or r > c * 0.93:
                    g.append(100)
                else:
                    g.append(0)
        self.grid = g

    def bump(self):
        self.version += 1
        k = (self.version * 7919) % len(self.grid)
        self.grid[k] = 100 if self.grid[k] != 100 else 0

    def handler(self, request):
        if request.url.path == "/slam/pose":
            return httpx.Response(200, json={"session": self.session,
                                             "map": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
                                             "odom": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
                                             "start_truth": {"x_m": 5.0, "y_m": 5.0, "heading_deg": 0.0}})
        if request.url.path == "/slam/map":
            tag = f'"{self.session}:{self.version}"'
            if request.headers.get("if-none-match") == tag:
                return httpx.Response(304, headers={"etag": tag})
            body = json.dumps({"session": self.session, "version": self.version, "resolution_m": 0.05,
                               "width": self.n, "height": self.n,
                               "origin_x_m": -self.n * 0.025, "origin_y_m": -self.n * 0.025,
                               "origin_yaw_rad": 0.0, "data": self.grid})
            return httpx.Response(200, content=body.encode(),
                                  headers={"etag": tag, "content-type": "application/json"})
        return httpx.Response(404)


class Probe:
    """A thread that sleeps 5 ms and keeps its worst lateness."""

    def __init__(self):
        self.worst, self._stop, self._lock = 0.0, False, threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop:
            t = time.perf_counter()
            time.sleep(0.005)
            late = (time.perf_counter() - t - 0.005) * 1000
            with self._lock:
                self.worst = max(self.worst, late)

    def take(self):
        with self._lock:
            w, self.worst = self.worst, 0.0
        return w


def timed(fn, n, probe, before=None):
    ms, held = [], []
    for _ in range(n):
        if before:
            before()
        time.sleep(0.02)
        probe.take()
        t = time.perf_counter()
        fn()
        ms.append((time.perf_counter() - t) * 1000)
        time.sleep(0.01)                       # the probe wakes once after
        held.append(probe.take())
    q = statistics.quantiles(ms, n=10)
    return {"n": n, "p50_ms": round(statistics.median(ms), 2), "p90_ms": round(q[-1], 2),
            "max_ms": round(max(ms), 2), "probe_late_max_ms": round(max(held), 2),
            "probe_late_p50_ms": round(statistics.median(held), 2)}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--cells", type=int, default=400)
    ap.add_argument("--n", type=int, default=30)
    a = ap.parse_args(argv)
    from fastapi.responses import JSONResponse
    from fastapi.testclient import TestClient

    from world.ros_world import RosWorld

    class Truth:
        def get_truth(self):
            return {"usable": True, "source": "sim", "x_m": 5.0, "y_m": 5.0, "heading_deg": 0.0}

    bridge = Bridge(a.cells)
    client = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(bridge.handler))
    world = RosWorld("http://bridge", truth=Truth(), client=client)
    world.get_pose()
    m = world.get_map()
    assert m.get("usable"), m
    probe = Probe()
    out = {"cells": a.cells, "reply_bytes": len(JSONResponse(m).body)}
    raw = world._fetch_map()
    out["fetch_full"] = timed(world._fetch_map, a.n, probe, before=bridge.bump)
    out["fetch_304"] = timed(world._fetch_map, a.n, probe)
    out["resample"] = timed(lambda: world._resample(raw, world._anchor), a.n, probe)
    out["encode"] = timed(lambda: JSONResponse(world.get_map()).body, a.n, probe)
    from robot.server import map_json             # 3.141.F1: the route's encode since
    out["encode_sliced"] = timed(lambda: map_json(world.get_map()), a.n, probe)

    import robot.server as server
    server.get_world = lambda *a_, **k: world       # the app's world: this RosWorld
    tc = TestClient(server.create_app())
    r = tc.get("/world/map")
    assert r.status_code == 200, r.status_code
    out["route_full"] = timed(lambda: tc.get("/world/map"), a.n, probe, before=bridge.bump)
    tag = tc.get("/world/map").headers["etag"]
    out["route_304"] = timed(lambda: tc.get("/world/map", headers={"if-none-match": tag}), a.n, probe)
    out["route_same"] = timed(lambda: tc.get("/world/map"), a.n, probe)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
