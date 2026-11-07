# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.165 criteria 1-3: the inventory over a long mission, at G2's scale
(docs/plans/ros-alignment/3.165-inventory-landmarks-bounded.md).

4,800 frames x 10 detections straight into `Inventory.observe()`: 40
objects round a room, the robot circling slowly (radius 0.5 m, its camera
turning 1 deg a frame), stopped for 400 frames every 1,200, 3 deg bearing
noise, and a 360-beam scan built from the objects. Deterministic.

    .venv/bin/python evaluations/inventory-3165/long_mission.py timing  <out.json>
    .venv/bin/python evaluations/inventory-3165/long_mission.py memory  <out.json>
    .venv/bin/python evaluations/inventory-3165/long_mission.py check   <out.json>

`timing` measures each `observe()`; `memory` the inventory's live
allocations (tracemalloc, traces from brain/inventory.py) at the
checkpoints; `check` follows every placed point from outside the
inventory (wrapping `_count_look` and `_absorb`, as running sums by landmark id) and asserts, after every
frame, each landmark's position against the mean of its points, and
records the most members any landmark held.
"""
import collections
import json
import math
import random
import statistics
import sys
import time
import tracemalloc
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from brain import inventory as inv                     # noqa: E402
from robot.safety import LIDAR_X_M                     # noqa: E402

FRAMES = 4800
PER_FRAME = 10
OBJECTS = 40
LABELS = ("chair", "table", "sofa", "lamp", "plant", "shoe", "box", "bin")
FOV_DEG = 120.0
NOISE_DEG = 3.0
CHECKPOINTS = (1200, 2400, 3600, 4800)
STOPS = ((1200, 1600), (2400, 2800), (3600, 4000))


def world(seed=3165):
    rng = random.Random(seed)
    objs = []
    while len(objs) < OBJECTS:
        r, a = rng.uniform(1.4, 3.6), rng.uniform(0, 2 * math.pi)
        x, y = r * math.cos(a), r * math.sin(a)
        if all(math.hypot(x - ox, y - oy) > 0.7 for ox, oy, _ in objs):
            objs.append((x, y, LABELS[len(objs) % len(LABELS)]))
    return objs


def poses():
    """The robot's pose each frame: circling, with its stops."""
    s = 0
    for f in range(FRAMES):
        if not any(a <= f < b for a, b in STOPS):
            s += 1
        a = s * 0.02                       # 1 cm a frame on a 0.5 m circle
        yield {"usable": True, "x_m": 0.5 * math.cos(a), "y_m": 0.5 * math.sin(a),
               "heading_deg": (s * 1.0) % 360.0, "map_id": "bench"}


def body_view(pose, x, y):
    dx, dy = x - pose["x_m"], y - pose["y_m"]
    compass = math.degrees(math.atan2(dx, -dy))
    return inv._wrap(compass - pose["heading_deg"]), math.hypot(dx, dy)


def scan_for(pose, objs):
    """A 360-beam scan from the lidar (4 cm ahead), as tests/test_inventory's
    `_scan` builds one: each object a 0.15 m disc, walls at 4.5 m."""
    ranges = [4.5] * 360
    for x, y, _ in objs:
        b, r = body_view(pose, x, y)
        fwd, right = r * math.cos(math.radians(b)) - LIDAR_X_M, r * math.sin(math.radians(b))
        lr, la = math.hypot(fwd, right), math.degrees(math.atan2(right, fwd))
        half = math.degrees(math.atan2(0.15, lr))
        for a in range(math.floor(la - half), math.ceil(la + half) + 1):
            i = (a + 180) % 360
            ranges[i] = min(ranges[i], lr)
    return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
            "range_min_m": 0.0, "range_max_m": 12.0, "ranges_m": ranges}


def detections_for(pose, objs, rng):
    seen = []
    for x, y, label in objs:
        b, r = body_view(pose, x, y)
        if abs(b) <= FOV_DEG / 2 and r <= inv.MAX_RANGE_M:
            seen.append((r, label, b))
    seen.sort()
    return [{"label": label, "bearing_deg": b + rng.gauss(0, NOISE_DEG)}
            for _, label, b in seen[:PER_FRAME]]


def frames():
    objs, rng = world(), random.Random(1)
    for f, pose in enumerate(poses(), 1):
        yield f, pose, detections_for(pose, objs, rng), scan_for(pose, objs)


def members(lm):
    return len(lm.points)


def run_timing():
    i, ms = inv.Inventory(), []
    for f, pose, dets, scan in frames():
        t0 = time.perf_counter()
        i.observe(dets, pose, scan, fov_deg=FOV_DEG, step=f)
        ms.append((time.perf_counter() - t0) * 1000)
    early, late = statistics.median(ms[100:200]), statistics.median(ms[4700:4800])
    return {"median_ms_101_200": round(early, 3), "median_ms_4701_4800": round(late, 3),
            "ratio": round(late / early, 3),
            "median_ms_at": {str(c): round(statistics.median(ms[c - 100:c]), 3)
                             for c in (100, *CHECKPOINTS)},
            "landmarks": len(i.landmarks), "observations": i.observations}


def run_memory():
    tracemalloc.start()
    i, out = inv.Inventory(), {}
    # The Counters the inventory keeps (the build's cached votes) are
    # allocated inside collections, so its traces count too (review).
    flt = [tracemalloc.Filter(True, str(ROOT / "brain" / "inventory.py")),
           tracemalloc.Filter(True, collections.__file__)]
    for f, pose, dets, scan in frames():
        i.observe(dets, pose, scan, fov_deg=FOV_DEG, step=f)
        if f in CHECKPOINTS:
            snap = tracemalloc.take_snapshot().filter_traces(flt)
            out[str(f)] = sum(s.size for s in snap.statistics("filename"))
    tracemalloc.stop()
    return {"bytes_at": out, "landmarks": len(i.landmarks)}


def run_check():
    shadow = {}
    real_look, real_absorb = inv.Inventory._count_look, inv.Inventory._absorb

    def look(self, lm, label, x, y, pose, joined):
        acc = shadow.setdefault(lm.id, [0.0, 0.0, 0])
        acc[0] += x
        acc[1] += y
        acc[2] += 1
        return real_look(self, lm, label, x, y, pose, joined)

    def absorb(self, keep, others):
        for o in others:
            acc, gone = shadow.setdefault(keep.id, [0.0, 0.0, 0]), shadow.pop(o.id)
            for k in range(3):
                acc[k] += gone[k]
        return real_absorb(self, keep, others)

    inv.Inventory._count_look, inv.Inventory._absorb = look, absorb
    try:
        i, most, worst = inv.Inventory(), 0, 0.0
        for f, pose, dets, scan in frames():
            i.observe(dets, pose, scan, fov_deg=FOV_DEG, step=f)
            for lm in i.landmarks:
                sx, sy, n = shadow[lm.id]
                worst = max(worst, abs(lm.x - sx / n), abs(lm.y - sy / n))
                most = max(most, members(lm))
    finally:
        inv.Inventory._count_look, inv.Inventory._absorb = real_look, real_absorb
    return {"most_members": most, "max_mean_error_m": worst,
            "landmarks": len(i.landmarks), "observations_placed": sum(a[2] for a in shadow.values())}


if __name__ == "__main__":
    mode, out = sys.argv[1], sys.argv[2]
    res = {"timing": run_timing, "memory": run_memory, "check": run_check}[mode]()
    res["mode"] = mode
    Path(out).write_text(json.dumps(res, indent=1) + "\n")
    print(json.dumps(res))
