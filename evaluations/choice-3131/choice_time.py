"""
evaluations/choice-3131/choice_time.py -- 3.131 criterion 2: F-large's whole
goal choice as explore now makes it, timed on 3.96's benchmark (the six live
0.05 m maps, 20 poses each, `fc.SEED`), and split into parts at the same
poses.

The same as `evaluations/goal-learn-3103/jetson_time.py`'s F-large arm and
`parts()`, with one difference, the one 3.131 made in `brain/explore.py`:
the choice's `goal_context()` is handed to the room features
(`room_features(..., ctx=ctx)`), so the map's cell list is converted once a
choice. The camera-seen set is empty, as 3.96/3.103/3.125 timed it.

    python evaluations/choice-3131/choice_time.py <label> <out.json>
"""
import json
import os
import platform
import random
import statistics
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "evaluations", "goal-learn-396"))
import time_choice as T396                                    # noqa: E402
from model import score_np, to_np                             # noqa: E402
from brain import frontier as fr                              # noqa: E402
from brain.goal_policy import features, goal_context, room_features  # noqa: E402
from brain.room_labels import segment                         # noqa: E402

fc = T396.fc


def choice(m, x, y, heading, trail, P, ms=None) -> int:
    """F-large's choice (`T396.choice` with explore's ctx hand-off); with
    `ms`, each part's time is written into it."""
    t = time.perf_counter()
    pose = {"x_m": x, "y_m": y, "heading_deg": heading}
    r = fr.reach(m, x, y, clearance_m=fc.LEAVE)
    fs = fr.find_frontiers(m, x, y, clearance_m=fc.LEAVE, reach_map=r)
    gs = fr.find_view_gaps(m, x, y, set(), clearance_m=fc.LEAVE, reach_map=r)
    if ms is not None:
        ms["map_work"], t = (time.perf_counter() - t) * 1000, time.perf_counter()
    ctx = goal_context(m)
    feats = [features(m, pose, f.goal, kind="frontier", path_m=f.distance_m, size_m=f.size_m,
                      ctx=ctx) for f in fs]
    feats += [features(m, pose, g.goal, kind="view", path_m=g.distance_m, area_m2=g.area_m2,
                       ctx=ctx) for g in gs]
    if ms is not None:
        ms["ten_facts"], t = (time.perf_counter() - t) * 1000, time.perf_counter()
    goals = [f.goal for f in fs] + [g.goal for g in gs]
    if not goals:
        return 0
    room = room_features(m, pose, goals, trail=trail, types=[None] * len(goals), prior={},
                         ctx=ctx)
    if ms is not None:
        ms["room_facts"], t = (time.perf_counter() - t) * 1000, time.perf_counter()
    X = np.asarray([a + b for a, b in zip(feats, room)], dtype=np.float32)
    int(np.argmax(score_np(P, "F", X)))
    if ms is not None:
        ms["network"] = (time.perf_counter() - t) * 1000
    return len(goals)


def stats(xs) -> dict:
    return {"p50_ms": round(statistics.median(xs), 2), "p90_ms": round(T396.p90(xs), 2),
            "max_ms": round(max(xs), 2)}


def main(label: str, out_path: str) -> None:
    P = to_np(json.load(open(T396.BEST))["params"])
    whole, split, counts = [], {}, []
    t0, rng = time.time(), random.Random(fc.SEED)
    for path in fc.MAPS:
        m = fc.load(path)
        segment(T396.coarse(m))                 # warm once, as a running brain would be
        for x, y in fc.poses(m, T396.POSES, rng):
            trail, heading = T396.trail_near(m, x, y), rng.uniform(0, 360)
            t = time.perf_counter()
            k = choice(m, x, y, heading, trail, P)
            if k:
                whole.append((time.perf_counter() - t) * 1000)
                counts.append(k)
            ms = {}
            if choice(m, x, y, heading, trail, P, ms):
                for part, v in ms.items():
                    split.setdefault(part, []).append(v)
        print(f"{time.strftime('%T')} {os.path.basename(path)} done", flush=True)
    out = {"label": label, "host": platform.node(), "machine": platform.machine(),
           "start_unix": round(t0), "end_unix": round(time.time()),
           "loadavg": os.getloadavg(), "poses": len(whole),
           "candidates_median": statistics.median(counts),
           "whole_choice": {"F-large": {"choices": len(whole), **stats(whole)}},
           "F-large_parts": {p: stats(v) for p, v in split.items()}}
    with open(out_path, "w") as f:
        f.write(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main(*sys.argv[1:3])
