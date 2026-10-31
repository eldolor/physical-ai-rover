"""
evaluations/fast-map-3130/time_map.py -- 3.130 criterion 3: explore's map
work (`reach` + `find_frontiers` + `find_view_gaps`, as 3.103's
`jetson_time.py` `parts()` calls them) by `brain/frontier.py` and by
`tests/frontier_dev.py` (`dev`'s copy at this section's fork), both at every
pose, in alternating order, so a change in load reaches both alike.

Poses are 3.103's (the six live maps, 20 each, `random.Random(fc.SEED)`
consumed as `jetson_time.py` consumes it). Two arms: `empty` (`seen=set()`,
as 3.103 and 3.109 recorded it) and `seen` (the camera's seen cells from 30
poses per map, drawn from `random.Random(3130)`, as explore passes them).
One untimed pass first; the two answers are checked equal at every pose.

    python evaluations/fast-map-3130/time_map.py <label> <out.json>
"""
import json
import os
import platform
import random
import statistics
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
from brain import frontier as new                             # noqa: E402
from tests import frontier_compare as fc                      # noqa: E402
from tests import frontier_dev as dev                         # noqa: E402

POSES = 20
SEEN_POSES = 30


def p90(xs):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(0.9 * len(xs)))]


def map_work(fr, m, x, y, seen):
    r = fr.reach(m, x, y, clearance_m=fc.LEAVE)
    fs = fr.find_frontiers(m, x, y, clearance_m=fc.LEAVE, reach_map=r)
    gs = fr.find_view_gaps(m, x, y, seen, clearance_m=fc.LEAVE, reach_map=r)
    return [vars(f) for f in fs], [vars(g) for g in gs]


def jobs():
    """(map name, m, x, y, seen) for both arms, 3.103's poses."""
    rng, seen_rng = random.Random(fc.SEED), random.Random(3130)
    out = []
    for path in fc.MAPS:
        m = fc.load(path)
        seen = set()
        for x, y in fc.poses(m, SEEN_POSES, seen_rng):
            seen |= new.camera_seen(m, x, y, seen_rng.uniform(0, 360))
        for x, y in fc.poses(m, POSES, rng):
            rng.uniform(0, 360)             # jetson_time.py's heading draw
            out.append((path.name.split(".")[0], m, x, y, seen))
    return out


def main(label: str, out_path: str) -> None:
    js = jobs()
    for _, m, x, y, seen in js:                     # warm, and the answers agree
        for s in (set(), seen):
            if map_work(new, m, x, y, s) != map_work(dev, m, x, y, s):
                raise SystemExit(f"answers differ at {x:.3f}, {y:.3f} ({len(s)} seen)")
    t0 = time.time()
    times = {(arm, code): [] for arm in ("empty", "seen") for code in ("dev", "new")}
    for i, (_, m, x, y, seen) in enumerate(js):
        for arm, s in (("empty", set()), ("seen", seen)):
            order = (("dev", dev), ("new", new)) if i % 2 else (("new", new), ("dev", dev))
            for code, fr in order:
                t = time.perf_counter()
                map_work(fr, m, x, y, s)
                times[arm, code].append((time.perf_counter() - t) * 1000)
    out = {"label": label, "host": platform.node(), "machine": platform.machine(),
           "start_unix": round(t0), "end_unix": round(time.time()),
           "loadavg": os.getloadavg(), "poses": len(js),
           "seen_cells": sorted({len(j[4]) for j in js})}
    for arm in ("empty", "seen"):
        row = {}
        for code in ("dev", "new"):
            xs = times[arm, code]
            row[code] = {"p50_ms": round(statistics.median(xs), 2), "p90_ms": round(p90(xs), 2),
                         "max_ms": round(max(xs), 2)}
        row["p90_ratio"] = round(row["new"]["p90_ms"] / row["dev"]["p90_ms"], 3)
        out[arm] = row
    with open(out_path, "w") as f:
        f.write(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main(*sys.argv[1:3])
