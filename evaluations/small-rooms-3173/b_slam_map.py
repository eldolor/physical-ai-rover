# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.173.B's first task: each room's goal room on SLAM's saved map.

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    ROS_IMAGE=vision-picar-ros:plan-3.156 python evaluations/small-rooms-3173/b_slam_map.py VOLUME OUT.json

A fresh stack (`demo_explore.stack()`, which takes the exclusive desktop lock) on
`home_first_floor_stairs` with the saved map volume mounted, as 3.172.E's
rehearsal ran it: the bridge localises on the saved map from the dock. Once
`/world/map` is usable and held unchanged (id, version, cells) for SETTLE_S, the map is read once,
the stack stopped, and per truth room (a map cell's centre in the room's 0.30 m
truth cells) it reports:

* `stops`: reachable stops from the dock (`brain.frontier.reach`, its PASS
  clearance) at each clearance: 0.20, `SMALL_ROOM_CLEARANCE_M` (0.25), 0.30 and
  the goal clearance `leave_clearance_m` (0.3265, 7 cells, 0.35 m on 5 cm cells);
* `best_m`: the most clearance any of its free cells keeps from an occupied cell
  (centre to centre; not limited to reachable cells).

The map is a house's floor plan: it is written only to OUT (keep it local; never
commit or upload it). OUT holds the numbers, not the map.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy import ndimage

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from brain.explore import SMALL_ROOM_CLEARANCE_M, leave_clearance_m  # noqa: E402
from brain.frontier import reach  # noqa: E402
from brain.room_labels import segment  # noqa: E402
from sim.maps import build_world  # noqa: E402
from tests import demo_explore as de  # noqa: E402
from world.interface import CELL_FREE, CELL_OCCUPIED  # noqa: E402

HOUSE = "home_first_floor_stairs"
SETTLE_S = 20.0
TRUTH_CELL = 0.30
MIN_DISTANCE_CM = 20.0          # config/robot.yaml's safety.min_distance_cm


def main():
    volume, out = sys.argv[1], Path(sys.argv[2])
    run = de.subprocess.run

    def with_maps(cmd, **kw):
        if isinstance(cmd, list) and cmd[:2] == ["docker", "run"]:
            cmd = cmd[:2] + ["-v", f"{volume}:/maps"] + cmd[2:]
        return run(cmd, **kw)
    de.subprocess.run = with_maps
    settled = False
    try:
        robot = de.stack(HOUSE)
        last, since, t0 = None, time.time(), time.time()
        while time.time() - t0 < 300:
            m = robot.get("/world/map").json()
            key = (m.get("map_id"), m.get("map_version"), m.get("width"), m.get("height"),
                   hash(tuple(m.get("cells") or ())))
            if key != last:
                last, since = key, time.time()
            elif m.get("usable") and time.time() - since >= SETTLE_S:
                settled = True
                break
            time.sleep(2)
        pose = robot.get("/world/pose").json()
    finally:
        de._kill_ports()
        de.subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
    out.with_suffix(".map.json").write_text(json.dumps(m))      # local only: a floor plan
    if not settled:
        raise SystemExit("the map did not hold for SETTLE_S within 300 s; not measured")
    measure(m, pose, out)


def measure(m, pose, out):
    w, h, res = m["width"], m["height"], m["resolution_m"]
    x0, y0 = m["origin_x_m"], m["origin_y_m"]
    cells = np.array(m["cells"]).reshape(h, w)
    occ = cells == CELL_OCCUPIED
    best = ndimage.distance_transform_edt(~occ) * res
    truth = build_world(HOUSE)
    room = {c: n for n, cs in sorted(truth.rooms.items(), key=lambda kv: -len(kv[1])) for c in cs}
    jj, ii = np.mgrid[0:h, 0:w]
    xs, ys = x0 + (ii + 0.5) * res, y0 + (jj + 0.5) * res
    names = np.full((h, w), None, dtype=object)
    for (cx, cy), n in room.items():
        sel = (xs >= cx * TRUTH_CELL) & (xs < (cx + 1) * TRUTH_CELL) & (ys >= cy * TRUTH_CELL) & (ys < (cy + 1) * TRUTH_CELL)
        names[sel] = n
    leave = leave_clearance_m(MIN_DISTANCE_CM)
    clear = [0.20, SMALL_ROOM_CLEARANCE_M, 0.30, leave]
    rows = {n: {"stops": {}, "best_m": None, "free_cells": 0} for n in sorted(truth.rooms)}
    flat = names.ravel()
    for c in clear:
        r = reach(m, pose["x_m"], pose["y_m"], clearance_m=c)
        for n in rows:
            rows[n]["stops"][f"{c:.4g}"] = 0
        for cell in r.order.tolist():
            n = flat[cell]
            if n is not None:
                rows[n]["stops"][f"{c:.4g}"] += 1
    free = cells == CELL_FREE
    for n in rows:
        sel = free & (names == n)
        rows[n]["free_cells"] = int(sel.sum())
        rows[n]["best_m"] = round(float(best[sel].max()), 3) if sel.any() else None
    seg, _ = segment(m)
    res_out = {"map": {"map_id": m.get("map_id"), "map_version": m.get("map_version"),
                       "width": w, "height": h, "resolution_m": res},
               "dock_pose": {k: pose.get(k) for k in ("x_m", "y_m", "heading_deg")},
               "leave_clearance_m": leave, "rooms": rows,
               "cut_pieces": {n: sorted({int(v) for v in seg[(names == n) & (seg >= 0)].tolist()})
                              for n in rows}}
    out.write_text(json.dumps(res_out, indent=1) + "\n")
    print(json.dumps({n: (r["best_m"], r["stops"]) for n, r in rows.items()}, indent=1))


if __name__ == "__main__":
    main()
