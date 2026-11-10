"""3.173.A2/A3: truth reach per clearance, per house and room.

    python evaluations/small-rooms-3173/a2_reach.py OUT.jsonl

For each house: a 5 cm raster of the truth's walls and things, and its distance
transform (a pixel's centre to the nearest blocked pixel's edge, so within half a
pixel, 2.5 cm, of the true distance). Explore's two clearances (`brain/frontier.py`):
a path passes where a centre keeps `PASS_CLEARANCE_M` (0.15 m); a frontier or view
goal stands only where it keeps `leave_clearance_m` (0.3265 m). Per room:
* `goal_room_m`: the most clearance any of its floor keeps that the dock reaches
  through passes -- the best place a goal could stand in it. A room with less than
  a goal clearance has nowhere a goal at that clearance may stand: A2's
  `no_goal_at` lists them per clearance.
* `path_m`: its bottleneck -- the largest clearance a whole path from the dock keeps
  into it (A3).

A3: the chassis' circumscribed radius is the corner distance of
`tests/footprint_sweep.py`'s rectangle; a centre that keeps radius + G_BAR from
truth sweeps no contact at any heading, so the smallest safe clearance at every
doorway is that number wherever the path keeps it. Reported alongside.

Holds the desktop lock shared and steps aside between houses (3.128).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
import desktop_lock  # noqa: E402
from sim.grid_world import CELL_WALL  # noqa: E402
from sim.maps import build_world  # noqa: E402
from brain.frontier import PASS_CLEARANCE_M as PASS_M  # noqa: E402
from tests.footprint_sweep import CELL_CM, G_BAR_CM, HALF_LENGTH, HALF_WIDTH  # noqa: E402

CELL = CELL_CM / 100.0
SUB = 0.05
K = int(round(CELL / SUB))                      # pixels a cell
A2 = (0.20, 0.25, 0.30, 0.3265)                 # A2's clearances (3.43's goal clearance last)
CLEARANCES = [round(0.15 + 0.01 * i, 2) for i in range(21)] + [0.3265]
HOUSES = (["home_first_floor_stairs", "scaled_house", "complex_house"]
          + [f"procthor:val:{i}" for i in range(20)])
RADIUS_M = math.hypot(HALF_LENGTH, HALF_WIDTH) * CELL
SAFE_M = RADIUS_M + G_BAR_CM / 100.0


def house(name):
    w = build_world(name)
    blocked = np.zeros((w.height * K, w.width * K), dtype=bool)
    for y in range(w.height):
        for x in range(w.width):
            if w._cell(x, y) == CELL_WALL or (x, y) in w.objects:
                blocked[y * K:(y + 1) * K, x * K:(x + 1) * K] = True
    # Pixel centre to the nearest blocked pixel's centre, less half a pixel: its edge.
    dist = ndimage.distance_transform_edt(~blocked) * SUB - SUB / 2
    start = (int((w.robot_y + .5) * K), int((w.robot_x + .5) * K))
    room_px = {}
    for room, cells in w.rooms.items():
        m = np.zeros_like(blocked)
        for cx, cy in cells:
            m[cy * K:(cy + 1) * K, cx * K:(cx + 1) * K] = True
        room_px[room] = m
    lab, _ = ndimage.label(dist >= PASS_M)
    passable = lab == lab[start] if dist[start] >= PASS_M else np.zeros_like(blocked)
    goal_room = {r: round(float(dist[m & passable].max()), 3) if (m & passable).any() else None
                 for r, m in room_px.items()}
    reached = {}
    for c in sorted(CLEARANCES):
        free = dist >= c
        if not free[start]:
            reached[c] = set()
            continue
        lab, _ = ndimage.label(free)
        comp = lab == lab[start]
        reached[c] = {r for r, m in room_px.items() if (comp & m).any()}
    bottleneck = {r: max((c for c in CLEARANCES if r in reached[c]), default=None) for r in w.rooms}
    return {"house": name, "rooms": len(w.rooms), "start_clear_m": round(float(dist[start]), 3),
            "goal_room_m": goal_room, "path_m": bottleneck,
            "no_goal_at": {str(c): sorted(r for r, v in goal_room.items() if v is None or v < c)
                           for c in A2}}


def main(out):
    desktop_lock.hold_for_life("shared", f"3.173.A2 truth reach ({__file__})")
    print(f"chassis circumscribed radius {RADIUS_M:.3f} m; no contact at any heading from "
          f"{SAFE_M:.3f} m", flush=True)
    with open(out, "w") as fh:
        for name in HOUSES:
            desktop_lock.step_aside()
            row = house(name)
            row["safe_any_heading_m"] = round(SAFE_M, 3)
            fh.write(json.dumps(row) + "\n")
            fh.flush()
            print(name, row["rooms"], {k: v for k, v in row["no_goal_at"].items() if v}, flush=True)


if __name__ == "__main__":
    main(sys.argv[1])
