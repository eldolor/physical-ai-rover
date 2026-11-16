# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.178.E: where to put the list items so the robot passes them -- chosen
from the routes it ACTUALLY drives, on the tuning seeds only (3.178.B's
3178/3179 and 3.178.D's 4178/4179), never on E's fresh seeds.

    python evaluations/watch-list-3178/placement.py      # -> placements.json

(On a shared desktop, under `tools/desktop_lock.py shared`: it uses every core.)

Method (fixed in the section file before E's fresh run):

1. Every tuning start (20 per seed, `inventory_sweep.starts`) runs the
   frontier policy through `MissionRunner` -> safety -> `MockRobot` in the
   UNCHANGED `complex_house`, list off, for `ROUTE_STEPS`; each frame's
   robot cell is recorded.
2. A candidate cell has the 5 x 5 cells round it free floor with no object
   (`CLEAR_CELLS`): never in a doorway, the hall's corridor, or against a
   wall or furniture, so a placed item never closes a way through. (A
   legged table claims only its legs; no placed item lies under one, checked
   by hand against `FURNITURE_FT`.)
3. One item per room (starts rotate through the rooms, and a frontier
   mission often stays near its start: on the tuning seeds a 400-step route
   covered a median of ~40 cells): the candidate the most tuning routes came
   within `NEAR_CELLS` (1.8 m) of. The list items are assigned in turn
   (`DEFAULT_WATCH_LIST` order), rooms in name order.

Placing solid things changes routes; that is why E's passes are counted on
the run itself (B1's rule), never predicted from this score.
"""
from __future__ import annotations

import json
import math
import random
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from brain.watch_list import DEFAULT_WATCH_LIST  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.maps import complex_house as house  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from tests import inventory_sweep as base  # noqa: E402
from tests.conftest import mock_world_for  # noqa: E402

TUNING_SEEDS = (3178, 3179, 4178, 4179)    # B's and D's; never E's
ROUTE_STEPS = 400
NEAR_CELLS = 6            # 1.8 m: a route this close leaves room inside B1's 2.5 m
CLEAR_CELLS = 2           # the 5 x 5 cells round a placed item are free floor
OUT = Path(__file__).with_name("placements.json")


def route(args) -> list:
    start, seed, target = args
    (cx, cy), theta, _room = start
    grid = build_world("complex_house")
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    robot = MockRobot(grid, render=False)
    det = base.ErringDetector(grid, random.Random(seed))
    cells = []

    def detections(frame):
        cells.append((int(math.floor(grid.x)), int(math.floor(grid.y))))
        return det(frame)

    runner = MissionRunner(robot, target_object=target, max_steps=ROUTE_STEPS,
                           policy="frontier", world=mock_world_for(robot),
                           detections_fn=detections, watch_list=())
    runner.start()
    while runner.tick():
        pass
    return sorted(set(cells))


def _floor(c) -> bool:
    i, j = c
    return 0 <= j < len(house.LAYOUT) and 0 <= i < len(house.LAYOUT[j]) and house.LAYOUT[j][i] == "."


def candidate(c) -> bool:
    """Free floor with every cell within `CLEAR_CELLS` (Chebyshev) free floor
    and no object: never in a doorway, the hall's corridor or against a wall
    or a piece of furniture, so a placed item never closes a way through."""
    i, j = c
    return all(_floor((i + dx, j + dy)) and (i + dx, j + dy) not in house.OBJECTS
               for dx in range(-CLEAR_CELLS, CLEAR_CELLS + 1)
               for dy in range(-CLEAR_CELLS, CLEAR_CELLS + 1))


def choose(routes: list) -> list:
    """One item per room: the candidate cell the most tuning routes came
    within `NEAR_CELLS` of (Euclidean, cell centres), fewer routes standing
    on it breaking a tie, then the cell's order. Items in turn, rooms in name
    order. A room with no candidate holds none."""
    routes = [set(map(tuple, r)) for r in routes]
    placed = []
    for room, cells in sorted(house.ROOMS.items()):
        best = None
        for c in sorted(cells):
            if not candidate(c):
                continue
            near = sum(1 for r in routes
                       if any((c[0] - a) ** 2 + (c[1] - b) ** 2 <= NEAR_CELLS ** 2 for a, b in r))
            key = (near, -sum(1 for r in routes if c in r))
            if best is None or key > best[0]:
                best = (key, c)
        if best is not None:
            placed.append((room, best[0][0], best[1]))
    items = list(DEFAULT_WATCH_LIST)
    return [{"item": items[k % len(items)], "cell": list(c), "room": room, "routes_near": n}
            for k, (room, n, c) in enumerate(placed)]


def main():
    jobs = []
    for seed in TUNING_SEEDS:
        target = base.ABSENT_TARGET if seed % 2 == 0 else "red backpack"
        for k, start in enumerate(base.starts(n=20, seed=seed)):
            jobs.append((start, seed * 100 + k, target))
    with ProcessPoolExecutor() as ex:
        routes = list(ex.map(route, jobs))
    placed = choose(routes)
    OUT.write_text(json.dumps({"tuning_seeds": list(TUNING_SEEDS), "route_steps": ROUTE_STEPS,
                               "missions": len(routes), "near_cells": NEAR_CELLS,
                               "clear_cells": CLEAR_CELLS,
                               "placed": placed}, indent=1) + "\n")
    for p in placed:
        print(p, file=sys.stderr)


if __name__ == "__main__":
    main()
