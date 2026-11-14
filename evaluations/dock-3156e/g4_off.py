# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/dock-3156e/g4_off.py -- 3.156.E bar G4's in-process half
(`docs/plans/ros-alignment/3.156-return-to-dock.md`, section E).

    .venv/bin/python evaluations/dock-3156e/g4_off.py <repo-root> <out.json>

With `return_to_dock` off, a mission that does not explore must build no
navigator and end exactly as `dev`'s: the same outcome, steps and metres.
Run once against this branch's tree and once against a `dev` worktree (the
first argument puts that tree first on the path), then compare the two
files row by row (`compare` below).

`MissionRunner` -> the frontier policy (rule-based: no model, deterministic)
-> `robot/safety.py` -> `MockRobot`, a navigator passed (as the brain
server would pass one with the return on) to show it is ignored when off.
Starts: `scaled_house` and `home_first_floor` x 10 spread starts each
(`tests/dock_sweep.py`'s `spread_starts`, seeded): 20 missions.
"""

import json
import math
import sys
from pathlib import Path

HOUSES = ("scaled_house", "home_first_floor")
STARTS = 10
MAX_STEPS = 300
LIMIT_S = 1200.0
CELL = 0.30


def starts():
    from tests.dock_sweep import spread_starts
    return [(house, start) for house in HOUSES for start in spread_starts(house, STARTS)]


def run(root: str):
    sys.path.insert(0, root)
    import logging
    logging.disable(logging.WARNING)
    return [one(house, start) for house, start in starts()]


def one(house, start):
    """One mission; the row the comparison reads."""
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld
    from tests.test_explore import FakeNav

    grid = build_world(house)
    grid.x, grid.y = start[0], start[1]
    grid.theta = math.radians(start[2] - 90)
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    sent = []
    real_set = nav.set_goal
    nav.set_goal = lambda x, y: sent.append((x, y)) or real_set(x, y)
    runner = MissionRunner(robot, policy="frontier", navigator=nav, world=MockWorld(grid),
                           target_object="red backpack", max_steps=MAX_STEPS,
                           mission_time_limit_s=LIMIT_S, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=False)
    metres = [0.0]
    last = [(grid.x, grid.y)]
    step = robot.step

    def counted(dt, step=step, grid=grid, metres=metres, last=last):
        r = step(dt)
        metres[0] += math.hypot(grid.x - last[0][0], grid.y - last[0][1]) * CELL
        last[0] = (grid.x, grid.y)
        return r
    robot.step = counted
    runner.start()
    while runner.tick():
        pass
    s = runner.status()
    return {"house": house, "start": list(start), "outcome": s["outcome"],
            "end_reason": s.get("end_reason"), "steps": s["step"],
            "metres": round(metres[0], 4),
            "end": [round(grid.x, 4), round(grid.y, 4), round(grid.heading_deg, 2)],
            "navigator": getattr(runner, "_navigator", None) is not None,
            "goals_sent": len(sent), "return": s.get("return")}


def compare(a, b):
    keys = ("house", "start", "outcome", "end_reason", "steps", "metres", "end", "return")
    return [i for i, (x, y) in enumerate(zip(a, b)) if any(x[k] != y[k] for k in keys)]


if __name__ == "__main__":
    if sys.argv[1] == "compare":
        a, b = (json.loads(Path(p).read_text()) for p in sys.argv[2:4])
        diff = compare(a, b)
        print(json.dumps({"missions": len(a), "differ": len(diff), "rows": diff,
                          "navigators": sum(r["navigator"] for r in a),
                          "goals_sent": sum(r["goals_sent"] for r in a),
                          "outcomes": {o: sum(r["outcome"] == o for r in a)
                                       for o in sorted({r["outcome"] for r in a})}}))
    else:
        out = run(sys.argv[1])
        Path(sys.argv[2]).write_text(json.dumps(out, indent=1) + "\n")
        print(len(out), "missions ->", sys.argv[2])
