# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/tight_home_sweep.py -- 3.228.B's in-process harness
(`docs/plans/ros-alignment/3.228-trip-home-fails.md`, section B, B1-B3).

    python tests/tight_home_sweep.py <before|after> <out.json> [seeds] [workers]

The trip home from a tight spot, through the real path: `MissionRunner` ->
explore -> the brain's `robot/safety.py` -> a robot-server stand-in -> the
body. The stand-in is a second `SafetyController` on the body, as the robot
server's: nav's twists are vetted by it at the wheel loop's 20 Hz (its
retrace track sampled there, 3.224), and the brain's moves reach the body
only through its `check_and_execute`, as `RemoteRobot`'s reach the server.
The body (`MockRobot`) reports the Rover's lidar self box, so both
controllers refuse an unseen rear as `astern_occluded`.

Per seed, on `scaled_house`: a nook of three objects (a U one 0.30 m cell
wide) on free floor at least 4 cells from the dock; the mission starts at
the dock, the robot is stood a cell before the nook's mouth facing in and
driven in by FORWARDs through the server's vet (to its 20 cm bar: a ~0.40 m
track, as nav2 driving in from open floor leaves), and
the mission's time limit (1 s, already spent) sends it home. nav2 is
`tests/dock_sweep.py`'s InflatedNav whose planner fails while the robot
cannot spin at its TRUE pose (a truth obstacle within the chassis' corner
radius): an assumption from 3.229.B's record (every plan failed from a
start where 5 of 6 spins exited `Collision Ahead`).

Arms: `before` (dev's behaviour: `brain.dock.ESCAPE_BACKS = 0` and the body
not saying the server re-vets, so the brain refuses `astern_occluded`
itself) and `after` (both on).
"""

import json
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

HOUSE = "scaled_house"
CELL = 0.30
ROVER_BOX = "-7.1,3.2,-10.7,8.5"
TICK_CAP = 3000
SEEDS = 40


def _escape_default():
    import brain.dock
    return brain.dock.ESCAPE_BACKS


ESCAPE_BACKS_DEFAULT = _escape_default()     # the shipped value, before any arm sets it


def corner_radius_cells():
    from robot.safety import FOOTPRINT_LENGTH_M, FOOTPRINT_WIDTH_M
    return math.hypot(FOOTPRINT_LENGTH_M / 2, FOOTPRINT_WIDTH_M / 2) / CELL


def can_spin(grid) -> bool:
    """No wall or object cell within the corner radius of the TRUE centre."""
    from sim.grid_world import CELL_WALL
    r = corner_radius_cells()
    cx, cy = grid.x, grid.y
    for i in range(math.floor(cx - r) - 1, math.floor(cx + r) + 2):
        for j in range(math.floor(cy - r) - 1, math.floor(cy + r) + 2):
            if grid._cell(i, j) != CELL_WALL and (i, j) not in grid.objects:
                continue
            dx = max(i - cx, 0.0, cx - (i + 1))
            dy = max(j - cy, 0.0, cy - (j + 1))
            if math.hypot(dx, dy) < r:
                return False
    return True


def nav_class():
    from tests.dock_sweep import inflated_nav_class
    base = inflated_nav_class()

    class NavfnStandIn(base):
        """InflatedNav whose planner refuses a start the robot cannot spin
        at (nav2's behaviour tree: its planning retries and recoveries, then
        an abort; 10 s here)."""
        plan_failures = 0

        def get_goal(self):
            if self.goal and self.goal["state"] == "active" and not can_spin(self.grid):
                self.plan_failures += 1
                self.robot.set_wheel_velocity(0.0, 0.0)
                self.robot.pass_time(10.0)
                self.goal["state"] = "aborted"
                return {"goal": self.goal, "plan": []}
            return super().get_goal()

    return NavfnStandIn


def rover_mock_class():
    from sim.mock_robot import MockRobot

    class RoverMock(MockRobot):
        def lidar_self_box_cm(self):
            return ROVER_BOX

    return RoverMock


class ServerBody:
    """The brain's view of the body through the robot-server stand-in: moves
    go through the server's `check_and_execute` (its SafetyViolation reaches
    the brain, as `RemoteRobot` re-raises the server's); everything else is
    the body's. No `verb_plan`: the brain runs no guarded verb itself, as
    over HTTP."""

    def __init__(self, robot, server, vets: bool):
        self._robot, self._server, self._vets = robot, server, vets
        self.moves = 0

    def __getattr__(self, name):
        return getattr(self._robot, name)

    def _do(self, action, **kw):
        self.moves += 1
        self._server._retrace_sample()
        try:
            return self._server.check_and_execute(action, **kw)
        finally:
            self._server._retrace_sample()

    def drive_forward(self, speed=50, duration=0.5):
        return self._do("FORWARD", speed=speed, duration=duration)

    def reverse(self, speed=50, duration=0.5):
        return self._do("REVERSE", speed=speed, duration=duration)

    def turn_left(self, angle=90):
        return self._do("LEFT", angle=angle)

    def turn_right(self, angle=90):
        return self._do("RIGHT", angle=angle)

    def verb_plan(self, *a, **k):
        return None

    def lidar_self_box_cm(self):
        return ROVER_BOX

    def vets_moves_on_server(self):
        return self._vets


def nooks(seeds):
    """Seeded (nook cell, opening direction) on free floor, its U cells and
    the two approach cells free, the robot able to spin at both approach
    cells with the nook in place, at least 4 cells from the dock."""
    from sim.grid_world import CELL_WALL
    from sim.maps import build_world
    w = build_world(HOUSE)
    dock = (w.x, w.y)

    def free(c):
        return w._cell(*c) != CELL_WALL and c not in w.objects

    out = []
    rng = random.Random("tight-home-3228")
    cand = [(x, y) for y, row in enumerate(w.layout) for x, ch in enumerate(row)
            if ch == "." and free((x, y))]
    while len(out) < seeds:
        c = rng.choice(cand)
        d = rng.choice(((1, 0), (-1, 0), (0, 1), (0, -1)))
        p = (-d[1], d[0])
        need = [c, (c[0] + d[0], c[1] + d[1]), (c[0] + p[0], c[1] + p[1]), (c[0] - p[0], c[1] - p[1]),
                (c[0] - d[0], c[1] - d[1]), (c[0] - 2 * d[0], c[1] - 2 * d[1])]
        if not all(free(n) for n in need) or math.hypot(c[0] + 0.5 - dock[0], c[1] + 0.5 - dock[1]) < 4:
            continue
        if any(o[0] == c for o in out):
            continue
        # On free floor: a nook, not the end of a corridor -- with the nook
        # in place the robot can spin at the mouth and at the cell before it.
        saved = dict(w.objects)
        for o in need[1:4]:
            w.objects[o] = "box"
        roomy = True
        for k in (1, 2):
            w.x, w.y = c[0] - k * d[0] + 0.5, c[1] - k * d[1] + 0.5
            roomy = roomy and can_spin(w)
        w.objects = saved
        if not roomy:
            continue
        out.append((c, d))
    return out


def _quiet():
    import logging
    logging.disable(logging.WARNING)


def run_one(args):
    import brain.dock as docking
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_world import MockWorld
    from robot.safety import SafetyViolation
    from tests.footprint_sweep import truth
    arm, seed, (c, d) = args
    after = arm == "after"
    docking.ESCAPE_BACKS = ESCAPE_BACKS_DEFAULT if after else 0
    grid = build_world(HOUSE)
    p = (-d[1], d[0])
    for o in ((c[0] + d[0], c[1] + d[1]), (c[0] + p[0], c[1] + p[1]), (c[0] - p[0], c[1] - p[1])):
        grid.objects[o] = "box"
    robot = rover_mock_class()(grid, render=False, exact_verbs=True)
    nav = nav_class()(robot)
    body = ServerBody(robot, nav.safety, vets=after)
    runner = MissionRunner(body, policy="explore", navigator=nav, world=MockWorld(grid),
                           mission_time_limit_s=1.0, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=True, job="survey")
    worst = {"gap_cm": math.inf}
    step = robot.step

    def watched(dt):
        r = step(dt)
        nav.safety._retrace_sample()     # the server's 20 Hz wheel loop samples its track
        worst["gap_cm"] = min(worst["gap_cm"], truth(grid)[1])
        return r
    robot.step = watched
    runner.start()
    dock = (grid.x, grid.y)
    # Into the nook: stood a cell before its mouth facing in, then FORWARDs through
    # the server's vet (its retrace track holds the drive). nav's own drive
    # cannot line up with a mouth 3.5 cm wider than the chassis each side.
    grid.x, grid.y = c[0] - 2 * d[0] + 0.5, c[1] - 2 * d[1] + 0.5
    grid.theta = math.atan2(d[1], d[0])
    for _ in range(4):
        nav.safety._retrace_sample()
        try:
            nav.safety.check_and_execute("FORWARD", speed=50, duration=1.0)
        except SafetyViolation:          # refused at the bar: as deep as it goes
            break
    nook = {"x": round(grid.x, 3), "y": round(grid.y, 3), "can_spin": can_spin(grid),
            "in_nook": math.hypot(grid.x - (c[0] + 0.5), grid.y - (c[1] + 0.5)) < 0.75}
    nav.plan_failures = 0
    t0, moves0, gap_before = grid.sim_time, body.moves, worst["gap_cm"]
    worst["gap_cm"] = math.inf
    ticks = 0
    while runner.tick() and ticks < TICK_CAP:
        ticks += 1
    s = runner.status()
    ret = s.get("return") or {}
    return {"arm": arm, "seed": seed, "nook": [list(c), list(d)], **nook,
            "outcome": s["outcome"], "return": ret.get("state"), "reason": ret.get("reason"),
            "trip_s": round(grid.sim_time - t0, 1), "metres": ret.get("metres"),
            "escapes": ret.get("escapes"), "reversals": ret.get("reversals"),
            "plan_failures": nav.plan_failures, "brain_moves": body.moves - moves0,
            "err_m": round(math.hypot(grid.x - dock[0], grid.y - dock[1]) * CELL, 3),
            "min_gap_cm_trip": round(worst["gap_cm"], 2), "min_gap_cm_drive_in": round(gap_before, 2),
            "ticks": ticks, "capped": ticks >= TICK_CAP,
            "log": list(runner._return_trip.log) if runner._return_trip else []}


def verdict(rows):
    def med(xs):
        xs = sorted(xs)
        return xs[len(xs) // 2] if xs else None
    out = {}
    for arm in ("before", "after"):
        rs = [r for r in rows if r["arm"] == arm]
        if not rs:
            continue
        docked = [r for r in rs if r["return"] == "docked"]
        out[arm] = {"runs": len(rs), "docked": len(docked), "docked_share": round(len(docked) / len(rs), 3),
                    "reasons": sorted({str(r["reason"])[:60] for r in rs if r["return"] != "docked"}),
                    "trip_s_median": med([r["trip_s"] for r in rs]),
                    "trip_s_max": max(r["trip_s"] for r in rs),
                    "metres_median": med([r["metres"] or 0 for r in rs]),
                    "timeouts": sum(str(r["reason"]).startswith("timeout") for r in rs),
                    "contacts": sum(r["min_gap_cm_trip"] <= 0.0 for r in rs),
                    "min_gap_cm": min(r["min_gap_cm_trip"] for r in rs),
                    "started_unable_to_spin": sum(not r["can_spin"] for r in rs),
                    "capped": sum(r["capped"] for r in rs)}
    return out


def main(argv):
    arm, out = argv[1], argv[2]
    seeds = int(argv[3]) if len(argv) > 3 else SEEDS
    workers = int(argv[4]) if len(argv) > 4 else 1
    arms = ("before", "after") if arm == "both" else (arm,)
    cases = [(a, i, n) for a in arms for i, n in enumerate(nooks(seeds))]
    _quiet()                    # here, not in run_one: the suite runs it in process (review)
    if workers > 1:
        from multiprocessing import Pool
        with Pool(workers, initializer=_quiet) as pool:
            rows = pool.map(run_one, cases)
    else:
        rows = [run_one(x) for x in cases]
    v = verdict(rows)
    Path(out).write_text(json.dumps({"verdict": v, "rows": rows}, indent=1))
    print(json.dumps(v, indent=1))


if __name__ == "__main__":
    main(sys.argv)
