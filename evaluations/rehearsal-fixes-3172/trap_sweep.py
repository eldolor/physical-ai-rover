#!/usr/bin/env python3
"""3.172.A's instrument: 20 true traps around 3.158.B's, the real ExploreAgent
and ReturnTrip through the real SafetyController, the drop guard on, simulated time.

    python evaluations/rehearsal-fixes-3172/trap_sweep.py [out.jsonl]
"""
import json
import logging
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import brain.dock as dock                                    # noqa: E402
import brain.explore as explore                              # noqa: E402
from brain.memory import MissionMemory                       # noqa: E402
from robot.safety import SafetyController, SafetyViolation   # noqa: E402
from sim.grid_world import CELL_WALL                         # noqa: E402
from sim.maps import build_world                             # noqa: E402
from sim.mock_robot import MockRobot                         # noqa: E402
from sim.mock_world import MockWorld                         # noqa: E402
from tests.dock_sweep import inflated_nav_class              # noqa: E402
from tests.footprint_sweep import CELL_CM, chassis, gap, square  # noqa: E402

HOUSE = "home_first_floor_stairs"
CELL = 0.30
TRAP = (9.56, 10.63, 52.0)        # 3.158.B missions 2 and 18 (m, m, heading deg)
DOCK = (5.25, 11.25, 0.0)         # the rehearsal's dock
N = 20
PIVOT_CM = 1.3
CONTACT_CM = 1.0
TRAP_GAP = 1.1
CONST = {"explore": ("NUDGE_S",), "dock": ("NUDGE_S", "STUCK_S")}


def blocks(world, x, y):
    cells = {(cx, cy) for cy, row in enumerate(world.layout) for cx, c in enumerate(row) if c == CELL_WALL}
    cells |= set(world.objects)
    return [square(*c) for c in cells if abs(c[0] + .5 - x) < 4 and abs(c[1] + .5 - y) < 4]


def min_gap(bs, x, y, h):
    a = chassis(x, y, math.radians(h) - math.pi / 2)
    return min(gap(a, b) for b in bs) * CELL_CM


def swing(bs, x, y, h):
    """Degrees of continuous pivot freedom through this heading, truth."""
    def free(d):
        return min_gap(bs, x, y, h + d) >= PIVOT_CM
    hi = 0
    while hi < 360 and free(hi + 4):
        hi += 4
    lo = 0
    while lo > -360 and free(lo - 4):
        lo -= 4
    return min(360, hi - lo)


def forward_over_the_bar(pose):
    """Amendment 1: the trap's other half. Under the bar the existing creep leaves it."""
    _, _, s, _, _ = body(pose)
    f, _ = s.forward_clearance()
    return f is not None and f > s.min_distance_cm


def traps(world, n=N, seed=3172):
    r, bs, out = random.Random(seed), blocks(world, TRAP[0] / CELL, TRAP[1] / CELL), []
    while len(out) < n:
        x = TRAP[0] + r.uniform(-.03, .03)
        y = TRAP[1] + r.uniform(-.03, .03)
        h = TRAP[2] + r.uniform(-5, 5)
        if (min_gap(bs, x / CELL, y / CELL, h) >= TRAP_GAP and swing(bs, x / CELL, y / CELL, h) < 180
                and forward_over_the_bar((x, y, h))):
            out.append((x, y, h))
    return out


def place(world, pose):
    world.x, world.y = pose[0] / CELL, pose[1] / CELL
    world.theta = math.radians(pose[2] - 90)


def body(pose, wall_in=False):
    w = build_world(HOUSE)
    place(w, pose)
    if wall_in:                       # A4: the cells around it made solid, the robot's own left free
        cx, cy = int(w.x), int(w.y)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if max(abs(dx), abs(dy)) == 2:
                    w.objects.setdefault((cx + dx, cy + dy), "test wall")
    r = MockRobot(w, render=False, exact_verbs=True)
    s = SafetyController(r, 20.0)
    from robot import drop_guard
    g = s._drop_guard()               # the floor it has driven: seeded where it stands
    return w, r, s, g, drop_guard


class Contact:
    def __init__(self, r):
        self.w, self.n, self.low, self.was = r.world, 0, 99.0, False
        self.bs = blocks(self.w, self.w.x, self.w.y)
        step = r.step

        def counted(dt):
            out = step(dt)
            g = min_gap(self.bs, self.w.x, self.w.y, self.w.heading_deg)
            self.low = min(self.low, g)
            now = g < CONTACT_CM
            self.n += now and not self.was
            self.was = now
            return out
        r.step = counted


def run_escape(pose):
    w, r, s, _, _ = body(pose)
    c = Contact(r)
    nav = inflated_nav_class()(r)
    a = explore.ExploreAgent(r, MissionMemory("x"), navigator=nav, clock=lambda: w.sim_time, world=MockWorld(w))
    bs = blocks(w, w.x, w.y)
    t0 = w.sim_time
    a._queue_escape("wedged")
    freed = None
    for _ in range(60):
        if a._pending:
            a._do_pending()
        else:
            break
        if swing(bs, w.x, w.y, w.heading_deg) >= 180:
            freed = round(w.sim_time - t0, 1)
            break
    return {"freed_s": freed, "contacts": c.n, "low_cm": round(c.low, 1),
            "moved_cm": round(math.hypot(w.x * CELL - pose[0], w.y * CELL - pose[1]) * 100, 1)}


def hold_nav(r, forever):
    """nav2 at the trap: the goal stays `active` while the robot cannot swing
    (live: 11,208 refused turns, no abort for minutes); once it can, the
    stand-in drives. `forever`: A4's, never drives."""
    base = inflated_nav_class()(r)

    class Hold:
        def __init__(self):
            self.goal, self.seen = None, (None, False)

        def set_goal(self, x, y):
            self.goal = (x, y)
            return {"accepted": True, "goal_id": "g"}

        def cancel_goal(self):
            self.goal = None
            return base.cancel_goal() if getattr(base, "goal", None) else {}

        def _free(self):
            w = r.world
            key = (round(w.x, 3), round(w.y, 3), round(w.heading_deg, 1))
            if key != self.seen[0]:        # polled every 0.5 s; the robot rarely moved
                self.seen = (key, swing(blocks(w, w.x, w.y), w.x, w.y, w.heading_deg) >= 180)
            return self.seen[1]

        def get_goal(self):
            if self.goal is None:
                return {"goal": None}
            if forever or (not getattr(base, "goal", None) and not self._free()):
                return {"goal": {"state": "active", "x_m": self.goal[0], "y_m": self.goal[1]}}
            if not getattr(base, "goal", None):
                base.set_goal(*self.goal)
            return base.get_goal()
    return Hold()


def run_return(pose, wall_in=False):
    w, r, s, _, _ = body(pose, wall_in)
    c = Contact(r)
    nav = hold_nav(r, wall_in)
    world = MockWorld(w)
    trip = dock.ReturnTrip(dock.dock_from_pose({"x_m": DOCK[0], "y_m": DOCK[1], "heading_deg": DOCK[2], "usable": True}),
                           nav, s, world.get_pose, lambda: w.sim_time, idle=r.pass_time, get_map=world.get_map)
    got, t0, ticks = None, w.sim_time, 0
    while got is None and ticks < 40000:
        ticks += 1
        got = trip.step()
    return {"state": got and got[0], "reason": got and got[1], "s": round(w.sim_time - t0, 1),
            "stuck_rounds": trip.stuck_rounds, "contacts": c.n, "low_cm": round(c.low, 1), "ticks": ticks}


def arm_off():
    saved = {(m, n): getattr(m, n) for m, n in ((explore, "NUDGE_S"), (dock, "NUDGE_S"), (dock, "STUCK_S"))}
    for (m, n) in saved:
        setattr(m, n, 0.0)
    return saved


def main():
    logging.disable(logging.WARNING)
    out = open(sys.argv[1], "w") if len(sys.argv) > 1 else None
    poses = traps(build_world(HOUSE))
    for arm in ("off", "on"):
        saved = arm_off() if arm == "off" else {}
        rows = []
        for i, p in enumerate(poses):
            row = {"arm": arm, "i": i, "pose": [round(v, 3) for v in p], "escape": run_escape(p),
                   "ret": run_return(p), "sealed": run_return(p, wall_in=True)}
            rows.append(row)
            if out:
                out.write(json.dumps(row) + "\n")
                out.flush()
        for (m, n), v in saved.items():
            setattr(m, n, v)
        print(arm, "A1 freed", sum(r["escape"]["freed_s"] is not None and r["escape"]["freed_s"] <= 60 for r in rows),
              "A2 contacts", sum(r["escape"]["contacts"] + r["ret"]["contacts"] for r in rows),
              "low", min(min(r["escape"]["low_cm"], r["ret"]["low_cm"]) for r in rows),
              "A3 docked", sum(r["ret"]["state"] == "docked" for r in rows), sorted({str(r["ret"]["reason"])[:50] for r in rows}),
              "A4", sum(r["sealed"]["state"] == "failed" and "no_motion" in str(r["sealed"]["reason"]) for r in rows),
              "max s", max(r["sealed"]["s"] for r in rows), sorted({str(r["sealed"]["reason"])[:50] for r in rows}), flush=True)


if __name__ == "__main__":
    main()
