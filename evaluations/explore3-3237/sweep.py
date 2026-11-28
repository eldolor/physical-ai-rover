# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/explore3-3237/sweep.py

3.237 B0's baseline (`docs/plans/ros-alignment/3.237-explore-three-sensors.md`):
explore in process, through MissionRunner -> ExploreAgent -> the safety layer
-> MockRobot, over houses with low things, judged on ground truth.

**nav2 is stood in for by a driving planner that is blind to low things**
(`LidarNav`): 3.159's VettedNav (it drives every goal through
`vet_wheel_velocity()` then `advance()`, as nav2's twists reach the wheels),
planning over walls and `lidar_solid_cells` only, as nav2's costmap built from
the lidar does. `FakeNav` and `WalkingNav` plan over every object, low ones
included, so with them no low contact can happen and the problem is invisible.

**The judge shares no code with what it judges** (`tests/footprint_sweep.py`'s
rule): the chassis is that module's URDF rectangle and the geometry its
polygons; a contact is the outline within CONTACT_CM of a low object's cell,
counted again only after it has left by LEAVE_CM. Coverage is the camera's
view cast here from the true pose against walls and tall objects, not
explore's own `seen` set.

"Low" is the judge's own rule, written here: an object whose labelled height
(`object_heights`) is under LOW_BELOW_M, the D500's plane by the xacro --
not `GridWorld.low_cells`, which the scan and the planner use.

The container's 4 cores, not the desktop, so no desktop lock (3.128) is taken;
on the desktop, wrap the command in `python tools/desktop_lock.py shared`.

Arms (B0): `L` lidar only (the floor block withheld from the safety layer, as
the Rover today: `no_floor_band`); `V` today's sim (the floor band's veto).

Arms (B, amendment 2), on A's scatter (`SIM_LOW_SCATTER=8:0`) and
`SIM_FLOOR_BAND`: `B1` lidar only; `BV` the veto alone (reference); `B2` the
veto and the band's low points as hazard zones; `B3` the camera's hazard
detections as zones, no band; `B4` all three. `LidarNav` avoids hazard zones'
cells (grown by the half-width), as nav2 would with 3.231.C2's keepouts.

    python evaluations/explore3-3237/sweep.py L V            # B0 (recorded)
    python evaluations/explore3-3237/sweep.py B1 BV B2 B3 B4  # B -> results/B/
"""

import json
import logging
import math
import random
import statistics
import sys
import time
from collections import deque
from multiprocessing import Pool
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from sim.grid_world import CELL_WALL  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.maps.complex_house import LOW_HEIGHTS_M  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from tests.drop_sweep import _vetted_nav_class  # noqa: E402
from tests.footprint_sweep import CELL_CM, chassis, gap, square  # noqa: E402

# 3.227's houses (3.80.G's), the first ten after the furnished home.
PROCTHOR = [f"procthor:val:{i}" for i in (0, 2, 3, 4, 5, 9, 10, 11, 12, 13)]
HOUSES = ["complex_house_hazards"] + PROCTHOR
STARTS = 5
SCATTER = 8                      # low things per procthor house (B)
SCATTER_LABELS = ("cable", "sock", "shoe", "toy", "slippers", "book", "towel", "pet bowl")
TARGET = "red backpack"
LIMIT_S = 1200.0
CONTACT_CM, LEAVE_CM = 1.0, 3.0
LOW_BELOW_M = 0.120              # the lidar's plane: laser_z in the xacro, 12 cm
TICK_CAP = 5000
ARMS = ("L", "V", "B1", "BV", "B2", "B3", "B4")
# B's arms: (floor band published, hazard sources)
B_ARMS = {"B1": (False, ()), "BV": (True, ()), "B2": (True, ("floor_band",)),
          "B3": (False, ("camera",)), "B4": (True, ("camera", "floor_band"))}
B_SCATTER = "8:0"
B_OUT = Path(__file__).with_name("results") / "B"
AT_TARGET_M = 0.9
CAM_FOV = math.radians(60.0)     # the sim camera's (renderer.FPV_FOV), stated here
CAM_RANGE = 14.0                 # cells (renderer.FPV_MAX_DIST)
CAM_RAYS = 61                    # 1 degree apart: 0.24 cell between rays at CAM_RANGE
SEEN_EVERY_S = 0.5
OUT = Path(__file__).with_name("results")


def _free(world, cell):
    x, y = cell
    return (0 <= y < len(world.layout) and 0 <= x < len(world.layout[0])
            and world.layout[y][x] != CELL_WALL)


def reachable(world, start, blocked):
    seen, q = {start}, deque([start])
    while q:
        c = q.popleft()
        for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
            if n not in seen and _free(world, n) and n not in blocked:
                seen.add(n)
                q.append(n)
    return seen


def scatter(world, house, start):
    """SCATTER low things on free cells, seeded by the house: never within
    1 m of the start, never on the target, and no placement may cut off any
    free cell it does not itself cover (3.237 A4's rule, here until A)."""
    rng = random.Random(f"3237-scatter-{house}")
    objects = dict(world.objects)
    base = reachable(world, start, set(objects))
    cells = sorted(base)
    placed = 0
    while placed < SCATTER and cells:
        c = rng.choice(cells)
        cells.remove(c)
        if math.dist(c, start) * 0.3 < 1.0:
            continue
        trial = set(objects) | {c}
        if reachable(world, start, trial) != base - {c}:
            continue
        objects[c] = SCATTER_LABELS[placed % len(SCATTER_LABELS)]
        base = base - {c}
        placed += 1
    world.objects = objects
    world.object_heights = dict(LOW_HEIGHTS_M)


def house_world(house, b=False):
    """B0: the harness's own scatter. B: A's (`SIM_LOW_SCATTER`), on every
    house but the hazard house (which holds every scatter label already)."""
    import os
    if b:
        if house.startswith("procthor"):
            os.environ["SIM_LOW_SCATTER"] = B_SCATTER
        else:
            os.environ.pop("SIM_LOW_SCATTER", None)
        try:
            return build_world(house)
        finally:
            os.environ.pop("SIM_LOW_SCATTER", None)
    world = build_world(house)
    if house.startswith("procthor"):
        scatter(world, house, (world.robot_x, world.robot_y))
    return world


def starts(house, b=False):
    """STARTS seeded (x, y, heading): a free cell's centre with nothing
    within the chassis' circumscribed circle, at least 1 m from the target."""
    world = house_world(house, b)
    rng = random.Random(f"3237-starts-{house}")
    targets = [c for c, n in world.objects.items() if n == TARGET]
    occupied = set(world.objects)
    low = low_of(world)
    free = sorted(reachable(world, (world.robot_x, world.robot_y), occupied))
    rng.shuffle(free)                       # without replacement (review)
    radius = math.hypot(12.65, 11.55) / CELL_CM
    out = []
    while len(out) < STARTS:
        if not free:
            raise RuntimeError(f"{house}: only {len(out)} starts qualify")
        cx, cy = free.pop()
        x, y = cx + 0.5, cy + 0.5
        # A4's 1 m, at the start each mission actually uses (review).
        if any(math.dist((x, y), (t[0] + 0.5, t[1] + 0.5)) * 0.3 < 1.0 for t in targets + low):
            continue
        near = [(i, j) for j in range(cy - 2, cy + 3) for i in range(cx - 2, cx + 3)
                if not _free(world, (i, j)) or (i, j) in occupied]
        if any(_point_square(x, y, c) <= radius for c in near):
            continue
        out.append((x, y, rng.randrange(0, 360, 15)))
    return out


def low_of(world):
    """The judge's low things: labelled height under LOW_BELOW_M."""
    heights = world.object_heights or {}
    return sorted(c for c, n in world.objects.items()
                  if heights.get(n, math.inf) < LOW_BELOW_M)


def _point_square(x, y, c):
    dx = max(c[0] - x, 0.0, x - (c[0] + 1))
    dy = max(c[1] - y, 0.0, y - (c[1] + 1))
    return math.hypot(dx, dy)


class Judge:
    """Ground truth after every integration step: contact events with low
    things and with anything else, and the camera's coverage."""

    def __init__(self, world, low, tall, robot=None):
        self.world, self.low, self.tall, self.robot = world, low, tall, robot
        self.by_motion = {"forward": 0, "astern": 0, "pivot": 0, "still": 0}
        self.touching: dict = {}
        self.low_events = self.other_events = 0
        self.seen: set = set()
        self.seen_at_300 = None
        self.next_seen = 0.0

    def check(self):
        w = self.world
        A = chassis(w.x, w.y, w.theta)
        r = 2
        for cy in range(int(w.y) - r, int(w.y) + r + 1):
            for cx in range(int(w.x) - r, int(w.x) + r + 1):
                c = (cx, cy)
                is_low = c in self.low
                if not (is_low or c in self.tall or not _free(w, c)):
                    continue
                g = gap(A, square(cx, cy)) * CELL_CM
                was = self.touching.get(c, False)
                if not was and g <= CONTACT_CM:
                    self.touching[c] = True
                    if is_low:
                        self.low_events += 1
                        self.by_motion[self._motion()] += 1
                    else:
                        self.other_events += 1
                elif was and g >= LEAVE_CM:
                    self.touching[c] = False
        # The 300 s snapshot before a look taken past 300 s (review).
        if self.seen_at_300 is None and w.sim_time >= 300.0:
            self.seen_at_300 = set(self.seen)
        if w.sim_time >= self.next_seen:
            self.next_seen = w.sim_time + SEEN_EVERY_S
            self._look()

    def _motion(self):
        """What the wheels were commanded at the contact (the judge reads
        the command, not any sensor)."""
        left, right = getattr(self, "cmd", (0.0, 0.0))
        v, w = (left + right) / 2.0, right - left
        if abs(v) < 0.05 and abs(w) < 0.05:
            return "still"
        if abs(v) < 0.25 * abs(w):
            return "pivot"
        return "forward" if v > 0 else "astern"

    def _look(self):
        """Floor cells in the camera's view, rays stopped by walls and tall
        things (a low thing hides nothing from a camera above it)."""
        w = self.world
        view = w.view_angle()
        for k in range(CAM_RAYS):
            a = view - CAM_FOV / 2 + CAM_FOV * k / (CAM_RAYS - 1)
            dx, dy = math.cos(a), math.sin(a)
            t = 0.0
            while t < CAM_RANGE:
                c = (math.floor(w.x + dx * t), math.floor(w.y + dy * t))
                if not _free(w, c) or c in self.tall:
                    break
                self.seen.add(c)
                t += 0.1


def build_robot(arm, world):
    import os
    if arm in B_ARMS:
        os.environ["SIM_FLOOR_BAND"] = "1" if B_ARMS[arm][0] else "0"
    robot = MockRobot(world, render=False)
    if arm == "L":
        grid_fn = robot.get_depth_grid

        def no_band():
            grid = grid_fn()
            if isinstance(grid, dict):
                grid = {k: v for k, v in grid.items() if k != "floor"}
            return grid
        robot.get_depth_grid = no_band
    return robot


def lidar_nav(robot, zones_fn=None):
    base = _vetted_nav_class()

    class LidarNav(base):
        """VettedNav planning over what the lidar sees: walls and
        `lidar_solid_cells`, never a low thing. Counts every refusal or
        slowing by its reason (VettedNav counts only drops)."""

        def __init__(self, robot):
            super().__init__(robot)
            self.refusals = {"floor_band": 0, "other": 0}
            vet = self.safety.vet_wheel_velocity

            def counted(left, right):
                out = vet(left, right)
                if out[2] is not None:
                    self.refusals["floor_band" if "floor_band" in str(out[2]) else "other"] += 1
                return out
            self.safety.vet_wheel_velocity = counted

        def _hazard_cells(self):
            """Cells whose centre lies in a hazard zone grown by the
            half-width (nav2's keepout with its inflation), cached by the
            zones' ids."""
            zones = [z for z in (zones_fn() if zones_fn else ())
                     if str(z.get("id", "")).startswith("hazard-")]
            key = tuple(z["id"] for z in zones)
            if getattr(self, "_hz", (None,))[0] != key:
                grow, cells = 0.12, set()
                for z in zones:
                    for j in range(int((z["y0"] - grow) / 0.3), int((z["y1"] + grow) / 0.3) + 1):
                        for i in range(int((z["x0"] - grow) / 0.3), int((z["x1"] + grow) / 0.3) + 1):
                            cx, cy = (i + 0.5) * 0.3, (j + 0.5) * 0.3
                            if (z["x0"] - grow <= cx <= z["x1"] + grow
                                    and z["y0"] - grow <= cy <= z["y1"] + grow):
                                cells.add((i, j))
                self._hz = (key, cells)
            return self._hz[1]

        def _path(self, a, b):
            g, prev, q = self.grid, {a: None}, deque([a])
            solid = g.lidar_solid_cells
            keep = self._hazard_cells()
            while q:
                c = q.popleft()
                if c == b:
                    out = []
                    while c is not None:
                        out.append(c)
                        c = prev[c]
                    return out[::-1]
                for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
                    if (n not in prev and g._cell(*n) != CELL_WALL and n not in solid
                            and n not in keep):
                        prev[n] = c
                        q.append(n)
            return None

        def get_goal(self):
            """VettedNav's drive, with nav2's answer to a keepout (review): a
            goal whose cell is in a hazard zone, or that no path reaches
            around them now (a hazard marked mid-goal), is aborted -- never
            driven to in a straight line, which VettedNav's fallback did."""
            if self.goal and self.goal["state"] == "active":
                g = self.grid
                to = (math.floor(self.goal["x_m"] / 0.3), math.floor(self.goal["y_m"] / 0.3))
                if to in self._hazard_cells() or self._path((g.robot_x, g.robot_y), to) is None:
                    self.calls.append(("get",))
                    self.robot.set_wheel_velocity(0.0, 0.0)
                    self.robot.pass_time(1.0)
                    self.goal["state"] = "aborted"
                    return {"goal": self.goal, "plan": []}
            # Hazards change only on explore's ticks, between polls, so the
            # check above covers the whole poll's drive.
            return super().get_goal()
    return LidarNav(robot)


def run(args):
    try:
        return _run(args)
    except Exception as e:  # noqa: BLE001 -- one failed mission must not lose the arm
        arm, house, x, y, heading, job = args
        return {"arm": arm, "house": house, "x": x, "y": y, "heading": heading, "job": job,
                "error": f"{type(e).__name__}: {e}"}


def _run(args):
    from control.mission_runner import MissionRunner
    from sim.mock_world import MockWorld
    arm, house, x, y, heading, job = args
    b = arm in B_ARMS
    world = house_world(house, b)
    world.x, world.y = x, y
    world.theta = math.radians(heading - 90)
    low = set(low_of(world))
    tall = set(world.objects) - low
    start = (math.floor(x), math.floor(y))
    floor = reachable(world, start, set(world.objects))
    robot = build_robot(arm, world)
    judge = Judge(world, low, tall, robot)
    looks = {"n": 0, "s": 0.0}
    for verb in ("look_left", "look_right", "look_center", "look_step", "tilt_step"):
        fn = getattr(robot, verb)

        def timed(*a, _fn=fn, **k):
            t0 = world.sim_time
            out = _fn(*a, **k)
            looks["n"] += 1
            looks["s"] += world.sim_time - t0
            return out
        setattr(robot, verb, timed)
    step = robot.step

    def checked(dt):
        # The command in effect for this step (review: the veto or the nav
        # loop may zero it within the step that touched).
        judge.cmd = (robot._cmd_left_rad_s, robot._cmd_right_rad_s)
        out = step(dt)
        judge.check()
        return out
    robot.step = checked
    holder = {}
    nav = lidar_nav(robot, zones_fn=lambda: getattr(holder.get("agent"), "zones", ()))
    kw = {"job": "survey"} if job == "survey" else {"target_object": TARGET}
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=MockWorld(world),
                           mission_time_limit_s=LIMIT_S, clock=lambda: world.sim_time,
                           idle=robot.pass_time, **kw)
    t0 = time.time()
    runner.start()
    holder["agent"] = runner.agent
    if b and B_ARMS[arm][1]:
        from brain.hazards import HazardList
        runner.agent.hazards = HazardList()
        runner.agent.hazard_sources = B_ARMS[arm][1]
    ticks = 0
    while runner.tick() and ticks < TICK_CAP:
        ticks += 1
    s = runner.status()
    targets = [c for c, n in world.objects.items() if n == TARGET]
    end_m = min((math.dist((world.x, world.y), (c[0] + 0.5, c[1] + 0.5)) * 0.3 for c in targets),
                default=math.inf)
    at300 = judge.seen_at_300 if judge.seen_at_300 is not None else judge.seen
    return {"arm": arm, "house": house, "x": x, "y": y, "heading": heading, "job": job,
            "outcome": s.get("outcome"), "sim_s": round(world.sim_time, 1),
            "end_target_m": round(end_m, 2),
            "low_contacts": judge.low_events, "other_contacts": judge.other_events,
            "coverage": round(len(judge.seen & floor) / len(floor), 4),
            "coverage_300": round(len(at300 & floor) / len(floor), 4),
            "ticks": ticks, "capped": ticks >= TICK_CAP,
            "low_by_motion": judge.by_motion,
            "looks": looks["n"], "look_s": round(looks["s"], 1),
            "hazards": (dict(runner.agent.hazards.added)
                        if getattr(runner.agent, "hazards", None) is not None else None),
            "refusals_floor_band": nav.refusals["floor_band"],
            "refusals_other": nav.refusals["other"], "wall_s": round(time.time() - t0, 1)}


def summary(all_rows):
    bad = [r for r in all_rows if "error" in r or r.get("capped")]
    rows = [r for r in all_rows if r not in bad]
    finds = [r for r in rows if r["job"] == "find"]
    surveys = [r for r in rows if r["job"] == "survey"]
    hit = [r for r in finds if r["outcome"] == "found"
           or (r["outcome"] == "arrived_unconfirmed" and r["end_target_m"] <= AT_TARGET_M)]
    med = (lambda xs: round(statistics.median(xs), 4) if xs else None)
    return {"missions": len(rows), "left_out_error_or_capped": len(bad),
            "refusals_floor_band": sum(r["refusals_floor_band"] for r in rows),
            "low_contacts": sum(r["low_contacts"] for r in rows),
            "low_contacts_survey_median": med([r["low_contacts"] for r in surveys]),
            "missions_with_low_contact": sum(r["low_contacts"] > 0 for r in rows),
            "other_contacts": sum(r["other_contacts"] for r in rows),
            "find_rate": round(len(hit) / len(finds), 4) if finds else None,
            "find_time_median_s": med([r["sim_s"] for r in hit]),
            "look_share_median": med([r.get("look_s", 0.0) / r["sim_s"] for r in rows
                                      if r["sim_s"] and "look_s" in r]),
            "low_by_motion": {k: sum((r.get("low_by_motion") or {}).get(k, 0) for r in rows)
                              for k in ("forward", "astern", "pivot", "still")},
            "coverage_median": med([r["coverage"] for r in surveys]),
            "coverage_300_median": med([r["coverage_300"] for r in surveys]),
            "outcomes": {o: sum(r["outcome"] == o for r in rows)
                         for o in sorted({str(r["outcome"]) for r in rows})}}


def main(*arms):
    logging.disable(logging.WARNING)
    for arm in arms:
        if arm not in ARMS:
            raise SystemExit(f"unknown arm {arm!r}: one of {ARMS}")
    for arm in arms or ("L",):
        b = arm in B_ARMS
        out_dir = B_OUT if b else OUT
        out_dir.mkdir(parents=True, exist_ok=True)
        jobs = [(h, *s) for h in HOUSES for s in starts(h, b)]
        t = time.time()
        log = out_dir / f"{arm}.jsonl"
        log.write_text("")
        tasks = [(arm, h, x, y, hd, job) for h, x, y, hd in jobs for job in ("find", "survey")]
        with Pool(4) as pool:
            rows = []
            for r in pool.imap(run, tasks):
                rows.append(r)
                with log.open("a") as f:
                    f.write(json.dumps(r) + "\n")
                print(json.dumps(r), flush=True)
        out = {"arm": arm, **summary(rows), "wall_s": round(time.time() - t), "rows": rows}
        (out_dir / f"{arm}.json").write_text(json.dumps(out, indent=1) + "\n")
        print(json.dumps({k: v for k, v in out.items() if k != "rows"}), flush=True)


if __name__ == "__main__":
    main(*sys.argv[1:])
