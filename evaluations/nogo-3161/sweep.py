# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3161/sweep.py -- 3.161.B's in-process sweeps (Z1, Z2, Z3,
Z5, Z10 of docs/plans/ros-alignment/3.161-no-go-zones-on-twin.md).

The real path in process: `MissionRunner` -> explore -> `robot/safety.py`
(MockRobot) -> the simulator, with `ZoneNav` for nav2: a goal walks one
0.30 m cell per poll along the shortest floor path that crosses no cell whose
centre lies in a drawn zone (nav2's keepout mask: lethal to the robot's
centre), and aborts if no such path exists or the goal is in one. Zones a
robot stands in are left out of that mask by the bridge's own rule
(`picar_bridge/keepout.py suspend`), fed the robot's true pose each poll.

* `find`: houses `home_first_floor`, `scaled_house` and the first 30 usable
  ProcTHOR val houses (as tests/procthor_sweep.py takes them), the red
  backpack, `SEEDS` seeds each; two arms -- `zones` (3 zones a house placed by
  `place_zones`, a seeded rule that never covers the start or the target and
  keeps the target reachable outside every zone grown by the robot's
  half-width) and `none` (no zones, the same seed and navigator). Z1: every
  goal explore sent, against the zones. Z2: the true centre after every tick.
  Z5: found and search time, paired.
* `survey`: the same houses' surveys, both arms, coverage of the floor
  outside the zones (`map_coverage(m, zones)`) at the end. Z5's second half.
* `leave`: 10 houses, a 0.8 m zone centred on the robot's start (Z3): does
  it leave, and is the zone drawn again within 0.30 m of clearing it?

    python tools/desktop_lock.py shared --why "3.161.B sweep" -- \\
        python evaluations/nogo-3161/sweep.py find OUT.json [--arm none]
    python evaluations/nogo-3161/sweep.py survey OUT.json
    python evaluations/nogo-3161/sweep.py leave OUT.json

`--arm none` alone, run from a `dev` worktree with this file copied in, is
Z10's baseline (no zones there: explore and the navigator are `dev`'s).
"""

import json
import math
import os
import random
import sys
import time
from collections import deque

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
if sys.path[:1] != [ROOT]:
    sys.path.insert(0, ROOT)
sys.path.insert(1, os.path.join(ROOT, "service/slam/src/picar_bridge"))

import logging                                           # noqa: E402
logging.disable(logging.WARNING)

from control.mission_runner import MissionRunner        # noqa: E402
from sim.grid_world import CELL_WALL                     # noqa: E402
from sim.maps import build_world                         # noqa: E402
from sim.maps.procthor import usable                     # noqa: E402
from sim.mock_robot import MockRobot                     # noqa: E402
from sim.mock_world import MockWorld                     # noqa: E402
from tests.test_explore import CELL, FakeNav             # noqa: E402

for pkg in ("brain", "control", "sim", "world", "tests"):
    where = os.path.dirname(os.path.realpath(__import__(pkg).__file__))
    if where != os.path.join(ROOT, pkg):
        raise SystemExit(f"{pkg} is {where}, not under {ROOT}: this harness's own checkout")
print(f"brain: {os.path.join(ROOT, 'brain')}", file=sys.stderr)

TARGET = "red backpack"
HOUSES = ["home_first_floor", "scaled_house"] + [f"procthor:val:{i}" for i in usable()[:30]]
SEEDS = (0, 1, 2)
N_ZONES = 3
MAX_STEPS = 200
TICK_CAP = 5000
SURVEY_STEPS = 120
HALF_WIDTH_M = 0.12
LEAVE_M = 0.30
TRIES = 400


# ---------- zones, as plain dicts (the dev arm imports nothing new) ----------

def _dist(z, x, y):
    dx = max(z["x0"] - x, 0.0, x - z["x1"])
    dy = max(z["y0"] - y, 0.0, y - z["y1"])
    return math.hypot(dx, dy)


def _in(z, x, y, grow=0.0):
    return _dist(z, x, y) <= grow if grow > 0 else (z["x0"] <= x <= z["x1"] and z["y0"] <= y <= z["y1"])


def _floor(g, c):
    return g._cell(*c) != CELL_WALL and c not in g.objects


def _reach(g, a, b, blocked):
    seen, q = {a}, deque([a])
    while q:
        c = q.popleft()
        if c == b:
            return True
        for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
            if n not in seen and (n == b or (_floor(g, n) and n not in blocked)):
                seen.add(n)
                q.append(n)
    return False


def _centre(c):
    return ((c[0] + 0.5) * CELL, (c[1] + 0.5) * CELL)


def target_cells(g):
    return [c for c, lab in g.objects.items() if lab == TARGET]


def place_zones(house, seed, n=N_ZONES):
    """3 zones on free floor, seeded: none within 1.0 m of the start or the
    target, none overlapping, and the target still reachable from the start
    through floor whose cell centres are outside every zone grown by the
    robot's half-width (the zone walls nothing off, Z5)."""
    g = build_world(house)
    rng = random.Random(f"{house}:{seed}")
    start = (g.robot_x, g.robot_y)
    targets = target_cells(g)
    free = [(x, y) for y in range(g.height) for x in range(g.width) if _floor(g, (x, y))]
    zones = []
    for _ in range(TRIES):
        if len(zones) == n:
            break
        cx, cy = _centre(rng.choice(free))
        w, h = rng.uniform(0.4, 1.2), rng.uniform(0.4, 1.2)
        z = {"id": f"z{len(zones)}{seed}", "x0": round(cx - w / 2, 3), "y0": round(cy - h / 2, 3),
             "x1": round(cx + w / 2, 3), "y1": round(cy + h / 2, 3), "kind": "area",
             "set_at": 0.0, "by": "person"}
        keep = [_centre(start)] + [_centre(t) for t in targets]
        if any(_dist(z, *p) < 1.0 for p in keep):
            continue
        if any(not (z["x1"] < o["x0"] or o["x1"] < z["x0"] or z["y1"] < o["y0"] or o["y1"] < z["y0"])
               for o in zones):
            continue
        blocked = {c for c in free if any(_in(q, *_centre(c), HALF_WIDTH_M) for q in zones + [z])}
        if targets and not any(_reach(g, start, t, blocked) for t in targets):
            continue
        zones.append(z)
    return zones


class ZoneNav(FakeNav):
    """FakeNav that walks a cell per poll around the zones (nav2's mask),
    leaving out a zone the robot is in by the bridge's rule (D4)."""

    def __init__(self, robot, zones=(), on_move=None):
        super().__init__(robot)
        self.zones = list(zones)
        self.suspended = set()
        self.fresh = {z["id"] for z in self.zones}
        self.on_move = on_move
        self.events = []

    def _pose(self):
        g = self.grid
        return (g.x * CELL, g.y * CELL)

    def watch(self):
        """The bridge's suspension, on the true pose (its 5 Hz watch)."""
        if not self.zones:
            return
        from picar_bridge import keepout
        polys = [{"id": z["id"], "poly": [[z["x0"], z["y0"]], [z["x1"], z["y0"]],
                                           [z["x1"], z["y1"]], [z["x0"], z["y1"]]]}
                 for z in self.zones]
        x, y = self._pose()
        after = keepout.suspend(polys, self.suspended, (x, y), self.fresh)
        self.fresh = set()
        for zid in after - self.suspended:
            self.events.append(("suspended", zid, round(self.grid.sim_time, 1)))
        for zid in self.suspended - after:
            z = next(q for q in self.zones if q["id"] == zid)
            self.events.append(("restored", zid, round(_dist(z, x, y), 3)))
        self.suspended = after

    def _lethal(self, c):
        x, y = _centre(c)
        return any(_in(z, x, y) for z in self.zones if z["id"] not in self.suspended)

    def _path(self, a, b):
        g, prev, q = self.grid, {a: None}, deque([a])
        while q:
            c = q.popleft()
            if c == b:
                out = []
                while c is not None:
                    out.append(c)
                    c = prev[c]
                return out[::-1]
            for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
                if n not in prev and _floor(g, n) and not self._lethal(n):
                    prev[n] = c
                    q.append(n)
        return None

    def get_goal(self):
        self.calls.append(("get",))
        self.watch()
        if self.goal and self.goal["state"] == "active":
            g = self.grid
            to = (math.floor(self.goal["x_m"] / CELL), math.floor(self.goal["y_m"] / CELL))
            inside = any(_in(z, self.goal["x_m"], self.goal["y_m"]) for z in self.zones
                         if z["id"] not in self.suspended)
            path = None if (to in g.objects or inside) else self._path((g.robot_x, g.robot_y), to)
            self.robot.pass_time(1.0)
            if path is None:
                self.goal["state"] = "aborted"
            elif len(path) <= 2:
                tx, ty = self.goal["x_m"] / CELL, self.goal["y_m"] / CELL
                if (tx, ty) != (g.x, g.y):
                    g.theta = math.atan2(ty - g.y, tx - g.x)
                g.x, g.y = tx, ty
                self.goal["state"] = "succeeded"
            else:
                nx, ny = path[1][0] + 0.5, path[1][1] + 0.5
                g.theta = math.atan2(ny - g.y, nx - g.x)
                g.x, g.y = nx, ny
            if self.on_move:
                self.on_move()
        return {"goal": self.goal, "plan": []}


def run(house, seed, arm, job="find", zones=None, max_steps=MAX_STEPS):
    g = build_world(house)
    zones = (place_zones(house, seed) if zones is None else zones) if arm == "zones" else []
    robot = MockRobot(g, render=False)
    entries = []
    started_inside = any(_in(z, g.x * CELL, g.y * CELL) for z in zones)
    left = [not started_inside]

    def check():
        x, y = g.x * CELL, g.y * CELL
        inside = [z["id"] for z in zones if _in(z, x, y)]
        if not inside:
            left[0] = True
        elif left[0]:
            entries.append({"zones": inside, "at": [round(x, 2), round(y, 2)],
                            "t": round(g.sim_time, 1)})

    nav = ZoneNav(robot, zones, on_move=check)
    kw = dict(target_object=TARGET) if job == "find" else dict(job="survey")
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=MockWorld(g),
                           max_steps=max_steps, clock=lambda: g.sim_time, idle=robot.pass_time, **kw)
    if zones and hasattr(runner, "set_nogo"):
        runner.set_nogo("map-sweep", zones)
    row = {"house": house, "seed": seed, "arm": arm, "job": job, "zones": zones}
    t0 = time.time()
    try:
        runner.start()
        ticks = 0
        while runner.tick() and ticks < TICK_CAP:
            ticks += 1
            check()
        s = runner.status()
        sets = [(c[1], c[2]) for c in nav.calls if c[0] == "set"]
        row.update(outcome=s["outcome"], ticks=ticks, capped=ticks >= TICK_CAP,
                   sim_s=round(g.sim_time, 1), goals=len(sets),
                   goals_in_zone=[list(p) for p in sets if any(_in(z, *p) for z in zones)],
                   goals_in_grown=[list(p) for p in sets
                                   if any(_in(z, *p, HALF_WIDTH_M) for z in zones)],
                   entries=entries, suspend_events=nav.events,
                   nogo=(s.get("explore") or {}).get("nogo"),
                   goal_seq=sets[:400])
        if job == "survey":
            m = runner.world.get_map()
            row["map_cells"] = list(m.get("cells"))
        row["left"] = left[0]
    except Exception as e:  # noqa: BLE001 -- counted, never hidden
        import traceback
        row.update(outcome="exception", error=f"{type(e).__name__}: {e}",
                   tb=traceback.format_exc()[-1500:])
    row["wall_s"] = round(time.time() - t0, 1)
    return row


def _pool(fn, jobs, workers):
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(workers) as ex:
        return list(ex.map(fn, jobs))


def _find(job):
    house, seed, arm = job
    r = run(house, seed, arm)
    r.pop("goal_seq") if arm == "zones" else None
    print(f"{arm:5} {house:20} s{seed} {r['outcome']:13} goals {r.get('goals')} "
          f"inzone {len(r.get('goals_in_zone', []))} entries {len(r.get('entries', []))} "
          f"{r['wall_s']} s", file=sys.stderr, flush=True)
    return r


def _survey(job):
    house, seed = job
    from brain.mission_report import map_coverage
    zones = place_zones(house, seed)
    out = []
    for arm in ("zones", "none"):
        r = run(house, seed, arm, job="survey", zones=zones, max_steps=SURVEY_STEPS)
        cells = r.pop("map_cells", None)
        r.pop("goal_seq", None)
        if cells is not None:
            g = build_world(house)
            m = dict(MockWorld(g).get_map(), cells=cells)
            r["coverage_outside"] = map_coverage(m, zones)["coverage"]
            r["mapped_free_outside_m2"] = map_coverage(m, zones)["mapped_free_m2"]
        r["zones"] = zones
        out.append(r)
        print(f"survey {arm:5} {house:20} s{seed} {r['outcome']} cov_out {r.get('coverage_outside')}",
              file=sys.stderr, flush=True)
    return out


def _leave(house):
    g = build_world(house)
    x, y = g.x * CELL, g.y * CELL
    z = {"id": "zstart", "x0": round(x - 0.4, 3), "y0": round(y - 0.4, 3), "x1": round(x + 0.4, 3),
         "y1": round(y + 0.4, 3), "kind": "area", "set_at": 0.0, "by": "person"}
    r = run(house, 0, "zones", zones=[z], max_steps=30)
    r.pop("goal_seq", None)
    restored = [e for e in r.get("suspend_events", []) if e[0] == "restored"]
    r["restored_clear_m"] = restored[0][2] if restored else None
    print(f"leave {house:20} {r['outcome']} events {r.get('suspend_events')} "
          f"first goal {r.get('nogo')}", file=sys.stderr, flush=True)
    return r


def main(argv):
    what, out = argv[0], argv[1]
    arms = ("zones", "none")
    if "--arm" in argv:
        arms = (argv[argv.index("--arm") + 1],)
    workers = int(os.environ.get("SWEEP_WORKERS", "3"))
    t = time.time()
    if what == "find":
        jobs = [(h, s, a) for h in HOUSES for s in SEEDS for a in arms]
        rows = _pool(_find, jobs, workers)
    elif what == "survey":
        jobs = [(h, 0) for h in HOUSES[:12]]
        rows = [r for pair in _pool(_survey, jobs, workers) for r in pair]
    elif what == "leave":
        rows = _pool(_leave, HOUSES[:10], workers)
    else:
        raise SystemExit(f"unknown {what}")
    with open(out, "w") as fh:
        json.dump({"what": what, "rows": rows, "wall_s": round(time.time() - t, 1)}, fh)
    print(f"{what}: {len(rows)} rows in {time.time() - t:.0f} s -> {out}", file=sys.stderr)


if __name__ == "__main__":
    main(sys.argv[1:])
