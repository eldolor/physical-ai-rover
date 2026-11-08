# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/drop_sweep.py -- the instrument for 3.159 B
(`docs/plans/ros-alignment/3.159-drop-detection.md`): no fall at a stair,
judged on ground truth.

**The truth shares no code with the rule** (`tests/footprint_sweep.py`'s
way): the chassis is that module's URDF rectangle, a drop is a `v` cell
read straight off the layout, and the geometry is that module's separating
axes. Not `robot/drop_guard.py`, not the renderer's `cast_ray_drop()`.
The loop under test is the wheel loop's two calls, `vet_wheel_velocity()`
then `advance()`, every 0.05 s.

* a **fall**: the chassis centre over a `v` cell;
* an **overhang**: any part of the chassis rectangle over one (cm deep);
* the **edge gap**: the distance from the outline to the nearest one.
"""

import math
import random
from collections import deque

from robot import drop_guard
from robot.safety import SafetyController
from sim.grid_world import CELL_WALL, GridWorld
from sim.mock_robot import WHEEL_RADIUS_M, MockRobot
from tests.footprint_sweep import CELL_CM, HALF_LENGTH, HALF_WIDTH, PERIOD_S, chassis, gap, penetration, square

DROP = "v"
SEED_CM = 30.0            # robot/drop_guard.py's SEED_RADIUS_CM: the dock assumption
SPEEDS = (0.1, 0.2, 0.4, 0.6)
NOISE = {"noise_stddev_cm": 3.0, "dropout_rate": 0.2}


def drops_near(world, x, y, reach_cells=3.0):
    r = int(math.ceil(reach_cells)) + 1
    return [square(cx, cy) for cy in range(int(y) - r, int(y) + r + 1)
            for cx in range(int(x) - r, int(x) + r + 1)
            if 0 <= cy < len(world.layout) and 0 <= cx < len(world.layout[0])
            and world.layout[cy][cx] == DROP]


def drop_truth(world):
    """(fell, overhang_cm, gap_cm) at the world's pose. The gap is capped at
    the search reach (3 cells, 90 cm)."""
    cx, cy = math.floor(world.x), math.floor(world.y)
    fell = world.layout[cy][cx] == DROP
    A = chassis(world.x, world.y, world.theta)
    over, g = 0.0, 3.0
    for B in drops_near(world, world.x, world.y):
        d = gap(A, B)
        g = min(g, d)
        if d == 0.0:
            over = max(over, penetration(A, B))
    return fell, over * CELL_CM, g * CELL_CM


def centre_to_drop_cm(world, x, y):
    def pt(B):
        (x0, y0), (x1, y1) = B[0], B[2]
        return math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))
    return min((pt(B) for B in drops_near(world, x, y, 4.0)), default=math.inf) * CELL_CM


def world_of(layout, x, y, theta):
    world = GridWorld(layout, {"room": set()})
    world.x, world.y, world.theta = x, y, theta
    return world


def drive(world, speed_m_s, run_s, sensor=None, rule=True, direction=+1, robot=None):
    """A standing straight command through the wheel loop. Returns the run's
    truth: fell, the deepest overhang, the gap at rest, the travel."""
    robot = robot or MockRobot(world, render=False, sensor=sensor, drop_sensor=True)
    safety = SafetyController(robot, 20.0)
    if not rule:
        safety.drop_clearance = lambda *a, **k: (None, "no_drop_sensor")
        safety.drop_pivot_blocked = lambda *a, **k: None
    w = direction * speed_m_s / WHEEL_RADIUS_M
    x0, y0 = world.x, world.y
    fell, over, reasons = False, 0.0, set()
    for _ in range(int(run_s / PERIOD_S)):
        left, right, reason = safety.vet_wheel_velocity(w, w)
        if reason:
            reasons.add(reason.split("(")[-1].rstrip(")"))
        robot.set_wheel_velocity(left, right)
        before = (world.x, world.y)
        robot.advance(PERIOD_S)
        f, o, _ = drop_truth(world)
        fell, over = fell or f, max(over, o)
        if f:
            break
        if math.hypot(world.x - before[0], world.y - before[1]) <= 1e-9 and sensor is None:
            break      # exact sensor, static house, same command: every later period is this one
    _, _, g = drop_truth(world)
    return {"fell": fell, "overhang": round(over, 2), "gap": round(g, 2),
            "travel": round(math.hypot(world.x - x0, world.y - y0) * CELL_CM, 2),
            "reasons": sorted(reasons)}


# ---------------- the approach sweep (criteria D1-D3) ----------------

ROOM = 24
KINDS = ("edge", "corner", "opening")
AHEAD_CM = (30.0, 150.0)
DEVIATION_DEG = 70.0


def layout_of(kind):
    """A 24-cell room with, from row 14 down: a straight edge across it, the
    outside corner of a drop (x >= 12), or a 0.9 m stairwell opening in a
    wall (x 11-13)."""
    rows = []
    for y in range(ROOM):
        row = []
        for x in range(ROOM):
            if x in (0, ROOM - 1) or y in (0, ROOM - 1):
                row.append("#")
            elif y < 14:
                row.append(".")
            elif kind == "edge" or (kind == "corner" and x >= 12):
                row.append(DROP)
            elif kind == "opening":
                row.append(DROP if 11 <= x <= 13 else "#")
            else:
                row.append(".")
        rows.append("".join(row))
    return rows


TARGET = {"edge": None, "corner": (12.0, 14.0), "opening": (12.5, 14.0)}


def _contact_cm(world, squares, theta):
    from tests.footprint_sweep import travel_to_contact
    A = chassis(world.x, world.y, theta)
    d = (math.cos(theta), math.sin(theta))
    return min((travel_to_contact(A, B, d) for B in squares), default=math.inf) * CELL_CM


def approach_placements(n, seed=0):
    """`n` seeded (kind, x, y, theta): heading within DEVIATION_DEG of the
    way to the drop, the chassis' path meeting a drop 30-150 cm on and
    before any wall, and the centre at least SEED_CM from every drop."""
    rng = random.Random(f"drop-approach-{seed}")
    out = []
    while len(out) < n:
        kind = KINDS[len(out) % len(KINDS)]
        layout = layout_of(kind)
        x, y = rng.uniform(3, ROOM - 3), rng.uniform(4, 13.5)
        tx, ty = TARGET[kind] or (x, 14.0)
        theta = math.atan2(ty - y, tx - x) + math.radians(rng.uniform(-DEVIATION_DEG, DEVIATION_DEG))
        world = world_of(layout, x, y, theta)
        if centre_to_drop_cm(world, x, y) < SEED_CM + 0.5:
            continue
        drops = [square(cx, cy) for cy in range(ROOM) for cx in range(ROOM) if layout[cy][cx] == DROP]
        walls = [square(cx, cy) for cy in range(1, ROOM - 1) for cx in range(1, ROOM - 1)
                 if layout[cy][cx] == "#"] + [square(cx, cy) for cy in range(ROOM) for cx in range(ROOM)
                                             if cx in (0, ROOM - 1) or cy in (0, ROOM - 1)]
        t_drop = _contact_cm(world, drops, theta)
        if not (AHEAD_CM[0] <= t_drop <= AHEAD_CM[1]) or _contact_cm(world, walls, theta) <= t_drop:
            continue
        out.append((kind, x, y, theta, round(t_drop, 2)))
    return out


def approach_run(args):
    kind, x, y, theta, t0, speed, noise, rule = args
    world = world_of(layout_of(kind), x, y, theta)
    sensor = None
    if noise:
        from sim.sensors import DistanceSensorModel
        sensor = DistanceSensorModel(rng=random.Random(f"{kind}-{x}-{y}-{theta}"), **NOISE)
    run_s = (t0 + 40.0) / 100.0 / speed + 1.0
    out = drive(world, speed, run_s, sensor=sensor, rule=rule)
    out.update(kind=kind, x=x, y=y, theta=theta, T0=t0, speed=speed, noise=noise, rule=rule)
    return out


def approach_verdict(rows):
    out = {}
    for noise in (False, True):
        rs = [r for r in rows if r["noise"] == noise and r["rule"]]
        if not rs:
            continue
        by_speed = {}
        for s in sorted({r["speed"] for r in rs}):
            g = [r["gap"] for r in rs if r["speed"] == s]
            by_speed[s] = {"runs": len(g), "under_10": sum(v < 10 for v in g), "under_5": sum(v < 5 for v in g),
                           "share_ge_15": round(sum(v >= 15 for v in g) / len(g), 4), "min_gap": min(g)}
        over = [r["overhang"] for r in rs if r["overhang"] > 0]
        out["noise" if noise else "exact"] = {
            "runs": len(rs), "falls": sum(r["fell"] for r in rs),
            "overhang_runs": len(over), "overhang_share": round(len(over) / len(rs), 4),
            "overhang_max_cm": max(over, default=0.0), "by_speed": by_speed}
    return out


# ---------------- turns and astern (criterion D4) ----------------

def _push_to(world, robot, safety, gap_cm, max_s=20.0):
    """Drive straight with the veto bypassed, the memory updated every
    period (amendment 1.5), until the true gap is at most `gap_cm`."""
    w = 0.1 / WHEEL_RADIUS_M
    for _ in range(int(max_s / PERIOD_S)):
        safety.forward_clearance()
        if drop_truth(world)[2] <= gap_cm:
            return True
        robot.set_wheel_velocity(w, w)
        robot.advance(PERIOD_S)
    return False


def _gap_after_turn(world, deg):
    t = world.theta
    world.theta = t + math.radians(deg)
    g = drop_truth(world)
    world.theta = t
    return g


def turn_run(args):
    """One pivot from an outline 1-15 cm from an edge, `sign` +1 a right
    (clockwise on the grid) turn, -1 a left one."""
    kind, x, y, theta, t0, gap_cm, sign = args
    world = world_of(layout_of(kind), x, y, theta)
    robot = MockRobot(world, render=False, drop_sensor=True)
    safety = SafetyController(robot, 20.0)
    if not _push_to(world, robot, safety, gap_cm):
        return {"skipped": "never reached the gap"}
    _, over0, g0 = drop_truth(world)
    _, o5, g5 = _gap_after_turn(world, sign * 5.0)
    away = o5 == 0.0 and g5 >= g0 - 1e-6
    # The grid's theta grows clockwise; the wheels' CCW is a left turn.
    w = 1.0 * 0.172 / 2 / WHEEL_RADIUS_M
    left_w, right_w = (w, -w) if sign > 0 else (-w, w)
    fell, over, turned, refused_first = False, over0, 0.0, None
    for k in range(int(3.0 / PERIOD_S)):
        left, right, reason = safety.vet_wheel_velocity(left_w, right_w)
        if k == 0:
            refused_first = (left == right == 0.0 and "drop_pivot" in str(reason))
        robot.set_wheel_velocity(left, right)
        th = world.theta
        robot.advance(PERIOD_S)
        turned += abs((world.theta - th + math.pi) % (2 * math.pi) - math.pi)
        f, o, _ = drop_truth(world)
        fell, over = fell or f, max(over, o)
    return {"kind": kind, "gap0": round(g0, 2), "sign": sign, "away": away, "refused_first": refused_first,
            "fell": fell, "overhang": round(over, 2), "overhang0": round(over0, 2),
            "turned_deg": round(math.degrees(turned), 1)}


def turn_args(n):
    """The turn sweep's runs, in order: each placement at a random gap, both ways."""
    rng = random.Random("drop-turn")
    return [p + (round(rng.uniform(1.0, 15.0), 2), sign)
            for p in approach_placements(n, seed=1) for sign in (+1, -1)]


def astern_run(args):
    """`seen`: approach the edge with the rule, turn about, drive 40 cm away,
    then reverse toward it over the floor just driven. Not `seen`: placed
    facing away from an edge it never saw, then reverse."""
    kind, x, y, theta, t0, seen = args
    world = world_of(layout_of(kind), x, y, theta if seen else theta + math.pi)
    robot = MockRobot(world, render=False, drop_sensor=True)
    out = {"kind": kind, "seen": seen}
    if seen:
        drive(world, 0.2, (t0 + 40) / 20.0 + 1.0, robot=robot)
        safety = SafetyController(robot, 20.0)
        w = 1.0 * 0.172 / 2 / WHEEL_RADIUS_M
        th0 = world.theta
        for _ in range(int(4.0 / PERIOD_S)):
            left, right, _r = safety.vet_wheel_velocity(w, -w)
            robot.set_wheel_velocity(left, right)
            robot.advance(PERIOD_S)
            if abs((world.theta - th0 + math.pi) % (2 * math.pi) - math.pi) >= math.radians(178):
                break
        out["about"] = round(math.degrees(abs((world.theta - th0 + math.pi) % (2 * math.pi) - math.pi)), 1)
        away = drive(world, 0.2, 2.0, robot=robot)
        out["away_cm"] = away["travel"]
    back = drive(world, 0.2, 10.0, robot=robot, direction=-1)
    out.update(fell=back["fell"], overhang=back["overhang"], gap=back["gap"], back_cm=back["travel"],
               reasons=back["reasons"])
    return out


# ---------------- flat floor (criterion D5) ----------------

HOUSES = ("starter_house", "scaled_house", "home_first_floor", "complex_house")


def flat_run(args):
    """3.74 B2a's forward run (`tests/floor_band_sweep.py` `progress_run`)
    with the drop block on, and the drop rule on or off."""
    from sim.maps import build_world
    from sim.sensors import DistanceSensorModel
    from tests.footprint_sweep import truth
    house, x, y, heading_deg, speed, rule, noise = args
    world = build_world(house)
    world.x, world.y, world.theta = x, y, math.radians(heading_deg)
    sensor = (DistanceSensorModel(rng=random.Random(f"{house}-{x}-{y}-{heading_deg}-0"), **NOISE)
              if noise else None)
    robot = MockRobot(world, render=False, sensor=sensor, drop_sensor=True)
    safety = SafetyController(robot, 20.0)
    if not rule:
        safety.drop_clearance = lambda *a, **k: (None, "no_drop_sensor")
        safety.drop_pivot_blocked = lambda *a, **k: None
    w = speed / WHEEL_RADIUS_M
    T0 = truth(world)[0]
    x0, y0, bound = world.x, world.y, 0
    for _ in range(int(3.0 / PERIOD_S)):
        left, right, reason = safety.vet_wheel_velocity(w, w)
        bound += "drop" in str(reason)
        robot.set_wheel_velocity(left, right)
        before = (world.x, world.y)
        robot.advance(PERIOD_S)
        if math.hypot(world.x - before[0], world.y - before[1]) <= 1e-9 and not noise:
            break
    return {"house": house, "x": x, "y": y, "heading": heading_deg, "speed": speed, "rule": rule,
            "noise": noise, "T0": T0, "travel": math.hypot(world.x - x0, world.y - y0) * CELL_CM,
            "drop_bound": bound}


def flat_args(starts_per_house=10, seed=0):
    from tests.footprint_sweep import HEADINGS, starts
    return [(house, x, y, h * 15, s, rule, noise)
            for house in HOUSES for x, y in starts(house, starts_per_house, seed)
            for h in range(HEADINGS) for s in (0.1, 0.2)
            for noise in (False, True) for rule in (True, False)]


def flat_verdict(rows):
    from tests.footprint_sweep import PROGRESS_MIN_CM, PROGRESS_START_CM
    out = {}
    for noise in (False, True):
        on = [r for r in rows if r["noise"] == noise and r["rule"]]
        off = {(r["house"], r["x"], r["y"], r["heading"], r["speed"]): r
               for r in rows if r["noise"] == noise and not r["rule"]}

        def share(rs):
            el = [r for r in rs if r["T0"] >= PROGRESS_START_CM]
            return sum(r["travel"] >= PROGRESS_MIN_CM for r in el) / len(el) if el else 1.0
        early = [r for r in on if off[(r["house"], r["x"], r["y"], r["heading"], r["speed"])]["travel"]
                 - r["travel"] > 3.0]
        out["noise" if noise else "exact"] = {
            "runs": len(on), "early_by_3cm": len(early), "early_share": round(len(early) / len(on), 4),
            "progress_on": round(share(on), 4), "progress_off": round(share(list(off.values())), 4),
            "drop_bound_runs": sum(r["drop_bound"] > 0 for r in on)}
    return out


# ---------------- missions (criterion D6, amendment 2) ----------------

STAIRS_HOUSES = ("scaled_house_stairs", "home_first_floor_stairs")
MISSION_LIMIT_S = 1200.0         # mission_time_limit_s's default (3.70)


def _vetted_nav_class():
    from tests.test_explore import WalkingNav

    class VettedNav(WalkingNav):
        """nav2 in miniature that DRIVES through the veto: it plans as
        WalkingNav does over the true floor -- a drop is open floor to it, as
        it is to nav2 without the mark (amendment 1.6) -- and follows the path
        with the wheel loop's two calls, `vet_wheel_velocity()` then
        `advance()`, as nav2's twists reach the wheels. One poll is 1 s;
        a goal is aborted after 3 polls with no motion (the veto held it) or
        120 polls in all.

        3.159.B2 (amendment 3), with `marks_on`: at a drop refusal the
        robot's remembered lips become discs (`DropMarks`, the book the held
        mark used, `evaluations/drop-3159/mark-held.patch`) at the true pose,
        and the plan avoids every cell whose centre a disc covers, as nav2's
        keepout does. `marked_vetoes` counts the drop refusal episodes that
        begin with a disc within MARKED_NEAR_M. Off by default: the mark is
        held back (3.159.B3 missed its bar too; the robot server does not mark)."""

        MARKED_NEAR_M = 0.80

        def __init__(self, robot, marks_on=False):
            super().__init__(robot)
            self.safety = SafetyController(robot, 20.0)
            self.stuck = self.polls = 0
            self.vetoed = 0
            self.marks = drop_guard.DropMarks()
            self.marks_on = marks_on
            self.marked_vetoes = 0
            self.in_veto = False
            self.covered = set()

        def _mark(self, grow=False):
            """As the robot server's `mark_drop`, at the true pose. `grow`:
            only the lips a disc already covers (amendment 4)."""
            g = self.grid
            lips = self.safety._drop_guard().lips()
            heading = math.degrees(g.theta) + 90.0
            points = [drop_guard.to_house(g.x * 0.3, g.y * 0.3, heading, a, l) for a, l in lips]
            if grow:
                points = self.marks.on_marked(points)
            if self.marks.add(0.0, points, at=(g.x * 0.3, g.y * 0.3)):
                spots = self.marks.spots(0.0)
                h, w = len(g.layout), len(g.layout[0])
                self.covered = {(i, j) for j in range(h) for i in range(w)
                                if any(math.hypot((i + 0.5) * 0.3 - s["x_m"], (j + 0.5) * 0.3 - s["y_m"])
                                       <= s["r_m"] for s in spots)}

        def _covered(self, cell):
            return cell in self.covered

        def _path(self, a, b):
            if not (self.marks_on and self.marks.marks):
                return super()._path(a, b)
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
                    if (n not in prev and g._cell(*n) != CELL_WALL and n not in g.objects
                            and not self._covered(n)):
                        prev[n] = c
                        q.append(n)
            return None

        def set_goal(self, x_m, y_m):
            self.stuck = self.polls = 0
            self.in_veto = False
            return super().set_goal(x_m, y_m)

        def get_goal(self):
            self.calls.append(("get",))
            if not (self.goal and self.goal["state"] == "active"):
                return {"goal": self.goal, "plan": []}
            g = self.grid
            to = (math.floor(self.goal["x_m"] / 0.3), math.floor(self.goal["y_m"] / 0.3))
            # A goal inside a disc is lethal to nav2's planner: no path.
            path = (None if to in g.objects or self._covered(to)
                    else self._path((g.robot_x, g.robot_y), to))
            if path is None:
                self.robot.pass_time(1.0)
                self.goal["state"] = "aborted"
                return {"goal": self.goal, "plan": []}
            x0, y0, th0 = g.x, g.y, g.theta
            marked_this_poll = False
            tx, ty = self.goal["x_m"] / 0.3, self.goal["y_m"] / 0.3
            for _ in range(int(1.0 / PERIOD_S)):
                if math.hypot(tx - g.x, ty - g.y) <= 0.4:
                    self.robot.set_wheel_velocity(0.0, 0.0)
                    self.goal["state"] = "succeeded"
                    return {"goal": self.goal, "plan": []}
                path = self._path((g.robot_x, g.robot_y), to) or [to]
                wx, wy = path[min(2, len(path) - 1)]
                wx, wy = (tx, ty) if len(path) <= 2 else (wx + 0.5, wy + 0.5)
                err = (math.atan2(wy - g.y, wx - g.x) - g.theta + math.pi) % (2 * math.pi) - math.pi
                # The grid's theta grows clockwise; the wheels' omega is CCW.
                if abs(err) > math.radians(30):
                    v, omega = 0.0, -math.copysign(1.0, err)
                else:
                    v, omega = 0.2, -2.0 * err
                half = omega * 0.172 / 2.0
                left, right, reason = self.safety.vet_wheel_velocity((v - half) / WHEEL_RADIUS_M,
                                                                     (v + half) / WHEEL_RADIUS_M)
                self.vetoed += "drop" in str(reason)
                edge = "drop_edge" in str(reason) or "drop_pivot_edge" in str(reason)
                if edge and not self.in_veto:
                    near = any(math.hypot(g.x * 0.3 - s["x_m"], g.y * 0.3 - s["y_m"])
                               <= self.MARKED_NEAR_M for s in self.marks.spots(0.0))
                    self.marked_vetoes += near
                if edge and self.marks_on and not marked_this_poll:
                    self._mark()              # at most once a poll (1 s), as the server
                    marked_this_poll = True
                self.in_veto = edge
                self.robot.set_wheel_velocity(left, right)
                self.robot.advance(PERIOD_S)
            self.robot.set_wheel_velocity(0.0, 0.0)
            if self.marks_on and self.marks.marks:
                self._mark(grow=True)         # once a poll (1 s), as the server
            self.polls += 1
            moved = math.hypot(g.x - x0, g.y - y0) * 30.0 + abs(g.theta - th0) * 10.0
            self.stuck = self.stuck + 1 if moved < 1.0 else 0
            if self.stuck >= 3 or self.polls >= 120:
                self.goal["state"] = "aborted"
            return {"goal": self.goal, "plan": []}

    return VettedNav


def mission_starts(house, n, seed=0):
    """`n` seeded (x, y, heading, target): a free floor cell's centre at
    least SEED_CM (plus a margin) from every drop (amendment 1.7), and a
    target the house holds, in turn."""
    from sim.maps import build_world
    world = build_world(house)
    rng = random.Random(f"drop-missions-{house}-{seed}")
    targets = sorted(set(world.objects.values()))
    free = [(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
            if c == "." and (x, y) not in world.objects]
    out = []
    while len(out) < n:
        cx, cy = rng.choice(free)
        x, y = cx + 0.5, cy + 0.5
        if centre_to_drop_cm(world, x, y) < SEED_CM + 15.0:
            continue
        out.append((x, y, rng.randrange(0, 360, 90), targets[len(out) % len(targets)]))
    return out


def mission_run(args):
    """One explore mission (`job` find or survey) through `MissionRunner`,
    nav2 stood in for by VettedNav, every motion through the safety layer;
    the truth checked after every integration step. `walled`: the same
    house with its stairwell a wall. An eighth field, True, turns the held
    mark on in the harness (`missions-marked`)."""
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_world import MockWorld
    house, x, y, heading, target, job, walled = args[:7]
    marks_on = len(args) > 7 and args[7]
    world = build_world(house)
    if walled:
        world.layout = [row.replace(DROP, "#") for row in world.layout]
    world.x, world.y = x, y
    world.theta = math.radians(heading - 90)
    robot = MockRobot(world, render=False, drop_sensor=True)
    state = {"fell": False, "overhang": 0.0, "rooms": set()}
    step = robot.step

    def checked(dt):
        result = step(dt)
        state["rooms"].add(world.room_at(math.floor(world.x), math.floor(world.y)))
        if not walled and not state["fell"]:
            f, o, _ = drop_truth(world)
            state["fell"] = f
            state["overhang"] = max(state["overhang"], o)
        return result
    robot.step = checked
    nav = _vetted_nav_class()(robot, marks_on=marks_on)
    kw = {"job": "survey"} if job == "survey" else {"target_object": target}
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=MockWorld(world),
                           mission_time_limit_s=MISSION_LIMIT_S, clock=lambda: world.sim_time,
                           idle=robot.pass_time, **kw)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < 5000 and not state["fell"]:
        ticks += 1
    status = runner.status()
    return {"house": house, "x": x, "y": y, "heading": heading, "target": target, "job": job,
            "walled": walled, "outcome": status.get("outcome"),
            "found": status.get("outcome") == "found",
            "rooms": len(state["rooms"] - {"unknown"}), "fell": state["fell"],
            "overhang": round(state["overhang"], 2), "nav_drop_vetoes": nav.vetoed, "marked_vetoes": nav.marked_vetoes,
            "marks": len(nav.marks.marks),
            "sim_s": round(world.sim_time, 1), "ticks": ticks, "capped": ticks >= 5000}


def mission_verdict(rows):
    out = {}
    for house in STAIRS_HOUSES:
        for job in ("find", "survey"):
            for walled in (False, True):
                rs = [r for r in rows if r["house"] == house and r["job"] == job and r["walled"] == walled]
                if rs:
                    out[f"{house} {job}{' walled' if walled else ''}"] = {
                        "runs": len(rs), "falls": sum(r["fell"] for r in rs),
                        "overhang_runs": sum(r["overhang"] > 0 for r in rs),
                        "found_share": round(sum(r["found"] for r in rs) / len(rs), 3),
                        "rooms_mean": round(sum(r["rooms"] for r in rs) / len(rs), 2),
                        "nav_drop_vetoes": sum(r["nav_drop_vetoes"] for r in rs),
                        "marked_vetoes_max": max(r.get("marked_vetoes", 0) for r in rs),
                        "missions_over_1_marked_veto": sum(r.get("marked_vetoes", 0) > 1 for r in rs),
                        "marks_mean": round(sum(r.get("marks", 0) for r in rs) / len(rs), 1)}
    return out


def main(argv):
    import json
    import multiprocessing as mp
    what, out_path = argv[0], argv[1]
    n = int(argv[2]) if len(argv) > 2 else 300
    if what == "approach":
        places = approach_placements(n)
        args = [p + (s, noise, True) for p in places for s in SPEEDS for noise in (False, True)]
        verdict = approach_verdict
        fn = approach_run
    elif what == "turn":
        args = turn_args(n)
        fn, verdict = turn_run, None
    elif what == "astern":
        places = approach_placements(n, seed=2)
        args = [p + (seen,) for p in places for seen in (True, False)]
        fn, verdict = astern_run, None
    elif what in ("missions", "missions-marked"):
        args = [(house,) + st + (job, walled, what == "missions-marked")
                for house in STAIRS_HOUSES
                for st in mission_starts(house, n) for job in ("find", "survey")
                for walled in (False, True)]
        fn, verdict = mission_run, mission_verdict
    elif what == "flat":
        args = flat_args(n)
        fn, verdict = flat_run, flat_verdict
    else:
        raise SystemExit(f"unknown sweep {what!r}")
    with mp.Pool(int(argv[3]) if len(argv) > 3 else 6) as pool:
        rows = pool.map(fn, args, chunksize=4)
    summary = verdict(rows) if verdict else None
    with open(out_path, "w") as f:
        json.dump({"what": what, "n": n, "summary": summary, "rows": rows}, f)
    print(json.dumps(summary, indent=1) if summary else f"{len(rows)} rows -> {out_path}")


if __name__ == "__main__":
    import sys
    main(sys.argv[1:])
