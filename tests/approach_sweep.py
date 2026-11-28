# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/approach_sweep.py

The instrument and the sweep for `PLAN-ros-alignment.md` 3.234.A (A2, A3):
how close does today's veto let the robot come, through its OWN motion, to a
person or pet that walks at it or across its route?

    python -m tests.approach_sweep out.json      # the full A3 sweep (72 missions)

**A2, the instrument, is ground truth and shares no code with what it
judges** (`tests/footprint_sweep.py`'s rule): not `robot/safety.py`, not
`get_scan()`, not the renderer and not `sim.movers`' own contact rule. The
chassis is the URDF rectangle as `tests/footprint_sweep.py` writes it out;
the disc is read straight off the mover's state; the point-to-rectangle
distance is written out here.

**What "robot-caused" means** (written before the sweep ran; 3.234's
amendment 1). Every integration step of the body (`MockRobot.step`, the call
both the wheel loop and a guarded verb reach) is split in two, against the
disc where it stood when the step began:

    robot part = gap(chassis before, disc before) - gap(chassis after, disc before)
    mover part = gap(chassis after, disc before)  - gap(chassis after, disc after)

The distance judged is 3.18's and 3.30's: TRAVEL-TO-CONTACT, how far the
chassis could carry on along the way it just moved before touching the
disc, measured after a step that translated the robot and against the disc
where it stood when the step began (`T`). Not the Euclidean gap: driving
past a person beside the route closes the gap and is not an approach, as
it is not beside a wall.

An ENTRY starts at the first such step with `T` under a line and ends when
the true gap is back above the line plus `REARM_M` (or the mover's leg
ends). It is a ROBOT-CAUSED APPROACH once the robot has translated at least
`MIN_ROBOT_CLOSE_M` (1 cm, `tests/footprint_sweep.py`'s contact bar) in
steps that ended with `T` under the line: it drove a centimetre on toward a
person already inside the line. One period's lag is under that at nav2's
0.2 m/s. A pet that dashes in at a robot standing still is never counted;
nor is a pivot (3.19's guard is judged on its own metric). The robot's
closest approach is the least `T` in a counted entry (either line). Each approach
also records the robot's share of all closing since the gap was last at
least `ENCOUNTER_M` (or since the mover's leg began).

The loop under test is the whole mission path: `MissionRunner` -> the policy
-> `robot/safety.py` -> `MockRobot`. `drive: verbs` is the `frontier`
policy's guarded verbs; `drive: ros` is `explore` with nav2 stood in for by
`tests/drop_sweep.py`'s VettedNav, which follows its plan through
`vet_wheel_velocity()` then `advance()`, as nav2's twists reach the wheels.
"""

import math
import random
import statistics
import sys

from sim.maps import build_movers, build_world
from sim.mock_robot import MockRobot
from tests.footprint_sweep import CELL_CM, HALF_LENGTH, HALF_WIDTH

CELL_M = CELL_CM / 100.0
HOUSES = ("starter_house", "scaled_house", "home_first_floor", "complex_house")
STARTS = 3
SPEEDS_M_S = (0.5, 1.2, 2.5)
DRIVES = ("verbs", "ros")
LINE_M = 0.20                 # min_distance_cm: the gate's line
NEAR_M = 0.30                 # C1's second line, recorded now as its baseline
REARM_M = 0.10               # re-armed at line + this
MIN_ROBOT_CLOSE_M = 0.01
ENCOUNTER_M = 1.0
MISSION_LIMIT_S = 600.0       # sim seconds a mission may take
MAX_STEPS = 150               # the frontier policy's step budget
MAX_TICKS = 4000


def chassis_gap_m(x, y, theta, disc):
    """Metres from the chassis rectangle at (x, y, theta) to the disc's edge
    (negative: overlapping). Cells in, metres out."""
    px, py, r = disc
    ux, uy = math.cos(theta), math.sin(theta)
    dx, dy = px - x, py - y
    along, side = dx * ux + dy * uy, -dx * uy + dy * ux
    out = math.hypot(max(abs(along) - HALF_LENGTH, 0.0), max(abs(side) - HALF_WIDTH, 0.0))
    return (out - r) * CELL_M


def travel_to_contact_m(x, y, theta, sign, disc):
    """Metres the chassis rectangle at (x, y, theta) can translate along
    `sign` x its heading before touching the disc (0 if touching): the disc's
    centre cast backwards against the rectangle grown by the radius -- four
    offset sides and four corner circles, in the body frame."""
    px, py, r = disc
    ux, uy = math.cos(theta), math.sin(theta)
    dx, dy = px - x, py - y
    a, b = dx * ux + dy * uy, -dx * uy + dy * ux
    ea, eb = -sign, 0.0                       # the disc's motion relative to the body
    hl, hw = HALF_LENGTH, HALF_WIDTH
    out = math.hypot(max(abs(a) - hl, 0.0), max(abs(b) - hw, 0.0))
    if out <= r:
        return 0.0
    best = math.inf
    # The ends (a = +-(hl + r)), |b| <= hw; the sides never meet a ray along a.
    for face in (hl + r, -(hl + r)):
        t = (face - a) / ea
        if t >= 0 and abs(b + eb * t) <= hw:
            best = min(best, t)
    for ca in (hl, -hl):
        for cb in (hw, -hw):
            oa, ob = a - ca, b - cb
            bq = oa * ea + ob * eb
            c = oa * oa + ob * ob - r * r
            disc_q = bq * bq - c
            if disc_q >= 0:
                t = -bq - math.sqrt(disc_q)
                if t >= 0:
                    best = min(best, t)
    return best * CELL_M


class Instrument:
    """Wraps a `MockRobot`'s `step` and `pass_time` and keeps the A2 books."""

    def __init__(self, robot):
        self.robot, self.world = robot, robot.world
        self.mover = self.world.disc_movers[0]
        self.approaches = {LINE_M: [], NEAR_M: []}
        # Per line: the entry in progress -- the robot's closing inside it,
        # and its approach record once counted -- or None between entries.
        self._entry = {LINE_M: None, NEAR_M: None}
        self.closest_robot_m = math.inf
        self.closest_any_m = math.inf
        self.contacts = 0
        self._c_robot = self._c_mover = 0.0
        self._leg_count = self._legs()
        step, pass_time = robot.step, robot.pass_time

        def checked_step(dt):
            before = self._snap()
            result = step(dt)
            self._account(before, robot_moved=True)
            return result

        def checked_pass(dt):
            before = self._snap()
            pass_time(dt)
            self._account(before, robot_moved=False)

        robot.step, robot.pass_time = checked_step, checked_pass

    def _legs(self):
        return sum(self.mover.legs.values())

    def _snap(self):
        w = self.world
        return (w.x, w.y, w.theta), self.mover.disc()

    def _account(self, before, robot_moved):
        (x0, y0, t0), d0 = before
        w = self.world
        d1 = self.mover.disc()
        if d1 is None:
            return
        g1 = chassis_gap_m(w.x, w.y, w.theta, d1)
        self.closest_any_m = min(self.closest_any_m, g1)
        if g1 <= 0.0:
            self.contacts += 1
        legs = self._legs()
        # A new leg starts where the disc was placed: nothing to split.
        new_leg = d0 is None or legs != self._leg_count
        self._leg_count = legs
        if new_leg or g1 >= ENCOUNTER_M:
            self._c_robot = self._c_mover = 0.0
            for line in (LINE_M, NEAR_M):
                self._entry[line] = None
        if new_leg:
            return
        g0 = chassis_gap_m(x0, y0, t0, d0)
        g_r = chassis_gap_m(w.x, w.y, w.theta, d0) if robot_moved else g0
        self._c_robot += g0 - g_r
        self._c_mover += g_r - g1
        dx, dy = w.x - x0, w.y - y0
        moved = math.hypot(dx, dy)
        if robot_moved and moved > 1e-9:
            # Travel-to-contact along the way the chassis just moved, against
            # the disc where it stood: what the robot's own motion left.
            sign = 1.0 if dx * math.cos(w.theta) + dy * math.sin(w.theta) >= 0 else -1.0
            T = travel_to_contact_m(w.x, w.y, w.theta, sign, d0)
            for line in (LINE_M, NEAR_M):
                if T >= line:
                    continue
                entry = self._entry[line]
                if entry is None:
                    entry = self._entry[line] = {"travel": 0.0, "record": None}
                entry["travel"] += moved * CELL_M
                rec = entry["record"]
                if rec is None and entry["travel"] >= MIN_ROBOT_CLOSE_M:
                    total = self._c_robot + self._c_mover
                    rec = entry["record"] = {
                        "T_m": round(T, 4), "t": round(w.sim_time, 2),
                        "robot_share": round(self._c_robot / total, 3) if total > 0 else None,
                        "speed": self.mover.speed_m_s}
                    self.approaches[line].append(rec)
                if rec is not None:
                    rec["T_m"] = round(min(rec["T_m"], T), 4)
                    rec["travel_inside_m"] = round(entry["travel"], 4)
                    self.closest_robot_m = min(self.closest_robot_m, T)
        for line in (LINE_M, NEAR_M):
            if g1 > line + REARM_M:
                self._entry[line] = None


def starts(house, n=STARTS, seed=0):
    """`n` seeded (x, y, heading_deg, target): a free cell's centre with its
    neighbours free, a cardinal heading, and an object the house holds."""
    world = build_world(house)
    rng = random.Random(f"approach-{house}-{seed}")
    targets = sorted(set(world.objects.values()))

    def free(c):
        return (0 <= c[1] < len(world.layout) and 0 <= c[0] < len(world.layout[0])
                and world.layout[c[1]][c[0]] in ".D" and c not in world.objects)
    cells = [(x, y) for y, row in enumerate(world.layout) for x, _ in enumerate(row)
             if all(free((x + i, y + j)) for i in (-1, 0, 1) for j in (-1, 0, 1))]
    out = []
    while len(out) < n:
        cx, cy = rng.choice(cells)
        out.append((cx + 0.5, cy + 0.5, rng.randrange(0, 360, 90), targets[len(out) % len(targets)]))
    return out


def mission(args):
    """One mission with one disc mover; the A2 books and the outcome."""
    from control.mission_runner import MissionRunner
    from sim.mock_world import MockWorld
    house, x, y, heading, target, speed, drive, seed = args
    world = build_world(house)
    world.x, world.y, world.theta = x, y, math.radians(heading - 90)
    if speed:
        for m in build_movers(house, f"approach:{speed}:{seed}"):
            world.add_mover(m)
    robot = MockRobot(world, render=False)
    inst = Instrument(robot) if speed else None
    common = dict(world=MockWorld(world), mission_time_limit_s=MISSION_LIMIT_S,
                  clock=lambda: world.sim_time, idle=robot.pass_time, target_object=target)
    if drive == "ros":
        from tests.drop_sweep import _vetted_nav_class
        runner = MissionRunner(robot, policy="explore", navigator=_vetted_nav_class()(robot), **common)
    else:
        runner = MissionRunner(robot, policy="frontier", max_steps=MAX_STEPS, **common)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < MAX_TICKS:
        ticks += 1
    outcome = runner.status().get("outcome")
    row = {"house": house, "x": x, "y": y, "heading": heading, "target": target,
           "speed": speed, "drive": drive, "outcome": outcome, "found": outcome == "found",
           "sim_s": round(world.sim_time, 1), "ticks": ticks, "capped": ticks >= MAX_TICKS}
    if inst:
        m = inst.mover
        row.update({
            "approaches_20": inst.approaches[LINE_M], "approaches_30": inst.approaches[NEAR_M],
            "closest_robot_T_m": round(inst.closest_robot_m, 4) if inst.closest_robot_m < math.inf else None,
            "closest_any_m": round(inst.closest_any_m, 4) if inst.closest_any_m < math.inf else None,
            "contacts": inst.contacts, "legs": dict(m.legs), "mover_stops": m.stops})
    return row


def mission_args(speeds=SPEEDS_M_S, drives=DRIVES, houses=HOUSES, n=STARTS):
    out = []
    for house in houses:
        for k, (x, y, h, target) in enumerate(starts(house, n)):
            for speed in speeds:
                for drive in drives:
                    out.append((house, x, y, h, target, speed, drive, k))
    return out


def verdict(rows):
    """A3: robot-caused approaches inside 20 cm (the gate) and 30 cm, with
    the arrival rate and median mission time, per drive and overall."""
    out = {}
    for drive in DRIVES + ("all",):
        rs = [r for r in rows if drive in ("all", r["drive"])]
        if not rs:
            continue
        a20 = [a for r in rs for a in r.get("approaches_20", [])]
        out[drive] = {
            "missions": len(rs),
            "approaches_20": len(a20),
            "approaches_20_robot_share_ge_half": sum((a["robot_share"] or 0) >= 0.5 for a in a20),
            "approaches_30": sum(len(r.get("approaches_30", [])) for r in rs),
            "missions_with_20": sum(bool(r.get("approaches_20")) for r in rs),
            "contacts": sum(r.get("contacts", 0) for r in rs),
            "closest_robot_T_m": min((r["closest_robot_T_m"] for r in rs
                                      if r.get("closest_robot_T_m") is not None), default=None),
            "arrival_rate": round(sum(r["found"] for r in rs) / len(rs), 3),
            "median_sim_s": statistics.median(r["sim_s"] for r in rs),
            "capped": sum(r["capped"] for r in rs),
        }
        for s in SPEEDS_M_S:
            out[drive][f"approaches_20_at_{s}"] = sum(len(r.get("approaches_20", []))
                                                      for r in rs if r["speed"] == s)
    out["gate_open"] = out["all"]["approaches_20"] > 0
    return out


def main(argv):
    import json
    import multiprocessing as mp
    out_path = argv[0]
    args = mission_args()
    with mp.Pool(max(1, mp.cpu_count() // 2)) as pool:
        rows = pool.map(mission, args, chunksize=1)
    v = verdict(rows)
    with open(out_path, "w") as f:
        json.dump({"verdict": v, "rows": rows}, f, indent=1)
    print(json.dumps(v, indent=1))


if __name__ == "__main__":
    main(sys.argv[1:])
