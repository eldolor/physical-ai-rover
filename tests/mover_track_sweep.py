# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/mover_track_sweep.py

`PLAN-ros-alignment.md` 3.234.B (amendment 2): `robot/mover_tracker.py`
run offline on A's missions, through the D500's real timing.

    python -m tests.mover_track_sweep out.json          # B1 + B2 on A3's starts (seed 0)
    python -m tests.mover_track_sweep out.json --dev    # the development starts (seed 1)

**Record, then replay.** A mission runs in process exactly as A3's
(`tests.approach_sweep.mission`), and every body step leaves its time, true
pose and disc in a timeline. The timeline is then replayed through
`sim.fake_lidar.Emitter` -- point k cast at its own instant from the lidar
pose and the disc interpolated to it -- LD19-encoded and decoded by
`robot.lidar_ld19.Ld19Scanner`, each packet fed at its last point's instant.
The tracker reads each revolution's `get_scan()` with the odometry pose (the
true pose) at its `capture_unix`.

The ground truth (the disc's position and velocity, which returns are the
disc's) comes from the timeline and the cast, never from the tracker.
"""

import bisect
import math
import statistics
import sys
import time

from tests.approach_sweep import HOUSES, SPEEDS_M_S, DRIVES, mission, mission_args

CELL_M = 0.30                # sim.mock_robot.DEFAULT_CELL_M
MATCH_M = 0.4                # a moving track this close to the disc's centre is the disc's
DISC_HIT_M = 0.02            # a return this close to the disc's edge is the disc's
IN_VIEW_POINTS = 2
FIRST_TRACK_SCANS = 3        # B2's bar
LONG_VISIT_SCANS = 3         # visits shorter than this are not judged for time to first track


def record(args):
    """Run one mission; return its row and its timeline
    [(t, x, y, theta, disc or None, legs)] in cells and radians."""
    timeline = []

    def hook(robot):
        w = robot.world
        movers = w.disc_movers

        def snap():
            m = movers[0] if movers else None
            timeline.append((w.sim_time, w.x, w.y, w.theta,
                             m.disc() if m else None, sum(m.legs.values()) if m else 0))
        step, pass_time = robot.step, robot.pass_time

        def rec_step(dt):
            r = step(dt)
            snap()
            return r

        def rec_pass(dt):
            pass_time(dt)
            snap()

        robot.step, robot.pass_time = rec_step, rec_pass
        snap()

    row = mission(args, hook=hook)
    return row, timeline


def to_odom(x, y, theta):
    """Cells (y down, theta clockwise) -> the odometry frame (metres, y up, CCW)."""
    return x * CELL_M, -y * CELL_M, -theta


class Timeline:
    def __init__(self, samples):
        self.s = samples
        self.t = [p[0] for p in samples]

    def _pair(self, t):
        i = bisect.bisect_right(self.t, t)
        if i <= 0:
            return self.s[0], self.s[0], 0.0
        if i >= len(self.s):
            return self.s[-1], self.s[-1], 0.0
        a, b = self.s[i - 1], self.s[i]
        f = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        return a, b, f

    def pose(self, t):
        a, b, f = self._pair(t)
        dth = (b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi
        return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dth

    def disc(self, t):
        """(x, y, r, legs) in cells at time t, or None. Across a jump to a
        new leg the disc is held at its earlier sample (amendment 2)."""
        a, b, f = self._pair(t)
        if a[4] is None:
            return None
        if b[4] is None or b[5] != a[5]:
            return a[4] + (a[5],)
        (ax, ay, r), (bx, by, _r) = a[4], b[4]
        return ax + f * (bx - ax), ay + f * (by - ay), r, a[5]


def scans_of(house, samples):
    """The revolutions `get_scan()` gives over a recorded timeline, with
    the ground truth beside each: [{"scan", "t", "disc_pts", "disc", "v"}].
    The expensive half (every point cast), kept apart so tracker revisions
    can be replayed against the same scans (`--cache`)."""
    from robot.lidar_ld19 import RANGE_MAX_M, LIDAR_YAW_DEG, Ld19Scanner
    from robot.safety import LIDAR_X_M
    from sim import renderer
    from sim.fake_lidar import Emitter
    from sim.maps import build_world

    tl = Timeline(samples)
    world = build_world(house)
    base = world.lidar_solid_cells
    solid = renderer.SolidWithDiscs(base, ())
    max_cells = RANGE_MAX_M / CELL_M
    off = LIDAR_X_M / CELL_M
    hits = []                      # (t, is_disc) per point of the revolution in progress

    def cast_mm(angle_deg, t):
        x, y, theta = tl.pose(t)
        d = tl.disc(t)
        solid.discs = (d[:3],) if d else ()
        ox, oy = x + off * math.cos(theta), y + off * math.sin(theta)
        ray = theta + math.radians(angle_deg + LIDAR_YAW_DEG)
        dist = renderer.cast_ray_exact(world.layout, ox, oy, ray, solid=solid, max_dist=max_cells)
        if dist >= max_cells:
            hits.append((t, False))
            return 0
        is_disc = False
        if d:
            hx, hy = ox + dist * math.cos(ray), oy + dist * math.sin(ray)
            is_disc = abs(math.hypot(hx - d[0], hy - d[1]) - d[2]) * CELL_M <= DISC_HIT_M
        hits.append((t, is_disc))
        return int(round(dist * CELL_M * 1000))

    clock = [0.0]
    scanner = Ld19Scanner(clock=lambda: clock[0], wall=lambda: clock[0])
    emitter = Emitter(cast_mm, t0=samples[0][0])
    out = []
    t_end = samples[-1][0]
    t = samples[0][0]
    while t < t_end:
        t = min(t + 0.01, t_end)
        for t_pkt, pkt in emitter.due_timed(t):
            clock[0] = t_pkt
            if not scanner.feed(pkt, t_pkt):
                continue
            scan = scanner.get_scan()
            if not scan.get("usable"):
                continue
            mid = scan["capture_unix"]
            # This revolution's own span (scan_time() is older when bins
            # were carried from the revolution before).
            t_new = scanner._scan_newest
            t_old = 2 * mid - t_new
            disc_pts = sum(1 for th, d in hits if d and t_old <= th <= t_new)
            del hits[:bisect.bisect_left([h[0] for h in hits], t_old)]
            rev = {"scan": {k: scan[k] for k in ("usable", "ranges_m", "angle_min_deg",
                                                  "angle_increment_deg")},
                   "t": mid, "disc_pts": disc_pts, "disc": None, "v": None}
            d = tl.disc(mid)
            if d:
                dx, dy, _ = to_odom(d[0], d[1], 0.0)
                rev["disc"] = (dx, dy, d[3])
                a, b = tl.disc(t_old), tl.disc(t_new)
                if a and b and a[3] == b[3] == d[3] and t_new > t_old:
                    rev["v"] = ((b[0] - a[0]) * CELL_M / (t_new - t_old),
                                -(b[1] - a[1]) * CELL_M / (t_new - t_old))
            out.append(rev)
    return out


def track(scans, samples, tracker_cls=None):
    """The tracker over `scans_of()`'s revolutions: per revolution its
    movers and the ground truth, and the tracker's cost per scan (s)."""
    from robot.mover_tracker import MoverTracker
    tl = Timeline(samples)
    tracker = (tracker_cls or MoverTracker)()
    revs, cost = [], []
    for s in scans:
        c0 = time.perf_counter()
        movers = tracker.update(s["scan"], lambda tk: to_odom(*tl.pose(tk)), s["t"])
        cost.append(time.perf_counter() - c0)
        revs.append({"t": s["t"], "movers": [(m.id, m.x, m.y, m.vx, m.vy) for m in movers],
                     "disc_pts": s["disc_pts"], "disc": s["disc"], "v": s["v"]})
    return revs, cost


def replay(house, samples, tracker_cls=None):
    return track(scans_of(house, samples), samples, tracker_cls)


def driving_s(samples):
    """Sim seconds in which the body translated or turned."""
    total = 0.0
    for a, b in zip(samples, samples[1:]):
        if abs(b[1] - a[1]) + abs(b[2] - a[2]) > 1e-9 or abs(b[3] - a[3]) > 1e-9:
            total += b[0] - a[0]
    return total


def judge(revs):
    """Per mission: false moving tracks (never matched to the disc), visits
    and their time to first track, velocity errors."""
    disc_ids, all_ids = set(), set()
    errors, visits = [], []
    out_run, visit, last_leg = 2, None, None
    for rev in revs:
        disc = rev["disc"]
        matched = None
        for mid, x, y, vx, vy in rev["movers"]:
            all_ids.add(mid)
            if disc and math.hypot(x - disc[0], y - disc[1]) <= MATCH_M:
                disc_ids.add(mid)
                if matched is None or math.hypot(x - disc[0], y - disc[1]) < matched[0]:
                    matched = (math.hypot(x - disc[0], y - disc[1]), vx, vy)
        if matched and rev["v"] is not None:
            errors.append(math.hypot(matched[1] - rev["v"][0], matched[2] - rev["v"][1]))
        in_view = rev["disc_pts"] >= IN_VIEW_POINTS
        leg = disc[2] if disc else None
        if leg != last_leg:
            out_run, last_leg = 2, leg
            if visit:
                visits.append(visit)
                visit = None
        if in_view:
            if visit is None and out_run >= 2:
                visit = {"scans": 0, "first_track": None, "disc_moving_scans": 0}
            if visit is not None:
                visit["scans"] += 1
                if visit["scans"] <= FIRST_TRACK_SCANS and rev["v"] is not None \
                        and math.hypot(*rev["v"]) > 0.3:
                    visit["disc_moving_scans"] += 1
                if matched and visit["first_track"] is None:
                    visit["first_track"] = visit["scans"]
            out_run = 0
        else:
            out_run += 1
            if out_run >= 2 and visit:
                visits.append(visit)
                visit = None
    if visit:
        visits.append(visit)
    return {"false_tracks": len(all_ids - disc_ids), "errors": errors, "visits": visits}


CACHE_DIR = None             # --cache DIR: each mission's timeline and scans kept there


def recorded(args):
    """(row, samples, scans) for one mission, from `CACHE_DIR` when it is
    there (development only: a measurement runs without it)."""
    import gzip
    import hashlib
    import os
    import pickle
    path = None
    if CACHE_DIR:
        path = os.path.join(CACHE_DIR, hashlib.sha1(repr(args).encode()).hexdigest() + ".pkl.gz")
        if os.path.exists(path):
            with gzip.open(path, "rb") as f:
                return pickle.load(f)
    row, samples = record(args)
    out = (row, samples, scans_of(args[0], samples))
    if path:
        with gzip.open(path + ".tmp", "wb", compresslevel=3) as f:
            pickle.dump(out, f)
        os.replace(path + ".tmp", path)
    return out


def run_one(args, tracker_cls=None):
    row, samples, scans = recorded(args)
    revs, cost = track(scans, samples, tracker_cls)
    j = judge(revs)
    keep = {k: row[k] for k in ("house", "speed", "drive", "outcome", "sim_s")}
    keep.update(start=args[-1], driving_s=round(driving_s(samples), 2), scans=len(revs),
                false_tracks=j["false_tracks"],
                errors=[round(e, 4) for e in j["errors"]],
                visits=j["visits"], cost_ms=[round(c * 1000, 4) for c in cost])
    return keep


def p90(xs):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(math.ceil(0.9 * len(xs))) - 1)] if xs else None


def slim(row):
    """A row as recorded: its error samples as count, median and p90; its
    per-scan costs dropped (the verdict holds their summary)."""
    r = {k: v for k, v in row.items() if k not in ("errors", "cost_ms")}
    e = row["errors"]
    r.update(errors_n=len(e), error_median=round(statistics.median(e), 4) if e else None,
             error_p90=p90(e))
    return r


def verdict(b1_rows, b2_rows):
    drive_min = sum(r["driving_s"] for r in b1_rows) / 60.0
    false1 = sum(r["false_tracks"] for r in b1_rows)
    errs = [e for r in b2_rows for e in r["errors"]]
    long_visits = [v for r in b2_rows for v in r["visits"] if v["scans"] >= LONG_VISIT_SCANS]
    ftt = [v["first_track"] if v["first_track"] is not None else math.inf for v in long_visits]
    met = sum(f <= FIRST_TRACK_SCANS for f in ftt)
    cost = [c for r in b1_rows + b2_rows for c in r["cost_ms"]]
    rate = false1 / drive_min * 10 if drive_min else None
    out = {
        "B1": {"missions": len(b1_rows), "driving_min": round(drive_min, 2),
               "false_moving_tracks": false1,
               "per_10_min": round(rate, 3) if rate is not None else None,
               "met": rate is not None and rate <= 1.0},
        "B2": {"missions": len(b2_rows),
               "velocity_error_samples": len(errs),
               "velocity_error_median_m_s": round(statistics.median(errs), 4) if errs else None,
               "velocity_error_p90_m_s": round(p90(errs), 4) if errs else None,
               "visits_judged": len(long_visits),
               "visits_first_track_le_3": met,
               "first_track_share": round(met / len(long_visits), 4) if long_visits else None,
               # None: the p90 visit was never tracked (no Infinity in the JSON).
               "first_track_p90_scans": (p90(ftt) if p90(ftt) != math.inf else None)
               if ftt else None,
               "unmatched_moving_tracks": sum(r["false_tracks"] for r in b2_rows)},
        "cost_cloud_indication_ms": {"scans": len(cost),
                                     "median": round(statistics.median(cost), 4) if cost else None,
                                     "p90": round(p90(cost), 4) if cost else None,
                                     "max": round(max(cost), 4) if cost else None},
    }
    b2 = out["B2"]
    b2["velocity_met"] = bool(errs) and b2["velocity_error_median_m_s"] <= 0.15 \
        and b2["velocity_error_p90_m_s"] <= 0.35
    b2["first_track_met"] = bool(long_visits) and met == len(long_visits)
    for s in SPEEDS_M_S:
        e = [x for r in b2_rows if r["speed"] == s for x in r["errors"]]
        b2[f"velocity_error_median_at_{s}"] = round(statistics.median(e), 4) if e else None
        b2[f"velocity_error_p90_at_{s}"] = round(p90(e), 4) if e else None
    for d in DRIVES:
        e = [x for r in b2_rows if r["drive"] == d for x in r["errors"]]
        b2[f"velocity_error_p90_{d}"] = round(p90(e), 4) if e else None
    return out


def sweep_args(seed):
    import tests.approach_sweep as A
    b2 = []
    for house in HOUSES:
        for k, (x, y, h, target) in enumerate(A.starts(house, seed=seed)):
            for drive in DRIVES:
                b2 += [(house, x, y, h, target, s, drive, k) for s in SPEEDS_M_S]
    b1 = [a[:5] + (0, a[6], a[7]) for a in b2 if a[5] == SPEEDS_M_S[0]]
    return b1, b2


def main(argv):
    """out.json [--dev | --seed N] [--quick] [--procs N] [--cache DIR]. B3 on the board runs
    `--quick --procs 1`: one replay beside the stack, so the cost measured
    is the tracker's, not a harness filling every core."""
    import json
    import multiprocessing as mp
    flags = [a for a in argv if a.startswith("--")]
    procs = mp.cpu_count()
    for opt in ("--procs", "--seed", "--cache"):
        if opt in argv:
            flags.append(argv[argv.index(opt) + 1])
    if "--procs" in argv:
        procs = int(argv[argv.index("--procs") + 1])
    out_path = next(a for a in argv if a not in flags)
    seed = 1 if "--dev" in argv else 0
    if "--seed" in argv:
        seed = int(argv[argv.index("--seed") + 1])
    if "--cache" in argv:
        global CACHE_DIR
        CACHE_DIR = argv[argv.index("--cache") + 1]
    b1, b2 = sweep_args(seed)
    if "--quick" in argv:
        b1, b2 = b1[1::4], b2[1::9]
    if procs <= 1:
        rows = [run_one(a) for a in b1 + b2]
    else:
        with mp.Pool(procs) as pool:
            rows = pool.map(run_one, b1 + b2, chunksize=1)
    b1_rows, b2_rows = rows[:len(b1)], rows[len(b1):]
    v = verdict(b1_rows, b2_rows)
    v["seed"] = seed
    with open(out_path, "w") as f:
        json.dump({"verdict": v, "b1": [slim(r) for r in b1_rows],
                   "b2": [slim(r) for r in b2_rows]}, f, indent=1)
    print(json.dumps(v, indent=1))


if __name__ == "__main__":
    main(sys.argv[1:])
