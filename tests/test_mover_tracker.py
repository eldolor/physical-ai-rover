# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""`robot/mover_tracker.py` (PLAN-ros-alignment.md 3.234.B) on synthetic
scans, and B's recorded numbers pinned (`evaluations/mover-prediction-3234/
b-sweep.json`)."""

import json
import math
import os

import pytest

from robot import mover_tracker as mt
from robot.mover_tracker import MoverTracker, clusters, scan_points

ROOM_R = 4.0


def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return None
    s = math.sqrt(disc)
    for t in (-b - s, -b + s):
        if t > 1e-9:
            return t
    return None


def scan(discs=(), pose=(0.0, 0.0, 0.0), room_r=ROOM_R, walls=()):
    """A get_scan() dict seen from `pose` (odometry frame): a round room of
    radius `room_r` about the odometry origin, discs (x, y, r) in the
    odometry frame, and straight walls ((x0, y0), (x1, y1))."""
    px, py, yaw = pose
    ox, oy = px + mt.LIDAR_X_M * math.cos(yaw), py + mt.LIDAR_X_M * math.sin(yaw)
    ranges = []
    for i in range(360):
        b = math.radians(-180.0 + i)               # body bearing, clockwise positive
        a = yaw - b
        dx, dy = math.cos(a), math.sin(a)
        best = _ray_circle(ox, oy, dx, dy, 0.0, 0.0, room_r)
        for cx, cy, r in discs:
            t = _ray_circle(ox, oy, dx, dy, cx, cy, r)
            if t is not None and t < best:
                best = t
        for (x0, y0), (x1, y1) in walls:
            ex, ey = x1 - x0, y1 - y0
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x0 - ox) * ey - (y0 - oy) * ex) / den
            u = ((x0 - ox) * dy - (y0 - oy) * dx) / den
            if t > 1e-9 and 0.0 <= u <= 1.0 and t < best:
                best = t
        ranges.append(round(best, 4))
    return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
            "ranges_m": ranges}


PREROLL = 12                # scans of the empty room first: the free-space test's history


def run(tracker, frames, dt=0.1, preroll=PREROLL, static=()):
    """frames: [(discs, pose)] after `preroll` scans of the room and the
    `static` discs from the first frame's pose; returns the movers after
    each of `frames`."""
    out = []
    for k in range(preroll):
        tracker.update(scan(static, frames[0][1]), frames[0][1], k * dt)
    for k, (discs, pose) in enumerate(frames):
        out.append(tracker.update(scan(discs, pose), pose, (preroll + k) * dt))
    return out


def test_a_still_scene_has_no_mover():
    tr = MoverTracker()
    still = [(1.5, 0.5, 0.15), (-1.0, -1.2, 0.15)]
    assert all(m == [] for m in run(tr, [(still, (0.0, 0.0, 0.0))] * 30, static=still))


def test_a_still_scene_from_a_moving_robot_has_no_mover():
    tr = MoverTracker()
    frames = [([(2.0, 0.6, 0.15)], (0.02 * k, 0.0, 0.01 * k)) for k in range(30)]
    assert all(m == [] for m in run(tr, frames, static=[(2.0, 0.6, 0.15)]))


def test_a_crossing_disc_is_tracked_within_three_scans_with_its_velocity():
    tr = MoverTracker(revolution_s=0.0)        # synthetic scans are instantaneous
    v = (0.0, 1.2)
    frames = [([(1.5, -1.5 + v[1] * 0.1 * k, 0.15)], (0.0, 0.0, 0.0)) for k in range(20)]
    out = run(tr, frames)
    first = next(k for k, m in enumerate(out) if m)
    assert first <= 1                           # the second scan (3.234.B2's early tier)
    for m in out[first:]:
        assert len(m) == 1
        assert math.hypot(m[0].vx - v[0], m[0].vy - v[1]) < 0.1
        assert m[0].moving


def test_a_disc_that_stops_is_not_moving_within_two_scans():
    tr = MoverTracker(revolution_s=0.0)
    ys = [-1.5 + 0.25 * k for k in range(8)] + [0.25] * 6
    out = run(tr, [([(1.5, y, 0.15)], (0.0, 0.0, 0.0)) for y in ys])
    assert out[7]                               # moving at 2.5 m/s
    assert out[9] == [] and all(m == [] for m in out[9:])


def test_a_wall_is_not_a_cluster_and_a_hidden_edge_is_refused():
    pts = scan_points(scan(walls=[((1.0, -1.0), (1.0, 1.0))])["ranges_m"])
    assert clusters(pts) == []
    # A disc half behind a nearer post: its edge is hidden, so no cluster for it.
    s = scan(discs=[(0.6, 0.0, 0.05), (2.0, 0.1, 0.15)])
    found = clusters(scan_points(s["ranges_m"]))
    assert len(found) == 1 and math.hypot(found[0][0] - 0.6, found[0][1]) < 0.1


def test_a_room_open_only_at_a_door_is_not_a_cluster():
    # The wall ring is broken only at a 30 cm door: end to end it is
    # door-wide, along its returns it is the room.
    s = scan(room_r=2.0)
    for i in range(360):
        b = math.radians(-180.0 + i)
        if abs(math.degrees(math.atan2(-math.sin(b) * 2.0, mt.LIDAR_X_M + 2.0 * math.cos(b)))) < 4.3:
            s["ranges_m"][i] = 3.5                   # through the door
    assert clusters(scan_points(s["ranges_m"])) == []
    old, mt.MAX_PATH_M = mt.MAX_PATH_M, 99.0
    try:                                             # the path test is what refuses it
        assert len(clusters(scan_points(s["ranges_m"]))) == 1
    finally:
        mt.MAX_PATH_M = old


def test_a_cluster_never_spans_the_revolution_seam():
    # Body bearing -90 (yaw -90) is lidar angle 0: the bins either side
    # were taken a revolution apart, so a disc there is two clusters.
    s = scan(discs=[(mt.LIDAR_X_M, 1.0, 0.15)])     # on the left: bearing -90
    pts = scan_points(s["ranges_m"])
    assert len(clusters(pts)) == 1
    assert len(clusters(pts, seam=90)) == 2


def test_the_centre_is_estimated_not_the_near_face():
    for d in (0.5, 1.0, 3.0):
        (x, y, *_rest), = clusters(scan_points(scan(discs=[(d, 0.0, 0.15)])["ranges_m"]))
        # Within 6 cm: at 3 m the 1-degree bins miss the disc's edges, so its
        # width reads short (the centroid of the returns was 10 cm short).
        assert abs(x - d) < 0.06 and abs(y) < 0.02


def test_a_static_object_revealed_by_a_moving_robot_is_not_a_mover():
    # A post hidden behind a nearer one comes into view as the robot moves
    # sideways; its first sightings were never free space.
    tr = MoverTracker(revolution_s=0.0)
    posts = [(1.0, 0.0, 0.05), (2.5, 0.0, 0.15)]
    frames = [(posts, (0.0, 0.05 * k, 0.0)) for k in range(30)]
    assert all(m == [] for m in run(tr, frames, static=posts))


def test_without_history_nothing_is_moving_on_the_first_two_scans():
    # The free-space test asks scans 0.2-1 s old: none on the first two.
    # (On the third the observation is free against the first scan, and
    # the fit has three observations: moving, as it should be.)
    tr = MoverTracker(revolution_s=0.0)
    out = run(tr, [([(1.5, -1.5 + 0.12 * k, 0.15)], (0.0, 0.0, 0.0)) for k in range(4)],
              preroll=0)
    assert out[0] == [] and out[1] == []


def _turning(rate_dps, x0=0.0):
    """A pose function: turning in place at `rate_dps` about (x0, 0)."""
    return lambda t: (x0, 0.0, math.radians(rate_dps) * t)


def scan_timed(discs_at, pose, t, rev_s=0.1):
    """A revolution as the lidar takes it: each bin cast from the pose and
    the discs at the instant its beam swept it (lidar angle 0 at
    `t - rev_s / 2`), as `MoverTracker` stamps it."""
    ranges = []
    for i in range(360):
        b = -180.0 + i
        tb = t + (((b - mt.LIDAR_YAW_DEG) % 360.0) / 360.0 - 0.5) * rev_s
        px, py, yaw = pose(tb)
        ox, oy = px + mt.LIDAR_X_M * math.cos(yaw), py + mt.LIDAR_X_M * math.sin(yaw)
        a = yaw - math.radians(b)
        dx, dy = math.cos(a), math.sin(a)
        hits = [_ray_circle(ox, oy, dx, dy, 0.0, 0.0, ROOM_R)]
        hits += [_ray_circle(ox, oy, dx, dy, cx, cy, r) for cx, cy, r in discs_at(tb)]
        ranges.append(round(min(h for h in hits if h is not None), 4))
    return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
            "ranges_m": ranges}


def _feed(tr, discs_at, pose, t0, n, static=()):
    """n revolutions at 10 Hz from t0, swept in time (`scan_timed`);
    discs_at(t) -> the moving discs at t."""
    out = []
    for k in range(n):
        t = t0 + 0.1 * k
        sc = scan_timed(lambda tb: list(static) + discs_at(tb), pose, t)
        out.append(tr.update(sc, pose, t))
    return out


def test_a_mover_is_tracked_while_the_body_turns():
    # 3.234.B2: B skipped every revolution turning over 3 degrees; a pet
    # crossing during a turn went unseen. Steady first (the free-space
    # history), then a 57 deg/s turn (an arc's rate) while it crosses.
    tr = MoverTracker()
    still = lambda t: (0.0, 0.0, 0.0)
    _feed(tr, lambda t: [], still, 0.0, 12)
    turn = lambda t: (0.0, 0.0, math.radians(57.0) * (t - 1.2))
    out = _feed(tr, lambda t: [(1.5, -1.5 + 1.2 * (t - 1.2), 0.15)], turn, 1.2, 8)
    first = next(k for k, m in enumerate(out) if m)
    assert first <= 2
    m = out[-1][0]
    assert math.hypot(m.vx, m.vy - 1.2) < 0.15


def test_a_still_scene_from_a_turning_body_has_no_mover():
    tr = MoverTracker()
    still = [(1.5, 0.5, 0.15), (-1.0, -1.2, 0.15), (0.3, 2.0, 0.2)]
    out = _feed(tr, lambda t: [], _turning(57.0), 0.0, 40, static=still)
    out += _feed(tr, lambda t: [], _turning(400.0), 4.0, 20, static=still)
    assert all(m == [] for m in out)


def test_a_pivot_revolution_is_not_kept_for_the_free_space_test():
    tr = MoverTracker()
    tr.update(scan(), lambda t: (0.0, 0.0, math.radians(400.0) * t), 1.0)
    assert tr._history == []                    # 40 degrees a revolution
    tr.update(scan(), lambda t: (0.0, 0.0, math.radians(20.0) * t), 1.1)
    assert len(tr._history) == 1                # 2 degrees: kept


def test_two_free_observations_on_a_turning_body_are_not_an_early_mover():
    tr = MoverTracker()
    tr.update(scan(), (0.0, 0.0, 0.0), 0.0)
    tr.update(scan(), (0.0, 0.0, 0.0), 0.3)
    turn = _turning(20.0)                       # 2 degrees a revolution: not steady
    for k, t in enumerate((0.6, 0.7)):
        tr.update(scan([(1.5, -1.5 + 0.12 * k, 0.15)], turn(t)), turn, t)
    assert tr.movers() == [] and any(len(x.obs) == 2 and x.free for x in tr.tracks)


def test_a_restarted_fit_must_stand_in_free_space_again():
    tr = mt.Track(1, 0.0, 0.0, 0.0)
    tr.add(0.1, 0.05, 0.0, free=True)
    tr.add(0.2, 0.10, 0.0)
    assert tr.free
    tr.add(0.3, 0.60, 0.0)                      # far off the prediction: a restart
    assert len(tr.obs) == 1 and not tr.free and tr.free_n == 0


def test_each_cluster_is_stamped_when_the_beam_crossed_it():
    seen = []

    def pose(t):
        seen.append(t)
        return (0.0, 0.0, 0.0)
    MoverTracker().update(scan(discs=[(1.5, 0.0, 0.15)]), pose, 10.0)
    # Body bearing 0 is lidar angle 90 (yaw -90): a quarter turn in, so
    # 0.025 s before the revolution's midpoint.
    # The other calls are the turn check (capture -+ half a revolution) and
    # the capture pose kept for the free-space history.
    assert [t for t in seen if t not in (9.95, 10.0, 10.05)] == [
        pytest.approx(10.0 - 0.025, abs=0.002)]


def test_the_lidar_offset_is_the_vetoes():
    from robot.safety import LIDAR_X_M
    assert mt.LIDAR_X_M == LIDAR_X_M


def test_a_revolution_read_twice_adds_nothing():
    tr = MoverTracker(revolution_s=0.0)
    frames = [([(1.5, -1.5 + 0.12 * k, 0.15)], (0.0, 0.0, 0.0)) for k in range(2)]
    run(tr, frames)
    hits = [t.hits for t in tr.tracks]
    t = (PREROLL + 1) * 0.1
    tr.update(scan(*frames[1]), frames[1][1], t)
    assert [x.hits for x in tr.tracks] == hits


def test_a_clock_stepped_back_starts_over():
    tr = MoverTracker(revolution_s=0.0)
    run(tr, [([(1.5, -1.5 + 0.12 * k, 0.15)], (0.0, 0.0, 0.0)) for k in range(6)])
    assert tr.tracks
    tr.update(scan(), (0.0, 0.0, 0.0), -5.0)
    assert tr.tracks == [] and tr.scans == 1


def test_a_stamp_that_is_not_a_number_is_ignored():
    tr = MoverTracker(revolution_s=0.0)
    run(tr, [([(1.5, -1.5 + 0.12 * k, 0.15)], (0.0, 0.0, 0.0)) for k in range(6)])
    t, scans = tr.t, tr.scans
    tr.update(scan(), (0.0, 0.0, 0.0), float("nan"))
    assert tr.t == t and tr.scans == scans


def test_an_unusable_scan_never_asks_the_pose():
    def pose(t):
        raise AssertionError("asked")
    MoverTracker().update({"usable": False}, pose, 1.0)


def test_an_empty_usable_scan_is_harmless():
    assert MoverTracker().update({"usable": True, "ranges_m": []}, (0.0, 0.0, 0.0), 0.0) == []


def test_an_unusable_scan_ages_tracks_out():
    tr = MoverTracker(revolution_s=0.0)
    run(tr, [([(1.5, -1.5 + 0.12 * k, 0.15)], (0.0, 0.0, 0.0)) for k in range(6)])
    assert tr.movers()
    t = (PREROLL + 5) * 0.1
    for _ in range(4):
        t += 0.1
        assert tr.update({"usable": False}, (0.0, 0.0, 0.0), t) == []
    assert tr.tracks == []


def test_a_coasting_track_is_not_reported_as_seen():
    tr = MoverTracker(revolution_s=0.0)
    run(tr, [([(1.5, -1.5 + 0.12 * k, 0.15)], (0.0, 0.0, 0.0)) for k in range(6)])
    assert tr.update(scan(), (0.0, 0.0, 0.0), (PREROLL + 6) * 0.1) == []
    assert len(tr.coasting()) == 1


# --------------------------------------------------------------------------
# B's record, pinned (CLAUDE.md section 7 point 3)
# --------------------------------------------------------------------------

RECORD = os.path.join(os.path.dirname(__file__), "..", "evaluations",
                      "mover-prediction-3234", "b-sweep.json")


@pytest.mark.skipif(not os.path.exists(RECORD), reason="B's sweep not recorded yet")
def test_b_record_pinned():
    with open(RECORD) as f:
        v = json.load(f)["verdict"]
    assert v["seed"] == 0
    assert v["B1"]["missions"] == 24 and v["B2"]["missions"] == 72
    # The cost is this host's timing, not a result: never pinned.
    got = {"B1": v["B1"], "B2": {k: v["B2"][k] for k in PINNED["B2"]}}
    assert got == PINNED
    assert v["B1"]["met"] and v["B2"]["velocity_met"]       # B1, B2's velocity: met
    assert not v["B2"]["first_track_met"]                    # B2's time to first track: FAILED


PINNED = {'B1': {'missions': 24,
        'driving_min': 58.0,
        'false_moving_tracks': 5,
        'per_10_min': 0.862,
        'met': True},
 'B2': {'missions': 72,
        'velocity_error_samples': 35532,
        'velocity_error_median_m_s': 0.0375,
        'velocity_error_p90_m_s': 0.1377,
        'visits_judged': 5156,
        'visits_first_track_le_3': 2289,
        'velocity_met': True,
        'first_track_met': False,
        'velocity_error_p90_at_2.5': 0.8675}}


REVIEWED = RECORD.replace("b-sweep.json", "b-sweep-reviewed.json")


@pytest.mark.skipif(not os.path.exists(REVIEWED), reason="B's reviewed run not recorded")
def test_b_reviewed_record_pinned():
    """The run that governs (the reviewed tracker): B1 FAILED, B2's velocity
    met, B2's time to first track FAILED."""
    with open(REVIEWED) as f:
        v = json.load(f)["verdict"]
    assert v["seed"] == 0
    got = {"B1": v["B1"], "B2": {k: v["B2"][k] for k in PINNED_REVIEWED["B2"]}}
    assert got == PINNED_REVIEWED
    assert not v["B1"]["met"] and v["B2"]["velocity_met"] and not v["B2"]["first_track_met"]


PINNED_REVIEWED = {'B1': {'missions': 24,
        'driving_min': 58.0,
        'false_moving_tracks': 8,
        'per_10_min': 1.379,
        'met': False},
 'B2': {'missions': 72,
        'velocity_error_samples': 36146,
        'velocity_error_median_m_s': 0.0384,
        'velocity_error_p90_m_s': 0.1741,
        'visits_judged': 5156,
        'visits_first_track_le_3': 2331,
        'velocity_met': True,
        'first_track_met': False,
        'velocity_error_p90_at_2.5': 0.8783}}
