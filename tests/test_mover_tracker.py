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
    assert first <= 2                           # the third scan, counting from 1
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


def test_without_history_nothing_is_moving():
    tr = MoverTracker(revolution_s=0.0)
    out = run(tr, [([(1.5, -1.5 + 0.12 * k, 0.15)], (0.0, 0.0, 0.0)) for k in range(4)],
              preroll=0)
    assert all(m == [] for m in out)


def test_each_cluster_is_stamped_when_the_beam_crossed_it():
    seen = []

    def pose(t):
        seen.append(t)
        return (0.0, 0.0, 0.0)
    MoverTracker().update(scan(discs=[(1.5, 0.0, 0.15)]), pose, 10.0)
    # Body bearing 0 is lidar angle 90 (yaw -90): a quarter turn in, so
    # 0.025 s before the revolution's midpoint.
    assert seen[0] == pytest.approx(10.0 - 0.025, abs=0.002)   # then the capture pose, for the history


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
    v = json.load(open(RECORD))["verdict"]
    assert v["seed"] == 0
    assert v["B1"]["missions"] == 24 and v["B2"]["missions"] == 72
    assert v == PINNED


PINNED = None
