# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_rover_slam.py

PLAN-ros-alignment.md 3.221: SLAM on the real Rover. Two changes on the way
to ROS, pinned here without ROS:

* the robot's own mast (the safety layer's self box, 3.218) leaves the
  bridge as -inf, "too close to measure" -- exactly the beams the veto calls
  the body's own, and nothing else (criteria 1a, 2, 7);
* `/scan`'s `stamp_unix` is when the revolution was taken, not when it was
  asked for (criterion 6a), and the bridge sends no revolution twice (6c).
"""

import json
import math
import random
import sys
from pathlib import Path

import pytest

from robot.safety import SafetyController, scan_points_cm

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "service/slam/src/picar_bridge"))
from picar_bridge import convert  # noqa: E402

ROVER_BOX = (-7.1, 3.2, -10.7, 8.5)
RECORD = Path(__file__).resolve().parents[1] / "evaluations/lidar-self-box-3218/selfhits-20261126.json"


def _recorded(which):
    rows = json.loads(RECORD.read_text())["under_30cm"]
    ranges = [None] * 360
    for a, v in rows.items():
        ranges[int(a) + 180] = v[which] / 100.0
    return ranges


def _ours_index_of_ros(j, n=360, start=-180.0, inc=1.0):
    return int(round((-(start + j * inc) - start) / inc)) % n


def _safety_mask(ranges):
    """Per beam of OUR scan, whether robot/safety.py places its return in
    the box -- its own `scan_points_cm()` and `_in_self_box()`."""
    scan = {"angle_min_deg": -180.0, "angle_increment_deg": 1.0, "ranges_m": ranges}
    pts = iter(scan_points_cm(scan))
    import numpy as np
    ctl = SafetyController.__new__(SafetyController)
    ctl.self_box_cm = ROVER_BOX
    out = []
    for r in ranges:
        if r is None:
            out.append(False)
            continue
        x, y = next(pts)
        out.append(bool(ctl._in_self_box(np.array([x]), np.array([y]))[0]))
    return out


# ---- criterion 1a: the mast leaves as -inf, and only the mast ---------------

@pytest.mark.parametrize("which", ["median_cm", "max_cm"])
def test_every_recorded_self_return_goes_to_ros_as_minus_inf(which):
    ours = _recorded(which)
    ros = convert.ros_ranges(ours, -180.0, 1.0, list(ROVER_BOX))
    hits = 0
    for j, r in enumerate(ros):
        if ours[_ours_index_of_ros(j)] is None:
            assert r == math.inf
        else:
            assert r == -math.inf, (j, ours[_ours_index_of_ros(j)])
            hits += 1
    assert hits == 92


def test_the_masked_beams_are_exactly_the_ones_the_veto_calls_its_own():
    rng = random.Random("3.221-mask")
    masked_total = 0
    for trial in range(10_000):
        ours = []
        for deg in range(-180, 180):
            u = rng.random()
            if u < 0.2:
                ours.append(None)
            elif u < 0.6:                        # near: the box's edges get hit
                ours.append(round(rng.uniform(0.02, 0.20), 4))
            else:
                ours.append(round(rng.uniform(0.20, 6.0), 4))
        want = _safety_mask(ours)
        ros = convert.ros_ranges(ours, -180.0, 1.0, list(ROVER_BOX))
        for j, r in enumerate(ros):
            i = _ours_index_of_ros(j)
            if ours[i] is None:
                assert r == math.inf, (trial, j)
            elif want[i]:
                assert r == -math.inf, (trial, j, ours[i])
                masked_total += 1
            else:
                assert r == ours[i], (trial, j, ours[i])
    assert masked_total > 10_000          # the edges were really exercised


@pytest.mark.parametrize("box", [None, []])
def test_no_box_leaves_the_ros_scan_as_it_was(box):
    ours = _recorded("median_cm")
    assert convert.ros_ranges(ours, -180.0, 1.0, box) == convert.ros_ranges(ours, -180.0, 1.0)
    assert -math.inf not in convert.ros_ranges(ours, -180.0, 1.0, box)


def test_a_box_edge_is_inside_as_safety_counts_it():
    # A return straight behind at exactly the box's x_min: inside for both.
    for r in (0.111, 0.1109999, 0.1110001, 0.0251, 0.0249):
        ours = [None] * 360
        ours[0] = r                        # our beam 0 is -180, straight behind
        assert _safety_mask(ours)[0] == convert.in_self_box(r, -180.0, ROVER_BOX), r


# ---- criterion 6c: one revolution is sent once ------------------------------

def test_a_revolution_read_twice_is_not_a_new_scan():
    assert not convert.same_revolution(100.0, None)
    assert convert.same_revolution(100.0004, 100.0)        # jitter
    assert convert.same_revolution(100.0, 100.0)           # the server fixes a stamp once
    assert not convert.same_revolution(100.1, 100.0)       # the next revolution
    # Review (second round): a stamp a little older would reach SLAM out of
    # order: held off. A clock stepped back past a second is a new start.
    assert convert.same_revolution(99.95, 100.0)
    assert convert.same_revolution(99.2, 100.0)
    assert not convert.same_revolution(95.0, 100.0)


@pytest.mark.parametrize("bad", [[1, 2, 3], [1, 2, 3, 4, 5], ["a", 2, 3, 4], {"x": 1},
                                 [3, 1, 2, 4], [float("nan"), 1, 2, 3], 7])
def test_a_malformed_box_masks_nothing_and_never_raises(bad):
    ours = _recorded("median_cm")
    assert convert.ros_ranges(ours, -180.0, 1.0, bad) == convert.ros_ranges(ours, -180.0, 1.0)


# ---- criteria 2 and 6a: the robot server's /scan -----------------------------

def _server(monkeypatch, tmp_path, env=None):
    import yaml
    from fastapi.testclient import TestClient
    import robot.server as server

    cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/robot.yaml").read_text())
    cfg["mode"] = "sim"
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(cfg))
    monkeypatch.delenv("ROBOT_MODE", raising=False)
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    if env is None:
        monkeypatch.delenv("ROBOT_LIDAR_SELF_BOX_CM", raising=False)
    else:
        monkeypatch.setenv("ROBOT_LIDAR_SELF_BOX_CM", env)
    return TestClient(server.create_app(str(path)))


def test_the_simulator_keeps_the_request_stamp_and_no_box(monkeypatch, tmp_path):
    import time
    with _server(monkeypatch, tmp_path) as client:
        t0 = time.time()
        s = client.get("/scan").json()
        t1 = time.time()
    assert s["stamp_source"] == "request" and t0 <= s["stamp_unix"] <= t1
    assert s["lidar_self_box_cm"] is None


def test_the_scan_carries_the_box_and_is_never_masked_itself(monkeypatch, tmp_path):
    from sim.mock_robot import MockRobot
    with _server(monkeypatch, tmp_path) as client:
        plain = client.get("/scan").json()
    with _server(monkeypatch, tmp_path, env="-7.1,3.2,-10.7,8.5") as client:
        boxed = client.get("/scan").json()
    assert boxed["lidar_self_box_cm"] == list(ROVER_BOX)
    assert boxed["ranges_m"] == plain["ranges_m"]     # the veto's scan is whole
    assert MockRobot  # the same simulated body behind both


def _capture(monkeypatch, value):
    from sim.mock_robot import MockRobot
    real = MockRobot.get_scan
    monkeypatch.setattr(MockRobot, "get_scan",
                        lambda self, max_range_m=None: {**real(self, max_range_m),
                                                        "capture_unix": value})


def test_a_scan_that_carries_its_capture_time_is_stamped_with_it(monkeypatch, tmp_path):
    _capture(monkeypatch, 1_800_000_000.25)
    with _server(monkeypatch, tmp_path) as client:
        s = client.get("/scan").json()
    assert s["stamp_source"] == "capture" and s["stamp_unix"] == 1_800_000_000.25
    assert "capture_unix" not in s                 # one stamp on the wire, not two


@pytest.mark.parametrize("bad", [None, float("nan"), float("inf"), True, "1.0"])
def test_a_capture_time_that_cannot_be_used_falls_back_to_the_request(monkeypatch, tmp_path, bad):
    _capture(monkeypatch, bad)
    with _server(monkeypatch, tmp_path) as client:
        s = client.get("/scan").json()
    assert s["stamp_source"] == "request"


# ---- criterion 6a: the lidar's capture age is the revolution's midpoint ------

def test_the_capture_time_is_this_revolutions_midpoint_and_fixed():
    from robot.lidar_ld19 import Ld19Scanner
    from tests.test_lidar_ld19 import _Clock, _feed_revolutions
    clock = _Clock()
    offset = 1_700_000_000.0                       # wall = monotonic + offset
    s = Ld19Scanner(clock=clock, wall=lambda: clock.t + offset)
    _feed_revolutions(s, clock, 3)
    scan = s.get_scan()
    # this revolution's midpoint: about half a 10 Hz revolution before its newest point
    assert 0.03 < s._scan_newest - (scan["capture_unix"] - offset) < 0.07
    assert scan["capture_unix"] - offset > s._scan_oldest       # never the oldest point
    clock.t += 0.05
    assert s.get_scan()["capture_unix"] == scan["capture_unix"]   # a re-read: same stamp
    clock.t += 0.31
    assert "capture_unix" not in s.get_scan()                    # stale: no claim


def test_carried_bins_do_not_move_the_stamp_back(monkeypatch):
    """Review (3.221): a revolution that fills empty bins from the one before
    is aged for the veto by the older one; its stamp stays its own."""
    import robot.lidar_ld19 as ld
    from tests.test_lidar_ld19 import _Clock, _feed_revolutions
    real = ld.bin_scan
    monkeypatch.setattr(ld, "bin_scan", lambda *a, **k: {**real(*a, **k), "carried": True})
    clock = _Clock()
    s = ld.Ld19Scanner(clock=clock, wall=lambda: clock.t)
    _feed_revolutions(s, clock, 3)
    scan = s.get_scan()
    assert s._scan_oldest < s._scan_newest - 0.15      # the veto ages by the carried bins
    assert s._scan_newest - 0.11 < scan["capture_unix"] < s._scan_newest  # the stamp: this revolution
