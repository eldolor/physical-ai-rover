# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_lidar_self_box.py

PLAN-ros-alignment.md 3.218: the Rover's D500 sees the pan-tilt mast right
behind it (2.3-14 cm, the rear quarter, every scan). A measured self box in
the safety layer lets FORWARD ignore those returns; REVERSE stays refused,
as `astern_occluded`, because the mast hides what is behind. Criteria 1-6
are here; 7 is on the Rover. The recorded returns are
`evaluations/lidar-self-box-3218/selfhits-20261126.json` (only those under
30 cm; never the room).
"""

import json
import math
import random
from pathlib import Path

import pytest

from robot.safety import (FOOTPRINT_LENGTH_M, FOOTPRINT_WIDTH_M, LIDAR_X_M,
                          SELF_BOX_EDGE_MIN_CM, SafetyController, self_box)

ROVER_BOX = (-7.1, 3.2, -10.7, 8.5)
RECORD = Path(__file__).resolve().parents[1] / "evaluations/lidar-self-box-3218/selfhits-20261126.json"


class _Body:
    """A body with a lidar scan and nothing else, wheels reporting (so a
    blind astern reads as blind, not as a phone walk)."""

    def __init__(self, ranges):
        self.ranges = ranges

    def get_scan(self, max_range_m=None):
        return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
                "range_min_m": 0.02, "range_max_m": 12.0, "ranges_m": list(self.ranges)}

    def get_wheel_state(self):
        return {"usable": True}

    def get_depth_grid(self):
        return {"usable": False}

    def get_distance(self):
        return 0.0


def _recorded(which):
    rows = json.loads(RECORD.read_text())["under_30cm"]
    ranges = [None] * 360
    for a, v in rows.items():
        ranges[int(a) + 180] = v[which] / 100.0
    return ranges


def _body_point(r_m, deg):
    a = math.radians(deg)
    return r_m * 100 * math.cos(a) + LIDAR_X_M * 100, -r_m * 100 * math.sin(a)


# ---- criterion 1: every recorded self return is inside the box --------------

@pytest.mark.parametrize("which", ["median_cm", "max_cm"])
def test_every_recorded_self_return_is_inside_the_box(which):
    x0, x1, y0, y1 = ROVER_BOX
    ranges = _recorded(which)
    hits = [(i - 180, r) for i, r in enumerate(ranges) if r is not None]
    assert len(hits) == 92
    for deg, r in hits:
        x, y = _body_point(r, deg)
        assert x0 <= x <= x1 and y0 <= y <= y1, (deg, r, x, y)
    safety = SafetyController(_Body(ranges), self_box_cm=ROVER_BOX)
    assert safety.footprint_clearance(+1) == (None, "scan_footprint_no_target")
    # Without the box the same scan blocks every forward move (the bug).
    assert SafetyController(_Body(ranges)).footprint_clearance(+1) == (0.0, "scan_footprint")


# ---- criterion 2: the box is inside the outline -----------------------------

def test_the_rover_box_is_inside_the_outline_by_the_margin():
    hl, hw = FOOTPRINT_LENGTH_M * 50, FOOTPRINT_WIDTH_M * 50
    x0, x1, y0, y1 = self_box(ROVER_BOX)
    assert min(hl + x0, hl - x1, hw + y0, hw - y1) >= SELF_BOX_EDGE_MIN_CM


@pytest.mark.parametrize("bad", [(-7.1, 3.2, -11.2, 8.5), (-12.5, 0, 0, 1), (1, 0, 0, 1),
                                 (-2, 5, -1, 1),      # reaches ahead of the lidar (x 4 cm)
                                 (0, 1, 0, float("nan")), (0, 1, 2), "a,b,c,d"])
def test_a_box_outside_the_outline_or_malformed_is_refused(bad):
    with pytest.raises(ValueError):
        SafetyController(_Body([None] * 360), self_box_cm=bad)


def test_the_box_reads_from_a_string_and_unset_is_none():
    assert self_box("-7.1,3.2,-10.7,8.5") == ROVER_BOX
    assert self_box(None) is None and self_box("") is None


# ---- criterion 3: nothing else changes --------------------------------------

def test_scans_with_nothing_in_the_box_give_identical_answers():
    rng = random.Random(3218)
    x0, x1, y0, y1 = ROVER_BOX
    for trial in range(10_000):
        ranges = []
        for deg in range(-180, 180):
            r = None if rng.random() < 0.3 else rng.choice(
                (rng.uniform(0.02, 0.6), rng.uniform(0.6, 4.0)))
            if r is not None:
                x, y = _body_point(r, deg)
                if x0 <= x <= x1 and y0 <= y <= y1:
                    r = None
            ranges.append(r)
        plain = SafetyController(_Body(ranges))
        boxed = SafetyController(_Body(ranges), self_box_cm=ROVER_BOX)
        for d in (+1, -1):
            assert plain.footprint_clearance(d) == boxed.footprint_clearance(d), (trial, d)
        # Amendment 1 (2026-11-26, after the reviews, before re-running):
        # with a box set, EVERY reverse is astern_occluded -- the mast hides
        # the rear whether or not a scan caught a return off it.
        assert boxed.reverse_clearance() == (0.0, "astern_occluded"), trial
        omega = rng.choice((-1.2, -0.4, 0.4, 1.2))
        assert plain.pivot_scale(omega) == boxed.pivot_scale(omega), trial


# ---- criterion 4: reverse stays refused, as what it is ----------------------

def test_reverse_is_refused_as_occluded_while_the_mast_is_seen():
    safety = SafetyController(_Body(_recorded("median_cm")), self_box_cm=ROVER_BOX)
    assert safety.reverse_clearance() == (0.0, "astern_occluded")


def test_a_scan_that_missed_the_mast_still_refuses_reverse():
    # Thermos finding 1: a revolution with no return off the mast read its
    # shadow as clear astern.
    safety = SafetyController(_Body([None] * 360), self_box_cm=ROVER_BOX)
    assert safety.reverse_clearance() == (0.0, "astern_occluded")


# ---- the brain's controller judges the scan as the server does --------------

class _RemoteLike(_Body):
    def __init__(self, ranges, box):
        super().__init__(ranges)
        self.box = box

    def lidar_self_box_cm(self):
        if isinstance(self.box, Exception):
            raise self.box
        return self.box


def test_a_controller_without_a_box_asks_the_body():
    # Code review finding 1: the brain's controller had no box and refused
    # every forward move the server allowed.
    ranges = _recorded("median_cm")
    asked = SafetyController(_RemoteLike(ranges, list(ROVER_BOX)))
    assert asked.self_box_cm == ROVER_BOX
    assert asked.footprint_clearance(+1) == (None, "scan_footprint_no_target")


@pytest.mark.parametrize("answer", [None, OSError("down"), [1, 2], "nonsense", (-2, 5, -1, 1)])
def test_a_body_that_cannot_give_a_valid_box_gives_none(answer):
    ranges = _recorded("median_cm")
    safety = SafetyController(_RemoteLike(ranges, answer))
    assert safety.self_box_cm is None
    assert safety.footprint_clearance(+1) == (0.0, "scan_footprint")    # refuses more, not less


def test_remote_robot_reads_the_box_from_health():
    from control.remote_robot import RemoteRobot

    robot = RemoteRobot.__new__(RemoteRobot)
    robot._request = lambda method, path, json=None: {"lidar_self_box_cm": list(ROVER_BOX)}
    assert robot.lidar_self_box_cm() == list(ROVER_BOX)

    def down(*a, **k):
        raise OSError("refused")
    robot._request = down
    assert robot.lidar_self_box_cm() is None


# ---- the robot server: config, environment, refusal -------------------------

def _server(monkeypatch, tmp_path, yaml_box=None, env=None):
    import yaml
    from fastapi.testclient import TestClient
    import robot.server as server

    cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/robot.yaml").read_text())
    cfg["mode"] = "sim"
    if yaml_box is not None:
        cfg.setdefault("safety", {})["lidar_self_box_cm"] = yaml_box
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(cfg))
    monkeypatch.delenv("ROBOT_MODE", raising=False)
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    if env is None:
        monkeypatch.delenv("ROBOT_LIDAR_SELF_BOX_CM", raising=False)
    else:
        monkeypatch.setenv("ROBOT_LIDAR_SELF_BOX_CM", env)
    return TestClient(server.create_app(str(path)))


def test_the_server_reports_the_box_and_an_empty_variable_switches_it_off(monkeypatch, tmp_path):
    with _server(monkeypatch, tmp_path, yaml_box=list(ROVER_BOX)) as client:
        assert client.get("/health").json()["lidar_self_box_cm"] == list(ROVER_BOX)
    with _server(monkeypatch, tmp_path, yaml_box=list(ROVER_BOX), env="") as client:
        assert client.get("/health").json()["lidar_self_box_cm"] is None
    with _server(monkeypatch, tmp_path, env="-7.1,3.2,-10.7,8.5") as client:
        assert client.get("/health").json()["lidar_self_box_cm"] == list(ROVER_BOX)


def test_the_server_refuses_to_start_on_a_bad_box(monkeypatch, tmp_path):
    with pytest.raises(ValueError):
        _server(monkeypatch, tmp_path, env="-7.1,3.2,-11.5,8.5")


# ---- criterion 5: turns are not blocked by the mast -------------------------

@pytest.mark.parametrize("box", [None, ROVER_BOX])
def test_turns_are_not_blocked_by_the_mast(box):
    safety = SafetyController(_Body(_recorded("max_cm")), self_box_cm=box)
    for omega in (-1.2, -0.4, 0.4, 1.2):
        assert safety.pivot_scale(omega)[0] == 1.0


# ---- a real obstacle ahead still stops a forward move ------------------------

def test_an_object_ahead_still_blocks_forward_with_the_box():
    ranges = _recorded("median_cm")
    ranges[180] = (FOOTPRINT_LENGTH_M * 50 - LIDAR_X_M * 100 + 10) / 100   # 10 cm past the bumper
    safety = SafetyController(_Body(ranges), self_box_cm=ROVER_BOX)
    cm, source = safety.footprint_clearance(+1)
    assert source == "scan_footprint" and cm == pytest.approx(10.0, abs=0.2)
