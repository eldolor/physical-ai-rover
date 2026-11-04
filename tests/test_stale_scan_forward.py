# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_stale_scan_forward.py

A body that really moves and has no usable scan does not drive FORWARD --
PLAN-ros-alignment.md 3.144, from 3.135.B's FMEA row F9.

On the Jetson the fake D500 was SIGSTOPped mid-FORWARD: the scan went stale
after `STALE_S` (0.3 s), but `forward_clearance()` refused only when the
depth cone faced away TOO, so with the camera ahead the move was vetted on
the cone alone and the robot drove 101-105 cm blind (5 of 5 runs). REVERSE
already refused on an unusable scan (`astern_not_observed`); this pins the
same rule ahead (`ahead_not_observed`), with the cone seeing open floor so
the case is F9's, not 3.18's.

Judged on the simulator's ground truth (CLAUDE.md section 6).
"""

import math
import time

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from robot.interface import unusable_scan
from robot.safety import SafetyController, SafetyViolation
from sim.maps.scaled_house import build_scaled_world
from sim.mock_robot import WHEEL_RADIUS_M, MockRobot
from sim.teleop_robot import TeleopRobot
from tests import footprint_sweep as fs

# Open floor ahead: the cone and the corridor both read ~2.4 m.
HOUSE, X, Y, THETA_DEG = "scaled_house", 2.5, 4.5, 0
STALE_TRAVEL_BAR_CM = 2.0      # criterion 2: true path once the scan is unusable


class LidarStops(MockRobot):
    """The simulator with a lidar that can stop: while `blind`, its scan is
    unusable, as the LD19 driver's is past `STALE_S`."""

    blind = False

    def get_scan(self, max_range_m=None):
        return unusable_scan() if self.blind else super().get_scan(max_range_m=max_range_m)


def _placed(robot):
    robot.world.x, robot.world.y = X, Y
    robot.world.theta = math.radians(THETA_DEG)
    return robot


def _xy(robot):
    return robot.world.x, robot.world.y


def _cm(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]) * fs.CELL_CM


@pytest.fixture
def blind_robot():
    robot = _placed(LidarStops(build_scaled_world(), render=False))
    robot.blind = True
    return robot


def test_the_cone_alone_would_let_it_go(blind_robot):
    """The precondition that makes this F9: the camera sees open floor."""
    safety = SafetyController(blind_robot)
    cone, source = safety.path_clearance(blind_robot.get_depth_grid(), None)
    assert source != "depth_grid_facing_away"
    assert cone is None or cone > safety.required_clearance_cm(fs.SPEED_M_S) + 5


def test_forward_reads_not_observed_on_an_unusable_scan(blind_robot):
    assert SafetyController(blind_robot).forward_clearance() == (0.0, "ahead_not_observed")


def test_a_forward_verb_is_refused_and_nothing_moves(blind_robot):
    before = _xy(blind_robot)
    with pytest.raises(SafetyViolation, match="ahead_not_observed"):
        SafetyController(blind_robot).check_and_execute("FORWARD", speed=50, duration=0.5)
    assert _xy(blind_robot) == before


def test_a_standing_forward_command_is_clamped_and_turns_stay_allowed(blind_robot):
    safety = SafetyController(blind_robot)
    left, right, reason = safety.vet_wheel_velocity(5.0, 5.0)
    assert (left, right) == (0.0, 0.0)
    assert "ahead_not_observed" in reason
    _, _, reason = safety.vet_wheel_velocity(-3.0, 3.0)
    assert reason is None


def test_a_body_with_no_wheels_is_unchanged():
    # A phone walk: FORWARD moves a person, who can see.
    _, source = SafetyController(TeleopRobot()).forward_clearance()
    assert source != "ahead_not_observed"


def test_a_body_with_a_scan_is_judged_by_it():
    robot = _placed(LidarStops(build_scaled_world(), render=False))
    clearance, source = SafetyController(robot).forward_clearance()
    assert source != "ahead_not_observed"
    assert clearance is None or clearance > 0


def test_a_forward_under_way_stops_when_the_scan_goes(monkeypatch):
    """Criterion 2a: the guarded verb re-vets every period; the first vet
    after the scan goes must stop it."""
    robot = _placed(LidarStops(build_scaled_world(), render=False))
    start, at_blind = _xy(robot), {}
    real_get_scan = LidarStops.get_scan

    def stops_after_8cm(self, max_range_m=None):
        if not self.blind and _cm(_xy(self), start) >= 8.0:
            self.blind, at_blind["xy"] = True, _xy(self)
        return real_get_scan(self, max_range_m=max_range_m)

    monkeypatch.setattr(LidarStops, "get_scan", stops_after_8cm)
    try:
        SafetyController(robot).check_and_execute("FORWARD", speed=50, duration=2.0)
    except SafetyViolation:
        pass                                     # refused under the minimum: also a stop
    assert "xy" in at_blind, "the verb never covered 8 cm: the fault was not fired"
    after = _cm(_xy(robot), at_blind["xy"])
    assert after <= STALE_TRAVEL_BAR_CM, f"{after:.1f} cm after the scan went"


def test_the_wheel_loop_stops_it_and_it_does_not_resume(monkeypatch):
    """Criterion 2b: through POST /wheels and robot/server.py's own wheel
    loop. The scan comes back inside the watchdog's 1 s, with no new
    command: the clamped command was written back, so nothing drives."""
    monkeypatch.setenv("SIM_MAP", HOUSE)
    built = {}

    def placed(*a, **kw):
        built["robot"] = _placed(LidarStops(build_scaled_world(), render=False))
        return built["robot"]

    monkeypatch.setattr(server, "get_robot", placed)
    fwd = 0.3 / WHEEL_RADIUS_M      # 1.5 cm a 50 ms period: the bar is under two
    with TestClient(server.create_app()) as c:
        robot = built["robot"]
        start = _xy(robot)
        r = c.post("/wheels", json={"left_rad_s": fwd, "right_rad_s": fwd},
                   headers={"x-driver": "ros"})
        assert r.status_code == 200, r.text
        time.sleep(0.25)
        assert _cm(_xy(robot), start) > 1.0, "the robot must be moving before the fault"
        robot.blind = True
        at_blind = _xy(robot)
        time.sleep(0.2)
        stopped_at = _xy(robot)
        robot.blind = False                      # the scan is back; no new command
        time.sleep(0.25)
        w = robot.get_wheel_state()
        back = _xy(robot)
        c.post("/stop")
    after = _cm(stopped_at, at_blind)
    resumed = _cm(back, stopped_at)
    assert after <= STALE_TRAVEL_BAR_CM, f"{after:.1f} cm after the scan went"
    assert (w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]) == (0.0, 0.0)
    assert resumed == 0.0, f"drove {resumed:.1f} cm on the old command once the scan returned"
