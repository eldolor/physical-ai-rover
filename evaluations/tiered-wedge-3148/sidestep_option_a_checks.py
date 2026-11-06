# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/tiered-wedge-3148/sidestep_option_a_checks.py (not collected by pytest: run it by name with option (a)'s patch applied)

3.148 criterion 4 (`docs/plans/ros-alignment/3.148-tiered-wedge-without-
cloud.md`): a tiered mission with no tier able to steer (the cloud refused, no
local VLM, no nav2) aimed straight at the target along a line that clips
`scaled_house`'s door jamb, and every back-off of the stuck breaker retraced
it until the mission ended `blocked` -- at (2.97, 1.71), the board's own spot
(3.135.B F11a/F11b/F12a). The back-off now sidesteps away from the nearest
return (`MissionRunner._sidestep_turn`). Through the real path in process:
`MissionRunner` -> tiered -> `robot/safety.py` -> `MockRobot`.
"""

import logging
import os
from unittest import mock

import pytest

from brain.navigate import CloudUnavailable
from control import brain_server
from control.brain_config import load_brain_config
from control.mission_runner import BLOCKED, MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld

KEEP_ENV = ("PATH", "HOME", "PYTHONPATH", "TMPDIR", "LANG")


class Refused:
    def __init__(self):
        self.calls = 0

    def __call__(self, frame):
        self.calls += 1
        raise CloudUnavailable("refused (3.148)")


@pytest.fixture
def clean_env():
    env = {k: os.environ[k] for k in KEEP_ENV if k in os.environ}
    logging.disable(logging.WARNING)
    with mock.patch.dict(os.environ, env, clear=True):
        yield
    logging.disable(logging.NOTSET)


def _mission(x, y, theta=0.0):
    grid = build_world("scaled_house")
    grid.x, grid.y, grid.theta = x, y, theta
    robot = MockRobot(grid, render=False)
    config = load_brain_config()
    config.update(tier_local_vlm_url="", sim_arrival_confirm="cloud")
    cloud = Refused()
    tier = brain_server._tiered_vision_fn("red backpack", cloud, config, simulated=True,
                                          confirm_vision_fn=cloud)
    runner = MissionRunner(robot, policy="tiered", vision_fn=tier, target_object="red backpack",
                           world=MockWorld(grid), max_steps=400, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, vision_timeout_s=config["vision_timeout_s"],
                           arrival_confirm_timeout_s=config["arrival_confirm_timeout_s"],
                           max_vision_failures=config["max_vision_failures"])
    try:
        runner.start()
        ticks = 0
        while runner.tick() and ticks < 3000:
            ticks += 1
        return runner.status(), grid
    finally:
        runner.stop()
        tier.close()


# Start lines west of the first door, facing east, that wedged at its jamb on
# `dev` before 3.148 (y in cells; 5.6 cells = 1.68 m).
@pytest.mark.parametrize("y", [5.6, 5.75])
def test_no_steering_tier_passes_the_jamb_instead_of_ending_blocked(y, clean_env):
    s, grid = _mission(8.0, y)
    assert s["outcome"] != BLOCKED, s.get("end_reason")
    assert s["outcome"] == "arrived_unconfirmed"
    assert grid.x * 0.3 > 6.0            # through both doors to the backpack


class _Scan:
    def __init__(self, ranges, a0=-180.0, da=1.0):
        self.scan = {"usable": True, "angle_min_deg": a0, "angle_increment_deg": da,
                     "ranges_m": ranges}

    def get_scan(self, max_range_m=None):
        return self.scan


def _turn(ranges):
    r = MissionRunner.__new__(MissionRunner)
    r.robot = _Scan(ranges)
    return r._sidestep_turn()


def _beams(*at):
    """360 beams of nothing, with a return of `m` metres at each (deg, m)."""
    ranges = [None] * 360
    for deg, m in at:
        ranges[deg + 180] = m
    return ranges


def test_sidestep_turns_away_from_an_off_centre_return():
    assert _turn(_beams((12, 0.6))) == "LEFT"        # clockwise-positive: on the right
    assert _turn(_beams((-20, 0.5))) == "RIGHT"       # on the left


def test_sidestep_leaves_a_square_wall_or_nothing_near_alone():
    assert _turn(_beams((2, 0.4))) is None           # within 5 deg of dead ahead
    assert _turn(_beams((30, 1.4))) is None          # nothing within 1.0 m
    assert _turn(_beams((90, 0.3))) is None          # beside, not ahead (> 45 deg)
    r = MissionRunner.__new__(MissionRunner)
    r.robot = _Scan([])
    r.robot.scan["usable"] = False
    assert r._sidestep_turn() is None
