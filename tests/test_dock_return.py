# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.156.B: the return to the dock (`docs/plans/ros-alignment/
3.156-return-to-dock.md`), in process -- `MissionRunner` -> the explore
policy -> `robot/safety.py` -> `MockRobot`, nav2 stood in for by
`tests/test_explore.py`'s FakeNav (the sweep drives), the dock the start pose (D1, D6).

C8's red-first tests: D2's every end state (a return after `found`,
`exhausted`, `out_of_budget`, a survey's end, `arrived_unconfirmed` and
`blocked`; none after `cancelled`, `failed` or a battery at Reserve), D5's
every refusal (a Stop, a D-pad, Reserve mid-return), D3's reserve, and
`return_to_dock` off reproducing today's end."""

import math

import pytest

from brain.dock import DOCK_TOL_DEG, DOCK_TOL_M, HomeBudget, ahead, staging_point
from control.mission_runner import (ARRIVED_UNCONFIRMED, BLOCKED, CANCELLED, EXHAUSTED, FAILED,
                                    FOUND, OUT_OF_BUDGET, MissionRunner)
from robot.interface import BatteryReserve, Preempted
from sim.battery import SimBattery
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav

CELL = 0.30


class RecNav(FakeNav):
    """FakeNav that can be told to refuse (never ready) or to raise."""

    def __init__(self, robot, never_ready=False):
        super().__init__(robot)
        self.never_ready = never_ready
        self.raise_on_set = None

    def set_goal(self, x_m, y_m):
        if self.raise_on_set is not None:
            raise self.raise_on_set
        if self.never_ready:
            raise RuntimeError("POST /goal -> 503 hold: nav2_not_ready")
        return super().set_goal(x_m, y_m)


def mission(target="red backpack", house="scaled_house", job=None, never_ready=False,
            battery=None, return_to_dock=True, **kw):
    grid = build_world(house)
    robot = MockRobot(grid, render=False, battery=battery)
    if battery is not None:
        battery.clock = lambda: grid.sim_time
        battery.since = grid.sim_time
    nav = RecNav(robot, never_ready=never_ready)
    args = {"job": "survey"} if job == "survey" else {"target_object": target}
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=MockWorld(grid),
                           max_steps=kw.pop("max_steps", 200), clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=return_to_dock, **args, **kw)
    return runner, nav, grid, robot


def run(runner, cap=6000):
    runner.start()
    n = 0
    while runner.tick() and n < cap:
        n += 1
    return runner.status()


def truth_error(grid, dock):
    x, y = grid.x * CELL, grid.y * CELL
    return (math.hypot(x - dock["x_m"], y - dock["y_m"]),
            abs((grid.heading_deg - dock["theta"] + 180) % 360 - 180))


def docked_at_start(s, grid):
    assert s["return"]["state"] == "docked", s["return"]
    m, deg = truth_error(grid, s["dock"])
    assert m <= DOCK_TOL_M and deg <= DOCK_TOL_DEG, (m, deg)


# -- D2: a return after every end state that asks for one -------------------

def test_found_then_home():
    runner, nav, grid, _ = mission()
    s = run(runner)
    assert s["outcome"] == FOUND, s["log_tail"]
    assert s["dock"]["source"] == "start"
    docked_at_start(s, grid)
    assert any("returning to the dock" in l for l in s["log_tail"])


def test_a_survey_ends_then_home():
    runner, nav, grid, _ = mission(job="survey")
    s = run(runner)
    assert s["outcome"] == EXHAUSTED
    docked_at_start(s, grid)


@pytest.mark.parametrize("outcome", [EXHAUSTED, OUT_OF_BUDGET, ARRIVED_UNCONFIRMED, BLOCKED])
def test_every_returning_end_state_goes_home(outcome):
    """The outcome is decided first and never changed by the return."""
    runner, nav, grid, robot = mission()
    runner.start()
    for _ in range(8):                  # leave the dock
        runner.tick()
    runner._finish(outcome, "decided")
    assert runner.is_running(), "the return runs before the end"
    n = 0
    while runner.tick() and n < 3000:
        n += 1
    s = runner.status()
    assert s["outcome"] == outcome and s["end_reason"] == "decided"
    docked_at_start(s, grid)


@pytest.mark.parametrize("outcome,why", [(CANCELLED, "person"), (FAILED, "fault")])
def test_no_return_after_a_person_or_a_fault(outcome, why):
    runner, nav, grid, robot = mission()
    runner.start()
    for _ in range(8):
        runner.tick()
    if outcome == CANCELLED:
        runner.stop("a person")
    else:
        runner.abort("a fault")
    s = runner.status()
    assert not runner.is_running() and s["outcome"] == outcome
    assert s["return"] == {"state": "skipped", "reason": why}


def test_no_return_at_the_battery_reserve():
    runner, nav, grid, robot = mission()
    runner.start()
    runner.tick()
    runner._battery_level = "reserve"
    runner._finish(OUT_OF_BUDGET, "battery at reserve (10.15 V), as the robot server reads it")
    s = runner.status()
    assert not runner.is_running()
    assert s["return"] == {"state": "skipped", "reason": "battery_reserve"}


def test_no_nav2_is_skipped_never_guessed():
    runner, nav, grid, robot = mission()
    runner.start()
    for _ in range(4):                  # nav2 up for the search ...
        runner.tick()
    nav.never_ready = True              # ... then gone for the return
    runner._finish(EXHAUSTED, "decided")
    x0, y0 = grid.x, grid.y
    n = 0
    while runner.tick() and n < 500:
        n += 1
    s = runner.status()
    assert s["outcome"] == EXHAUSTED
    assert s["return"]["state"] == "skipped" and s["return"]["reason"].startswith("no_nav2")
    assert (grid.x, grid.y) == (x0, y0), "no straight-line verbs home"


def test_return_to_dock_off_is_todays_end():
    runner, nav, grid, robot = mission(return_to_dock=False)
    s = run(runner)
    assert s["outcome"] == FOUND
    assert s["return"] is None
    assert not any("returning" in l for l in s["log_tail"])


# -- D5: a person and the battery still win ----------------------------------

def _returning(**kw):
    runner, nav, grid, robot = mission(**kw)
    runner.start()
    for _ in range(8):
        runner.tick()
    runner._finish(EXHAUSTED, "decided")
    for _ in range(3):
        runner.tick()
    assert runner.is_running() and runner.status()["return"]["state"] == "returning"
    return runner, nav, grid, robot


def test_a_stop_during_the_return_cancels_it_and_keeps_the_outcome():
    runner, nav, grid, robot = _returning()
    runner.stop("a person pressed Stop")
    s = runner.status()
    assert not runner.is_running()
    assert s["outcome"] == EXHAUSTED and s["end_reason"] == "decided"
    assert s["return"]["state"] == "cancelled"
    x, y = grid.x, grid.y
    assert not runner.tick()
    assert (grid.x, grid.y) == (x, y), "no motion after the Stop"


def test_a_dpad_preempts_the_return():
    runner, nav, grid, robot = _returning()
    nav.raise_on_set = Preempted("twin-dpad is driving")
    nav.goal = None                     # the goal ended under the D-pad: the next set raises
    runner._return_trip.state = "goal"
    n = 0
    while runner.tick() and n < 50:
        n += 1
    s = runner.status()
    assert s["outcome"] == EXHAUSTED and s["return"]["state"] == "cancelled"
    assert "dpad" in s["return"]["reason"] or "preempted" in s["return"]["reason"]


def test_reserve_mid_return_ends_it_failed():
    runner, nav, grid, robot = _returning()
    nav.raise_on_set = BatteryReserve("battery_reserve: 10.15 V")
    runner._return_trip.state = "goal"
    n = 0
    while runner.tick() and n < 50:
        n += 1
    s = runner.status()
    assert s["outcome"] == EXHAUSTED
    assert s["return"] == {**s["return"], "state": "failed", "reason": "battery_reserve"}


# -- D3: the return reserve ---------------------------------------------------

def test_the_time_reserve_ends_the_search_early_and_docks_by_the_limit():
    runner, nav, grid, robot = mission(target="unicorn", mission_time_limit_s=60.0)
    s = run(runner)
    assert s["outcome"] == OUT_OF_BUDGET
    assert "return reserve" in s["end_reason"], s["end_reason"]
    docked_at_start(s, grid)
    assert grid.sim_time <= 60.0 + 60.0


def test_the_battery_reserve_ends_the_search_early():
    bat = SimBattery(v0=11.0, drain_v_per_s=0.004)
    runner, nav, grid, robot = mission(target="unicorn", house="home_first_floor", battery=bat,
                                       battery_floor_v=10.5)
    s = run(runner)
    assert s["outcome"] == OUT_OF_BUDGET
    assert "return reserve" in s["end_reason"] and "battery" in s["end_reason"], s["end_reason"]
    docked_at_start(s, grid)
    assert bat.true_v() > 10.2, "Reserve reached before docking"


def test_home_budget_units():
    dock = {"x_m": 1.0, "y_m": 1.0, "theta": 90.0, "source": "start"}
    b = HomeBudget(dock)
    assert b.v_obs() == 0.10                        # before 30 s of driving
    b.observe_pose({"usable": True, "x_m": 1.0, "y_m": 1.0}, 0.0)
    b.observe_pose({"usable": True, "x_m": 7.0, "y_m": 1.0}, 40.0)
    assert b.v_obs() == pytest.approx(0.15)
    b.observe_battery(12.0, 0.0)
    b.observe_battery(12.0, 10.0)                   # the same reading: not a sample
    b.observe_battery(11.9, 30.0)
    assert b.drain_v_s() == 0.0                     # under 60 s apart
    b.observe_battery(11.8, 100.0)
    assert b.drain_v_s() == pytest.approx(0.002)
    sx, sy = staging_point(dock)
    fx, fy = ahead(90.0)
    assert (fx, fy) == pytest.approx((1.0, 0.0))
    assert (sx, sy) == pytest.approx((0.7, 1.0))    # 0.30 m behind, facing east
