# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_explore_approach_3191.py

3.191: explore's approach livelock (3.70.M2 run 1, unsurveyed k 19). A
sighting 0.70 m out sits just past `_at_target()`'s band, `_band_point`
answers the robot's own cell (its centre in band), nav2 ends that goal
`succeeded` at once without moving, and `_goal_ended` cleared the approach
and charged nothing -- so the same goal went out on every other tick, 4,657
times in 1,200 s. Since 3.191 an approach that succeeds without leaving
(`STUCK_MOVED_M`) is a retry failure of its sighting.
"""

import math

from brain.explore import STUCK_MOVED_M, ExploreAgent
from brain.frontier import RETRY_LIMIT, RETRY_RADIUS_M
from brain.memory import MissionMemory
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav

CELL = 0.30


class ToleranceNav(FakeNav):
    """nav2 with an xy goal tolerance: a goal within it of the robot ends
    `succeeded` on the next poll and the robot does not move (what the
    stand-in nav, `tests/dock_sweep.inflated_nav_class`, did in k 19);
    others as `FakeNav`."""

    TOL_M = 0.25

    def get_goal(self):
        g = self.goal
        if g and g["state"] == "active":
            here = (self.grid.x * CELL, self.grid.y * CELL)
            if math.hypot(g["x_m"] - here[0], g["y_m"] - here[1]) <= self.TOL_M:
                self.calls.append(("get",))
                self.robot.pass_time(0.5)
                g["state"] = "succeeded"
                return {"goal": g, "plan": []}
        return super().get_goal()


def _k19_geometry():
    """scaled_house's start cell (6, 4), centre (1.95, 1.35): the robot
    0.12 m east of the centre, the sighting 0.58 m west of it -- 0.70 m from
    the robot (just past `_at_target`), the cell centre in the band."""
    grid = build_world("scaled_house")
    cx, cy = 6.5 * CELL, 4.5 * CELL
    grid.x, grid.y, grid.theta = (cx + 0.12) / CELL, cy / CELL, math.pi
    robot = MockRobot(grid, render=False)
    nav = ToleranceNav(robot)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="shoes"),
                         navigator=nav, clock=lambda: grid.sim_time, world=MockWorld(grid))
    sight = (cx - 0.5801, cy)
    agent._glimpse = lambda frame: None
    agent._pending = []
    return grid, nav, agent, sight


def test_the_geometry_is_k19s():
    grid, nav, agent, sight = _k19_geometry()
    pose = agent._pose()
    d = math.hypot(sight[0] - pose["x_m"], sight[1] - pose["y_m"])
    assert d > 0.70 and not agent._at_target(*sight)
    band = agent._band_point(pose, *sight)
    assert band is not None and math.hypot(band[0] - pose["x_m"], band[1] - pose["y_m"]) < STUCK_MOVED_M


def test_an_approach_that_succeeds_without_moving_runs_out_of_tries():
    """Red on dev 0c6da335: the sighting stays in view every frame, and the
    same in-place approach went out every other step (about 100 in 200)."""
    grid, nav, agent, sight = _k19_geometry()
    agent._sighting = lambda frame: sight
    approaches = []
    for _ in range(200):
        r = agent.step()
        if r.action == "GOAL" and agent._goal and agent._goal["kind"] == "approach":
            approaches.append(agent._goal["target"])
    near = [t for t in approaches if math.dist(t, sight) <= RETRY_RADIUS_M]
    assert 1 <= len(near) <= RETRY_LIMIT
    # Each in-place success is one failure of the sighting (cooled 30 s, so
    # 200 steps of a few sim seconds see at most a couple).
    assert agent.retry.failures_near(*sight) == len(near)
    assert not agent.retry.available(*sight, grid.sim_time, approach=True)


def test_an_approach_that_moves_and_succeeds_charges_nothing():
    """Progress is not a failure: a real approach that drives there and
    succeeds leaves the retry book as it was (dev's behaviour)."""
    grid = build_world("scaled_house")
    grid.x, grid.y, grid.theta = 6.5, 4.5, 0.0
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="shoes"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    pose = agent._pose()
    target = (pose["x_m"] + 1.5, pose["y_m"])
    out = agent._send_approach(*target, grid.sim_time)
    assert out[0] == "GOAL"
    agent.step()                                  # FakeNav: there on this poll
    assert agent.goals_reached and agent.goals_reached[-1][2] == "approach"
    assert agent.retry.entries == []
