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


def _k19_geometry(east_m=0.12):
    """scaled_house's start cell (6, 4), centre (1.95, 1.35): the robot
    `east_m` east of the centre, the sighting 0.58 m west of it -- with 0.12,
    0.70 m from the robot (just past `_at_target`), the cell centre in the band."""
    grid = build_world("scaled_house")
    cx, cy = 6.5 * CELL, 4.5 * CELL
    grid.x, grid.y, grid.theta = (cx + east_m) / CELL, cy / CELL, math.pi
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
    near = _approaches(agent, sight)
    assert 1 <= len(near) <= RETRY_LIMIT
    # Each in-place success is one failure of the sighting (cooled 30 s, so
    # 200 steps of a few sim seconds see at most a couple).
    assert agent.retry.failures_near(*sight) == len(near)
    assert not agent.retry.available(*sight, grid.sim_time, approach=True)


def _approaches(agent, sight, steps=200):
    approaches = []
    for _ in range(steps):
        r = agent.step()
        if r.action == "GOAL" and agent._goal and agent._goal["kind"] == "approach":
            approaches.append(agent._goal["target"])
    while agent._goal is not None and agent._goal["kind"] == "approach":
        agent.step()                  # review: the last one ended, so charged or kept
    return [t for t in approaches if math.dist(t, sight) <= RETRY_RADIUS_M]


def test_an_in_place_approach_inside_the_band_closes_in_not_charged():
    """Review: 0.62 m out the robot is within `_at_target`; the in-place
    success keeps the approach on record, so the next frames take 3.106's
    in-sight-and-unfound path (bounded by CLOSE_IN_REFUSED, then one retry
    failure and a back-off), not another goal. Red on dev too (re-sent)."""
    from brain.explore import CLOSE_IN_REFUSED
    grid, nav, agent, sight = _k19_geometry(east_m=0.04)
    pose = agent._pose()
    assert agent._at_target(*sight)
    assert math.hypot(sight[0] - pose["x_m"], sight[1] - pose["y_m"]) > 0.55
    agent._sighting = lambda frame: sight
    agent.step()                                   # the approach goal
    assert agent._goal and agent._goal["kind"] == "approach"
    agent.step()                                   # succeeded in place
    assert agent._approach == sight and agent.retry.entries == []
    # 3.106: CLOSE_IN_REFUSED ticks in sight and unfound, then one failure
    # of the sighting and a back-off -- not a goal.
    for _ in range(CLOSE_IN_REFUSED):
        r = agent.step()
        assert r.action != "GOAL"
    assert agent.retry.failures_near(*sight) == 1 and agent._approach is None
    assert "REVERSE" in [r.action, *agent._pending]
    near = _approaches(agent, sight)
    assert 1 + len(near) <= RETRY_LIMIT


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


# ---- the sweep's records, pinned (3.191.A; rows in evaluations/explore-approach-3191/) ----

def _rows(name):
    import json
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / "evaluations" / "explore-approach-3191" / name
    with open(path) as f:
        return {r["k"]: r for r in (json.loads(x) for x in f if x.strip())}


def test_the_sweep_records_hold():
    for nav, found_fix, found_dev in (("inflated", 6, 5), ("fake", 20, 20)):
        fix, dev = _rows(f"rows-fix-{nav}.jsonl"), _rows(f"rows-dev-{nav}.jsonl")
        assert len(fix) == len(dev) == 20
        assert max(r["no_move_max_per_place"] for r in fix.values()) <= RETRY_LIMIT
        assert sum(r["outcome"] == "found" and not r["wrong_found"] for r in fix.values()) == found_fix
        assert sum(r["outcome"] == "found" and not r["wrong_found"] for r in dev.values()) == found_dev
        assert not any(r["wrong_found"] or r["contacts"] for r in fix.values())
        assert [k for k in fix if fix[k]["outcome"] != dev[k]["outcome"]] == (
            [19] if nav == "inflated" else [])
    k19, k19_dev = _rows("rows-fix-inflated.jsonl")[19], _rows("rows-dev-inflated.jsonl")[19]
    assert k19_dev["goals"] == 4662 and k19_dev["no_move_max_per_place"] == 4657
    assert k19["outcome"] == "found" and k19["goals"] <= 300 and k19["no_move_max_per_place"] <= 3
    assert k19["distance_m"] < 10.0            # criterion 3 FAILED as written: found first
