# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.172.C (F5): a find goes first to where earlier missions on the same saved
map put the target (docs/plans/ros-alignment/3.172-autonomy-rehearsal-fixes.md)."""

import math

from brain.explore import APPROACH_FAR_M, RECALL_MAX
from brain.inventory import REPORT_BELIEF
from control.mission_runner import MissionRunner
from sim.grid_world import CELL_WALL
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import CELL, FakeNav


def _runner(target="washer"):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=MockWorld(grid),
                           target_object=target, **({} if target else {"job": "survey"}))
    runner.start()
    return runner, grid, nav


def _far_floor(grid, nav, min_m=3.0):
    """A floor cell the robot can reach, at least `min_m` away, with nothing on it."""
    here = (grid.robot_x, grid.robot_y)
    for y in range(grid.height):
        for x in range(grid.width):
            if (grid._cell(x, y) != CELL_WALL and (x, y) not in grid.objects
                    and math.hypot(x - here[0], y - here[1]) * CELL >= min_m
                    and nav._connected(here, (x, y))):
                return (x + 0.5) * CELL, (y + 0.5) * CELL
    raise AssertionError("no reachable floor")


def _report(rows, frame_id="f1"):
    return {"frame_id": frame_id, "reported": rows, "candidates": []}


def _row(label, x, y, belief=0.98):
    return {"label": label, "x_m": x, "y_m": y, "belief": belief, "votes": {label: 3}}


def test_carried_landmarks_named_as_the_target_go_to_explore_best_first():
    runner, _, _ = _runner("washer")
    runner.inventory.seed("f1", _report([
        _row("washer", 1.0, 1.0, 0.85), _row("dryer", 2.0, 2.0),
        _row("washer", 3.0, 3.0, 0.98), _row("washer", 4.0, 4.0, REPORT_BELIEF - 0.1)]))
    assert runner._recall_carried() == 2
    assert runner.agent.recall == [(3.0, 3.0), (1.0, 1.0)]


def test_at_most_recall_max():
    runner, _, _ = _runner("washer")
    runner.inventory.seed("f1", _report([_row("washer", float(i), 1.0) for i in range(RECALL_MAX + 2)]))
    assert runner._recall_carried() == RECALL_MAX


def test_a_survey_and_nothing_carried_recall_nothing():
    runner, _, _ = _runner(None)
    runner.inventory.seed("f1", _report([_row("washer", 1.0, 1.0)]))
    assert runner._recall_carried() == 0 and runner.agent.recall == []
    runner, _, _ = _runner("washer")
    assert runner._recall_carried() == 0 and runner.agent.recall == []


def test_this_missions_own_landmarks_are_not_recalled():
    runner, _, _ = _runner("washer")
    runner.inventory.seed("f1", _report([_row("washer", 1.0, 1.0)]))
    runner.inventory.landmarks[0].carried = False
    assert runner._recall_carried() == 0


def test_the_first_goal_is_the_recall_and_a_target_not_there_costs_one_goal():
    """C5: a landmark the target has left: one goal, a look, then frontiers --
    never another recall, never an end."""
    runner, grid, nav = _runner("washer")
    lx, ly = _far_floor(grid, nav)
    runner.agent.recall_from([(lx, ly)])
    kinds = []
    for _ in range(200):
        runner.tick()
        kinds = [k for _, _, k in runner.agent.sent]
        if "frontier" in kinds or runner.agent.exhausted:
            break
    assert kinds[0] == "recall"
    first = next(c for c in nav.calls if c[0] == "set")
    assert math.hypot(first[1] - lx, first[2] - ly) <= APPROACH_FAR_M + CELL
    assert kinds.count("recall") == 1 and "frontier" in kinds
    assert runner.agent.recall == [] and not runner.agent.exhausted
