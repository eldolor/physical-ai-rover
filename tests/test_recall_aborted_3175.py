# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.175.E: a recall goal that ends without arriving goes again once, from
another approach point, before the place is spent
(docs/plans/ros-alignment/3.175-memory-when-things-move.md)."""

import math

from brain.explore import RECALL_TRIES
from tests.test_explore import FakeNav
from tests.test_recall_3172 import _far_floor, _runner


class AbortingNav(FakeNav):
    """FakeNav whose first `abort` accepted goals abort where the robot stands
    (nav2's ABORTED: no path, or the controller gave up)."""

    def __init__(self, robot, abort=1):
        super().__init__(robot)
        self.abort = abort

    def get_goal(self):
        if self.goal and self.goal["state"] == "active" and self.abort:
            self.calls.append(("get",))
            self.abort -= 1
            self.robot.pass_time(2.0)
            self.goal["state"] = "aborted"
            return {"goal": self.goal, "plan": []}
        return super().get_goal()


def _mapped(runner, grid):
    """The whole house on the map, as a saved map's reload gives it (3.175.B
    amendment B0), so another approach point exists."""
    from sim.grid_world import CELL_WALL
    from sim.mock_world import CELL_FREE, CELL_OCCUPIED
    w = runner.agent.world
    w._cells = [CELL_OCCUPIED if grid._cell(x, y) == CELL_WALL or (x, y) in grid.objects
                else CELL_FREE for y in range(grid.height) for x in range(grid.width)]


def _setup(abort, mapped=True):
    runner, grid, nav = _runner("washer")
    if mapped:
        _mapped(runner, grid)
    nav = AbortingNav(runner.robot, abort=abort)
    runner.agent.navigator = nav
    lx, ly = _far_floor(grid, nav)
    runner.agent.recall_from([(lx, ly)])
    return runner, nav, (lx, ly)


def _run(runner, n=300):
    for _ in range(n):
        runner.tick()
        kinds = [k for _, _, k in runner.agent.sent]
        if "frontier" in kinds or runner.agent.exhausted:
            break
    return [k for _, _, k in runner.agent.sent]


def _events(runner, name):
    return [e for e in runner.agent.events if e.get("event") == name]


def test_an_aborted_recall_goes_again_from_another_approach_point():
    runner, nav, place = _setup(abort=1)
    kinds = _run(runner)
    assert kinds[:2] == ["recall", "recall"] and kinds.count("recall") == 2
    sets = [c for c in nav.calls if c[0] == "set"][:2]
    from brain.explore import RECALL_OTHER_M
    assert math.dist(sets[0][1:], sets[1][1:]) >= RECALL_OTHER_M - 0.01
    again = _events(runner, "recall_again")
    assert len(again) == 1 and again[0]["state"] == "aborted"
    assert runner.agent.recall == []


def test_tries_are_capped():
    runner, nav, place = _setup(abort=5)
    kinds = _run(runner)
    assert kinds.count("recall") == RECALL_TRIES == 2
    assert kinds[:2] == ["recall", "recall"] and runner.agent.recall == []


def test_with_no_other_approach_point_the_best_goes_again():
    runner, nav, place = _setup(abort=1)
    agent = runner.agent
    first = agent._band_point(agent._pose(), *place)
    agent._recall_tried[place] = [place]           # every band point is too near
    out = agent._recall_goal(agent.clock())
    assert out[0] == "GOAL" and agent.recall == []
    assert agent._recall_tried[place][-1] == first


def test_a_place_with_no_band_floor_yet_is_kept_after_a_try():
    runner, nav, place = _setup(abort=1, mapped=False)
    agent = runner.agent
    agent.recall = [(-50.0, -50.0)]                # off every map
    agent._recall_tried[(-50.0, -50.0)] = [(-50.0, -49.5)]
    assert agent._recall_goal(agent.clock()) is None and agent.recall == [(-50.0, -50.0)]


def test_a_recall_that_arrives_is_one_goal():
    runner, nav, place = _setup(abort=0)
    kinds = _run(runner)
    assert kinds.count("recall") == 1 and not _events(runner, "recall_again")
