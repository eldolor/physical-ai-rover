# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.175.E review: a recall goal ended by a charge trip (`set_aside`) is
forgotten uncounted, so its place goes back to the recall list
(docs/plans/ros-alignment/3.175-memory-when-things-move.md)."""

from tests.test_recall_3172 import _far_floor, _runner


def test_a_recall_goal_ended_by_charging_keeps_its_place():
    runner, grid, nav = _runner("washer")
    agent = runner.agent
    place = _far_floor(grid, nav)
    agent.recall_from([place])
    out = agent._recall_goal(agent.clock())
    assert out[0] == "GOAL" and agent.recall == []
    agent.set_aside()
    assert agent.recall == [place] and agent._goal is None
    out = agent._recall_goal(agent.clock())
    assert out[0] == "GOAL" and agent.recall == []


def test_set_aside_without_a_recall_goal_adds_nothing():
    runner, grid, nav = _runner("washer")
    agent = runner.agent
    agent.set_aside()
    assert agent.recall == []
