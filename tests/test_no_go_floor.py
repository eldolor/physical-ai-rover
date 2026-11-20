# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.192 (F7): a person's no-go zone is a wall for reach only. What explore
understands of the map -- frontiers' cells, the room cut, the goal features --
reads the map without the zones; every goal still keeps out of a zone grown
by the robot's half-width (3.161 D3, amended in 3.192).

The map: 4.0 x 1.4 m at 0.05 m, walls round it, floor west of x = 2.0 and
unknown east of it, so one frontier (1.3 m) runs along x = 1.95-2.0.
"""

import pytest

import brain.explore as explore_mod
from brain.explore import ExploreAgent
from brain.goal_policy import goal_policy_for
from brain.memory import MissionMemory
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests.test_explore import FakeNav
from world import nogo
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN

RES, W, H = 0.05, 80, 28
FRAME = "map-floor"


def _map():
    cells = []
    for j in range(H):
        for i in range(W):
            if j in (0, H - 1) or i in (0, W - 1):
                cells.append(CELL_OCCUPIED)
            elif i < 40:
                cells.append(CELL_FREE)
            else:
                cells.append(CELL_UNKNOWN)
    return {"usable": True, "width": W, "height": H, "resolution_m": RES,
            "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": cells,
            "map_id": "m1", "map_version": 1}


class _World:
    def __init__(self):
        self.m = _map()

    def get_map(self):
        return self.m

    def get_pose(self):
        return {"usable": True, "x_m": 0.5, "y_m": 0.7, "heading_deg": 0.0,
                "frame_id": FRAME}


def _agent(zones=(), policy=None):
    robot = MockRobot(build_world("scaled_house"), render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=FakeNav(robot), clock=lambda: 0.0, world=_World(),
                         goal_policy=policy)
    agent.set_zones([nogo.clean_zone(z) for z in zones])
    return agent


def _frontiers_of(agent, monkeypatch):
    """The frontiers explore's choice works with (whatever map it reads them
    off), and the goal it sends."""
    got = []
    real = explore_mod.find_frontiers

    def spy(*a, **k):
        out = real(*a, **k)
        got.append(out)
        return out
    monkeypatch.setattr(explore_mod, "find_frontiers", spy)
    agent._choose_next(0.0)
    monkeypatch.setattr(explore_mod, "find_frontiers", real)
    assert got, "explore made no frontier search"
    goal = agent._goal and (agent._goal["x_m"], agent._goal["y_m"])
    # The caller's own filter (a frontier wholly inside a zone) runs after.
    return got[-1], goal


def _in_grown(zones, x, y):
    return nogo.zone_at([nogo.clean_zone(z) for z in zones], x, y, nogo.HALF_WIDTH_M) is not None


def test_a_zone_cutting_a_frontier_keeps_its_zone_free_middle_and_size(monkeypatch):
    """F7 (b): a zone over the frontier's south end. On `dev` the frontier was
    read off the masked map: cut short, its middle and goal moved north. Now
    it is the zone-free frontier, its goal outside the grown zone."""
    free, _ = _frontiers_of(_agent(), monkeypatch)
    zone = {"x0": 1.85, "y0": 0.05, "x1": 2.30, "y1": 0.45}
    agent = _agent([zone])
    cut, goal = _frontiers_of(agent, monkeypatch)
    assert len(free) == 1 and len(cut) == 1
    assert cut[0].size_m == pytest.approx(free[0].size_m)
    assert sorted(cut[0].cells) == sorted(free[0].cells)
    assert cut[0].goal == pytest.approx(free[0].goal)
    assert not _in_grown([zone], *cut[0].goal)
    assert goal is not None and not _in_grown([zone], *goal)


def test_a_frontiers_goal_never_lies_in_a_grown_zone(monkeypatch):
    """D3's core, unchanged: the zone-free goal (the reachable cell nearest the
    middle) would lie in this zone; the goal moves to a stop outside it."""
    free, _ = _frontiers_of(_agent(), monkeypatch)
    gx, gy = free[0].goal
    zone = {"x0": gx - 0.25, "y0": gy - 0.25, "x1": gx + 0.25, "y1": gy + 0.25}
    agent = _agent([zone])
    got, goal = _frontiers_of(agent, monkeypatch)
    assert got and all(not _in_grown([zone], *f.goal) for f in got)
    assert goal is None or not _in_grown([zone], *goal)


def test_a_frontier_wholly_inside_a_grown_zone_is_dropped():
    """F7 (c): every frontier cell inside the grown zone -- no goal for it.
    (`dev` drops it too, by masking its cells; this pins the drop.)"""
    zone = {"x0": 1.85, "y0": 0.05, "x1": 2.30, "y1": 1.35}
    agent = _agent([zone])
    agent._choose_next(0.0)
    sent = [e for e in agent.events if e["event"] == "goal_sent"]
    assert not [e for e in sent if e.get("kind") == "frontier"]


def test_the_features_read_the_map_without_the_zones(monkeypatch):
    """F7 (a): a zone over unknown floor 0.5 m past the frontier holds no
    candidate and cuts no frontier. Every pooled candidate's features (goal
    and room features, the shipped `rooms` chooser) equal those with no zone.
    On `dev` the masked cells read as neither unknown nor free (and as walls
    to the room cut), so `unknown_near` and the room features changed."""
    policy = goal_policy_for("rooms")

    def feats(zones):
        agent = _agent(zones, policy=policy)
        got = []
        real = agent._pool

        def spy(*a, **k):
            pool, f = real(*a, **k)
            got.append(sorted((tuple(round(v, 6) for v in c.goal), tuple(round(v, 9) for v in x))
                              for (_, c, _), x in zip(pool, f)))
            return pool, f
        agent._pool = spy
        agent._choose_next(0.0)
        assert got, "no pool was built"
        return got[-1]

    zone = {"x0": 2.55, "y0": 0.40, "x1": 3.00, "y1": 1.00}
    without, with_zone = feats([]), feats([zone])
    assert without, "no candidate"
    assert with_zone == without


def test_with_no_zones_the_choice_reads_one_map():
    """No zones: the floor map is the map itself -- nothing changes."""
    agent = _agent()
    agent._choose_next(0.0)
    assert agent._floor_of is None
    m = agent.world.get_map()
    assert agent._floor(m) is m

