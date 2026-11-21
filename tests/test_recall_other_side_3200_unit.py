# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.200.C (R1): a recall whose turns to face it were all refused keeps its
place for one more goal, from another side of it
(docs/plans/ros-alignment/3.200-recall-other-side.md). Brought back from
3.200.B's `d82bcb5f` without R2: a navigator failure never keeps a place."""

import math

import pytest

import brain.explore as ex
from tests.test_recall_3172 import _far_floor, _runner


def _events(agent):
    out, note = [], agent._event

    def kept(name, pose=None, **fields):
        out.append({"event": name, **fields})
        note(name, pose, **fields)
    agent._event = kept
    return out


def _turns_refused(agent):
    verb = agent._verb

    def refused(action, **kw):
        if action in ("LEFT", "RIGHT"):
            return action, False, "pivot guard: a corner would come within 1.0 cm"
        return verb(action, **kw)
    agent._verb = refused


def _bearing(p, t):
    return math.atan2(p[1] - t[1], p[0] - t[0])


def _apart_deg(a, b, t):
    d = abs((_bearing(a, t) - _bearing(b, t) + math.pi) % (2 * math.pi) - math.pi)
    return math.degrees(d)


def _run(runner, n=400):
    for _ in range(n):
        runner.tick()
        kinds = [k for _, _, k in runner.agent.sent]
        if any(k not in ("recall",) for k in kinds) and len(kinds) >= 4 or runner.agent.exhausted:
            return kinds
    return [k for _, _, k in runner.agent.sent]


@pytest.fixture
def rule(monkeypatch):
    def set_(on):
        monkeypatch.setattr(ex, "RECALL_FACE_RETRY", on)
    return set_


def _place_with_two_sides(grid, nav, runner):
    """A reachable place whose band has floor on two sides at least
    RECALL_SIDE_DEG apart, on the map the agent sees."""
    pose = runner.agent._pose()
    here = (grid.robot_x, grid.robot_y)
    for y in range(grid.height):
        for x in range(grid.width):
            p = ((x + 0.5) * ex_cell(), (y + 0.5) * ex_cell())
            if math.hypot(x - here[0], y - here[1]) * ex_cell() < 2.0:
                continue
            first = runner.agent._band_point(pose, *p)
            if first is None:
                continue
            if runner.agent._band_point(pose, *p, away=[first]) is not None:
                return p
    raise AssertionError("no place with two sides")


def ex_cell():
    from tests.test_explore import CELL
    return CELL


def _reveal(grid, runner):
    """The whole house mapped, as a saved map is (3.175.B amendment B0)."""
    w = runner.world
    for _ in range(3):
        w.get_map()
    from sim.grid_world import CELL_WALL
    cells = getattr(w, "_cells", None)
    if cells is not None:
        for y in range(grid.height):
            for x in range(grid.width):
                cells[y * grid.width + x] = (100 if grid._cell(x, y) == CELL_WALL
                                             or (x, y) in grid.objects else 0)


def test_a_refused_face_keeps_the_place_for_one_goal_from_another_side(rule):
    rule(True)
    runner, grid, nav = _runner("washer")
    _reveal(grid, runner)
    place = _place_with_two_sides(grid, nav, runner)
    ev = _events(runner.agent)
    _turns_refused(runner.agent)
    runner.agent.recall_from([place])
    kinds = _run(runner)
    recalls = [e for e in ev if e["event"] == "goal_sent" and e["kind"] == "recall"]
    assert len(recalls) == 2 and kinds.count("recall") == 2      # N = 1 retry
    assert recalls[1]["side"] == 2 and "side" not in recalls[0]
    assert _apart_deg(recalls[0]["goal"], recalls[1]["goal"], place) >= ex.RECALL_SIDE_DEG - 1
    unlooked = [e for e in ev if e["event"] == "recall_unlooked"]
    assert [(u["why"], u["kept"]) for u in unlooked] == [("face", True), ("face", False)]
    assert runner.agent.recall == [] and len(kinds) > 2 and "recall" not in kinds[2:]


def test_dev_rule_spends_a_refused_face_as_before(rule):
    rule(False)
    runner, grid, nav = _runner("washer")
    _reveal(grid, runner)
    place = _place_with_two_sides(grid, nav, runner)
    ev = _events(runner.agent)
    _turns_refused(runner.agent)
    runner.agent.recall_from([place])
    kinds = _run(runner)
    assert kinds.count("recall") == 1
    assert [(u["why"], u["kept"]) for u in ev if u["event"] == "recall_unlooked"] == [("face", False)]


def test_a_face_that_turns_is_a_look_and_spends_the_place(rule):
    rule(True)
    runner, grid, nav = _runner("washer")
    lx, ly = _far_floor(grid, nav)
    ev = _events(runner.agent)
    runner.agent.recall_from([(lx, ly)])
    kinds = _run(runner)
    assert kinds.count("recall") == 1
    assert not [e for e in ev if e["event"] == "recall_unlooked"]


def test_no_floor_on_another_side_spends_the_place(rule):
    rule(True)
    runner, grid, nav = _runner("washer")
    lx, ly = _far_floor(grid, nav)
    a = runner.agent
    ev = _events(a)
    a.recall_from([(lx, ly)])
    # earlier tries on four sides: nothing is 90 degrees from all of them
    a._recall_tried[(lx, ly)] = [(lx + 0.5, ly), (lx, ly + 0.5), (lx - 0.5, ly), (lx, ly - 0.5)]
    assert a._band_point(a._pose(), lx, ly) is not None
    assert a._recall_goal(a.clock()) is None
    assert a.recall == [] and not a.sent
    assert [e["event"] for e in ev] == ["recall_set", "recall_spent"]


def test_thermos_a_tried_place_with_no_band_floor_at_all_is_spent(rule):
    """Thermos: kept, it held every later place behind it for the mission."""
    rule(True)
    runner, _, _ = _runner("washer")
    a = runner.agent
    a.recall_from([(-50.0, -50.0), (1.0, 1.0)])
    a._recall_tried[(-50.0, -50.0)] = [(-50.5, -50.0)]
    assert a._recall_goal(a.clock()) is None and a.recall == [(1.0, 1.0)]


def _failed_recall(a, state, place=(5.0, 5.0)):
    a._recall_tried[place] = [(5.5, 5.0)]
    pose = a._pose()
    a._goal = {"kind": "recall", "recall": place, "x_m": 5.5, "y_m": 5.0,
               "sent_at": a.clock(), "from": (pose["x_m"] - 2.0, pose["y_m"])}
    a._goal_ended(state, a.clock())
    return place


@pytest.mark.parametrize("state", ["aborted", "stalled", "rejected", "canceled", "lost"])
def test_r1_never_keeps_a_place_whose_goal_failed(rule, state):
    """R2 is not brought back (3.175.E and 3.200.B: measured twice, lost twice)."""
    rule(True)
    runner, _, _ = _runner("washer")
    a = runner.agent
    ev = _events(a)
    _failed_recall(a, state)
    assert a.recall == [] and not [e for e in ev if e["event"] == "recall_unlooked"]


def test_r1_does_not_keep_a_place_whose_goal_failed(rule):
    rule(True)
    runner, _, _ = _runner("washer")
    _failed_recall(runner.agent, "aborted")
    assert runner.agent.recall == []


def test_the_cap_two_tries_per_place(rule):
    rule(True)
    runner, _, _ = _runner("washer")
    a = runner.agent
    a._recall_tried[(1.0, 1.0)] = [(1.5, 1.0), (1.0, 1.5)]
    a._recall_unlooked((1.0, 1.0))
    assert a.recall == []
    assert ex.RECALL_SIDES == 2


def test_a_charge_trip_does_not_spend_a_side(rule):
    rule(True)
    runner, _, _ = _runner("washer")
    a = runner.agent
    a._recall_tried[(1.0, 1.0)] = [(1.5, 1.0)]
    a._goal = {"kind": "recall", "recall": (1.0, 1.0), "x_m": 1.5, "y_m": 1.0, "sent_at": 0.0,
               "recall_spot": (1.5, 1.0)}
    a.set_aside()
    assert a.recall == [(1.0, 1.0)] and a._recall_tried[(1.0, 1.0)] == []


def test_other_side_is_ninety_degrees_round_the_place():
    t = (0.0, 0.0)
    assert ex.ExploreAgent._other_side(0.0, 0.5, *t, [(0.5, 0.0)])
    assert ex.ExploreAgent._other_side(-0.5, 0.0, *t, [(0.5, 0.0)])
    assert not ex.ExploreAgent._other_side(0.4, 0.3, *t, [(0.5, 0.0)])
    assert ex.ExploreAgent._other_side(0.4, 0.3, *t, [(0.0, 0.0)])   # a spot on the place has no side


def test_review_unknown_band_floor_waits_for_another_side_then_spends(rule, monkeypatch):
    """Review: floor on the other side not mapped yet is waited for,
    RECALL_SIDE_WAIT_S at most, then the place is spent."""
    rule(True)
    runner, grid, nav = _runner("washer")
    lx, ly = _far_floor(grid, nav)
    a = runner.agent
    a.recall_from([(lx, ly)])
    a._recall_tried[(lx, ly)] = [(lx + 0.5, ly), (lx, ly + 0.5), (lx - 0.5, ly), (lx, ly - 0.5)]
    t = [100.0]
    a.clock = lambda: t[0]
    a._recall_unlooked_at[(lx, ly)] = 100.0
    monkeypatch.setattr(a, "_band_unknown", lambda x, y: True)
    assert a._recall_goal(t[0]) is None and a.recall == [(lx, ly)]
    t[0] += ex.RECALL_SIDE_WAIT_S
    assert a._recall_goal(t[0]) is None and a.recall == []


def test_review_band_unknown_reads_the_map():
    runner, grid, nav = _runner("washer")
    a = runner.agent
    m = runner.world.get_map()
    w = m["width"]
    i = int((2.0 - m["origin_x_m"]) / m["resolution_m"])
    j = int((2.0 - m["origin_y_m"]) / m["resolution_m"])
    unknown = [-1] * len(m["cells"])
    a._last_map = dict(m, cells=unknown)
    assert not a._band_unknown(2.0, 2.0)          # unknown with no floor beside it: no frontier
    a._last_map = dict(m, cells=[100] * len(m["cells"]))
    assert not a._band_unknown(2.0, 2.0)          # Thermos: unknown behind walls is not a side
    edge = list(unknown)
    edge[j * w + i] = 0                            # floor at the place, unknown round it
    a._last_map = dict(m, cells=edge)
    assert a._band_unknown(2.0, 2.0)
    a._last_map = dict(m, cells=[0] * len(m["cells"]))
    assert not a._band_unknown(2.0, 2.0)


def test_review_a_charge_trip_restores_a_queued_recall_face(rule):
    rule(True)
    runner, _, _ = _runner("washer")
    a = runner.agent
    a._recall_tried[(1.0, 1.0)] = [(1.5, 1.0)]
    a._pending = [("FACE", (1.0, 1.0), 90, False, "recall")]
    a.set_aside()
    assert a.recall == [(1.0, 1.0)] and a._recall_tried[(1.0, 1.0)] == [] and a._pending == []


def test_off_records_no_tries_and_keeps_no_queued_face(rule):
    """Off is `dev`: no side is remembered, a charge trip puts back no queued
    face, and a refused face is only logged (3.200.C counts them)."""
    rule(False)
    runner, _, _ = _runner("washer")
    a = runner.agent
    ev = _events(a)
    a._pending = [("FACE", (1.0, 1.0), 90, False, "recall")]
    a.set_aside()
    assert a.recall == [] and a._recall_tried == {}
    a._recall_unlooked((1.0, 1.0))
    assert a.recall == []
    assert [(e["why"], e["kept"]) for e in ev if e["event"] == "recall_unlooked"] == [("face", False)]
