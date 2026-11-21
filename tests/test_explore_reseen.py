# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.192.B amendment 3 (C1, C9): a find that saw the target and lost it looks
again where it saw it before it ends `exhausted`.

* in process, on 3.161's sweep harness (`evaluations/nogo-3161/sweep.py`),
  `scaled_house` s0's no-zone find: red with the rounds held at 0 (`dev`), found with C1;
* on a small room known and seen end to end (no frontier, no view gap), the
  rule itself: places kept one per retry radius, rounds bounded, a second
  round only on sightings since, a survey never, a place whose identity was
  refused never.
"""

import importlib.util
from pathlib import Path

import pytest

import brain.explore as explore_mod
from brain.explore import ExploreAgent
from brain.memory import MissionMemory
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests.test_explore import FakeNav
from world.interface import CELL_FREE, CELL_OCCUPIED

ROOT = Path(__file__).resolve().parents[1]
RES, W, H = 0.05, 40, 28


def _sweep():
    spec = importlib.util.spec_from_file_location(
        "nogo_sweep_3161_reseen", ROOT / "evaluations" / "nogo-3161" / "sweep.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_red_without_c1_a_seen_target_ends_exhausted(monkeypatch):
    monkeypatch.setattr(explore_mod, "RESEEN_ROUNDS", 0)       # dev before 3.192.B
    row = _sweep().run("scaled_house", 0, "none")
    assert row["outcome"] == "exhausted"


def test_a_seen_target_lost_is_looked_at_again_and_found():
    row = _sweep().run("scaled_house", 0, "none")
    assert row["outcome"] == "found"
    assert row["goals_in_zone"] == [] and row["entries"] == []


# ---------- the rule, on a room known and seen end to end ----------

class _World:
    def __init__(self):
        self.m = {"usable": True, "width": W, "height": H, "resolution_m": RES,
                  "origin_x_m": 0.0, "origin_y_m": 0.0, "map_id": "m1", "map_version": 1,
                  "cells": [CELL_OCCUPIED if j in (0, H - 1) or i in (0, W - 1) else CELL_FREE
                            for j in range(H) for i in range(W)]}

    def get_map(self):
        return self.m

    def get_pose(self):
        return {"usable": True, "x_m": 0.5, "y_m": 0.7, "heading_deg": 0.0, "frame_id": "f"}


def _agent(target="red backpack", room=None):
    robot = MockRobot(build_world("scaled_house"), render=False)
    world = _World()
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object=target, target_room=room),
                         navigator=FakeNav(robot), clock=lambda: 0.0, world=world)
    agent._pose = world.get_pose
    agent.seen = {(RES * (i + 0.5), RES * (j + 0.5)) for j in range(H) for i in range(W)}
    events = []
    note = agent._event
    agent._event = lambda name, pose=None, **f: (events.append((name, f)), note(name, pose, **f))[1]
    return agent, events


def test_with_nothing_seen_the_search_ends_as_on_dev():
    agent, events = _agent()
    assert agent._choose_next(0.0)[0] == "STOP"
    assert [n for n, _ in events] == ["exhausted"]


def test_a_place_seen_is_looked_at_again_before_the_end():
    agent, events = _agent()
    agent._note_seen_at(1.5, 0.7)
    action, executed, detail = agent._choose_next(0.0)
    assert (action, executed) == ("WAIT", True) and "looking again" in detail
    assert events[-1] == ("reseen", {"round": 1, "places": [[1.5, 0.7]]})
    assert agent.recall == [(1.5, 0.7)]
    out = agent._recall_goal(0.0)                         # 3.172's recall goal: beside it
    assert out[0] == "GOAL" and agent._goal["kind"] == "recall"
    gx, gy = agent._goal["x_m"], agent._goal["y_m"]
    assert explore_mod.APPROACH_NEAR_M <= ((gx - 1.5) ** 2 + (gy - 0.7) ** 2) ** 0.5 \
        <= explore_mod.APPROACH_FAR_M


def test_places_are_one_per_retry_radius_latest_first_and_capped():
    agent, events = _agent()
    r = agent.retry.radius_m
    agent._note_seen_at(0.5, 0.5)
    agent._note_seen_at(0.5 + r / 2, 0.5)                # the same place, seen again: kept once, latest
    for k in range(1, 5):
        agent._note_seen_at(0.5 + k * (r + 0.01), 0.5 + 2 * r)
    assert len(agent._seen_at) == 5
    assert agent._seen_at[0] == (0.5 + r / 2, 0.5)
    agent._choose_next(0.0)
    places = events[-1][1]["places"]
    assert len(places) == explore_mod.RECALL_MAX
    assert places[0] == [round(0.5 + 4 * (r + 0.01), 2), round(0.5 + 2 * r, 2)]   # the latest first


def test_a_second_round_only_on_new_sightings_and_never_a_third():
    agent, events = _agent()
    agent._note_seen_at(1.5, 0.7)
    assert agent._choose_next(0.0)[0] == "WAIT"
    agent.recall = []                                      # the round's goals spent
    assert agent._choose_next(0.0)[0] == "STOP"            # nothing seen since: the end
    agent, events = _agent()
    agent._note_seen_at(1.5, 0.7)
    agent._choose_next(0.0)
    agent.recall = []
    agent._note_seen_at(1.0, 1.0)                          # seen again during the round
    assert agent._choose_next(0.0)[0] == "WAIT"
    assert events[-1][1]["round"] == 2
    agent.recall = []
    agent._note_seen_at(1.2, 0.4)
    assert agent._choose_next(0.0)[0] == "STOP"            # RESEEN_ROUNDS (2) spent
    assert [n for n, _ in events].count("reseen") == explore_mod.RESEEN_ROUNDS == 2


def test_a_survey_never_looks_again():
    agent, events = _agent(target=None)
    agent._note_seen_at(1.5, 0.7)
    assert agent._choose_next(0.0)[0] == "STOP"
    assert "reseen" not in [n for n, _ in events]


@pytest.mark.parametrize("handed", [False, True])
def test_a_place_whose_identity_was_refused_is_never_looked_at_again(handed):
    from brain.arrival import REFUSED
    agent, events = _agent()
    agent.sight_from = "perception"
    agent._note_seen_at(1.5, 0.7)
    agent._note_seen_at(0.6, 1.0)
    agent.recall_from([(0.4, 0.4)])                        # a carried place stays
    if handed:
        agent._choose_next(0.0)                            # the round already holds it
        assert (1.5, 0.7) in agent.recall
    agent._last_seen = (1.5, 0.7)
    agent._note_refusal({"_arrival": {"state": REFUSED}})
    assert (1.5, 0.7) not in agent._seen_at and (1.5, 0.7) not in agent.recall
    assert (0.4, 0.4) in agent.recall
    if not handed:
        agent._choose_next(0.0)
        assert events[-1][0] == "reseen" and [1.5, 0.7] not in events[-1][1]["places"]
