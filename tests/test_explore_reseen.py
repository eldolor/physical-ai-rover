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
RES, W, H = 0.05, 80, 56   # a 4.0 x 2.8 m room


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
    assert events[-1] == ("reseen", {"round": 1, "kept_of": 1, "places": [[1.5, 0.7]]})
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


def test_a_refused_object_seen_again_is_still_never_looked_at_again():
    """Review: the detector keeps firing on the object the cloud refused."""
    from brain.arrival import REFUSED
    agent, events = _agent()
    agent.sight_from = "perception"
    agent._note_seen_at(1.5, 0.7)
    agent._last_seen = (1.5, 0.7)
    agent._note_refusal({"_arrival": {"state": REFUSED}})
    agent._note_seen_at(1.52, 0.71)                        # the same chair leg, next frame
    assert agent._seen_at == []
    assert agent._choose_next(0.0)[0] == "STOP"
    assert "reseen" not in [n for n, _ in events]


def test_a_refusal_during_a_look_again_goal_takes_its_place():
    """Review: a refusal with no sighting on record (`_last_seen` and
    `_approach` None) names the look-again goal's own place."""
    from brain.arrival import REFUSED
    agent, events = _agent()
    agent.sight_from = "perception"
    agent._note_seen_at(1.5, 0.7)
    agent._choose_next(0.0)
    assert agent._recall_goal(0.0)[0] == "GOAL"
    agent._last_seen = agent._approach = None
    agent._note_refusal({"_arrival": {"state": REFUSED}})
    agent._note_seen_at(1.5, 0.7)
    assert agent._seen_at == []
    agent.set_aside()                                      # a charge trip: not put back
    assert (1.5, 0.7) not in agent.recall


def test_a_place_with_no_floor_beside_it_does_not_hold_the_round():
    """Review: a place no reachable floor stands beside is left out, so the
    others still get their look."""
    agent, events = _agent()
    agent._note_seen_at(1.0, 0.7)
    agent._note_seen_at(9.0, 9.0)                          # off the map: no floor beside it
    agent._choose_next(0.0)
    assert events[-1][0] == "reseen"
    assert events[-1][1]["places"] == [[1.0, 0.7]] and events[-1][1]["kept_of"] == 2
    agent, events = _agent()
    agent._note_seen_at(9.0, 9.0)
    assert agent._choose_next(0.0)[0] == "STOP"            # none reachable: no round spent
    assert agent._reseen == 0


# ---------- amendment 3's sweep, pinned (evaluations/nogo-3192/fix/summary.json) ----------

def _summary():
    import json
    return json.loads((ROOT / "evaluations" / "nogo-3192" / "fix" / "summary.json").read_text())


def test_f1_rule_met_with_c1_90_of_96_each_arm():
    f = _summary()["F1_rule"]
    assert (f["found_zones"], f["found_none"]) == (90, 90) and f["lost"] == [] == f["gained"]
    assert (f["median_zones_s"], f["median_none_s"]) == (131.4, 130.6) and f["time_met"]
    d = _summary()["F1_rule_dev"]
    assert (d["found_zones"], d["found_none"]) == (86, 87)          # dev's control, as re-measured


def test_f1_prime_rooms_still_failed_on_time():
    f = _summary()["F1_rooms"]
    assert (f["found_zones"], f["found_none"]) == (89, 87)
    assert f["lost"] == ["procthor:val:26 s0"]
    assert f["gained"] == ["procthor:val:44 s0", "procthor:val:44 s1", "procthor:val:44 s2"]
    assert (f["median_zones_s"], f["median_none_s"], f["time_ratio"]) == (127.0, 109.2, 1.163)
    assert not f["rate_met"] and not f["time_met"]
    d = _summary()["F1_rooms_dev"]
    assert (d["found_zones"], d["found_none"], d["time_ratio"]) == (81, 81, 1.0508)


def test_c2_to_c8_met():
    s = _summary()
    assert s["C2"]["met"] and len(s["C2"]["changed"]) == 25
    assert all(c["dev"] == "exhausted" for c in s["C2"]["changed"])
    assert sum(c["branch"] == "found" for c in s["C2"]["changed"]) == 21
    assert s["C3_rule"] == {"rows": 24, "identical": 24} == s["C3_rooms"]
    assert s["C4"]["arms"] == {"rule_zones": {"branch": 90, "dev": 86},
                               "rule_none": {"branch": 90, "dev": 87},
                               "rooms_zones": {"branch": 89, "dev": 81},
                               "rooms_none": {"branch": 87, "dev": 81}}
    assert s["C5"] == {"goals_in_zone": 0, "goals_in_grown": 0, "entries": 0, "met": True}
    assert s["C6"]["most_near_one_zone"] == 0
    assert (s["C7"]["most_rounds"], s["C7"]["most_places"]) == (2, 6)
    for k in ("C8_M3", "C8_M2"):
        assert s[k]["met"] and s[k]["changed"] == []
        assert s[k]["found_branch_vs_dev"] == {"surveyed": {"branch": 20, "dev": 20},
                                               "unsurveyed": {"branch": 20, "dev": 20}}


def test_c11_live_find_as_recorded():
    import json
    d = json.loads((ROOT / "evaluations" / "nogo-3192" / "fix" / "live.json").read_text())
    m = d["mission"]
    assert (m["outcome"], m["seconds"], m["goals_sent"]) == ("found", 75, 3)
    assert m["entries"] == [] and m["truth_samples"] == 148 and d["goals_in_zone"] == []
    assert d["z6_refused"] == 10 and d["peak_load"] == 3.05 <= 4
    bad = json.loads((ROOT / "evaluations" / "nogo-3192" / "fix" / "live-1-invalid-load.json").read_text())
    assert bad["peak_load"] == 4.51 > 4                     # invalid (3.79), re-run


# ---------- 3.210: a round's left-out places are kept for the next ----------

def _four_places(agent):
    """The true place seen first, three false ones after it (3.210's val:44)."""
    r = agent.retry.radius_m
    true = (0.6, 0.6)
    agent._note_seen_at(*true)
    for k in range(1, 4):
        agent._note_seen_at(0.6 + k * (r + 0.05), 1.8)
    assert len(agent._seen_at) == 4 and agent._seen_at[0] == true
    return true


def test_3210_a_place_the_first_round_left_out_gets_the_second():
    agent, events = _agent()
    true = _four_places(agent)
    agent._choose_next(0.0)
    assert len(events[-1][1]["places"]) == explore_mod.RECALL_MAX
    assert list(true) not in events[-1][1]["places"]       # crowded out of round 1
    agent.recall = []                                      # the round's goals spent; nothing new seen
    action, executed, detail = agent._choose_next(0.0)
    assert (action, executed) == ("WAIT", True) and "looking again" in detail
    assert events[-1][0] == "reseen"
    assert events[-1][1]["round"] == 2 and events[-1][1]["places"] == [list(true)]
    agent.recall = []
    assert agent._choose_next(0.0)[0] == "STOP"            # RESEEN_ROUNDS spent
    assert [n for n, _ in events].count("reseen") == 2


def test_3210_a_left_out_place_refused_is_never_looked_at():
    from brain.arrival import REFUSED
    agent, events = _agent()
    agent.sight_from = "perception"
    true = _four_places(agent)
    agent._choose_next(0.0)
    agent.recall = []
    agent._last_seen = true                                # the cloud refuses what stood there
    agent._note_refusal({"_arrival": {"state": REFUSED}})
    assert agent._choose_next(0.0)[0] == "STOP"
    assert [n for n, _ in events].count("reseen") == 1


def test_3210_round_one_holding_everything_ends_as_before():
    agent, events = _agent()
    agent._note_seen_at(1.5, 0.7)
    agent._note_seen_at(0.6, 1.8)
    agent._choose_next(0.0)
    assert agent._seen_at == []
    agent.recall = []
    assert agent._choose_next(0.0)[0] == "STOP"


def test_3210_false_places_seen_again_do_not_crowd_out_a_left_out_one():
    """Review: round 1 looks at the false places, sees them again there (the
    latest sightings now), and round 2 must still give the left-out one its look."""
    agent, events = _agent()
    true = _four_places(agent)
    agent._choose_next(0.0)
    falses = [tuple(p) for p in events[-1][1]["places"]]
    agent.recall = []
    for x, y in falses:                                    # seen again, a little off
        agent._note_seen_at(x + 0.02, y)
    agent._choose_next(0.0)
    assert events[-1][0] == "reseen" and events[-1][1]["round"] == 2
    assert events[-1][1]["places"][0] == list(true)
    assert len(events[-1][1]["places"]) == explore_mod.RECALL_MAX
