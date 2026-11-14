# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
3.176.B: a find asked in plain words -- the rule parser (`brain/request.py`),
the room hint as the room policy's prior (`brain/goal_policy.py`), its path
through explore and `MissionRunner`, and the brain's parse route and start
fields. The parser's bars P1-P6 are re-scored here on the pinned request set
(`evaluations/request-3176/requests.jsonl`) against the recorded first score
(`evaluations/request-3176/parser_score.json`).
"""
import json
import socket
import subprocess
import sys
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from brain import request as rq
from brain.goal_policy import RoomGoalPolicy, hint_matches, hinted_prior

ROOT = Path(__file__).resolve().parent.parent
SET = ROOT / "evaluations" / "request-3176" / "requests.jsonl"
SCORE = ROOT / "evaluations" / "request-3176" / "parser_score.json"


def _rows():
    return [json.loads(line) for line in SET.read_text(encoding="utf-8").splitlines()]


def score(rows) -> dict:
    """The parser's P1-P4 counts over rows (3.176.A's definitions)."""
    out = {"rows": len(rows), "target": 0, "room": 0, "attributes": 0, "wrong_room": 0,
           "misses": []}
    for r in rows:
        p, e = rq.parse(r["text"], r["names"]), r["expect"]
        t, room = p.target == e["target"], p.room_hint == e["room"]
        a = set(p.colours) == set(e["colours"]) and p.size == e["size"]
        out["target"] += t
        out["room"] += room
        out["attributes"] += a
        out["wrong_room"] += p.room_hint is not None and p.room_hint != e["room"]
        if not (t and room and a):
            out["misses"].append(r["id"])
    return out


# ---------- P1-P6: the parser on the pinned set ----------

def test_the_recorded_score_is_what_the_parser_scores():
    rec = json.loads(SCORE.read_text())
    rows = _rows()
    for split in ("dev", "test"):
        assert score([r for r in rows if r["split"] == split]) == rec[split]["score"], split


def test_p1_to_p4_on_the_test_split():
    s = json.loads(SCORE.read_text())["test"]["score"]
    n = s["rows"]
    assert n == 76
    assert s["target"] >= 69                 # P1: >= 90%
    assert s["room"] >= 73                   # P2: >= 95%
    assert s["wrong_room"] <= 1              # P3
    assert s["attributes"] >= 73             # P4: >= 95%


def test_p5_a_plain_target_reads_back_as_typed():
    for r in _rows():
        if r["kind"] == "plain":
            p = rq.parse(r["text"])
            assert (p.target, p.room_hint) == (r["text"], None), r["id"]
            assert p.detector_prompts == (r["text"],)


def test_p6_offline_every_connection_refused(monkeypatch):
    rows = _rows()
    want = [rq.parse(r["text"], r["names"]).to_dict() for r in rows]

    def refuse(*a, **k):
        raise OSError("connection refused (3.176 P6)")

    monkeypatch.setattr(socket.socket, "connect", refuse)
    monkeypatch.setattr(socket, "create_connection", refuse)
    assert [rq.parse(r["text"], r["names"]).to_dict() for r in rows] == want


def test_p6_the_parser_imports_no_network_client():
    code = ("import sys; import brain.request; "
            "print(sorted(m for m in sys.modules if m.split('.')[0] in "
            "{'socket','ssl','http','urllib','httpx','requests','anthropic','boto3','botocore'}))")
    out = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True,
                         text=True, check=True).stdout.strip()
    assert out == "[]"


# ---------- the parser's rules, one each ----------

@pytest.mark.parametrize("text,target,room", [
    ("find my keys and my phone", None, None),               # two things
    ("black and white sneakers", "black and white sneakers", None),
    ("check the bedroom", None, {"type": "bedroom"}),         # a room, no thing
    ("find it in the kitchen", None, {"type": "kitchen"}),
    ("my phone, not in the kitchen", "phone", None),          # negated
    ("the remote in the kitchen or the living room", "remote", None),
    ("my bag in the kitchn", "bag", None),                    # never a guess
    ("kitchen scissors", "kitchen scissors", None),
    ("the keys on the kitchen counter", "keys", {"type": "kitchen"}),
])
def test_rules(text, target, room):
    p = rq.parse(text)
    assert (p.target, p.room_hint) == (target, room)
    assert p.detector_prompts == ((target,) if target else ())


def test_a_name_on_the_map_wins_over_a_type():
    assert rq.parse("the remote in the den").room_hint == {"type": "living room"}
    assert rq.parse("the remote in the den", ["Den"]).room_hint == {"name": "Den"}


def test_canonical_room_and_check_hint():
    assert rq.canonical_room("Den") == "living room"
    assert rq.canonical_room("bedroom") == "bedroom" and rq.canonical_room("attic") is None
    assert rq.check_hint(None) is None
    assert rq.check_hint({"type": "kitchen"}) == {"type": "kitchen"}
    assert rq.check_hint({"name": "Den"}, ["Den"]) == {"name": "Den"}
    for bad in ({"type": "attic"}, {"room": "kitchen"}, {"type": ""}, "kitchen",
                {"type": "kitchen", "name": "x"}, {"name": "x" * 41}):
        with pytest.raises(ValueError):
            rq.check_hint(bad)
    with pytest.raises(ValueError):
        rq.check_hint({"name": "Den"}, ["Kitchen"])


# ---------- the hint as a prior ----------

def test_no_hint_or_no_hinted_room_leaves_the_prior_as_it_is():
    prior = {"bedroom": 0.53, "kitchen": 0.0}
    types = ["bedroom", None, "kitchen"]
    assert hinted_prior(prior, types, None) is prior
    assert hinted_prior(prior, types, {"type": "garage"}) is prior


def test_a_hint_sets_its_rooms_to_one_and_nothing_else():
    prior = {"bedroom": 0.53, "kitchen": 0.0}
    got = hinted_prior(prior, ["bedroom", "kitchen", "den", None], {"type": "living room"})
    assert got == {"bedroom": 0.53, "kitchen": 0.0, "den": 1.0}
    got = hinted_prior({}, ["Ben's room", "kitchen"], {"name": "ben's ROOM"})
    assert got == {"Ben's room": 1.0}
    assert hint_matches("bedroom", {"type": "bedroom"})
    assert not hint_matches(None, {"type": "bedroom"}) and not hint_matches("x", None)


def test_the_policy_prefers_the_hinted_room_and_falls_back_without_it():
    """A hinted room's goal outranks an equal goal elsewhere; with the hinted
    room's goals gone (searched), the other is chosen -- a hint orders, it
    never ends the search."""
    pol = RoomGoalPolicy.load()
    m = {"width": 40, "height": 40, "resolution_m": 0.1, "origin_x_m": 0.0, "origin_y_m": 0.0,
         "cells": [0] * 1600}
    pose = {"x_m": 2.0, "y_m": 2.0}
    goals = [(1.0, 2.0), (3.0, 2.0)]
    feats = [[0.5] * 10, [0.5] * 10]
    kw = dict(trail=[], types=["kitchen", "bedroom"], target="red backpack")
    base = pol.scores(pol.extend(m, pose, goals, feats, **kw))
    for i, t in enumerate(("kitchen", "bedroom")):
        s = pol.scores(pol.extend(m, pose, goals, feats, hint={"type": t}, **kw))
        assert int(s.argmax()) == i
        assert s[i] >= base[i] and s[1 - i] == pytest.approx(base[1 - i])
    one = pol.extend(m, pose, goals[1:], feats[1:], hint={"type": "kitchen"},
                     trail=[], types=["bedroom"], target="red backpack")
    assert pol.best(one) == 0


# ---------- the mission ----------

def test_a_hint_never_turns_asks_on_for_a_flat_prior():
    from tests.test_explore import _mission
    runner, _nav, _grid = _mission(explore_goal_policy="rooms", find_room_labels=True,
                                   room_hint={"type": "bedroom"},
                                   room_label_fn=lambda f: pytest.fail("asked"))
    assert runner.room_labeller is None              # the red backpack's prior is flat
    assert runner.status()["room_hint"]["hint"] == {"type": "bedroom"}


def test_the_status_says_whether_the_hint_can_act():
    from tests.test_explore import _mission
    r, _n, _g = _mission(explore_goal_policy="rule", room_hint={"type": "kitchen"},
                         request="the red backpack in the kitchen")
    st = r.status()
    assert st["request"] == "the red backpack in the kitchen"
    assert st["room_hint"] == {"hint": {"type": "kitchen"}, "acts": False,
                               "why": "explore_goal_policy is not 'rooms'", "cells": None}
    r, _n, _g = _mission(explore_goal_policy="rooms", room_hint={"type": "kitchen"})
    assert r.status()["room_hint"]["why"] == "no room labels on a find" or \
        r.status()["room_hint"]["acts"]


def test_a_plain_find_has_no_hint_and_no_request_in_its_status():
    from tests.test_explore import _mission
    r, _n, _g = _mission(explore_goal_policy="rooms")
    st = r.status()
    assert st["request"] is None and st["room_hint"] is None


def test_the_runner_refuses_a_bad_hint_a_survey_hint_and_a_blank_target():
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from control.mission_runner import MissionRunner
    robot = MockRobot(build_world("scaled_house"), render=False)
    with pytest.raises(ValueError):
        MissionRunner(robot, target_object="keys", room_hint={"type": "attic"})
    with pytest.raises(ValueError):
        MissionRunner(robot, target_object="   ")
    with pytest.raises(ValueError):
        MissionRunner(robot, job="survey", policy="explore", navigator=object(),
                      room_hint={"type": "kitchen"})


# ---------- the brain's route and start fields ----------

class _World:
    def __init__(self, frame_id):
        self.frame_id = frame_id

    def get_pose(self):
        return {"usable": True, "x_m": 0.0, "y_m": 0.0, "theta_deg": 0.0,
                "frame_id": self.frame_id}

    def get_map(self):
        return {"usable": False}


class _Store:
    def rooms(self, frame_id):
        assert frame_id == "map-3176b001"
        return [{"name": "Ben's room"}, {"name": "Den"}]


def _client(monkeypatch, frame_id="map-3176b001"):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    from control import brain_server
    monkeypatch.setattr(brain_server.room_names_mod, "default_store", lambda: _Store())
    app = brain_server.create_app(robot_factory=lambda: object(),
                                  world_factory=lambda robot: _World(frame_id))
    return TestClient(app)


def test_the_parse_route_reads_against_the_saved_maps_names(monkeypatch):
    c = _client(monkeypatch)
    got = c.post("/mission/request/parse", json={"text": "the lego set in Ben's room"}).json()
    assert got["target"] == "lego set" and got["room_hint"] == {"name": "Ben's room"}
    assert got["names_read"] == 2 and got["detector_prompts"] == ["lego set"]
    got = c.post("/mission/request/parse", json={"text": "my keys and my phone"}).json()
    assert got["target"] is None and got["detector_prompts"] == []
    assert c.post("/mission/request/parse", json={"text": "x", "more": 1}).status_code == 422


def test_the_parse_route_without_a_saved_map_reads_no_names(monkeypatch):
    c = _client(monkeypatch, frame_id=None)
    got = c.post("/mission/request/parse", json={"text": "the remote in the den"}).json()
    assert got["room_hint"] == {"type": "living room"} and got["names_read"] == 0


@pytest.mark.parametrize("body,why", [
    ({"target_object": "keys", "policy": "explore", "room_hint": {"type": "attic"}},
     "not one of"),
    ({"target_object": "keys", "policy": "explore", "room_hint": {"name": "Nowhere"}},
     "not a room named on this map"),
    ({"target_object": "keys", "policy": "explore", "request": "x" * 501}, "at most 500"),
    ({"target_object": "   ", "policy": "explore"}, "needs a target_object"),
])
def test_a_bad_start_is_refused_before_anything_is_built(monkeypatch, body, why):
    c = _client(monkeypatch)
    r = c.post("/mission/start", json=body)
    assert 400 <= r.status_code < 500 and why in r.text, r.text


def test_a_name_hint_needs_a_saved_map(monkeypatch):
    c = _client(monkeypatch, frame_id=None)
    r = c.post("/mission/start", json={"target_object": "keys", "policy": "explore",
                                       "room_hint": {"name": "Den"}})
    assert 400 <= r.status_code < 500 and "saved map" in r.text
