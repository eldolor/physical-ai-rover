# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.190.C, criteria 10-12: a find's cloud room prior through the brain's
`POST /mission/start` -- one call, only when it can act; a failed call costs
nothing but its deadline; the rates reach this mission's goal choice only;
a typed room hint wins. Every model call is a stub: nothing is paid."""
import time

import pytest
from fastapi.testclient import TestClient

import brain.room_names as rn
from brain import room_prior as rp
from sim.mock_world import MockWorld
from tests.conftest import RecordingRobot, fresh_mock_robot
from tests.test_brain_server import _StubNavigator

FRAME = "map-3190c000"
GOOD = '{"kitchen": 0.9, "living room": 0.05, "bedroom": 0.01, "bathroom": 0.02}'
RATES = rp.parse(GOOD)
REAL_ASK = rp.ask              # before any test patches it


class Store:
    """A person's room names on the saved map (3.160's store), or none."""

    def __init__(self, names):
        self.names = names

    def rooms(self, frame_id):
        assert frame_id == FRAME
        return [{"name": n, "_keys": set()} for n in self.names]

    def load(self, frame_id):
        return {"version": 1, "frame_id": frame_id, "history": [], "rev": 1,
                "rooms": self.rooms(frame_id)}


@pytest.fixture
def calls(monkeypatch, tmp_path):
    """Every /room-prior call the start makes, answered with GOOD; the cache
    a fresh file."""
    made = []

    def ask(target, url, secret=None, timeout_s=None, client=None):
        made.append({"target": target, "url": url, "secret": secret})
        return {"reply": GOOD, "stop": "end_turn", "model_id": rp.MODEL_ID}

    monkeypatch.setattr(rp, "ask", ask)
    monkeypatch.setenv("ROOM_PRIOR_CACHE", str(tmp_path / "room-prior.json"))
    for k in ("EXPLORE_GOAL_POLICY", "FIND_ROOM_LABELS", "FIND_PRIOR_SOURCE",
              "VISION_URL", "APP_SHARED_SECRET", "VISION_SHARED_SECRET"):
        monkeypatch.delenv(k, raising=False)
    return made


def _app(tmp_path, monkeypatch, names=("kitchen",), **extra):
    import control.brain_server as bs
    monkeypatch.setattr(bs, "RemoteNavigator", _StubNavigator)
    cfg = {"explore_goal_policy": "rooms", "find_prior_source": "cloud",
           "vision_url": "https://vision.example", **extra}
    lines = ["brain:", "  tick_interval_s: 0.0"] + [f"  {k}: {v}" for k, v in cfg.items()]
    path = tmp_path / "robot.yaml"
    path.write_text("\n".join(lines) + "\n")
    real = MockWorld.get_pose
    monkeypatch.setattr(MockWorld, "get_pose",
                        lambda self: {**real(self), "frame_id": FRAME if names is not None else None})
    monkeypatch.setattr(rn, "default_store", lambda: Store(list(names or ())))
    robot = fresh_mock_robot()
    return bs.create_app(config_path=str(path), robot_factory=lambda: RecordingRobot(robot),
                         world_factory=lambda r: MockWorld(robot.world))


def _start(app, body):
    with TestClient(app) as client:
        t0 = time.monotonic()
        resp = client.post("/mission/start", json={"policy": "explore", **body})
        took = time.monotonic() - t0
        runner = app.state.brain_state["runner"]
        client.post("/mission/stop")
    return resp, runner, took


def _policy_prior(runner, target):
    return runner.agent.goal_policy.prior.get(target)


# ---------- criterion 10: one call, and only when it can act ----------

def test_an_unknown_target_with_named_rooms_makes_exactly_one_call(tmp_path, monkeypatch, calls):
    resp, runner, _ = _start(_app(tmp_path, monkeypatch), {"target_object": "Oven Mitt"})
    assert resp.status_code == 200, resp.text
    assert [c["target"] for c in calls] == ["Oven Mitt"], resp.json()["status"]["room_prior"]
    assert calls[0]["url"] == "https://vision.example"
    st = resp.json()["status"]["room_prior"]
    assert st == {"source": "cloud", "why": None, "rates": RATES, "model_id": rp.MODEL_ID,
                  "acts": True}
    # The rates sit where the shipped prior's would, in this mission's policy only.
    assert _policy_prior(runner, "oven mitt") == RATES
    assert runner.metrics_config["room_prior"] == "cloud"
    from brain.goal_policy import RoomGoalPolicy
    assert "oven mitt" not in RoomGoalPolicy.load().prior        # the shipped file untouched


def test_a_cache_hit_makes_no_call(tmp_path, monkeypatch, calls):
    app = _app(tmp_path, monkeypatch)
    _start(app, {"target_object": "oven mitt"})
    resp, runner, _ = _start(app, {"target_object": "oven mitt"})
    assert len(calls) == 1
    assert resp.json()["status"]["room_prior"]["source"] == "cache"
    assert _policy_prior(runner, "oven mitt") == RATES


@pytest.mark.parametrize("body, extra, names, source, why", [
    ({"target_object": "Garbage Bag"}, {}, ("kitchen",), "shipped", None),
    ({"job": "survey"}, {}, ("kitchen",), "none", "not a find"),
    ({"target_object": "oven mitt"}, {"explore_goal_policy": "rule"}, ("kitchen",),
     "none", "explore_goal_policy is not 'rooms'"),
    ({"target_object": "oven mitt", "policy": "frontier"}, {}, ("kitchen",),
     "none", "no explore search"),
    ({"target_object": "oven mitt"}, {}, (), "none", "no room-label layer"),
    ({"target_object": "oven mitt"}, {}, None, "none", "no room-label layer"),
    # find_room_labels on, for a target the shipped prior lacks: its asks stay
    # on the shipped prior (3.116 amendment 7), so they make no layer here.
    ({"target_object": "oven mitt"}, {"find_room_labels": "frame"}, (), "none",
     "no room-label layer"),
])
def test_no_call_when_it_cannot_act(tmp_path, monkeypatch, calls, body, extra, names, source, why):
    resp, runner, _ = _start(_app(tmp_path, monkeypatch, names=names, **extra), body)
    assert resp.status_code == 200, resp.text
    assert calls == []
    st = resp.json()["status"]["room_prior"]
    assert (st["source"], st["why"]) == (source, why)


def test_a_shipped_target_reads_the_shipped_prior(tmp_path, monkeypatch, calls):
    from brain.goal_policy import shipped_rooms_prior
    resp, runner, _ = _start(_app(tmp_path, monkeypatch), {"target_object": "garbage bag"})
    st = resp.json()["status"]["room_prior"]
    assert st["rates"] == shipped_rooms_prior("garbage bag") and st["acts"]
    assert _policy_prior(runner, "garbage bag") == shipped_rooms_prior("garbage bag")


def test_off_makes_no_call_and_reports_nothing(tmp_path, monkeypatch, calls):
    resp, runner, _ = _start(_app(tmp_path, monkeypatch, find_prior_source='"off"'),
                             {"target_object": "oven mitt"})
    assert resp.status_code == 200, resp.text
    assert calls == [] and resp.json()["status"]["room_prior"] is None
    assert "room_prior" not in runner.metrics_config
    assert "oven mitt" not in runner.agent.goal_policy.prior
    assert "room_prior" not in runner._build_report()


def test_the_report_carries_the_prior(tmp_path, monkeypatch, calls):
    _resp, runner, _ = _start(_app(tmp_path, monkeypatch), {"target_object": "oven mitt"})
    assert runner._build_report()["room_prior"]["source"] == "cloud"


def test_a_refused_start_pays_nothing(tmp_path, monkeypatch, calls):
    resp, _r, _t = _start(_app(tmp_path, monkeypatch),
                          {"target_object": "oven mitt", "frame_vote": "majority"})
    assert resp.status_code == 400 and calls == []


# ---------- criterion 11: a failed call costs nothing but its deadline ----------

DEADLINE = 1.0


def _failing(monkeypatch, how):
    import httpx

    def ask(target, url, secret=None, timeout_s=None, client=None):
        if how == "refused":
            return REAL_ASK(target, "http://127.0.0.1:1", timeout_s=timeout_s)
        if how == "not json":
            return {"reply": "It is usually in the kitchen.", "model_id": rp.MODEL_ID}
        if how == "late":
            time.sleep(DEADLINE + 2)
            return {"reply": GOOD, "model_id": rp.MODEL_ID}
        req = httpx.Request("POST", url)
        raise httpx.HTTPStatusError("x", request=req, response=httpx.Response(502, request=req))

    monkeypatch.setattr(rp, "ask", ask)


@pytest.mark.parametrize("how, why", [("refused", "unreachable"),
                                      ("not json", "reply is not a prior"),
                                      ("late", "timeout"), ("5xx", "http 502")])
def test_a_failed_call_starts_the_mission_with_no_prior(tmp_path, monkeypatch, calls, how, why):
    _failing(monkeypatch, how)
    resp, runner, took = _start(_app(tmp_path, monkeypatch, find_prior_timeout_s=DEADLINE),
                                {"target_object": "oven mitt"})
    assert resp.status_code == 200, resp.text
    st = resp.json()["status"]["room_prior"]
    assert (st["source"], st["why"], st["rates"]) == ("none", why, None)
    assert "oven mitt" not in runner.agent.goal_policy.prior
    assert runner.metrics_config["room_prior"] == f"none ({why})"
    assert took < DEADLINE + 0.5, took


def test_the_config_bounds_the_deadline(tmp_path, monkeypatch):
    from control.brain_config import load_brain_config
    for bad in ("0", "-1", "31", "true", '"cloud"'):
        p = tmp_path / "r.yaml"
        p.write_text(f"brain:\n  find_prior_timeout_s: {bad}\n")
        with pytest.raises(ValueError, match="find_prior_timeout_s"):
            load_brain_config(str(p))
    p.write_text("brain:\n  find_prior_source: always\n")
    with pytest.raises(ValueError, match="find_prior_source"):
        load_brain_config(str(p))
    monkeypatch.setenv("FIND_PRIOR_SOURCE", "cloud")
    p.write_text("brain: {}\n")
    assert load_brain_config(str(p))["find_prior_source"] == "cloud"
    monkeypatch.delenv("FIND_PRIOR_SOURCE")
    assert load_brain_config(str(p))["find_prior_source"] == "off"
    assert load_brain_config(str(p))["find_prior_timeout_s"] == 4.0


def test_the_runner_refuses_rates_that_are_not_a_prior():
    from tests.test_explore import _mission
    with pytest.raises(ValueError, match="room_prior"):
        _mission(explore_goal_policy="rooms",
                 room_prior={"source": "cloud", "rates": {"kitchen": 2.0}})


# ---------- criterion 12: the hint wins ----------

def _hinted_find(monkeypatch, room_prior, hint):
    """A find of a target no sim house holds (it never ends found), on
    scaled_house with its true rooms as a person's names, a typed hint."""
    from sim.maps import build_world
    from tests.test_explore import _mission
    from tests.test_room_names_find import FRAME as NAMES_FRAME, Store as NamesStore
    real = MockWorld.get_pose
    monkeypatch.setattr(MockWorld, "get_pose", lambda self: {**real(self), "frame_id": NAMES_FRAME})
    store = NamesStore({n: list(c) for n, c in build_world("scaled_house").rooms.items()})
    monkeypatch.setattr(rn, "default_store", lambda: store)
    runner, _nav, _g = _mission(target="oven mitt", explore_goal_policy="rooms", max_steps=60,
                                room_hint=hint, room_prior=room_prior)
    return runner


def test_with_a_hint_the_hinted_type_outranks_every_cloud_rate(monkeypatch):
    """The hint is applied on top of the cloud prior (3.190's scope): the
    hinted type reads 1.0 -- no rate can exceed it -- and the other types
    keep the cloud's rates. Criterion 12 as written (a cloud prior with the
    other types at 0 sends the same goals as none) FAILED: a type known at
    rate 0 is a different feature from no type (3.190.C results)."""
    from brain.goal_policy import hinted_prior
    cloud = {"source": "cloud", "why": None, "model_id": rp.MODEL_ID,
             "rates": {"kitchen": 0.95, "living room": 0.9, "bedroom": 0.01, "bathroom": 0.5}}
    runner = _hinted_find(monkeypatch, cloud, {"type": "bedroom"})
    rates = runner.agent.goal_policy.prior["oven mitt"]
    assert rates == cloud["rates"]
    read = hinted_prior(rates, ["kitchen", "bedroom", "living room"], runner.room_hint)
    assert read["bedroom"] == 1.0 == max(read.values())
    assert {k: v for k, v in read.items() if k != "bedroom"} == {
        "kitchen": 0.95, "living room": 0.9, "bathroom": 0.5}
    assert runner.status()["room_hint"]["acts"] and runner.status()["room_prior"]["acts"]
