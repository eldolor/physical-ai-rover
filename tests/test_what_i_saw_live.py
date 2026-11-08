# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.162.C: the end-of-mission hook and the brain's report routes (W1, W7's
"a writer that raises never changes an outcome", and the ways a mission ends
since 3.156: docs/plans/ros-alignment/3.162-what-i-saw.md).

In process: `MissionRunner` -> agent -> `MockRobot` with the renderer on, the
sink the brain server sets standing in as a list; the routes on the real app.
"""

import base64
import hashlib
import io
import os
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from PIL import Image

from brain import room_names as rn
from brain import what_i_saw as wis
from brain.inventory import REPORT_BELIEF
from control import brain_server
from control.mission_runner import (ARRIVED_UNCONFIRMED, CANCELLED, EXHAUSTED, FOUND, OUT_OF_BUDGET,
                                    MissionRunner, _look_box)
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_resume_recharge import finish, state, to_charging
from tests.test_resume_recharge import mission as charge_mission


def _jpeg(colour=(120, 90, 40)):
    buf = io.BytesIO()
    Image.new("RGB", (320, 240), colour).save(buf, "JPEG", quality=85)
    return buf.getvalue()


def _sink(runner):
    got = []
    runner.saw_sink = got.append
    return got


def _rendered(target="red backpack", house="scaled_house", **kw):
    grid = build_world(house)
    robot = MockRobot(grid, render=True)
    kw.setdefault("max_steps", 400)
    return MissionRunner(robot, policy="frontier", world=MockWorld(grid), target_object=target, **kw), grid


def _run(runner, cap=4000):
    runner.start()
    n = 0
    while runner.tick() and n < cap:
        n += 1
    return runner.status()


# ---------- W1: the frame, once per mission ----------

@pytest.mark.parametrize("target", ["red backpack"])
def test_a_found_mission_hands_its_frame_once(target):
    runner, _ = _rendered(target, max_steps=2000)
    got = _sink(runner)
    s = _run(runner)
    assert s["outcome"] == FOUND, s["log_tail"]
    assert len(got) == 1
    p = got[0]
    assert p["found"]["image_base64"] and p["found"]["confirmed"] is True
    assert p["report"]["outcome"] == FOUND and p["report"]["target"] == target
    assert p["report"]["step"] == s["step"]               # the twin matches a report on this
    assert runner._saw_looks == {} and runner._last_seen_frame is None    # nothing held after


def test_the_found_frame_is_the_one_the_yes_was_given_on():
    runner, _ = _rendered()
    got = _sink(runner)
    runner.start()
    no, yes, later = ({"image_base64": x} for x in ("Tk8=", "WUVT", "TEFURVI="))
    runner._guarded_confirm = lambda f, local_ok=True: {"confirmed": f is yes}
    runner._judged_confirm(no)
    assert runner._judged_frame is None                    # a no is never the found frame
    runner._judged_confirm(yes)
    runner._last_seen_frame = later
    runner._finish(FOUND, "found it")
    assert got[0]["found"]["image_base64"] == "WUVT"


def test_arrived_unconfirmed_keeps_the_frame_the_cloud_could_not_judge():
    runner, _ = _rendered()
    got = _sink(runner)
    runner.start()
    runner._last_seen_frame = {"image_base64": "TEFURVI="}
    runner._finish(ARRIVED_UNCONFIRMED, "no cloud", arrival={"state": "unconfirmed"},
                   arrival_frame={"image_base64": "U0VOVA=="})
    assert got[0]["found"] == {"image_base64": "U0VOVA==", "box": None, "confirmed": False}


@pytest.mark.parametrize("outcome", [EXHAUSTED, OUT_OF_BUDGET, CANCELLED])
def test_other_ends_have_no_found_frame(outcome):
    runner, _ = _rendered()
    got = _sink(runner)
    runner.start()
    runner._last_seen_frame = {"image_base64": "TEFURVI="}
    runner._judged_frame = {"image_base64": "WUVT"}
    runner._finish(outcome, "ended")
    assert len(got) == 1 and got[0]["found"] is None


def test_no_sink_holds_nothing():
    runner, _ = _rendered(max_steps=30)
    _run(runner)
    assert runner._saw_looks == {} and runner._last_seen_frame is None


def test_a_mission_that_never_started_writes_nothing():
    runner, _ = _rendered()
    got = _sink(runner)
    runner._finish(CANCELLED, "never started")
    assert got == []


def test_a_sink_that_raises_never_changes_the_outcome():
    runner, _ = _rendered(max_steps=2000)
    def boom(_):
        raise RuntimeError("disk full")
    runner.saw_sink = boom
    s = _run(runner)
    assert s["outcome"] == FOUND and runner.finished.is_set()


# ---------- the list: only what this mission saw ----------

def test_the_list_is_this_missions_reported_things_each_with_a_look():
    runner, _ = _rendered("unicorn", max_steps=250)
    got = _sink(runner)
    s = _run(runner)
    rows = got[0]["looks"]
    assert rows, s["log_tail"]
    assert all(r["belief"] >= REPORT_BELIEF and r["image_base64"] for r in rows)
    assert all(0 <= r["box"][0] < r["box"][2] <= 1 for r in rows)
    carried = {r["label"] for r in rows}
    assert carried <= {d["label"] for d in runner.inventory_report()["reported"]}


def test_a_carried_landmark_not_seen_again_is_not_in_the_list():
    runner, _ = _rendered()
    got = _sink(runner)
    runner.start()
    runner.inventory_report = lambda: {"frame_id": "map-1", "reported": [
        {"id": 1, "label": "sofa", "x_m": 1, "y_m": 2, "room": "lounge", "belief": .9, "hits": 2, "first_step": None},
        {"id": 2, "label": "chair", "x_m": 1, "y_m": 2, "room": "lounge", "belief": .9, "hits": 2, "first_step": 3}]}
    runner._finish(CANCELLED, "stop")
    assert [r["label"] for r in got[0]["looks"]] == ["chair"]
    assert got[0]["report"]["frame_id"] == "map-1"


class _LM:
    def __init__(self, i, belief, step=5):
        self.id, self.belief, self.first_step, self.last_step, self.x, self.y = i, belief, 1, step, 1.0, 0.0


def test_at_most_max_thumbs_looks_are_held_the_likeliest_kept():
    runner, _ = _rendered()
    runner._ticks = 5
    pose = {"x_m": 0.0, "y_m": 0.0, "heading_deg": 90.0}
    frame = {"image_base64": "QUJD", "fov_deg": 60.0}
    runner.inventory.landmarks = [_LM(i, 0.5 + i * 1e-4) for i in range(wis.MAX_THUMBS + 25)]
    runner._keep_looks(frame, pose)
    assert len(runner._saw_looks) == wis.MAX_THUMBS
    assert min(runner._saw_looks) == 25
    # A merged-away landmark's slot goes before a live one's.
    runner.inventory.landmarks = [_LM(i, 0.5 + i * 1e-4, step=0) for i in range(26, wis.MAX_THUMBS + 25)] + [_LM(999, 0.1)]
    runner._keep_looks(frame, pose)
    assert 999 in runner._saw_looks and 25 not in runner._saw_looks


def test_the_best_look_stands_until_a_likelier_one():
    runner, _ = _rendered()
    runner._ticks = 5
    pose = {"x_m": 0.0, "y_m": 0.0, "heading_deg": 90.0}
    lm = _LM(1, 0.9)
    runner.inventory.landmarks = [lm]
    runner._keep_looks({"image_base64": "AAAA"}, pose)
    lm.belief = 0.7
    runner._keep_looks({"image_base64": "BBBB"}, pose)
    assert runner._saw_looks[1]["image_base64"] == "AAAA"
    lm.belief = 0.95
    runner._keep_looks({"image_base64": "CCCC"}, pose)
    assert runner._saw_looks[1]["image_base64"] == "CCCC"


def test_look_box_follows_the_bearing():
    centre = _look_box(0.0, 1.0, 60.0)
    assert centre[0] == pytest.approx(1 - centre[2]) and centre[1] == 0.0 and centre[3] == 1.0
    right = _look_box(20.0, 1.0, 60.0)
    assert right[0] > centre[0]
    assert _look_box(0.0, 8.0, 60.0)[2] - _look_box(0.0, 8.0, 60.0)[0] < centre[2] - centre[0]


# ---------- 3.156's ends: once, whichever way ----------

def test_a_found_that_drives_home_is_reported_after_the_return_once():
    from tests.test_dock_return import mission as dock_mission
    runner, *_ = dock_mission()
    runner.robot.render = True if hasattr(runner.robot, "render") else None
    got = _sink(runner)
    runner.start()
    runner._last_seen_frame = None
    runner._judged_frame = {"image_base64": "WUVT"}
    runner._finish(FOUND, "found it")
    assert (runner.status().get("return") or {}).get("state") == "returning"
    assert got == []                                       # still driving home
    n = 0
    while runner.tick() and n < 6000:
        n += 1
    assert len(got) == 1 and got[0]["report"]["outcome"] == FOUND
    assert got[0]["found"]["image_base64"] == "WUVT"       # the judged one, not the dock's view


def test_a_stop_during_the_return_is_still_one_report():
    from tests.test_dock_return import mission as dock_mission
    runner, *_ = dock_mission()
    got = _sink(runner)
    runner.start()
    runner._judged_frame = {"image_base64": "WUVT"}
    runner._finish(FOUND, "found it")
    runner.stop()
    runner.stop()
    assert len(got) == 1 and got[0]["report"]["outcome"] == FOUND
    assert got[0]["found"]["confirmed"] is True


def test_a_stop_on_the_charger_is_one_report_and_a_resume_writes_none_until_the_end():
    runner, *_ = charge_mission()
    got = _sink(runner)
    to_charging(runner)
    assert got == []                                       # the pause is not an end
    runner.stop()
    assert state(runner) in ("cancelled", "failed") or runner.status()["running"] is False
    assert len(got) == 1 and got[0]["report"]["outcome"] == OUT_OF_BUDGET


def test_a_charge_and_resume_reports_once_at_the_real_end():
    runner, *_ = charge_mission()
    got = _sink(runner)
    to_charging(runner)
    assert got == []
    s = finish(runner)
    assert s["running"] is False and len(got) == 1
    assert got[0]["report"]["outcome"] == s["outcome"]


# ---------- the brain's side: names, the writer, the routes ----------

@pytest.fixture
def app_client(tmp_path, monkeypatch):
    monkeypatch.setenv("WHAT_I_SAW_DIR", str(tmp_path / "saw"))
    monkeypatch.setenv("ROOM_NAMES_DIR", str(tmp_path / "names"))
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(rn, "_DEFAULT", None)
    app = brain_server.create_app(robot_factory=lambda: object(), world_factory=lambda robot: None)
    return TestClient(app), wis.ReportStore(tmp_path / "saw"), tmp_path


def _payload(frame_id=None):
    b64 = base64.b64encode(_jpeg()).decode()
    return {"report": {"outcome": FOUND, "job": "find", "target": "red backpack", "step": 7,
                       "frame_id": frame_id, "rooms_entered": ["kitchen"], "rooms_source": "model"},
            "found": {"image_base64": b64, "box": None, "confirmed": True},
            "looks": [{"label": "chair", "x_m": 1.25, "y_m": 2.25, "room": "kitchen", "belief": .9,
                       "looks": 3, "image_base64": b64, "box": [.2, 0, .6, 1]},
                      {"label": "sofa", "x_m": 9.0, "y_m": 9.0, "room": "lounge", "belief": .85, "looks": 2}]}


class _Writer:
    def __init__(self, store):
        self.w = wis.ReportWriter(store)

    def __call__(self):
        return self.w


def test_the_judged_frames_hash_is_the_sent_frames(app_client):
    _, store, _ = app_client
    writer = _Writer(store)
    p = _payload()
    brain_server._write_saw(writer, "m-1", p)
    assert writer.w.flush()
    rep = store.get("m-1")
    sent = base64.b64decode(p["found"]["image_base64"])
    assert rep["found"]["sha256"] == hashlib.sha256(sent).hexdigest()
    assert store.image("m-1", "found.jpg") == sent
    assert [m["thumb"] is not None for m in rep["landmarks"]] == [True, False]


def test_a_persons_room_name_replaces_the_models(app_client):
    _, store, tmp = app_client
    names = rn.default_store()
    keys = sorted({(x, y) for x in range(10, 15) for y in range(20, 25)})
    names._save("map-1", {"version": 1, "frame_id": "map-1", "history": [],
                          "rooms": [{"id": "a", "by": "person", "name": "Den", "runs": rn.to_runs(set(keys))}]})
    writer = _Writer(store)
    brain_server._write_saw(writer, "m-2", _payload("map-1"))
    assert writer.w.flush()
    rooms = {m["label"]: m["room"] for m in store.get("m-2")["landmarks"]}
    assert rooms == {"chair": "Den", "sofa": "lounge"}


def test_a_name_reaches_an_object_against_the_wall_only_nearby():
    rooms = [{"name": "Den", "_keys": {(10, 10)}}]
    assert brain_server._person_room(rooms, 1.05, 1.05) == "Den"
    assert brain_server._person_room(rooms, 1.35, 1.05) == "Den"        # 3 cells off
    assert brain_server._person_room(rooms, 1.55, 1.05) is None


def test_a_bad_frame_loses_itself_only(app_client):
    _, store, _ = app_client
    writer = _Writer(store)
    p = _payload()
    p["found"]["image_base64"] = "not base64!!"
    brain_server._write_saw(writer, "m-3", p)
    assert writer.w.flush()
    rep = store.get("m-3")
    assert rep["found"] is None and len(rep["landmarks"]) == 2


def test_the_routes(app_client):
    client, store, _ = app_client
    store.save("m-4", {"outcome": FOUND, "target": "x", "step": 3},
               found={"jpeg": _jpeg(), "box": None, "confirmed": True},
               looks=[{"label": "chair", "x_m": 1, "y_m": 1, "room": "k", "belief": .9, "looks": 2,
                       "jpeg": _jpeg(), "box": None}])
    rows = client.get("/mission/reports").json()["reports"]
    assert [r["mission_id"] for r in rows] == ["m-4"] and rows[0]["found"] is True
    assert client.get("/mission/reports/m-4").json()["landmarks"][0]["label"] == "chair"
    for path in ("found.jpg", "thumbs/0.jpg"):
        r = client.get(f"/mission/reports/m-4/{path}")
        assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg"
        assert r.headers["x-content-type-options"] == "nosniff"
        assert r.content[:3] == b"\xff\xd8\xff"
    assert client.get("/mission/reports/nope").status_code == 404
    assert client.get("/mission/reports/m-4/thumbs/report.json").status_code == 404
    assert client.get("/mission/reports/m-4/thumbs/9.jpg").status_code == 404
    assert client.delete("/mission/reports/m-4").json() == {"deleted": True}
    assert client.get("/mission/reports").json() == {"reports": []}


def test_the_routes_are_behind_the_secret(tmp_path, monkeypatch):
    monkeypatch.setenv("WHAT_I_SAW_DIR", str(tmp_path))
    monkeypatch.setenv("APP_SHARED_SECRET", "s3")
    client = TestClient(brain_server.create_app(robot_factory=lambda: object(), world_factory=lambda r: None))
    assert client.get("/mission/reports").status_code == 401
    assert client.get("/mission/reports/x/found.jpg").status_code == 401
    assert client.delete("/mission/reports/x").status_code == 401
    assert client.get("/mission/reports", headers={"x-app-secret": "s3"}).status_code == 200


def test_the_store_defaults_beside_the_inventory(tmp_path, monkeypatch):
    monkeypatch.delenv("WHAT_I_SAW_DIR", raising=False)
    monkeypatch.setenv("INVENTORY_DIR", str(tmp_path / "inventory"))
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    client = TestClient(brain_server.create_app(robot_factory=lambda: object(), world_factory=lambda r: None))
    wis.ReportStore(tmp_path / "what-i-saw").save("m-5", {"outcome": FOUND})
    assert [r["mission_id"] for r in client.get("/mission/reports").json()["reports"]] == ["m-5"]
