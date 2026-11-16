# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.175.C: "where did you last see it?" -- `brain/last_seen.py` and the
brain's read-only `GET /mission/last-seen`
(docs/plans/ros-alignment/3.175-memory-when-things-move.md, C1-C5).

The answer's order is pinned against `MissionRunner._recall_carried` itself
(the place a find would go first); the measured numbers on real chains are
pinned from `evaluations/last-seen-3175c/result.json`.
"""

import json
import os
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from brain import last_seen
from brain.inventory import REPORT_BELIEF, Inventory
from brain.memory import MissionMemory

ROOT = Path(__file__).resolve().parents[1]
RESULT = ROOT / "evaluations" / "last-seen-3175c" / "result.json"
FRAME = "map-a"


def _row(label, x, y, belief, seen_seq=None, room="kitchen", **kw):
    return {"label": label, "x_m": x, "y_m": y, "belief": belief, "seen_seq": seen_seq,
            "room": room, "votes": {label: 3}, **kw}


def _head(rows, carry_seq=4, frame_id=FRAME):
    return {"frame_id": frame_id, "carry_seq": carry_seq, "mission_id": "m-head",
            "reported": [r for r in rows if r["belief"] >= REPORT_BELIEF],
            "candidates": [r for r in rows if r["belief"] < REPORT_BELIEF]}


def _recalled(head, target):
    """What a real mission seeded from `head` hands explore, through the
    runner's own `_recall_carried`."""
    from control.mission_runner import MissionRunner
    inv = Inventory()
    inv.seed(head["frame_id"], head)
    got = []
    agent = SimpleNamespace(recall=got)
    agent.recall_from = lambda places: got.extend(places)
    fake = SimpleNamespace(agent=agent, inventory=inv, memory=MissionMemory("find", target_object=target))
    MissionRunner._recall_carried(fake)
    return [(round(x, 3), round(y, 3)) for x, y in got]


# ---------- the answer ----------

def test_the_place_is_the_first_a_find_would_recall_newest_first_then_belief():
    rows = [_row("mug", 1.0, 1.0, 0.98, seen_seq=1), _row("mug", 2.0, 2.0, 0.85, seen_seq=3),
            _row("coffee mug", 3.0, 3.0, 0.9, seen_seq=3), _row("mug", 4.0, 4.0, 0.95),
            _row("mug", 5.0, 5.0, 0.7, seen_seq=4), _row("sofa", 6.0, 6.0, 0.99, seen_seq=4)]
    head = _head(rows)
    a = last_seen.answer("Mug ", head)
    assert a["status"] == "seen"
    places = [(p["x_m"], p["y_m"]) for p in a["places"]]
    assert places == [(3.0, 3.0), (2.0, 2.0), (1.0, 1.0), (4.0, 4.0)]
    # The runner's whole order (explore keeps the first RECALL_MAX of it).
    assert places == _recalled(head, "mug")
    assert a["place"]["label"] == "coffee mug" and a["place"]["room"] == "kitchen"


def test_ties_keep_the_heads_order_as_the_runner_does():
    rows = [_row("cup", 1.0, 0.0, 0.9, 2), _row("cup", 2.0, 0.0, 0.9, 2), _row("cup", 3.0, 0.0, 0.9, 2)]
    head = _head(rows)
    assert [(p["x_m"], p["y_m"]) for p in last_seen.answer("cup", head)["places"]] == _recalled(head, "cup")


def test_a_carried_row_under_report_belief_is_doubtful_never_a_place():
    head = _head([_row("mug", 1.0, 1.0, 0.7, seen_seq=2), _row("mug", 2.0, 2.0, 0.6)])
    a = last_seen.answer("mug", head)
    assert a["status"] == "doubtful" and a["place"] is None and a["places"] == []
    assert (a["doubtful"]["x_m"], a["doubtful"]["belief"]) == (1.0, 0.7)
    assert _recalled(head, "mug") == []


def test_not_seen_no_memory_and_rows_a_mission_would_not_carry():
    head = _head([_row("mug", 1.0, 1.0, 0.4), _row("lamp", 0.0, 0.0, 0.9)])
    assert last_seen.answer("mug", head)["status"] == "not_seen"      # under CARRY_MIN_BELIEF
    assert last_seen.answer("zeppelin", head)["status"] == "not_seen"
    assert last_seen.answer("mug", None)["status"] == "no_memory"


@pytest.mark.parametrize("bad", ["", "   ", None, "x" * 81])
def test_a_blank_or_long_name_is_refused(bad):
    with pytest.raises(last_seen.NameRefused):
        last_seen.answer(bad, _head([]))


def test_bad_rows_and_seen_seq_are_read_as_the_seed_reads_them():
    rows = [_row("mug", "x", 1.0, 0.9), _row("mug", float("nan"), 1.0, 0.9),
            _row("mug", 1.0, 1.0, True), "junk",
            _row("mug", 7.0, 7.0, 0.9, seen_seq=99), _row("mug", 8.0, 8.0, 0.85, seen_seq=1)]
    head = {"frame_id": FRAME, "carry_seq": 2, "reported": rows}
    a = last_seen.answer("mug", head)
    # seen_seq 99 is past the head's carry_seq: read None, recalled last. A
    # belief of True is 1.0 to the seed's float(), so it is carried too.
    assert [(p["x_m"], p["seen_seq"]) for p in a["places"]] == [(8.0, 1), (1.0, None), (7.0, None)]
    assert [(p["x_m"], p["y_m"]) for p in a["places"]] == _recalled(head, "mug")


def test_rows_parse_as_the_seed_parses_them():
    """Review: numeric strings and integral float seen_seq, as `Inventory.seed`."""
    rows = [_row("mug", "1.5", "2", "0.9", seen_seq=2.0), _row("mug", 3.0, 3.0, 0.95, seen_seq=1)]
    head = {"frame_id": FRAME, "carry_seq": 2, "reported": rows}
    a = last_seen.answer("mug", head)
    assert [(p["x_m"], p["seen_seq"]) for p in a["places"]] == [(1.5, 2), (3.0, 1)]
    assert [(p["x_m"], p["y_m"]) for p in a["places"]] == _recalled(head, "mug")


def test_reports_are_read_only_as_far_as_when_needs():
    head = _head([_row("mug", 1.0, 1.0, 0.95, seen_seq=3)])
    read = []

    def reports():
        for i in range(10):
            read.append(i)
            yield {"mission_id": f"m{i}", "frame_id": FRAME,
                   "landmarks": [{"label": "mug", "x_m": 1.0, "y_m": 1.0}] if i == 1 else []}
    assert last_seen.answer("mug", head, reports())["when"]["mission_id"] == "m1"
    assert read == [0, 1]
    read.clear()
    last_seen.answer("zeppelin", head, reports())
    assert read == []                                    # not seen: no report read


def test_when_is_the_newest_kept_report_on_the_map_with_the_thing_there():
    head = _head([_row("mug", 1.0, 1.0, 0.95, seen_seq=3)])
    reports = [
        {"mission_id": "other-map", "frame_id": "map-b", "ended": 9.0,
         "landmarks": [{"label": "mug", "x_m": 1.0, "y_m": 1.0, "thumb": "thumbs/0.jpg"}]},
        {"mission_id": "too-far", "frame_id": FRAME, "ended": 8.0,
         "landmarks": [{"label": "mug", "x_m": 1.6, "y_m": 1.0, "thumb": "thumbs/0.jpg"}]},
        {"mission_id": "right", "frame_id": FRAME, "ended": 7.0, "job": "find", "target": "keys",
         "landmarks": [{"label": "lamp", "x_m": 1.0, "y_m": 1.0, "thumb": "thumbs/0.jpg"},
                       {"label": "mug", "x_m": 1.2, "y_m": 1.1, "thumb": "thumbs/3.jpg",
                        "room": "Anna's study"}]},
        {"mission_id": "older", "frame_id": FRAME, "ended": 6.0,
         "landmarks": [{"label": "mug", "x_m": 1.0, "y_m": 1.0, "thumb": "thumbs/1.jpg"}]},
    ]
    w = last_seen.answer("mug", head, reports)["when"]
    assert (w["mission_id"], w["thumb"], w["ended"], w["room"]) == ("right", "thumbs/3.jpg", 7.0, "Anna's study")
    assert last_seen.answer("mug", head, reports[:2])["when"] is None      # its report pruned


def test_saved_head_reads_the_newest_saved_maps_chain(tmp_path):
    from control.inventory_store import InventoryStore
    store = InventoryStore(str(tmp_path))
    assert last_seen.saved_head(store) is None
    store.save("a1", _head([_row("mug", 1, 1, 0.9)], carry_seq=1, frame_id="map-a"))
    store.save("a2", _head([_row("mug", 2, 2, 0.9)], carry_seq=2, frame_id="map-a"))
    store.save("b1", _head([_row("mug", 3, 3, 0.9)], carry_seq=1, frame_id="map-b"))
    (tmp_path / "junk.json").write_text("{")
    os.utime(tmp_path / "junk.json", (time.time() + 60,) * 2)
    os.utime(tmp_path / "a2.json", (time.time() - 60,) * 2)    # lineage, not time, within a map
    assert last_seen.saved_head(store)["frame_id"] == "map-b"
    os.utime(tmp_path / "a1.json", (time.time() + 30,) * 2)
    assert last_seen.saved_head(store)["carry_seq"] == 2       # map-a's head, by lineage


# ---------- the route ----------

class _Untouchable:
    """A robot (or world) the route must never read."""

    def __init__(self):
        object.__setattr__(self, "reads", [])

    def __getattr__(self, name):
        self.reads.append(name)
        raise AttributeError(name)


@pytest.fixture
def brain(tmp_path, monkeypatch):
    monkeypatch.setenv("INVENTORY_DIR", str(tmp_path / "inv"))
    monkeypatch.setenv("WHAT_I_SAW_DIR", str(tmp_path / "saw"))
    from control import brain_server
    robot, world = _Untouchable(), _Untouchable()
    app = brain_server.create_app(robot_factory=lambda: robot, world_factory=lambda r: world)
    client = TestClient(app)
    client.robot, client.world, client.tmp = robot, world, tmp_path
    return client


def _save_chain(tmp, head):
    from control.inventory_store import InventoryStore
    InventoryStore(str(tmp / "inv")).save("m-head", head)


def test_the_route_answers_from_the_saved_chain_and_the_reports_reading_nothing_else(brain):
    from brain.what_i_saw import ReportStore
    _save_chain(brain.tmp, _head([_row("red backpack", 1.0, 2.0, 0.95, seen_seq=4)]))
    ReportStore(brain.tmp / "saw").save("m-4", {"job": "find", "target": "mug", "frame_id": FRAME,
                                                 "ended": 123.0},
                                        looks=[{"label": "red backpack", "x_m": 1.1, "y_m": 2.0,
                                                "room": "hall", "belief": 0.95, "looks": 3}])
    a = brain.get("/mission/last-seen", params={"name": "backpack"}).json()
    assert a["status"] == "seen" and a["source"] == "saved" and a["frame_id"] == FRAME
    assert (a["place"]["x_m"], a["place"]["y_m"]) == (1.0, 2.0)
    assert a["when"]["mission_id"] == "m-4" and a["when"]["ended"] == 123.0
    assert brain.get("/mission/last-seen", params={"name": "zeppelin"}).json()["status"] == "not_seen"
    assert brain.get("/mission/last-seen", params={"name": " "}).status_code == 422
    assert brain.get("/mission/last-seen").status_code == 422
    assert brain.robot.reads == [] and brain.world.reads == []          # C4


def test_the_route_with_nothing_kept_says_no_memory(brain):
    assert brain.get("/mission/last-seen", params={"name": "mug"}).json()["status"] == "no_memory"
    assert brain.robot.reads == [] and brain.world.reads == []


def test_the_route_reads_a_running_missions_live_inventory(brain):
    from control import brain_server  # noqa: F401
    live = _head([_row("mug", 5.0, 5.0, 0.9, seen_seq=5)], carry_seq=5)
    _save_chain(brain.tmp, _head([_row("mug", 1.0, 1.0, 0.9, seen_seq=4)]))
    runner = SimpleNamespace(is_running=lambda: True, inventory_report=lambda: live)
    state = _route_state(brain)
    state["runner"] = runner
    a = brain.get("/mission/last-seen", params={"name": "mug"}).json()
    assert a["source"] == "mission" and a["place"]["x_m"] == 5.0
    # Ended, its save not yet queued (review, second pass): still its own
    # inventory, never the older saved head.
    runner.is_running = lambda: False
    a = brain.get("/mission/last-seen", params={"name": "mug"}).json()
    assert a["source"] == "mission" and a["place"]["x_m"] == 5.0
    # A saved chain on the same map further on (another brain process) wins.
    _save_chain(brain.tmp, _head([_row("mug", 9.0, 9.0, 0.9, seen_seq=7)], carry_seq=7))
    a = brain.get("/mission/last-seen", params={"name": "mug"}).json()
    assert a["source"] == "saved" and a["place"]["x_m"] == 9.0


def test_a_runner_with_no_saved_map_falls_back_to_the_saved_chain(brain):
    _save_chain(brain.tmp, _head([_row("mug", 1.0, 1.0, 0.9, seen_seq=4)]))
    state = _route_state(brain)
    state["runner"] = SimpleNamespace(is_running=lambda: False,
                                      inventory_report=lambda: _head([], frame_id=None))
    assert brain.get("/mission/last-seen", params={"name": "mug"}).json()["place"]["x_m"] == 1.0


def test_a_huge_number_in_a_row_is_skipped_not_a_500():
    head = {"frame_id": FRAME, "carry_seq": 1,
            "reported": [_row("mug", 10 ** 400, 1.0, 0.9), _row("mug", 2.0, 2.0, 0.9)]}
    assert [p["x_m"] for p in last_seen.answer("mug", head)["places"]] == [2.0]


def test_a_save_still_in_flight_is_a_503_not_a_wrong_answer(brain, monkeypatch):
    from control.inventory_store import InventoryStore
    _save_chain(brain.tmp, _head([_row("mug", 1.0, 1.0, 0.9)]))

    def slow(self, frame_id):
        raise TimeoutError("an inventory save still in flight after 2 s")
    monkeypatch.setattr(InventoryStore, "latest_for", slow)
    r = brain.get("/mission/last-seen", params={"name": "mug"})
    assert r.status_code == 503 and "still being saved" in r.json()["detail"]


def test_the_route_waits_for_a_save_in_flight_then_503s(brain, monkeypatch):
    """Review: a save still being written was listed around, so the first
    mission on a map read as "no memory"."""
    from control import inventory_store as inv_mod
    key = str((brain.tmp / "inv").resolve())
    monkeypatch.setattr(inv_mod, "PENDING_WAIT_S", 0.05)
    with inv_mod._PENDING_COND:
        inv_mod._PENDING[key] = 1
    try:
        r = brain.get("/mission/last-seen", params={"name": "mug"})
    finally:
        with inv_mod._PENDING_COND:
            inv_mod._PENDING.pop(key, None)
    assert r.status_code == 503 and "still being saved" in r.json()["detail"]


def test_the_route_needs_the_secret_when_one_is_set(tmp_path, monkeypatch):
    monkeypatch.setenv("APP_SHARED_SECRET", "correct-horse-battery-staple")
    monkeypatch.setenv("INVENTORY_DIR", str(tmp_path / "inv"))
    from control import brain_server
    client = TestClient(brain_server.create_app(robot_factory=lambda: _Untouchable(),
                                                world_factory=lambda r: _Untouchable()))
    assert client.get("/mission/last-seen", params={"name": "mug"}).status_code == 401
    ok = client.get("/mission/last-seen", params={"name": "mug"},
                    headers={"x-app-secret": "correct-horse-battery-staple"})
    assert ok.status_code == 200


def _route_state(client):
    """The app's mutable `state` dict, reached through the route's closure."""
    for route in client.app.routes:
        if getattr(route, "path", "") == "/mission/last-seen":
            fn = route.endpoint
            for name, cell in zip(fn.__code__.co_freevars, fn.__closure__):
                if name == "state":
                    return cell.cell_contents
    raise AssertionError("no /mission/last-seen route")


# ---------- the measured numbers (C1-C5), pinned ----------

def test_the_measured_numbers_on_real_chains():
    import importlib.util
    spec = importlib.util.spec_from_file_location("measure", RESULT.parent / "measure.py")
    measure = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(measure)
    s = measure.score_result(json.loads(RESULT.read_text()))
    assert s["C1"]["wrong"] == [] and s["C1"]["right"] == s["C1"]["recalled_labels"] >= 20
    assert s["C2"]["bad"] == [] and s["C2"]["thumb_jpeg"] == s["C2"]["with_report"]
    assert s["C3"]["not_seen"] == s["C3"]["asked"] and s["C3"]["empty_store"] == "no_memory"
    assert s["C4"]["robot_reads_and_world_calls"] == 0
    assert s["C5"]["reports"] == 50 and s["C5"]["p95_ms"] <= 200
