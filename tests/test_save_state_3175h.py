# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.175.H: the runner says whether its inventory save landed, and the
last-seen route reads that instead of inferring it from the store
(docs/plans/ros-alignment/3.175-memory-when-things-move.md, H1-H7).

G11 was the pin this removes: an ended mission whose save never becomes its
map's head (one that never carried, even with its save landed; or one whose
save failed) was read first until the brain restarted.
"""
from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest

from tests.test_last_seen import FRAME, _head, _other_map_newer, _route_state, _row, _save_chain
from tests.test_last_seen import brain  # noqa: F401 -- the fixture

SAVED = "saved"
FAILED = "failed"


def _ended(live, state, mission_id=None):
    return SimpleNamespace(
        is_running=lambda: False, inventory_report=lambda: live,
        inventory_save_state=lambda: {"state": state,
                                      "mission_id": live.get("mission_id") if mission_id is None else mission_id,
                                      "error": None})


def _ask(client, name="mug"):
    r = client.get("/mission/last-seen", params={"name": name})
    assert r.status_code == 200, r.text
    return r.json()


def _key(a):
    place = a.get("place") or {}
    return (a["status"], a.get("source"), a.get("frame_id"), place.get("x_m"))


def _no_runner_answer(client, name="mug"):
    st = _route_state(client)
    keep, st["runner"] = st["runner"], None
    try:
        return _ask(client, name)
    finally:
        st["runner"] = keep


# ---------- H1: G11's cases ----------

def test_h1a_never_carried_save_landed_but_not_the_head_answers_as_no_runner(brain):  # noqa: F811
    from control.inventory_store import InventoryStore
    live = dict(_head([_row("mug", 5.0, 5.0, 0.9)], carry_seq=0), mission_id="run-1")
    _save_chain(brain.tmp, _head([_row("mug", 1.0, 1.0, 0.9, seen_seq=3)], carry_seq=3))
    InventoryStore(str(brain.tmp / "inv")).save("run-1", live)      # landed, seq 0: not the head
    _route_state(brain)["runner"] = _ended(live, SAVED)
    a = _ask(brain)
    assert _key(a) == _key(_no_runner_answer(brain)) == ("seen", "saved", FRAME, 1.0), a
    # A newer map saved since: the newest saved map, as with no runner (G3).
    _other_map_newer(brain.tmp)
    a = _ask(brain)
    assert _key(a) == _key(_no_runner_answer(brain)) == ("seen", "saved", "map-b", 9.0), a
    assert brain.robot.reads == [] and brain.world.reads == []          # H5


@pytest.mark.parametrize("carry", [0, 4])
def test_h1b_a_failed_save_answers_as_no_runner(brain, carry):  # noqa: F811
    live = dict(_head([_row("mug", 5.0, 5.0, 0.9, seen_seq=carry or None)], carry_seq=carry),
                mission_id="run-1")
    _save_chain(brain.tmp, _head([_row("mug", 1.0, 1.0, 0.9, seen_seq=3)], carry_seq=3))
    _route_state(brain)["runner"] = _ended(live, FAILED)
    a = _ask(brain)
    assert _key(a) == _key(_no_runner_answer(brain)) == ("seen", "saved", FRAME, 1.0), a
    # A name only the failed mission saw: not recalled by a find, so not seen.
    live["reported"].append(_row("kettle", 6.0, 6.0, 0.9, seen_seq=carry or None))
    assert _ask(brain, "kettle")["status"] == "not_seen"
    assert brain.robot.reads == [] and brain.world.reads == []


def test_h1b_a_failed_save_with_nothing_saved_says_no_memory(brain):  # noqa: F811
    live = dict(_head([_row("mug", 5.0, 5.0, 0.9)], carry_seq=0), mission_id="run-1")
    _route_state(brain)["runner"] = _ended(live, FAILED)
    assert _ask(brain)["status"] == "no_memory"


# ---------- H2: not settled, or another mission's state ----------

@pytest.mark.parametrize("state,mid", [("pending", None), ("none", None),
                                       (SAVED, "another"), (FAILED, "another")])
@pytest.mark.parametrize("carry", [0, 4])
def test_h2_unsettled_or_another_missions_state_reads_its_own_first(brain, state, mid, carry):  # noqa: F811
    live = dict(_head([_row("mug", 5.0, 5.0, 0.9, seen_seq=carry or None)], carry_seq=carry),
                mission_id="run-1")
    _save_chain(brain.tmp, _head([_row("mug", 1.0, 1.0, 0.9, seen_seq=3)], carry_seq=3))
    _route_state(brain)["runner"] = _ended(live, state, mission_id=mid)
    a = _ask(brain)
    assert (a["source"], a["place"]["x_m"]) == ("mission", 5.0), a
    assert brain.robot.reads == [] and brain.world.reads == []


def test_h2_a_running_mission_ignores_a_settled_state(brain):  # noqa: F811
    live = dict(_head([_row("mug", 5.0, 5.0, 0.9)], carry_seq=0), mission_id="run-1")
    _save_chain(brain.tmp, _head([_row("mug", 1.0, 1.0, 0.9, seen_seq=3)], carry_seq=3))
    runner = _ended(live, SAVED)
    runner.is_running = lambda: True
    _route_state(brain)["runner"] = runner
    assert _ask(brain)["source"] == "mission"


def test_h2_a_state_reader_that_raises_is_not_settled(brain):  # noqa: F811
    live = dict(_head([_row("mug", 5.0, 5.0, 0.9)], carry_seq=0), mission_id="run-1")

    def boom():
        raise RuntimeError("mid-teardown")
    _route_state(brain)["runner"] = SimpleNamespace(is_running=lambda: False,
                                                    inventory_report=lambda: live,
                                                    inventory_save_state=boom)
    assert _ask(brain)["source"] == "mission"


# ---------- the runner's state ----------

def _runner():
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from tests.conftest import mock_world_for
    robot = MockRobot(build_world("scaled_house"), render=False)
    r = MissionRunner(robot, target_object="unicorn", max_steps=3, policy="frontier",
                      world=mock_world_for(robot))
    r.mission_id = "m-1"
    return r


def test_the_runner_state_goes_none_pending_saved_once():
    r = _runner()
    assert r.inventory_save_state()["state"] == "none"
    seen = []

    def sink(report):
        seen.append(r.inventory_save_state()["state"])
    r.inventory_sink = sink
    r.start()
    while r.tick():
        pass
    assert seen == ["pending"]
    assert r.inventory_save_state() == {"state": "pending", "mission_id": "m-1", "error": None}
    r.inventory_saved("other", True)                   # not this mission's: ignored
    assert r.inventory_save_state()["state"] == "pending"
    r.inventory_saved("m-1", True)
    assert r.inventory_save_state() == {"state": "saved", "mission_id": "m-1", "error": None}
    r.inventory_saved("m-1", False, "late")            # once
    assert r.inventory_save_state()["state"] == "saved"


# ---------- H6: a save is never failed or slowed ----------

def test_h6a_a_raising_callback_never_fails_the_save(tmp_path):
    from control import inventory_store as inv_mod
    store = inv_mod.InventoryStore(str(tmp_path / "inv"))

    def boom(ok, error):
        raise RuntimeError("callback down")
    t = store.save_async("m-1", {"frame_id": FRAME, "carry_seq": 1}, on_local=boom)
    t.join(5)
    assert (tmp_path / "inv" / "m-1.json").exists()
    assert str((tmp_path / "inv").resolve()) not in inv_mod._PENDING
    assert store.latest_for(FRAME)["carry_seq"] == 1     # no wait left behind


def test_h6a_the_callback_says_written_or_not_before_the_count_drops(tmp_path):
    from control import inventory_store as inv_mod
    got = []

    def note(ok, error):
        with inv_mod._PENDING_COND:
            got.append((ok, error, inv_mod._PENDING.get(str(d.resolve()))))
    d = tmp_path / "inv"
    inv_mod.InventoryStore(str(d)).save_async("m-1", {"frame_id": FRAME}, on_local=note).join(5)
    assert got == [(True, None, 1)]                      # still counted when it ran
    blocker = tmp_path / "file"
    blocker.write_text("x")                              # a directory that cannot be made
    got.clear()
    d = blocker / "inv"
    inv_mod.InventoryStore(str(d)).save_async("m-2", {"frame_id": FRAME}, on_local=note).join(5)
    assert len(got) == 1 and got[0][0] is False and got[0][1].startswith("local:")


def test_h6b_the_state_never_waits_on_the_runners_tick_lock():
    r = _runner()
    r.inventory_sink = lambda report: None
    r.start()
    while r.tick():
        pass
    done = threading.Event()
    with r._lock:                                        # a tick in progress
        threading.Thread(target=lambda: (r.inventory_saved("m-1", True), done.set()),
                         daemon=True).start()
        assert done.wait(2.0)
    assert r.inventory_save_state()["state"] == "saved"


def test_h6c_a_raising_sink_ends_the_mission_normally_its_state_failed():
    r = _runner()

    def boom(report):
        raise RuntimeError("disk gone")
    r.inventory_sink = boom
    r.start()
    while r.tick():
        pass
    assert r.status()["outcome"] == "out_of_budget"
    st = r.inventory_save_state()
    assert st["state"] == "failed" and st["mission_id"] == "m-1" and "disk gone" in st["error"]


def test_h6d_the_callback_costs_under_a_millisecond():
    r = _runner()
    times = []
    for i in range(1000):
        with r._inventory_save_lock:
            r._inventory_save = {"state": "pending", "mission_id": i, "error": None}
        t0 = time.perf_counter()
        r.inventory_saved(i, True)
        times.append(time.perf_counter() - t0)
    times.sort()
    p99 = times[989]
    assert p99 <= 0.001, p99


# ---------- H7: end to end through the brain app ----------

def test_h7_a_real_mission_reports_pending_then_saved(monkeypatch, tmp_path):
    from fastapi.testclient import TestClient
    from control import brain_server, inventory_store as inv_mod
    from tests.conftest import RecordingRobot, fresh_mock_robot
    monkeypatch.setenv("INVENTORY_DIR", str(tmp_path / "inv"))
    monkeypatch.setenv("INVENTORY_BUCKET", "")
    release = threading.Event()
    real = inv_mod._fsync

    def held(fd):
        release.wait(10)
        real(fd)
    monkeypatch.setattr(inv_mod, "_fsync", held)
    app = brain_server.create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": "unicorn", "max_steps": 3})
        runner = _route_state(client)["runner"]
        client.post("/mission/stop")
        deadline = time.monotonic() + 5
        while runner.inventory_save_state()["state"] == "none" and time.monotonic() < deadline:
            time.sleep(0.01)
        st = runner.inventory_save_state()
        assert st["state"] == "pending" and st["mission_id"] == runner.mission_id, st
        release.set()
        deadline = time.monotonic() + 5
        while runner.inventory_save_state()["state"] == "pending" and time.monotonic() < deadline:
            time.sleep(0.01)
        st = runner.inventory_save_state()
        assert st == {"state": "saved", "mission_id": runner.mission_id, "error": None}, st
        assert (tmp_path / "inv" / f"{inv_mod._slug(runner.mission_id)}.json").exists()


def test_h7_an_unwritable_directory_reports_failed(monkeypatch, tmp_path):
    from fastapi.testclient import TestClient
    from control import brain_server
    from tests.conftest import RecordingRobot, fresh_mock_robot
    (tmp_path / "file").write_text("x")
    monkeypatch.setenv("INVENTORY_DIR", str(tmp_path / "file" / "inv"))
    monkeypatch.setenv("INVENTORY_BUCKET", "")
    app = brain_server.create_app(robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    with TestClient(app) as client:
        client.post("/mission/start", json={"target_object": "unicorn", "max_steps": 3})
        runner = _route_state(client)["runner"]
        client.post("/mission/stop")
        deadline = time.monotonic() + 5
        while runner.inventory_save_state()["state"] in ("none", "pending") \
                and time.monotonic() < deadline:
            time.sleep(0.01)
        st = runner.inventory_save_state()
        assert st["state"] == "failed" and st["error"].startswith("local:"), st
