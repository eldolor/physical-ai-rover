# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.161.B's open item: a mission that left a zone out for its own dock gives
the robot server the stored set back however it ends (bars R-a..R-g of
docs/plans/ros-alignment/3.161-no-go-zones-on-twin.md, "a stopped mission's
reduced set").

Through the brain's real routes and its real mission loop; the robot
server's copy is what `nogo_store.push` was last handed.
"""

import time

import pytest
from fastapi.testclient import TestClient

from control import nogo_store
from tests.test_no_go_dock import FRAME, _runner, _Saved


@pytest.fixture
def rig(tmp_path, monkeypatch):
    """A store holding a zone over the mission's start (its dock) and one far
    away; the robot server's copy recorded; a brain whose runner is `runner`."""
    monkeypatch.setenv("NOGO_DIR", str(tmp_path))
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(nogo_store, "_DEFAULT", None)
    runner, grid, over, far = _runner(max_steps=100000)
    store = nogo_store.NoGoStore(tmp_path)
    store.edit(FRAME, "add", zone={k: over[k] for k in ("x0", "y0", "x1", "y1", "kind")})
    store.edit(FRAME, "add", zone={k: far[k] for k in ("x0", "y0", "x1", "y1", "kind")})
    ids = [z["id"] for z in store.zones(FRAME)]
    sent = []
    monkeypatch.setattr(nogo_store, "push",
                        lambda base, secret, f, zones, **kw: sent.append(
                            (f, [z["id"] for z in zones])) or {"accepted": True})
    from control import brain_server

    class Body:
        """The robot as the brain sees it: a robot server at an address."""
        base_url = "http://robot.invalid"

    world = _Saved(grid)

    def make(runner_=None, tick=None):
        r = runner_ or runner
        if tick is not None:
            r.tick = tick(r)
        return brain_server.create_app(robot_factory=lambda: Body(),
                                       world_factory=lambda _r: world,
                                       runner_factory=lambda _b, _req: r)
    return {"runner": runner, "ids": ids, "sent": sent, "make": make, "store": store}


def _slow(r):
    """A tick that keeps the mission running until it is stopped."""
    def tick():
        time.sleep(0.02)
        return r.is_running()
    return tick


START = {"target_object": "red backpack", "policy": "explore"}


def _wait(cond, s=5.0):
    end = time.monotonic() + s
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_a_person_stopped_mission_gives_the_stored_set_back(rig):
    """R-a: the bug. Red before the fix: the last set sent is the reduced one."""
    over, far = rig["ids"]
    with TestClient(rig["make"](tick=_slow)) as c:
        assert c.post("/mission/start", json=START).status_code == 200
        assert rig["runner"].nogo_dock_refused == [over]
        assert rig["sent"][-1] == (FRAME, [far])        # reduced while it runs
        assert c.post("/mission/stop").status_code == 200
        assert rig["sent"][-1] == (FRAME, [over, far]), rig["sent"]


def test_a_hung_tick_abort_gives_the_stored_set_back(rig):
    """R-b: B3.3's abort ends the loop on its own path."""
    over, far = rig["ids"]
    rig["runner"].tick_timeout_s = 0.2

    def hang(r):
        def tick():
            time.sleep(1.0)
            return True
        return tick
    with TestClient(rig["make"](tick=hang)) as c:
        assert c.post("/mission/start", json=START).status_code == 200
        assert _wait(lambda: rig["sent"][-1] == (FRAME, [over, far])), rig["sent"]
        assert not rig["runner"].is_running()


def test_a_mission_ending_on_its_own_gives_the_stored_set_back(rig):
    """R-c: an end the mission reaches itself (the robot's watchdog included:
    it stops the motors, the mission ends by its own budget)."""
    over, far = rig["ids"]
    n = {"i": 0}

    def ends(r):
        def tick():
            n["i"] += 1
            if n["i"] >= 3:
                r.abort("a fault budget spent")
            return r.is_running()
        return tick
    with TestClient(rig["make"](tick=ends)) as c:
        assert c.post("/mission/start", json=START).status_code == 200
        assert _wait(lambda: rig["sent"][-1] == (FRAME, [over, far])), rig["sent"]


def test_a_restored_set_never_overwrites_a_newer_missions(rig):
    """R-d: a restore for an old mission after a newer start sends nothing."""
    with TestClient(rig["make"](tick=_slow)) as c:
        assert c.post("/mission/start", json=START).status_code == 200
        app = c.app
        old_gen = app.state.brain_state["generation"]
        app.state.brain_state["generation"] += 1         # a newer start
        rig["sent"].clear()
        app.state.nogo_restore(rig["runner"], old_gen)
        assert rig["sent"] == []
        app.state.brain_state["generation"] = old_gen
        c.post("/mission/stop")


def test_a_stopped_missions_late_lookup_sends_nothing(rig):
    """R-e: a tick still in flight after the Stop never sends the reduced set
    over the restored one."""
    over, far = rig["ids"]
    with TestClient(rig["make"](tick=_slow)) as c:
        assert c.post("/mission/start", json=START).status_code == 200
        sink = rig["runner"].nogo_sink
        assert c.post("/mission/stop").status_code == 200
        rig["sent"].clear()
        sink(FRAME, [{"id": far}])                       # what a late tick would send
        time.sleep(0.2)
        assert rig["sent"] == []


def test_a_restarted_brain_gives_the_robot_server_the_stored_set(rig):
    """R-f: a brain that died mid-mission left the reduced set behind; the
    next brain's start-up sends the stored set (in force: no saved dock)."""
    over, far = rig["ids"]
    with TestClient(rig["make"]()):
        assert _wait(lambda: (FRAME, [over, far]) in rig["sent"]), rig["sent"]


def test_a_restarted_brain_with_no_zones_sends_nothing(rig, tmp_path):
    """R-f: nothing stored for the map: no send (no set to give back)."""
    (tmp_path / f"{FRAME}.json").unlink()
    with TestClient(rig["make"]()):
        time.sleep(0.3)
    assert rig["sent"] == []
