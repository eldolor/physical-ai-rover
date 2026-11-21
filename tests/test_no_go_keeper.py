# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.161.B K2's fix: a new stack never runs with an empty keepout mask while
zones are stored (bars K2'/K5-K8 of
docs/plans/ros-alignment/3.161-no-go-zones-on-twin.md, "Z4's re-map live").

The brain's keeper sends the stored set in force to the robot server until
it is taken (a bounded backoff, each failure logged once per step), and again
whenever the pose names a new SLAM session or map, or the robot server's copy
or the bridge it reports lacks the set. Through the brain's real app; the
robot server is `nogo_store.push` / `nogo_store.held`, faked.
"""

import logging
import threading
import time

import pytest
from fastapi.testclient import TestClient

from control import brain_server, nogo_store
from tests.test_no_go_dock import FRAME, _runner, _Saved


class _World(_Saved):
    """A saved-map world whose pose the test drives: `mode` is "down" (the
    robot server not answering), "unusable" (SLAM not ready) or "ok";
    `session` is the pose's map_id."""

    def __init__(self, grid):
        super().__init__(grid)
        self.mode, self.session = "ok", "slam-a"

    def get_pose(self):
        if self.mode == "down":
            raise ConnectionError("robot server not up")
        p = super().get_pose()
        if self.mode == "unusable":
            return dict(p, usable=False, frame_id=None)
        return dict(p, map_id=self.session)


class _Robot:
    """The robot server as `push` and `held` reach it."""

    def __init__(self):
        self.fail = 0                    # refuse the next N pushes
        self.sends = []                  # (frame_id, ids) of every push, taken or not
        self.copy = {"frame_id": None, "zones": [], "sent": None, "bridge": None}
        self.bridge_zones = None         # what the bridge reports holding (None: no bridge)
        self.lock = threading.Lock()

    def push(self, base, secret, frame_id, zones, **kw):
        with self.lock:
            self.sends.append((frame_id, [z["id"] for z in zones]))
            if self.fail > 0:
                self.fail -= 1
                return {"accepted": False, "detail": "refused (test)"}
            self.copy = {"frame_id": frame_id, "zones": [dict(z) for z in zones], "sent": True}
            self.bridge_zones = len(zones)
            return {"accepted": True, "frame_id": frame_id, "zones": len(zones), "sent": True}

    def held(self, base, secret, **kw):
        with self.lock:
            b = None if self.bridge_zones is None else {"zones": self.bridge_zones}
            return {**self.copy, "bridge": b}


@pytest.fixture
def rig(tmp_path, monkeypatch):
    monkeypatch.setenv("NOGO_DIR", str(tmp_path))
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(nogo_store, "_DEFAULT", None)
    monkeypatch.setattr(brain_server, "NOGO_KEEP_POLL_S", 0.02, raising=False)
    monkeypatch.setattr(brain_server, "NOGO_BACKOFF_S", (0.02, 0.04, 0.08), raising=False)
    runner, grid, over, far = _runner(max_steps=100000)
    store = nogo_store.NoGoStore(tmp_path)
    store.edit(FRAME, "add", zone={k: over[k] for k in ("x0", "y0", "x1", "y1", "kind")})
    store.edit(FRAME, "add", zone={k: far[k] for k in ("x0", "y0", "x1", "y1", "kind")})
    ids = [z["id"] for z in store.zones(FRAME)]
    bot = _Robot()
    monkeypatch.setattr(nogo_store, "push", bot.push)
    monkeypatch.setattr(nogo_store, "held", bot.held, raising=False)
    world = _World(grid)

    class Body:
        base_url = "http://robot.invalid"

    def make():
        return brain_server.create_app(robot_factory=lambda: Body(),
                                       world_factory=lambda _r: world,
                                       runner_factory=lambda _b, _req: runner)
    return {"bot": bot, "world": world, "ids": ids, "make": make, "runner": runner,
            "tmp": tmp_path}


def _wait(cond, s=5.0):
    end = time.monotonic() + s
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_a_refused_start_up_send_is_retried_until_taken(rig):
    """K5: the bug (R-f's one attempt). Red before the fix: one push, refused,
    and nothing after it."""
    bot = rig["bot"]
    bot.fail = 3
    with TestClient(rig["make"]()):
        assert _wait(lambda: bot.copy["frame_id"] == FRAME), bot.sends
        assert len(bot.sends) == 4
        n = len(bot.sends)
        time.sleep(0.3)                          # taken: no more sends while nothing changes
        assert len(bot.sends) == n, bot.sends
    assert sorted(z["id"] for z in bot.copy["zones"]) == sorted(rig["ids"])


def test_a_robot_server_not_up_at_start_up_gets_the_set_once_it_answers(rig):
    """K5: the live failure (`GET /world/pose failed: Connection refused`)."""
    rig["world"].mode = "down"
    bot = rig["bot"]
    with TestClient(rig["make"]()):
        time.sleep(0.2)
        assert bot.sends == []
        rig["world"].mode = "ok"
        assert _wait(lambda: bot.copy["frame_id"] == FRAME), bot.sends


def test_slam_not_ready_at_start_up_then_a_saved_map_gets_the_set(rig):
    """K5: the pose names no saved map until the reload is judged."""
    rig["world"].mode = "unusable"
    bot = rig["bot"]
    with TestClient(rig["make"]()):
        time.sleep(0.2)
        assert bot.sends == []
        rig["world"].mode = "ok"
        assert _wait(lambda: bot.copy["frame_id"] == FRAME), bot.sends


def test_a_new_slam_session_gets_the_set_again(rig):
    """K6: a new SLAM session (a new bridge, a reload) re-sends it."""
    bot = rig["bot"]
    with TestClient(rig["make"]()):
        assert _wait(lambda: len(bot.sends) == 1)
        rig["world"].session = "slam-b"
        assert _wait(lambda: len(bot.sends) == 2), bot.sends


def test_a_bridge_or_copy_that_lacks_the_set_gets_it_again(rig):
    """K6: the bridge reports fewer zones (a new container under the same
    session, a re-map), or the robot server's copy was emptied (its restart)."""
    bot = rig["bot"]
    with TestClient(rig["make"]()):
        assert _wait(lambda: len(bot.sends) == 1)
        with bot.lock:
            bot.bridge_zones = 0
        assert _wait(lambda: len(bot.sends) == 2), bot.sends
        assert bot.bridge_zones == 2
        with bot.lock:
            bot.copy = {"frame_id": None, "zones": [], "sent": None}
            bot.bridge_zones = None
        assert _wait(lambda: len(bot.sends) == 3), bot.sends


def test_an_unreadable_copy_sends_nothing_more(rig, monkeypatch):
    """No information is no reason to send: a held read that fails never
    re-sends a set already taken."""
    bot = rig["bot"]
    with TestClient(rig["make"]()):
        assert _wait(lambda: len(bot.sends) == 1)
        monkeypatch.setattr(nogo_store, "held", lambda *a, **k: None)
        time.sleep(0.3)
        assert len(bot.sends) == 1


def test_each_failure_is_logged_once_per_backoff_step(rig, caplog):
    """K7: never silent, never one line a poll."""
    bot = rig["bot"]
    bot.fail = 3
    with caplog.at_level(logging.WARNING, logger="brain_server"):
        with TestClient(rig["make"]()):
            assert _wait(lambda: bot.copy["frame_id"] == FRAME)
    lines = [r for r in caplog.records if "no-go zones not taken" in r.getMessage()]
    assert len(lines) == 3, [r.getMessage() for r in lines]


def test_nothing_stored_sends_nothing(rig):
    """R-f kept: no zones for the map, no send."""
    (rig["tmp"] / f"{FRAME}.json").unlink()
    bot = rig["bot"]
    with TestClient(rig["make"]()):
        time.sleep(0.3)
    assert bot.sends == []


def test_a_running_missions_reduced_set_is_what_is_kept(rig):
    """K8: while a mission that left a zone out for its dock runs, the keeper
    re-sends that mission's set, never the stored one over it."""
    over, far = rig["ids"]
    bot = rig["bot"]
    runner = rig["runner"]

    def tick():
        time.sleep(0.02)
        return runner.is_running()
    runner.tick = tick
    with TestClient(rig["make"]()) as c:
        assert _wait(lambda: len(bot.sends) >= 1)
        assert c.post("/mission/start", json={"target_object": "red backpack",
                                              "policy": "explore"}).status_code == 200
        assert _wait(lambda: bot.sends[-1] == (FRAME, [far])), bot.sends
        n = len(bot.sends)
        rig["world"].session = "slam-b"
        assert _wait(lambda: len(bot.sends) > n), bot.sends
        assert all(s == (FRAME, [far]) for s in bot.sends[n:]), bot.sends
        assert c.post("/mission/stop").status_code == 200
        assert _wait(lambda: bot.sends[-1] == (FRAME, [over, far])), bot.sends


def test_the_keeper_stops_with_the_app(rig):
    """The thread ends at shutdown: no send after the app is gone."""
    bot = rig["bot"]
    with TestClient(rig["make"]()):
        assert _wait(lambda: len(bot.sends) == 1)
    n = len(bot.sends)
    rig["world"].session = "slam-z"
    time.sleep(0.2)
    assert len(bot.sends) == n
