# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.156.E (D2, bar G4's in-process half): a mission that does not explore
gets a navigator for the trip home only
(`docs/plans/ros-alignment/3.156-return-to-dock.md`, section E).

* The runner gates it (`_GatedNavigator(goals_allowed=..., clears_keepout=False)`):
  no goal before the return phase, the agent never holds it, and the robot
  server's keepout marks are not cleared at the end (no search goal of this
  mission's made them).
* `control/brain_server.py` builds it only with `return_to_dock` on and no
  explore search; a world that cannot take goals, an unreachable server or a
  refused secret starts the mission as before, its return `skipped`,
  `no_nav2`. Off builds none.
"""

import json
import logging

import pytest
from fastapi.testclient import TestClient

from control.mission_runner import EXHAUSTED, MissionRunner, _GatedNavigator
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_brain_server import RecordingRobot, fresh_mock_robot
from tests.test_dock_return import CELL, RecNav, docked_at_start


class KeepoutNav(RecNav):
    def __init__(self, robot, **kw):
        super().__init__(robot, **kw)
        self.cleared = 0
        self.closed = 0
        self.sent = []

    def set_goal(self, x_m, y_m):
        self.sent.append((x_m, y_m))
        return super().set_goal(x_m, y_m)

    def clear_keepout(self):
        self.cleared += 1
        return {"cleared": 0}

    def close(self):
        self.closed += 1


def frontier(return_to_dock=True, nav=True, **kw):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    navigator = KeepoutNav(robot) if nav else None
    runner = MissionRunner(robot, policy="frontier", navigator=navigator,
                           world=MockWorld(grid), target_object="red backpack",
                           max_steps=kw.pop("max_steps", 200), clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=return_to_dock, **kw)
    return runner, navigator, grid


def test_a_mission_that_does_not_explore_goes_home_on_its_navigator():
    runner, nav, grid = frontier()
    runner.start()
    for _ in range(8):
        runner.tick()
    assert nav.sent == [], "no goal before the end decision"
    runner._finish(EXHAUSTED, "decided")
    n = 0
    while runner.tick() and n < 3000:
        n += 1
    s = runner.status()
    assert s["outcome"] == EXHAUSTED and s["end_reason"] == "decided"
    assert nav.sent, "the trip home used the navigator"
    docked_at_start(s, grid)


def test_the_search_never_holds_the_return_navigator():
    runner, nav, _ = frontier()
    assert isinstance(runner._navigator, _GatedNavigator)
    assert getattr(runner.agent, "navigator", None) is None
    runner.start()
    assert runner._navigator.set_goal(1.0, 1.0) == {"accepted": False, "reason": "not_returning"}
    assert nav.sent == [] and not runner._navigator.sent_goal
    runner.stop("test")


def test_a_return_only_navigator_leaves_the_keepout_marks():
    runner, nav, _ = frontier()
    runner.start()
    runner.tick()
    runner.stop("a person")              # no return after a person: the end now
    runner._navigator.end()              # what _end's thread runs; idempotent
    assert nav.cleared == 0
    assert nav.closed == 1               # the client is still closed


def test_an_explore_navigator_still_clears_them():
    from tests.test_dock_return import mission

    runner, nav, grid, robot = mission(return_to_dock=False)
    nav.cleared = 0
    nav.clear_keepout = lambda: setattr(nav, "cleared", nav.cleared + 1) or {"cleared": 0}
    runner.start()
    runner.tick()
    runner.stop("a person")
    runner._navigator.end()
    assert nav.cleared == 1


def test_off_holds_no_navigator_for_a_mission_that_does_not_explore():
    runner, nav, _ = frontier(return_to_dock=False)
    assert runner._navigator is None


def test_no_navigator_is_skipped_no_nav2_and_the_mission_ends():
    runner, _, grid = frontier(nav=False)
    runner.start()
    runner.tick()
    runner._finish(EXHAUSTED, "decided")
    s = runner.status()
    assert not runner.is_running()
    assert s["return"] == {"state": "skipped", "reason": "no_nav2"}


def test_the_time_reserve_now_ends_a_mission_that_does_not_explore():
    """The one intended change (pinned): with a navigator home, D3's
    reserve applies to every mission, as it does to explore's."""
    runner, nav, grid = frontier(mission_time_limit_s=20.0, max_steps=2000)
    runner.start()
    n = 0
    while runner.tick() and n < 4000:
        n += 1
    s = runner.status()
    assert s["outcome"] == "out_of_budget" and "return reserve" in s["end_reason"], s["end_reason"]


# ---------- the brain server ----------

class _Nav:
    """RemoteNavigator's surface; `ok` is what available() answers."""
    built = []
    ok = True

    def __init__(self, *a, **k):
        self.closed = False
        _Nav.built.append(self)

    def available(self):
        if isinstance(_Nav.ok, Exception):
            raise _Nav.ok
        return _Nav.ok

    def close(self):
        self.closed = True

    def set_goal(self, x_m, y_m):
        return {"accepted": False, "reason": "stub"}

    def get_goal(self):
        return {"goal": None}

    def cancel_goal(self):
        return {}


def _app(tmp_path, monkeypatch, ok, return_to_dock=True):
    import control.brain_server as bs
    _Nav.built, _Nav.ok = [], ok
    monkeypatch.setattr(bs, "RemoteNavigator", _Nav)
    config = tmp_path / "robot.yaml"
    config.write_text(f"brain:\n  tick_interval_s: 0.0\n  return_to_dock: {str(return_to_dock).lower()}\n")
    return bs.create_app(config_path=str(config),
                         robot_factory=lambda: RecordingRobot(fresh_mock_robot()))


def _start(app):
    with TestClient(app) as client:
        resp = client.post("/mission/start", json={"target_object": "x", "policy": "frontier"})
        runner = app.state.brain_state["runner"]
        client.post("/mission/stop")
    assert resp.status_code == 200, resp.text
    return runner


def test_a_world_that_takes_goals_gives_the_return_its_navigator(tmp_path, monkeypatch):
    runner = _start(_app(tmp_path, monkeypatch, ok=True))
    assert len(_Nav.built) == 1 and isinstance(runner._navigator, _GatedNavigator)
    assert runner._navigator._nav is _Nav.built[0]


@pytest.mark.parametrize("ok", [False, ConnectionError("refused")])
def test_a_world_that_cannot_take_goals_still_starts(tmp_path, monkeypatch, ok):
    import httpx
    ok = httpx.ConnectError("refused") if isinstance(ok, ConnectionError) else ok
    runner = _start(_app(tmp_path, monkeypatch, ok=ok))
    assert runner._navigator is None
    assert all(n.closed for n in _Nav.built), "a navigator not used is closed"


def test_a_refused_secret_starts_and_is_a_warning(tmp_path, monkeypatch, caplog):
    with caplog.at_level(logging.WARNING, logger="brain_server"):
        runner = _start(_app(tmp_path, monkeypatch, ok=PermissionError("HTTP 401")))
    assert runner._navigator is None and _Nav.built[0].closed
    assert any("refused the brain's secret" in r.getMessage() for r in caplog.records)


def test_off_builds_no_navigator(tmp_path, monkeypatch):
    runner = _start(_app(tmp_path, monkeypatch, ok=True, return_to_dock=False))
    assert _Nav.built == [] and runner._navigator is None


# ---------- G4: off is dev's end, pinned ----------

def _g4():
    import importlib.util
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / "evaluations/dock-3156e/g4_off.py"
    spec = importlib.util.spec_from_file_location("g4_off_3156e", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod, json.loads(path.with_name("g4-dev.json").read_text())


@pytest.mark.parametrize("row", [1, 8, 12, 18])
def test_g4_off_ends_as_dev_did(row):
    """3.156.E G4 (in process): with the return off, a mission that does not
    explore ends as `dev`'s did on the same seeded start -- outcome, steps,
    metres, end pose -- and holds no navigator. The full 20 were compared
    against a `dev` worktree (0 differ, `evaluations/dock-3156e/`); four of
    the fast rows (three finds, one 300-step budget end) are re-run here."""
    g4, dev = _g4()
    house, start = g4.starts()[row]
    got = g4.one(house, start)
    assert g4.compare([got], [dev[row]]) == []
    assert not got["navigator"] and got["goals_sent"] == 0
