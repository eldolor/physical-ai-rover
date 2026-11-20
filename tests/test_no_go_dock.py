# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.161.B part 2: a no-go zone over the dock is refused (E1-E6, bars E-a..E-d
and E-f of docs/plans/ros-alignment/3.161-no-go-zones-on-twin.md).

The dock in these tests faces north (heading 0) at (2.0, 2.0); the staging
point is STAGING_M behind it, at (2.0, 2.3) (a robot facing h moves along
(sin h, -cos h)), so the return's last leg is the segment x = 2.0,
y in [2.0, 2.3].
"""

import json
import math

import pytest
from fastapi.testclient import TestClient

from brain import dock as docking
from control import nogo_store
from control.mission_runner import MissionRunner
from robot.safety import FOOTPRINT_LENGTH_M, FOOTPRINT_WIDTH_M
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from world import nogo

FRAME = "map-d0c4beef"
DOCK = {"x_m": 2.0, "y_m": 2.0, "theta": 0.0, "source": docking.SAVED_MAP}
SAVED_DOCK = {"x_m": 2.0, "y_m": 2.0, "heading_deg": 0.0}


def _z(x0, y0, x1, y1, kind="area"):
    return {"x0": x0, "y0": y0, "x1": x1, "y1": y1, "kind": kind}


# Covering cases (E-a): over the dock; over the staging point only; a wall
# across the leg; a zone 0.17 m from the leg.
COVERING = {
    "over_the_dock": _z(1.8, 1.8, 2.2, 2.2),
    "over_the_staging_point_only": _z(1.8, 2.25, 2.2, 2.6),
    "a_wall_across_the_leg": nogo.wall_from_line(1.0, 2.15, 3.0, 2.15),
    "0.17_m_from_the_leg": _z(2.17, 1.0, 2.6, 3.0),
}
CLEAR = _z(2.19, 1.0, 2.6, 3.0)          # 0.19 m from the leg


# ---------- E1: geometry ----------

def test_the_margin_covers_the_chassis():
    r = math.hypot(FOOTPRINT_LENGTH_M / 2, FOOTPRINT_WIDTH_M / 2)
    assert r <= nogo.DOCK_CLEAR_M < r + 0.02


def test_the_leg_is_staging_to_dock():
    a, b = nogo_store.dock_leg(DOCK)
    assert b == pytest.approx((2.0, 2.0))
    assert a == pytest.approx((2.0, 2.0 + docking.STAGING_M))


@pytest.mark.parametrize("name", sorted(COVERING))
def test_covering_zones_cover_the_leg(name):
    z = nogo.clean_zone(COVERING[name])
    assert nogo_store.dock_conflicts([z], DOCK) == [z["id"]]


def test_a_zone_019_clear_does_not_cover_it():
    assert nogo_store.dock_conflicts([nogo.clean_zone(CLEAR)], DOCK) == []


def test_segment_distance_is_exact():
    z = nogo.clean_zone(_z(0.0, 0.0, 1.0, 1.0))
    assert nogo.segment_distance(z, (2.0, 0.5), (3.0, 0.5)) == pytest.approx(1.0)
    assert nogo.segment_distance(z, (-1.0, 0.5), (2.0, 0.5)) == 0.0     # crosses it
    assert nogo.segment_distance(z, (2.0, 2.0), (2.0, 2.0)) == pytest.approx(math.sqrt(2))
    assert nogo.segment_distance(z, (0.5, 2.0), (2.0, 0.5)) == pytest.approx(
        math.sqrt(2) / 4)                                                   # past the corner


def test_no_dock_no_conflict():
    z = nogo.clean_zone(COVERING["over_the_dock"])
    assert nogo_store.dock_conflicts([z], None) == []
    assert nogo_store.split_for_dock([z], None) == ([z], [])


# ---------- E3: the store at the draw (E-a, E-b) ----------

@pytest.mark.parametrize("name", sorted(COVERING))
def test_an_add_covering_a_known_dock_is_refused_and_nothing_stored(tmp_path, name):
    store = nogo_store.NoGoStore(tmp_path)
    with pytest.raises(nogo_store.NoGoRefused) as e:
        store.edit(FRAME, "add", zone=COVERING[name], dock=DOCK)
    assert str(e.value) == nogo_store.DOCK_REFUSED
    assert "dock" in str(e.value) and "home" in str(e.value)
    assert not (tmp_path / f"{FRAME}.json").exists()


def test_an_add_clear_of_the_dock_is_stored_as_on_dev(tmp_path):
    store = nogo_store.NoGoStore(tmp_path)
    doc = store.edit(FRAME, "add", zone=CLEAR, dock=DOCK)
    assert doc["rev"] == 1 and len(doc["zones"]) == 1


def test_with_no_known_dock_every_add_is_accepted(tmp_path):
    store = nogo_store.NoGoStore(tmp_path)
    for name in sorted(COVERING):
        store.edit(FRAME, "add", zone=COVERING[name])
    assert len(store.zones(FRAME)) == len(COVERING)


# ---------- the brain's routes (E2, E3, E-c) ----------

class _World:
    def __init__(self, dock=SAVED_DOCK):
        self.dock = dock

    def get_pose(self):
        p = {"usable": True, "x_m": 1.0, "y_m": 1.0, "heading_deg": 0.0,
             "map_id": "slam-x", "frame_id": FRAME}
        if self.dock is not None:
            p["dock"] = dict(self.dock)
        return p


@pytest.fixture
def brain(tmp_path, monkeypatch):
    monkeypatch.setenv("NOGO_DIR", str(tmp_path))
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(nogo_store, "_DEFAULT", None)
    from control import brain_server

    def make(dock=SAVED_DOCK):
        return TestClient(brain_server.create_app(robot_factory=lambda: object(),
                                                  world_factory=lambda r: _World(dock)))
    return make


def test_the_route_refuses_a_zone_over_the_saved_dock_in_plain_words(brain, tmp_path):
    c = brain()
    r = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 0,
                                      "zone": COVERING["over_the_dock"]})
    assert r.status_code == 409 and r.json()["detail"] == nogo_store.DOCK_REFUSED
    assert not (tmp_path / f"{FRAME}.json").exists()
    r = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 0,
                                      "line": [[1.0, 2.15], [3.0, 2.15]]})
    assert r.status_code == 409 and r.json()["detail"] == nogo_store.DOCK_REFUSED
    v = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 0,
                                      "zone": CLEAR}).json()
    assert len(v["zones"]) == 1 and v["dock_conflicts"] == []


def test_a_zone_saved_before_is_reported_and_its_file_kept(brain, tmp_path):
    store = nogo_store.NoGoStore(tmp_path)
    store.edit(FRAME, "add", zone=COVERING["over_the_dock"])     # no dock known then
    store.edit(FRAME, "add", zone=CLEAR)
    before = (tmp_path / f"{FRAME}.json").read_bytes()
    over = store.zones(FRAME)[0]["id"]
    v = brain().get("/mission/nogo").json()
    assert [d["id"] for d in v["dock_conflicts"]] == [over]
    assert v["dock_conflicts"][0]["reason"] == nogo_store.DOCK_CONFLICT
    assert len(v["zones"]) == 2                     # still listed: a person deletes it
    assert (tmp_path / f"{FRAME}.json").read_bytes() == before


def test_with_no_known_dock_the_route_accepts_and_reports_nothing(brain):
    c = brain(dock=None)
    v = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 0,
                                      "zone": COVERING["over_the_dock"]}).json()
    assert len(v["zones"]) == 1 and v["dock_conflicts"] == []


def test_the_robot_server_is_sent_only_zones_in_force(brain, tmp_path, monkeypatch):
    store = nogo_store.NoGoStore(tmp_path)
    store.edit(FRAME, "add", zone=COVERING["over_the_dock"])
    sent = []
    monkeypatch.setattr(nogo_store, "push",
                        lambda base, secret, f, zones, **kw: sent.append(list(zones))
                        or {"accepted": True})
    from control import brain_server

    class R:
        base_url = "http://robot.invalid"
    c = TestClient(brain_server.create_app(robot_factory=lambda: R(),
                                           world_factory=lambda r: _World()))
    v = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 1,
                                      "zone": CLEAR}).json()
    assert len(v["zones"]) == 2
    assert [[z["id"] for z in s] for s in sent] == [[v["zones"][1]["id"]]]


# ---------- E4, E5: the mission (E-c, E-d) ----------

class _Saved(MockWorld):
    """The simulator's world on a saved map (its frame named), no saved dock."""

    def get_pose(self):
        p = super().get_pose()
        return dict(p, frame_id=FRAME) if p.get("usable") else p


def _runner(return_to_dock=True, max_steps=6):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    runner = MissionRunner(robot, target_object="red backpack", policy="explore", navigator=nav,
                           world=_Saved(grid), max_steps=max_steps,
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           return_to_dock=return_to_dock)
    x, y = grid.x * 0.3, grid.y * 0.3
    over = nogo.clean_zone(_z(x - 0.4, y - 0.4, x + 0.4, y + 0.4))      # over the start
    far = nogo.clean_zone(_z(3.0, 2.0, 4.5, 3.0))
    return runner, grid, over, far


def test_a_zone_over_the_missions_dock_is_not_in_force_and_the_trip_home_docks():
    runner, grid, over, far = _runner()
    runner.set_nogo(FRAME, [over, far])          # before the start: no dock yet
    sent = []
    runner.nogo_sink = lambda f, z: sent.append([q["id"] for q in z])
    runner.start()
    assert runner.nogo_zones == [far]
    assert runner.nogo_dock_refused == [over["id"]]
    assert runner.agent.zones == [far]
    n = 0
    while runner.tick() and n < 5000:
        n += 1
    s = runner.status()
    assert s["explore"]["nogo"]["dock_refused"] == [over["id"]]
    assert (s["return"] or {}).get("state") == docking.DOCKED, s["return"]
    lines = [ln for ln in runner._log if "covers the dock" in ln]
    assert len(lines) == 1 and over["id"] in lines[0]
    # An edit mid-mission keeps the rule.
    runner.set_nogo(FRAME, [over, far])
    assert runner.nogo_zones == [far] and runner.nogo_dock_refused == [over["id"]]
    assert len([ln for ln in runner._log if "covers the dock" in ln]) == 1


def test_with_no_trip_home_the_same_zone_stays_in_force():
    runner, grid, over, far = _runner(return_to_dock=False)
    runner.set_nogo(FRAME, [over, far])
    runner.start()
    assert runner.nogo_zones == [over, far] and runner.nogo_dock_refused == []
    assert runner.agent.zones == [over, far]


def test_the_lookup_sends_the_robot_server_only_zones_in_force():
    runner, grid, over, far = _runner()
    runner.start()
    sent = []
    runner.nogo_source = lambda f: [over, far]
    runner.nogo_sink = lambda f, z: sent.append([q["id"] for q in z])
    runner._nogo_lookup(FRAME)
    assert sent == [[far["id"]]]


def test_dock_conflicts_report_is_json_plain():
    z = nogo.clean_zone(COVERING["over_the_dock"])
    json.dumps(nogo_store.dock_report([z], DOCK))


# ---------- review round 1 ----------

def test_with_no_navigator_there_is_no_trip_home_to_keep_clear():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object="red backpack", policy="frontier",
                           world=_Saved(grid), max_steps=3, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=True)
    x, y = grid.x * 0.3, grid.y * 0.3
    over = nogo.clean_zone(_z(x - 0.4, y - 0.4, x + 0.4, y + 0.4))
    runner.set_nogo(FRAME, [over])
    runner.start()
    assert runner.nogo_zones == [over] and runner.nogo_dock_refused == []


def test_a_dock_read_on_one_saved_map_never_filters_anothers_zones():
    runner, grid, over, far = _runner()
    runner.read_dock(runner.world.get_pose())
    runner.set_nogo(FRAME, [over, far])
    assert runner.nogo_dock_refused == [over["id"]]
    runner.set_nogo("map-other", [over, far])
    assert runner.nogo_zones == [over, far] and runner.nogo_dock_refused == []


def test_start_reads_the_dock_again_when_the_brains_read_was_unusable():
    runner, grid, over, far = _runner()
    runner.read_dock({"usable": False})
    runner.set_nogo(FRAME, [over, far])
    assert runner.nogo_dock_refused == []
    sent = []
    runner.nogo_sink = lambda f, z: sent.append([q["id"] for q in z])
    runner.start()
    assert runner.nogo_dock_refused == [over["id"]] and sent == [[far["id"]]]


class _Ran:
    """A mission on FRAME whose dock is its start pose (no saved dock)."""
    nogo_frame, _saved_map = FRAME, {"frame_id": FRAME}
    nogo_dock_refused: list = []

    def __init__(self, running):
        self.running = running

    def is_running(self):
        return self.running

    def _nogo_dock(self):
        return {"x_m": 2.0, "y_m": 2.0, "theta": 0.0, "source": docking.START}

    def set_nogo(self, frame_id, zones):
        self.nogo_zones = list(zones)


def test_a_finished_missions_start_is_no_dock_for_the_editor(brain):
    c = brain(dock=None)
    c.app.state.brain_state["runner"] = _Ran(running=False)
    v = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 0,
                                      "zone": COVERING["over_the_dock"]}).json()
    assert len(v["zones"]) == 1 and v["dock_conflicts"] == []


def test_a_running_missions_dock_refuses_and_its_own_list_is_reported(brain):
    c = brain(dock=None)
    ran = _Ran(running=True)
    c.app.state.brain_state["runner"] = ran
    r = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 0,
                                      "zone": COVERING["over_the_dock"]})
    assert r.status_code == 409 and r.json()["detail"] == nogo_store.DOCK_REFUSED
    v = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 0,
                                      "zone": CLEAR}).json()
    ran.nogo_dock_refused = [v["zones"][0]["id"]]       # the mission's own word
    v = c.get("/mission/nogo").json()
    assert [d["id"] for d in v["dock_conflicts"]] == [v["zones"][0]["id"]]


# ---------- review round 2 ----------

def test_a_dock_read_with_no_saved_map_never_filters_a_saved_maps_zones():
    runner, grid, over, far = _runner()
    runner.world = MockWorld(grid)                 # no frame_id: a fresh map's start
    runner.set_nogo(FRAME, [over, far])
    runner.start()
    assert runner.nogo_zones == [over, far] and runner.nogo_dock_refused == []


def test_the_saved_dock_is_kept_clear_with_no_trip_home():
    runner, grid, over, far = _runner(return_to_dock=False)
    x, y = grid.x * 0.3, grid.y * 0.3

    class Docked(_Saved):
        def get_pose(self):
            return dict(super().get_pose(), dock={"x_m": x, "y_m": y, "heading_deg": 0.0})
    runner.world = Docked(grid)
    runner.set_nogo(FRAME, [over, far])
    runner.start()
    assert runner.nogo_dock_refused == [over["id"]] and runner.nogo_zones == [far]


def test_the_saved_dock_refuses_the_draw_while_a_mission_without_a_return_runs(brain):
    c = brain()                                     # the pose names the saved dock

    class NoReturn(_Ran):
        def _nogo_dock(self):
            return None
    c.app.state.brain_state["runner"] = NoReturn(running=True)
    r = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 0,
                                      "zone": COVERING["over_the_dock"]})
    assert r.status_code == 409 and r.json()["detail"] == nogo_store.DOCK_REFUSED


def test_a_mission_that_left_zones_out_sends_the_stored_set_again_at_its_end(
        tmp_path, monkeypatch):
    monkeypatch.setenv("NOGO_DIR", str(tmp_path))
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(nogo_store, "_DEFAULT", None)
    store = nogo_store.NoGoStore(tmp_path)
    store.edit(FRAME, "add", zone=COVERING["over_the_dock"])
    sent = []
    monkeypatch.setattr(nogo_store, "push",
                        lambda base, secret, f, zones, **kw: sent.append(
                            (f, [z["id"] for z in zones])) or {"accepted": True})
    from control import brain_server

    class R:
        base_url = "http://robot.invalid"
    runner, grid, over, far = _runner(max_steps=3)
    app = brain_server.create_app(robot_factory=lambda: R(),
                                  world_factory=lambda r: _World(dock=None))
    zid = store.zones(FRAME)[0]["id"]
    runner.set_nogo(FRAME, store.zones(FRAME))
    runner.nogo_dock_refused = [zid]                # left out for its start-pose dock
    app.state.brain_state["runner"] = runner
    app.state.nogo_restore(runner)                  # what mission_loop runs at the end
    assert sent == [(FRAME, [zid])]
    sent.clear()
    app.state.brain_state["runner"] = object()      # a newer mission: never overwritten
    app.state.nogo_restore(runner)
    assert sent == []
