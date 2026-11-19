# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.161.B: no-go zones a person draws -- the geometry (`world/nogo.py`), the
brain's store (`control/nogo_store.py`, D2), the robot server's copy and its
refusal of a goal in one (D5, Z6), the bridge's mask and its suspension
(`picar_bridge/keepout.py`, D2/D4), explore's goal choice (D3, Z1) and the
"nothing else moves" checks (Z4, Z9, Z10).
docs/plans/ros-alignment/3.161-no-go-zones-on-twin.md."""

import json
import math
import os
import sys
from pathlib import Path

import httpx
import pytest
from fastapi.testclient import TestClient

import robot.server as server
from control import nogo_store
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from tests.test_ros_goals import world_with
from tests.test_ros_world import FakeTruth
from tests.test_wedge_watch import KeepoutBridge
from world import nogo
from world.interface import CELL_FREE, CELL_OCCUPIED

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "service/slam/src/picar_bridge"))
from picar_bridge import keepout  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
FRAME = "map-5161cafe"


# ---------- D1: a zone ----------

def test_a_zone_is_normalised_and_checked():
    z = nogo.clean_zone({"x0": 2.0, "y0": 3.0, "x1": 1.0, "y1": 1.5})
    assert (z["x0"], z["y0"], z["x1"], z["y1"]) == (1.0, 1.5, 2.0, 3.0)
    assert z["kind"] == "area" and z["by"] == "person" and nogo.ID_RE.fullmatch(z["id"])
    for bad in ({"x0": 0, "y0": 0, "x1": 0.1, "y1": 1},          # under 0.20 m a side
                {"x0": "nan", "y0": 0, "x1": 1, "y1": 1},
                {"x0": 0, "y0": 0, "x1": 1e6, "y1": 1},
                {"x0": True, "y0": 0, "x1": 1, "y1": 1},
                {"x0": 0, "y0": 0, "x1": 1, "y1": 1, "kind": "lava"},
                {"x0": 0, "y0": 0, "x1": 1, "y1": 1, "id": "../x"}):
        with pytest.raises(nogo.ZoneRefused):
            nogo.clean_zone(bad)


def test_a_wall_is_a_tenth_of_a_metre_along_the_lines_longer_axis():
    w = nogo.clean_zone(nogo.wall_from_line(1.0, 2.0, 3.0, 2.2))
    assert w["kind"] == "wall"
    assert (w["x0"], w["x1"]) == (1.0, 3.0)
    assert w["y1"] - w["y0"] == pytest.approx(nogo.WALL_WIDTH_M)
    v = nogo.clean_zone(nogo.wall_from_line(1.0, 1.0, 1.1, 0.0))
    assert v["x1"] - v["x0"] == pytest.approx(nogo.WALL_WIDTH_M) and v["y1"] - v["y0"] == 1.0
    with pytest.raises(nogo.ZoneRefused):
        nogo.clean_zone(nogo.wall_from_line(1.0, 1.0, 1.1, 1.05))      # too short


def test_at_most_thirty_and_ids_unique():
    zs = [nogo.clean_zone({"x0": i, "y0": 0, "x1": i + 0.5, "y1": 0.5}) for i in range(31)]
    with pytest.raises(nogo.ZoneRefused):
        nogo.clean_zones(zs)
    with pytest.raises(nogo.ZoneRefused):
        nogo.clean_zones([zs[0], dict(zs[1], id=zs[0]["id"])])


def test_inside_distance_and_a_grown_zone():
    z = nogo.clean_zone({"x0": 1, "y0": 1, "x1": 2, "y1": 2})
    assert nogo.inside(z, 1.5, 1.5) and not nogo.inside(z, 2.05, 1.5)
    assert nogo.inside(z, 2.10, 1.5, grow=0.12) and not nogo.inside(z, 2.10, 2.10, grow=0.12)
    assert nogo.distance(z, 3.0, 2.0) == pytest.approx(1.0)
    assert nogo.distance(z, 1.5, 1.5) == 0.0


def test_a_masked_map_is_a_copy_and_keeps_its_cells_type():
    m = {"usable": True, "width": 4, "height": 3, "resolution_m": 0.5,
         "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": [CELL_FREE] * 12, "map_version": 3}
    z = nogo.clean_zone({"x0": 0.5, "y0": 0.5, "x1": 1.0, "y1": 1.0})
    out = nogo.masked_map(m, [z], CELL_OCCUPIED)
    assert m["cells"] == [CELL_FREE] * 12                       # the input untouched
    assert isinstance(out["cells"], list)
    assert out["cells"][1 * 4 + 1] == CELL_OCCUPIED and sum(out["cells"]) == CELL_OCCUPIED
    assert nogo.masked_map(m, [], CELL_OCCUPIED) is m           # no zones: the map itself


# ---------- D2: the brain's store; Z4: kept ----------

def test_the_store_adds_deletes_and_keeps_across_a_reload(tmp_path):
    s = nogo_store.NoGoStore(tmp_path)
    doc = s.edit(FRAME, "add", zone={"x0": 1, "y0": 1, "x1": 2, "y1": 2})
    doc = s.edit(FRAME, "add", rev=doc["rev"], zone=nogo.wall_from_line(0, 3, 2, 3))
    assert [z["kind"] for z in doc["zones"]] == ["area", "wall"] and doc["rev"] == 2
    # A reload (a new process: a new store on the same directory), Z4.
    again = nogo_store.NoGoStore(tmp_path)
    assert again.zones(FRAME) == doc["zones"]
    with pytest.raises(nogo_store.NoGoRefused):
        again.edit(FRAME, "delete", rev=1, zone_id=doc["zones"][0]["id"])   # stale rev
    left = again.edit(FRAME, "delete", rev=2, zone_id=doc["zones"][0]["id"])
    assert [z["kind"] for z in left["zones"]] == ["wall"]
    assert again.edit(FRAME, "clear")["zones"] == []
    assert not list(tmp_path.glob(".*"))                       # no temp file left


def test_no_saved_map_refuses_every_edit_and_reads_empty(tmp_path):
    s = nogo_store.NoGoStore(tmp_path)
    assert s.zones(None) == []
    for frame in (None, "", "../etc", "a/b"):
        with pytest.raises(nogo_store.NoGoRefused, match="Survey first"):
            s.edit(frame, "add", zone={"x0": 1, "y0": 1, "x1": 2, "y1": 2})
    assert list(tmp_path.iterdir()) == []


def test_a_damaged_file_is_never_overwritten(tmp_path):
    (tmp_path / f"{FRAME}.json").write_text("{not json")
    s = nogo_store.NoGoStore(tmp_path)
    assert s.load(FRAME)["unreadable"] and s.zones(FRAME) == []
    with pytest.raises(nogo_store.NoGoRefused, match="damaged"):
        s.edit(FRAME, "add", zone={"x0": 1, "y0": 1, "x1": 2, "y1": 2})
    assert (tmp_path / f"{FRAME}.json").read_text() == "{not json"


def test_the_store_refuses_a_thirty_first_zone(tmp_path):
    s = nogo_store.NoGoStore(tmp_path)
    for i in range(nogo.MAX_ZONES):
        s.edit(FRAME, "add", zone={"x0": i, "y0": 0, "x1": i + 0.5, "y1": 0.5})
    with pytest.raises(nogo_store.NoGoRefused, match="at most 30"):
        s.edit(FRAME, "add", zone={"x0": 40, "y0": 0, "x1": 40.5, "y1": 0.5})


# ---------- Z9: local only ----------

def test_zones_live_under_recordings_which_git_ignores():
    assert nogo_store.DEFAULT_DIR == ROOT / "recordings" / "nogo"
    import subprocess
    r = subprocess.run(["git", "check-ignore", "-q", "recordings/nogo/map-x.json"], cwd=ROOT)
    assert r.returncode == 0, "the zones directory is not ignored by git"


UPLOAD_PATHS = ["control/walk_store.py", "control/inventory_store.py", "control/metrics_client.py",
                "control/recording_routes.py", "control/stream_recorder.py",
                "service/lambda", "service/static/sync.sh", "service/vision_analyze",
                "brain/vision.py", "brain/navigate.py", "cloudformation"]


def test_no_upload_path_reads_the_zones():
    """Z9: every path that leaves the machine (S3 stores, the metrics row, the
    walks Lambda, the static deploy, the cloud vision calls) is checked for
    any reference to the zones' module, store or directory."""
    hits = []
    for rel in UPLOAD_PATHS:
        p = ROOT / rel
        files = [p] if p.is_file() else [f for f in p.rglob("*") if f.is_file()
                                          and f.suffix in (".py", ".sh", ".yaml", ".yml", ".json")]
        assert files, rel
        for f in files:
            text = f.read_text(errors="ignore")
            if any(k in text for k in ("nogo", "no_go", "NOGO_DIR")):
                hits.append(str(f.relative_to(ROOT)))
    assert hits == []


def test_the_metrics_row_carries_no_zone(tmp_path):
    from control.metrics_client import row_for
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object="red backpack", policy="explore",
                           navigator=FakeNav(robot), world=MockWorld(grid), max_steps=3,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.set_nogo(FRAME, [nogo.clean_zone({"x0": 9.123, "y0": 9.456, "x1": 9.9, "y1": 9.9})])
    runner.start()
    while runner.tick():
        pass
    row = json.dumps(row_for(runner.status()))
    assert "9.123" not in row and "9.456" not in row and "x0" not in row


# ---------- D5, Z6: the robot server ----------

class ZoneBridge(KeepoutBridge):
    """KeepoutBridge on a saved map (`map_id` on its pose), plus /nav/zones."""

    def __init__(self, frame=FRAME, **kw):
        super().__init__(**kw)
        self.frame = frame
        self.zones = None
        self.zone_posts = 0

    def handler(self, request):
        if request.url.path == "/slam/pose":
            r = super().handler(request)
            return httpx.Response(200, json={**r.json(), "map_id": self.frame})
        if request.url.path == "/nav/zones":
            self.zones = json.loads(request.content)["zones"]
            self.zone_posts += 1
            return httpx.Response(200, json={"zones": len(self.zones), "suspended": []})
        if request.url.path == "/health":
            return httpx.Response(200, json={"nogo": {"zones": len(self.zones or []),
                                                      "suspended": []}})
        return super().handler(request)


@pytest.fixture
def zoned_server(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    truth = FakeTruth(1.0, 1.0, 90.0)
    bridge = ZoneBridge(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    world, _ = world_with(truth=truth, bridge=bridge)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    return TestClient(server.create_app()), bridge, world


ZONE = {"x0": 2.0, "y0": 0.5, "x1": 3.0, "y1": 1.5}


def test_a_set_reaches_the_bridge_as_ros_polygons(zoned_server):
    client, bridge, world = zoned_server
    z = nogo.clean_zone(ZONE)
    r = client.put("/world/nogo", json={"frame_id": FRAME, "zones": [z]})
    assert r.status_code == 200 and r.json()["sent"] is True, r.text
    poly = bridge.zones[0]["poly"]
    # Facing east from (1, 1): x 2..3 is 1..2 m ahead (ROS +x); y 0.5..1.5
    # (north is smaller y, to the left) is ROS +0.5..-0.5.
    xs, ys = sorted({p[0] for p in poly}), sorted({p[1] for p in poly})
    assert xs == pytest.approx([1.0, 2.0]) and ys == pytest.approx([-0.5, 0.5])
    assert client.get("/health").json()["nogo"] == {"frame_id": FRAME, "zones": 1, "sent": True}
    got = client.get("/world/nogo").json()
    assert got["zones"][0]["id"] == z["id"] and got["bridge"]["zones"] == 1


def test_a_set_for_another_saved_map_is_kept_but_not_drawn(zoned_server):
    client, bridge, _ = zoned_server
    r = client.put("/world/nogo", json={"frame_id": "map-other", "zones": [nogo.clean_zone(ZONE)]})
    assert r.json()["sent"] is False and r.json()["detail"] == "another saved map"
    assert bridge.zones is None


def test_a_bad_set_is_refused_422(zoned_server):
    client, _, _ = zoned_server
    assert client.put("/world/nogo", json={"frame_id": FRAME, "zones": [{"x0": 0}]}).status_code == 422
    assert client.put("/world/nogo", json={"zones": [nogo.clean_zone(ZONE)]}).status_code == 422


def test_a_goal_tap_inside_a_zone_is_refused_ten_of_ten(zoned_server):
    """Z6: a person's tap (x-driver twin-dpad) inside a zone, 10 places."""
    client, bridge, _ = zoned_server
    client.put("/world/nogo", json={"frame_id": FRAME, "zones": [nogo.clean_zone(ZONE)]})
    refused = 0
    for i in range(10):
        x, y = 2.05 + 0.09 * i, 0.55 + 0.09 * i
        r = client.post("/world/goal", json={"x_m": x, "y_m": y},
                        headers={"x-driver": "twin-dpad"}).json()
        refused += r == {**r, "accepted": False, "reason": "no_go_zone"}
    assert refused == 10
    assert bridge.goal is None                                   # none reached nav2
    # Outside the zone, the same person's tap is taken.
    assert client.post("/world/goal", json={"x_m": 1.5, "y_m": 1.0},
                       headers={"x-driver": "twin-dpad"}).json()["accepted"]


def test_a_dpad_press_is_never_refused_for_a_zone(zoned_server):
    """Z6's other half: a zone is not a safety veto (rule 4) -- the robot
    stands in one and the D-pad still drives it."""
    client, _, _ = zoned_server
    client.put("/world/nogo", json={"frame_id": FRAME, "zones": [
        nogo.clean_zone({"x0": 0.0, "y0": 0.0, "x1": 3.0, "y1": 2.0})]})
    for verb in ("FORWARD", "LEFT", "REVERSE"):
        r = client.post("/action", json={"action": verb}, headers={"x-driver": "twin-dpad"})
        body = r.json()
        assert "no_go" not in json.dumps(body), body


def test_the_mission_end_clear_leaves_the_zones(zoned_server):
    """Z4: DELETE /world/keepout clears the robot's spots only."""
    client, bridge, _ = zoned_server
    client.put("/world/nogo", json={"frame_id": FRAME, "zones": [nogo.clean_zone(ZONE)]})
    posts = bridge.zone_posts
    client.delete("/world/keepout")
    assert bridge.zone_posts == posts and len(bridge.zones) == 1
    assert client.get("/world/nogo").json()["zones"]


def test_safety_never_reads_a_zone():
    """Z10 / rule 4: robot/safety.py is not changed and imports nothing of it."""
    src = (ROOT / "robot/safety.py").read_text()
    assert "nogo" not in src and "no_go" not in src


# ---------- the bridge's mask (D2) and its suspension (D4) ----------

def _zone_poly(x0, y0, x1, y1, zid="z1"):
    return {"id": zid, "poly": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]}


def _marked(m):
    w, res = m["width"], m["resolution_m"]
    return [((i + 0.5) * res + m["origin_x_m"], (j + 0.5) * res + m["origin_y_m"])
            for j in range(m["height"]) for i in range(w) if m["data"][j * w + i] == 100]


def test_the_mask_is_the_union_of_spots_and_zones_and_unknown_elsewhere():
    spots = keepout.parse([{"x_m": 5.0, "y_m": 5.0, "r_m": 0.15, "ttl_s": 5}], now=0.0)
    zones = keepout.parse_zones([_zone_poly(0.0, 0.0, 1.0, 0.5)])
    m = keepout.mask(spots, zones=zones)
    pts = _marked(m)
    assert set(m["data"]) == {-1, 100}
    in_zone = [p for p in pts if p[0] < 2]
    assert len(in_zone) == 20 * 10                                 # 1.0 x 0.5 m at 0.05 m
    assert all(0 <= x <= 1 and 0 <= y <= 0.5 for x, y in in_zone)
    assert any(math.hypot(x - 5, y - 5) <= 0.15 for x, y in pts)
    # The spots' set replaced by nothing (POST /nav/keepout {spots: []}) or
    # all expired: the zone is still drawn (Z4).
    assert len(_marked(keepout.mask(keepout.live(spots, 10.0), zones=zones))) == 200


def test_a_rotated_zone_is_drawn_rotated():
    c, s = math.cos(math.radians(30)), math.sin(math.radians(30))
    poly = [[c * x - s * y, s * x + c * y] for x, y in ((0, 0), (1, 0), (1, 1), (0, 1))]
    m = keepout.mask([], zones=keepout.parse_zones([{"id": "r", "poly": poly}]))
    pts = _marked(m)
    assert abs(len(pts) * 0.05 ** 2 - 1.0) < 0.08
    for x, y in pts:                                               # back into the unit square
        u, v = c * x + s * y, -s * x + c * y
        assert -0.04 <= u <= 1.04 and -0.04 <= v <= 1.04


@pytest.mark.parametrize("bad", [
    [{"id": "a", "poly": [[0, 0], [1, 0], [1, 1]]}],
    [{"id": "a", "poly": [[0, 0], [1, 0], [1, "nan"], [0, 1]]}],
    [{"id": "a", "poly": [[0, 0], [99, 0], [99, 1], [0, 1]]}],
    [{"id": "a", "poly": [[0, 0], [1, 0], [1, 1], [0, 1]]}] * 2,
    [{"poly": [[0, 0], [1, 0], [1, 1], [0, 1]]}],
])
def test_a_malformed_zone_is_refused_not_drawn(bad):
    with pytest.raises((ValueError, KeyError, TypeError)):
        keepout.parse_zones(bad)


def test_a_zone_is_suspended_while_the_robot_is_in_it_and_restored_030_clear():
    z = keepout.parse_zones([_zone_poly(0.0, 0.0, 1.0, 1.0)])
    s = keepout.suspend(z, set(), (0.5, 0.5))
    assert s == {"z1"}
    s = keepout.suspend(z, s, (1.2, 0.5))                         # 0.20 m out: still left out
    assert s == {"z1"}
    s = keepout.suspend(z, s, (1.31, 0.5))                        # 0.31 m clear: drawn again
    assert s == set()
    assert keepout.suspend(z, set(), None) == set()               # no pose: nothing changes


def test_a_path_along_the_edge_never_suspends_a_zone():
    """A plan that hugs the lethal edge, or a robot one cell in (NavFn clears
    the robot's own cell), keeps the zone drawn."""
    z = keepout.parse_zones([_zone_poly(0.0, 0.0, 1.0, 1.0)])
    for p in ((1.03, 0.5), (1.0, 0.5), (0.97, 0.5), (0.5, -0.02)):
        assert keepout.suspend(z, set(), p) == set(), p


def test_a_zone_arriving_with_the_robot_close_is_suspended():
    z = keepout.parse_zones([_zone_poly(0.0, 0.0, 1.0, 1.0)])
    assert keepout.suspend(z, set(), (1.2, 0.5), fresh={"z1"}) == {"z1"}
    assert keepout.suspend(z, set(), (1.4, 0.5), fresh={"z1"}) == set()


def test_suspended_zones_leave_the_mask():
    zones = keepout.parse_zones([_zone_poly(0.0, 0.0, 1.0, 1.0, "a"),
                                 _zone_poly(3.0, 0.0, 4.0, 1.0, "b")])
    drawn = [z for z in zones if z["id"] not in {"a"}]
    assert all(x >= 2.9 for x, _ in _marked(keepout.mask([], zones=drawn)))


def test_the_bridge_serves_the_zones_route_under_the_cap():
    src = (ROOT / "service/slam/src/picar_bridge/picar_bridge/bridge.py").read_text()
    assert 'path == "/nav/zones"' in src
    assert "keepout.suspend" in src and '"nogo": self._nogo_view()' in src


# ---------- D3: explore (Z1 in miniature; the sweeps are tests/test_no_go_sweep.py) ----------

def _mission(house, zones, max_steps=60, start_inside=False):
    grid = build_world(house)
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    runner = MissionRunner(robot, target_object="red backpack", policy="explore", navigator=nav,
                           world=MockWorld(grid), max_steps=max_steps,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.set_nogo(FRAME, zones)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < 3000:
        ticks += 1
    return runner, nav, grid


def test_explore_sends_no_goal_into_a_zone():
    # A band across scaled_house's middle, clear of its start and target.
    zones = [nogo.clean_zone({"x0": 3.0, "y0": 2.0, "x1": 4.5, "y1": 3.0})]
    runner, nav, _ = _mission("scaled_house", zones)
    sets = [c for c in nav.calls if c[0] == "set"]
    assert sets
    assert not [c for c in sets if nogo.zone_at(zones, c[1], c[2], nogo.HALF_WIDTH_M)]
    assert runner.status()["explore"]["nogo"]["zones"] == 1


def test_a_robot_that_starts_inside_a_zone_is_sent_out_first():
    grid = build_world("scaled_house")
    x, y = grid.x * 0.3, grid.y * 0.3
    zones = [nogo.clean_zone({"x0": x - 0.4, "y0": y - 0.4, "x1": x + 0.4, "y1": y + 0.4})]
    runner, nav, _ = _mission("scaled_house", zones, max_steps=4)
    first = [c for c in nav.calls if c[0] == "set"][0]
    assert nogo.nearest_distance(zones, first[1], first[2]) >= nogo.LEAVE_GOAL_M
    events = [e for e in runner.agent.events if e["event"] == "goal_sent"]
    assert events[0]["kind"] == "leave_zone"


def test_no_zones_changes_nothing():
    """Z10 in miniature: with no zones, the goals are the ones dev sends."""
    a, nav_a, _ = _mission("scaled_house", [], max_steps=20)
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav_b = FakeNav(robot)
    b = MissionRunner(robot, target_object="red backpack", policy="explore", navigator=nav_b,
                      world=MockWorld(grid), max_steps=20, clock=lambda: grid.sim_time,
                      idle=robot.pass_time)
    b.start()
    ticks = 0
    while b.tick() and ticks < 3000:
        ticks += 1
    assert nav_a.calls == nav_b.calls
    assert "nogo" not in a.status()["explore"]


def test_coverage_counts_only_floor_outside_zones():
    from brain.mission_report import map_coverage
    m = {"usable": True, "width": 10, "height": 1, "resolution_m": 1.0, "origin_x_m": 0.0,
         "origin_y_m": 0.0, "cells": [CELL_FREE] * 10}
    assert map_coverage(m)["mapped_free_m2"] == 10.0
    z = nogo.clean_zone({"x0": 0.0, "y0": 0.0, "x1": 4.0, "y1": 1.0})
    assert map_coverage(m, [z])["mapped_free_m2"] == 6.0


def test_the_send_guard_moves_a_goal_just_out_or_refuses_it():
    zones = [nogo.clean_zone({"x0": 1.0, "y0": 1.0, "x1": 2.0, "y1": 2.0})]
    runner, _, _ = _mission("scaled_house", [], max_steps=1)
    agent = runner.agent
    agent.set_zones(zones)
    x, y = agent._out_of_zones(1.9, 1.5)
    assert x == pytest.approx(2.0 + nogo.HALF_WIDTH_M + 0.05) and y == 1.5
    assert agent._out_of_zones(5.0, 5.0) == (5.0, 5.0)
    agent.set_zones(zones + [nogo.clean_zone({"x0": 2.0, "y0": 0.0, "x1": 3.0, "y1": 3.0}),
                             nogo.clean_zone({"x0": 0.0, "y0": 0.0, "x1": 1.0, "y1": 3.0}),
                             nogo.clean_zone({"x0": 1.0, "y0": 0.0, "x1": 2.0, "y1": 1.0}),
                             nogo.clean_zone({"x0": 1.0, "y0": 2.0, "x1": 2.0, "y1": 3.0})])
    assert agent._out_of_zones(1.5, 1.5) is None


# ---------- the brain's routes ----------

class _World:
    def __init__(self, frame_id):
        self.frame_id = frame_id

    def get_pose(self):
        return {"usable": True, "x_m": 1.0, "y_m": 1.0, "heading_deg": 0.0,
                "map_id": "slam-x", "frame_id": self.frame_id}


@pytest.fixture
def brain(tmp_path, monkeypatch):
    monkeypatch.setenv("NOGO_DIR", str(tmp_path))
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(nogo_store, "_DEFAULT", None)
    from control import brain_server

    def make(frame_id=FRAME):
        return TestClient(brain_server.create_app(robot_factory=lambda: object(),
                                                  world_factory=lambda r: _World(frame_id)))
    return make


def test_the_brains_routes_store_on_the_saved_map(brain, tmp_path):
    c = brain()
    v = c.get("/mission/nogo").json()
    assert v["frame_id"] == FRAME and v["zones"] == [] and v["closed"] is None
    v = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 0,
                                      "zone": {"x0": 1, "y0": 1, "x1": 2, "y1": 2}}).json()
    v = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": v["rev"],
                                      "line": [[0, 3], [2, 3]]}).json()
    assert [z["kind"] for z in v["zones"]] == ["area", "wall"]
    assert json.loads((tmp_path / f"{FRAME}.json").read_text())["rev"] == 2
    r = c.post("/mission/nogo", json={"frame_id": "map-other", "op": "clear"})
    assert r.status_code == 409 and "reload" in r.json()["detail"]
    r = c.post("/mission/nogo", json={"frame_id": FRAME, "op": "add", "rev": 0,
                                      "zone": {"x0": 1, "y0": 1, "x1": 2, "y1": 2}})
    assert r.status_code == 409                                   # a stale rev


def test_no_saved_map_closes_the_editor(brain):
    c = brain(frame_id=None)
    v = c.get("/mission/nogo").json()
    assert v["frame_id"] is None and "Survey first" in v["closed"]
    assert c.post("/mission/nogo", json={"frame_id": None, "op": "clear"}).status_code == 409
