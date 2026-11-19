# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_saved_map_mission.py

3.70 M1 (docs/plans/ros-alignment/3.70-survey-and-mission-ends.md): a
mission runs on the saved map when the bridge reloaded it, or on a new map
(a first visit, or a fall-back) under a new id -- and says which, in
`status()['saved_map']` and its log. In both cases the map it reads is the
bridge's own, built from the robot's scans: the brain adds no cell.

Against a fake bridge answering `/slam/pose` as picar_bridge does (3.50.D5's
`map_id`, 3.156.E's `dock_pose` only on a matched reload -- the bridge's side
of that is pinned in tests/test_dock_saved.py), through `RosWorld` and
`MissionRunner`.
"""
import httpx
import pytest

from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests.test_dock_saved import DockBridge, _world
from tests.test_ros_world import FakeTruth

SAVED = "map-0a1b2c3d"
NEW = "map-77777777"
TRUTH = FakeTruth(1.05, 1.05, 90.0)
START = {k: TRUTH.pose[k] for k in ("x_m", "y_m", "heading_deg")}


def _mission(bridge, truth=TRUTH, ticks=6, inventory_prior=True):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    world = _world(bridge, truth=truth)
    runner = MissionRunner(robot, target_object="unicorn", policy="frontier", max_steps=50,
                           world=world)
    if inventory_prior:
        runner.inventory_prior = lambda frame_id: None
    runner.start()
    for _ in range(ticks):
        if not runner.tick():
            break
    return runner, world


def _known(cells):
    return sum(c != -1 for c in cells)


# ---------- the two cases ----------

def test_with_a_saved_map_the_mission_runs_on_it_reloaded():
    """A matched reload: the saved map's id and its dock on the pose."""
    runner, _ = _mission(DockBridge(map_id=SAVED, dock={"x": 0.0, "y": 0.0, "theta": 0.0},
                                    start_truth=START))
    s = runner.status()
    assert s["saved_map"] == {"frame_id": SAVED, "map_id": "slam-s1", "reloaded": True}
    assert runner.inventory.frame_id == SAVED          # what it records, it records on that map
    assert any(f"on the saved map {SAVED}, reloaded" in line for line in runner._log)
    runner.stop("test")


def test_with_none_the_mission_starts_on_a_new_map_under_a_new_id():
    """No saved map: a new id, no dock."""
    runner, _ = _mission(DockBridge(map_id=NEW, dock=None, start_truth=START))
    s = runner.status()
    assert s["saved_map"] == {"frame_id": NEW, "map_id": "slam-s1", "reloaded": False}
    assert any(f"a new map {NEW}" in line for line in runner._log)
    runner.stop("test")


class Judged(DockBridge):
    """The bridge while it judges a reload (no map pose, no map_id, no
    dock), then its verdict."""

    def __init__(self, checking_polls, **kw):
        super().__init__(**kw)
        self.left = checking_polls

    def handler(self, request):
        r = super().handler(request)
        if request.url.path == "/slam/pose" and self.left > 0:
            self.left -= 1
            body = dict(r.json(), map=None, map_id=None)
            body.pop("dock_pose", None)
            return httpx.Response(200, json=body)
        return r


def test_a_fall_back_after_the_check_is_a_new_map_not_a_reload(monkeypatch):
    from control import mission_runner as mr
    monkeypatch.setattr(mr, "SEED_EVERY_S", 0.0)
    runner, _ = _mission(Judged(3, map_id=NEW, dock=None, start_truth=START), ticks=10)
    assert runner.status()["saved_map"] == {"frame_id": NEW, "map_id": "slam-s1",
                                            "reloaded": False}
    runner.stop("test")


def test_a_matched_reload_after_the_check_is_a_reload(monkeypatch):
    from control import mission_runner as mr
    monkeypatch.setattr(mr, "SEED_EVERY_S", 0.0)
    runner, _ = _mission(Judged(3, map_id=SAVED, dock={"x": 0.0, "y": 0.0, "theta": 0.0},
                                start_truth=START), ticks=10)
    assert runner.status()["saved_map"]["reloaded"] is True
    runner.stop("test")


def test_a_new_session_on_the_same_saved_map_keeps_its_id():
    """A container restart onto the same saved map: the session (and so
    `map_id`) changes, the saved map's id does not."""
    one, _ = _mission(DockBridge(map_id=SAVED, dock={"x": 0.0, "y": 0.0, "theta": 0.0},
                                 start_truth=START, session="s1"))
    two, _ = _mission(DockBridge(map_id=SAVED, dock={"x": 0.0, "y": 0.0, "theta": 0.0},
                                 start_truth=START, session="s2"))
    assert one.status()["saved_map"]["frame_id"] == two.status()["saved_map"]["frame_id"] == SAVED
    assert one.status()["saved_map"]["map_id"] != two.status()["saved_map"]["map_id"]
    one.stop("test")
    two.stop("test")


# ---------- 3.50 criterion 12: the map's cells are the bridge's ----------

@pytest.mark.parametrize("dock", [{"x": 0.0, "y": 0.0, "theta": 0.0}, None])
def test_the_mission_reads_only_the_bridges_cells(dock):
    """The brain seeds no cell: the map the mission reads holds exactly the
    cells the bridge's scans made, saved map or new."""
    bridge = DockBridge(map_id=SAVED if dock else NEW, dock=dock, start_truth=START)
    runner, world = _mission(bridge)
    m = world.get_map()
    assert m["usable"]
    assert _known(m["cells"]) == _known(bridge.grid)
    assert len(m["cells"]) == len(bridge.grid)
    runner.stop("test")


# ---------- the edges ----------

def test_an_older_bridge_names_no_saved_map():
    runner, _ = _mission(DockBridge(map_id=None, dock=None, start_truth=START))
    assert runner.status()["saved_map"] == {"frame_id": None, "map_id": "slam-s1",
                                            "reloaded": False}
    assert any("not on a saved map's frame" in line for line in runner._log)
    runner.stop("test")


def test_a_dock_on_no_saved_frame_is_not_a_reload():
    """A first-contact anchor is not the saved map's frame: RosWorld drops
    the dock, and the mission is not on a reloaded map."""
    runner, _ = _mission(DockBridge(map_id=SAVED, dock={"x": 0.0, "y": 0.0, "theta": 0.0},
                                    start_truth=None))
    assert runner.status()["saved_map"]["reloaded"] is False
    assert runner.status()["saved_map"]["frame_id"] is None
    runner.stop("test")


def test_before_a_usable_pose_nothing_is_named():
    runner, _ = _mission(Judged(10 ** 6, map_id=SAVED, start_truth=START), ticks=2)
    assert runner.status()["saved_map"] is None
    runner.stop("test")


def _pose_reads(elapsed_s, monkeypatch, ticks=4):
    """/slam/pose reads over `ticks` ticks of a mission at `elapsed_s`, on
    frames with no detections, the pose never usable."""
    from control import mission_runner as mr
    monkeypatch.setattr(mr, "SEED_EVERY_S", 0.0)
    bridge = Judged(10 ** 6, map_id=SAVED, start_truth=START)
    reads = []
    real = bridge.handler
    bridge.handler = lambda req: (reads.append(1) if req.url.path == "/slam/pose"
                                  else None) or real(req)
    grid = build_world("scaled_house")
    clock = [0.0]
    runner = MissionRunner(MockRobot(grid, render=False), target_object="unicorn",
                           policy="frontier", max_steps=50, world=_world(bridge, truth=TRUTH),
                           clock=lambda: clock[0], detections_fn=lambda frame: None)
    runner.start()
    runner.tick()
    clock[0] = elapsed_s
    before = len(reads)
    for _ in range(ticks):
        runner.tick()
    runner.stop("test")
    return len(reads) - before


def test_reads_only_to_name_the_map_stop_after_the_give_up_time(monkeypatch):
    from control import mission_runner as mr
    # The tick reads the pose for its own needs either way; the note's own
    # read (one a tick here, SEED_EVERY_S at 0) is there only before it.
    early, late = _pose_reads(1.0, monkeypatch), _pose_reads(mr.NOTE_GIVE_UP_S + 1, monkeypatch)
    assert early - late == 4, (early, late)


def test_the_map_is_named_without_an_inventory_store():
    """A harness with no store (no `inventory_prior`) still names its map."""
    runner, _ = _mission(DockBridge(map_id=SAVED, dock={"x": 0.0, "y": 0.0, "theta": 0.0},
                                    start_truth=START), inventory_prior=False)
    assert runner.status()["saved_map"]["frame_id"] == SAVED
    runner.stop("test")
