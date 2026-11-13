# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.156.E (D1, bar G2): the saved map's `dock_pose` is the dock
(`docs/plans/ros-alignment/3.156-return-to-dock.md`, section E).

The fact crosses three hands, each pinned here:
* the bridge (`picar_bridge/bridge.py`) adds the dock a matched reload
  localised at to its existing `/slam/pose` reply -- no new route -- and
  nothing on a fresh map, a fall-back or before the frame is decided;
* `world/ros_world.py` converts it through the session's anchor, as it
  converts the pose, and answers it as the pose's optional `dock` only when
  the anchor's frame is that saved map;
* `brain/dock.py`'s `dock_from_pose` takes it as `source: saved_map`; with
  none (a fresh map, a fall-back, the simulator's world) the start pose, as
  `dev` (`source: start`).
"""

import json
import math
import os

import httpx
import pytest

from brain.dock import SAVED_MAP, START, dock_from_pose
from tests.test_map_persist import _reload_bridge
from tests.test_review_376h1 import bridge_mod  # noqa: F401 -- the stubbed bridge
from tests.test_ros_world import FakeBridge, FakeTruth
from world.ros_world import RosWorld

from picar_bridge import relocalise  # noqa: E402 -- test_map_persist put it on the path

MATCH = relocalise.judge([0.9] * 5, (0.0, 0.0, 0.0))
MISMATCH = relocalise.judge([0.2] * 5, (0.0, 0.0, 0.0))


def _set_dock(store, dock):
    meta_path = os.path.join(os.path.dirname(store.current().prefix), "meta.json")
    with open(meta_path) as f:
        meta = json.load(f)
    meta["dock_pose"] = dock
    with open(meta_path, "w") as f:
        json.dump(meta, f)


# ---------- the bridge ----------

def test_a_matched_reload_answers_the_saved_dock(bridge_mod, monkeypatch, tmp_path):
    b, fake, store = _reload_bridge(bridge_mod, monkeypatch, tmp_path, MATCH)
    _set_dock(store, {"x": 1.5, "y": -0.5, "theta": 0.25})
    b.start_slam()
    assert fake.calls[0][2] == (1.5, -0.5, 0.25)        # localised there ...
    assert b.slam_pose()[1]["dock_pose"] == {"x": 1.5, "y": -0.5, "theta": 0.25}   # ... and said so


def test_the_default_dock_is_the_origin(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path, MATCH)
    b.start_slam()
    assert b.slam_pose()[1]["dock_pose"] == {"x": 0.0, "y": 0.0, "theta": 0.0}


def test_an_unreadable_dock_is_the_origin_it_localises_at(bridge_mod, monkeypatch, tmp_path):
    b, fake, store = _reload_bridge(bridge_mod, monkeypatch, tmp_path, MATCH)
    _set_dock(store, {"x": float("nan"), "y": True, "theta": 0.0})
    b.start_slam()
    assert fake.calls[0][2] == (0.0, 0.0, 0.0)
    assert b.slam_pose()[1]["dock_pose"] == {"x": 0.0, "y": 0.0, "theta": 0.0}


def test_a_fall_back_has_no_saved_dock(bridge_mod, monkeypatch, tmp_path):
    b, _, store = _reload_bridge(bridge_mod, monkeypatch, tmp_path, MISMATCH)
    _set_dock(store, {"x": 1.5, "y": -0.5, "theta": 0.25})
    b.start_slam()
    assert b.reload["result"] == "fell_back"
    assert b.slam_pose()[1]["dock_pose"] is None


def test_no_saved_map_has_no_dock(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path, None)
    monkeypatch.setattr(bridge_mod, "MAP_RELOAD", False)
    b.start_slam()
    assert b.slam_pose()[1]["dock_pose"] is None


def test_no_dock_until_the_frame_is_decided(bridge_mod, monkeypatch, tmp_path):
    b, _, _ = _reload_bridge(bridge_mod, monkeypatch, tmp_path, MATCH)
    b.start_slam()
    b.slam_state = "checking"            # e.g. a re-map's swap: no frame answered
    assert b.slam_pose()[1]["dock_pose"] is None


# ---------- the world ----------

class DockBridge(FakeBridge):
    """FakeBridge, answering a saved map's id and dock as the bridge does."""

    def __init__(self, map_id="map-0a1b2c3d", dock=None, **kw):
        super().__init__(**kw)
        self.map_id, self.dock = map_id, dock

    def handler(self, request):
        if request.url.path == "/slam/pose":
            body = {"session": self.session, "map": self.map_pose, "odom": self.odom_pose,
                    "start_truth": self.start_truth, "map_id": self.map_id}
            if self.dock is not None:
                body["dock_pose"] = self.dock
            return httpx.Response(200, json=body)
        return super().handler(request)


def _world(bridge, truth=None):
    client = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(bridge.handler))
    return RosWorld("http://bridge", truth=truth, client=client)


def test_the_car_answers_the_saved_dock_in_our_frame():
    """No truth (the car): SLAM's frame is the saved map's. ROS (1, 2, +90
    deg CCW) is ours (1, -2) facing compass 0 (north)."""
    pose = _world(DockBridge(dock={"x": 1.0, "y": 2.0, "theta": math.pi / 2})).get_pose()
    assert pose["frame_id"] == "map-0a1b2c3d"
    assert pose["dock"]["x_m"] == pytest.approx(1.0)
    assert pose["dock"]["y_m"] == pytest.approx(-2.0)
    assert pose["dock"]["heading_deg"] == pytest.approx(0.0)


def test_the_sim_lays_the_dock_through_the_same_anchor_as_the_pose():
    """The saved dock is the first session's start: with its start truth as
    the anchor, the origin dock lands on that truth, as the pose does."""
    truth = FakeTruth(0.75, 1.05, 90.0)
    start = {k: truth.pose[k] for k in ("x_m", "y_m", "heading_deg")}
    world = _world(DockBridge(dock={"x": 0.0, "y": 0.0, "theta": 0.0}, start_truth=start),
                   truth=truth)
    pose = world.get_pose()
    assert pose["dock"] == pytest.approx({"x_m": 0.75, "y_m": 1.05, "heading_deg": 90.0})
    assert pose["x_m"] == pytest.approx(0.75) and pose["y_m"] == pytest.approx(1.05)


def test_no_dock_pose_no_dock_key():
    pose = _world(DockBridge(dock=None)).get_pose()
    assert pose["usable"] and "dock" not in pose


def test_a_dock_on_no_saved_frame_is_dropped():
    """A first-contact anchor is not the saved map's frame (3.50.D5)."""
    truth = FakeTruth(0.75, 0.75, 90.0)
    pose = _world(DockBridge(dock={"x": 0.0, "y": 0.0, "theta": 0.0}), truth=truth).get_pose()
    assert pose["usable"] and pose["frame_id"] is None and "dock" not in pose


@pytest.mark.parametrize("dock", [{"x": 10 ** 400, "y": 0.0, "theta": 0.0},
                                  {"x": 0.0, "y": 0.0}, {"x": "1", "y": 0.0, "theta": 0.0},
                                  {"x": True, "y": 0.0, "theta": 0.0}, [0, 0, 0]])
def test_an_unreadable_dock_is_dropped(dock):
    pose = _world(DockBridge(dock=dock)).get_pose()
    assert pose["usable"] and "dock" not in pose


# ---------- the brain ----------

POSE = {"usable": True, "map_id": "slam-s1", "frame_id": None,
        "x_m": 2.0, "y_m": 3.0, "heading_deg": 45.0}


def test_no_dock_is_the_start_pose_exactly_as_dev():
    assert dock_from_pose(POSE) == {"x_m": 2.0, "y_m": 3.0, "theta": 45.0, "source": START}


def test_a_saved_dock_is_the_dock():
    pose = {**POSE, "frame_id": "map-0a1b2c3d",
            "dock": {"x_m": 0.5, "y_m": 0.25, "heading_deg": 90.0}}
    assert dock_from_pose(pose) == {"x_m": 0.5, "y_m": 0.25, "theta": 90.0, "source": SAVED_MAP}


@pytest.mark.parametrize("dock", [{"x_m": 0.5}, {"x_m": "a", "y_m": 0, "heading_deg": 0},
                                  {"x_m": float("nan"), "y_m": 0, "heading_deg": 0}, None, 3])
def test_an_unreadable_saved_dock_is_the_start_pose(dock):
    assert dock_from_pose({**POSE, "dock": dock})["source"] == START


def test_an_unusable_pose_is_no_dock_whatever_it_carries():
    assert dock_from_pose({"usable": False, "dock": {"x_m": 0, "y_m": 0, "heading_deg": 0}}) is None


def test_the_runner_records_the_saved_dock():
    """MissionRunner -> the world's pose at start -> status.dock."""
    from tests.test_dock_return import mission

    runner, nav, grid, robot = mission()
    real = runner.world.get_pose

    def with_dock():
        p = real()
        return {**p, "frame_id": "map-0a1b2c3d",
                "dock": {"x_m": 0.45, "y_m": 0.45, "heading_deg": 180.0}}

    runner.world.get_pose = with_dock
    runner.start()
    s = runner.status()
    assert s["dock"] == {"x_m": 0.45, "y_m": 0.45, "theta": 180.0, "source": SAVED_MAP}
    runner.stop("test")


def test_the_bridge_and_the_world_agree_across_the_wall(bridge_mod, monkeypatch, tmp_path):
    """The field's name and shape, end to end: the REAL bridge's /slam/pose
    reply (a matched reload, the car: no truth) read by `RosWorld`. A rename
    on either side of the wall fails here, not on the car."""
    b, _, store = _reload_bridge(bridge_mod, monkeypatch, tmp_path, MATCH)
    _set_dock(store, {"x": 1.0, "y": 2.0, "theta": math.pi / 2})
    b.start_slam()
    monkeypatch.setattr(b, "_pose_in", lambda frame: {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0,
                                                      "age_s": 0.0})
    reply = b.slam_pose()[1]

    def handler(request):
        return httpx.Response(200, json=reply)
    client = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(handler))
    pose = RosWorld("http://bridge", client=client).get_pose()
    assert pose["frame_id"] == "map-0a1b2c3d"
    assert pose["dock"] == pytest.approx({"x_m": 1.0, "y_m": -2.0, "heading_deg": 0.0})
    assert dock_from_pose(pose)["source"] == SAVED_MAP
