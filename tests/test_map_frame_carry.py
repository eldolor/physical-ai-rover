"""
tests/test_map_frame_carry.py

3.50.D5's code half (docs/plans/ros-alignment/3.50-map-persistence.md,
"3.50.D5: sightings carry over"): the saved map's id reaches the pose as
`frame_id` (D5-b), an inventory is seeded from the last one saved on the
same frame (D5-c, D5-d), and a landmark from session 1 is in session 2's
inventory after a matched reload but not after a fall-back (D5-e).

`map_id` itself stays per session (`slam-<session>`): the map's ETag, the
keepout spots, the room labeller's cache and explore's trail rely on it
changing with the session.
"""
import httpx
import pytest

from tests.test_ros_world import FakeBridge, FakeTruth
from world.ros_world import RosWorld

SAVED = "map-0a1b2c3d"


class SavedMapBridge(FakeBridge):
    """FakeBridge plus 3.50.D5's `map_id` on /slam/pose (None: an older
    bridge, which sends no key at all)."""

    def __init__(self, saved_map=SAVED, **kw):
        super().__init__(**kw)
        self.saved_map = saved_map

    def handler(self, request):
        r = super().handler(request)
        if request.url.path == "/slam/pose" and self.saved_map is not None:
            body = r.json()
            body["map_id"] = self.saved_map
            return httpx.Response(200, json=body)
        return r


def world_on(bridge, truth=None):
    client = httpx.Client(base_url="http://bridge", transport=httpx.MockTransport(bridge.handler))
    return RosWorld("http://bridge", truth=truth, client=client)


START = {"x_m": 1.0, "y_m": 2.0, "heading_deg": 90.0}


# ---------- D5-b: the pose names the saved map's frame ----------

def test_a_start_truth_anchor_is_the_saved_frame():
    world = world_on(SavedMapBridge(start_truth=START), truth=FakeTruth(1.0, 2.0, 90.0))
    pose = world.get_pose()
    assert pose["usable"] and pose["frame_id"] == SAVED
    assert pose["map_id"] == "slam-s1"                     # map_id stays per session
    assert world.get_odom_pose()["frame_id"] == SAVED


def test_no_truth_at_all_is_the_saved_frame():
    """The car: SLAM's frame is the saved map's own."""
    pose = world_on(SavedMapBridge()).get_pose()
    assert pose["usable"] and pose["frame_id"] == SAVED


def test_a_first_contact_anchor_is_not_the_saved_frame():
    """A bridge that gave up on its start truth: the frame is laid where the
    robot was first seen, not where the saved map began."""
    world = world_on(SavedMapBridge(start_truth=None), truth=FakeTruth(1.0, 2.0, 90.0))
    pose = world.get_pose()
    assert pose["usable"] and pose["frame_id"] is None


def test_an_older_bridge_names_no_frame():
    world = world_on(SavedMapBridge(saved_map=None, start_truth=START),
                     truth=FakeTruth(1.0, 2.0, 90.0))
    pose = world.get_pose()
    assert pose["usable"] and pose["frame_id"] is None


def test_an_unusable_pose_names_no_frame():
    bridge = SavedMapBridge(start_truth=START)
    bridge.map_pose = None                       # nothing published yet
    pose = world_on(bridge, truth=FakeTruth(1.0, 2.0, 90.0)).get_pose()
    assert pose["usable"] is False and pose["frame_id"] is None


def test_a_new_session_on_the_same_saved_map_keeps_the_frame():
    """A container restart that reloaded the same map: a new session (a new
    map_id) in the same frame."""
    bridge = SavedMapBridge(start_truth=START)
    world = world_on(bridge, truth=FakeTruth(1.0, 2.0, 90.0))
    first = world.get_pose()
    bridge.session = "s2"
    second = world.get_pose()
    assert (first["map_id"], second["map_id"]) == ("slam-s1", "slam-s2")
    assert first["frame_id"] == second["frame_id"] == SAVED


# ---------- D5-c: an inventory seeded from the last one on the same map ----------

from brain.inventory import MERGE_M, Inventory  # noqa: E402
from tests.test_inventory import _scan  # noqa: E402


def _pose(x=0.0, y=0.0, h=0.0, map_id="slam-s2", frame_id=SAVED):
    return {"usable": True, "x_m": x, "y_m": y, "heading_deg": h,
            "map_id": map_id, "frame_id": frame_id}


def _session_one():
    """A stool seen from two places (reported) and a box once (a candidate),
    on the saved map, in session 1."""
    inv = Inventory()
    # Heading 0 is -y: the stool at (0, -2), from the origin and from the east.
    inv.observe([{"label": "stool", "bearing_deg": 0.0}], _pose(map_id="slam-s1"), _scan({0: 2.0}))
    inv.observe([{"label": "stool", "bearing_deg": 0.0}],
                _pose(x=2.0, y=-2.0, h=270.0, map_id="slam-s1"), _scan({0: 2.0}))
    inv.observe([{"label": "box", "bearing_deg": 0.0}], _pose(h=90.0, map_id="slam-s1"),
                _scan({0: 3.0}))
    r = inv.report()
    r["mission"] = "m1"
    return r


def test_a_seeded_inventory_holds_the_saved_landmarks():
    saved = _session_one()
    assert saved["frame_id"] == SAVED and saved["reported"] and saved["candidates"]
    inv = Inventory()
    assert inv.seed(SAVED, saved) == len(saved["reported"]) + len(saved["candidates"])
    back = {d["label"]: d for d in inv.report()["reported"] + inv.report()["candidates"]}
    for d in saved["reported"] + saved["candidates"]:
        b = back[d["label"]]
        assert abs(b["x_m"] - d["x_m"]) <= 0.001 and abs(b["y_m"] - d["y_m"]) <= 0.001
        assert abs(b["belief"] - d["belief"]) <= 0.001
        assert b["votes"] == d["votes"] and b["carried"] is True
    assert len({lm.id for lm in inv.landmarks}) == len(inv.landmarks)
    r = inv.report()
    assert r["frame_id"] == SAVED and r["carried_from"] == "m1" and r["counts"]["carried"] == 2


def test_a_new_sighting_joins_the_carried_landmark():
    saved = _session_one()
    inv = Inventory()
    inv.seed(SAVED, saved)
    stool = next(d for d in saved["reported"] if d["label"] == "stool")
    # Seen again in session 2 from the far side, within MERGE_M of it.
    inv.observe([{"label": "stool", "bearing_deg": 0.0}], _pose(x=stool["x_m"] - 1.0,
                y=stool["y_m"], h=90.0), _scan({0: 1.0 + MERGE_M / 2}))
    stools = [lm for lm in inv.landmarks if lm.label == "stool"]
    assert len(stools) == 1 and stools[0].carried and len(stools[0].points) == 2
    assert stools[0].hits == sum(stool["votes"].values()) + 1


@pytest.mark.parametrize("frame, report", [
    (None, "saved"), ("map-ffffffff", "saved"), (SAVED, None), (SAVED, "other"),
])
def test_another_frame_or_none_carries_nothing(frame, report):
    saved = _session_one()
    if report == "other":
        saved = dict(saved, frame_id="map-ffffffff")
    inv = Inventory()
    assert inv.seed(frame, saved if report else None) == 0
    assert inv.landmarks == [] and inv.report()["counts"]["carried"] == 0


def test_nothing_is_carried_once_recording_began():
    inv = Inventory()
    inv.observe([{"label": "cup", "bearing_deg": 0.0}], _pose(), _scan({0: 1.0}))
    assert inv.seed(SAVED, _session_one()) == 0
    assert [lm.label for lm in inv.landmarks] == ["cup"]


def test_a_seeded_inventory_refuses_another_saved_maps_poses():
    inv = Inventory()
    inv.seed(SAVED, _session_one())
    n = len(inv.landmarks)
    inv.observe([{"label": "cup", "bearing_deg": 0.0}], _pose(frame_id="map-ffffffff"),
                _scan({0: 1.0}))
    assert len(inv.landmarks) == n and inv.no_pose == 1


def test_three_missions_in_a_chain_carry_each_landmark_once():
    first = _session_one()
    second = Inventory()
    second.seed(SAVED, first)
    r2 = dict(second.report(), mission="m2")
    third = Inventory()
    third.seed(SAVED, r2)
    labels = sorted(lm.label for lm in third.landmarks)
    assert labels == sorted(d["label"] for d in first["reported"] + first["candidates"])
    assert third.report()["carried_from"] == "m2"


def test_a_carried_report_with_bad_rows_keeps_the_good_ones():
    saved = _session_one()
    saved["candidates"] = saved["candidates"] + [
        {"label": "", "x_m": 1, "y_m": 1, "belief": 0.5},
        {"label": "mug", "x_m": "nan", "y_m": 1, "belief": 0.5},
        {"label": "mug", "y_m": 1, "belief": 0.5},
        {"label": "lamp", "x_m": 1.0, "y_m": 1.0, "belief": 1.0, "votes": {"lamp": "x"}}]
    inv = Inventory()
    assert inv.seed(SAVED, saved) == 3
    lamp = next(lm for lm in inv.landmarks if lm.label == "lamp")
    assert 0.0 < lamp.belief < 1.0 and lamp.hits == 0


# ---------- D5-d: the store's newest report on a frame ----------

import json  # noqa: E402
import os  # noqa: E402
import time  # noqa: E402

from control.inventory_store import InventoryStore  # noqa: E402


def _write(d, name, report, age_s=0.0):
    p = d / name
    p.write_text(json.dumps(report) if not isinstance(report, str) else report)
    t = time.time() - age_s
    os.utime(p, (t, t))


def test_the_store_returns_the_newest_report_on_the_frame(tmp_path):
    _write(tmp_path, "a.json", {"frame_id": SAVED, "mission": "old"}, age_s=300)
    _write(tmp_path, "b.json", {"frame_id": SAVED, "mission": "new"}, age_s=100)
    _write(tmp_path, "c.json", {"frame_id": "map-ffffffff", "mission": "other"}, age_s=10)
    _write(tmp_path, "d.json", {"map_id": "slam-x", "mission": "no frame"}, age_s=5)
    _write(tmp_path, "e.json", "{not json", age_s=1)
    _write(tmp_path, "f.json", "[1, 2]", age_s=1)
    _write(tmp_path, ".g.json.tmp", {"frame_id": SAVED, "mission": "half"}, age_s=0)
    store = InventoryStore(str(tmp_path))
    assert store.latest_for(SAVED)["mission"] == "new"
    assert store.latest_for("map-ffffffff")["mission"] == "other"
    assert store.latest_for("map-00000000") is None
    assert store.latest_for(None) is None and store.latest_for("") is None


def test_the_store_with_no_directory_has_nothing(tmp_path):
    assert InventoryStore(str(tmp_path / "missing")).latest_for(SAVED) is None


def test_a_saved_report_is_found_again(tmp_path):
    store = InventoryStore(str(tmp_path))
    store.save("m1", _session_one())
    assert store.latest_for(SAVED)["mission"] == "m1"


# ---------- D5-e: the round trip through MissionRunner (criterion 7, offline) ----------

import math  # noqa: E402

from control.mission_runner import MissionRunner  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from tests.conftest import mock_world_for  # noqa: E402


class _FramedWorld:
    """The sim's world, its poses on a saved map's frame: what a RosWorld
    answers on a matched reload (a new session's map_id, the same frame)."""

    def __init__(self, inner, frame_id, session):
        self._inner, self._frame, self._session = inner, frame_id, session

    def get_pose(self):
        p = dict(self._inner.get_pose())
        if p.get("usable"):
            p["map_id"], p["frame_id"] = f"slam-{self._session}", self._frame
        return p

    def __getattr__(self, name):
        return getattr(self._inner, name)


def _session(store, frame_id, session, house="complex_house", steps=40):
    grid = build_world(house)
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=steps, policy="frontier",
                           world=_FramedWorld(mock_world_for(robot), frame_id, session))
    runner.mission_id = f"mission-{session}"
    runner.inventory_prior = store.latest_for
    runner.inventory_sink = lambda report: store.save(runner.mission_id, report)
    runner.start()
    while runner.tick():
        pass
    return runner


def test_a_landmark_from_session_one_is_in_session_two_on_the_same_map(tmp_path):
    store = InventoryStore(str(tmp_path))
    one = _session(store, SAVED, "s1")
    saved = store.latest_for(SAVED)
    first = saved["reported"] + saved["candidates"]
    assert first, "session 1 recorded nothing"
    two = _session(store, SAVED, "s2")
    carried = [lm for lm in two.inventory.landmarks if lm.carried]
    assert len(carried) == len(first)
    assert two.inventory.report()["carried_from"] == "mission-s1"
    for d in first:
        mine = [lm for lm in carried if lm.points[0][2] == d["label"]
                and math.hypot(lm.points[0][0] - d["x_m"], lm.points[0][1] - d["y_m"]) <= 0.001]
        assert mine, d                           # seeded where session 1 saved it
        assert math.hypot(mine[0].x - d["x_m"], mine[0].y - d["y_m"]) <= 0.15   # the live bar
    assert one.inventory.report()["carried_from"] is None


def test_after_a_fall_back_no_landmark_of_session_one_is_used(tmp_path):
    store = InventoryStore(str(tmp_path))
    _session(store, SAVED, "s1")
    assert store.latest_for(SAVED)
    two = _session(store, "map-ffffffff", "s2")     # a fall-back: a new saved-map id
    assert not any(lm.carried for lm in two.inventory.landmarks)
    assert two.inventory.report()["counts"]["carried"] == 0


def test_without_a_saved_frame_nothing_is_read_or_carried(tmp_path):
    """The sim's own world (frame_id None): no lookup, nothing carried."""
    store = InventoryStore(str(tmp_path))
    _session(store, SAVED, "s1")
    asked = []
    grid = build_world("complex_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=10, policy="frontier",
                           world=mock_world_for(robot))
    runner.inventory_prior = lambda f: asked.append(f) or store.latest_for(f)
    runner.start()
    while runner.tick():
        pass
    assert asked == [] and not any(lm.carried for lm in runner.inventory.landmarks)
