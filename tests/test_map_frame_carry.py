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


# ---------- review round 1: the chain cannot be cut ----------

from control import inventory_store as store_mod  # noqa: E402
from control import mission_runner as runner_mod  # noqa: E402
from brain.inventory import CARRY_WEIGHT, carry_seq_of  # noqa: E402


def test_the_head_is_chosen_by_lineage_not_by_the_clock(tmp_path):
    """A clock set back at boot: the newest report has the older mtime."""
    _write(tmp_path, "head.json", {"frame_id": SAVED, "mission": "head", "carry_seq": 3},
           age_s=10_000)
    _write(tmp_path, "older.json", {"frame_id": SAVED, "mission": "older", "carry_seq": 2},
           age_s=0)
    _write(tmp_path, "pre.json", {"frame_id": SAVED, "mission": "no seq"}, age_s=0)
    assert InventoryStore(str(tmp_path)).latest_for(SAVED)["mission"] == "head"


def test_a_save_in_flight_is_waited_for(tmp_path, monkeypatch):
    """A mission started while the last one's report is still being written
    carries that report, not the one before it."""
    store = InventoryStore(str(tmp_path))
    store.save("m0", dict(_session_one(), mission_id="m0"))
    import threading
    gate = threading.Event()
    real = store_mod._fsync
    monkeypatch.setattr(store_mod, "_fsync", lambda fd: (gate.wait(5), real(fd)))
    newer = dict(_session_one(), mission_id="m1", carry_seq=1)
    t = store.save_async("m1", newer)
    threading.Timer(0.3, gate.set).start()
    assert store.latest_for(SAVED)["mission_id"] == "m1"
    t.join(5)
    assert not store_mod._PENDING


def test_a_directory_that_cannot_be_listed_is_a_failure_not_nothing(tmp_path, monkeypatch):
    store = InventoryStore(str(tmp_path))
    store.save("m0", _session_one())
    def denied(path):
        raise PermissionError("denied")
    # os.listdir, as the store lists: Path.glob would have swallowed this
    # and read as "nothing saved" (review 5).
    monkeypatch.setattr(store_mod.os, "listdir", denied)
    with pytest.raises(PermissionError):
        store.latest_for(SAVED)


def test_a_failed_lookup_detaches_and_the_chain_survives(tmp_path):
    store = InventoryStore(str(tmp_path))
    _session(store, SAVED, "s1")
    first = store.latest_for(SAVED)

    def broken(frame_id):
        raise OSError("disk")
    grid = build_world("complex_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=40, policy="frontier",
                           world=_FramedWorld(mock_world_for(robot), SAVED, "s2"))
    runner.mission_id = "mission-s2"
    runner.inventory_prior = broken
    runner.inventory_sink = lambda report: store.save(runner.mission_id, report)
    runner.start()
    while runner.tick():
        pass
    saved_two = json.loads((tmp_path / "mission-s2.json").read_text())
    assert saved_two["frame_id"] is None and runner.inventory.frames > 0
    assert store.latest_for(SAVED)["mission_id"] == first["mission_id"]   # still the head
    three = _session(store, SAVED, "s3")
    assert sum(lm.carried for lm in three.inventory.landmarks) == (
        len([d for d in first["reported"] + first["candidates"] if d["belief"] >= 0.5]))


def test_a_landmark_with_more_evidence_of_absence_is_not_carried():
    saved = _session_one()
    saved["candidates"] = saved["candidates"] + [
        {"label": "sock", "x_m": 1.0, "y_m": 1.0, "belief": 0.3, "votes": {"sock": 1}}]
    inv = Inventory()
    inv.seed(SAVED, saved)
    assert "sock" not in [lm.label for lm in inv.landmarks]


def test_carried_votes_keep_their_shares_bounded_and_a_tie_keeps_the_name():
    """Review 4: weights scaled to CARRY_WEIGHT, never rounded -- shares and
    `disputed` hold exactly, a corrupt count costs nothing, names normalised."""
    saved = _session_one()
    saved["candidates"] = saved["candidates"] + [
        {"label": "shoe", "x_m": 1.0, "y_m": 1.0, "belief": 0.9,
         "votes": {"boot": 2, "shoe": 2}},
        {"label": "chair", "x_m": 3.0, "y_m": 3.0, "belief": 0.9,
         "votes": {"chair": 10**9, "stool": 1, "bad": "x", "neg": -3}},
        {"label": "table", "x_m": 5.0, "y_m": 5.0, "belief": 0.9,
         "votes": {"table": 41, "Desk ": 20}}]
    inv = Inventory()
    inv.seed(SAVED, saved)
    shoe = next(lm for lm in inv.landmarks if lm.points[0][2] == "shoe")
    assert shoe.label == "shoe" and shoe.disputed
    chair = next(lm for lm in inv.landmarks if lm.label == "chair")
    assert sum(chair.votes.values()) == pytest.approx(CARRY_WEIGHT) and chair.hits == 20
    table = next(lm for lm in inv.landmarks if lm.label == "table")
    assert set(table.votes) == {"table", "desk"}
    assert table.votes["desk"] / sum(table.votes.values()) == pytest.approx(20 / 61)
    assert not table.disputed                       # 20/61 < 1/3, exactly as saved


def test_two_carried_parts_merged_weigh_no_more_than_the_heavier():
    saved = _session_one()
    saved["candidates"] = [
        {"label": "sofa", "x_m": 2.0, "y_m": 2.0, "belief": 0.9, "votes": {"sofa": 5}},
        {"label": "sofa", "x_m": 2.0 + 2 * MERGE_M * 0.9, "y_m": 2.0, "belief": 0.9,
         "votes": {"sofa": 5}}]
    saved["reported"] = []
    inv = Inventory()
    inv.seed(SAVED, saved)
    # A point between them bridges the two (each within MERGE_M of it).
    inv.observe([{"label": "sofa", "bearing_deg": 0.0}], _pose(x=2.0 + MERGE_M * 0.9, y=3.0,
                h=0.0), _scan({0: 1.0}))
    sofas = [lm for lm in inv.landmarks if lm.label == "sofa"]
    assert len(sofas) == 1
    assert sum(sofas[0]._carried_votes.values()) == pytest.approx(5)


def test_a_carried_landmark_seen_again_counts_both():
    saved = _session_one()
    inv = Inventory()
    inv.seed(SAVED, saved)
    stool = next(d for d in saved["reported"] if d["label"] == "stool")
    inv.observe([{"label": "stool", "bearing_deg": 0.0}], _pose(x=stool["x_m"] - 1.0,
                y=stool["y_m"], h=90.0), _scan({0: 1.0}))
    lm = next(lm for lm in inv.landmarks if lm.label == "stool")
    assert lm.votes["stool"] == stool["votes"]["stool"] + 1
    again = dict(inv.report(), mission_id="m2")
    nxt = Inventory()
    nxt.seed(SAVED, again)
    assert next(lm for lm in nxt.landmarks if lm.label == "stool").hits == lm.hits


def test_the_report_carries_its_place_in_the_chain():
    inv = Inventory()
    inv.seed(SAVED, dict(_session_one(), carry_seq=4))
    assert inv.report()["carry_seq"] == 5
    assert Inventory().report()["carry_seq"] == 0


class _HeldPose:
    """A world whose pose is unusable for its first `held` reads (a reload's
    hold, or a phone walk with None), then the inner, framed pose."""

    def __init__(self, inner, held=None):
        self._inner, self.held, self.reads = inner, held, 0

    def get_pose(self):
        self.reads += 1
        if self.held is None or self.reads <= self.held:
            return {"usable": False, "map_id": None, "frame_id": None,
                    "x_m": None, "y_m": None, "heading_deg": None}
        return self._inner.get_pose()

    def __getattr__(self, name):
        return getattr(self._inner, name)


def test_no_usable_pose_costs_a_read_every_few_seconds_not_every_frame():
    """A phone walk never has a usable pose (review 2: counted in time)."""
    grid = build_world("complex_house")
    robot = MockRobot(grid, render=False)
    world = _HeldPose(mock_world_for(robot))
    runner = MissionRunner(robot, target_object="unicorn", max_steps=80, policy="frontier",
                           world=world, detections_fn=lambda frame: None)
    runner.inventory_prior = lambda f: None
    seeds = []
    real = runner._seed_inventory
    runner._seed_inventory = lambda pose: (seeds.append(pose), real(pose))
    runner.start()
    while runner.tick():
        pass
    assert not runner._inventory_seeded
    assert 1 <= len(seeds) < runner._ticks, ("a pose read on every frame", len(seeds))


def test_a_long_hold_then_a_usable_pose_still_carries(tmp_path):
    """Review 2: giving up after N frames let a held mission take the frame
    at carry_seq 0 and cut the chain; now it seeds when the pose comes."""
    store = InventoryStore(str(tmp_path))
    _session(store, SAVED, "s1")
    first = store.latest_for(SAVED)
    grid = build_world("complex_house")
    robot = MockRobot(grid, render=False)
    world = _HeldPose(_FramedWorld(mock_world_for(robot), SAVED, "s2"), held=40)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=80, policy="frontier",
                           world=world)
    runner.mission_id = "mission-s2"
    runner.inventory_prior = store.latest_for
    runner.inventory_sink = lambda report: store.save(runner.mission_id, report)
    runner.start()
    while runner.tick():
        pass
    assert world.reads > 40 and runner._inventory_seeded
    assert runner.inventory.report()["carry_seq"] == carry_seq_of(first) + 1
    assert store.latest_for(SAVED)["mission_id"] == "mission-s2"
    assert any(lm.carried for lm in runner.inventory.landmarks)


def test_a_wait_that_runs_out_is_a_failed_lookup(tmp_path, monkeypatch):
    store = InventoryStore(str(tmp_path))
    key = str(tmp_path.resolve())
    monkeypatch.setattr(store_mod, "PENDING_WAIT_S", 0.05)
    with store_mod._PENDING_COND:
        store_mod._PENDING[key] = 1
    try:
        with pytest.raises(TimeoutError):
            store.latest_for(SAVED)
    finally:
        with store_mod._PENDING_COND:
            store_mod._PENDING.pop(key, None)


def test_a_stray_file_is_skipped_not_fatal(tmp_path):
    """Not UTF-8, or a directory: not one of our reports."""
    _write(tmp_path, "good.json", {"frame_id": SAVED, "mission": "good"}, age_s=10)
    (tmp_path / "bad.json").write_bytes(b"\xff\xfe" + SAVED.encode() + b"\x80")
    (tmp_path / "dir.json").mkdir()
    assert InventoryStore(str(tmp_path)).latest_for(SAVED)["mission"] == "good"


def test_a_report_that_cannot_be_read_is_skipped_and_named(tmp_path, monkeypatch, caplog):
    """Review 4: the lesser harm -- skipping it can fork the chain (losing that
    mission's own sightings); failing would stop all carrying until someone
    removed the file."""
    _write(tmp_path, "good.json", {"frame_id": SAVED, "mission": "good"}, age_s=10)
    _write(tmp_path, "head.json", {"frame_id": SAVED, "mission": "head", "carry_seq": 9})
    real = type(tmp_path).read_text

    def read_text(self, *a, **kw):
        if self.name == "head.json":
            raise PermissionError("denied")
        return real(self, *a, **kw)
    monkeypatch.setattr(type(tmp_path), "read_text", read_text)
    assert InventoryStore(str(tmp_path)).latest_for(SAVED)["mission"] == "good"
    assert "head.json" in caplog.text


def test_a_held_pose_with_no_detections_still_seeds_when_it_comes(tmp_path, monkeypatch):
    """Review 3: the read limit must not stall seeding once the pose is usable."""
    monkeypatch.setattr(runner_mod, "SEED_EVERY_S", 0.0)
    store = InventoryStore(str(tmp_path))
    store.save("m1", dict(_session_one(), mission_id="m1"))
    grid = build_world("complex_house")
    robot = MockRobot(grid, render=False)
    world = _HeldPose(_FramedWorld(mock_world_for(robot), SAVED, "s2"), held=5)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=20, policy="frontier",
                           world=world, detections_fn=lambda frame: None)
    runner.inventory_prior = store.latest_for
    runner.start()
    while runner.tick():
        pass
    assert runner._inventory_seeded and runner.inventory.carried == 2


@pytest.mark.parametrize("value, seq", [
    (3, 3), (3.0, 3), (2.5, 0), ("3", 0), (True, 0), (-4, 0), (None, 0), (float("inf"), 0)])
def test_one_carry_seq_rule(value, seq):
    assert carry_seq_of({"carry_seq": value}) == seq


def test_detach_clears_a_frame_already_taken():
    inv = Inventory()
    inv.observe([{"label": "cup", "bearing_deg": 0.0}], _pose(), _scan({0: 1.0}))
    assert inv.frame_id == SAVED
    inv.detach()
    assert inv.report()["frame_id"] is None


def _throttled_runner(world, wall, mission):
    grid = build_world("complex_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object="unicorn", max_steps=5, policy="frontier",
                           world=world, detections_fn=lambda frame: None)
    runner.inventory_prior = lambda f: None
    runner.seed_wall_clock = lambda: wall[0]
    runner.clock = lambda: mission[0]
    return runner


@pytest.mark.parametrize("moving", ["wall", "mission"])
def test_the_read_limit_is_two_seconds_on_either_clock(moving):
    """Review 4: a pose read for seeding once either clock has moved
    SEED_EVERY_S -- a simulated clock can stand still, or run fast."""
    world = _HeldPose(None)
    wall, mission = [0.0], [0.0]
    runner = _throttled_runner(world, wall, mission)
    clock = wall if moving == "wall" else mission
    reads = []
    for step in range(41):                      # 0.0 .. 4.0 s in 0.1 s steps
        clock[0] = step * 0.1
        before = world.reads
        runner._observe({})
        reads.append(world.reads - before)
    assert sum(reads) == 3                      # at 0, 2 and 4 s
    assert [i for i, r in enumerate(reads) if r] == [0, 20, 40]


@pytest.mark.parametrize("weight, hits", [(0.5, 1), (2.5, 3), (0.49, 0), (3.0, 3)])
def test_carried_hits_round_half_up(weight, hits):
    saved = _session_one()
    saved["reported"], saved["candidates"] = [], [
        {"label": "sofa", "x_m": 2.0, "y_m": 2.0, "belief": 0.9, "votes": {"sofa": weight}}]
    inv = Inventory()
    inv.seed(SAVED, saved)
    assert inv.landmarks[0].hits == hits


# ---------- the S3 copy (the user, 2026-10-31: strip the new fields) ----------

def test_the_s3_copy_holds_only_this_missions_own_landmarks(tmp_path):
    """G12/G13, the user's call: S3 gets what it got before D5 -- no map id,
    mission id or chain fields, and no landmark this mission did not see
    itself. The local file keeps everything."""
    class _S3:
        def __init__(self):
            self.puts = []

        def put_object(self, **kw):
            self.puts.append(kw)

    inv = Inventory()
    inv.seed(SAVED, _session_one())                      # carried, never seen here
    inv.observe([{"label": "lamp", "bearing_deg": 0.0}], _pose(h=180.0), _scan({0: 1.5}),
                step=3)                                  # this mission's own
    report = dict(inv.report(), mission="Find the lamp.", mission_id="m2", outcome="found")
    s3 = _S3()
    store = InventoryStore(str(tmp_path), bucket="b", client=s3)
    store.save("m2", report)
    up = json.loads(s3.puts[0]["Body"])
    for key in ("frame_id", "carried_from", "carry_seq", "mission_id"):
        assert key not in up, key
    seen = up["reported"] + up["candidates"]
    assert [d["label"] for d in seen] == ["lamp"]
    assert all("carried" not in d for d in seen) and "carried" not in up["counts"]
    assert up["counts"]["landmarks"] == len(seen)
    local = json.loads((tmp_path / "m2.json").read_text())
    assert local["frame_id"] == SAVED and local["carry_seq"] == 1
    assert {"stool", "box", "lamp"} <= {d["label"] for d in local["reported"] + local["candidates"]}
