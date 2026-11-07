# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.160.B: rooms a person names, kept with the saved map
(docs/plans/ros-alignment/3.160-rooms-named-on-twin.md, N3-N6 and N8; the
twin's N1, N2 and N7 are tests/test_ui_room_names.py)."""

import os
import random
import subprocess
from pathlib import Path

import numpy as np
import pytest
from fastapi.testclient import TestClient

from brain import room_labels as rl
from brain import room_names as rn
from sim.maps import build_world
from tests import room_cut_compare as rc
from tests.test_survey import _two_rooms
from world.interface import CELL_FREE, CELL_UNKNOWN

FRAME = "map-0123abcd"
ROOT = Path(__file__).resolve().parents[1]


def _cut(m):
    seg, _ = rl.segment(m)
    return seg


def _rooms_of(seg):
    return sorted(int(r) for r in np.unique(seg) if r >= 0)


# ---------- the store (N3) ----------

def test_a_name_is_kept_and_read_back_on_the_same_rooms_after_a_reload(tmp_path):
    m, store = _two_rooms(), rn.RoomNames(tmp_path)
    seg = _cut(m)
    left = int(seg[16, 15])
    store.edit(FRAME, m, seg, "name", room=left, name="  Kitchen ")
    again = rn.RoomNames(tmp_path)                       # a new process, the same file
    rooms = again.rooms(FRAME)
    assert [r["name"] for r in rooms] == ["Kitchen"]
    assert rn.match(m, seg, rooms) == {left: 0}
    assert (tmp_path / f"{FRAME}.json").exists()


def test_no_frame_id_refuses_every_edit(tmp_path):
    m = _two_rooms()
    seg = _cut(m)
    for fid in (None, "", "../etc/passwd"):
        with pytest.raises(rn.NamesRefused):
            rn.RoomNames(tmp_path).edit(fid, m, seg, "name", room=0, name="x")
    assert list(tmp_path.iterdir()) == []


def test_merge_keeps_the_larger_rooms_name_and_split_makes_two(tmp_path):
    m, store = _two_rooms(), rn.RoomNames(tmp_path)
    seg = _cut(m)
    a, b = int(seg[16, 15]), int(seg[16, 47])
    store.edit(FRAME, m, seg, "name", room=a, name="Den")
    store.edit(FRAME, m, seg, "merge", room=a, other=b)
    rooms = store.rooms(FRAME)
    assert [r["name"] for r in rooms] == ["Den"]
    assert rn.match(m, seg, rooms) == {a: 0, b: 0}
    store.edit(FRAME, m, seg, "split", room=a, line=[[3.05, 0.0], [3.05, 3.2]])
    names = sorted(r["name"] for r in store.rooms(FRAME))
    assert names == ["", "Den"]
    with pytest.raises(rn.NamesRefused):
        store.edit(FRAME, m, seg, "split", room=a, line=[[9.0, 9.0], [9.5, 9.0]])
    with pytest.raises(rn.NamesRefused):
        store.edit(FRAME, m, seg, "name", room=99, name="nowhere")
    with pytest.raises(rn.NamesRefused):
        store.edit(FRAME, m, seg, "name", room=a, name="x" * 41)


# ---------- the labeller (N5, N8) ----------

class _Names:
    def __init__(self, store):
        self.store = store

    def rooms(self, frame_id):
        return self.store.rooms(frame_id)


def _pose(x, y, frame_id=FRAME):
    return {"usable": True, "x_m": x, "y_m": y, "heading_deg": 0.0, "frame_id": frame_id}


def test_a_named_room_is_never_asked_about_and_its_name_wins(tmp_path):
    m, store = _two_rooms(), rn.RoomNames(tmp_path)
    seg = _cut(m)
    left = int(seg[16, 15])
    store.edit(FRAME, m, seg, "name", room=left, name="Kitchen")
    asked = []
    lab = rl.RoomLabeller(lambda f: asked.append(f) or "bedroom", mode="rooms", names=store)
    assert lab.observe(m, _pose(1.5, 1.6), lambda: {"frame": 1}) is None   # named: not asked
    assert asked == []
    assert lab.observe(m, _pose(4.7, 1.6), lambda: {"frame": 2}) == "bedroom"
    assert len(asked) == 1
    layer = lab.layer(m)
    w = m["width"]
    assert layer["names"][layer["cells"][16 * w + 15]] == "Kitchen"
    assert layer["names"][layer["cells"][16 * w + 47]] == "bedroom"
    assert lab.status()["person_named"] == ["Kitchen"]


def test_a_person_name_overrides_the_models_label_in_the_layer(tmp_path):
    m, store = _two_rooms(), rn.RoomNames(tmp_path)
    seg = _cut(m)
    lab = rl.RoomLabeller(lambda f: "garage", mode="rooms", names=store)
    lab.observe(m, _pose(1.5, 1.6), lambda: {"frame": 1})                  # the model says garage
    store.edit(FRAME, m, seg, "name", room=int(seg[16, 15]), name="Study")  # then a person names it
    layer = lab.layer(m)
    assert layer["names"][layer["cells"][16 * m["width"] + 15]] == "Study"
    assert "garage" not in layer["names"]


def test_with_no_names_the_labeller_is_exactly_as_before(tmp_path):
    """N8: an empty store, or no frame_id, changes nothing."""
    for mode in rl.MODES:
        runs = []
        for names, fid in ((None, None), (rn.RoomNames(tmp_path), FRAME), (rn.RoomNames(tmp_path), None)):
            calls = iter(["kitchen", "hall", "bedroom", "office"] * 5)
            lab = rl.RoomLabeller(lambda f: next(calls), mode=mode, names=names)
            m = _two_rooms()
            got = [lab.observe(m, _pose(x, 1.6, fid), lambda: {"f": 1}) for x in (1.5, 2.5, 4.0, 5.5)]
            runs.append((got, lab.layer(m), lab.calls))
        assert runs[0] == runs[1] == runs[2], mode


# ---------- a re-map, on the simulator's truth (N4) ----------

def _truth_map(house, shift=0, drop=0.0, seed=0):
    """The sim house fully mapped, at 0.10 m (k=3), with `shift` cells of
    unknown added west and north (the grid grown: the same metres), and a
    share `drop` of the free cells next to a wall unknown (seen differently)."""
    m = rc.house_map(house, k=3, pad=shift)
    m.update({"usable": True, "map_id": "slam-x", "map_version": 1,
              "origin_x_m": -shift * 0.10, "origin_y_m": -shift * 0.10})
    if drop:
        rng = random.Random(seed)
        g = np.array(m["cells"]).reshape(m["height"], m["width"])
        near = np.zeros_like(g, dtype=bool)
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            near |= np.roll(g, (dy, dx), axis=(0, 1)) == 1
        for y, x in zip(*np.nonzero((g == CELL_FREE) & near)):
            if rng.random() < drop:
                g[y, x] = CELL_UNKNOWN
        m["cells"] = g.ravel().tolist()
    return m


def _truth_room(world, m, seg, rid):
    """The sim room most of a cut room lies in (layout cells are 0.30 m)."""
    votes = {}
    ys, xs = np.nonzero(seg == rid)
    for y, x in zip(ys, xs):
        mx = m["origin_x_m"] + (x + 0.5) * m["resolution_m"]
        my = m["origin_y_m"] + (y + 0.5) * m["resolution_m"]
        name = world.room_at(int(mx // 0.30), int(my // 0.30))
        if name and name != "unknown":
            votes[name] = votes.get(name, 0) + 1
    return max(votes, key=votes.get) if votes else ""


def _store_truth_rooms(store, world):
    """What a person ends with after naming (merging and splitting the cut
    until it matches): each sim room's floor, stored under its name."""
    rooms = []
    for name, cells in world.rooms.items():
        keys = {(x * 3 + i, y * 3 + j) for x, y in cells for i in range(3) for j in range(3)}
        rooms.append({"id": name[:8], "by": "person", "name": name, "runs": rn.to_runs(keys)})
    store._save(FRAME, {"version": 1, "frame_id": FRAME, "rooms": rooms, "history": []})
    return store.rooms(FRAME)


@pytest.mark.parametrize("house", ["scaled_house", "home_first_floor", "complex_house"])
def test_names_follow_a_remap_by_overlap_and_never_land_on_a_wrong_room(house, tmp_path):
    """N4 on the simulator's truth: the house's rooms named, then three
    re-maps (the grid grown west and north; 5% and 10% of the floor by the
    walls seen differently). Kept: a named room still placed and naming its
    cells; wrong: a free cell given a name though it lies in another room."""
    world = build_world(house)
    rooms = _store_truth_rooms(rn.RoomNames(tmp_path), world)
    kept = total = wrong = named = 0
    for variant in (dict(shift=4), dict(drop=0.05, seed=1), dict(shift=2, drop=0.10, seed=2)):
        m1 = _truth_map(house, **variant)
        names = rn.name_cells(m1, rooms)
        ok = rn.placed(m1, rooms)
        kept += sum(ok)
        total += len(rooms)
        w = m1["width"]
        for i, n in enumerate(names):
            if not n or m1["cells"][i] != CELL_FREE:
                continue
            x, y = i % w, i // w
            mx = m1["origin_x_m"] + (x + 0.5) * 0.10
            my = m1["origin_y_m"] + (y + 0.5) * 0.10
            truth = world.room_at(int(mx // 0.30), int(my // 0.30))
            named += 1
            wrong += truth != n
    assert named and wrong == 0, (wrong, named)
    assert kept / total >= 0.90, (kept, total)


def test_the_cut_room_rule_alone_lost_rooms_a_person_split(tmp_path):
    """Recorded (3.160.B): naming by cut room (>= 60% inside a stored room),
    as first designed, kept 6 of 15 rooms in scaled_house -- the cut merges
    rooms across wide doorways -- which is why names are per cell."""
    world = build_world("scaled_house")
    rooms = _store_truth_rooms(rn.RoomNames(tmp_path), world)
    m1 = _truth_map("scaled_house", shift=4)
    assert len(set(rn.match(m1, _cut(m1), rooms).values())) < 0.9 * len(rooms)


# ---------- the routes ----------

class _World:
    def __init__(self, m, frame_id=FRAME):
        self.m, self.frame_id = m, frame_id

    def get_map(self):
        return self.m

    def get_pose(self):
        return _pose(1.5, 1.6, self.frame_id)


def _client(monkeypatch, tmp_path, world):
    monkeypatch.setenv("ROOM_NAMES_DIR", str(tmp_path))
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setattr(rn, "_DEFAULT", None)
    from control import brain_server
    app = brain_server.create_app(robot_factory=lambda: object(), world_factory=lambda robot: world)
    return TestClient(app)


def test_the_routes_name_a_room_and_refuse_a_stale_cut(monkeypatch, tmp_path):
    m = _two_rooms()
    c = _client(monkeypatch, tmp_path, _World(m))
    got = c.get("/rooms").json()
    assert got["frame_id"] == FRAME and len(got["rooms"]) == 2
    assert all(r["name"] == "" and r["source"] == "none" for r in got["rooms"])
    rid = got["rooms"][0]["key"]
    r = c.post("/rooms/names", json={"frame_id": FRAME, "map_version": m["map_version"],
                                     "op": "name", "room": rid, "name": "Hall"})
    assert r.status_code == 200, r.text
    assert [x["name"] for x in r.json()["rooms"] if x["source"] == "person"] == ["Hall"]
    stale = c.post("/rooms/names", json={"frame_id": FRAME, "map_version": 0, "op": "name",
                                         "room": rid, "name": "x"})
    assert stale.status_code == 409
    other = c.post("/rooms/names", json={"frame_id": "map-ffffffff", "map_version": 1,
                                         "op": "name", "room": rid, "name": "x"})
    assert other.status_code == 409


def test_with_no_saved_map_the_editor_says_survey_first(monkeypatch, tmp_path):
    c = _client(monkeypatch, tmp_path, _World(_two_rooms(), frame_id=None))
    got = c.get("/rooms").json()
    assert got["frame_id"] is None and "Survey first" in got["closed"]
    r = c.post("/rooms/names", json={"frame_id": None, "map_version": 1, "op": "name",
                                     "room": "c0", "name": "x"})
    assert r.status_code == 409


# ---------- local only (N6) ----------

def test_names_stay_local_no_other_module_reads_the_store():
    """Only the store, the labeller and the brain's routes name it; nothing
    that uploads (the inventory's S3 copy, the walks store) does."""
    out = subprocess.run(["git", "grep", "--untracked", "-l", "-e", "import room_names",
                          "-e", "room_names as", "-e", "ROOM_NAMES_DIR", "--", "*.py"],
                         cwd=ROOT, capture_output=True, text=True).stdout.split()
    assert sorted(out) == ["brain/room_labels.py", "brain/room_names.py",
                           "control/brain_server.py", "tests/test_room_names.py"]
    assert "recordings/" in (ROOT / ".gitignore").read_text()
