# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.160.D: a find mission with no labeller (`find_room_labels` off, or
skipped for a flat prior) reads the rooms a person named on the saved map for
explore's room goal policy, and asks nothing."""
import pytest

import brain.room_names as rn
from sim.mock_world import MockWorld
from tests.test_explore import CELL, _mission

FRAME = "map-3160d001"


class Store:
    """The person's rooms as RoomNames.load() gives them: `_keys` on the store
    grid. `rooms`: {name: [grid cells]}; `rev` moves with each edit."""

    def __init__(self, rooms=None):
        self.set(rooms or {})

    def set(self, rooms):
        k = round(CELL / rn.STORE_RES_M)
        self.doc = {"rooms": [{"name": n, "_keys": {(i * k + a, j * k + b) for i, j in cells
                                                     for a in range(k) for b in range(k)}}
                              for n, cells in sorted(rooms.items())],
                    "rev": getattr(self, "doc", {"rev": 0})["rev"] + 1}

    def load(self, frame_id):
        assert frame_id == FRAME
        return {"version": 1, "frame_id": frame_id, "history": [], **self.doc}

    def rooms(self, frame_id):
        return self.load(frame_id)["rooms"]


@pytest.fixture
def saved_map(monkeypatch):
    """The simulator's pose under a saved map's frame_id, and a store."""
    real = MockWorld.get_pose
    monkeypatch.setattr(MockWorld, "get_pose",
                        lambda self: {**real(self), "frame_id": FRAME})
    store = Store()
    monkeypatch.setattr(rn, "default_store", lambda: store)
    return store


def _true_rooms(house="scaled_house"):
    from sim.maps import build_world
    return {n: list(c) for n, c in build_world(house).rooms.items()}


def test_a_find_types_its_goals_from_the_names_and_asks_nothing(saved_map):
    saved_map.set(_true_rooms())
    runner, _nav, grid = _mission(explore_goal_policy="rooms",
                                  room_label_fn=lambda frame: pytest.fail("asked"))
    s = runner.status()
    assert s["outcome"] == "found", s["log_tail"]
    assert runner.room_labeller is None and s["survey"] is None
    assert s["room_labels"] == {"label_calls": 0, "person_rooms": len(grid.rooms)}
    m = runner.world.get_map()
    assert "room_labels" not in m                      # never written into the map
    cm, layer = runner.agent.goal_labels(m)
    assert len(layer["cells"]) == cm["width"] * cm["height"]
    assert layer["names"] and set(layer["names"]) <= set(grid.rooms)
    assert runner.room_labels() is None                # not a survey's labels
    # The names reach explore's typing: a goal in a known, named room is
    # typed with that room's name, and never with another's (review).
    w, res = m["width"], m["resolution_m"]
    goals, truth = [], []
    for name, cells in grid.rooms.items():
        for i, j in cells:
            mx, my = int((i + 0.5) * CELL / res), int((j + 0.5) * CELL / res)
            if m["cells"][my * w + mx] == 0:
                goals.append(((i + 0.5) * CELL, (j + 0.5) * CELL))
                truth.append(name)
    types = runner.agent._goal_types(m, goals)
    typed = [(t, n) for t, n in zip(types, truth) if t is not None]
    assert len(typed) > len(goals) // 2
    assert sum(t == n for t, n in typed) >= 0.95 * len(typed)


@pytest.mark.parametrize("why", ["no names", "no frame_id", "unnamed rooms"])
def test_no_names_is_no_source(why, saved_map, monkeypatch):
    if why == "no frame_id":
        saved_map.set(_true_rooms())
        named = MockWorld.get_pose
        monkeypatch.setattr(MockWorld, "get_pose", lambda self: {**named(self), "frame_id": None})
    if why == "unnamed rooms":
        saved_map.set({"": c for _n, c in list(_true_rooms().items())[:1]})
    runner, _nav, _g = _mission(explore_goal_policy="rooms")
    s = runner.status()
    assert s["outcome"] == "found", s["log_tail"]
    assert runner.agent.goal_labels(runner.world.get_map()) is None
    assert s["room_labels"] is None


def test_a_store_that_fails_is_no_names(saved_map, monkeypatch):
    def boom():
        raise OSError("disk gone")
    monkeypatch.setattr(rn, "default_store", boom)
    runner, _nav, _g = _mission(explore_goal_policy="rooms")
    s = runner.status()
    assert s["outcome"] == "found", s["log_tail"]
    assert runner.agent.goal_labels(runner.world.get_map()) is None
    assert s["room_labels"] is None


def test_a_damaged_file_is_no_names(monkeypatch, tmp_path):
    real = MockWorld.get_pose
    monkeypatch.setattr(MockWorld, "get_pose", lambda self: {**real(self), "frame_id": FRAME})
    (tmp_path / f"{FRAME}.json").write_text("{not json")
    store = rn.RoomNames(tmp_path)
    monkeypatch.setattr(rn, "default_store", lambda: store)
    runner, _nav, _g = _mission(explore_goal_policy="rooms")
    assert runner.status()["outcome"] == "found"
    assert runner.agent.goal_labels(runner.world.get_map()) is None


def test_an_edit_mid_mission_is_read_at_the_next_choice(saved_map):
    rooms = _true_rooms()
    first = dict(list(sorted(rooms.items()))[:1])
    saved_map.set(first)
    runner, _nav, _g = _mission(explore_goal_policy="rooms")
    m = runner.world.get_map()
    _cm, layer = runner.agent.goal_labels(m)
    assert layer["names"] == sorted(first)
    saved_map.set(rooms)
    _cm, layer = runner.agent.goal_labels(m)
    assert set(layer["names"]) <= set(rooms) and len(layer["names"]) > 1
    assert runner.status()["room_labels"]["person_rooms"] == len(rooms)


def test_a_flat_prior_find_reads_names_and_still_asks_nothing(saved_map):
    """3.116 amendment 7 skips the asks for the (flat-prior) backpack; the
    names are free, so it reads them."""
    saved_map.set(_true_rooms())
    runner, _nav, grid = _mission(explore_goal_policy="rooms", find_room_labels=True,
                                  room_label_fn=lambda frame: pytest.fail("asked"))
    s = runner.status()
    assert runner.room_labeller is None
    assert s["room_labels"] == {"skipped": "flat_prior", "label_calls": 0,
                                "person_rooms": len(grid.rooms)}
    assert runner.agent.goal_labels(runner.world.get_map()) is not None


def test_the_rule_chooser_reads_no_names(saved_map):
    """Only the rooms policy reads types: any other chooser gets no source."""
    saved_map.set(_true_rooms())
    runner, _nav, _g = _mission(explore_goal_policy="rule")
    assert runner.agent.goal_labels is None
    assert runner.status()["room_labels"] is None


def test_the_sweep_s_recorded_bars_hold():
    """3.160.D D1-D5, re-scored from the recorded arms (the sweep itself is
    200 missions an arm: evaluations/room-names-3160d/sweep.py)."""
    import importlib.util
    from pathlib import Path
    d = Path(__file__).resolve().parents[1] / "evaluations" / "room-names-3160d"
    spec = importlib.util.spec_from_file_location("score_3160d", d / "score.py")
    score = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(score)
    s = score.main(d)
    assert all(s[b]["met"] for b in ("D1", "D2", "D3", "D4", "D5")), s
    assert s["D1"]["ci95"][0] >= 0.08 and s["D4"]["noid"]["differ"] == s["D4"]["empty"]["differ"] == 0


def test_a_pose_dropout_keeps_the_names(saved_map, monkeypatch):
    """A momentarily unusable pose (stale SLAM, a dropped fetch) keeps the
    saved map the mission last read: the types do not flap (review)."""
    saved_map.set(_true_rooms())
    runner, _nav, grid = _mission(explore_goal_policy="rooms")
    m = runner.world.get_map()
    assert runner.agent.goal_labels(m) is not None
    monkeypatch.setattr(MockWorld, "get_pose", lambda self: {"usable": False, "frame_id": None})
    assert runner.agent.goal_labels(m) is not None
    assert runner.status()["room_labels"]["person_rooms"] == len(grid.rooms)


def test_a_failing_store_after_names_counts_none(saved_map, monkeypatch):
    saved_map.set(_true_rooms())
    runner, _nav, _g = _mission(explore_goal_policy="rooms")
    m = runner.world.get_map()
    assert runner.agent.goal_labels(m) is not None

    def boom():
        raise OSError("disk gone")
    monkeypatch.setattr(rn, "default_store", boom)
    assert runner.agent.goal_labels(m) is None
    assert runner.status()["room_labels"] is None


def test_a_failed_pose_fetch_keeps_the_names(saved_map, monkeypatch):
    """A pose fetch that raises (RemoteWorld's transport error) is a dropout
    too: the saved map last read still types the goals (Thermos)."""
    saved_map.set(_true_rooms())
    runner, _nav, grid = _mission(explore_goal_policy="rooms")
    m = runner.world.get_map()
    assert runner.agent.goal_labels(m) is not None

    def boom(self):
        raise OSError("world timed out")
    monkeypatch.setattr(MockWorld, "get_pose", boom)
    assert runner.agent.goal_labels(m) is not None
    assert runner.status()["room_labels"]["person_rooms"] == len(grid.rooms)


def test_names_off_the_map_count_none(saved_map):
    """Named rooms the live map cannot place type nothing, and the status says
    none were read, not how many the store holds (Thermos)."""
    saved_map.set({"attic": [(900, 900), (901, 900)]})
    runner, _nav, _g = _mission(explore_goal_policy="rooms")
    assert runner.agent.goal_labels(runner.world.get_map()) is None
    assert runner.status()["room_labels"] is None
