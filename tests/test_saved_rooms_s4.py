# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_saved_rooms_s4.py

3.70 S4 (docs/plans/ros-alignment/3.70-survey-and-mission-ends.md): a
survey's room labels and its inventory are saved as one file under the saved
map's `frame_id`; a restart and a reload on the same map return both equal to
what was written (labels cell for cell; landmarks by id, position, belief and
votes); after a fall-back (a new saved-map id) none of them is used.

Offline: the runner through `MissionRunner` on the sim, its poses on a saved
map's frame as RosWorld answers them on a matched reload (tests/
test_map_frame_carry.py's `_FramedWorld`); "a restart" is a new store and a
new runner over the same directory. The live round trip is in the section.
"""
import json

import pytest

import threading

from brain import room_labels as rl
from brain.goal_policy import coarse
from brain.room_labels import frame_room, read_saved, rle, unrle
from control.inventory_store import InventoryStore, _for_s3
from control.mission_runner import EXHAUSTED, MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from tests.test_map_frame_carry import _FramedWorld

SAVED = "map-5a4e0000"
OTHER = "map-ffffffff"


def _drive(runner, max_ticks=20000):
    ticks = 0
    while runner.tick():
        ticks += 1
        assert ticks < max_ticks, "runner never terminated"
    return runner.status()


def _run(store, frame_id, session, job="survey", house="scaled_house", lazy=False, **kw):
    grid = build_world(house)
    robot = MockRobot(grid, render=False)
    world = _FramedWorld(MockWorld(grid), frame_id, session)
    if job == "survey":
        kw.setdefault("room_label_fn", frame_room)
        runner = MissionRunner(robot, job="survey", policy="explore", navigator=FakeNav(robot),
                               world=world, clock=lambda: grid.sim_time,
                               idle=robot.pass_time, **kw)
    else:
        runner = MissionRunner(robot, target_object="unicorn", policy="explore",
                               navigator=FakeNav(robot), world=world,
                               clock=lambda: grid.sim_time, idle=robot.pass_time,
                               max_steps=kw.pop("max_steps", 60), **kw)
    runner.mission_id = f"mission-{session}"
    runner.inventory_prior = store.latest_for
    if lazy:
        threads = []
        runner.inventory_sink = lambda report: threads.append(
            store.save_async(runner.mission_id, report))
        runner.inventory_sink_lazy = True
    else:
        runner.inventory_sink = lambda report: store.save(runner.mission_id, report)
    runner.start()
    s = _drive(runner)
    if lazy:
        for t in threads:
            t.join(30)
    return runner, s


@pytest.fixture(scope="module")
def surveyed(tmp_path_factory):
    """Session 1: a survey of scaled_house on SAVED, saved to a store."""
    d = tmp_path_factory.mktemp("inv")
    store = InventoryStore(str(d))
    runner, s = _run(store, SAVED, "s1")
    return d, runner, s


def _saved_file(d, session):
    return json.loads((d / f"mission-{session}.json").read_text())


# ---------- the block ----------

def test_rle_round_trips_and_refuses_a_bad_run():
    cells = [-1] * 5 + [0] * 3 + [2] + [-1]
    assert unrle(rle(cells), len(cells)) == cells
    assert unrle(rle(cells), len(cells) + 1) is None
    assert unrle([0, 0], 0) is None and unrle([0, True], 1) is None and unrle([0], 1) is None


def test_a_damaged_block_reads_as_none():
    m = {"width": 2, "height": 1, "resolution_m": 0.3, "origin_x_m": 0.0, "origin_y_m": 0.0}
    ok = rl.saved_block({"names": ["hall"], "cells": [0, -1]}, m, [[0.1, 0.1, "hall"]],
                        mode="spacing", mission_id="m")
    assert read_saved(ok)["cells"] == [0, -1]
    for bad in ({**ok, "format": 2}, {**ok, "cells_rle": [1, 1, -1, 1]},   # index past names
                {**ok, "width": 3}, {**ok, "names": [3]}, {**ok, "resolution_m": 0},
                {**ok, "resolution_m": True}, {**ok, "origin_x_m": "0"},
                {**ok, "seeds": ["x"]}, {**ok, "seeds": [[0, 0, 1]]}, {**ok, "frame_id": 3},
                {**ok, "width": 100_000, "height": 100_000, "cells_rle": [0, 10 ** 10]},
                "x", None):
        assert read_saved(bad) is None


# ---------- S4: written as one, under one map ----------

def test_a_survey_writes_its_labels_and_inventory_in_one_file_on_its_map(surveyed):
    d, runner, s = surveyed
    assert s["outcome"] == EXHAUSTED, s["log_tail"]
    saved = _saved_file(d, "s1")
    assert saved["frame_id"] == SAVED
    block = read_saved(saved["room_labels"])
    assert block is not None and block["names"], "no labels saved"
    assert block["mission_id"] == "mission-s1"
    assert saved["reported"] + saved["candidates"], "no landmarks saved"
    # The labels as the survey's own layer, on the room policy's grid of the
    # last map it saw, cell for cell, and named for the inventory's frame.
    m = coarse(runner._label_seen[0])
    assert block["cells"] == runner.room_labeller.model_layer(m)["cells"]
    assert (block["width"], block["height"]) == (m["width"], m["height"])
    assert block["frame_id"] == SAVED


def test_the_s3_copy_carries_no_label_grid(surveyed):
    d, _runner, _s = surveyed
    assert "room_labels" not in _for_s3(_saved_file(d, "s1"))


def test_a_lazy_sink_builds_the_labels_on_the_save_thread(tmp_path, monkeypatch):
    built_on = []
    real = rl.saved_block
    monkeypatch.setattr(rl, "saved_block",
                        lambda *a, **k: built_on.append(threading.current_thread()) or real(*a, **k))
    store = InventoryStore(str(tmp_path))
    _run(store, SAVED, "s1", lazy=True)
    assert read_saved(_saved_file(tmp_path, "s1")["room_labels"])["names"]
    assert built_on and threading.current_thread() not in built_on


def test_labels_from_a_map_of_another_frame_are_never_saved(tmp_path):
    """Review 1: a map seen on another saved frame (a fresh map mid-survey)
    is never saved under the inventory's."""
    store = InventoryStore(str(tmp_path))
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, job="survey", policy="explore", navigator=FakeNav(robot),
                           world=_FramedWorld(MockWorld(grid), SAVED, "s1"),
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           room_label_fn=frame_room)
    runner.mission_id = "mission-s1"
    runner.inventory_prior = store.latest_for
    runner.inventory_sink = lambda report: store.save(runner.mission_id, report)
    runner.start()
    for _ in range(20):
        assert runner.tick()
    assert runner.inventory.frame_id == SAVED
    runner.world._frame = OTHER              # the world moves to another saved map
    while runner.tick():
        pass
    saved = _saved_file(tmp_path, "s1")
    assert saved["frame_id"] == SAVED and "room_labels" not in saved


def test_a_survey_whose_layer_names_nothing_keeps_the_carried_labels(surveyed, tmp_path,
                                                                    monkeypatch):
    d, _runner, _s = surveyed
    written = _saved_file(d, "s1")
    for p in d.iterdir():
        (tmp_path / p.name).write_text(p.read_text())
    monkeypatch.setattr(rl.RoomLabeller, "model_layer",
                        lambda self, m: {"names": [], "cells": [-1] * (m["width"] * m["height"])})
    _run(InventoryStore(str(tmp_path)), SAVED, "s2")
    assert _saved_file(tmp_path, "s2")["room_labels"] == written["room_labels"]


def test_a_detached_survey_saves_no_labels(tmp_path):
    def broken(frame_id):
        raise OSError("disk")
    store = InventoryStore(str(tmp_path))
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, job="survey", policy="explore", navigator=FakeNav(robot),
                           world=_FramedWorld(MockWorld(grid), SAVED, "s1"),
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           room_label_fn=frame_room)
    runner.mission_id = "mission-s1"
    runner.inventory_prior = broken
    runner.inventory_sink = lambda report: store.save(runner.mission_id, report)
    runner.start()
    while runner.tick():
        pass
    saved = _saved_file(tmp_path, "s1")
    assert saved["frame_id"] is None and "room_labels" not in saved


# ---------- S4: a restart and a reload return all three ----------

def test_a_reload_returns_the_labels_and_landmarks_as_written(surveyed, tmp_path):
    d, _runner, _s = surveyed
    written = _saved_file(d, "s1")
    # "A restart": a new store over a copy of the directory, a new runner.
    for p in d.iterdir():
        (tmp_path / p.name).write_text(p.read_text())
    store = InventoryStore(str(tmp_path))
    got = store.latest_for(SAVED)
    assert read_saved(got["room_labels"])["cells"] == read_saved(written["room_labels"])["cells"]
    assert got["room_labels"] == written["room_labels"]
    key = lambda r: (r["id"], r["label"], r["x_m"], r["y_m"], r["belief"],  # noqa: E731
                     json.dumps(r["votes"], sort_keys=True))
    assert sorted(map(key, got["reported"] + got["candidates"])) == \
        sorted(map(key, written["reported"] + written["candidates"]))
    # The next mission on the reloaded map holds the same labels block.
    two, _s2 = _run(store, SAVED, "s2", job="find")
    assert two.inventory_report()["room_labels"] == written["room_labels"]
    assert any("room labels:" in line and "carried" in line for line in two._log)
    # ...and passes it on, so the chain keeps it past a mission with no survey.
    assert _saved_file(tmp_path, "s2")["room_labels"] == written["room_labels"]
    three, _s3 = _run(store, SAVED, "s3", job="find")
    assert three.inventory_report()["room_labels"] == written["room_labels"]


def test_after_a_fall_back_no_old_label_or_landmark_is_used(surveyed, tmp_path):
    d, _runner, _s = surveyed
    for p in d.iterdir():
        (tmp_path / p.name).write_text(p.read_text())
    store = InventoryStore(str(tmp_path))
    two, _s2 = _run(store, OTHER, "s2", job="find")
    report = two.inventory_report()
    assert "room_labels" not in report
    assert report["counts"]["carried"] == 0
    assert not any(lm.carried for lm in two.inventory.landmarks)


def test_a_newer_survey_replaces_the_labels(surveyed, tmp_path):
    d, _runner, _s = surveyed
    for p in d.iterdir():
        (tmp_path / p.name).write_text(p.read_text())
    store = InventoryStore(str(tmp_path))
    two, _s2 = _run(store, SAVED, "s2")
    assert _saved_file(tmp_path, "s2")["room_labels"]["mission_id"] == "mission-s2"


def test_an_unreadable_saved_block_is_not_carried(surveyed, tmp_path):
    d, _runner, _s = surveyed
    written = _saved_file(d, "s1")
    written["room_labels"]["cells_rle"] = [0, 1]
    (tmp_path / "mission-s1.json").write_text(json.dumps(written))
    two, _s2 = _run(InventoryStore(str(tmp_path)), SAVED, "s2", job="find")
    assert "room_labels" not in two.inventory_report()
    assert two.inventory_report()["counts"]["carried"] > 0      # the landmarks still carry


def test_a_label_block_that_fails_never_costs_the_inventory(tmp_path, monkeypatch):
    monkeypatch.setattr(rl, "saved_block", lambda *a, **k: (_ for _ in ()).throw(ValueError("x")))
    store = InventoryStore(str(tmp_path))
    _run(store, SAVED, "s1")
    saved = _saved_file(tmp_path, "s1")
    assert "room_labels" not in saved and saved["frame_id"] == SAVED


# ---------- review round 2 ----------

def test_a_survey_cut_short_keeps_the_earlier_surveys_labels(surveyed, tmp_path):
    """A survey stopped or out of time saw part of the house: it must not
    replace a whole earlier survey's labels (it ends `out_of_budget` here)."""
    d, _runner, _s = surveyed
    written = _saved_file(d, "s1")
    for p in d.iterdir():
        (tmp_path / p.name).write_text(p.read_text())
    two, s2 = _run(InventoryStore(str(tmp_path)), SAVED, "s2", mission_time_limit_s=20)
    assert s2["outcome"] != EXHAUSTED and two.room_labeller.seeds
    assert _saved_file(tmp_path, "s2")["room_labels"] == written["room_labels"]


def test_a_first_survey_with_no_detections_still_saves_on_its_frame(tmp_path):
    """The car: frames carry no detections (3.76 E), so the inventory never
    records one -- it takes the frame at the seed, and the labels save."""
    store = InventoryStore(str(tmp_path))
    _run(store, SAVED, "s1", detections_fn=lambda frame: None)
    saved = _saved_file(tmp_path, "s1")
    assert saved["frame_id"] == SAVED
    assert read_saved(saved["room_labels"])["frame_id"] == SAVED
