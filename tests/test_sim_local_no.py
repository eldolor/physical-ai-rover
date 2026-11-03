# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_sim_local_no.py

3.143, fix V (the user's option (a), 2026-11-03): in the simulator a local
no is judged by the simulator's geometry -- at arrival (`confirm_local`'s no
gives way to 3.98's `geometry_identity`) and on a frame's question (a no
about perception's candidate is not remembered, 3.91.C) -- because VE judging
a raycaster's render refused the true target in 5 of the 3.135 soak's 9
misses. Real frames are untouched (criterion 3, V2).
"""

import functools
import logging

import pytest

from brain.arrival import geometry_identity
from brain.perceive import DETECTED, FrameReportedPipeline
from brain.tiered import TieredVision, _LocalAnswer, _Question
from control.mission_runner import FOUND, MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav

TARGET = "red backpack"


@pytest.fixture(autouse=True)
def _quiet_logs():
    logging.disable(logging.WARNING)
    yield
    logging.disable(logging.NOTSET)


class NoLocal:
    """The worst VE: no to every question."""

    def __init__(self):
        self.asked = 0

    def __call__(self, frame, box=False):
        self.asked += 1
        return {"status": "answered", "score": 0.0}


class NoCloud:
    def __init__(self):
        self.calls = 0

    def __call__(self, frame):
        self.calls += 1
        return {"obstacles_ahead": [], "free_space": "clear", "doorway_visible": False,
                "important_objects": [], "safest_direction": "RIGHT",
                "_navigate": {"reasoning": "stand-in", "target_visible": False}}


def _tier(local, sim=True):
    return TieredVision(FrameReportedPipeline(TARGET), NoCloud(), confirm_vision_fn=NoCloud(),
                        local_identity_fn=local, local_sighting_fn=local,
                        sim_identity_fn=(functools.partial(geometry_identity, target=TARGET)
                                         if sim else None))


def _frame(label=TARGET, bearing=2.0, source="sim"):
    f = {"detections": [{"label": label, "bearing_deg": bearing, "distance_m": 0.5}],
         "objects_visible": [label], "pan_deg": 0.0}
    if source is not None:
        f["metadata"] = {"source": source}
    return f


def test_a_local_no_at_a_sim_arrival_on_the_target_is_the_geometrys_yes():
    local = NoLocal()
    v = _tier(local).confirm_local(_frame())
    assert local.asked == 1                     # VE still asked (V3: at every arrival)
    assert v["confirmed"] is True and v["tier"] == "sim" and v["local_overruled"] is True
    assert v["local_called"] is True and v["local_score"] == 0.0 and not v["cloud_called"]
    assert v["stats"]["local"]["no"] == 1 and v["stats"]["local"]["no_overruled_sim"] == 1


def test_a_local_no_at_a_sim_arrival_on_something_else_stands():
    t = _tier(NoLocal())
    v = t.confirm_local(_frame(label="chair"))
    # VE's own no, the geometry agreeing: unrelabelled, not counted (review).
    assert v["confirmed"] is False and v["tier"] == "local" and "local_overruled" not in v
    assert "no_overruled_sim" not in t.stats.local


def test_a_local_no_on_a_real_frame_is_the_local_tiers():
    # V2: a real camera's frame (no sim provenance) never takes the geometry.
    for t in (_tier(NoLocal()), _tier(NoLocal(), sim=False)):
        v = t.confirm_local(_frame(source=None))
        assert v["confirmed"] is False and v["tier"] == "local" and "local_overruled" not in v


def test_a_local_yes_and_an_unable_answer_are_unchanged_in_the_sim():
    yes = _tier(lambda f, box=False: {"status": "answered", "score": 0.9}).confirm_local(_frame())
    assert yes["confirmed"] is True and yes["tier"] == "local"
    unable = _tier(lambda f, box=False: {"status": "timeout"}).confirm_local(_frame(label="chair"))
    assert unable.get("undecided") and unable["tier"] == "local"


def _mission(house, target, local, stamp=True):
    grid = build_world(house)

    class Robot(MockRobot):
        def get_camera_frame(self):
            f = super().get_camera_frame()
            if stamp:
                f.setdefault("metadata", {"source": "sim"})   # the board's rendered frames'
            return f

    robot = Robot(grid, render=False)
    tier = TieredVision(FrameReportedPipeline(target), NoCloud(), confirm_vision_fn=NoCloud(),
                        local_identity_fn=local, local_sighting_fn=local,
                        sim_identity_fn=functools.partial(geometry_identity, target=target))
    runner = MissionRunner(robot, target_object=target, policy="tiered", vision_fn=tier,
                           tier_search="explore", navigator=FakeNav(robot),
                           world=MockWorld(grid), max_steps=200, clock=lambda: grid.sim_time,
                           idle=robot.pass_time)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < 5000:
        ticks += 1
    return runner.status(), tier


def test_an_always_no_local_tier_still_finds_in_the_sim():
    # Red first: before the fix the always-no VE's refusals decided this
    # mission (3.143 A3n); with it the geometry judges, VE still asked.
    local = NoLocal()
    s, tier = _mission("home_first_floor", TARGET, local)
    assert s["outcome"] == FOUND
    assert local.asked > 0
    assert s["arrival_verdicts"][-1]["tier"] == "sim"
    assert s["arrival_verdicts"][-1]["local_score"] == 0.0


def test_a_frame_no_on_a_sim_frame_is_not_remembered():
    local = NoLocal()
    t = _tier(local)
    for _ in range(6):
        t(_frame(bearing=1.0))
    assert local.asked > 0
    assert t._rejection is None


def test_a_frame_no_on_a_real_frame_is_remembered():
    local = NoLocal()
    t = _tier(local)
    for _ in range(6):
        t(_frame(bearing=1.0, source=None))
    assert local.asked > 0
    assert t._rejection is not None and t._rejection["by"] == "local"


def test_a_band_no_on_a_sim_frame_rejects_no_candidate():
    # Review: 3.80.F's band rejection is a second memory of the same no.
    t = _tier(NoLocal())
    for sim, want in ((True, False), (False, True)):
        t._candidate_rejected = False
        q = _Question(trigger="candidate", epoch=t._epoch, pose=None, pan_deg=0.0,
                      hfov_deg=60.0, band=True, sim=sim)
        t._apply_local(_LocalAnswer(q=q, got={"status": "answered", "score": 0.0}))
        assert t._candidate_rejected is want


def test_perception_reads_the_stub_frames_as_a_detection():
    # The two tests above lean on FrameReportedPipeline calling the target.
    assert FrameReportedPipeline(TARGET).perceive(_frame()).status == DETECTED
