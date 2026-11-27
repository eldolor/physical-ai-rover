# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.227: plain explore sees the target on frames with no `detections`
(brain/explore_sight.py; docs/plans/ros-alignment/3.227-explore-real-detections.md)."""

import functools
import importlib.util
import json
import statistics
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from brain.explore_sight import NO_IDENTITY_TIER, PerceptionSight
from brain.perceive import (ABSENT, DETECTED, FrameReportedPipeline, Perception,
                            PerceptionUnavailable)
from tests.conftest import RecordingRobot, fresh_mock_robot
from tests.test_brain_server import _StubNavigator

ROOT = Path(__file__).resolve().parents[1]
SWEEP = ROOT / "evaluations/explore-sight-3227"


@functools.lru_cache(maxsize=None)
def _sweep():
    """The sweep module under a name of its own, with sys.path untouched
    (eighteen evaluations have a `sweep.py`)."""
    spec = importlib.util.spec_from_file_location("explore_sight_3227_sweep",
                                                  SWEEP / "sweep.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------- the vision_fn ----------

class _Pipe:
    def __init__(self, status=ABSENT):
        self.status, self.closed, self.frames = status, 0, []

    def perceive(self, frame):
        self.frames.append(frame)
        return Perception(status=self.status)

    def close(self):
        self.closed += 1


def test_the_scene_is_the_agents_own_plus_perception():
    pipe = _Pipe(DETECTED)
    sight = PerceptionSight(pipe)
    sight.bind_base(lambda frame: {"free_space": "clear", "important_objects": []})
    scene = sight({"image_base64": "x"})
    assert scene["free_space"] == "clear" and scene["important_objects"] == []
    assert scene["_perception"]["status"] == DETECTED
    assert (sight.frames, sight.detected) == (1, 1)


def test_an_unbound_sight_refuses_rather_than_inventing_a_scene():
    with pytest.raises(RuntimeError, match="bind_base"):
        PerceptionSight(_Pipe())({})


def test_with_no_identity_tier_an_arrival_is_undecided_and_free():
    v = PerceptionSight(_Pipe()).confirm_arrival({}, local_ok=False)
    assert v["undecided"] and not v["confirmed"] and not v["cloud_called"]
    assert v[NO_IDENTITY_TIER] is True


@pytest.mark.parametrize("said,confirmed", [(True, True), (False, False)])
def test_the_geometry_identity_is_a_decided_sim_verdict(said, confirmed):
    v = PerceptionSight(_Pipe(), identity_fn=lambda f: said).confirm_arrival({})
    assert (v["confirmed"], v["tier"], v["cloud_called"]) == (confirmed, "sim", False)
    assert NO_IDENTITY_TIER not in v and not v.get("undecided")


def test_a_geometry_that_cannot_judge_is_no_tier():
    v = PerceptionSight(_Pipe(), identity_fn=lambda f: None).confirm_arrival({})
    assert v[NO_IDENTITY_TIER] is True


def test_close_closes_the_pipeline():
    pipe = _Pipe()
    PerceptionSight(pipe).close()
    assert pipe.closed == 1


# ---------- through MissionRunner, on a real-camera stand-in ----------

def _run(arm, house):
    return _sweep().run(arm, house)


def test_dev_explore_on_real_frames_never_sees_the_target():
    """Arm B, criterion 2's "before": stripped frames, no perception."""
    row = _run("B", "procthor:val:5")
    assert row["outcome"] == "exhausted" and row["sighted"] == 0
    assert row["sight_from"] == "frame"


def test_explore_on_real_frames_arrives_and_ends_unconfirmed():
    """Arm R: the same house, perception's sightings; at the target, never
    `found`, no late confirmation pending (no cloud on this policy)."""
    sweep = _sweep()
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_world import MockWorld
    from tests.test_explore import FakeNav

    grid = build_world("procthor:val:5")
    robot, vision_fn = sweep.build("R", grid)
    runner = MissionRunner(robot, target_object=sweep.TARGET, policy="explore",
                           vision_fn=vision_fn, navigator=FakeNav(robot),
                           world=MockWorld(grid), mission_time_limit_s=1200,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    for _ in range(20000):
        if not runner.tick():
            break
    s = runner.status()
    assert runner.agent.sight_from == "perception"
    assert s["outcome"] == "arrived_unconfirmed", s["log_tail"][-3:]
    assert sweep.target_m(grid) <= sweep.AT_TARGET_M
    assert not runner.memory.found
    assert not runner.late_confirmation_pending()
    assert vision_fn.detected > 0 and "identity tier" in s["log_tail"][-1]


def test_the_simulator_keeps_devs_frame_source():
    """Arm S on one house: no vision_fn, `frame` sightings, `found`."""
    row = _run("S", "procthor:val:5")
    assert (row["sight_from"], row["outcome"]) == ("frame", "found")


def test_the_recorded_sweep_meets_section_a():
    """Criteria 1-5 as recorded in results/ (3.227 section B)."""
    load = lambda a: json.loads((SWEEP / "results" / f"{a}.json").read_text())  # noqa: E731
    f, s, b, r = (load(a) for a in "FSBR")
    key = lambda row: (row["outcome"], row["sim_s"], row["goals"])  # noqa: E731
    assert [key(x) for x in f["rows"]] == [key(x) for x in s["rows"]]          # 1
    assert b["sighted_missions"] == 0 and b["found"] == 0                       # 2
    assert f["found"] == 20
    assert r["arrived_unconfirmed_at_target"] == 19 >= int(0.9 * f["found"])    # 3
    assert r["found"] == 0 and r["arrived_unconfirmed_away"] == 0               # 4
    fr = {x["house"]: x for x in f["rows"]}
    both = [x for x in r["rows"] if fr[x["house"]]["outcome"] == "found"
            and x["outcome"] == "arrived_unconfirmed" and x["end_target_m"] <= 0.9]
    ratio = (statistics.median(x["sim_s"] for x in both)
             / statistics.median(fr[x["house"]]["sim_s"] for x in both))
    assert len(both) == 19 and ratio <= 1.5                                     # 5


# ---------- the brain server's choice, at the start ----------

def _app(tmp_path, monkeypatch, **extra):
    import control.brain_server as bs
    closes = []

    class Nav(_StubNavigator):
        def close(self):
            closes.append(1)

    monkeypatch.setattr(bs, "RemoteNavigator", Nav)
    lines = ["brain:", "  tick_interval_s: 0.0"] + [f"  {k}: {v}" for k, v in extra.items()]
    config = tmp_path / "robot.yaml"
    config.write_text("\n".join(lines) + "\n")
    app = bs.create_app(config_path=str(config),
                        robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
    return app, closes


FIND = {"policy": "explore", "target_object": "red backpack"}


def test_auto_on_a_simulated_camera_loads_nothing(tmp_path, monkeypatch):
    import brain.perceive as perceive
    monkeypatch.setattr(perceive, "pipeline_for",
                        lambda *a, **k: pytest.fail("no model on a simulated camera"))
    app, _ = _app(tmp_path, monkeypatch)
    with TestClient(app) as client:
        resp = client.post("/mission/start", json=FIND)
        runner = app.state.brain_state["runner"]
        client.post("/mission/stop")
    assert resp.status_code == 200, resp.text
    assert runner.agent.sight_from == "frame"
    assert not isinstance(runner.vision_fn, PerceptionSight)


def test_a_real_camera_find_sees_with_the_tiers_pipeline(tmp_path, monkeypatch):
    import brain.perceive as perceive
    import control.brain_server as bs
    pipe, asked = _Pipe(), {}

    def pipeline_for(target, **kw):
        asked.update(target=target, **kw)
        return pipe

    monkeypatch.setattr(perceive, "pipeline_for", pipeline_for)
    monkeypatch.setattr(bs, "_frame_source", lambda robot: "")
    app, _ = _app(tmp_path, monkeypatch, perception_match_probability=0.85)
    with TestClient(app) as client:
        resp = client.post("/mission/start", json=FIND)
        runner = app.state.brain_state["runner"]
        client.post("/mission/stop")
    assert resp.status_code == 200, resp.text
    assert runner.agent.sight_from == "perception"
    assert isinstance(runner.vision_fn, PerceptionSight) and runner.vision_fn.pipeline is pipe
    assert asked == {"target": "red backpack", "match_probability": 0.85}
    assert pipe.closed >= 1                     # closed at the mission's end (criterion 7)


def test_a_real_camera_find_with_no_perception_install_is_refused(tmp_path, monkeypatch):
    import brain.perceive as perceive
    import control.brain_server as bs

    def missing(*a, **k):
        raise PerceptionUnavailable("pip install -r requirements-perception.txt")

    monkeypatch.setattr(perceive, "pipeline_for", missing)
    monkeypatch.setattr(bs, "_frame_source", lambda robot: "")
    app, closes = _app(tmp_path, monkeypatch)
    with TestClient(app) as client:
        resp = client.post("/mission/start", json=FIND)
        runner = app.state.brain_state.get("runner")
    assert resp.status_code == 400, resp.text
    assert "explore policy" in resp.text and "requirements-perception.txt" in resp.text
    assert closes, "a refused start closes its navigator"
    assert runner is None or runner.status()["outcome"] in (None, "idle")


@pytest.mark.parametrize("sight", ["auto", "perception"])
def test_a_failed_camera_probe_refuses_the_start_and_loads_nothing(tmp_path, monkeypatch,
                                                                    sight):
    """Reviews: a failed read at the start says nothing about the camera --
    no model is loaded (a render's, 1.12) and no blind find runs (a real
    camera's); a 400 that says to try again."""
    import brain.perceive as perceive
    import control.brain_server as bs
    monkeypatch.setattr(perceive, "pipeline_for",
                        lambda *a, **k: pytest.fail("a failed probe loads no model"))
    monkeypatch.setattr(bs, "_frame_source", lambda robot: None)
    app, closes = _app(tmp_path, monkeypatch, explore_sight=sight)
    with TestClient(app) as client:
        resp = client.post("/mission/start", json=FIND)
    assert resp.status_code == 400, resp.text
    assert "could not read the camera" in resp.text and closes


@pytest.mark.parametrize("frame", [None, {"metadata": None}, "not a frame"])
def test_the_probe_reads_odd_frames_without_raising(frame):
    import control.brain_server as bs

    class Body:
        def get_camera_frame(self):
            return frame

    assert bs._frame_source(Body()) == (None if not isinstance(frame, dict) else "")


def test_the_sweep_runs_3_80_gs_houses():
    """The copied house list stays 3.80.G's (review)."""
    spec = importlib.util.spec_from_file_location(
        "pilot_380e", ROOT / "evaluations/tiered-380/E/pilot.py")
    pilot = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pilot)
    assert _sweep().SWEEP_HOUSES == pilot.SWEEP_HOUSES


def test_a_real_camera_survey_loads_nothing(tmp_path, monkeypatch):
    import brain.perceive as perceive
    import control.brain_server as bs
    monkeypatch.setattr(perceive, "pipeline_for",
                        lambda *a, **k: pytest.fail("a survey has no target to perceive"))
    monkeypatch.setattr(bs, "_frame_source", lambda robot: "")
    app, _ = _app(tmp_path, monkeypatch)
    with TestClient(app) as client:
        resp = client.post("/mission/start", json={"job": "survey", "policy": "explore"})
        runner = app.state.brain_state["runner"]
        client.post("/mission/stop")
    assert resp.status_code == 200, resp.text
    assert runner.agent.sight_from == "frame"


def test_a_refused_start_after_the_build_closes_the_pipeline(tmp_path, monkeypatch):
    """Find room labels from the cloud with no vision service refuse after
    perception was built: its models are let go (review of the build)."""
    import brain.perceive as perceive
    import control.brain_server as bs
    pipe = _Pipe()
    monkeypatch.setattr(perceive, "pipeline_for", lambda *a, **k: pipe)
    monkeypatch.setattr(bs, "_frame_source", lambda robot: "")
    monkeypatch.setattr(bs, "rooms_prior_informative", lambda target: True)
    app, _ = _app(tmp_path, monkeypatch, find_room_labels="cloud")
    with TestClient(app) as client:
        resp = client.post("/mission/start", json=FIND)
    assert resp.status_code == 400, resp.text
    assert pipe.closed == 1


def test_perception_in_the_simulator_reads_the_frames_report(tmp_path, monkeypatch):
    import brain.perceive as perceive
    monkeypatch.setattr(perceive, "pipeline_for",
                        lambda *a, **k: pytest.fail("1.12: no detector on a render"))
    app, _ = _app(tmp_path, monkeypatch, explore_sight="perception")
    with TestClient(app) as client:
        resp = client.post("/mission/start", json=FIND)
        runner = app.state.brain_state["runner"]
        client.post("/mission/stop")
    assert resp.status_code == 200, resp.text
    assert runner.agent.sight_from == "perception"
    assert isinstance(runner.vision_fn.pipeline, FrameReportedPipeline)
    assert runner.vision_fn.identity_fn is not None       # sim_arrival_confirm: geometry


def test_an_unknown_explore_sight_is_refused_at_load(tmp_path):
    from control.brain_config import load_brain_config
    config = tmp_path / "robot.yaml"
    config.write_text("brain:\n  explore_sight: frame\n")
    with pytest.raises(ValueError, match="explore_sight"):
        load_brain_config(str(config))
