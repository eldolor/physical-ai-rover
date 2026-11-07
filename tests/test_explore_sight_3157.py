# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.157.B: explore's sightings on the labelled rig walks -- the scorer's
reading of a frame, and the recorded numbers pinned
(docs/plans/ros-alignment/3.157-perception-into-explore.md)."""
import importlib.util
import json
import os
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1] / "evaluations" / "explore-sight-3157"


def _score():
    spec = importlib.util.spec_from_file_location("score_3157", HERE / "score.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_a_detection_reaches_explore_at_the_pipelines_bearing():
    s = _score()
    p = {"status": "detected", "p": 0.9, "box": [960, 0, 1280, 10], "w": 1280, "h": 720}
    hits = s.hits({"_perception": s.perception_scene(p)}, "blue bottle")
    # box centre 0.875 of the width -> 0.375 * 66 = 24.75 deg right
    assert hits == [{"bearing_deg": pytest.approx(24.75)}]


def test_no_detection_and_a_refused_candidate_give_no_sighting():
    s = _score()
    absent = {"status": "absent", "p": None, "box": None, "w": 1280, "h": 720}
    assert s.hits({"_perception": s.perception_scene(absent)}, "blue bottle") == []
    det = {"status": "detected", "p": 0.9, "box": [600, 0, 680, 10], "w": 1280, "h": 720}
    scene = {"_perception": s.perception_scene(det), "_tier": {"candidate_refused": True}}
    assert s.hits(scene, "blue bottle") == []


def test_a_local_yes_with_a_box_is_read_in_the_models_frame():
    s = _score()
    mw, mh = s.model_size(1280, 720)
    q = {"status": "answered", "score": 0.9, "box": [0, 0, mw / 2, mh]}
    land = s.local_landed(q, 1280, 720)
    assert land["answer"] == "yes"
    # box centre 0.25 of the width -> -0.25 * 66 = -16.5 deg
    assert land["bearing_deg"] == pytest.approx(-16.5)
    assert s.local_landed({"status": "answered", "score": 0.3, "box": [0, 0, 1, 1]}, 1280, 720) \
        == {"answer": "unable"}


def test_the_recorded_scores_are_pinned():
    d = json.loads((HERE / "scores.json").read_text())
    assert d["frames"] == 2261 and d["in_view"] == 535
    p = d["sources"]["perc"]
    assert p["B1_in_view_rate"] == pytest.approx(0.7963)
    assert p["B2_k3"]["rate"] == pytest.approx(0.9495) and p["B2_k5"]["rate"] == pytest.approx(0.9873)
    assert (p["B3"]["sighted"], p["B3"]["p50"], p["B3"]["p90"]) == (49, 0.12, 1.19)
    assert (p["B4_frame"]["false"], p["B4_k5"]["two_or_more"]) == (5, 1)
    assert p["B5"]["right"] == 14
    assert d["verdicts"] == {"B1": "MET", "B2": "MET", "B3": "MET", "B4": "MET", "B5": "FAILED"}


@pytest.mark.skipif(not os.environ.get("PICAR_RECORDINGS"),
                    reason="the labelled walks are not in the repo (PICAR_RECORDINGS)")
def test_the_scores_reproduce_from_the_records(tmp_path, monkeypatch):
    before = (HERE / "scores.json").read_text()
    s = _score()
    monkeypatch.setattr(s, "HERE", HERE)
    s.main()
    assert (HERE / "scores.json").read_text() == before
