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


def _frame(walk, i, visible, perc=None, target="blue bottle"):
    return {"key": f"{walk}/frame-{i:04d}.jpg", "walk": walk, "target": target,
            "visible": visible, "perc": perc, "both": perc, "local": None}


def test_windows_and_first_sightings_follow_the_written_bars():
    s = _score()
    # walk a: 4 in view (sighted on the 2nd only), then 5 out of view with two
    # false sightings; walk b: target never in view, never sighted.
    a = [_frame("a", 0, True), _frame("a", 1, True, perc=3.3), _frame("a", 2, True),
         _frame("a", 3, True), *(_frame("a", 4 + i, False, perc=(1.0 if i in (1, 3) else None))
                                 for i in range(5))]
    b = [_frame("b", i, False) for i in range(3)]
    labels = {"a/frame-0001.jpg": {"key": "a/frame-0001.jpg", "set": "S1", "centre": 0.55}}
    r = s.score(a + b, labels)["sources"]["perc"]
    assert r["B1_in_view_rate"] == 0.25
    # k = 3 in-view windows of walk a: [0-2], [1-3] -> both hold frame 1
    assert (r["B2_k3"]["windows"], r["B2_k3"]["with_sighting"]) == (2, 2)
    assert r["B2_k5"]["windows"] == 0
    # 0.55 -> 0.05 * 66 = 3.3 deg: no error
    assert r["B3"]["sighted"] == 1 and r["B3"]["p50"] == 0.0
    assert (r["B4_frame"]["false"], r["B4_frame"]["of"]) == (2, 8)
    # out-of-view k = 5 windows: walk a's frames 4-8 only (b has 3 frames)
    assert (r["B4_k5"]["windows"], r["B4_k5"]["two_or_more"]) == (1, 1)
    # a's first sighting is the target, b has none: a miss by B5's wording
    assert r["B5"]["walks"]["a"]["right"] is True and r["B5"]["walks"]["b"]["right"] is False
    assert r["B5"]["right"] == 1


def test_an_unlabelled_first_sighting_of_perceptions_is_refused():
    s = _score()
    with pytest.raises(ValueError):
        s.score([_frame("a", 0, True, perc=1.0)], {})


@pytest.mark.skipif(not os.environ.get("PICAR_RECORDINGS"),
                    reason="the labelled walks are not in the repo (PICAR_RECORDINGS)")
def test_the_scores_reproduce_from_the_records():
    s = _score()
    assert s.score(s.build_frames(), s.load_labels()) == json.loads((HERE / "scores.json").read_text())
