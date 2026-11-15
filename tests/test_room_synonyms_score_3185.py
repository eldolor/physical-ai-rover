# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.185's bars S2-S4 and its expected effect, pinned from the sweep's recorded
arms (`evaluations/room-synonyms-3185/`, `docs/plans/ros-alignment/
3.185-room-name-synonyms.md`). Re-scored from the arms, not read from
score.json, so an edit to either shows."""

import importlib.util
import json
import os

HERE = os.path.join(os.path.dirname(__file__), "..", "evaluations", "room-synonyms-3185")
_spec = importlib.util.spec_from_file_location("score_3185", os.path.join(HERE, "score.py"))
score = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(score)


def _arms():
    return {f"{t}_{a}": score.load(t, a) for t, a in score.ARMS}


def test_bars_s2_to_s4_met():
    r = score.score(_arms())
    assert r["S2"] == {"names_differ": 0, "no_names_differ": 0, "met": True}
    assert r["S3"] == {"differ": 0, "met": True}
    assert r["S4"] == {"differ": 0, "met": True}


def test_expected_effect():
    r = score.score(_arms())
    assert r["reported"] == {"dev_syn_vs_dev_no_names": 0, "dev_syn_vs_dev_names": 175}
    s = r["summary"]
    assert s["branch_names_syn"] == s["branch_names"]
    assert s["branch_names"]["R"] == {"missions": 100, "found": 100,
                                      "goals_mean": 6.32, "metres_mean": 22.1}
    assert s["branch_dev"]["R"] == {"missions": 100, "found": 100,
                                    "goals_mean": 9.85, "metres_mean": 32.93}


def test_score_json_matches_the_arms():
    with open(os.path.join(HERE, "score.json")) as f:
        assert json.load(f) == json.loads(json.dumps(score.score(_arms())))


def test_each_arm_ran_its_own_tree_without_exceptions():
    """S2 is only a comparison if the dev arms imported another tree's brain
    (3.180), and differ() reads two failed runs as equal, so none may fail."""
    arms = _arms()
    for name, rep in arms.items():
        assert rep["brain_path"].endswith("/brain")
        assert ("plan-3.185" in rep["brain_path"]) == name.startswith("branch_"), name
        for m in ("R", "U"):
            assert rep[m]["exceptions"] == 0, (name, m)
            assert not any(r["outcome"] == "exception" for r in rep["rows"]), name
