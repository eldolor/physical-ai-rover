# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
3.180.A's sweep, pinned: the paired arms (`evaluations/explore-view-3180/
runs/`: `base-*` from a `dev` 2c1041ef worktree, run from inside it; `a-*`
from this branch) re-scored by `score.py` must give the recorded score, each
bar's verdict as it came out (A6 FAILED stays recorded as failed), and S7's
three traced missions as measured (A1).
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNS = ROOT / "evaluations" / "explore-view-3180" / "runs" / "am1"   # amendment 1
FIRST = ROOT / "evaluations" / "explore-view-3180" / "runs" / "first"   # the first run
sys.path.insert(0, str(ROOT / "evaluations" / "explore-view-3180"))
import score                                                    # noqa: E402

VERDICTS = {"A2": True, "A3": True, "A4": True, "A5": True, "A6": False}


def _rows(name):
    return {(r["index"], r["mode"]): r
            for r in json.loads((RUNS / f"{name}.json").read_text())["rows"]}


def test_the_recorded_score_is_what_the_arms_score(tmp_path):
    for runs in (RUNS, FIRST):
        out = tmp_path / "score.json"
        score.main(str(runs), str(out))
        assert json.loads(out.read_text()) == json.loads((runs / "score.json").read_text()), runs


def test_each_bar_is_pinned_as_it_came_out():
    """Both runs: A6 FAILED in the first and again after amendment 1."""
    for runs in (RUNS, FIRST):
        rec = json.loads((runs / "score.json").read_text())
        assert {k: rec[k]["met"] for k in VERDICTS} == VERDICTS, runs
    assert json.loads((FIRST / "score.json").read_text())["A6"]["max"] == 9
    assert json.loads((RUNS / "score.json").read_text())["A6"]["max"] == 9


def test_the_baseline_ran_devs_code():
    """The first baseline ran the fix ("." on the path); this one did not:
    a brain before 3.180.A has no `view_reuses`."""
    for arm in ("dev", "none", "right", "wrong"):
        assert all(r.get("view_reuses") is None for r in _rows(f"base-{arm}").values()), arm
        assert any(r.get("view_reuses") for r in _rows(f"a-{arm}").values()), arm


def test_a1_s7s_three_now_find():
    for arm, key, base_goals, a_goals in (("right", (195, "R"), 39, 7),
                                          ("right", (194, "U"), 22, 10),
                                          ("wrong", (247, "R"), 32, 39)):
        b, a = _rows(f"base-{arm}")[key], _rows(f"a-{arm}")[key]
        assert (b["outcome"], b["goals"]) == ("exhausted", base_goals), key
        assert (a["outcome"], a["goals"]) == ("found", a_goals), key


def test_3176_s6_holds_against_real_dev():
    """3.176.B's S6 correction (found here): its `dev` arm ran B's brain;
    re-run from inside a d9104b97 worktree, real `dev` equals B's
    `nolabels` rows, 0 of 200."""
    sim = ROOT / "evaluations" / "request-3176" / "sim"
    load = lambda p: {(r["index"], r["mode"]): r for r in json.loads(p.read_text())["rows"]}
    real, nolabels = load(sim / "dev-real-d9104b97.json"), load(sim / "nolabels.json")
    f = ("outcome", "goals", "metres")
    assert len(real) == 200
    assert [k for k in real if [real[k][x] for x in f] != [nolabels[k][x] for x in f]] == []
