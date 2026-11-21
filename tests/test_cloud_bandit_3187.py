# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.187.D: the learned bandit against the tree, pinned. Recomputed from
committed records only (`voi/cloud-answers.jsonl`, `voi/fixed-F*.jsonl`,
3.92.A's rows, the pinned set): no recordings, no model, no call. D0's
criteria and verdict rule are in
`docs/plans/ros-alignment/3.187-cloud-call-bandit.md`, written before the
fit; D1 records the result."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1] / "evaluations" / "cloud-bandit-3187"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope="module")
def bandit():
    """bandit.py, with this section's `common` on the path; sys.modules and
    sys.path left as found (other evaluations ship a `common` too)."""
    before = sys.modules.get("common")
    path_before = list(sys.path)
    sys.modules["common"] = _load("common", HERE / "common.py")
    try:
        yield _load("cloud_bandit_3187_bandit", HERE / "bandit.py")
    finally:
        sys.path[:] = path_before
        if before is None:
            sys.modules.pop("common", None)
        else:
            sys.modules["common"] = before


@pytest.fixture(scope="module")
def out(bandit):
    return bandit.run(bandit.voi.load())


def _t(t):
    return {k: t[k] for k in ("useful", "harmful", "net", "paid", "local", "d", "d_right", "frames")}


def test_the_context_is_d0s(out):
    """Policy-dependent features (frames since the last call) and heading
    are dropped, as D0 wrote before the fit."""
    assert out["features"] == ["detected", "logit_p", "log1p_streak", "passed_in_last_5",
                               "log1p_frames_since_start", "coco_word"]


def test_the_judged_cell_is_chosen_on_training(out):
    j = out["judged"]
    assert j["passing_on_train"] == []
    assert j["cell"] == "lambda=10,mu=0.01"
    tr = out["cells"][j["cell"]]["W/train"]
    assert (tr["net"], out["fixed"]["W/train"]["F"]["net"]) == (17, 21)
    assert max(c["W/train"]["net"] for c in out["cells"].values()) == 17


def test_held_out_w_criterion_1a_fails(out):
    v = out["cells"][out["judged"]["cell"]]["W/held_out"]
    assert _t(v) == {"useful": 30, "harmful": 6, "net": 24, "paid": 10, "local": 314,
                     "d": 12, "d_right": 5, "frames": 800}
    assert _t(out["fixed"]["W/held_out"]["F"])["net"] == 26
    c = v["criteria"]
    assert c["C1a"] is False
    assert c["C1b_i"] == "too few to judge"
    assert c["C1b_ii"] is False          # local 314 <= 423 holds; paid 10 > the tree's 0 fails it
    assert (c["C2"], c["C2_walks_checked"], c["C2_abandoned"]) == (True, 3, [])
    assert (c["C3"], c["C3_walks_checked"]) == (True, 8)
    assert c["vs_F0_3_65"] == {"useful_ge": True, "paid_30pct_fewer": True}
    assert v["sign_test_vs_tree"] == {"bandit_right_tree_wrong": 1, "tree_right_bandit_wrong": 3,
                                      "p_two_sided": 0.625}


def test_held_out_p(out):
    v = out["cells"][out["judged"]["cell"]]["P/held_out"]
    assert _t(v) == {"useful": 1, "harmful": 0, "net": 1, "paid": 16, "local": 4,
                     "d": 8, "d_right": 6, "frames": 86}
    c = v["criteria"]
    assert (c["C1a"], c["C2"], c["C2_walks_checked"], c["C3"]) == (True, True, 0, True)


def test_the_verdict_is_failed(out):
    assert out["verdict"] == "failed"


def test_criterion_3_holds_in_every_cell(out):
    for cell in out["cells"].values():
        for v in cell.values():
            assert v["criteria"]["C3"], v


def test_the_verdict_rule(bandit):
    ok = {"C1a": True, "C1b": True, "C1b_i": "too few to judge", "C2": True, "C3": True}
    assert bandit.verdict(ok, ok) == "met"
    small = dict(ok, C1b=False)
    assert bandit.verdict(small, ok) == "sample too small to tell"
    assert bandit.verdict(dict(ok, C1a=False), ok) == "failed"
    assert bandit.verdict(ok, dict(ok, C2=False)) == "failed"
    assert bandit.verdict(dict(small, C1b_i=False), ok) == "failed"


def test_voi_beside_is_gs(out):
    vb = out["voi_beside"]
    assert vb["cell"] == "lambda=50,mu=0"
    assert (vb["W/held_out"]["net"], vb["W/held_out"]["local"], vb["W/held_out"]["paid"]) == (26, 578, 0)


def test_the_committed_scores_match(out, bandit):
    """bandit-scores.json is what bandit.py writes from the committed records."""
    committed = json.loads((HERE / "bandit-scores.json").read_text())
    assert committed == json.loads(json.dumps(bandit.voi.finite(out)))
