# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.187.G: value of information against the tree, pinned. The numbers are
recomputed from committed records only (`voi/cloud-answers.jsonl`,
`voi/fixed-F*.jsonl`, 3.92.A's rows, the pinned set): no recordings, no
model, no call. G0's bars and verdict rule are in
`docs/plans/ros-alignment/3.187-cloud-call-bandit.md`."""

import importlib.util
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
def score():
    """voi/score.py, with this section's `common` on the path; sys.modules
    and sys.path left as found (other evaluations ship a `common` too)."""
    before = sys.modules.get("common")
    path_before = list(sys.path)
    sys.modules["common"] = _load("common", HERE / "common.py")
    try:
        yield _load("cloud_bandit_3187_voi_score", HERE / "voi" / "score.py")
    finally:
        sys.path[:] = path_before
        if before is None:
            sys.modules.pop("common", None)
        else:
            sys.modules["common"] = before


@pytest.fixture(scope="module")
def out(score):
    return score.run(score.load())


def _strip(t):
    return {k: v for k, v in t.items() if k not in ("per_walk", "bars")}


def test_the_samples_are_g0s(out):
    w, p = out["samples"]["W"], out["samples"]["P"]
    assert (w["frames"], w["train"], w["held_out"]) == (2243, 1443, 800)
    assert (p["frames"], p["train"], p["held_out"]) == (386, 300, 86)


def test_the_cloud_answers_against_the_labels(out):
    a = out["cloud_accuracy"]
    assert a["P (shipped model)"] == {"frames": 386, "right": 335}
    assert a["W minus P (September, unrecorded model)"] == {"frames": 1857, "right": 1681}
    assert a["pilot frames with a walk answer too"] == {"frames": 20, "agree": 14}


def test_the_tree_on_the_held_out_walks(out):
    f = out["fixed"]
    assert _strip(f["W/held_out"]["F"]) == {
        "useful": 31, "harmful": 5, "paid": 0, "local": 605, "local_s": 764.8, "d": 12,
        "d_right": 4, "frames": 800, "net": 26, "dollars": 0.0}
    assert _strip(f["P/held_out"]["F"]) == {
        "useful": 0, "harmful": 1, "paid": 0, "local": 41, "local_s": 53.7, "d": 8,
        "d_right": 4, "frames": 86, "net": -1, "dollars": 0.0}
    assert (f["W/held_out"]["F0"]["net"], f["W/held_out"]["F0"]["paid"]) == (-5, 119)
    assert (f["W/train"]["F"]["net"], f["W/train"]["F"]["paid"]) == (21, 3)


def test_no_cell_passes_on_training_so_the_judged_cell_is_the_best_net(out):
    j = out["judged"]
    assert j["passing_on_train"] == []
    assert j["cell"] == "lambda=50,mu=0"
    assert out["cells"][j["cell"]]["W/train"]["net"] == 20     # the tree's 21: G1 fails there


def test_the_bars_at_the_judged_cell(out):
    c = out["cells"][out["judged"]["cell"]]
    w, p = c["W/held_out"], c["P/held_out"]
    assert _strip(w) == {
        "useful": 31, "harmful": 5, "paid": 0, "local": 578, "local_s": 700.8, "d": 12,
        "d_right": 4, "frames": 800, "net": 26, "dollars": 0.0}
    assert _strip(p) == {
        "useful": 0, "harmful": 1, "paid": 0, "local": 34, "local_s": 38.0, "d": 8,
        "d_right": 4, "frames": 86, "net": -1, "dollars": 0.0}
    for b in (w["bars"], p["bars"]):
        assert b["G1"] is True and b["G3"] is True and b["G4"] is True
        assert b["G2_i"] == "too few to judge"
        assert b["G2_ii"] is False and b["G2"] is False


def test_the_verdict(out):
    assert out["verdict"] == "sample too small to tell"


def test_g4_holds_in_every_cell(out):
    assert all(v[h]["bars"]["G4"] for v in out["cells"].values() for h in v)


def test_the_verdict_rule(score):
    ok = {"G1": True, "G2": True, "G2_ii": True, "G3": True}
    assert score.verdict(ok, ok, 12) == "VOI enough: C is not needed"
    assert score.verdict(dict(ok, G2=False, G2_ii=False), ok, 12) == "sample too small to tell"
    assert score.verdict(dict(ok, G2=False, G2_ii=False), ok, 20) == "C needed"
    assert score.verdict(ok, dict(ok, G3=False), 12) == "C needed"
    assert score.verdict(dict(ok, G1=False), ok, 30) == "C needed"
    # A failed G1 or G3 is "C needed" even when D is too small to judge G2.
    no_g2 = dict(ok, G2=False, G2_ii=False)
    assert score.verdict(dict(no_g2, G3=False), ok, 12) == "C needed"
    assert score.verdict(no_g2, dict(ok, G1=False), 12) == "C needed"


def test_branch_i_is_too_few_under_twenty(score):
    f = {"net": 0, "d": 19, "local": 100, "paid": 0, "per_walk": {}}
    v = {"net": 0, "d": 19, "d_right": 19, "local": 100, "paid": 0, "per_walk": {}}
    assert score.bars(v, f)["G2_i"] == "too few to judge"
    f["d"] = v["d"] = 20
    assert score.bars(v, f)["G2_i"] is True
    v["paid"] = 11         # paid calls over 10% of the tree's local questions
    assert score.bars(v, f)["G2_i"] is False
    v["paid"] = 0
    v["local"] = 70        # 30% fewer, no more paid
    assert score.bars(v, f)["G2_ii"] is True
