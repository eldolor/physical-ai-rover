# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.177.A: the visual-promise scorer's labels and within-walk AUROC, and the
pinned frame set's hash (`evaluations/visual-promise-3177/`)."""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1] / "evaluations" / "visual-promise-3177"


def _load(name):
    spec = importlib.util.spec_from_file_location(f"vp3177_{name}", HERE / f"{name}.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope="module")
def mods():
    """score.py and its `common`, loaded by path; sys.modules left as found
    (other evaluations ship a `common` too)."""
    before = sys.modules.get("common")
    common = sys.modules["common"] = _load("common")
    try:
        yield {"common": common, "score": _load("score")}
    finally:
        if before is None:
            sys.modules.pop("common", None)
        else:
            sys.modules["common"] = before


def test_labels_split_before_far_and_leave_out_just_after(mods):
    v = [0] * 8 + [1, 1] + [0] * 8
    pos, neg = mods["score"].labels(v, 3)
    assert pos == [5, 6, 7]
    # 0-4 are far; 10-12 follow the sighting (left out); 13-17 are far.
    assert neg == [0, 1, 2, 3, 4, 13, 14, 15, 16, 17]


def test_labels_count_recorded_steps_not_positions(mods):
    # Frames 3 and 4 are missing: frame 2 is 3 steps before the sighting at
    # 5, not 1; and a frame within k of a missing step is not a negative.
    v, steps = [0, 0, 0, 1], [0, 1, 2, 5]
    assert mods["score"].labels(v, 2, steps) == ([], [0])
    assert mods["score"].labels(v, 3, steps) == ([2], [])
    assert mods["score"].labels([0, 0, 0, 0, 1], 2) == ([2, 3], [0])


def test_a_torn_last_line_is_dropped(mods, tmp_path):
    p = tmp_path / "r.jsonl"
    p.write_text('{"key": "a"}\n{"key": "b"}\n{"key": "c", "fu')
    assert set(mods["common"].rows(p)) == {"a", "b"}


def test_pairs_count_ties_half(mods):
    s = [0.1, 0.5, 0.5, 0.9]
    assert mods["score"].pairs(s, [3, 1], [0, 2]) == (3.5, 4)
    assert mods["score"].pairs(s, [], [0]) == (0.0, 0)


def _walk(name, object_, visible, target_score, objects):
    n = len(visible)
    m = np.zeros((n, len(objects)))
    m[:, objects.index(object_)] = target_score
    return {"walk": name, "object": object_, "visible": visible, "steps": list(range(n)),
            "own": objects.index(object_),
            "others": {o: j for j, o in enumerate(objects) if o != object_},
            "full": m, "tiles": m}


def test_evaluate_is_within_walk_and_balanced_by_object(mods):
    objects = ["a", "b"]
    v = [0] * 8 + [1, 1] + [0] * 8
    rise = np.r_[np.zeros(5), np.ones(3), np.ones(2), np.zeros(8)]
    # Walk 2's scores are all higher: pooled across walks they would mix, within walks they do not.
    w1 = _walk("w1", "a", v, rise, objects)
    w2 = _walk("w2", "b", v, -rise + 5.0, objects)
    e = mods["score"].evaluate([w1, w2], "full", 3)
    assert e["per_walk"]["w1"]["target"] == 1.0
    assert e["per_walk"]["w2"]["target"] == 0.0
    assert e["balanced_target"] == pytest.approx(0.5)
    # The other object's column is flat zero: the control is all ties, chance.
    assert e["balanced_control"] == pytest.approx(0.5)
    assert e["per_walk"]["w1"]["margin"] == 1.0


def test_set_hash_matches_its_rows(mods):
    d = json.loads((HERE / "set.json").read_text())
    assert d["n"] == len(d["frames"]) == 2261
    assert sum(r["visible"] for r in d["frames"]) == 535
    assert mods["common"].set_hash(d["frames"]) == d["hash"]
    assert d["hash"] == mods["common"].PINNED


def test_build_walks_scores_own_words_and_controls_with_first_words(mods):
    frames = [{"key": "red-1/frame-0000.jpg", "target": "burgundy backpack", "visible": False},
              {"key": "red-2/frame-0003.jpg", "target": "red backpack", "visible": False},
              {"key": "red-2/frame-0005.jpg", "target": "red backpack", "visible": True},
              {"key": "box-1/frame-0000.jpg", "target": "box", "visible": False}]
    d = {"frames": frames}
    cols = mods["common"].all_texts(d)
    rows = {f["key"]: {"texts": cols, "full": [0.0] * len(cols), "tiles": [[0.0] * len(cols)] * 3}
            for f in frames}
    walks = {w["walk"]: w for w in mods["score"].build_walks(d, rows)}
    assert walks["red-2"]["steps"] == [3, 5]
    assert cols[walks["red-2"]["own"]] == "red backpack"
    # In another object's control the bag stands for its first walk's words.
    assert {o: cols[j] for o, j in walks["box-1"]["others"].items()} == {"red backpack": "burgundy backpack"}
    rows["red-2/frame-0005.jpg"] = dict(rows["red-2/frame-0005.jpg"], texts=list(reversed(cols)))
    with pytest.raises(SystemExit):
        mods["score"].build_walks(d, rows)
