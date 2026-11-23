# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.206.A (amendment 6): the larger steering test set's draw, how a viewed
frame becomes a label, and the local order's score on it -- from committed
records; no frame is read and no model asked."""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("a_3206", ROOT / "evaluations/steer-distill-3206/a_set.py")
a = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(a)


def test_the_draw_is_the_committed_one():
    rows = a.draw()
    assert rows == json.loads(a.DRAW.read_text())["rows"] and len(rows) == 140
    keys = {f"{r['walk']}/{r['frame']}" for r in rows}
    man = a.mf.build()
    assert len(keys) == 140 and not keys & set(man["steer"])            # none of S1 or S2
    assert all(man["frames"][k]["visible"] for k in keys)               # people labelled each in view
    per = {}
    for r in rows:
        per[r["walk"]] = per.get(r["walk"], 0) + 1
    assert max(per.values()) <= a.PER_WALK


def test_a_label_from_the_extent():
    row = {"walk": "w", "frame": "f.jpg", "target": "x"}
    mid = a.label_row(row, 0.40, 0.60)
    assert (mid["truth"], mid["border"], mid["wide"], mid["centre_x_frac"]) == ("FORWARD", False, False, 0.5)
    edge = a.label_row(row, 0.30, 0.44)                                 # centre 0.37: 3.7% from the line
    assert edge["truth"] == "FORWARD" and edge["border"]
    assert a.label_row(row, 0.93, 1.00)["truth"] == "RIGHT"
    assert a.label_row(row, 0.00, 0.81)["wide"]


def test_the_local_order_reproduces_3_80_i_on_s1():
    rows = json.loads(a.S1_LABELS.read_text())["rows"]
    ans = [(a.local_order(f"{r['walk']}/{r['frame']}"), r["truth"]) for r in rows]
    assert sum(x == t for x, t in ans) == 51 and sum(x is not None for x, _ in ans) == 55


def test_a_border_frame_accepts_the_third_across_its_own_line_only():
    left = [{"key": "k", "truth": "LEFT", "centre": 0.31, "border": True}]
    assert a.tally(left, lambda k: "FORWARD")["right"] == 1
    assert a.tally(left, lambda k: "RIGHT")["wrong_side"] == 1
    assert a.tally(left, lambda k: None)["silent"] == 1
    mid = [{"key": "k", "truth": "FORWARD", "centre": 0.37, "border": True}]      # by the 1/3 line
    assert a.tally(mid, lambda k: "LEFT")["right"] == 1
    assert a.tally(mid, lambda k: "RIGHT")["wrong_side"] == 1                     # the far third
    clear = [{"key": "k", "truth": "LEFT", "centre": 0.1, "border": False}]
    assert a.tally(clear, lambda k: "FORWARD")["wrong_side"] == 1


def test_a_box_past_the_frame_is_clipped_edge_by_edge():
    row = {"key": "k", "box": [330.0, 0.0, 960.0, 448.0], "w": 840}
    assert abs(a.box_frac(row) - (330 + 840) / 2 / 840) < 1e-9
    assert a.box_frac({"key": "k", "box": None}) is None


def test_the_spot_check_draw_is_twenty_labelled_frames():
    pick = a.spot_check_draw()
    labelled = {r["i"] for r in json.loads(a.LABELS.read_text())["rows"]}
    assert len(pick) == 20 == len(set(pick)) and set(pick) <= labelled


def test_a_scores_pinned():
    """The 197 frames (S1's 60 and the 137 placed of the 140 drawn)."""
    labels = json.loads(a.LABELS.read_text())
    assert len(labels["rows"]) == 137 and len(labels["dropped"]) == 3
    out = a.scores()
    u = out["unflagged"]
    assert (u["frames"], u["right"], u["silent"], u["wrong_side"]) == (163, 139, 19, 5)
    b = out["border"]
    assert (b["frames"], b["right"]) == (34, 34)
    w, n = out["unflagged_wide"], out["unflagged_not_wide"]
    assert (w["frames"], w["wrong_side"], n["frames"], n["wrong_side"], n["silent"]) == (27, 5, 136, 0, 19)
    assert [d["local_order"] for d in out["dropped"]] == [None, None, None]
    assert out["unflagged_right_share_with_dropped_as_silent"] == round(139 / 166, 4)
    assert not out["criterion_1_stop"]
    assert not out["provisional"]


def test_the_spot_check_is_recorded_and_the_labels_stand():
    labels = json.loads(a.LABELS.read_text())
    spot = labels["spot_check"]
    assert sorted(int(i) for i in spot["marks"]) == a.spot_check_draw()
    rows = {r["i"]: r for r in labels["rows"]}
    differs = [int(i) for i, v in spot["marks"].items() if rows[int(i)]["truth"] != v]
    assert differs == spot["differs"] == []
    assert len(spot["unflagged_differs"]) <= a.SPOT_MAX_DIFFER and spot["labels_stand"]


def test_the_users_correction_to_s1_is_applied_here_only():
    labels = json.loads(a.LABELS.read_text())
    (key, fix), = labels["s1_corrections"].items()
    assert key.endswith("frame-0184.jpg") and (fix["was"], fix["truth"]) == ("RIGHT", "FORWARD")
    assert next(f for f in a.test_set() if f["key"] == key)["truth"] == "FORWARD"
    s1 = json.loads(a.S1_LABELS.read_text())["rows"]
    assert next(r for r in s1 if f"{r['walk']}/{r['frame']}" == key)["truth"] == "RIGHT"
