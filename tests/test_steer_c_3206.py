# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.206.C (amendments 4-6): how two recorded boxes become a kept training
label, the conversion to the student's pixels, and the committed result --
no model is asked and no frame read."""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("c_3206", ROOT / "evaluations/steer-distill-3206/c_labels.py")
c = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(c)


def test_a_box_is_clipped_to_the_frame_edge_by_edge():
    assert c.frac_box([420, 112, 840, 448], 840, 448) == [0.5, 0.25, 1.0, 1.0]
    assert c.frac_box([800, 100, 952, 448], 840, 448) == [0.9524, 0.2232, 1.0, 1.0]
    assert c.frac_box(None, 840, 448) is None
    assert c.frac_box([900, 0, 950, 10], 840, 448) is None              # nothing left in the frame


def test_iou():
    assert c.iou([0, 0, 0.5, 0.5], [0, 0, 0.5, 0.5]) == 1.0
    assert c.iou([0, 0, 0.5, 0.5], [0.5, 0.5, 1, 1]) == 0.0
    assert abs(c.iou([0, 0, 0.5, 1], [0.25, 0, 0.75, 1]) - 1 / 3) < 1e-9


def test_the_label_is_in_the_students_pixels():
    assert c.model_size(1280, 720) == (840, 448) and c.model_size(640, 480) == (644, 476)
    assert c.student_box([0.25, 0.5, 0.75, 1.0], (1280, 720)) == [210, 224, 630, 448]
    assert c.student_box([0.25, 0.5, 0.75, 1.0], (640, 480)) == [161, 238, 483, 476]
    frac = c.exact_frac([100, 50, 400, 300], 840, 448)
    assert c.student_box(frac, (1280, 720)) == [100, 50, 400, 300]      # the teacher's own pixels back


def test_the_bar_is_read_on_the_unrounded_overlap():
    size = [1000, 1000]
    w, h = c.model_size(*size)
    t72 = {"k": {"size": size, "box": [0, 0, w, h]}}
    assert c.judge("k", t72, {"k": [0, 0, 1000, 500]})["kept"]                     # exactly 0.5
    under = c.judge("k", t72, {"k": [0, 0, 1000, 499.96]})                         # 0.49996
    assert under["iou"] == 0.5 and not under["kept"] and under["why_not"] == "IoU under 0.5"


def test_a_label_with_no_area_in_pixels_is_not_kept():
    size = [1280, 720]
    thin = {"k": {"size": size, "box": [100.0, 50.0, 100.4, 80.0]}}
    out = c.judge("k", thin, {"k": [119.0, 111.6, 119.5, 178.6]})
    assert not out["kept"] and "no area" in out["why_not"]


def test_a_frame_the_235b_never_answered_is_an_error_not_a_no():
    import pytest
    size = [1280, 720]
    with pytest.raises(KeyError):
        c.judge("k", {"k": {"size": size, "box": [1, 1, 9, 9]}}, {})


def test_a_teacher_box_past_the_image_is_flagged():
    size = [1280, 720]
    out = c.judge("k", {"k": {"size": size, "box": [390.0, 0.0, 840.0, 476.0]}}, {"k": [470, 0, 1000, 1000]})
    assert out["teacher_past_the_image"] and out["teacher"][3] == 1.0


def test_the_check_keeps_a_label_only_when_both_boxes_overlap():
    size = [1280, 720]
    w, h = c.model_size(*size)
    t72 = {"k": {"size": size, "box": [0.4 * w, 0.4 * h, 0.6 * w, 0.6 * h]}}
    assert c.judge("k", t72, {"k": [400, 400, 600, 600]})["kept"]
    assert c.judge("k", t72, {"k": [700, 400, 900, 600]})["why_not"] == "IoU under 0.5"
    assert c.judge("k", t72, {"k": None})["why_not"] == "no 235B box"
    assert c.judge("k", {"k": {"size": size, "box": None}}, {"k": [400, 400, 600, 600]})["why_not"] == "no 72B box"
    assert c.judge("k", t72, {"k": [700, 400, 900, 600]})["label"] is None


def test_no_test_frame_is_a_training_frame():
    rows = json.loads(c.LABELS.read_text())["rows"]
    keys = {r["key"] for r in rows}
    assert len(keys) == len(rows) == 335 and not keys & c.test_keys()
    assert len(c.test_keys()) == 200


def test_c_pinned():
    """The committed labels and criterion 2, rebuilt from committed records."""
    labels, scores = c.build()
    assert labels == json.loads(c.LABELS.read_text())
    assert scores == json.loads(c.SCORES.read_text())
    assert (scores["training_frames"], scores["kept"]) == (335, 314)
    assert scores["not_kept"] == {"IoU under 0.5": 8, "no 235B box": 8, "no 72B box": 5}
    c2 = scores["criterion_2_new_unflagged"]
    assert (c2["frames"], c2["kept"], c2["kept_right"], c2["wrong"]) == (118, 106, 106, [])
    assert scores["criterion_2_met"]
    assert len(scores["audit"]["keys"]) == 50
    assert scores["kept_with_a_clipped_teacher_box"] == [
        "woven-laundry-basket-20260913-115703/frame-0193.jpg"]
    kept = {r["key"] for r in labels["rows"] if r["kept"]}
    assert set(scores["audit"]["keys"]) <= kept
