# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.178.B: the watch list -- its rules, perception's list pass, the report's
`watch` block, and the simulator sweep's numbers (criteria B1, B2, B4's
simulator half; PLAN-ros-alignment 3.178, bars written in 3.178.A)."""
import base64
import json
import math
from pathlib import Path

import pytest

from brain import watch_list as wl
from brain.inventory import REPORT_BELIEF
from brain.perceive import (ABSENT, CLIP_LOGIT_SCALE, CROP_LOW_CONFIDENCE, DETECTED, Box,
                            Detection, OpenVocabCropSource, PerceptionPipeline)

RESULTS = Path(__file__).resolve().parents[1] / "evaluations" / "watch-list-3178" / "results.json"
IMAGE = base64.b64encode(b"not-really-a-jpeg").decode()


def frame(pan=0.0):
    return {"image_base64": IMAGE, "media_type": "image/jpeg", "image_width": 640, "pan_deg": pan}


def det(label, x1=100.0, w=50.0, conf=0.5):
    return Detection(box=Box(x1, 100.0, x1 + w, 200.0), label=label, confidence=conf)


class Detector:
    def __init__(self, detections):
        self.detections = list(detections)

    def detect(self, image, confidence):
        return list(self.detections)


class Scorer:
    """Cosines by box x1 and text: {x1: {text: cosine}}; 0.2 otherwise."""

    def __init__(self, table):
        self.table, self.calls = table, []

    def score(self, image, box, texts):
        self.calls.append((box.x1, tuple(texts)))
        row = self.table.get(box.x1, {})
        return [row.get(t, 0.2) for t in texts]


def pipe(detections, table, watch=(), **kw):
    return PerceptionPipeline(Detector(detections), Scorer(table), "red backpack",
                              crop_path=CROP_LOW_CONFIDENCE, watch=watch, **kw)


# --- the list's rules -------------------------------------------------------

def test_the_list_defaults_to_five_and_holds_at_most_eight():
    assert wl.normalise(None) == ("keys", "wallet", "phone", "glasses", "remote")
    assert wl.normalise([" Keys ", "keys", "TV  Remote"]) == ("keys", "tv remote")
    assert wl.normalise([]) == ()
    wl.normalise([f"thing {i}" for i in range(8)])
    for bad in ([f"thing {i}" for i in range(9)], ["keys", ""], ["keys", 3], "keys",
                ["x" * 41]):
        with pytest.raises(ValueError):
            wl.normalise(bad)


def test_a_label_names_an_item_by_its_head_noun_only():
    items = wl.DEFAULT_WATCH_LIST
    assert wl.item_for("tv remote", items) == "remote"
    assert wl.item_for("Keys", items) == "keys"
    assert wl.item_for("cell phone", items) == "phone"
    # 3.178.A's counting rule: these are not phones.
    assert wl.item_for("phone charger cable", items) is None
    assert wl.item_for("phone box", items) is None
    assert wl.item_for(None, items) is None


def test_the_day_s_target_is_counted_once_as_the_target():
    assert "keys" not in wl.watched(wl.DEFAULT_WATCH_LIST, "car keys")
    assert "remote" not in wl.watched(wl.DEFAULT_WATCH_LIST, "remote")
    assert wl.watched(wl.DEFAULT_WATCH_LIST, "red backpack") == wl.DEFAULT_WATCH_LIST


def test_a_sighting_needs_p_0_9_over_every_text_and_items_compete():
    items = ("keys", "remote")
    # target, one distractor, then the list
    assert wl.sighting([0.20, 0.20, 0.30, 0.20], items, CLIP_LOGIT_SCALE)[0] == "keys"
    # keys and remote both fit: neither reaches 0.9 (precision first)
    assert wl.sighting([0.20, 0.20, 0.30, 0.30], items, CLIP_LOGIT_SCALE) is None
    # 0.8 would pass the target's gate, not the list's
    p = wl.softmax([0.2, 0.2, 0.2 + math.log(17) / 100, 0.2], CLIP_LOGIT_SCALE)[2]
    assert 0.8 < p < 0.9
    assert wl.sighting([0.2, 0.2, 0.2 + math.log(17) / 100, 0.2], items, CLIP_LOGIT_SCALE) is None
    assert wl.sighting([0.2, 0.3], (), CLIP_LOGIT_SCALE) is None


# --- perception's list pass -------------------------------------------------

def test_the_target_s_probability_is_the_same_with_the_list_on():
    """3.178.A: the list's texts never enter P(target)."""
    table = {100.0: {"red backpack": 0.31, "keys": 0.33}}
    off = pipe([det("red backpack")], table).perceive(frame())
    on = pipe([det("red backpack")], table, watch=wl.DEFAULT_WATCH_LIST).perceive(frame())
    assert off.best.probability == on.best.probability
    assert off.status == on.status
    assert len(on.best.scores) == len(off.best.scores)


def test_a_list_sighting_reaches_perception_with_its_camera_bearing():
    # A box at the image's centre-right: in-frame bearing (225/640-0.5)*66.
    table = {200.0: {"keys": 0.40}}
    p = pipe([det("keys", x1=200.0)], table, watch=wl.DEFAULT_WATCH_LIST).perceive(frame(pan=30.0))
    assert p.status == ABSENT          # nothing answered for the target
    assert [d["label"] for d in p.watch] == ["keys"]
    assert p.watch[0]["bearing_deg"] == pytest.approx((225 / 640 - 0.5) * 66.0)
    assert p.watch[0]["confidence"] >= wl.WATCH_GATE


def test_a_box_proposed_for_a_list_item_never_answers_for_the_target():
    # The keys box fits "red backpack" best of all texts: still not the target's.
    table = {200.0: {"red backpack": 0.45}}
    p = pipe([det("keys", x1=200.0)], table, watch=wl.DEFAULT_WATCH_LIST).perceive(frame())
    assert p.status == ABSENT and p.best is None and p.watch == []
    off = pipe([det("keys", x1=200.0)], table).perceive(frame())
    assert off.status == DETECTED      # without a list, any box is the target's


def test_list_boxes_cannot_crowd_the_target_out_of_the_crop_cap():
    big_keys = [det("keys", x1=float(i), w=400.0) for i in range(20)]
    target = det("red backpack", x1=500.0, w=10.0)
    table = {500.0: {"red backpack": 0.40}}
    p = pipe(big_keys + [target], table, watch=wl.DEFAULT_WATCH_LIST, max_crops=4).perceive(frame())
    assert p.status == DETECTED and p.best.detection is target
    # the list's crops are capped apart: 1 target crop + 4 list crops scored
    assert len(p.candidates) == 1


def test_one_image_encode_per_crop_with_the_list_last():
    sc = Scorer({})
    PerceptionPipeline(Detector([det("red backpack")]), sc, "red backpack",
                       crop_path=CROP_LOW_CONFIDENCE, watch=["Keys", "red backpack"]).perceive(frame())
    (x1, texts), = sc.calls
    assert texts[0] == "red backpack" and texts[-1:] == ("keys",)   # the target dropped from the list


def test_without_a_list_perception_is_unchanged():
    p = pipe([det("red backpack")], {100.0: {"red backpack": 0.40}}).perceive(frame())
    assert p.status == DETECTED and p.watch == [] and p.best.watch is None


def test_the_open_vocab_prompt_carries_the_list_after_the_target():
    seen = []

    class Backend:
        def detect_text(self, image, text):
            seen.append(text)
            return []
    OpenVocabCropSource(Backend(), "red backpack").detect(b"", 0.1)
    OpenVocabCropSource(Backend(), "red backpack", ("keys", "phone")).detect(b"", 0.1)
    assert seen == ["red backpack", ["red backpack", "keys", "phone"]]


def test_a_lone_string_is_refused_never_split_into_letters():
    with pytest.raises(ValueError):
        pipe([], {}, watch="keys")


def test_a_box_labelled_with_the_target_s_coco_word_stays_the_target_s():
    """Review: target "remote control" (COCO word "remote") with "remote"
    in the list -- dropped from the list, and its "remote" boxes the target's."""
    table = {100.0: {"remote control": 0.40}}
    p = PerceptionPipeline(Detector([det("remote")]), Scorer(table), "remote control",
                           crop_path=CROP_LOW_CONFIDENCE, watch=wl.DEFAULT_WATCH_LIST).perceive(frame())
    assert p.status == DETECTED
    assert "remote" not in wl.watched(wl.DEFAULT_WATCH_LIST, "remote control")
    assert "keys" not in wl.watched(wl.DEFAULT_WATCH_LIST, "key")   # names_target's rule


def test_a_distractor_repeating_an_item_does_not_halve_it():
    table = {200.0: {"keys": 0.40}}
    p = PerceptionPipeline(Detector([det("keys", x1=200.0)]), Scorer(table), "red backpack",
                           crop_path=CROP_LOW_CONFIDENCE, distractors=("a wall", "keys"),
                           watch=["keys"]).perceive(frame())
    assert [d["label"] for d in p.watch] == ["keys"]


# --- the report's watch block -----------------------------------------------

def _lm(label, belief, x=1.0):
    return {"id": 1, "label": label, "x_m": x, "y_m": 2.0, "room": "lounge", "belief": belief,
            "hits": 2, "last_step": 7, "carried": False}


def test_the_watch_block_pins_reported_list_items_only():
    report = {"reported": [_lm("tv remote", 0.9), _lm("sofa", 0.95), _lm("keys", 0.85, x=3.0),
                           _lm("keys", REPORT_BELIEF - 0.01)]}
    block = wl.watch_block(report, wl.DEFAULT_WATCH_LIST, target="car keys")
    assert block["items"] == ["wallet", "phone", "glasses", "remote"]
    assert [p["label"] for p in block["pins"]["remote"]] == ["tv remote"]
    assert block["pinned"] == 1 and block["gate"] == wl.WATCH_GATE


def test_the_mission_report_carries_the_watch_block():
    from control.mission_runner import MissionRunner
    from sim.mock_robot import MockRobot
    from sim.maps import build_world
    from tests.conftest import mock_world_for

    def runner(**kw):
        robot = MockRobot(build_world("complex_house"), render=False)
        return MissionRunner(robot, target_object="unicorn", max_steps=2, policy="frontier",
                             world=mock_world_for(robot), **kw)
    assert runner().inventory_report()["watch"]["items"] == list(wl.DEFAULT_WATCH_LIST)
    assert "watch" not in runner(watch_list=()).inventory_report()
    with pytest.raises(ValueError):
        runner(watch_list=[f"t{i}" for i in range(9)])


def test_the_brain_config_validates_the_list(tmp_path):
    from control.brain_config import load_brain_config
    assert load_brain_config()["watch_list"] == list(wl.DEFAULT_WATCH_LIST)
    for bad in ("[a, b, c, d, e, f, g, h, i]", "keys", "null"):
        p = tmp_path / "robot.yaml"
        p.write_text(f"brain:\n  watch_list: {bad}\n")
        with pytest.raises(ValueError, match="watch_list"):
            load_brain_config(p)
    p.write_text("brain:\n  watch_list: []\n")
    assert load_brain_config(p)["watch_list"] == []
