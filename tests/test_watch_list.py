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


# --- 3.178.D: a pin needs two looks voting its own item and no dispute -------

ITEMS = wl.DEFAULT_WATCH_LIST


def _voted(label, votes, belief=0.9, x=1.0, disputed=False):
    return dict(_lm(label, belief, x=x), votes=votes, disputed=disputed)


def test_two_different_false_list_labels_make_no_pin():
    # 3.178.B's search mission 1: a false "keys" and a false "glasses" fused
    # on one wall spot, reported as "keys".
    assert not wl.pin_ok(_voted("keys", {"keys": 1, "glasses": 1}), "keys", ITEMS)


def test_a_pin_needs_two_votes_for_its_own_item():
    assert not wl.pin_ok(_voted("keys", {"keys": 1}), "keys", ITEMS)
    assert wl.pin_ok(_voted("keys", {"keys": 2}), "keys", ITEMS)
    assert not wl.pin_ok(_voted("keys", {}), "keys", ITEMS)
    assert not wl.pin_ok(_lm("keys", 0.9), "keys", ITEMS)          # no votes at all


def test_labels_naming_one_item_vote_together_and_others_dispute():
    # "tv remote" and "remote" agree; a "keys" look against a remote disputes.
    assert wl.pin_ok(_voted("tv remote", {"tv remote": 1, "remote": 1}), "remote", ITEMS)
    assert wl.pin_ok(_voted("remote", {"remote": 3, "keys": 1}), "remote", ITEMS)   # 1/4 < 1/3
    assert not wl.pin_ok(_voted("remote", {"remote": 2, "keys": 1}), "remote", ITEMS)  # 1/3
    # A label naming no item disputes as its own group, at the inventory's share.
    assert not wl.pin_ok(_voted("keys", {"keys": 2, "coin": 1}), "keys", ITEMS)
    assert wl.pin_ok(_voted("keys", {"keys": 5, "coin": 1, "cup": 1}), "keys", ITEMS)
    # The item must be the most voted group even when the label says it.
    assert not wl.pin_ok(_voted("keys", {"keys": 2, "glasses": 2}), "keys", ITEMS)


def test_carried_float_votes_count_and_bad_votes_are_ignored():
    assert wl.pin_ok(_voted("keys", {"keys": 1.6, "glasses": 0.2}), "keys", ITEMS) is False  # 1.6 < 2
    assert wl.pin_ok(_voted("keys", {"keys": 2.4, "glasses": 0.2}), "keys", ITEMS)
    assert wl.pin_ok(_voted("keys", {"keys": 2, "x": "nan", "y": None, "z": -3,
                                     "w": float("inf")}), "keys", ITEMS)


def test_the_watch_block_scores_both_rules():
    report = {"reported": [_voted("keys", {"keys": 1, "glasses": 1}, x=5.0),
                           _voted("keys", {"keys": 3}, x=1.0),
                           _voted("phone", {"phone": 1}, x=2.0)]}
    label = wl.watch_block(report, ITEMS, rule="label")
    votes = wl.watch_block(report, ITEMS, rule="votes")
    assert [p["x_m"] for p in label["pins"]["keys"]] == [5.0, 1.0]
    assert [p["x_m"] for p in votes["pins"]["keys"]] == [1.0] and votes["pins"]["phone"] == []
    assert (label["held"], votes["held"], votes["pinned"]) == (0, 2, 1)
    assert votes["pins"]["keys"][0]["votes"] == {"keys": 3}
    with pytest.raises(ValueError):
        wl.watch_block(report, ITEMS, rule="belief")


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


def test_the_brain_config_agrees_with_normalise_and_never_imports_brain():
    """Thermos (review): the walks Lambda ships control/ without brain/."""
    import subprocess
    import sys
    from control.brain_config import _watch_list
    import itertools
    words = ["Keys", " keys ", "TV  Remote", "phone", "x" * 40, "WALLET", "a\tb", "glasses"]
    for n in range(len(words) + 1):
        for case in itertools.combinations(words, n):
            assert tuple(_watch_list(list(case))) == wl.normalise(list(case))
    assert tuple(_watch_list([f"t{i}" for i in range(8)])) == wl.normalise([f"t{i}" for i in range(8)])
    for bad in ([f"t{i}" for i in range(9)], ["", "keys"], [" "], [3], ["x" * 41]):
        with pytest.raises(ValueError):
            _watch_list(bad)
        with pytest.raises(ValueError):
            wl.normalise(bad)
    assert (wl.MAX_ITEMS, wl.MAX_ITEM_CHARS) == (8, 40)
    code = ("import sys; sys.modules['brain'] = None\n"
            "from control.brain_config import load_brain_config\n"
            "assert isinstance(load_brain_config()['watch_list'], list)")
    subprocess.run([sys.executable, "-c", code], check=True,
                   cwd=Path(__file__).resolve().parents[1])


# --- the sweep's numbers (3.178.B; bars written in 3.178.A) ------------------

def test_the_sweep_s_numbers_are_pinned():
    r = json.loads(RESULTS.read_text())
    search, present = r["search"], r["present"]
    assert (search["missions"], present["missions"]) == (20, 20)
    assert r["list"] == list(wl.DEFAULT_WATCH_LIST) and r["list_false_rate"] == 0.05
    # B1, met on a thin sample: 4 of 4 passes pinned within 0.5 m (bar 90%).
    assert (search["passes"], search["found"], present["passes"], present["found"]) == (3, 3, 1, 1)
    # B2, FAILED: 6 wrong pins in 40 missions (bar <= 0.1 a mission).
    assert (search["wrong_pins"], present["wrong_pins"]) == (4, 2)
    assert (search["wrong_pins"] + present["wrong_pins"]) / 40 > 0.1
    # B4's simulator half: every mission's actions and outcome equal (met,
    # and true by construction: no policy reads the inventory). The arrival
    # comparison is UNINFORMATIVE, not met: no mission arrived either way.
    assert (search["same_as_off"], present["same_as_off"]) == (20, 20)
    assert (present["arrived_on"], present["arrived_off"]) == (0, 0)


# --- 3.178.D's fresh sweep (seeds 4178/4179; bars written before the run) ----

RESULTS_D = RESULTS.with_name("results-d.json")
RESULTS_D_BSEED = RESULTS.with_name("results-d-bseed.json")


def test_d_s_fresh_sweep_numbers_are_pinned():
    r = json.loads(RESULTS_D.read_text())
    search, present = r["search"], r["present"]
    assert (search["seed"], present["seed"]) == (4178, 4179)
    assert (search["missions"], present["missions"], search["max_steps"]) == (20, 20, 150)
    assert r["list"] == list(wl.DEFAULT_WATCH_LIST) and r["list_false_rate"] == 0.05
    lab = (search["rules"]["label"], present["rules"]["label"])
    vot = (search["rules"]["votes"], present["rules"]["votes"])
    # D1, met: D's rule, 1 + 0 wrong pins (<= 2 a sweep, <= 4 over 40);
    # dev's rule on the same reports, 6 + 2.
    assert [v["wrong_pins"] for v in vot] == [1, 0]
    assert [v["wrong_pins"] for v in lab] == [6, 2]
    # D2, met and judged (dev's true pins >= 3): 4 vs 4.
    assert [v["true_pins"] for v in vot] == [v["true_pins"] for v in lab] == [2, 2]
    # D3, FAILED: 4 of 5 passes pinned (bar 90%), the same under dev's rule.
    assert (search["passes"], present["passes"]) == (3, 2)
    assert [v["found"] for v in vot] == [v["found"] for v in lab] == [2, 2]
    assert sum(v["found"] for v in vot) / (search["passes"] + present["passes"]) < 0.9
    # D4, met: the robot is the same with the list on and off.
    assert (search["same_as_off"], present["same_as_off"]) == (20, 20)
    # Not shipped: the watch block's default stays 3.178.A's rule.
    import inspect
    assert inspect.signature(wl.watch_block).parameters["rule"].default == "label"


def test_d_s_run_on_b_s_seeds_reproduces_b():
    r = json.loads(RESULTS_D_BSEED.read_text())
    lab = [r[k]["rules"]["label"] for k in ("search", "present")]
    vot = [r[k]["rules"]["votes"] for k in ("search", "present")]
    assert [v["wrong_pins"] for v in lab] == [4, 2] and [v["found"] for v in lab] == [3, 1]
    assert [v["wrong_pins"] for v in vot] == [1, 1] and [v["true_pins"] for v in vot] == [3, 1]


def test_pin_ok_groups_by_item_where_landmark_disputed_does_not():
    # Thermos (3.178.D): `Landmark.disputed` compares raw labels, `pin_ok`
    # groups them by item, so the two can disagree on one landmark. Pinned
    # so a step that ships the votes rule decides it knowingly.
    lm = _voted("remote", {"remote": 2, "tv remote": 1}, disputed=True)   # raw split 1/3
    assert wl.pin_ok(lm, "remote", ITEMS)


def test_the_default_block_adds_only_rule_held_and_pin_votes():
    # The shipped payload gains `rule`, `held` (0 under "label") and each
    # pin's `votes`; its pins are 3.178.A's.
    report = {"reported": [_voted("keys", {"keys": 1, "glasses": 1})]}
    block = wl.watch_block(report, ITEMS)
    assert (block["rule"], block["held"], block["pinned"]) == ("label", 0, 1)
    assert block["pins"]["keys"][0]["votes"] == {"keys": 1, "glasses": 1}


# --- 3.178.E: carried pins' scaled votes ---------------------------------------

def _carried_through_seed(votes, belief=0.9):
    """One landmark carried by the real `Inventory.seed`, as its next report
    shows it (votes scaled to the carry cap)."""
    from brain.inventory import Inventory
    saved = {"frame_id": "f1", "reported": [dict(_voted("keys", votes, belief=belief))]}
    inv = Inventory()
    assert inv.seed("f1", saved) == 1
    (lm,) = inv.report()["reported"]
    return lm


def test_a_carried_pin_whose_votes_were_scaled_stays_a_pin():
    # D's recorded gap: {keys 3, 30 other labels 1 each} was a pin (3 votes,
    # runner-up 1, share 1/4); carried, the cap scales keys to 1.82.
    votes = {"keys": 3, **{f"thing {k}": 1 for k in range(30)}}
    assert wl.pin_ok(_voted("keys", votes), "keys", ITEMS)
    lm = _carried_through_seed(votes)
    assert lm["carried"] and lm["votes"]["keys"] < wl.PIN_MIN_VOTES
    assert wl.pin_ok(lm, "keys", ITEMS)
    assert wl.watch_block({"reported": [lm]}, ITEMS, rule="votes")["pinned"] == 1


def test_a_capped_carried_landmark_is_still_judged_on_its_dispute():
    # Shares survive the scaling, so a disputed landmark stays refused.
    lm = _carried_through_seed({"keys": 12, "glasses": 10})
    assert lm["carried"] and not wl.pin_ok(lm, "keys", ITEMS)
    # A carried landmark under the cap is not scaled: the vote count holds.
    small = _carried_through_seed({"keys": 1})
    assert small["votes"] == {"keys": 1} and not wl.pin_ok(small, "keys", ITEMS)
    assert wl.pin_ok(_carried_through_seed({"keys": 2}), "keys", ITEMS)
    # Not carried: the share alone never stands in for two looks.
    assert not wl.pin_ok(dict(lm, carried=False, votes={"keys": 1.5}), "keys", ITEMS)


# --- 3.178.E's fresh sweep (seeds 5178/5179; bars written before the run) ----

RESULTS_E = RESULTS.with_name("results-e.json")


def test_e_s_fresh_sweep_numbers_are_pinned():
    r = json.loads(RESULTS_E.read_text())
    search, present = r["search"], r["present"]
    assert (search["seed"], present["seed"]) == (5178, 5179)
    assert (search["missions"], present["missions"], search["max_steps"]) == (20, 20, 150)
    assert r["list_false_rate"] == 0.05 and len(r["placed"]) == 9
    lab = (search["rules"]["label"], present["rules"]["label"])
    vot = (search["rules"]["votes"], present["rules"]["votes"])
    passes = search["passes"] + present["passes"]
    # Sample bar, met: 50 passes (>= 30).
    assert (search["passes"], present["passes"]) == (24, 26) and passes >= 30
    # D1, met: 2 + 1 wrong pins under D's rule (dev's 5 + 5).
    assert [v["wrong_pins"] for v in vot] == [2, 1]
    assert [v["wrong_pins"] for v in lab] == [5, 5]
    # D2, met: 45 vs 48 true pins, margin max(1, 4.8).
    assert [v["true_pins"] for v in vot] == [24, 21] and [v["true_pins"] for v in lab] == [24, 24]
    # D3, FAILED: 41 of 50 (82%); dev's rule 44 of 50. Over passes that
    # reached a report (information only): 41 of 44.
    assert [v["found"] for v in vot] == [22, 19] and [v["found"] for v in lab] == [22, 22]
    assert sum(v["found"] for v in vot) / passes < 0.9
    assert search["reached"] + present["reached"] == 44
    # D4, met; no landmark was carried (the `_capped` fix moves no number).
    assert (search["same_as_off"], present["same_as_off"]) == (20, 20)
    assert search["carried"] == present["carried"] == 0
    # Not shipped: the watch block's default stays 3.178.A's rule.
    import inspect
    assert inspect.signature(wl.watch_block).parameters["rule"].default == "label"


# --- 3.178.B's frame half: the list through perception_eval (B3, B4) --------

def test_a_list_sighting_carries_its_box_for_the_by_eye_check():
    """B3 checks every sighting's crop by eye, so the frame's sighting names
    the box it came from (in pixels)."""
    table = {200.0: {"keys": 0.40}}
    p = pipe([det("keys", x1=200.0)], table, watch=wl.DEFAULT_WATCH_LIST).perceive(frame())
    assert p.watch[0]["box"] == [200.0, 100.0, 250.0, 200.0]


def test_the_lab_builds_the_shipped_crop_source_prompted_with_the_list(monkeypatch):
    """`perception_eval --detector crops:yoloe --watch ...` is the shipped
    shape (`pipeline_for(watch=)`): the prompt and the pipeline both carry
    the list, less an item naming the target."""
    from brain import perceive_lab

    class Backend:
        weights = "fake"

    class FakeClip:
        def __init__(self, *a, **k):
            pass
    monkeypatch.setattr(perceive_lab, "_open_vocab", lambda *a, **k: Backend())
    monkeypatch.setattr(perceive_lab, "ClipScorer", FakeClip)
    p = perceive_lab.pipeline_for_spec("a phone", detector="crops:yoloe",
                                       watch=wl.DEFAULT_WATCH_LIST)
    assert p.watch == ("keys", "wallet", "glasses", "remote")
    assert p.detector.watch == p.watch
    off = perceive_lab.pipeline_for_spec("a phone", detector="crops:yoloe")
    assert off.watch == () and off.detector.watch == ()


def test_the_lab_refuses_a_list_on_a_detector_it_cannot_prompt(monkeypatch):
    """Only a backend whose detect_text takes a class list (review: OWLv2 and
    Grounding DINO take one string and would fail on every frame)."""
    from brain import perceive_lab
    for spec in ("none", "yoloe-11s-seg.pt", "crops:owlv2", "crops:gdino", "owlv2"):
        with pytest.raises(ValueError, match="watch list"):
            perceive_lab.pipeline_for_spec("a phone", detector=spec, watch=["keys"])
    assert perceive_lab.watch_capable("crops:yoloe:yoloe-11s-seg.pt")
    assert perceive_lab.watch_capable("crops:yoloworld")
    class FakeClip:
        def __init__(self, *a, **k):
            pass
    monkeypatch.setattr(perceive_lab, "ClipScorer", FakeClip)   # no model, no download
    # A list emptied by the target (counted once, as the target) is list off,
    # on any detector; a lone string is refused as one.
    assert perceive_lab.pipeline_for_spec("phone", detector="none", watch=["phone"]).watch == ()
    with pytest.raises(ValueError, match="one string"):
        perceive_lab.pipeline_for_spec("a phone", detector="none", watch="keys")


# --- B3 and B4's frame half on the 2,261 pinned frames (3.178.B, 2026-11-19) -

RESULTS_FRAMES = RESULTS.with_name("results-frames.json")


def test_the_frame_half_s_numbers_are_pinned():
    r = json.loads(RESULTS_FRAMES.read_text())
    s = r["frame_set"]
    assert (s["hash"], s["frames"], s["visible"], s["walks"]) == ("00d12dbc44f6", 2261, 535, 21)
    assert r["config"]["detector"] == "crops:yoloe" and r["config"]["clip"] == "RN50"
    assert r["config"]["watch"] == list(wl.DEFAULT_WATCH_LIST)
    # B4's frame half, FAILED on TP: 413 on vs 422 off at 0.8 (bar: >= 417);
    # FP 5 vs 5 (bar: no more than off) met.
    b4 = r["B4"]
    assert (b4["gate"], b4["tp_off"], b4["tp_on"], b4["fp_off"], b4["fp_on"]) == (0.8, 422, 413, 5, 5)
    assert b4["tp_on"] < b4["tp_off"] - 5 and b4["fp_on"] <= b4["fp_off"]
    assert b4["verdict"] == "FAILED"
    # B3, met: no sighting reached P >= 0.9, so 0 wrong frames and 0 walks
    # (bars <= 2 and <= 2). The list ran: 624 crops were its own, the best
    # list probability 0.886.
    b3 = r["B3"]
    assert (b3["gate"], b3["sightings"], b3["wrong_frames"], b3["wrong_walks"]) == (wl.WATCH_GATE, 0, 0, 0)
    assert b3["verdict"] == "met"
    diag = r["diagnostic_list_crops"]
    assert diag["list_only_crops"] == 624 and diag["at_or_above"]["0.9"] == 0
    assert r["cpu_timing"]["device"].startswith("CPU")
