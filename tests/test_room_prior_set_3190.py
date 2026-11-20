# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
3.190.A, pinned: B's 40 fresh targets (fixed before any call, amendment 1),
the wrong arm's tie rule, and criteria 1-3's numbers. The three arms' rows
are kept in `evaluations/room-prior-3190/a/`; this re-scores them with
`arms.score` and checks the result is the recorded `a/score.json`, so a
re-run that moves a number is seen. Each verdict is pinned as it came out:
a failed bar stays recorded as failed.
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HERE = ROOT / "evaluations" / "room-prior-3190"
OUT = HERE / "a"
sys.path.insert(0, str(HERE))
import arms                                                     # noqa: E402
import targets                                                  # noqa: E402

TYPES = ("kitchen", "living room", "bedroom", "bathroom")

# Amendment 1, verbatim: B's set may not change once a call is made.
EXPECTED = {
    "spatula": "kitchen", "cutting board": "kitchen", "oven mitt": "kitchen",
    "dish soap": "kitchen", "kettle": "kitchen",
    "toilet paper": "bathroom", "shampoo bottle": "bathroom", "bath mat": "bathroom",
    "toothpaste": "bathroom", "razor": "bathroom",
    "alarm clock": "bedroom", "pajamas": "bedroom", "jewelry box": "bedroom",
    "duvet": "bedroom", "slippers": "bedroom",
    "magazine rack": "living room", "board game": "living room",
    "piano bench": "living room", "dvd case": "living room",
    "fireplace poker": "living room",
}
OPEN = ("coffee mug", "scissors", "tape measure", "flashlight", "notebook", "pen",
        "sewing kit", "wristwatch", "tablet", "baseball cap", "jacket", "dog leash",
        "candle", "tissue box", "blanket", "hairbrush", "medicine bottle", "batteries",
        "stuffed animal", "spray bottle")

# Criteria 1-3 as measured (the section file, "A results").
VERDICTS = {"criterion_1": True, "criterion_2": True, "criterion_3": True}


def test_the_forty_targets_are_the_pinned_set():
    assert targets.EXPECTED == EXPECTED
    assert targets.OPEN == OPEN
    assert targets.TARGETS == tuple(EXPECTED) + OPEN
    assert len(targets.TARGETS) == len(set(targets.TARGETS)) == 40
    assert len(EXPECTED) == 20 and set(EXPECTED.values()) == set(TYPES)
    assert all(sum(v == t for v in EXPECTED.values()) == 5 for t in TYPES)
    assert all(t == t.strip().lower() for t in targets.TARGETS)


def test_no_target_is_shipped_or_already_asked():
    shipped = json.loads((ROOT / "brain" / "explore_goal_rooms.json").read_text())["prior"]
    asked = set(arms.PROCTHOR) | set(sys.modules["llm_prior"].EVERYDAY)
    assert len(asked) == 29
    assert not set(targets.TARGETS) & (set(shipped) | asked)


def test_the_wrong_arm_swaps_top_and_bottom_with_ties_in_key_order():
    assert arms.swap({"kitchen": 0.2, "living room": 0.9, "bedroom": 0.5, "bathroom": 0.1}) == \
        {"kitchen": 0.2, "living room": 0.1, "bedroom": 0.5, "bathroom": 0.9}
    # garbage can's tie (amendment 1): the first maximum is kitchen.
    gc = arms.cloud_prior()["garbage can"]
    assert arms.swap(gc) == {"kitchen": 0.30, "living room": 0.85, "bedroom": 0.50,
                             "bathroom": 0.85}
    assert arms.arm_prior("none") == {}
    assert set(arms.arm_prior("cloud")) == set(arms.PROCTHOR)


def test_the_recorded_score_is_what_the_rows_score():
    rows = {a: json.loads((OUT / f"{a}-R.json").read_text()) for a in arms.ARMS}
    assert arms.score(rows) == json.loads((OUT / "score.json").read_text())


def test_each_bar_is_pinned_as_it_came_out():
    rec = json.loads((OUT / "score.json").read_text())
    assert {k: rec[k]["met"] for k in VERDICTS} == VERDICTS
    assert all(rec["summary"][a]["n"] == 583 and rec["summary"][a]["exceptions"] == 0
               for a in arms.ARMS)
