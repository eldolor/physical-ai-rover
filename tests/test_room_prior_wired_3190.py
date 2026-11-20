# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.190.C, criteria 8 and 9 (amendment 3), re-scored from the rows in
`evaluations/room-prior-3190/c/` (`wired.py`; 3.116's 100 selection houses
through MissionRunner). The verdicts are pinned so a re-run that changes them
fails here."""
import json

import pytest

from tests.conftest import REPO_ROOT

C = REPO_ROOT / "evaluations" / "room-prior-3190" / "c"
FIELDS = ("outcome", "goals", "metres")


def _rows(name):
    return json.loads((C / f"{name}.json").read_text())


def _differ(a, b):
    ka = {(r["split"], r["index"], r["mode"]): r for r in _rows(a)["rows"]}
    kb = {(r["split"], r["index"], r["mode"]): r for r in _rows(b)["rows"]}
    assert set(ka) == set(kb)
    return len(ka), sum([ka[k][f] for f in FIELDS] != [kb[k][f] for f in FIELDS] for k in ka)


def test_criterion_8_off_is_off_0_of_200():
    dev, branch = _rows("off-dev"), _rows("off-branch")
    assert dev["commit"] == "098669bc" and dev["repo"].endswith("devwt")
    assert dev["commit"] != branch["commit"]
    assert all(r["mode"] in ("R", "U") for r in dev["rows"])
    assert _differ("off-dev", "off-branch") == (200, 0)


def test_criterion_9_as_written_failed_on_the_label_source():
    """The wired path (a person's names) against A's cloud arm (the stand-in
    labeller): 62 of 100 differ -- and 58 of 100 with the shipped prior on
    both sides, so the label source, not the prior's reading, differs."""
    assert _differ("harness", "wired") == (100, 62)
    assert _differ("harness-shipped", "shipped") == (100, 58)
    assert {r["room_prior"]["source"] for r in _rows("wired")["rows"]} == {"cache"}


def test_criterion_9b_the_same_labeller_0_of_100():
    assert _differ("harness-ask", "wired-ask") == (100, 0)
    rows = _rows("wired-ask")["rows"]
    assert {r["room_prior"]["source"] for r in rows} == {"cache"}
    # The rates the wired path read are 3.116.C's replies, target by target.
    seed = json.loads((REPO_ROOT / "evaluations" / "room-prior-3116" / "llm_prior.json")
                      .read_text())["us.anthropic.claude-opus-5-5"]
    for r in rows:
        assert r["room_prior"]["rates"] == seed[r["target"]]["prior"], r["index"]


@pytest.mark.parametrize("arm", ["harness", "wired", "harness-shipped", "shipped",
                                 "harness-ask", "wired-ask"])
def test_every_arm_ran_one_commit_and_no_exception(arm):
    rows = _rows(arm)
    assert rows["commit"] == _rows("off-branch")["commit"]
    assert rows["pairs"] == 100
    assert not any(r["outcome"] == "exception" for r in rows["rows"])
