# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
3.190.F, pinned: a cloud room prior weighted more in goal choice, swept on
3.190.A's free harness (`evaluations/room-prior-3190/f/weight.py`). The
bonus's arithmetic; the selection on the 100 selection houses and the bars
on the 588 test houses re-scored from the kept rows and checked against the
recorded `select.json` / `score.json`; each verdict pinned as it came out
(a failed bar stays recorded as failed).
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
F = ROOT / "evaluations" / "room-prior-3190" / "f"
sys.path.insert(0, str(F))
import weight                                                   # noqa: E402

SELECT = F / "select"
TEST = F / "test"


def test_the_bonus_is_relative_to_the_top_rate_and_zero_at_w0():
    rates = {"kitchen": 0.8, "living room": 0.2, "bedroom": 0.1, "bathroom": 0.0}
    col = np.array([0.8, 0.2, 0.0])
    assert weight.bonus(col, rates, 0.0).tolist() == [0.0, 0.0, 0.0]
    assert weight.bonus(col, rates, 2.0).tolist() == pytest.approx([2.0, 0.5, 0.0])
    # No rates, or none above 0: no bonus (the none arm is untouched).
    assert weight.bonus(col, {}, 8.0).tolist() == [0.0, 0.0, 0.0]
    assert weight.bonus(col, None, 8.0).tolist() == [0.0, 0.0, 0.0]
    assert weight.bonus(col, dict.fromkeys(rates, 0.0), 8.0).tolist() == [0.0, 0.0, 0.0]


def test_top_types_keep_a_tie():
    assert weight.top_types({"kitchen": 0.85, "living room": 0.3, "bedroom": 0.5,
                             "bathroom": 0.85}) == {"kitchen", "bathroom"}
    assert weight.top_types({}) == set()


def test_the_grid_and_the_prior_column_are_the_written_ones():
    assert weight.GRID == (0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 1000.0)
    assert weight.PRIOR_COL == 15


def test_the_recorded_selection_is_what_the_rows_select():
    rec = json.loads((SELECT / "select.json").read_text())
    rows = weight.load(str(SELECT))
    assert len(rows) == 1 + 2 * len(weight.GRID)
    assert {"commit": rec["commit"], **weight.select(rows)} == rec


# The bars as measured on the 588 test houses at w* = 0.5 (the section file,
# "F results"): F1 and F2 FAILED, so nothing shipped.
VERDICTS = {"F1": False, "F2": False, "F3": True, "F4": True, "F5": True, "F6": True}


def test_the_selection_chose_w_half_by_the_written_rule():
    rec = json.loads((SELECT / "select.json").read_text())
    assert rec["w_star"] == 0.5 and rec["stops"] is False
    assert all(g["cloud"]["n"] == 100 and g["cloud"]["exceptions"] == 0 for g in rec["grid"])


def test_the_recorded_score_is_what_the_rows_score():
    rec = json.loads((TEST / "score.json").read_text())
    rows = weight.load(str(TEST))
    assert len(rows) == 10
    assert {"commit": rec["commit"], **weight.score(rows, 0.5)} == rec


def test_each_bar_is_pinned_as_it_came_out():
    rec = json.loads((TEST / "score.json").read_text())
    assert {k: rec[k]["met"] for k in VERDICTS} == VERDICTS
    assert rec["every_bar_met"] is False
    assert all(v["n"] == 583 and v["exceptions"] == 0 for v in rec["summary"].values())
