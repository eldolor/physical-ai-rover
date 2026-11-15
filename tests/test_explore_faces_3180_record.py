# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
3.180.B's record, pinned: the faces change FAILED and is not shipped (its
code and unit tests stay on the branch). Its sweep's summaries and rows
(`evaluations/explore-view-3180/b/`) keep the verdicts as measured; a failed
bar stays recorded as failed.
"""
import json
from pathlib import Path

B = Path(__file__).resolve().parent.parent / "evaluations" / "explore-view-3180" / "b"
VERDICTS = {"B1": True, "B2": False, "B3": False, "B4": False, "B5": False, "B6": False}


def test_b_bars_as_measured():
    rec = json.loads((B / "score_b.json").read_text())
    assert {k: rec[k]["met"] for k in VERDICTS} == VERDICTS
    assert len(rec["B3"]["lost"]) == 9 and rec["B6"]["exhausted_B_base"] == [9, 1]


def test_b_rows_hold_the_lost_and_the_traced():
    rows = [json.loads(line) for line in (B / "rows.jsonl").read_text().splitlines()]
    assert len(rows) == 297
    at = {(r["arm"], r["b"]["index"], r["b"]["mode"]): r for r in rows}
    assert (at[("wrong", 247, "R")]["base"]["outcome"], at[("wrong", 247, "R")]["b"]["outcome"]) \
        == ("exhausted", "found")
    assert at[("right", 194, "U")]["b"]["outcome"] == "exhausted"
    for arm, i, mode, _ in json.loads((B / "score_b.json").read_text())["B3"]["lost"]:
        r = at[(arm, i, mode)]
        assert (r["base"]["outcome"], r["b"]["outcome"]) == ("found", "exhausted")


def test_the_faces_code_is_not_on_this_tree():
    import brain.frontier as frontier
    assert not hasattr(frontier, "camera_faces")
