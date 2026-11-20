"""3.70.M2's recorded sweep, pinned: the scorer over the committed rows gives
the numbers and verdicts the plan file records (CLAUDE.md 7.3). The sweep
itself takes ~20 min on four workers and is not re-run here."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    "find_last_seen_370", ROOT / "evaluations/find-last-seen-370/sweep.py")
sweep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sweep)


def test_run_1_scores_as_the_plan_records():
    s = sweep.score(ROOT / "evaluations/find-last-seen-370/rows.jsonl")
    assert s["survey"] == {"outcome": "exhausted", "search_s": 909.8, "contacts": 0, "capped": False}
    assert s["survey_reported"] == 19
    assert s["targets"] == ["mug", "shoes", "toy car", "trash can", "umbrella", "water bottle"]
    assert s["missions"] == {"surveyed": 20, "unsurveyed": 20}
    assert (s["found"]["surveyed"], s["found"]["unsurveyed"], s["found"]["met"]) == (7, 5, False)
    assert (s["first_goal"]["recall_first"], s["first_goal"]["met"]) == (17, False)
    assert [m[0] for m in s["first_goal"]["missed"]] == [3, 11, 15]
    assert (s["median_s"]["surveyed"], s["median_s"]["unsurveyed"], s["median_s"]["met"]) == (1200.0, 1200.0, False)
    assert s["beside"]["surveyed"]["median_found_s"] == 510.2
    assert s["beside"]["unsurveyed"]["median_found_s"] == 280.7
    st = s["stability"]
    assert (st["wrong_found"], st["contacts"], st["capped"], st["met"]) == (0, 0, 0, True)
