"""3.175.A's recorded sweep, pinned: the scorer over the committed rows gives
the numbers and verdicts the plan file records (CLAUDE.md 7.3). The sweep
itself takes ~15 min on four workers and is not re-run here."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    "memory_moved_3175", ROOT / "evaluations/memory-moved-3175/sweep.py")
sweep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sweep)


def test_recorded_sweep_scores_as_the_plan_records():
    s = sweep.score(ROOT / "evaluations/memory-moved-3175/rows.jsonl")
    assert (s["trials"], s["moved"], s["gone"]) == (75, 56, 19)
    assert (s["A1"]["stale_found"], s["A1"]["cold_found"], s["A1"]["met"]) == (24, 23, True)
    assert round(s["A2"]["p90_s"], 1) == 506.9 and not s["A2"]["met"]
    assert (s["A3"]["recalled_old_again"], s["A3"]["met"]) == (30, False)
    assert (s["A4"]["after_found"], s["A4"]["after_median_s"], s["A4"]["cold_median_s"],
            s["A4"]["met"]) == (27, 717.9, 762.0, True)
    assert (s["A5"]["fresh_found"], s["A5"]["seed_found"], s["A5"]["first_recall"],
            s["A5"]["met"]) == (47, 23, 0, False)
    assert s["A6"]["found_when_gone"] == 0 and s["A6"]["met"]
    assert (s["A7"]["wrong_found"], s["A7"]["contacts"], s["A7"]["met"]) == (0, 0, True)


def test_b_dev_scores_as_the_plan_records():
    """3.175.B B-dev: `dev` as it is, the map carried (amendment B0)."""
    assert sweep.trials_hash() == "fe61b6cf72157245"
    s = sweep.score(ROOT / "evaluations/memory-moved-3175/rows-b-dev.jsonl")
    assert (s["trials"], s["moved"], s["gone"]) == (75, 56, 19)
    assert (s["A1"]["stale_found"], s["A1"]["cold_found"], s["A1"]["met"]) == (21, 23, False)
    assert round(s["A2"]["p90_s"], 1) == 574.0 and not s["A2"]["met"]
    assert (s["A3"]["recalled_old_again"], s["A3"]["met"]) == (41, False)
    assert (s["A4"]["after_found"], s["A4"]["after_median_s"], s["A4"]["cold_median_s"],
            s["A4"]["met"]) == (20, 640.6, 762.0, False)
    assert (s["A5"]["fresh_found"], s["A5"]["seed_found"], s["A5"]["first_recall"],
            s["A5"]["fresh_median_s"], s["A5"]["met"]) == (28, 23, 19, 908.8, False)
    assert s["A6"]["found_when_gone"] == 0 and s["A6"]["met"]
    assert (s["A7"]["wrong_found"], s["A7"]["contacts"], s["A7"]["capped"], s["A7"]["met"]) == (0, 0, 0, True)


def test_b_fix_scores_as_the_plan_records():
    """3.175.B B-fix: stale at one empty look, recalls newest first (B1)."""
    s = sweep.score(ROOT / "evaluations/memory-moved-3175/rows-b-fix.jsonl")
    assert (s["trials"], s["moved"], s["gone"]) == (75, 56, 19)
    assert (s["A1"]["stale_found"], s["A1"]["cold_found"], s["A1"]["met"]) == (21, 23, False)
    assert round(s["A2"]["p90_s"], 1) == 573.5 and not s["A2"]["met"]
    assert (s["A3"]["recalled_old_again"], s["A3"]["met"]) == (23, False)
    assert (s["A4"]["after_found"], s["A4"]["after_median_s"], s["A4"]["cold_median_s"],
            s["A4"]["met"]) == (19, 640.6, 762.0, False)
    assert (s["A5"]["fresh_found"], s["A5"]["seed_found"], s["A5"]["first_recall"],
            s["A5"]["fresh_median_s"], s["A5"]["met"]) == (28, 23, 19, 908.8, False)
    assert s["A6"]["found_when_gone"] == 0 and s["A6"]["met"]
    assert (s["A7"]["wrong_found"], s["A7"]["contacts"], s["A7"]["capped"], s["A7"]["met"]) == (0, 0, 0, True)


def test_e_scores_as_the_plan_records():
    """3.175.E: an aborted recall goes again once, from another point."""
    s = sweep.score(ROOT / "evaluations/memory-moved-3175/rows-e.jsonl")
    assert (s["trials"], s["moved"], s["gone"]) == (75, 56, 19)
    assert (s["A1"]["stale_found"], s["A1"]["cold_found"], s["A1"]["met"]) == (17, 23, False)
    assert round(s["A2"]["p90_s"], 1) == 574.2 and not s["A2"]["met"]
    assert (s["A3"]["recalled_old_again"], s["A3"]["met"]) == (30, False)
    assert (s["A4"]["after_found"], s["A4"]["after_median_s"], s["A4"]["cold_median_s"],
            s["A4"]["met"]) == (14, 640.6, 762.0, False)
    assert (s["A5"]["fresh_found"], s["A5"]["seed_found"], s["A5"]["first_recall"],
            s["A5"]["fresh_median_s"], s["A5"]["met"]) == (28, 23, 19, 908.8, False)
    assert s["A6"]["found_when_gone"] == 0 and s["A6"]["met"]
    assert (s["A7"]["wrong_found"], s["A7"]["contacts"], s["A7"]["capped"], s["A7"]["met"]) == (0, 0, 0, True)
