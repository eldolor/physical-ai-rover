"""3.156.E's live bars, pinned: the scorer over the committed records gives
the numbers and verdicts the plan file records (CLAUDE.md 7.3). The runs
themselves (`evaluations/dock-3156e/live.py`: the ROS container, nav2, a
saved map) take about an hour and are not re-run here."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "evaluations/dock-3156e"
_spec = importlib.util.spec_from_file_location("dock_score_3156e", HERE / "score.py")
score = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(score)


def _scored():
    return score.score(score.rows(HERE / "live.jsonl"), score.rows(HERE / "live-g4-300s.jsonl"))


def test_g1_the_saved_dock_is_the_dock():
    g1 = _scored()["G1"]
    assert (g1["missions"], g1["saved_map"], g1["docked"], g1["in_every_bar"]) == (5, 5, 5, 5)
    assert (g1["dock_spread_m"], g1["dock_spread_deg"]) == (0.0, 0.0)
    assert g1["off_dock_m"] == [1.41, 2.4, 3.84, 2.31, 2.38]
    assert (g1["median_m"], g1["worst_m"], g1["worst_deg"]) == (0.09, 0.098, 2.3)
    assert g1["met"]


def test_g3_missions_that_do_not_explore_go_home():
    g3 = _scored()["G3"]
    m = g3["main"]
    assert (m["n"], m["docked"], m["in_every_bar"], m["contacts"]) == (10, 10, 10, 0)
    assert (m["median_m"], m["median_deg"], m["worst_m"], m["worst_deg"]) == (0.088, 0.9, 0.098, 2.3)
    s = g3["supplementary"]
    assert (s["n"], s["docked"], s["in_every_bar"], s["contacts"]) == SUPP_COUNTS
    assert g3["met"]


def test_g4_live_no_goal_before_the_end():
    g4 = _scored()["G4_live"]
    assert (g4["missions"], g4["goal_posts_before_end"], g4["unmeasured"]) == (10 + SUPP_COUNTS[0], 0, 0)
    assert all(n in (1, 2) for n in g4["goal_posts_after_end"])
    assert g4["met"]


def test_g5_a_stop_wins_on_the_new_path():
    g5 = _scored()["G5"]
    assert (g5["drills"], g5["cancelled"], g5["autonomy_lapsed_in_g3"]) == (5, 5, 0)
    assert g5["stopped_while"] == ["driving"] * 5
    assert g5["moved_after_stop_cm"] == [0.0] * 5
    assert g5["met"]


SUPP_COUNTS = (6, 6, 6, 0)
