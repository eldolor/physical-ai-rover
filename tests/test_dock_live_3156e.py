"""3.156.E's live bars, pinned: the scorer over the committed records gives
the numbers and verdicts the plan file records (CLAUDE.md 7.3). The runs
themselves (`evaluations/dock-3156e/live.py`: the ROS container, nav2, a
saved map) take about an hour and a half and are not re-run here."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "evaluations/dock-3156e"
_spec = importlib.util.spec_from_file_location("dock_score_3156e", HERE / "score.py")
score = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(score)


def _scored():
    return score.score(score.rows(HERE / "live.jsonl"), score.rows(HERE / "live-g4-300s.jsonl"),
                       score.rows(HERE / "live-g5.jsonl"))


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
    assert (s["n"], s["docked"], s["in_every_bar"], s["contacts"]) == (6, 6, 6, 0)
    assert (s["worst_m"], s["worst_deg"]) == (0.092, 0.9)
    assert g3["met"]


def test_g4_live_no_goal_before_the_end():
    g4 = _scored()["G4_live"]
    assert (g4["missions"], g4["with_window"], g4["unmeasured"]) == (26, 18, 0)
    assert (g4["goal_posts_before_end"], g4["the_returns_own_before_the_marker"], g4["early"]) == (1, 1, 0)
    assert g4["pre_end_goal_ids_polled"] == 0
    assert g4["met"]


def test_g4_a_post_before_the_marker_is_early_unless_it_is_the_returns_own():
    row = {"goal_posts_before_end": 1, "goal_posts_after_end": 0, "end_decided_s": 0.29,
           "ret": {"decided_at_s": 0.02, "goal_id": "a"}, "goals_seen": {"a": {"t": 0.5, "return": True}}}
    assert score._marker_lag(row) == 1
    assert score._marker_lag({**row, "goal_posts_after_end": 1}) == 0
    assert score._marker_lag({**row, "goals_seen": {"b": {"t": 0.5, "return": True}}}) == 0
    assert score._marker_lag({**row, "ret": {"decided_at_s": 0.4, "goal_id": "a"}}) == 0


def test_g5_a_stop_wins_on_the_new_path():
    g5 = _scored()["G5"]
    assert g5["first_run_invalid"]["return_metres_at_stop"] == [0.0, 0.0, 0.01, 0.0, 0.0]
    assert (g5["drills"], g5["cancelled"], g5["autonomy_lapsed_in_g3"]) == (5, 5, 0)
    assert g5["stopped_while"] == ["driving"] * 5
    assert g5["return_turned_deg_at_stop"] == [15.6, 22.1, 19.5, 17.2, 7.5]
    assert g5["return_moved_m_at_stop"] == [0.017, 0.0, 0.0, 0.0, 0.151]
    assert g5["moved_after_stop_cm"] == [0.0] * 5 and g5["turned_after_stop_deg"] == [0.0] * 5
    assert g5["met"]


def test_a_record_that_never_started_fails_its_bar_without_crashing():
    s = score.score([{"kind": "find", "house": "scaled_house", "start": 409, "detail": "busy"}])
    assert not s["G1"]["met"] and not s["G3"]["met"] and not s["G5"]["met"]
