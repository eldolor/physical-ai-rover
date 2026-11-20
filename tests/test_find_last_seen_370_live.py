"""3.70.M2's live half: the scorer's reading of a find's explore events (L1).
The run itself needs the ROS container and is not re-run here."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location(
    "find_last_seen_370_live", ROOT / "evaluations/find-last-seen-370/live.py")
live = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(live)

MUG = [{"label": "mug", "x_m": 1.1, "y_m": 2.1}]


def test_events_are_read_from_the_brain_log_lines():
    text = ("1.0 INFO brain.explore explore {\"event\": \"goal_sent\", \"kind\": \"recall\", "
            "\"goal\": [1.0, 1.5], \"at\": [1.0, 2.0]}\n"
            "2.0 INFO other line\n"
            "3.0 INFO brain.explore explore {not json\n"
            "4.0 INFO brain.explore explore {\"event\": \"goal_sent\", \"kind\": \"frontier\"}\n")
    ev = live.events_from_log(text)
    assert [e.get("kind") for e in ev] == ["recall", "frontier"]


def test_a_recall_goal_first_near_the_landmark_meets_l1():
    ev = [{"event": "goal_sent", "kind": "recall", "goal": [1.0, 1.5], "at": [1.0, 2.0]},
          {"event": "goal_sent", "kind": "frontier", "goal": [5.0, 5.0]}]
    r = live.first_act(ev, MUG, "mug")
    assert r["first_act"] == "recall" and r["first_goal_m"] == 0.5 and r["first_recall_ok"]


def test_a_recall_faced_here_counts_with_no_goal():
    ev = [{"event": "recall", "here": True, "at": [1.0, 2.0]}]
    r = live.first_act(ev, MUG, "mug")
    assert r["first_goal_m"] == 0.0 and r["first_recall_ok"]


def test_a_frontier_first_or_a_far_place_or_a_far_goal_fails_l1():
    frontier = [{"event": "goal_sent", "kind": "frontier", "goal": [5.0, 5.0]},
                {"event": "goal_sent", "kind": "recall", "goal": [1.0, 1.5], "at": [1.0, 2.0]}]
    assert not live.first_act(frontier, MUG, "mug")["first_recall_ok"]
    far_place = [{"event": "goal_sent", "kind": "recall", "goal": [3.0, 3.5], "at": [3.0, 4.0]}]
    assert not live.first_act(far_place, MUG, "mug")["first_recall_ok"]
    far_goal = [{"event": "goal_sent", "kind": "recall", "goal": [1.0, 2.8], "at": [1.0, 2.0]}]
    assert not live.first_act(far_goal, MUG, "mug")["first_recall_ok"]
    other_label = [{"event": "goal_sent", "kind": "recall", "goal": [1.0, 1.5], "at": [1.0, 2.0]}]
    assert not live.first_act(other_label, MUG, "shoes")["first_recall_ok"]
    assert not live.first_act([], MUG, "mug")["first_recall_ok"]


def _rows(tmp_path, finds, reload_result="matched"):
    import json
    sm = {"frame_id": "map-a", "reloaded": True}
    rows = [{"row": "survey", "map_id": "map-a", "report_frame_id": "map-a", "targets": ["mug"],
             "load": {"max": 3.0}},
            {"row": "reload", "reload": {"result": reload_result}, "map_id": "map-a"}]
    rows += [{"row": "find", "saved_map": sm, **f} for f in finds]
    p = tmp_path / "live.jsonl"
    p.write_text("".join(json.dumps(r) + "\n" for r in rows))
    return p


GOOD = {"outcome": "found", "found_m": 0.5, "first_recall_ok": True, "contacts": 0,
        "seconds": 100, "goals": 1}


def test_score_five_good_finds_meet_every_bar(tmp_path):
    s = live.score(_rows(tmp_path, [GOOD] * 5))
    assert s["valid"] and s["L1_first_goal"]["verdict"] == "MET"
    assert s["L2_found"]["verdict"] == "MET" and s["L3_stability"]["verdict"] == "MET"
    assert s["beside"]["median_seconds"] == 100 and s["beside"]["load_max"] == 3.0


def test_score_fails_and_invalidates_as_written(tmp_path):
    far = dict(GOOD, found_m=1.2)                  # a wrong found: L2 and L3 fail
    s = live.score(_rows(tmp_path, [GOOD] * 4 + [far]))
    assert s["L2_found"]["verdict"] == "FAILED" and s["L3_stability"]["wrong_found"] == 1
    unmeasured = dict(GOOD, found_m=None)          # fails L2, not counted wrong
    s = live.score(_rows(tmp_path, [GOOD] * 4 + [unmeasured]))
    assert s["L2_found"]["verdict"] == "FAILED" and s["L3_stability"]["verdict"] == "MET"
    refused = {"outcome": "start_refused"}         # a failed find, the run still valid
    s = live.score(_rows(tmp_path, [GOOD] * 4 + [refused]))
    assert s["valid"] and s["L3_stability"]["verdict"] == "FAILED"
    s = live.score(_rows(tmp_path, [GOOD] * 5, reload_result="fell_back"))
    assert not s["valid"] and s["L1_first_goal"]["verdict"] == "INVALID"
    s = live.score(_rows(tmp_path, [GOOD] * 4))    # a run cut short
    assert s["L2_found"]["verdict"] == "INVALID"


def test_the_live_run_scores_as_the_plan_records():
    s = live.score(ROOT / "evaluations/find-last-seen-370/live.jsonl")
    assert s["preconditions"] == {"survey_saved_one_map": True, "reload_matched": True,
                                  "every_find_on_the_reload": True}
    assert s["valid"] and s["finds"] == 5
    assert (s["L1_first_goal"]["recall_first"], s["L1_first_goal"]["verdict"]) == (5, "MET")
    assert (s["L2_found"]["found"], s["L2_found"]["verdict"]) == (5, "MET")
    st = s["L3_stability"]
    assert (st["contacts"], st["wrong_found"], st["failed"], st["wall_capped"], st["verdict"]) == (0, 0, 0, 0, "MET")
    b = s["beside"]
    assert (b["median_seconds"], b["median_goals"], b["median_distance_m"]) == (183, 2, 18.77)
    assert b["returns"] == ["docked", "docked", "docked", "skipped", "docked"]
