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
