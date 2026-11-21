"""3.70.M3: a memory that is out of date costs a detour, never a wrong Found.

Two halves (`docs/plans/ros-alignment/3.70-survey-and-mission-ends.md`, "M3:
how it is measured" and its records):

* the recorded sweep, pinned: the scorer over the committed rows gives the
  numbers and verdicts the plan file records (CLAUDE.md 7.3) -- the red run
  (a policy that trusts the memory), `dev` before the fix, and the fix;
* explore's re-search by sight (`ExploreAgent.resight`), in process, on a map
  known before the mission and a target moved off its remembered place: red
  with the re-search held off (the pre-fix `dev`), found with it.
"""

import importlib.util
import math
from pathlib import Path

import numpy as np

from control.mission_runner import EXHAUSTED, FOUND, MissionRunner
from sim.grid_world import CELL_WALL
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN

ROOT = Path(__file__).resolve().parents[1]
EVAL = ROOT / "evaluations" / "find-last-seen-370"
_spec = importlib.util.spec_from_file_location("stale_memory_370", EVAL / "stale.py")
stale = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(stale)

CELL = 0.30


# ---------- the recorded sweep ----------

def _common(s):
    assert s["nav"] == "fake" and s["moved"]
    # The same survey as M2 run 2 (amendment 1): 202.3 sim s, 56 reported.
    assert s["survey"] == {"outcome": "exhausted", "search_s": 202.3, "contacts": 0, "capped": False}
    assert s["survey_reported"] == 56 and len(s["targets"]) == 15


def test_red_a_policy_that_trusts_the_memory_ends_found_at_the_old_place():
    s = stale.score(EVAL / "rows-m3-trust.jsonl")
    _common(s)
    assert s["trust"] is True and s["missions"] == {"surveyed": 20, "unsurveyed": 0}
    assert s["K3_memory_alone"]["wrong_found"]["surveyed"] == 20
    assert s["K3_memory_alone"]["met"] is False
    assert s["K1_found"]["surveyed"] == 0 and s["K1_found"]["met"] is False


def test_dev_before_the_fix_scores_as_the_plan_records():
    s = stale.score(EVAL / "rows-m3.jsonl")
    _common(s)
    assert s["missions"] == {"surveyed": 20, "unsurveyed": 20}
    assert (s["K1_found"]["surveyed"], s["K1_found"]["unsurveyed"], s["K1_found"]["met"]) == (10, 20, False)
    assert [m[0] for m in s["beside"]["surveyed"]["misses"]] == [2, 5, 6, 8, 10, 12, 15, 16, 17, 18]
    assert s["beside"]["surveyed"]["outcomes"] == ["exhausted", "found"]
    assert (s["K2_belief"]["fell"], s["K2_belief"]["of"], s["K2_belief"]["met"]) == (17, 20, False)
    assert s["K3_memory_alone"]["met"] and s["K4_contacts"]["met"]
    assert s["first_recall"] == 20


def test_the_fix_scores_as_the_plan_records():
    s = stale.score(EVAL / "rows-m3-fix.jsonl")
    _common(s)
    assert s["missions"] == {"surveyed": 20, "unsurveyed": 20}
    assert (s["K1_found"]["surveyed"], s["K1_found"]["unsurveyed"], s["K1_found"]["met"]) == (20, 20, True)
    k2 = s["K2_belief"]
    assert (k2["fell"], k2["of"], k2["under_report"], k2["met"]) == (17, 20, 17, False)
    assert [e[0] for e in k2["ends"] if e[2] is not None and e[2] >= e[1]] == [3, 11, 12]
    assert s["K3_memory_alone"]["wrong_found"] == {"surveyed": 0, "unsurveyed": 0}
    assert s["K3_memory_alone"]["met"] and s["K4_contacts"]["met"]
    assert s["first_recall"] == 20
    # The detour, recorded beside (no bar): goals at the median.
    assert (s["beside"]["surveyed"]["median_goals"], s["beside"]["unsurveyed"]["median_goals"]) == (46.5, 18.5)


# ---------- explore's re-search by sight, in process ----------

def _known(grid):
    """The whole house mapped before the mission: a saved map's cells."""
    return [CELL_OCCUPIED if (grid._cell(x, y) == CELL_WALL or (x, y) in grid.objects) else CELL_FREE
            for y in range(grid.height) for x in range(grid.width)]


def _find(target, moved_to, start, resight=True, recall=True, house="complex_house"):
    grid = build_world(house)
    grid.x, grid.y = start
    old = next((c for c, n in grid.objects.items() if n == target), None)
    world = MockWorld(grid)
    world._cells = _known(grid)              # mapped with the target at its old place
    if old is not None and moved_to is not None:
        grid.move_object(old, moved_to)
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, target_object=target, policy="explore", navigator=FakeNav(robot),
                           world=world, max_steps=400, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=False)
    runner.start()
    events, note = [], runner.agent._event
    runner.agent._event = lambda name, pose=None, **f: (events.append(name), note(name, pose, **f))[1]
    if not resight:
        runner.agent._unseen_floor = lambda m, pose: False     # dev before 3.70.M3
    if recall:
        place = old if old is not None else (40, 20)
        runner.agent.recall_from([((place[0] + 0.5) * CELL, (place[1] + 0.5) * CELL)])
    ticks = 0
    while runner.tick() and ticks < 5000:
        ticks += 1
    return runner, grid, events


def _target_m(grid, target):
    tx, ty = next(c for c, n in grid.objects.items() if n == target)
    return math.dist((grid.x, grid.y), (tx + 0.5, ty + 0.5)) * CELL


def test_red_without_the_re_search_a_moved_target_ends_exhausted():
    """M3's k 2: the laptop bag moved from the study to the kitchen. With the
    whole house one unseen patch, looked at once, the search gives up."""
    runner, grid, events = _find("laptop bag", (30, 13), (35.5, 38.5), resight=False)
    assert runner.status()["outcome"] == EXHAUSTED
    assert "resight" not in events


def test_a_moved_target_is_found_where_it_went_after_the_stale_recall():
    runner, grid, events = _find("laptop bag", (30, 13), (35.5, 38.5))
    s = runner.status()
    assert s["outcome"] == FOUND, s["log_tail"][-5:]
    assert events.count("resight") == 1
    assert events.index("recall_set") < events.index("resight")
    assert _target_m(grid, "laptop bag") <= 0.9          # at the NEW place: sight, not memory


def test_a_find_with_no_recall_never_re_searches():
    """The re-search is the stale recall's: an unremembered target's search
    ends as it did."""
    runner, grid, events = _find("purple elephant", None, (6.5, 4.5), recall=False,
                                 house="scaled_house")
    assert runner.status()["outcome"] == EXHAUSTED
    assert "resight" not in events


def test_the_re_search_runs_once_and_still_ends():
    runner, grid, events = _find("purple elephant", None, (6.5, 4.5), house="scaled_house")
    assert runner.status()["outcome"] == EXHAUSTED
    assert events.count("resight") == 1 and events.count("exhausted") == 1


def test_the_sight_map_unknowns_only_unseen_floor_away_from_the_robot():
    runner, grid, _ = _find("purple elephant", None, (6.5, 4.5), recall=False, house="scaled_house")
    agent = runner.agent
    m = dict(runner.world.get_map(), map_version=None)
    pose = {"x_m": 3.0 * CELL + 0.15, "y_m": 3.0 * CELL + 0.15}
    agent.seen = {((12 + 0.5) * CELL, (3 + 0.5) * CELL)}       # one hallway cell seen
    out = agent._sight_map(m, pose)
    w = m["width"]
    cells, before = np.asarray(out["cells"]), np.asarray(m["cells"])
    assert out["map_version"] is None                        # no version: caches stay off
    assert cells[3 * w + 12] == CELL_FREE                     # seen
    assert cells[3 * w + 3] == CELL_FREE and cells[3 * w + 4] == CELL_FREE   # beside the robot
    assert cells[11 * w + 20] == CELL_UNKNOWN                 # the study, unseen
    assert ((before == CELL_OCCUPIED) == (cells == CELL_OCCUPIED)).all()     # walls stay walls
    assert ((cells == CELL_UNKNOWN) <= ((before == CELL_FREE) | (before == CELL_UNKNOWN))).all()
    v = agent._sight_map(dict(m, map_version=7), pose)["map_version"]
    assert v == (7, "sight", 1)
