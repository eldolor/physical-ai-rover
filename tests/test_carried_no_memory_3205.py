# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.205.A: a find on a saved map whose inventory has no memory of the target
searches the floor the camera has not seen (3.70.M3's re-search by sight,
`ExploreAgent.resight`, widened from "handed a recall" to "on a carried map").

`docs/plans/ros-alignment/3.205-carried-map-no-memory.md`, "3.205.A:
criteria" (N7): in process through `MissionRunner`'s own seed path -- the
runner carries landmarks saved on the pose's saved map (`inventory_prior`)
and tells explore (`carried_map()`), none of them the target.
"""

import math

from control.mission_runner import EXHAUSTED, FOUND, MissionRunner
from sim.grid_world import CELL_WALL
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from world.interface import CELL_FREE, CELL_OCCUPIED

CELL = 0.30
FRAME = "map-3205"


class _SavedMap(MockWorld):
    def get_pose(self):
        p = super().get_pose()
        return dict(p, frame_id=FRAME) if p.get("usable") else p


def _known(grid):
    """The whole house mapped before the mission: a saved map's cells."""
    return [CELL_OCCUPIED if (grid._cell(x, y) == CELL_WALL or (x, y) in grid.objects) else CELL_FREE
            for y in range(grid.height) for x in range(grid.width)]


def _prior(labels):
    """A saved report on FRAME holding `labels`, none of them the target."""
    return {"frame_id": FRAME, "candidates": [],
            "reported": [{"label": lab, "x_m": 1.0 + i, "y_m": 1.0, "belief": 0.95}
                         for i, lab in enumerate(labels)]}


def _find(target, start, carried=("sofa",), job=None, recall=None, house="complex_house",
          known=True, resight=True):
    grid = build_world(house)
    grid.x, grid.y = start
    world = _SavedMap(grid)
    if known:
        world._cells = _known(grid)
    robot = MockRobot(grid, render=False)
    kw = {"job": job} if job else {"target_object": target}
    runner = MissionRunner(robot, policy="explore", navigator=FakeNav(robot), world=world,
                           max_steps=400, clock=lambda: grid.sim_time, idle=robot.pass_time,
                           return_to_dock=False, **kw)
    prior = _prior(carried) if carried else None
    if recall is not None:
        prior["reported"].append({"label": target, "x_m": recall[0], "y_m": recall[1], "belief": 0.95})
    runner.inventory_prior = lambda frame_id: prior if frame_id == FRAME else None
    runner.start()
    events, note = [], runner.agent._event
    runner.agent._event = lambda name, pose=None, **f: (events.append((name, f)), note(name, pose, **f))[1]
    if not resight:
        runner.agent.carried_map = lambda: None     # dev before 3.205: the runner's word unheard
    ticks = 0
    while runner.tick() and ticks < 5000:
        ticks += 1
    return runner, grid, [n for n, _ in events], events


def _target_m(grid, target):
    tx, ty = next(c for c, n in grid.objects.items() if n == target)
    return math.dist((grid.x, grid.y), (tx + 0.5, ty + 0.5)) * CELL


def test_red_on_dev_a_carried_map_with_no_memory_ends_exhausted():
    """N7 (a), red: the runner's word unheard (dev), the laptop bag unremembered,
    the whole house one unseen patch looked at once: the find gives up."""
    runner, grid, names, _ = _find("laptop bag", (35.5, 38.5), resight=False)
    assert runner.inventory.carried == 1 and runner.agent.recall == []
    assert runner.agent._carried_map is False          # review: the stub held (seeded after it)
    assert runner.status()["outcome"] == EXHAUSTED
    assert "resight" not in names


def test_a_carried_map_with_no_memory_re_searches_by_sight_and_finds():
    """N7 (a): the same find on the branch searches the unseen floor and finds
    the target by sight."""
    runner, grid, names, events = _find("laptop bag", (35.5, 38.5))
    s = runner.status()
    assert runner.agent._carried_map and not runner.agent._recalled
    assert s["outcome"] == FOUND, s["log_tail"][-5:]
    assert names.count("resight") == 1 and "recall_set" not in names
    assert dict(events)["resight"]["recalled"] is False
    assert _target_m(grid, "laptop bag") <= 0.9


def test_nothing_carried_never_re_searches():
    """N7 (b): a saved map with nothing saved on it (no landmark carried) is
    not a carried map to explore: unchanged."""
    runner, grid, names, _ = _find("purple elephant", (6.5, 4.5), carried=(), house="scaled_house")
    assert not runner.agent._carried_map
    assert runner.status()["outcome"] == EXHAUSTED
    assert "resight" not in names


def test_a_survey_on_a_carried_map_never_re_searches():
    """N7 (c)."""
    runner, grid, names, _ = _find(None, (6.5, 4.5), job="survey", house="scaled_house")
    assert runner.agent._carried_map
    assert "resight" not in names
    assert runner.status()["outcome"] == EXHAUSTED      # review: it ends as a survey does


def test_a_find_with_a_recall_re_searches_once_as_before():
    """N7 (d): 3.70.M3's case through the runner -- recalled and carried."""
    runner, grid, names, events = _find("purple elephant", (6.5, 4.5), house="scaled_house",
                                        recall=(12.15, 3.15))
    assert runner.agent._recalled and runner.agent._carried_map
    assert runner.status()["outcome"] == EXHAUSTED
    assert names.count("resight") == 1 and names.count("exhausted") == 1
    assert names.index("recall_set") < names.index("resight")
    assert dict(events)["resight"]["recalled"] is True


def test_an_agent_without_the_hook_is_not_told():
    """The runner's call is optional (`getattr`), as `recall_from` is."""
    runner, *_ = _find("purple elephant", (6.5, 4.5), house="scaled_house", known=False)
    runner.agent = object()
    runner._mark_carried_map()          # no hook: nothing happens, nothing raises


# ---------- the recorded sweep, pinned (CLAUDE.md 7.3) ----------

import importlib.util                                    # noqa: E402
from pathlib import Path                                 # noqa: E402

EVAL = Path(__file__).resolve().parents[1] / "evaluations" / "carried-no-memory-3205"
_spec = importlib.util.spec_from_file_location("carried_3205", EVAL / "sweep.py")
sweep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sweep)


def _common(s):
    assert s["nav"] == "fake" and s["missions"] == {"carried": 20, "unsurveyed": 20}
    assert s["survey"] == {"outcome": "exhausted", "search_s": 202.3, "contacts": 0, "capped": False}
    assert s["survey_reported"] == 56 and len(s["targets"]) == 15
    assert s["dropped_min"] >= 1                         # every carried find lost X's rows


def test_n1_red_on_dev_twelve_carried_finds_end_exhausted():
    s = sweep.score(EVAL / "rows-dev.jsonl")
    _common(s)
    assert s["N1_exhausted_unfound"]["carried"] == [1, 3, 4, 5, 7, 10, 11, 12, 13, 14, 16, 19]
    assert (s["N2_found"]["carried"], s["N2_found"]["unsurveyed"]) == (8, 20)
    assert s["beside"]["carried"]["resight"] == 0


def test_n2_n3_the_fix_finds_as_many_as_the_unsurveyed_arm():
    s = sweep.score(EVAL / "rows-fix.jsonl")
    _common(s)
    assert (s["N2_found"]["carried"], s["N2_found"]["unsurveyed"]) == (20, 20)
    assert s["N2_found"]["met_vs_unsurveyed"] and s["N1_exhausted_unfound"]["carried"] == []
    assert s["N3"] == {"wrong_found": 0, "contacts": 0, "met": True}
    assert s["beside"]["carried"]["resight"] == 12
    assert (s["beside"]["carried"]["median_goals"], s["beside"]["unsurveyed"]["median_goals"]) == (46.5, 25.5)


def test_n4a_only_the_twelve_exhausted_finds_changed():
    d = sweep.same(EVAL / "rows-dev.jsonl", EVAL / "rows-fix.jsonl")
    assert sorted(d["differ"]) == sorted(str(("carried", k)) for k in (1, 3, 4, 5, 7, 10, 11, 12, 13, 14, 16, 19))


def test_n4bc_m3s_stale_sweep_and_m2s_sweep_identical_to_dev():
    for name in ("stale", "m2"):
        d = sweep.same(EVAL / f"{name}-dev.jsonl", EVAL / f"{name}-fix.jsonl")
        assert d["identical"] and d["rows"] == 41, name


def test_n8_the_live_find_scores_as_the_plan_records():
    live = importlib.util.spec_from_file_location("carried_3205_live", EVAL / "live.py")
    mod = importlib.util.module_from_spec(live)
    live.loader.exec_module(mod)
    # Run 2 (on an idle container): outcomes held, but the load went over 4,
    # so N8 is recorded INVALID (load), not met (the orchestrator's rule).
    s = mod.score(EVAL / "live.jsonl")
    assert all(s["preconditions"].values()) and s["valid"] is False
    assert s["N8"] == {"outcome": "found", "found_m": 0.878, "contacts": 0, "wrong_found": False,
                       "resight": 0, "met": False, "verdict": "INVALID (load)"}
    assert s["beside"]["target"] == "guitar" and s["beside"]["goals"] == 33
    assert (s["load_max"], s["load_ok"]) == (5.96, False)
    # Run 1, set aside for its load (6.4).
    s1 = mod.score(EVAL / "live-1-invalid-load.jsonl")
    assert s1["N8"]["found_m"] == 0.842 and s1["N8"]["contacts"] == 0
    assert s1["N8"]["verdict"] == "INVALID (load)" and s1["N8"]["met"] is False
    assert (s1["load_max"], s1["load_ok"]) == (6.4, False)


def test_n8_3205b_desktop_runs_score_as_recorded():
    live = importlib.util.spec_from_file_location("carried_3205_live", EVAL / "live.py")
    mod = importlib.util.module_from_spec(live)
    live.loader.exec_module(mod)
    # 3.205.B run 1, the one valid run: exhausted, resight fired, N8 met.
    s = mod.score(EVAL / "live-b1.jsonl")
    assert all(s["preconditions"].values()) and s["valid"] is True
    assert s["N8"] == {"outcome": "exhausted", "found_m": 4.416, "contacts": 0, "wrong_found": False,
                       "resight": 1, "met": True, "verdict": "MET"}
    assert s["beside"]["target"] == "headphones" and s["beside"]["goals"] == 29
    assert (s["load_max"], s["load_ok"]) == (3.68, True)
    # Runs 2 and 3: outcomes held, the load over 4, so INVALID (load).
    for name, out, found_m, load in (("live-b2.jsonl", "found", 0.817, 6.62),
                                     ("live-b3.jsonl", "out_of_budget", 5.295, 4.05)):
        s = mod.score(EVAL / name)
        assert all(s["preconditions"].values()) and s["valid"] is False, name
        assert s["N8"] == {"outcome": out, "found_m": found_m, "contacts": 0, "wrong_found": False,
                           "resight": 0, "met": False, "verdict": "INVALID (load)"}, name
        assert (s["load_max"], s["load_ok"]) == (load, False), name


def test_n4d_n5_3192s_sweeps_identical_and_no_goal_in_a_zone():
    spec = importlib.util.spec_from_file_location("same_3192", EVAL / "same3192.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    pinned = EVAL.parent / "nogo-3192" / "fix"
    for name, n in (("find_rule", 192), ("find_rooms", 192), ("survey_rule", 24), ("survey_rooms", 24)):
        d = mod.same(pinned / f"{name}.json", EVAL / "nogo-3192" / f"{name}.json")
        assert d["identical"] and d["rows"] == n, name
        assert d["N5"] == {"goals_in_zone": 0, "goals_in_grown": 0, "entries": 0}, name
