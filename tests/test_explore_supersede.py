# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.213.B (C1): a live approach is replaced by a new sighting only if the new
place is better supported -- more of the mission's sightings within
`SUPPORT_M` of it than of the live approach's place.

3.213.A: a true detection ranged by the lidar on a wall or furniture nearer
or farther than the target replaced a good approach in about half of 180
ProcTHOR finds; at the oracle's bound the cost was 4-7 finds and a quarter
of the slow tail.

* the rule itself, on its helper: strict, live approach goals only, every
  sighting counted once a tick. The in-process find is
  `tests/test_explore_supersede_sweep.py` (slow tier).
"""

import brain.explore as explore_mod
from brain.explore import ExploreAgent
from brain.memory import MissionMemory
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests.test_explore import FakeNav


def _agent():
    robot = MockRobot(build_world("scaled_house"), render=False)
    return ExploreAgent(robot, MissionMemory(mission="m", target_object="red backpack"),
                        navigator=FakeNav(robot), clock=lambda: 0.0)


def _live(agent, target, kind="approach"):
    agent._goal = {"kind": kind, "target": target, "x_m": 0.0, "y_m": 0.0, "sent_at": 0.0}


def test_a_better_supported_live_approach_is_kept():
    a = _agent()
    _live(a, (1.0, 1.0))
    a._sighted.extend([(1.0, 1.0), (1.1, 1.0), (3.0, 1.0)])
    assert a._keeps_approach(3.0, 1.0) is True


def test_a_tie_keeps_the_live_approach():
    a = _agent()
    _live(a, (1.0, 1.0))
    a._sighted.extend([(1.0, 1.0), (3.0, 1.0)])
    assert a._keeps_approach(3.0, 1.0) is True


def test_a_better_supported_new_place_replaces_it():
    a = _agent()
    _live(a, (1.0, 1.0))
    a._sighted.extend([(1.0, 1.0), (3.0, 1.0), (3.2, 1.0)])
    assert a._keeps_approach(3.0, 1.0) is False


def test_nothing_is_kept_without_a_live_approach_goal():
    a = _agent()
    a._sighted.extend([(1.0, 1.0)] * 5 + [(3.0, 1.0)])
    assert a._keeps_approach(3.0, 1.0) is False          # no goal
    _live(a, (1.0, 1.0), kind="frontier")
    assert a._keeps_approach(3.0, 1.0) is False          # a goal, not an approach


def test_support_counts_within_support_m():
    a = _agent()
    _live(a, (1.0, 1.0))
    off = explore_mod.SUPPORT_M + 0.01
    a._sighted.extend([(1.0, 1.0), (1.0 + off, 1.0), (1.0 + off, 1.0), (3.0, 1.0)])
    # The two just outside SUPPORT_M of the live place support neither.
    assert a._keeps_approach(3.0, 1.0) is True
    a._sighted.append((3.0 + explore_mod.SUPPORT_M - 0.01, 1.0))
    assert a._keeps_approach(3.0, 1.0) is False


def test_every_sighting_is_counted_once_a_tick(monkeypatch):
    a = _agent()
    calls = iter([(2.0, 2.0), (2.1, 2.0)])
    monkeypatch.setattr(a, "_sighting_now", lambda frame: next(calls))
    frame = {}
    a._ticks = 1
    assert a._sighting(frame) == (2.0, 2.0)
    assert a._sighting(frame) == (2.0, 2.0)              # memoised: not counted again
    a._ticks = 2
    a._sighting(frame)
    assert list(a._sighted) == [(2.0, 2.0), (2.1, 2.0)]


def test_a_refused_place_supports_nothing():
    """Review: the refused object's ticks must not out-vote a live approach."""
    from brain.arrival import REFUSED
    a = _agent()
    a.sight_from = "perception"
    a._sighted.extend([(3.0, 1.0)] * 5 + [(1.0, 1.0)] * 2)
    a._last_seen = (3.0, 1.0)
    a._note_refusal({"_arrival": {"state": REFUSED}})
    assert list(a._sighted) == [(1.0, 1.0)] * 2
    _live(a, (1.0, 1.0))
    assert a._keeps_approach(3.1, 1.0) is True           # unpruned: 5 > 2, replaced


def test_the_sightings_kept_are_capped_newest_kept():
    a = _agent()
    for i in range(explore_mod.SIGHTED_MAX + 5):
        a._sighted.append((float(i), 0.0))
    assert len(a._sighted) == explore_mod.SIGHTED_MAX
    assert a._sighted[0] == (5.0, 0.0)


def test_scaled_house_is_found_before_the_target_is_lost():
    """Thermos review: 3.192.B's scaled_house witnesses hold C1 off
    (`tests/test_explore_reseen.py`); with it on, the good approach is kept
    and the find never needs the look-again rounds (C1 off: 114.8 s)."""
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location(
        "nogo_sweep_3161_supersede_fast",
        Path(__file__).resolve().parents[1] / "evaluations" / "nogo-3161" / "sweep.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    row = mod.run("scaled_house", 0, "none")
    assert row["outcome"] == "found" and row["sim_s"] < 60     # 27.9 s


def test_a_refusal_at_the_kept_approach_drops_that_place_not_the_last_seen():
    """Review 2: a withheld far sighting is `_last_seen`; refused while standing
    at the approach kept, the approach's place is the one dropped."""
    from brain.arrival import REFUSED
    a = _agent()
    a.sight_from = "perception"
    here = (2.0, 2.0)
    a._approach, a._last_seen = here, (5.0, 2.0)
    a._sighted.extend([here] * 3 + [(5.0, 2.0)])
    a._at_target = lambda tx, ty: (tx, ty) == here
    a._note_refusal({"_arrival": {"state": REFUSED}})
    assert here in a._refused_at and (5.0, 2.0) not in a._refused_at
    assert list(a._sighted) == [(5.0, 2.0)]
