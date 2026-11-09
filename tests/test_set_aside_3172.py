# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.172.B (F3): places a mission could not reach, set aside for the next ones
on the same saved map (docs/plans/ros-alignment/3.172-autonomy-rehearsal-fixes.md)."""

from brain.frontier import SET_ASIDE_MIN_FAILURES, SET_ASIDE_MISSIONS, RetryBook
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav


def test_a_place_set_aside_is_no_goal():
    book = RetryBook(set_aside=[(5.0, 5.0, 0)])
    assert not book.available(5.1, 5.0, 0.0)
    assert book.available(7.0, 5.0, 0.0)


def test_an_approach_is_never_refused_for_one():
    book = RetryBook(set_aside=[(5.0, 5.0, 0)])
    assert book.available(5.1, 5.0, 0.0, approach=True)


def test_it_is_not_dropped_and_not_waited_on():
    book = RetryBook(set_aside=[(5.0, 5.0, 0)])
    assert not book.dropped(5.0, 5.0)
    assert not book.cooling(0.0)


def test_only_a_place_that_failed_enough_is_carried():
    book = RetryBook()
    book.fail(1.0, 1.0, 0.0)
    for _ in range(SET_ASIDE_MIN_FAILURES):
        book.fail(5.0, 5.0, 0.0)
    assert book.carry_out(SET_ASIDE_MIN_FAILURES, SET_ASIDE_MISSIONS) == [{"x_m": 5.0, "y_m": 5.0, "age": 0}]


def test_a_place_comes_back_after_its_missions():
    book = RetryBook(set_aside=[(5.0, 5.0, 0)])
    for age in range(1, SET_ASIDE_MISSIONS):
        out = book.carry_out(SET_ASIDE_MIN_FAILURES, SET_ASIDE_MISSIONS)
        assert out == [{"x_m": 5.0, "y_m": 5.0, "age": age}]
        book = RetryBook(set_aside=[(r["x_m"], r["y_m"], r["age"]) for r in out])
    assert book.carry_out(SET_ASIDE_MIN_FAILURES, SET_ASIDE_MISSIONS) == []


def test_failing_again_restarts_its_age_with_no_second_row():
    book = RetryBook(set_aside=[(5.0, 5.0, 2)])
    for _ in range(SET_ASIDE_MIN_FAILURES):
        book.fail(5.1, 5.0, 0.0)
    out = book.carry_out(SET_ASIDE_MIN_FAILURES, SET_ASIDE_MISSIONS)
    assert len(out) == 1 and out[0]["age"] == 0


def _runner():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, policy="explore", navigator=FakeNav(robot), world=MockWorld(grid),
                           target_object="x")
    runner.start()
    return runner


def test_the_next_mission_on_the_same_map_sets_it_aside():
    runner = _runner()
    prior = {"frame_id": "f1", "set_aside": [{"x_m": 5.0, "y_m": 5.0, "age": 1}]}
    assert runner._carry_set_aside("f1", prior) == 1
    assert runner.agent.retry.set_aside == [(5.0, 5.0, 1)]


def test_another_saved_map_carries_nothing():
    runner = _runner()
    assert runner._carry_set_aside("f2", {"frame_id": "f1", "set_aside": [{"x_m": 5.0, "y_m": 5.0, "age": 0}]}) == 0
    assert runner.agent.retry.set_aside == []


def test_a_bad_row_is_skipped_and_none_is_no_error():
    runner = _runner()
    assert runner._carry_set_aside("f1", {"frame_id": "f1", "set_aside": [{"x_m": "a"}, 7, {"x_m": 1.0, "y_m": 2.0, "age": 0}]}) == 1
    assert runner._carry_set_aside("f1", None) == 0


def test_the_report_carries_what_this_mission_learned():
    runner = _runner()
    for _ in range(SET_ASIDE_MIN_FAILURES):
        runner.agent.retry.fail(5.0, 5.0, 0.0)
    assert runner.inventory_report()["set_aside"] == [{"x_m": 5.0, "y_m": 5.0, "age": 0}]


def test_a_mission_with_only_places_set_aside_left_tries_them_rather_than_end():
    """Review: a place a person blocked last mission must not end this one
    `exhausted` with the target unlooked-for: it is tried last, not never."""
    runner = _runner()
    runner.agent.retry = RetryBook(radius_m=100.0, set_aside=[(0.0, 0.0, 0)])
    for _ in range(200):
        runner.tick()
        if not runner.agent.retry.set_aside or runner.agent.exhausted:
            break
    assert runner.agent.retry.set_aside == [] and not runner.agent.exhausted


def test_set_aside_stays_off_s3():
    """The chain's fields are local (the user, 2026-10-31): these are places on a floor plan."""
    from control.inventory_store import _for_s3
    assert "set_aside" not in _for_s3({"landmarks": [], "set_aside": [{"x_m": 1.0, "y_m": 2.0, "age": 0}]})
