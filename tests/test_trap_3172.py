# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.172.A: leaving 3.158.B's trap (F1) and the return that uses it (F2).

At the trap a pivot and a REVERSE are refused and the forward end is over
the bar, so nothing moved: the escape's bounded creep, and a return that
stops waiting (docs/plans/ros-alignment/3.172-autonomy-rehearsal-fixes.md).
"""

import pytest

import brain.dock as dock
from brain.explore import CREEP_SPEED, NUDGE_PLACE_M, NUDGE_S, OWN_GOAL_REFUSAL, ExploreAgent
from brain.memory import MissionMemory
from robot.interface import Preempted, StoppedByPerson
from robot.interface import VERB_MAX_TAG
from robot.safety import SafetyViolation
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav


def _agent(refuse=("REVERSE",), front_open=False, room=True):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="x"),
                         navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                         world=MockWorld(grid))
    asked = []

    def execute(action, **kw):
        asked.append((action, kw))
        if action in refuse:
            raise SafetyViolation(f"Blocked {action}")
        return {"action": action}

    agent.safety.check_and_execute = execute
    agent._front_open = lambda: front_open
    agent._creep_room = lambda way: room
    agent._creep_ends = lambda: []
    return agent, asked, grid


def _round(agent):
    agent._queue_escape("wedged")
    while agent._pending:
        agent._do_pending()


def test_a_refused_reverse_with_the_front_over_the_bar_creeps_forward_a_bounded_way():
    agent, asked, _ = _agent()
    _round(agent)
    assert asked[0][0] == "REVERSE"
    assert asked[1] == ("FORWARD", {"speed": CREEP_SPEED, "duration": NUDGE_S})
    assert asked[2][0] in ("LEFT", "RIGHT")         # the turns come after it


def test_the_creep_goes_through_the_same_vet_and_a_refusal_goes_on_to_the_turns():
    agent, asked, _ = _agent(refuse=("REVERSE", "FORWARD"))
    _round(agent)
    assert [a for a, _ in asked[:2]] == ["REVERSE", "FORWARD"]
    assert asked[2][0] in ("LEFT", "RIGHT")


def test_a_second_round_at_the_same_place_does_not_walk_the_robot_on():
    agent, asked, _ = _agent()
    _round(agent)
    n = len(asked)
    _round(agent)
    assert not [a for a, kw in asked[n:] if kw.get("duration") == NUDGE_S]


def test_a_new_place_gets_its_own():
    agent, asked, grid = _agent()
    _round(agent)
    n = len(asked)
    grid.x += (NUDGE_PLACE_M + 0.05) / 0.30
    _round(agent)
    assert [a for a, kw in asked[n:] if kw.get("duration") == NUDGE_S] == ["FORWARD"]


def test_no_room_ahead_no_creep():
    agent, asked, _ = _agent(room=False)
    _round(agent)
    assert not [kw for _, kw in asked if kw.get("duration") == NUDGE_S]


def test_3_85s_forward_still_goes_first_with_the_front_open():
    agent, asked, _ = _agent(front_open=True)
    _round(agent)
    assert asked[1][0] == "FORWARD" and "duration" not in asked[1][1]


def test_a_reverse_that_ran_is_not_followed_by_the_nudge():
    agent, asked, _ = _agent(refuse=())
    _round(agent)
    assert not [kw for _, kw in asked if kw.get("duration") == NUDGE_S]


def test_a_refused_reverse_left_over_from_a_wiped_round_does_not_nudge_the_next():
    """Review: a sighting can wipe `_pending` between the REVERSE and its CREEP."""
    agent, asked, _ = _agent()
    agent._queue_escape("wedged")
    agent._do_pending()                             # the REVERSE, refused
    agent._pending = []
    agent.safety.check_and_execute = lambda a, **kw: asked.append((a, kw)) or {"action": a}
    n = len(asked)
    _round(agent)
    assert not [kw for _, kw in asked[n:] if kw.get("duration") == NUDGE_S]


def test_a_nudge_is_one_piece_even_if_its_time_is_cut():
    agent, asked, _ = _agent()
    agent._cut = True
    real = agent.safety.check_and_execute

    def cut(action, **kw):
        real(action, **kw)
        return {"stopped_short": "timeout", "reason": VERB_MAX_TAG}
    agent.safety.check_and_execute = cut
    _round(agent)
    assert len([kw for _, kw in asked if kw.get("duration") == NUDGE_S]) == 1


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def idle(self, s):
        self.t += s


class _Nav:
    """nav2 holding a goal it cannot drive."""

    def __init__(self, cancel_raises=None):
        self.sent, self.cancels, self.goal, self.cancel_raises = 0, 0, None, cancel_raises

    def set_goal(self, x, y):
        self.sent += 1
        self.goal = {"state": "active"}
        return {"accepted": True, "goal_id": str(self.sent)}

    def get_goal(self):
        return {"goal": self.goal}

    def cancel_goal(self):
        self.cancels += 1
        if self.cancel_raises:
            raise self.cancel_raises
        self.goal = None


class _Safety:
    def __init__(self, refuse=("REVERSE",)):
        self.asked, self.refuse = [], refuse

    def check_and_execute(self, action, **kw):
        self.asked.append((action, kw))
        if action in self.refuse:
            raise SafetyViolation(f"Blocked {action}")


def _trip(nav, safety, pose):
    clock = _Clock()
    return (dock.ReturnTrip({"x_m": 1.0, "y_m": 1.0, "theta": 0.0}, nav, safety, pose,
                            clock, idle=clock.idle), clock)


def _until(trip, done, n=5000):
    """Steps while `done()` is false; a regression fails here, it does not hang."""
    for _ in range(n):
        if done():
            return
        assert trip.step() is None
    raise AssertionError("never happened")


def _run(trip, n=20000):
    for _ in range(n):
        got = trip.step()
        if got is not None:
            return got
    raise AssertionError("never ended")


_STILL = {"usable": True, "x_m": 5.0, "y_m": 5.0, "heading_deg": 52.0}


def test_a_return_that_cannot_move_ends_no_motion_long_before_its_timeout():
    """Red on dev: 600 s, `timeout`."""
    nav, safety = _Nav(), _Safety()
    trip, clock = _trip(nav, safety, lambda: _STILL)
    state, reason = _run(trip)
    assert state == dock.FAILED and reason.startswith("no_motion")
    assert clock.t <= 240.0
    assert nav.cancels == dock.STUCK_ROUNDS


def test_a_stall_cancels_then_backs_off_and_creeps_only_if_the_back_off_is_refused():
    nav, safety = _Nav(), _Safety()
    trip, _ = _trip(nav, safety, lambda: _STILL)
    _until(trip, lambda: safety.asked)
    assert nav.cancels == 1 and safety.asked[0][0] == "REVERSE"
    trip.step()
    assert safety.asked[1] == ("FORWARD", {"speed": dock.NUDGE_SPEED, "duration": dock.NUDGE_S})
    trip.step()
    assert nav.sent == 2                            # and the goal again


def test_a_back_off_that_ran_makes_no_creep():
    nav, safety = _Nav(), _Safety(refuse=())
    trip, _ = _trip(nav, safety, lambda: _STILL)
    _until(trip, lambda: safety.asked)
    trip.step()
    assert [a for a, _ in safety.asked] == ["REVERSE"]


def test_a_robot_that_is_moving_never_stalls():
    nav, safety = _Nav(), _Safety()
    pose = dict(_STILL)

    def moving():
        pose["x_m"] += 0.02
        return dict(pose)
    trip, clock = _trip(nav, safety, moving)
    for _ in range(600):
        assert trip.step() is None
    assert clock.t >= 200.0 and nav.cancels == 0 and not safety.asked


def test_stalls_at_a_new_place_start_their_count_again():
    nav, safety = _Nav(), _Safety(refuse=())
    pose = dict(_STILL)
    trip, _ = _trip(nav, safety, lambda: dict(pose))
    _until(trip, lambda: nav.cancels >= dock.STUCK_ROUNDS)
    pose["x_m"] += dock.STUCK_PLACE_M + 0.1
    _until(trip, lambda: nav.cancels >= dock.STUCK_ROUNDS + 1)  # would have failed at the old place
    assert trip.stuck_rounds == 1


def test_a_persons_stop_during_the_cancel_is_not_swallowed():
    nav, safety = _Nav(cancel_raises=StoppedByPerson("stop")), _Safety()
    trip, _ = _trip(nav, safety, lambda: _STILL)
    with pytest.raises(StoppedByPerson):
        _run(trip)


def test_a_new_goal_gets_its_own_stall_clock():
    """Review: 40 s still under one goal must not count against the next."""
    nav, safety = _Nav(), _Safety()
    trip, clock = _trip(nav, safety, lambda: _STILL)
    _until(trip, lambda: clock.t >= 40.0)
    assert nav.cancels == 0
    trip.state = "goal"                             # nav2 ended that one by itself
    trip.step()
    t = clock.t
    _until(trip, lambda: nav.cancels >= 1)
    assert clock.t - t >= dock.STUCK_S - 1.0


def test_a_goal_still_letting_go_is_a_wait_not_a_preemption():
    """Review: the robot server can answer the first REVERSE after the cancel
    with the mission's own goal; `Preempted` must not end the return."""
    nav, safety = _Nav(), _Safety(refuse=())
    held = [Preempted(OWN_GOAL_REFUSAL), Preempted(OWN_GOAL_REFUSAL)]
    real = safety.check_and_execute

    def execute(action, **kw):
        if held:
            raise held.pop()
        real(action, **kw)
    safety.check_and_execute = execute
    trip, _ = _trip(nav, safety, lambda: _STILL)
    _until(trip, lambda: safety.asked)
    assert safety.asked[0][0] == "REVERSE" and trip.refused == 0
    assert [a for a, _ in safety.asked] == ["REVERSE"]  # a wait queues no creep


def test_someone_elses_preemption_still_ends_the_return():
    nav, safety = _Nav(), _Safety(refuse=())

    def execute(action, **kw):
        raise Preempted("a person holds the robot")
    safety.check_and_execute = execute
    trip, _ = _trip(nav, safety, lambda: _STILL)
    with pytest.raises(Preempted):
        _run(trip)


def test_one_creep_per_place_on_the_way_home():
    nav, safety = _Nav(), _Safety()
    trip, _ = _trip(nav, safety, lambda: _STILL)
    _run(trip)
    assert len([kw for a, kw in safety.asked if a == "FORWARD"]) == 1


def test_a_refusal_before_the_goal_does_not_count_against_the_turns_after():
    nav, safety = _Nav(), _Safety()
    trip, _ = _trip(nav, safety, lambda: _STILL)
    _until(trip, lambda: len(safety.asked) >= 2)
    trip.step()
    assert nav.sent == 2 and trip.refused == 0


def test_a_goal_refused_just_after_a_move_is_not_nav2_being_away():
    """A6, live: the creep made the brain the driver, the next goal came back
    `preempted`, and the 30 s measured from the start of the return had run
    out, so the return ended `skipped` a body length from the find."""
    nav, safety = _Nav(), _Safety()
    trip, clock = _trip(nav, safety, lambda: _STILL)
    trip.step()                                     # goal accepted
    clock.t += dock.RETURN_WAIT_S + 5.0
    nav.goal = {"state": "aborted"}
    trip.step()                                     # back-off refused, creep queued
    trip.step()                                     # the creep
    accept, refusals = nav.set_goal, [0]

    def refused_then_taken(x, y):
        if refusals[0] < 3:
            refusals[0] += 1
            return {"accepted": False, "reason": "preempted"}
        return accept(x, y)
    nav.set_goal = refused_then_taken
    for _ in range(4):
        assert trip.step() is None
    assert nav.sent == 2
