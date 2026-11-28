# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_tight_home_3228.py

PLAN-ros-alignment.md 3.228.B (`docs/plans/ros-alignment/3.228-trip-home-fails.md`,
section B): the trip home from a tight spot.

* 3.229 follow-up (d): the brain's controller passes an `astern_occluded`
  REVERSE to a body that re-vets every move (the robot server over HTTP),
  and only that refusal; an in-process body is unchanged.
* F4: the trip home backs out first when it starts wedged, or when a goal
  is aborted with the robot unmoved; the time reserve counts the escape.
* The in-process sweep's numbers (`tests/tight_home_sweep.py`), one seed
  each way, pinned.
"""

import pytest

import brain.dock as docking
from robot.safety import SafetyController, SafetyViolation

ROVER_BOX = (-7.1, 3.2, -10.7, 8.5)


class _Body:
    """A body with a scan (open floor, or a return at `near_m` ahead),
    wheels reporting, and verbs that record what reached it."""

    def __init__(self, near_m=None, vets=None, ranges=None):
        self.ranges = ranges if ranges is not None else [3.0] * 360
        if near_m is not None:
            for a in range(-20, 21):
                self.ranges[180 + a] = near_m
        self.done = []
        if vets is not None:
            self.vets_moves_on_server = lambda: vets

    def get_scan(self, max_range_m=None):
        return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
                "range_min_m": 0.02, "range_max_m": 12.0, "ranges_m": list(self.ranges)}

    def get_wheel_state(self):
        return {"usable": True}

    def get_odometry(self):
        return {"usable": False}

    def get_depth_grid(self):
        return {"usable": False}

    def get_distance(self):
        return 0.0

    def lidar_self_box_cm(self):
        return ROVER_BOX

    def reverse(self, speed=50, duration=0.5):
        self.done.append(("REVERSE", speed, duration))
        return {"action": "reverse"}

    def drive_forward(self, speed=50, duration=0.5):
        self.done.append(("FORWARD", speed, duration))
        return {"action": "drive_forward"}

    def stop(self):
        return {}


class _Server(_Body):
    """A body that says the robot server re-vets its moves."""

    def vets_moves_on_server(self):
        return True


# ---- (d): astern_occluded is passed on, and only that ----------------------

def test_an_in_process_body_still_refuses_astern_occluded():
    body = _Body()
    safety = SafetyController(body, 20.0)
    assert safety.reverse_clearance() == (0.0, "astern_occluded")
    with pytest.raises(SafetyViolation, match="astern_occluded"):
        safety.check_and_execute("REVERSE", speed=5, duration=1.0)
    assert body.done == []


def test_a_body_the_server_revets_gets_the_astern_occluded_reverse():
    body = _Server()
    safety = SafetyController(body, 20.0)
    assert safety.defer_astern_occluded is True
    safety.check_and_execute("REVERSE", speed=5, duration=1.0)
    assert body.done == [("REVERSE", 5, 1.0)]


def test_other_refusals_stay_in_the_brain():
    # A measured retrace room under the bar is refused here as before.
    body = _Server()
    safety = SafetyController(body, 20.0)
    safety._retrace_room = lambda: 10.0
    safety.footprint_clearance = lambda *a, **k: (None, "scan_footprint_no_target")
    with pytest.raises(SafetyViolation, match="retrace"):
        safety.check_and_execute("REVERSE", speed=50, duration=1.0)
    # A forward refusal is untouched.
    near = _Server(near_m=0.05)
    with pytest.raises(SafetyViolation):
        SafetyController(near, 20.0).check_and_execute("FORWARD", speed=50, duration=1.0)
    assert body.done == [] and near.done == []


@pytest.mark.parametrize("answer", [False, None, "yes", OSError("down")])
def test_only_a_plain_true_defers(answer):
    class _Odd(_Body):
        def vets_moves_on_server(self):
            if isinstance(answer, Exception):
                raise answer
            return answer
    assert SafetyController(_Odd(), 20.0).defer_astern_occluded is False


def test_the_gate_and_remote_robot_pass_the_answer_through():
    from control.mission_runner import _HaltGate
    from control.remote_robot import RemoteRobot
    assert RemoteRobot.__new__(RemoteRobot).vets_moves_on_server() is True
    assert _HaltGate(_Server(), lambda: True).vets_moves_on_server() is True
    assert _HaltGate(_Body(), lambda: True).vets_moves_on_server() is False
    assert SafetyController(_HaltGate(_Server(), lambda: True), 20.0).defer_astern_occluded is True


# ---- F4: wedged, the escape, the reserve ------------------------------------

def test_wedged_reads_the_bar_and_the_turn_room():
    assert docking.wedged(SafetyController(_Body(), 20.0)) is False
    # Something 12 cm ahead of the lidar: forward under the bar.
    assert docking.wedged(SafetyController(_Body(near_m=0.12), 20.0)) is True
    # Something beside the body inside the corner radius plus the margin.
    ranges = [3.0] * 360
    ranges[180 + 90] = 0.20
    side = SafetyController(_Body(ranges=ranges), 20.0)
    assert docking.turn_room_cm(side) < docking.TURN_MARGIN_CM
    assert docking.wedged(side) is True


class _Nav:
    def __init__(self, states):
        self.states = list(states)
        self.goals = []

    def set_goal(self, x, y):
        self.goals.append((x, y))
        return {"accepted": True, "goal_id": len(self.goals)}

    def get_goal(self):
        return {"goal": {"state": self.states.pop(0) if self.states else "active"}}

    def cancel_goal(self):
        return {}


def _trip(body, nav, pose=None):
    pose = pose or {"usable": True, "x_m": 2.0, "y_m": 2.0, "heading_deg": 0.0}
    t = [0.0]

    def clock():
        t[0] += 1.0
        return t[0]
    return docking.ReturnTrip(docking.dock_from_pose({"usable": True, "x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0}), nav,
                              SafetyController(body, 20.0), lambda: dict(pose), clock)


def test_a_wedged_start_backs_out_at_creep_speed_before_the_goal():
    body = _Server(near_m=0.12)
    nav = _Nav([])
    trip = _trip(body, nav)
    trip.step()
    assert body.done == [("REVERSE", docking.NUDGE_SPEED, docking.ESCAPE_BACK_S)]
    assert nav.goals == [] and trip.escapes == 1
    for _ in range(docking.ESCAPE_BACKS):
        trip.step()
    # Still wedged (the fake body never moves): ESCAPE_BACKS in all, then the goal.
    assert [d[0] for d in body.done] == ["REVERSE"] * docking.ESCAPE_BACKS
    assert len(nav.goals) == 1


def test_an_open_start_goes_straight_to_the_goal():
    body = _Server()
    nav = _Nav([])
    _trip(body, nav).step()
    assert body.done == [] and len(nav.goals) == 1


def test_an_unmoved_abort_while_wedged_backs_out():
    body = _Server()
    nav = _Nav(["aborted"])
    trip = _trip(body, nav)
    trip.step()                         # open: the goal
    assert len(nav.goals) == 1
    body.ranges[180] = 0.12             # now wedged, and the robot never moved
    trip.step()                         # the abort
    assert trip.escapes == 1 and body.done == []
    trip.step()
    assert body.done == [("REVERSE", docking.NUDGE_SPEED, docking.ESCAPE_BACK_S)]


def test_a_refused_escape_reverse_ends_the_escape():
    body = _Body(near_m=0.12)           # no server: astern_occluded refused here
    nav = _Nav([])
    trip = _trip(body, nav)
    trip.step()
    assert body.done == [] and "escape REVERSE refused" in trip.log[-1]
    trip.step()
    assert len(nav.goals) == 1


def test_off_means_no_wedge_check(monkeypatch):
    monkeypatch.setattr(docking, "ESCAPE_BACKS", 0)
    body = _Server(near_m=0.12)
    nav = _Nav([])
    trip = _trip(body, nav)
    trip.step()
    assert body.done == [] and len(nav.goals) == 1 and trip.escapes == 0


def test_the_reserve_counts_the_escape_only_when_wedged_and_close():
    budget = docking.HomeBudget({"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0})
    pose = {"usable": True, "x_m": 1.0, "y_m": 0.0}
    asked = []

    def is_wedged(answer):
        def f():
            asked.append(answer)
            return answer
        return f
    th = budget.t_home(budget.path_m(lambda: {}, pose, 0.0))
    # Far from the line: not asked at all.
    assert budget.reserve_reason(lambda: {}, pose, 0.0, th + 100, None, None,
                                 is_wedged=is_wedged(True)) is None
    assert asked == []
    left = th + docking.ESCAPE_RESERVE_S / 2
    assert budget.reserve_reason(lambda: {}, pose, 0.0, left, None, None,
                                 is_wedged=is_wedged(False)) is None
    why = budget.reserve_reason(lambda: {}, pose, 0.0, left, None, None, is_wedged=is_wedged(True))
    assert "back out first" in why and budget.last["escape_s"] == docking.ESCAPE_RESERVE_S


# ---- the sweep, one seed each way (B1's numbers are in the section file) -----

def test_the_sweep_one_nook_before_and_after(monkeypatch):
    from tests import tight_home_sweep as sweep
    monkeypatch.setattr(docking, "ESCAPE_BACKS", docking.ESCAPE_BACKS)   # run_one sets it per arm
    nook = sweep.nooks(1)[0]
    before = sweep.run_one(("before", 0, nook))
    after = sweep.run_one(("after", 0, nook))
    assert not before["can_spin"] and before["return"] == "failed"
    assert before["reason"] == "nav2_aborted" and before["plan_failures"] >= 2
    assert after["return"] == "docked" and after["escapes"] >= 1
    assert min(before["min_gap_cm_trip"], after["min_gap_cm_trip"]) > 0.0
