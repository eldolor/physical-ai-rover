# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.198: the final approach on a body whose straight verbs move whole 0.30 m
(`docs/plans/ros-alignment/3.198-return-goal-race.md`).

Under `drive: ros` a straight verb is whole 0.30 m moves
(`robot/ros_drive.py` `moves_for`: at least one), and nav2 ends a goal the
moment the robot is within `xy_goal_tolerance` (0.10 m) of it. 3.195's c-1:
nav2 stopped 0.104 m short of the staging point on the dock's side, the one
creep then moved 0.30 m and left the robot 0.1025 m past the dock, and the
approach pivoted 180 degrees back and forth for 40 verbs (190 s) before
`not_aligned`. These replay that geometry on `ReturnTrip` itself: a body that
moves as `RosDriveRobot` does, a navigator that stops as nav2's goal checker
does.
"""

import math

import pytest

from brain import dock as dk
from brain.dock import DOCK_TOL_DEG, DOCK_TOL_M, ReturnTrip, ahead, wrap_deg

MOVE_M = 0.30                   # robot/ros_drive.py MOVE_M
NAV2_XY_TOL_M = 0.10            # service/slam/.../nav2.yaml xy_goal_tolerance
DOCK = {"x_m": 1.95, "y_m": 1.35, "theta": 90.0, "source": "start"}


class Body:
    """A pose, verbs as `RosDriveRobot` carries them out: a straight verb is
    whole 0.30 m moves (at least one), a turn is exact. Time passes per verb."""

    def __init__(self, x, y, heading, quantised=True):
        self.x, self.y, self.h = x, y, heading
        self.t = 0.0
        self.quantised = quantised
        self.verbs = []

    def pose(self):
        return {"usable": True, "x_m": self.x, "y_m": self.y, "heading_deg": self.h % 360.0}

    def check_and_execute(self, action, speed=50, duration=0.5, angle=90):
        self.verbs.append((action, round(self.x, 4), round(self.y, 4), round(self.h % 360, 1)))
        if action in ("FORWARD", "REVERSE"):
            v = speed / 100.0 * 2.0 * MOVE_M
            d = v * duration
            if self.quantised:
                d = max(1, round(d / MOVE_M)) * MOVE_M
            sign = 1.0 if action == "FORWARD" else -1.0
            fx, fy = ahead(self.h)
            self.x += sign * d * fx
            self.y += sign * d * fy
            self.t += d / 0.12 + 0.5
        else:
            self.h += angle if action == "RIGHT" else -angle
            self.t += abs(angle) / 40.0 + 0.5
        return {"ok": True}


class Nav2:
    """nav2 in miniature: a goal drives the robot straight at it and is
    `succeeded` the moment the robot is within the goal checker's radius
    (`stop_m`), the robot then facing the way it drove."""

    def __init__(self, body, stop_m=NAV2_XY_TOL_M):
        self.body, self.stop_m = body, stop_m
        self.goal = None
        self.sent = []

    def set_goal(self, x_m, y_m):
        self.sent.append((round(x_m, 4), round(y_m, 4)))
        self.goal = {"x_m": x_m, "y_m": y_m, "state": "active"}
        return {"accepted": True, "goal_id": f"g{len(self.sent)}"}

    def get_goal(self):
        g, b = self.goal, self.body
        if g and g["state"] == "active":
            dx, dy = g["x_m"] - b.x, g["y_m"] - b.y
            d = math.hypot(dx, dy)
            if d > self.stop_m:
                b.h = math.degrees(math.atan2(dx, -dy)) % 360.0
                b.x += dx * (d - self.stop_m) / d
                b.y += dy * (d - self.stop_m) / d
                b.t += (d - self.stop_m) / 0.2
            g["state"] = "succeeded"
        return {"goal": g, "plan": []}

    def cancel_goal(self):
        if self.goal:
            self.goal["state"] = "canceled"
        return {"cancelled": True}


def trip(body, nav):
    rt = ReturnTrip(DOCK, nav, body, body.pose, lambda: body.t, idle=None)
    for _ in range(2000):
        out = rt.step()
        if out is not None:
            return out, rt
    raise AssertionError("the return never ended")


def err(body):
    return (math.hypot(body.x - DOCK["x_m"], body.y - DOCK["y_m"]),
            abs(wrap_deg(body.h - DOCK["theta"])))


def test_c1_replayed_docks_instead_of_pivoting_for_40_verbs():
    """c-1: home from the east, nav2 stops 0.104 m short of the staging point
    on the dock's side; the creep's 0.30 m leaves the robot 0.1025 m past the
    dock. On dev: 40 verbs, every one after the creep a 180-degree pivot at
    one place, then `not_aligned`."""
    body = Body(6.2, 1.45, 270.0)
    nav = Nav2(body, stop_m=0.104)
    (state, reason), rt = trip(body, nav)
    assert state == dk.DOCKED, (state, reason, body.verbs[-6:])
    m, deg = err(body)
    assert m <= DOCK_TOL_M and deg <= DOCK_TOL_DEG
    assert rt.actions < 10, rt.actions


def test_no_pivot_cycle_at_one_place():
    """Whatever the residual, the approach never turns more than twice from
    one spot without moving (dev: 38 turns at (2.0525, 1.3439))."""
    for stop in (0.0, 0.03, 0.06, 0.09, 0.104, 0.12, 0.149):
        body = Body(6.2, 1.45, 270.0)
        (state, reason), rt = trip(body, Nav2(body, stop_m=stop))
        runs, last, n = [], None, 0
        for a, x, y, h in body.verbs:
            if a in ("LEFT", "RIGHT"):
                n = n + 1 if last == (x, y) else 1
                last = (x, y)
                runs.append(n)
            else:
                last, n = None, 0
        assert max(runs, default=0) <= 2, (stop, state, reason, body.verbs)


@pytest.mark.parametrize("stop", [0.0, 0.02, 0.05, 0.08, 0.095, 0.10, 0.104, 0.11])
def test_every_nav2_stop_within_its_tolerance_docks(stop):
    """nav2's checker stops anywhere inside its 0.10 m (and a few cm past it
    under braking): the return docks within 3.156's bars every time."""
    body = Body(6.2, 1.45, 270.0)
    (state, reason), _ = trip(body, Nav2(body, stop_m=stop))
    assert state == dk.DOCKED, (stop, state, reason)
    m, deg = err(body)
    assert m <= DOCK_TOL_M and deg <= DOCK_TOL_DEG, (stop, m, deg)
    # And not by margin: whole moves leave nav2's stop error, which D2 closes.
    assert m <= dk.FINE_ABOVE_M + 1e-9, (stop, m)


def test_continuous_verbs_are_unchanged():
    """A body whose verbs move what they are asked (the car under `drive:
    direct`, the in-process sweeps' `exact_verbs`): the creep closes the
    residual itself and no extra goal is sent."""
    body = Body(6.2, 1.45, 270.0, quantised=False)
    nav = Nav2(body, stop_m=0.104)
    (state, _), _ = trip(body, nav)
    assert state == dk.DOCKED
    assert len(nav.sent) == 1


def test_a_person_who_takes_the_robot_during_the_correcting_goal_still_wins():
    """Q6: the correcting goal is a nav2 goal through the gated navigator; a
    person's refusal of it (`Preempted`, raised for a MANUAL holder) ends the
    return there, for the runner to read as `cancelled` -- never retried."""
    from robot.interface import Preempted

    class Taken(Nav2):
        def set_goal(self, x_m, y_m):
            if self.sent:
                self.sent.append((x_m, y_m))
                raise Preempted("twin-dpad is driving -- brain is lower priority and was refused")
            return super().set_goal(x_m, y_m)

    body = Body(6.2, 1.45, 270.0)
    nav = Taken(body, stop_m=0.104)
    rt = ReturnTrip(DOCK, nav, body, body.pose, lambda: body.t, idle=None)
    with pytest.raises(Preempted):
        for _ in range(2000):
            assert rt.step() is None
    assert len(nav.sent) == 2


def test_a_correcting_goal_never_taken_ends_with_its_numbers():
    """nav2 holds the correcting goal (refused) past the wait: the return
    ran, so it is `failed`, `not_aligned` with the error -- not `skipped`."""
    class Holds(Nav2):
        def set_goal(self, x_m, y_m):
            if self.sent:
                self.sent.append((x_m, y_m))
                self.body.t += 1.0
                return {"accepted": False, "reason": "preempted"}
            return super().set_goal(x_m, y_m)

    body = Body(6.2, 1.45, 270.0)
    (state, reason), _ = trip(body, Holds(body, stop_m=0.104))
    assert state == dk.FAILED and reason.startswith("not_aligned: 0.10"), reason


def test_a_correcting_goal_that_ends_off_target_is_bounded():
    """A correcting goal that nav2 ends at once with the robot unmoved: the
    residual ends the return after FINE_TRIES goals, with its numbers, never
    after 40 verbs."""
    class Frozen(Nav2):
        def get_goal(self):
            if len(self.sent) > 1 and self.goal["state"] == "active":
                self.goal["state"] = "succeeded"   # ends at once, the robot unmoved
                return {"goal": self.goal, "plan": []}
            return super().get_goal()

    body = Body(6.2, 1.45, 270.0)
    nav = Frozen(body, stop_m=0.104)
    (state, reason), rt = trip(body, nav)
    assert state == dk.FAILED and reason.startswith("not_aligned: 0.10"), reason
    assert len(nav.sent) == 1 + dk.FINE_TRIES
    assert rt.actions < 12
