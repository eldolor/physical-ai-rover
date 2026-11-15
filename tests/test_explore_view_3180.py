# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
3.180.A: a view goal's look-from spot may be used again for a patch in
another direction (more than `VIEW_REUSE_DEG` from what the camera faced
there); the den guard -- one patch whose centre drifts, seen from one spot in
one direction -- still gets one view (bar A7). `ExploreAgent.view_reuses`
counts the reuses (A6).
"""
import math

from brain.explore import VIEW_REUSE_DEG, ExploreAgent
from brain.frontier import CAMERA_FOV_DEG, ViewGap
from brain.memory import MissionMemory
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav


def _agent():
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    return ExploreAgent(robot, MissionMemory(mission="m", target_object="red backpack"),
                        navigator=FakeNav(robot), clock=lambda: grid.sim_time,
                        world=MockWorld(grid))


def _gap(goal, centre, face=None):
    return ViewGap(centre=centre, goal=goal, distance_m=1.0, area_m2=0.5, face=face)


def test_the_reuse_angle_is_half_the_camera_field():
    assert VIEW_REUSE_DEG == CAMERA_FOV_DEG / 2


def test_the_den_case_one_patch_drifting_from_one_spot_gets_one_view():
    """The den run: the patch's centre moves as the camera sees more of it,
    each time past the looked-at radius, but always the same way from the
    same spot. One view, then refused -- as before 3.180."""
    a = _agent()
    spot = (2.0, 2.0)
    first = _gap(spot, (3.0, 2.0))
    assert a._view_allowed(first)
    a._note_view(first)
    a._looked.append(first.centre)
    for k in range(1, 12):                       # drifts 0.7 m a time, east and a little north
        drifted = _gap((spot[0] + 0.05, spot[1]), (3.0 + 0.7 * k, 2.0 + 0.1 * k))
        assert not a._view_allowed(drifted), k
    assert a.view_reuses == 0


def test_another_direction_from_the_same_spot_is_allowed():
    """3.176.B S7's 195 R: a spot used to look east may look south."""
    a = _agent()
    spot = (2.0, 2.0)
    east = _gap(spot, (3.0, 2.0))
    a._note_view(east)
    a._looked.append(east.centre)
    south = _gap((2.1, 2.0), (2.1, 0.8))          # 90 deg away, 0.1 m from the spot
    near_east = _gap((2.1, 2.0), (3.2, 2.5))      # about 24 deg away: the same look
    assert a._view_allowed(south)
    assert not a._view_allowed(near_east)
    a._note_view(south)
    assert a.view_reuses == 1


def test_a_patch_already_looked_at_is_refused_whatever_the_direction():
    a = _agent()
    a._looked.append((3.0, 2.0))
    assert not a._view_allowed(_gap((5.0, 5.0), (3.2, 2.1)))


def test_the_face_is_where_the_camera_looked():
    """A gap's `face` (the part seen from its goal) is the direction kept,
    as the agent faces it on arrival."""
    a = _agent()
    g = _gap((2.0, 2.0), (2.0, 4.0), face=(3.0, 2.0))   # centre north, face east
    a._note_view(g)
    assert not a._view_allowed(_gap((2.0, 2.0), (3.5, 2.2)))           # east again
    assert a._view_allowed(_gap((2.0, 2.0), (2.0, 0.5)))               # south
    assert math.isclose(a._view_looks[0][2], 0.0, abs_tol=1e-9)
