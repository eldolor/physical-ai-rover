# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.173.B: a survey's goal into a room of its map where no goal at the goal
clearance can stand (docs/plans/ros-alignment/3.173-surveys-small-rooms.md)."""

import math

from brain.explore import SMALL_ROOM_CLEARANCE_M, leave_clearance_m
from brain.frontier import reach
from control.mission_runner import MissionRunner
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from world.interface import CELL_FREE, CELL_OCCUPIED

RES = 0.05


def _map():
    """A 3 m room (x 0.1-3.1 m), a 0.45 m door in its east wall, and east of it a
    room 0.55 m wide (y) by 1.5 m: its middle cells keep 0.30 m, so no goal at the
    goal clearance (7 cells, 0.35 m) stands in it, and one at 0.25 m does."""
    w, h = 100, 70
    cells = [[CELL_OCCUPIED] * w for _ in range(h)]

    def free(x0, x1, y0, y1):
        for j in range(int(round(y0 / RES)), int(round(y1 / RES))):
            for i in range(int(round(x0 / RES)), int(round(x1 / RES))):
                cells[j][i] = CELL_FREE
    free(0.10, 3.10, 0.10, 3.10)                  # the big room
    free(3.10, 3.20, 1.375, 1.825)                # the door, 0.45 m
    free(3.20, 4.70, 1.325, 1.875)                # the small room, 0.55 m
    return {"usable": True, "resolution_m": RES, "width": w, "height": h,
            "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": [c for row in cells for c in row]}


def _agent(target=None):
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = FakeNav(robot)
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=MockWorld(grid),
                           target_object=target, **({} if target else {"job": "survey"}))
    runner.start()
    runner.agent.fake_nav = nav           # the runner wraps the navigator it is given
    return runner.agent


POSE = {"usable": True, "x_m": 1.5, "y_m": 1.5, "heading_deg": 0.0}


def _goal(agent, m=None, pose=POSE):
    m = m or _map()
    rch = reach(m, pose["x_m"], pose["y_m"], clearance_m=leave_clearance_m(agent.safety.min_distance_cm))
    return agent._small_room_goal(m, pose, rch, agent.clock())


def test_a_survey_sends_one_goal_into_the_room_no_goal_can_stand_in():
    agent = _agent()
    out = _goal(agent)
    assert out is not None and out[0] == "GOAL"
    g = agent._goal
    assert g["kind"] == "small_room"
    assert 3.2 <= g["x_m"] <= 4.7 and 1.325 <= g["y_m"] <= 1.875
    m = _map()                                       # it keeps the small-room clearance
    i, j = int(g["x_m"] / RES), int(g["y_m"] / RES)
    near = [(a, b) for a in range(i - 6, i + 7) for b in range(j - 6, j + 7)
            if m["cells"][b * m["width"] + a] == CELL_OCCUPIED]
    assert min(math.hypot((a - i) * RES, (b - j) * RES) for a, b in near) >= SMALL_ROOM_CLEARANCE_M - RES


def test_each_room_once():
    agent = _agent()
    assert _goal(agent)[0] == "GOAL"
    agent._goal, agent._small_at = None, None
    assert _goal(agent) is None


def test_not_into_a_room_it_has_stood_in():
    agent = _agent()
    agent._stood.add((round(4.0 / 0.10), round(1.6 / 0.10)))
    assert _goal(agent) is None


def test_a_find_sends_none():
    assert _goal(_agent("washer")) is None


def test_the_cut_runs_at_most_every_small_room_s():
    agent = _agent()
    agent._small_at = agent.clock()
    assert _goal(agent) is None


def test_a_gap_between_furniture_is_no_room():
    """A piece under SMALL_ROOM_MIN_M2 (in-process the dining chairs' gaps were
    0.09-0.63 m2): no goal."""
    m = _map()
    for j in range(m["height"]):
        for i in range(int(round(4.20 / RES)), int(round(4.70 / RES))):
            m["cells"][j * m["width"] + i] = CELL_OCCUPIED      # the small room now 1.0 m long
    assert _goal(_agent(), m) is None


def test_review_a_room_the_retry_book_holds_waits():
    """Review: a small-room goal that failed (or a place set aside by earlier
    missions) waits as any goal's would."""
    agent = _agent()
    agent.retry.radius_m = 1.0
    agent.retry.set_aside.append((4.0, 1.6, 0))
    assert _goal(agent) is None


def test_review_a_refused_goal_is_given_up_after_its_tries():
    from brain.explore import SMALL_ROOM_REFUSED
    agent = _agent()
    agent.fake_nav.refuse = 100
    for _ in range(SMALL_ROOM_REFUSED):
        agent._small_at = None
        assert _goal(agent)[0] == "WAIT"
    agent._small_at = None
    assert _goal(agent) is None


def test_review_a_whole_survey_sends_one_goal_into_each_small_room():
    """Review: through the runner and the chooser (the call, the track, the look),
    home_first_floor's survey goes into its pantry and laundry, once each."""
    from tests import survey_sweep as sw
    grid = build_world("home_first_floor")
    (cx, cy), theta, _ = sw.starts("home_first_floor")[0]
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    truth = build_world("home_first_floor")
    room = {c: n for n, cs in sorted(truth.rooms.items(), key=lambda kv: -len(kv[1])) for c in cs}
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, job="survey", policy="explore", navigator=FakeNav(robot),
                           world=MockWorld(grid), mission_time_limit_s=sw.TIME_LIMIT_S,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    for _ in range(sw.TICK_CAP):
        if not runner.tick():
            break
    events = runner.agent.events
    sent = [e for e in events if e.get("event") == "goal_sent" and e.get("kind") == "small_room"]
    rooms = [room.get((int(e["goal"][0] / 0.3), int(e["goal"][1] / 0.3))) for e in sent]
    assert rooms.count("pantry") == 1 and rooms.count("laundry") == 1, rooms
    assert len(runner.agent._stood) >= 20       # the track a room "stood in" is read from (FakeNav jumps)
    assert any(e["state"] == "succeeded" for e in events
               if e.get("event") == "goal_ended" and e.get("kind") == "small_room")
