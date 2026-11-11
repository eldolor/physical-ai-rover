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
    """A refusal other than the mission's own hold (3.172.E) counts."""
    from brain.explore import SMALL_ROOM_REFUSED
    agent = _agent()
    agent.fake_nav.set_goal = lambda x, y: {"accepted": False, "reason": "rejected"}
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


def _hall_map():
    """Amendment 3: shaped as SLAM's map holds laundry and pantry
    (`b_slam_map.py` on `picar-maps-3172e`). A 3 m room, a 0.85 m hall north
    from it whose middle holds goal stops, and off the hall two side rooms
    0.55 m by 1.6 m through 0.55 m doors at their ends. `segment()` cuts the
    room, the hall and both side rooms as ONE piece (the doors are no necks),
    and that piece holds goal stops."""
    from world.interface import CELL_UNKNOWN
    w, h = 90, 110
    cells = [[CELL_UNKNOWN] * w for _ in range(h)]

    def box(x0, x1, y0, y1, v):
        for j in range(int(round(y0 / RES)), int(round(y1 / RES))):
            for i in range(int(round(x0 / RES)), int(round(x1 / RES))):
                cells[j][i] = v
    box(0.0, 4.5, 0.0, 5.5, CELL_OCCUPIED)
    box(0.7, 3.7, 0.1, 2.1, CELL_FREE)               # the room
    hx0, hx1 = 1.775, 2.625                          # the hall
    box(hx0, hx1, 2.1, 5.4, CELL_FREE)
    box(hx0 - 0.65, hx0 - 0.10, 3.2, 4.8, CELL_FREE)  # west side room
    box(hx1 + 0.10, hx1 + 0.65, 3.2, 4.8, CELL_FREE)  # east side room
    box(hx0 - 0.10, hx0, 3.25, 3.80, CELL_FREE)       # their doors
    box(hx1, hx1 + 0.10, 3.25, 3.80, CELL_FREE)
    return {"usable": True, "resolution_m": RES, "width": w, "height": h,
            "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": [c for row in cells for c in row]}


HALL_POSE = {"usable": True, "x_m": 2.2, "y_m": 1.0, "heading_deg": 90.0}


def test_amendment3_two_small_rooms_in_one_piece_with_goal_stops():
    from brain.room_labels import segment
    m = _hall_map()
    seg, _ = segment(m)
    piece = {int(seg[int(y / RES), int(x / RES)]) for x, y in ((2.2, 1.0), (2.2, 4.0), (1.4, 4.4), (3.0, 4.4))}
    assert len(piece) == 1                         # as on SLAM's map: one piece
    agent = _agent()
    sent = []
    for _ in range(4):
        out = _goal(agent, m, HALL_POSE)
        if out is None:
            break
        sent.append((agent._goal["x_m"], agent._goal["y_m"]))
        agent._goal, agent._small_at = None, None
    sides = sorted("west" if x < 1.775 else "east" if x > 2.625 else "hall" for x, _ in sent)
    assert sides == ["east", "west"], sent
    for x, y in sent:
        assert 3.2 <= y <= 4.8


def test_amendment3_review_a_region_is_tried_even_once_its_goal_cell_is_reached():
    """Review: a map that grew so that a stop now reaches the sent goal's own
    cell leaves the rest of that region; it is still the region tried."""
    m = _hall_map()
    agent = _agent()
    assert _goal(agent, m, HALL_POSE)[0] == "GOAL"
    gx, gy = agent._goal["x_m"], agent._goal["y_m"]
    agent._goal, agent._small_at = None, None
    real = agent._small_regions

    def grown(m2, seg, rch):
        reg = real(m2, seg, rch)
        i, j = int(gx / RES), int(gy / RES)
        reg[j - 2:j + 3, i - 2:i + 3] = -1           # the goal's own cells now reached
        return reg
    agent._small_regions = grown
    out = _goal(agent, m, HALL_POSE)
    assert out is not None and out[0] == "GOAL"
    assert (agent._goal["x_m"] < 1.775) != (gx < 1.775)   # the other side room, not this one again
    agent._goal, agent._small_at = None, None
    assert _goal(agent, m, HALL_POSE) is None


def _back_to_back_map():
    """Review round 2: west of the hall, two side rooms 0.55 m by 1.475 m back to
    back along the hall, a 0.05 m wall between them, each with its own 0.55 m
    door at its far end."""
    w, h = 90, 110
    cells = [[CELL_OCCUPIED] * w for _ in range(h)]

    def box(x0, x1, y0, y1, v):
        for j in range(int(round(y0 / RES)), int(round(y1 / RES))):
            for i in range(int(round(x0 / RES)), int(round(x1 / RES))):
                cells[j][i] = v
    box(0.7, 3.7, 0.1, 2.1, CELL_FREE)
    hx0, hx1 = 1.775, 2.625
    box(hx0, hx1, 2.1, 5.4, CELL_FREE)
    box(hx0 - 0.65, hx0 - 0.10, 2.3, 3.75, CELL_FREE)     # south room
    box(hx0 - 0.65, hx0 - 0.10, 3.80, 5.3, CELL_FREE)     # north room, 0.05 m wall between
    box(hx0 - 0.10, hx0, 2.3, 2.85, CELL_FREE)            # their doors, far ends
    box(hx0 - 0.10, hx0, 4.75, 5.3, CELL_FREE)
    return {"usable": True, "resolution_m": RES, "width": w, "height": h,
            "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": [c for row in cells for c in row]}


def test_amendment3_review2_a_tried_goal_never_marks_the_room_behind_a_wall():
    """Review round 2: a goal sent 0.15 m south of a 0.05 m wall (0.20 m from the
    room behind it, inside the reach radius in a straight line) leaves that room
    untried: the reach is measured through floor connected to the goal."""
    from brain.room_labels import segment
    m = _back_to_back_map()
    agent = _agent()
    rch = reach(m, HALL_POSE["x_m"], HALL_POSE["y_m"],
                clearance_m=leave_clearance_m(agent.safety.min_distance_cm))
    seg, _ = segment(m)
    reg = agent._small_regions(m, seg, rch)
    south, north = int(reg[int(3.0 / RES), int(1.4 / RES)]), int(reg[int(4.6 / RES), int(1.4 / RES)])
    assert south >= 0 and north >= 0 and south != north
    agent._small_tried = [(1.4, 3.6)]                      # in the south room, by the wall
    sent = []
    while (out := _goal(agent, m, HALL_POSE)) is not None:
        sent.append(int(reg[int(agent._goal["y_m"] / RES), int(agent._goal["x_m"] / RES)]))
        agent._goal, agent._small_at = None, None
    assert sent == [north], sent


def test_3172e_a_held_refusal_is_sent_again_uncounted_then_counts_past_its_wait():
    from brain.explore import SMALL_ROOM_HELD_S
    agent = _agent()
    t = [100.0]
    agent.clock = lambda: t[0]
    agent.fake_nav.refuse = 1000
    first = _goal(agent)
    assert first[0] == "WAIT" and agent._small_held is not None
    place = (round(agent._small_held[0][0], 1), round(agent._small_held[0][1], 1))
    for _ in range(5):                       # sent again at once, no cut between, uncounted
        t[0] += 1.0
        assert _goal(agent)[0] == "WAIT"
    assert agent._small_refused[place] == 0
    t[0] = 100.0 + SMALL_ROOM_HELD_S + 0.1   # held past its wait: one refusal
    assert _goal(agent)[0] == "WAIT"
    assert agent._small_refused[place] == 1 and agent._small_held is None
    agent.fake_nav.refuse = 0
    agent._small_at = None
    assert _goal(agent)[0] == "GOAL"


class _HeldNav(FakeNav):
    """3.172.E live: the robot server refuses a goal `preempted` while the
    mission's own last verb still holds authority (it lapses on silence). Every
    goal kind met it (frontier 899 times over 7 surveys); this double refuses
    each place's first HELD sends so."""
    HELD = 5

    def __init__(self, robot):
        super().__init__(robot)
        self.held = {}

    def set_goal(self, x_m, y_m):
        place = (round(x_m, 1), round(y_m, 1))
        self.held[place] = self.held.get(place, 0) + 1
        if self.held[place] <= self.HELD:
            return {"accepted": False, "reason": "preempted"}
        return super().set_goal(x_m, y_m)


def test_3172e_a_held_refusal_is_waited_through_not_counted():
    """3.172.E: live, 30 of 30 small-room goals were refused `preempted` and the
    room given up after SMALL_ROOM_REFUSED; every other kind waits through that
    refusal, and so must this one."""
    from tests import survey_sweep as sw
    grid = build_world("home_first_floor")
    (cx, cy), theta, _ = sw.starts("home_first_floor")[0]
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    truth = build_world("home_first_floor")
    room = {c: n for n, cs in sorted(truth.rooms.items(), key=lambda kv: -len(kv[1])) for c in cs}
    robot = MockRobot(grid, render=False)
    runner = MissionRunner(robot, job="survey", policy="explore", navigator=_HeldNav(robot),
                           world=MockWorld(grid), mission_time_limit_s=sw.TIME_LIMIT_S,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    for _ in range(sw.TICK_CAP):
        if not runner.tick():
            break
    sent = [e for e in runner.agent.events if e.get("event") == "goal_sent" and e.get("kind") == "small_room"]
    rooms = [room.get((int(e["goal"][0] / 0.3), int(e["goal"][1] / 0.3))) for e in sent]
    assert rooms.count("pantry") == 1 and rooms.count("laundry") == 1, rooms


def _sides(agent, m):
    sent = []
    for _ in range(4):
        out = _goal(agent, m, HALL_POSE)
        if out is None:
            break
        x = agent._goal["x_m"]
        sent.append("west" if x < 1.775 else "east" if x > 2.625 else "hall")
        agent._goal, agent._small_at = None, None
    return sent


def test_thermos1_a_room_stood_in_stays_stood_in_once_a_stop_reaches_where_it_stood():
    """Thermos 1: the robot stood in the west side room; a grown map's stops
    now reach that cell (-1 in today's regions). The rest of the room is still
    the room it stood in, never a goal."""
    m = _hall_map()
    agent = _agent()
    agent._stood.add((round(1.4 / 0.10), round(4.4 / 0.10)))
    assert _sides(agent, m) == ["east"]              # stood in the west room
    real = agent._small_regions

    def grown(m2, seg, rch):
        reg = real(m2, seg, rch)
        i, j = int(1.4 / RES), int(4.4 / RES)
        reg[j - 2:j + 3, i - 2:i + 3] = -1           # where it stood, now reached
        return reg
    agent._small_regions = grown
    assert _sides(agent, m) == []


def test_thermos1_review_passing_a_room_s_door_is_not_standing_in_it():
    """Review of the Thermos 1 fix: the robot's path is mostly cells a stop
    reaches (-1); standing in the hall 0.2 m from the west room's door never
    marks that room stood in."""
    m = _hall_map()
    agent = _agent()
    agent._stood.add((round(1.9 / 0.10), round(3.5 / 0.10)))
    assert sorted(_sides(agent, m)) == ["east", "west"]


def test_thermos2_a_tried_goal_whose_cell_is_no_longer_free_still_marks_its_room():
    """Thermos 2: the map has shifted under a goal sent before, so its point
    now lies on the wall cell beside the west room (free from x 1.10 m); it still
    marks the room it was sent into."""
    m = _hall_map()
    agent = _agent()
    i, j = int(1.07 / RES), int(4.0 / RES)
    assert m["cells"][j * m["width"] + i] == CELL_OCCUPIED
    agent._small_tried = [(1.07, 4.0)]
    assert _sides(agent, m) == ["east"]
