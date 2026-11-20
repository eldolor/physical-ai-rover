# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.196 (G9): explore's goals beside a no-go zone.

The map is 3.196's trace in miniature (0.3 m cells, as the simulator's): a
wall across y = 1.2-1.5 m with a three-cell door (x 1.2-2.1 m), the robot in
the room above, and a zone filling the cells just below the door, as `z20`
filled `procthor:val:15`'s. No navigator can pass: the gaps between the
zone's corners and the wall beside the door have no width.

    6 ##########
    5 #...R....#
    4 ####...###     the door
    3 #...ZZZ..#
    2 #...ZZZ..#
    1 #...ZZZ..#
    0 ##########
"""

from brain.explore import ExploreAgent, NEAR_ZONE_M
from brain.frontier import reach
from brain.memory import MissionMemory
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests.test_explore import FakeNav
from world import nogo
from world.interface import CELL_FREE, CELL_OCCUPIED

RES, W, H = 0.3, 10, 7
ROBOT = (1.65, 1.65)
ZONE = {"id": "z196", "x0": 1.201, "y0": 0.301, "x1": 2.099, "y1": 1.199,
        "kind": "area", "set_at": 0.0, "by": "person"}


def _map():
    cells = []
    for j in range(H):
        for i in range(W):
            wall = j in (0, H - 1) or i in (0, W - 1) or (j == 4 and not 4 <= i <= 6)
            cells.append(CELL_OCCUPIED if wall else CELL_FREE)
    return {"usable": True, "width": W, "height": H, "resolution_m": RES,
            "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": cells,
            "map_id": "m196", "map_version": 1}


class _World:
    def __init__(self):
        self.m = _map()

    def get_map(self):
        return self.m

    def get_pose(self):
        return {"usable": True, "x_m": ROBOT[0], "y_m": ROBOT[1], "heading_deg": 0.0,
                "frame_id": "map-196"}


def _agent(zones=(ZONE,)):
    robot = MockRobot(build_world("scaled_house"), render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=FakeNav(robot), clock=lambda: 0.0, world=_World())
    agent.set_zones([nogo.clean_zone(z) for z in zones])
    return agent


def _below_door(rch):
    return sorted(c for c in rch.steps if c[1] < 4)


def test_a_door_filled_by_a_zone_reaches_nothing_behind_it():
    """G9 (a), red on `dev`: its 8-connected reach stepped diagonally from a
    door cell past the zone's corner, (4, 4) -> (3, 3), between the wall
    (3, 4) and the zone cell (4, 3)."""
    agent = _agent()
    m = agent.world.get_map()
    masked = agent._zone_map(m, agent.zones)
    rch = reach(masked, *ROBOT)
    assert _below_door(rch) == []
    # The room above, and the door itself away from its corners, still reach.
    assert (5, 4) in rch.steps and (1, 5) in rch.steps and (8, 5) in rch.steps


def test_without_the_zone_the_room_behind_the_door_reaches():
    """The control: the same door with no zone leads to the room below."""
    rch = reach(_map(), *ROBOT)
    assert (2, 2) in rch.steps and (5, 2) in rch.steps


def test_a_side_in_the_grown_band_only_is_no_pinch():
    """procthor:val:9 (3.196.B): a cell in the zone's 0.12 m band whose centre
    is outside the zone is passable to a navigator (nav2's keepout is lethal
    to the robot's centre only), so the diagonal past it stays open. Here the
    zone's west edge sits 0.10 m from cell 4's centre: (4, 3) is band, not
    zone, and (3, 3) still reaches; (6, 3) is zone, so (7, 3) does not."""
    agent = _agent(zones=(dict(ZONE, x0=1.45),))
    masked = agent._zone_map(agent.world.get_map(), agent.zones)
    rch = reach(masked, *ROBOT)
    assert (3, 3) in rch.steps
    assert (7, 3) not in rch.steps


def test_close_pinches_leaves_a_pinch_between_walls_as_it_is():
    """D1 touches only a pinch beside a zone: two walls meeting at a corner
    (an old, harmless case in the simulator's cell-aligned houses) stay."""
    m = {"usable": True, "width": 2, "height": 2, "resolution_m": RES,
         "origin_x_m": 0.0, "origin_y_m": 0.0,
         "cells": [CELL_FREE, CELL_OCCUPIED, CELL_OCCUPIED, CELL_FREE]}
    far = dict(ZONE, x0=5.0, y0=5.0, x1=5.6, y1=5.6)
    assert nogo.close_pinches(m, [far], nogo.CELL_NOGO, free=CELL_FREE,
                              occupied=CELL_OCCUPIED) is m
    on = dict(ZONE, x0=0.31, y0=0.01, x1=0.59, y1=0.29)    # cell (1, 0)'s centre inside
    m2 = dict(m, cells=[CELL_FREE, nogo.CELL_NOGO, CELL_OCCUPIED, CELL_FREE])
    out = nogo.close_pinches(m2, [on], nogo.CELL_NOGO, free=CELL_FREE, occupied=CELL_OCCUPIED)
    assert out["cells"] == [nogo.CELL_NOGO] * 2 + [CELL_OCCUPIED, nogo.CELL_NOGO]
    # A zone drawn over a wall's corner takes no floor there: still walls only.
    over = dict(ZONE, x0=0.31, y0=0.01, x1=0.89, y1=0.59)   # covers cell (1, 0) and (1, 1)
    m3 = dict(m, cells=[CELL_FREE, CELL_OCCUPIED, CELL_OCCUPIED, CELL_FREE])
    assert nogo.close_pinches(m3, [over], nogo.CELL_NOGO, free=CELL_FREE,
                              occupied=CELL_OCCUPIED) is m3


def test_a_zone_over_a_wall_corner_closes_no_pinch_on_the_real_path():
    """Thermos review: `masked_map` writes the zone value over a wall inside
    the zone, so `_zone_map` passes the floor map for the walls. Here the
    door's west corner is a pinch of walls only -- (3, 4) the wall, (4, 3) a
    box -- with the free pair (4, 4) and (3, 3); a zone over the wall cell
    (3, 4) alone takes no floor there, so the pair stays free."""
    m = _map()
    m["cells"][3 * W + 4] = CELL_OCCUPIED          # (4, 3)
    over = dict(ZONE, x0=0.91, y0=1.21, x1=1.19, y1=1.49)   # covers wall cell (3, 4) only
    agent = _agent(zones=(over,))
    masked = agent._zone_map(m, agent.zones)
    assert masked["cells"][4 * W + 3] == nogo.CELL_NOGO      # the zone's own cell
    assert masked.get("pinch_cells") is None
    assert masked["cells"][3 * W + 3] == CELL_FREE and masked["cells"][4 * W + 4] == CELL_FREE


def test_close_pinches_closes_both_ends_and_keeps_the_type():
    m = _map()
    masked = nogo.masked_map(m, [ZONE], nogo.CELL_NOGO, grow=nogo.HALF_WIDTH_M)
    out = nogo.close_pinches(masked, [ZONE], nogo.CELL_NOGO, free=CELL_FREE,
                             occupied=CELL_OCCUPIED)
    assert out is not masked and isinstance(out["cells"], list)
    shut = {(i % W, i // W) for i, v in enumerate(out["cells"])
            if v == nogo.CELL_NOGO and masked["cells"][i] != nogo.CELL_NOGO}
    assert {(3, 3), (4, 4), (6, 4), (7, 3)} <= shut
    assert masked["cells"][3 * W + 3] == CELL_FREE       # the input is unchanged


def _abort(agent, goal):
    agent._goal = {"x_m": goal[0], "y_m": goal[1], "kind": "frontier", "sent_at": 0.0,
                   "from": ROBOT}
    agent._goal_ended("aborted", 1.0)


def test_an_unmoved_abort_beside_a_zone_is_charged_at_once():
    """G9 (b), red on `dev`: there the first unmoved abort at a place is
    forgiven as a wedge, so val:15's three places round `z20` cost 5 aborts."""
    agent = _agent()
    goal = (2.25, 0.75)                      # 0.15 m east of the zone
    assert nogo.distance(ZONE, *goal) <= NEAR_ZONE_M
    _abort(agent, goal)
    assert agent.retry.failures_near(*goal) == 1


def test_an_unmoved_abort_far_from_zones_is_still_forgiven_once():
    agent = _agent()
    goal = (2.55, 1.65)                      # in the robot's room, 0.67 m from the zone
    assert nogo.distance(ZONE, *goal) > NEAR_ZONE_M
    _abort(agent, goal)
    assert agent.retry.failures_near(*goal) == 0
    _abort(agent, goal)                      # the same place again: now charged
    assert agent.retry.failures_near(*goal) == 1


def test_without_zones_an_unmoved_abort_is_forgiven_once():
    agent = _agent(zones=())
    _abort(agent, (2.25, 0.75))
    assert agent.retry.failures_near(2.25, 0.75) == 0
