# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.159 B (`docs/plans/ros-alignment/3.159-drop-detection.md`): no fall at
a stair. The depth camera's `drop` block, folded into a floor memory
(`robot/drop_guard.py`), is a wall at the end of seen floor, in series
with the cone, the corridor and the floor band; unseen floor is never
clear. Judged on ground truth (`tests/drop_sweep.py`)."""

import math

import pytest

from robot import drop_guard
from robot.interface import ZONE_UNUSABLE
from robot.safety import SafetyController
from sim.maps import build_world
from sim.mock_robot import WHEEL_RADIUS_M, MockRobot
from tests.drop_sweep import drive, drop_truth, world_of

# A room with a stairwell across its far end (rows 8-9), the robot facing it.
ROOM = (["############"]
        + ["#..........#"] * 7
        + ["#vvvvvvvvvv#"] * 2
        + ["#..........#", "############"])
DOWN = math.pi / 2          # theta grows clockwise on the y-down grid: +y


def _robot(x=6.0, y=4.0, theta=DOWN, layout=ROOM, sensor=None):
    world = world_of(layout, x, y, theta)
    return world, MockRobot(world, render=False, sensor=sensor, drop_sensor=True)


@pytest.mark.parametrize("speed", [0.1, 0.2, 0.6])
def test_a_stairwell_ahead_stops_the_robot_short_of_it(speed):
    world, robot = _robot()
    out = drive(world, speed, 30.0, robot=robot)
    assert not out["fell"] and out["overhang"] == 0.0, out
    assert out["gap"] >= 15.0, out
    assert "drop_edge" in " ".join(out["reasons"]), out


def test_without_the_rule_the_same_run_falls():
    """The red half: the lidar and the cone see open space."""
    world, robot = _robot()
    assert drive(world, 0.2, 30.0, robot=robot, rule=False)["fell"]


def test_reverse_toward_floor_never_seen_is_refused():
    world, robot = _robot(y=3.0, theta=-DOWN)       # facing away from the stairwell
    safety = SafetyController(robot, 20.0)
    clearance, source = safety.reverse_clearance()
    assert source == "drop_unseen_astern" and clearance <= 20.0, (clearance, source)


def test_reverse_over_floor_it_drove_is_allowed():
    world, robot = _robot(y=2.5)
    drive(world, 0.2, 2.5, robot=robot)              # 50 cm toward the stairwell, nowhere near it
    clearance, source = SafetyController(robot, 20.0).reverse_clearance()
    assert clearance is None or clearance > 20.0, (clearance, source)


def test_an_unusable_drop_block_is_never_clear():
    world, robot = _robot()
    real = robot.get_depth_grid

    def blind():
        grid = real()
        grid["drop"]["zones"] = [{"status": ZONE_UNUSABLE, "distance_cm": None}] * len(grid["drop"]["zones"])
        return grid
    robot.get_depth_grid = blind
    clearance, source = SafetyController(robot, 20.0).forward_clearance()
    assert source.startswith("drop_unseen") and clearance <= 20.0, (clearance, source)


def test_a_turn_toward_an_edge_beside_a_corner_is_refused_and_away_is_not():
    """Edge cells just ahead of the front-right corner: a left (CCW) pivot
    carries that corner forward onto them; a right pivot carries it back."""
    world, robot = _robot(layout=["#" * 12] + ["#" + "." * 10 + "#"] * 10 + ["#" * 12])
    safety = SafetyController(robot, 20.0)
    safety.forward_clearance()                       # the memory starts: the seed disc and a frame
    guard = drop_guard.guard_for(robot, 0, 0, 0)
    for along in (13.5, 15.0, 17.5):
        for lat in (-14.0, -11.5, -9.0):
            guard._mark(along, lat, drop_guard.EDGE, 0.0, force=True)
    w = 1.0 * 0.172 / 2 / WHEEL_RADIUS_M             # a 1 rad/s pivot's wheel speed
    left, right, reason = safety.vet_wheel_velocity(-w, w)         # CCW, a left turn
    assert left == right == 0.0 and "drop_pivot_edge" in reason, (left, right, reason)
    left, right, reason = safety.vet_wheel_velocity(w, -w)         # CW, a right turn
    assert reason is None and left > 0, (left, right, reason)


def test_a_grid_with_no_drop_block_answers_exactly_as_before():
    world = build_world("scaled_house")
    robot = MockRobot(world, render=False)
    assert "drop" not in robot.get_depth_grid()
    on = SafetyController(robot, 20.0)
    off = SafetyController(robot, 20.0)
    off.drop_clearance = lambda *a, **k: (None, "no_drop_sensor")
    assert on.forward_clearance() == off.forward_clearance()
    assert on.reverse_clearance() == off.reverse_clearance()


def test_the_drop_block_is_the_floor_seen_and_what_ended_it():
    world, robot = _robot(y=6.0)
    block = robot.get_depth_grid()["drop"]
    mid = block["zones"][len(block["zones"]) // 2]
    # The lens is 6.5 cm ahead of the centre at y=6.0 (180 cm); the edge at
    # y=8 (240 cm): 53.5 cm of floor.
    assert mid["kind"] == "edge" and mid["distance_cm"] == pytest.approx(53.5, abs=0.2)
    assert drop_truth(world)[2] == pytest.approx(60 - 12.65, abs=0.1)


def test_the_sweeps_numbers_hold_on_a_pinned_sample():
    """D1-D4's numbers pinned (`evaluations/drop-3159/`): the approach
    sweep's first placements, exact, at the slowest and fastest speed,
    and the reverse toward floor never seen."""
    from tests import drop_sweep as ds
    places = ds.approach_placements(8)
    rows = [ds.approach_run(p + (s, False, True)) for p in places for s in (0.1, 0.6)]
    assert not any(r["fell"] for r in rows)
    assert not any(r["overhang"] > 0 for r in rows)
    assert min(r["gap"] for r in rows) >= 10.0
    back = [ds.astern_run(p + (False,)) for p in ds.approach_placements(4, seed=2)]
    assert all(r["back_cm"] == 0.0 and r["reasons"] == ["drop_unseen_astern"] for r in back)
