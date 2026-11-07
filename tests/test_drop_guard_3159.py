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
            guard._mark(along, lat, drop_guard.EDGE, 0.0)
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


# ---- review fixes (red first) ----

def test_a_guard_switched_on_with_no_pose_is_never_clear():
    """Review: a block seen while the wheels are unusable switched the
    guard on with no pose, and every question then read clear."""
    world, robot = _robot()
    robot.get_wheel_state = lambda: {"usable": False}
    guard = drop_guard.DropGuard(12.65, 11.55, 3.0)
    guard.update(robot, robot.get_depth_grid())
    assert guard.active
    assert guard.clearance(+1) == (0.0, "drop_unusable")
    assert guard.pivot_blocked(1.0) == "drop_unusable"


def test_a_wall_end_never_paints_over_a_remembered_edge():
    """Review: a thing's end or a wall's join rewrote an edge cell, and the
    corridor then stopped looking at that line."""
    world, robot = _robot()
    guard = drop_guard.DropGuard(12.65, 11.55, 3.0)
    guard.update(robot, robot.get_depth_grid())
    guard._mark(40.0, 0.0, drop_guard.EDGE, 0.0)
    guard._mark(40.0, 0.0, drop_guard.THING, 0.0)
    assert guard._state(40.0, 0.0) == drop_guard.EDGE


def test_each_controller_keeps_its_own_margin():
    """Review: the shared memory kept the first controller's margin."""
    world, robot = _robot(y=6.0)
    SafetyController(robot, 10.0).forward_clearance()            # takes the guard first
    near, _ = SafetyController(robot, 10.0).drop_clearance(+1)
    far, source = SafetyController(robot, 30.0).drop_clearance(+1)
    assert source == "drop_edge" and far == pytest.approx(near - 20.0, abs=0.01), (near, far)


def test_a_house_with_no_drop_reads_one_grid_per_reverse_vet_and_no_more():
    """Review: reverse asked for a depth grid on every vet in every house,
    which moved the noise streams of houses without a drop."""
    world = build_world("scaled_house")
    robot = MockRobot(world, render=False)
    calls = {"n": 0}
    real = robot.get_depth_grid

    def counted(*a, **k):
        calls["n"] += 1
        return real(*a, **k)
    robot.get_depth_grid = counted
    safety = SafetyController(robot, 20.0)
    safety.reverse_clearance()                        # the one probe
    calls["n"] = 0
    for _ in range(5):
        safety.drop_clearance(-1)
    assert calls["n"] == 0


def test_a_fast_turn_is_looked_at_as_far_as_it_turns_in_two_periods():
    """Review: a 5-degree look ahead at any rate let a 2 rad/s turn carry a
    corner past an edge between two looks."""
    world, robot = _robot()
    guard = drop_guard.DropGuard(12.65, 11.55, 3.0)
    guard.update(robot, robot.get_depth_grid())
    # An edge 8 degrees CCW of the front-left corner, floor everywhere else.
    a, l = 12.65, 11.55
    th = math.radians(8.0)
    guard._mark(a * math.cos(th) - l * math.sin(th), a * math.sin(th) + l * math.cos(th),
                drop_guard.EDGE, 0.0)
    assert guard.pivot_blocked(0.5) is None              # 2.9 degrees in 0.1 s
    assert guard.pivot_blocked(2.0) == "drop_pivot_edge"  # 11.5 degrees


def test_a_thing_on_the_centre_line_hides_no_edge_beside_it():
    """Thermos: a thing at lateral 0 ended every line of the corridor; a
    thing never hides a remembered edge."""
    world, robot = _robot()
    guard = drop_guard.DropGuard(12.65, 11.55, 3.0)
    guard.update(robot, robot.get_depth_grid())
    for d in range(0, 60, 2):
        for lat in range(-15, 16, 3):
            guard._mark(12.65 + d, float(lat), drop_guard.FLOOR, 0.0)
    guard._mark(12.65 + 10.0, 0.0, drop_guard.THING, 0.0)
    guard._mark(12.65 + 30.0, 9.0, drop_guard.EDGE, 0.0)
    assert guard.clearance(+1, 0.0)[1] == "drop_edge"


def test_a_first_move_that_is_a_turn_switches_the_guard_on():
    """Thermos: a turn before any straight vet ran with no pivot check."""
    world, robot = _robot()
    safety = SafetyController(robot, 20.0)
    safety.drop_pivot_blocked(1.0)
    assert safety._drop_guard().active


def test_an_arc_with_a_blind_lidar_still_folds_the_frame_for_its_turn():
    """Review 2 (both): an arc whose straight vet returned before the fold
    (`ahead_not_observed`) skipped the pivot's fetch, and a stale pose
    judged the turn."""
    world, robot = _robot()
    safety = SafetyController(robot, 20.0)
    safety.forward_clearance()                                   # the guard is on
    calls = {"n": 0}
    real = robot.get_depth_grid

    def counted(*a, **k):
        calls["n"] += 1
        return real(*a, **k)
    robot.get_depth_grid = counted
    safety.forward_clearance = lambda grid=None: (0.0, "ahead_not_observed")
    w = 1.0 * 0.172 / 2 / WHEEL_RADIUS_M
    safety.vet_wheel_velocity(0.5 * w, 1.5 * w)                 # forward and CCW
    assert calls["n"] == 1, "the turn was judged on the last tick's pose"


def test_a_missing_first_frame_does_not_spend_the_probe():
    """Review 2 (both): a first reverse with no frame at all switched the
    astern probe off for good."""
    world, robot = _robot(y=3.0, theta=-DOWN)
    real = robot.get_depth_grid
    robot.get_depth_grid = lambda *a, **k: None
    safety = SafetyController(robot, 20.0)
    assert safety.drop_clearance(-1)[1] == "no_drop_sensor"
    robot.get_depth_grid = real
    assert safety.drop_clearance(-1)[1] == "drop_unseen_astern"
