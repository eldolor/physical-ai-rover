# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_reverse_retrace.py

PLAN-ros-alignment.md 3.224: with the lidar's rear hidden by the mast
(3.218), a reverse is allowed only back over ground the body itself just
drove across (`robot/retrace.py`). Criteria 1-5 of
`docs/plans/ros-alignment/3.224-reverse-retrace.md`, judged on the sim
body's TRUE pose, never on the track being tested (CLAUDE.md section 6).
"""
import math
import random

import pytest

from robot.retrace import (RETRACE_HEADING_DEG, RETRACE_LATERAL_CM, RETRACE_MAX_CM,
                           RETRACE_V_M_S, RETRACE_WINDOW_S, JUMP_M, RetraceTrack)
from robot.safety import SafetyController, SafetyViolation
from sim.maps import build_world
from sim.mock_robot import MockRobot, WHEEL_RADIUS_M
from tests import footprint_sweep as fs

ROVER_BOX = (-7.1, 3.2, -10.7, 8.5)     # 3.218's measured box
BAR_CM = 1.0                            # criterion 1: never behind its track by more
MAX_BACK_CM = RETRACE_MAX_CM - 20.0     # criterion 1: the 20 cm bar leaves this much
PROGRESS_FORWARD_CM = 40.0              # criterion 2
PROGRESS_BACK_CM = 15.0


# ---- criterion 3: the track ends where it should ---------------------------

def _straight(track, cm, t0=0.0, heading=0.0, step_cm=1.0):
    s = 0.0
    t = t0
    while s < cm / 100.0:
        s += step_cm / 100.0
        t += 0.05
        track.update(t, s, heading)
    return t, s


def test_a_straight_forward_gives_its_length_back():
    track = RetraceTrack()
    t, _ = _straight(track, 30)
    assert track.room_cm(t) == pytest.approx(30.0, abs=1.0)


def test_the_room_is_capped():
    track = RetraceTrack()
    t, _ = _straight(track, 120)
    assert track.room_cm(t) == RETRACE_MAX_CM


def test_no_track_no_room():
    assert RetraceTrack().room_cm(0.0) == 0.0


def test_a_pivot_ends_the_track():
    track = RetraceTrack()
    t, s = _straight(track, 40)
    track.update(t + 0.5, s, RETRACE_HEADING_DEG + 5.0)
    assert track.room_cm(t + 0.5) == 0.0


def test_a_small_heading_wobble_keeps_it():
    track = RetraceTrack()
    t, s = _straight(track, 40)
    track.update(t + 0.1, s, 3.0)
    assert track.room_cm(t + 0.1) > 30.0


def test_lateral_drift_ends_the_track():
    # Drive along heading 0, then 2 cm sideways is impossible for a skid
    # steer; simulate an arc: a heading that swung 9 deg and back, which
    # leaves the old line more than 3 cm to the side.
    track = RetraceTrack()
    t, s = _straight(track, 40, heading=0.0)
    for _ in range(40):                       # 40 cm at 9 deg: ~6 cm across
        s += 0.01
        t += 0.05
        track.update(t, s, 9.0)
    for _ in range(5):
        s += 0.01
        t += 0.05
        track.update(t, s, 0.0)
    room = track.room_cm(t)
    # Back along the arc only while it stays within 3 cm of the reverse
    # line (~5 cm straight + ~(3 / sin 9 deg) = ~24 cm), never on to the
    # first straight, 6 cm to the side and 45 cm back.
    assert 5.0 < room <= 5.0 + RETRACE_LATERAL_CM / math.sin(math.radians(9.0)) + 1.0


def test_old_track_is_forgotten():
    track = RetraceTrack()
    t, _ = _straight(track, 40)
    assert track.room_cm(t + RETRACE_WINDOW_S + 1.0) == 0.0


def test_a_wheel_travel_jump_drops_the_track():
    track = RetraceTrack()
    t, s = _straight(track, 40)
    track.update(t + 0.05, s - JUMP_M - 0.1, 0.0)    # a board reboot
    assert track.room_cm(t + 0.05) == 0.0


@pytest.mark.parametrize("travel,heading", [(None, 0.0), (0.4, None), (float("nan"), 0.0)])
def test_unusable_wheels_or_odometry_drop_the_track(travel, heading):
    track = RetraceTrack()
    t, _ = _straight(track, 40)
    track.update(t + 0.05, travel, heading)
    assert track.room_cm(t + 0.05) == 0.0


def test_backing_up_uses_the_room_up():
    track = RetraceTrack()
    t, s = _straight(track, 40)
    for _ in range(25):
        s -= 0.01
        t += 0.05
        track.update(t, s, 0.0)
    assert track.room_cm(t) == pytest.approx(15.0, abs=1.0)


# ---- the sim, judged on truth (criteria 1, 2 and 5) -------------------------

def _rear_bumper(world):
    """The rear bumper's centre in cm, world frame (cells are fs.CELL_CM)."""
    return ((world.x - fs.HALF_LENGTH * math.cos(world.theta)) * fs.CELL_CM,
            (world.y - fs.HALF_LENGTH * math.sin(world.theta)) * fs.CELL_CM)


def _drive(robot, safety, world, left, right, seconds, track_points=None):
    for _ in range(int(seconds / fs.PERIOD_S)):
        l, r, _why = safety.vet_wheel_velocity(left, right)
        robot.set_wheel_velocity(l, r)
        robot.advance(fs.PERIOD_S)
        if track_points is not None:
            track_points.append(_rear_bumper(world))
    robot.set_wheel_velocity(0.0, 0.0)


def _run(house, x, y, heading_deg, forward_cm, variant):
    world = build_world(house)
    world.x, world.y, world.theta = x, y, math.radians(heading_deg)
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0, self_box_cm=ROVER_BOX)
    w = fs.SPEED_M_S / WHEEL_RADIUS_M
    track = [_rear_bumper(world)]
    x0, y0 = world.x, world.y
    left = w * (1.03 if variant == "drift" else 1.0)
    _drive(robot, safety, world, left, w, forward_cm / 100.0 / fs.SPEED_M_S, track)
    forward = math.hypot(world.x - x0, world.y - y0) * fs.CELL_CM
    if variant == "pivot":
        wheels = robot.get_wheel_state()
        wp = 0.6 * wheels["track_width_m"] / 2 / WHEEL_RADIUS_M
        start = world.theta
        for _ in range(200):
            if abs((world.theta - start + math.pi) % (2 * math.pi) - math.pi) >= math.radians(20):
                break
            l, r, _ = safety.vet_wheel_velocity(-wp, wp)
            robot.set_wheel_velocity(l, r)
            robot.advance(fs.PERIOD_S)
        robot.set_wheel_velocity(0.0, 0.0)
    bx, by = _rear_bumper(world)
    xr, yr = world.x, world.y
    _drive(robot, safety, world, -2 * w, -2 * w, 4.0)
    back = math.hypot(world.x - xr, world.y - yr) * fs.CELL_CM
    # Truth: how far the final rear bumper lies behind the rearmost point
    # the bumper occupied on the track, along the final heading.
    ux, uy = math.cos(world.theta), math.sin(world.theta)
    fx, fy = _rear_bumper(world)
    rearmost = min(px * ux + py * uy for px, py in track + [(bx, by)])
    behind = rearmost - (fx * ux + fy * uy)
    return {"forward": forward, "back": back, "behind": behind, "variant": variant}


def _sweep(n=200, seed=3224):
    rng = random.Random(seed)
    houses = ("scaled_house", "home_first_floor")
    starts = [(h, x, y) for h in houses for x, y in fs.starts(h, n // len(houses) + 5, seed)]
    out = []
    for i in range(n):
        house, x, y = starts[i % len(starts)]
        roll = rng.random()
        variant = "straight" if roll < 0.6 else ("drift" if roll < 0.8 else "pivot")
        out.append(_run(house, x, y, rng.uniform(0, 360), rng.uniform(5, 80), variant))
    return out


@pytest.fixture(scope="module")
def sweep():
    return _sweep()


def test_criterion_1_never_behind_its_own_track(sweep):
    worst = max(r["behind"] for r in sweep)
    assert worst <= BAR_CM, worst
    assert max(r["back"] for r in sweep) <= MAX_BACK_CM + BAR_CM


def test_criterion_2_a_long_forward_can_back_out(sweep):
    eligible = [r for r in sweep if r["variant"] == "straight"
                and r["forward"] >= PROGRESS_FORWARD_CM]
    assert eligible
    assert all(r["back"] >= PROGRESS_BACK_CM for r in eligible), \
        sorted(r["back"] for r in eligible)[:5]


def test_a_pivot_before_the_reverse_refuses_it(sweep):
    pivots = [r for r in sweep if r["variant"] == "pivot"]
    assert pivots and all(r["back"] <= 0.5 for r in pivots)


def test_criterion_5_the_ros_path_backs_out_slowly():
    world = build_world("scaled_house")
    x, y = fs.starts("scaled_house", 1, 7)[0]
    world.x, world.y, world.theta = x, y, 0.0
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0, self_box_cm=ROVER_BOX)
    w = fs.SPEED_M_S / WHEEL_RADIUS_M
    _drive(robot, safety, world, w, w, 0.5)             # 5 cm: track starts
    _drive(robot, safety, world, w, w, 4.0)             # +40 cm
    fast = -0.4 / WHEEL_RADIUS_M
    l, r, _why = safety.vet_wheel_velocity(fast, fast)
    v = (l + r) / 2.0 * WHEEL_RADIUS_M
    assert -RETRACE_V_M_S - 1e-9 <= v < 0.0
    assert safety.reverse_clearance()[1] == "retrace"


def test_without_a_self_box_reverse_is_unchanged():
    # Criterion 4: no box, no track kept, the old answer.
    world = build_world("scaled_house")
    x, y = fs.starts("scaled_house", 1, 7)[0]
    world.x, world.y, world.theta = x, y, 0.0
    robot = MockRobot(world, render=False)
    plain = SafetyController(robot, 20.0)
    assert plain._retrace is None
    assert plain.reverse_clearance()[1] != "retrace"


def test_the_verb_path_backs_out_on_its_track_only():
    # Amendment 1: one seeded run of the verb path, held to criterion 1's
    # truth bar (verbs run on the wall clock, so one run, not 200).
    world = build_world("scaled_house")
    x, y = fs.starts("scaled_house", 3, 11)[0]
    world.x, world.y = x, y
    robot = MockRobot(world, render=False)
    safety = SafetyController(robot, 20.0, self_box_cm=ROVER_BOX)
    for heading in range(0, 360, 15):           # the first with room ahead (seeded)
        world.theta = math.radians(heading)
        if (safety.forward_clearance()[0] or 0.0) >= 80.0:
            break
    else:
        pytest.fail("no heading with 80 cm ahead")
    safety = SafetyController(robot, 20.0, self_box_cm=ROVER_BOX)   # a fresh track
    track = [_rear_bumper(world)]
    x0, y0 = world.x, world.y
    for _ in range(4):                          # the sim's verb is a ~10 cm step
        safety.check_and_execute("FORWARD", speed=30, duration=1.5)
        track.append(_rear_bumper(world))
    forward = math.hypot(world.x - x0, world.y - y0) * fs.CELL_CM
    assert forward > 25.0, forward
    xr, yr = world.x, world.y
    refused = None
    for _ in range(10):
        try:
            safety.check_and_execute("REVERSE", speed=30, duration=1.5)
        except SafetyViolation as e:            # refused at the end of the room
            refused = str(e)
            break
    assert refused and "(retrace)" in refused, refused
    back = math.hypot(world.x - xr, world.y - yr) * fs.CELL_CM
    ux, uy = math.cos(world.theta), math.sin(world.theta)
    fx, fy = _rear_bumper(world)
    behind = min(px * ux + py * uy for px, py in track) - (fx * ux + fy * uy)
    assert behind <= BAR_CM, behind
    assert 0.0 < back <= MAX_BACK_CM + BAR_CM, back
