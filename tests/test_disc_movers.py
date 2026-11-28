# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""PLAN-ros-alignment.md 3.234.A: disc movers that approach (A1), the
travel-to-contact instrument (A2), and A3's sweep numbers pinned."""

import json
import math
import os

import pytest

from sim import renderer
from sim.grid_world import FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH
from sim.maps import build_movers, build_world
from sim.mock_robot import MockRobot
from sim.movers import CONTACT_M, DiscMover, disc_rect_gap_cells
from tests.approach_sweep import travel_to_contact_m
from tests.footprint_sweep import HALF_LENGTH, HALF_WIDTH

RECORD = os.path.join(os.path.dirname(__file__), "..", "evaluations",
                      "mover-prediction-3234", "a3-sweep.json")


def _world(house="scaled_house", speed=1.2, seed=0):
    world = build_world(house)
    for m in build_movers(house, f"approach:{speed}:{seed}"):
        world.add_mover(m)
    return world


def test_without_a_disc_mover_solid_is_the_plain_frozenset():
    """Off means off: every caller gets exactly the type it always got."""
    world = build_world("scaled_house")
    MockRobot(world, render=False).pass_time(5.0)
    assert type(world.solid_cells) is frozenset
    assert type(world.lidar_solid_cells) is frozenset
    hazards = build_world("complex_house_hazards")
    assert type(hazards.lidar_solid_cells) is frozenset


def test_scenario_names():
    assert isinstance(build_movers("starter_house", "approach:2.5")[0], DiscMover)
    assert build_movers("procthor:val:0", "approach:0.5:3")[0].speed_m_s == 0.5
    for bad in ("approach", "approach:x", "approach:0", "approach:-1", "approach:1:2:3"):
        with pytest.raises(ValueError):
            build_movers("scaled_house", bad)


def test_deterministic_for_a_seed():
    a, b = _world(seed=4), _world(seed=4)
    MockRobot(a, render=False).pass_time(40.0)
    MockRobot(b, render=False).pass_time(40.0)
    assert a._discs == b._discs and a.disc_movers[0].legs == b.disc_movers[0].legs
    c = _world(seed=5)
    MockRobot(c, render=False).pass_time(40.0)
    assert c._discs != a._discs


@pytest.mark.parametrize("house", ["scaled_house", "home_first_floor", "complex_house"])
def test_walks_both_kinds_of_leg_and_never_into_the_chassis_or_a_wall(house):
    world = _world(house, speed=2.5)
    robot = MockRobot(world, render=False)
    m = world.disc_movers[0]
    for _ in range(1200):                         # 60 s, the robot standing
        robot.pass_time(0.05)
        if m.placed:
            gap = disc_rect_gap_cells(m.x, m.y, world.x, world.y, world.theta,
                                      FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH) - m.radius_cells
            assert gap * 0.30 >= CONTACT_M - 1e-9
            assert m._clear_at(world, m.x, m.y)
    assert m.legs["approach"] > 0 and m.legs["cross"] > 0
    # It came to the robot: an approach leg ends at the contact rule.
    assert m.stops > 0


def _place(world, x, y):
    """A disc stood by hand at (x, y), cells."""
    m = DiscMover("person", 1.0)
    m.x, m.y = x, y
    world.disc_movers.append(m)
    world._discs = (m.disc(),)
    return m


def test_the_scan_depth_and_collision_see_a_disc():
    world = build_world("scaled_house")             # (6.5, 4.5) facing east
    robot = MockRobot(world, render=False)
    before = robot.get_scan()["ranges_m"][180]
    dist_before = robot.get_distance()
    _place(world, world.x + 2.0, world.y)            # 0.6 m ahead, radius 0.15
    after = robot.get_scan()["ranges_m"][180]
    lidar_x = 0.04                                    # LIDAR_X_M
    assert after == pytest.approx(0.6 - 0.15 - lidar_x, abs=0.002)
    assert before is None or before > after
    assert robot.get_distance() < dist_before
    # Collision: the chassis stops short of the disc (the centre ray, half a
    # cell, as at a wall), never inside it.
    got = world.translate(3.0)
    assert got < 2.0 - 0.5 - FOOTPRINT_HALF_LENGTH + 1e-6
    assert not world.footprint_overlaps(world.x, world.y)
    assert world.footprint_overlaps(world.x + 0.1, world.y)   # the disc, 2.4 cm on


def test_ray_disc_hit():
    assert renderer.ray_disc_hit(0, 0, 0, [(3, 0, 1)]) == pytest.approx(2.0)
    assert renderer.ray_disc_hit(0, 0, math.pi, [(3, 0, 1)]) == math.inf
    assert renderer.ray_disc_hit(0, 0, 0, [(0.2, 0, 1)]) == math.inf      # starts inside
    assert renderer.ray_disc_hit(0, 0, 0, [(3, 1.01, 1)]) == math.inf     # passes beside


def test_travel_to_contact_on_hand_built_cases():
    hl, hw, r = HALF_LENGTH, HALF_WIDTH, 0.5
    assert travel_to_contact_m(0, 0, 0, 1, (hl + r + 0.5, 0, r)) == pytest.approx(0.15)
    assert travel_to_contact_m(0, 0, 0, -1, (hl + 1, 0, r)) == math.inf     # ahead of a reverse
    assert travel_to_contact_m(0, 0, 0, -1, (-hl - 1, 0, r)) == pytest.approx(0.15)
    assert travel_to_contact_m(0, 0, 0, 1, (2, hw + 0.6, r)) == math.inf    # beside the route
    assert travel_to_contact_m(0, 0, math.pi / 2, 1, (0, hl + 1, r)) == pytest.approx(0.15)
    # A corner: the disc's centre 0.3 cells outside the side.
    t = 2 - hl - math.sqrt(r * r - 0.3 * 0.3)
    assert travel_to_contact_m(0, 0, 0, 1, (2, hw + 0.3, r)) == pytest.approx(t * 0.30)
    assert travel_to_contact_m(0, 0, 0, 1, (hl + 0.2, 0, r)) == 0.0         # touching


# ---------------- A3, pinned ----------------

# The sweep's counts (3.234's Status, 2026-11-28): the gate is open.
A3_APPROACHES_20 = 19
A3_VERBS_20, A3_ROS_20 = 0, 19
A3_APPROACHES_30 = 309


def _record():
    with open(RECORD) as f:
        return json.load(f)


def test_a3_record_verdict_is_what_the_section_file_states():
    from tests.approach_sweep import verdict
    rec = _record()
    assert verdict(rec["rows"]) == rec["verdict"]
    assert len(rec["rows"]) == 72
    v = rec["verdict"]
    assert v["all"]["approaches_20"] == A3_APPROACHES_20
    assert (v["verbs"]["approaches_20"], v["ros"]["approaches_20"]) == (A3_VERBS_20, A3_ROS_20)
    assert v["all"]["approaches_30"] == A3_APPROACHES_30
    assert v["all"]["contacts"] == 0
    assert rec["verdict"]["gate_open"] is (A3_APPROACHES_20 > 0)


def test_a3_one_mission_reruns_to_the_bit():
    """A verbs mission from the record, run again: the same row."""
    from tests.approach_sweep import mission
    rec = _record()
    row = next(r for r in rec["rows"]
               if r["drive"] == "verbs" and r["house"] == "scaled_house" and r["speed"] == 2.5)
    args = (row["house"], row["x"], row["y"], row["heading"], row["target"], row["speed"],
            row["drive"], [r for r in rec["rows"] if r["house"] == row["house"]
                           and r["drive"] == "verbs" and r["speed"] == 2.5].index(row))
    assert json.loads(json.dumps(mission(args))) == row


def test_speed_is_bounded():
    for bad in ("approach:inf", "approach:nan", "approach:1e9", "approach:5.1"):
        with pytest.raises(ValueError):
            build_movers("scaled_house", bad)
    with pytest.raises(ValueError):
        DiscMover("person", float("inf"))
    assert build_movers("scaled_house", "approach:5")[0].speed_m_s == 5.0


@pytest.mark.parametrize("var", ["SIM_MOTOR_BOARD", "SIM_LIDAR"])
def test_the_split_body_refuses_a_disc_mover(monkeypatch, var):
    """Its sensor programs read a replica without discs: refused, not half-seen."""
    from robot.factory import _sim_world
    monkeypatch.setenv("SIM_MOVERS", "approach:1.2")
    monkeypatch.setenv("SIM_MAP", "scaled_house")
    monkeypatch.setenv(var, "fake")
    with pytest.raises(ValueError, match="one process"):
        _sim_world({})
    monkeypatch.delenv(var)
    assert _sim_world({}).disc_movers
