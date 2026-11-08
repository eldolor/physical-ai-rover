# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.156.D: recharge and resume (`docs/plans/ros-alignment/
3.156-return-to-dock.md`, D7a-h and E3, E4, E6-E8), in process -- `MissionRunner`
-> the explore policy -> `robot/safety.py` -> `MockRobot` on a `SimBattery` with
the charge stand-in, nav2 stood in for by `tests/test_dock_return.py`'s RecNav."""

import pytest

from brain.dock import Charge, HomeBudget
from control.mission_runner import OUT_OF_BUDGET, MissionRunner
from robot.interface import Preempted, StoppedByPerson
from sim.battery import SimBattery
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_dock_return import RecNav


def mission(v0=10.9, drain=0.004, charge=0.05, house="home_first_floor", target="unicorn",
            job=None, recharge=True, **kw):
    grid = build_world(house)
    bat = SimBattery(v0=v0, drain_v_per_s=drain, charge_v_per_s=charge,
                     clock=lambda: grid.sim_time)
    robot = MockRobot(grid, render=False, battery=bat)
    nav = RecNav(robot)
    args = {"job": "survey"} if job == "survey" else {"target_object": target}
    kw.setdefault("battery_floor_v", 10.5)
    kw.setdefault("max_steps", 2000)
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=MockWorld(grid),
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           return_to_dock=True, recharge_resume=recharge, **args, **kw)
    return runner, nav, grid, robot, bat


def state(runner):
    return (runner.status().get("charge") or {}).get("state")


def to_charging(runner, cap=9000):
    runner.start()
    n = 0
    while runner.tick() and n < cap:
        n += 1
        if state(runner) == "charging":
            return
    raise AssertionError(f"never charged: {runner.status()['log_tail']}")


def finish(runner, cap=20000):
    n = 0
    while runner.tick() and n < cap:
        n += 1
    return runner.status()


def run(runner, cap=20000):
    runner.start()
    return finish(runner, cap)


# -- D7a, D7b: who charges ----------------------------------------------------

def test_off_is_bs_end():
    runner, *_ = mission(recharge=False)
    s = run(runner)
    assert s["outcome"] == OUT_OF_BUDGET and s["return"]["state"] == "docked"
    assert s["charge"] is None


def test_a_battery_end_charges_and_resumes():
    runner, nav, grid, robot, bat = mission(v0=11.0)
    to_charging(runner)
    v_at_dock = bat.true_v()
    assert runner.is_running()
    n = 0
    while state(runner) == "charging" and runner.tick() and n < 5000:
        n += 1
    s = runner.status()
    assert s["charge"]["state"] == "resumed", s["log_tail"]
    assert s["charge"]["history"][0]["from_v"] <= v_at_dock + 0.05
    assert s["charge"]["history"][0]["to_v"] >= 11.8
    assert s["return"] is None and runner.is_running()
    assert s["outcome"] == "running"


def test_a_time_end_never_charges():
    runner, *_ = mission(v0=12.0, drain=0.0, mission_time_limit_s=60.0)
    s = run(runner)
    assert s["outcome"] == OUT_OF_BUDGET and "time" in s["end_reason"]
    assert s["charge"] is None


def test_the_floor_itself_charges():
    runner, *_ = mission(v0=10.45, drain=0.0)
    to_charging(runner)


def test_reserve_never_charges():
    runner, nav, grid, robot, bat = mission(v0=10.45, drain=0.0)
    robot.get_battery = lambda: {"usable": True, "v": 10.1, "level": "reserve"}
    s = run(runner)
    assert s["outcome"] == OUT_OF_BUDGET and s["charge"] is None


# -- D7d, E4: the clock ---------------------------------------------------------

def test_charging_pauses_the_limit_and_only_charging():
    runner, nav, grid, robot, bat = mission(v0=11.0, charge=0.02, mission_time_limit_s=800.0)
    to_charging(runner)
    t0, paused0 = grid.sim_time, runner.status()["budget"]["paused_s"]
    n = 0
    while state(runner) == "charging" and runner.tick() and n < 5000:
        n += 1
    took = grid.sim_time - t0
    assert took > 30.0
    assert runner.status()["budget"]["paused_s"] - paused0 == pytest.approx(took, abs=3.0)


def test_no_time_left_is_no_charge():
    runner, nav, grid, robot, bat = mission(v0=10.9, mission_time_limit_s=100.0)
    s = run(runner)
    assert s["outcome"] == OUT_OF_BUDGET
    assert s["charge"] is None or s["charge"]["state"] in ("no_time", "not_docked"), s["charge"]
    assert not (s["charge"] or {}).get("history")


def test_the_wall_limit_ends_a_charge_with_the_decided_end():
    runner, nav, grid, robot, bat = mission(v0=11.0, charge=0.0001, mission_wall_limit_s=200.0,
                                            charge_stall_s=5000.0)
    to_charging(runner)
    s = finish(runner)
    assert s["outcome"] == OUT_OF_BUDGET and s["charge"]["state"] == "wall_limit"
    assert s["return"]["state"] == "docked"


# -- D7d, E6: the bounds --------------------------------------------------------

def test_a_charger_that_gives_nothing_is_no_charge():
    runner, nav, grid, robot, bat = mission(v0=11.0, charge=0.0, charge_stall_s=120.0)
    to_charging(runner)
    t0 = grid.sim_time
    s = finish(runner)
    assert s["outcome"] == OUT_OF_BUDGET and s["charge"]["state"] == "no_charge"
    assert grid.sim_time - t0 == pytest.approx(120.0, abs=5.0)


def test_carried_off_the_dock_ends_it():
    runner, nav, grid, robot, bat = mission(v0=11.0, charge=0.0001, charge_stall_s=5000.0)
    to_charging(runner)
    grid.x += 2.0
    s = finish(runner)
    assert s["outcome"] == OUT_OF_BUDGET and s["charge"]["state"] == "left_dock"


def test_a_return_that_fails_ends_with_the_decided_end():
    runner, nav, grid, robot, bat = mission(v0=11.0)
    runner.start()
    n = 0
    while runner._pending_end is None and runner.tick() and n < 9000:
        n += 1
    assert runner._pending_end is not None
    nav.never_ready = True
    runner.return_wait_s = 2.0
    runner._return_trip.wait_s = 2.0
    runner._return_trip.state = "goal"
    s = finish(runner)
    assert s["outcome"] == OUT_OF_BUDGET and s["return"]["state"] != "docked"
    assert s["charge"]["state"] == "not_docked" and not s["charge"]["history"]


def test_at_most_three_charges():
    runner, nav, grid, robot, bat = mission(v0=11.9, drain=0.05, charge=0.5)
    s = run(runner)
    assert 1 <= len(s["charge"]["history"]) <= 3, s["charge"]


def test_the_fourth_battery_end_does_not_charge():
    runner, nav, grid, robot, bat = mission(v0=11.0)
    runner._charges = 3
    s = run(runner)
    assert s["outcome"] == OUT_OF_BUDGET and s["return"]["state"] == "docked"
    assert s["charge"] is None


# -- D7e, E3: what it kept ------------------------------------------------------

def test_what_it_kept_is_unchanged_across_a_charge():
    runner, nav, grid, robot, bat = mission(v0=11.0)
    to_charging(runner)
    a = runner.agent
    before = (list(a._reached), list(a.goals_reached), dict(a.retry.__dict__.get("_places", {})),
              set(a.rooms_entered), set(runner.memory.visited_rooms))
    agent, memory = runner.agent, runner.memory
    n = 0
    while state(runner) == "charging" and runner.tick() and n < 5000:
        n += 1
    assert state(runner) == "resumed"
    assert runner.agent is agent and runner.memory is memory
    after = (list(a._reached), list(a.goals_reached), dict(a.retry.__dict__.get("_places", {})),
             set(a.rooms_entered), set(runner.memory.visited_rooms))
    assert after == before


def test_the_goal_in_flight_costs_its_place_nothing():
    runner, nav, grid, robot, bat = mission(v0=11.0)
    to_charging(runner)
    a = runner.agent
    a._goal = {"x_m": 1.0, "y_m": 1.0, "kind": "frontier", "sent_at": 0.0, "from": (0.0, 0.0)}
    a._pending = ["REVERSE"]
    a._approach = (1.0, 1.0)
    fails = []
    a.retry.fail = lambda *x, **k: fails.append(x)
    n = 0
    while state(runner) == "charging" and runner.tick() and n < 5000:
        n += 1
    assert a._goal is None and a._pending == [] and a._approach is None and fails == []


def test_a_resumed_mission_goes_on_and_ends_home():
    runner, nav, grid, robot, bat = mission(v0=11.0)
    s = run(runner)
    assert s["charge"]["history"], s["log_tail"]
    assert s["outcome"] != "running" and s["return"]["state"] == "docked"


# -- D7f, E7: a person still wins ----------------------------------------------

@pytest.mark.parametrize("exc,why", [(StoppedByPerson("person_stop: a person pressed Stop"), "stop"),
                                     (Preempted("twin-dpad is driving -- brain is lower"), "preempted")])
def test_a_person_while_charging(exc, why):
    runner, nav, grid, robot, bat = mission(v0=11.0, charge=0.001, charge_stall_s=5000.0)
    to_charging(runner)
    x, y = grid.x, grid.y

    def refused():
        raise exc
    robot.look_center = refused
    s = finish(runner, 50)
    assert not runner.is_running()
    assert s["outcome"] == OUT_OF_BUDGET and s["charge"]["state"] == "cancelled"
    assert why in s["charge"]["reason"] and (grid.x, grid.y) == (x, y)


def test_the_brains_stop_while_charging():
    runner, nav, grid, robot, bat = mission(v0=11.0, charge=0.001, charge_stall_s=5000.0)
    to_charging(runner)
    runner.stop("a person")
    s = runner.status()
    assert not runner.is_running()
    assert s["outcome"] == OUT_OF_BUDGET and s["charge"]["state"] == "cancelled"


def test_reserve_on_the_charger_does_not_end_the_charge():
    from robot.interface import BatteryReserve
    runner, nav, grid, robot, bat = mission(v0=11.0)
    to_charging(runner)

    def refused():
        raise BatteryReserve("battery_reserve: 10.15 V")
    robot.look_center = refused
    n = 0
    while state(runner) == "charging" and runner.tick() and n < 5000:
        n += 1
    assert state(runner) == "resumed" and runner.is_running()


def test_an_autonomous_takeover_while_charging_is_a_fault():
    runner, nav, grid, robot, bat = mission(v0=11.0, charge=0.001, charge_stall_s=5000.0)
    to_charging(runner)

    def refused():
        raise Preempted("ros is driving")
    robot.look_center = refused
    s = finish(runner, 50)
    assert s["charge"]["state"] == "failed" and "autonomous" in s["error"]


# -- the pieces -----------------------------------------------------------------

def test_a_never_charged_pack_is_exact():
    clock = [0.0]
    a = SimBattery(v0=12.0, drain_v_per_s=0.003, clock=lambda: clock[0])
    for t in (0.0, 1.5, 77.7, 4000.0):
        clock[0] = t
        assert a.true_v() == max(0.0, 12.0 - 0.003 * t)


def test_a_pack_charges_only_on_the_charger_and_stops_at_full():
    clock = [0.0]
    a = SimBattery(v0=11.0, drain_v_per_s=0.01, charge_v_per_s=0.1, clock=lambda: clock[0])
    clock[0] = 10.0
    assert a.true_v() == pytest.approx(10.9)
    a.charging(True)
    clock[0] = 20.0
    assert a.true_v() == pytest.approx(11.9)
    clock[0] = 100.0
    assert a.true_v() == pytest.approx(a.FULL_V)
    a.charging(False)
    clock[0] = 110.0
    assert a.true_v() == pytest.approx(a.FULL_V - 0.1)


def test_the_pad_is_the_start_pose():
    grid = build_world("scaled_house")
    bat = SimBattery(v0=11.0, charge_v_per_s=0.1, clock=lambda: grid.sim_time)
    robot = MockRobot(grid, render=False, battery=bat)
    robot.get_battery()
    assert bat.is_charging
    grid.x += 2.0
    robot.get_battery()
    assert not bat.is_charging


def test_driving_off_the_pad_stops_the_charge_at_once():
    grid = build_world("scaled_house")
    bat = SimBattery(v0=11.0, charge_v_per_s=0.1, clock=lambda: grid.sim_time)
    robot = MockRobot(grid, render=False, battery=bat)
    robot.get_battery()
    assert bat.is_charging
    robot.set_wheel_velocity(1.0, 1.0)
    assert not bat.is_charging


def test_no_charge_rate_no_pad():
    grid = build_world("scaled_house")
    bat = SimBattery(v0=11.0, clock=lambda: grid.sim_time)
    robot = MockRobot(grid, render=False, battery=bat)
    robot.get_battery()
    assert not bat.is_charging


def test_the_drain_known_before_a_charge_stands_after_it():
    b = HomeBudget({"x_m": 0.0, "y_m": 0.0, "theta": 0.0, "source": "start"})
    b.observe_battery(12.0, 0.0)
    b.observe_battery(11.4, 60.0)
    assert b.drain_v_s() == pytest.approx(0.01)
    b.recharged()
    assert b.drain_v_s() == pytest.approx(0.01)
    b.observe_battery(11.8, 100.0)
    b.observe_battery(11.5, 130.0)                  # under the span: still the old rate
    assert b.drain_v_s() == pytest.approx(0.01)
    b.observe_battery(11.2, 160.0)                  # a new span
    assert b.drain_v_s() == pytest.approx(0.6 / 60.0)


def test_charge_reads_two_samples_over_resume_v():
    c = Charge({"x_m": 0.0, "y_m": 0.0, "theta": 0.0}, resume_v=11.8, t0=0.0, stall_s=300.0)
    pose = {"usable": True, "x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0}
    assert c.observe(11.0, pose, 1.0) is None
    assert c.observe(11.8, pose, 2.0) is None
    assert c.observe(11.8, pose, 3.0) is None       # the board's last reading again
    assert c.observe(11.81, pose, 4.0)[0] == "resumed"


