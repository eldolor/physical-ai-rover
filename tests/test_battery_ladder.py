"""
tests/test_battery_ladder.py

PLAN-ros-alignment.md 3.137.A: the battery ladder -- Low, Reserve, a clean
power-off -- in the robot server, and what the brain makes of it.

Three layers:

* `robot/battery_ladder.py`'s rules on planted samples: two distinct board
  samples per level, never one; a sample read with the wheels turning is read
  again at rest (a motor's sag clears, a flat pack fires); Reserve's timer.
* the robot server, in process (a `MockRobot` whose pack the test sets, on the
  simulator's clock): Reserve refuses autonomy by name and lets a person drive
  slowly; a nav2 goal ends and no new one is taken; Power-off refuses everyone
  and runs its steps in order, then the power-off command, once.
* the brain: a battery refusal ends a mission `out_of_budget`; no mission
  starts on a low battery.
"""

import time

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from robot.battery_ladder import (CHECK, CLEAR, LOW, OK, POWEROFF, RESERVE,
                                  BatteryLadder)
from robot.interface import (BATTERY_CHECK, BATTERY_POWEROFF, BATTERY_RESERVE,
                             BatteryReserve)
from sim.battery import SimBattery


# ---------- the rules, on planted samples ----------

def ladder(**kw):
    kw.setdefault("sample_s", 12.0)
    return BatteryLadder(low_v=10.5, reserve_v=10.2, poweroff_v=10.0, reserve_max_s=120.0, **kw)


def test_one_low_sample_never_fires():
    lad = ladder()
    assert lad.observe(10.10, 0.0, at_rest=True) is None
    assert lad.level == OK
    # Back over the line: the next low one is a first again.
    assert lad.observe(10.40, 1.0, at_rest=True) is None
    assert lad.observe(10.10, 2.0, at_rest=True) is None
    assert lad.level == LOW          # 10.40 and 10.10 are both under Low


def test_two_distinct_samples_at_rest_fire_reserve():
    lad = ladder()
    lad.observe(10.15, 0.0, at_rest=True)
    assert lad.observe(10.14, 0.5, at_rest=True) == RESERVE
    assert lad.level == RESERVE and lad.reserve_at == 0.5


def test_one_board_sample_read_twice_is_one_sample():
    """M2.7: the board repeats its 10 s measurement in every frame; the same
    value again inside `sample_s` is the same sample."""
    lad = ladder()
    lad.observe(10.15, 0.0, at_rest=True)
    for t in (0.5, 1.0, 5.0, 11.9):
        assert lad.observe(10.15, t, at_rest=True) is None
    assert lad.level == OK
    assert lad.observe(10.15, 12.0, at_rest=True) == RESERVE


def test_a_sag_while_driving_is_read_again_at_rest_and_clears():
    lad = ladder()
    assert lad.observe(10.10, 0.0, at_rest=False) is None
    assert lad.observe(10.05, 0.5, at_rest=False) == CHECK
    assert lad.check_level == RESERVE
    # Still turning: the check waits for a sample at rest.
    assert lad.observe(10.04, 1.0, at_rest=False) is None
    assert lad.observe(11.00, 1.5, at_rest=True) == CLEAR
    assert lad.level == OK and lad.sags == 1 and lad.check_level is None


def test_a_flat_pack_read_again_at_rest_fires():
    lad = ladder()
    lad.observe(10.10, 0.0, at_rest=False)
    assert lad.observe(10.05, 0.5, at_rest=False) == CHECK
    assert lad.observe(10.08, 1.0, at_rest=True) == RESERVE


def test_a_check_that_never_rests_fires_on_the_moving_samples():
    lad = ladder()
    lad.observe(10.10, 0.0, at_rest=False)
    assert lad.observe(10.05, 0.5, at_rest=False) == CHECK
    assert lad.tick(0.5 + lad.check_max_s - 0.1) is None
    assert lad.tick(0.5 + lad.check_max_s) == RESERVE


def test_a_recovered_reading_clears_a_check_before_the_wheels_count_as_at_rest():
    lad = ladder()
    lad.observe(10.10, 0.0, at_rest=False)
    assert lad.observe(10.05, 0.5, at_rest=False) == CHECK
    assert lad.observe(10.95, 0.6, at_rest=False) == CLEAR
    assert lad.check_level is None and lad.level == OK


def test_a_check_that_times_out_never_fires_on_a_recovered_pack():
    """The bug the sag evaluation found: a check that timed out fired the
    level it was opened for even when the newest samples were back over it."""
    lad = ladder()
    lad.observe(10.10, 0.0, at_rest=False)
    assert lad.observe(10.05, 0.5, at_rest=False) == CHECK
    # A higher sample the check never saw at rest (planted past the clear).
    lad.samples.append((10.9, False, 1.0))
    assert lad.tick(0.5 + lad.check_max_s) == CLEAR
    assert lad.level not in (RESERVE, POWEROFF)


def test_reserve_held_too_long_powers_off():
    lad = ladder()
    lad.observe(10.15, 0.0, at_rest=True)
    lad.observe(10.14, 0.5, at_rest=True)
    assert lad.status(60.5)["poweroff_in_s"] == 60.0
    assert lad.tick(120.4) is None
    assert lad.tick(120.5) == POWEROFF
    assert lad.poweroff_why.startswith("reserve held")


def test_under_power_off_straight_from_ok_powers_off():
    lad = ladder()
    lad.observe(9.95, 0.0, at_rest=True)
    assert lad.observe(9.94, 0.5, at_rest=True) == POWEROFF
    assert lad.level == POWEROFF
    # Latched: a charger's 12.6 V does not undo a power-off under way.
    assert lad.observe(12.6, 1.0, at_rest=True) is None and lad.level == POWEROFF


def test_reserve_is_latched_and_low_is_not():
    lad = ladder()
    lad.observe(10.40, 0.0, at_rest=True)
    lad.observe(10.39, 0.5, at_rest=True)
    assert lad.level == LOW
    lad.observe(12.0, 1.0, at_rest=True)
    assert lad.level == OK
    lad.observe(10.15, 1.5, at_rest=True)
    lad.observe(10.14, 2.0, at_rest=True)
    lad.observe(12.0, 2.5, at_rest=True)
    assert lad.level == RESERVE


def test_no_reading_moves_nothing():
    lad = ladder()
    for t in range(10):
        assert lad.observe(None, float(t), at_rest=True) is None
    assert lad.level == OK


def test_levels_must_fall():
    with pytest.raises(ValueError):
        BatteryLadder(low_v=10.0, reserve_v=10.2, poweroff_v=9.8)


# ---------- the simulated pack ----------

def test_the_sim_pack_sags_only_while_the_wheels_turn():
    t = [0.0]
    pack = SimBattery(v0=11.0, sag_v=1.0, sag_s=1.0, clock=lambda: t[0])
    assert pack.read() == 11.0
    pack.driving(True)
    assert pack.read() == 10.0
    t[0] = 0.5
    pack.driving(False)
    assert pack.read() == 11.0
    pack.driving(True)
    t[0] = 1.6
    assert pack.read() == 11.0          # past the sag, still driving


def test_the_sim_pack_repeats_its_reading_until_the_refresh():
    t = [0.0]
    pack = SimBattery(v0=11.0, drain_v_per_s=0.1, refresh_s=10.0, clock=lambda: t[0])
    assert pack.read() == 11.0
    t[0] = 9.9
    assert pack.read() == 11.0
    t[0] = 10.0
    assert pack.read() == pytest.approx(10.0)


# ---------- the robot server, in process ----------

class Pack:
    """A pack whose reading the test sets."""

    def __init__(self, v):
        self.v = v
        self.on = False

    def read(self):
        return self.v

    def driving(self, on):
        self.on = on


@pytest.fixture
def served(monkeypatch, tmp_path):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    log = tmp_path / "steps.log"
    monkeypatch.setenv("BATTERY_SHUTDOWN_CMDS",
                       f'[["brain", "echo brain >> {log}"], ["ros", "echo ros >> {log}"],'
                       f' ["vlm", "echo vlm >> {log}"]]')
    monkeypatch.setenv("BATTERY_POWEROFF_CMD", f"echo poweroff >> {log}")
    app = server.create_app()
    client = TestClient(app)
    bat = app.state.battery
    body = bat["robot"]
    body.battery = Pack(12.0)

    def reading(v, rest=True):
        """One new board sample, `v` volts, after enough sim time for the
        wheels to count as at rest."""
        body.battery.v = v
        body.pass_time(0.6 if rest else 0.01)
        bat["poll"](force=True)

    return client, bat, body, reading, log


def drive_to(reading, level_v):
    reading(level_v)
    reading(level_v - 0.01)


def test_reserve_refuses_the_brain_by_name(served):
    client, bat, body, reading, _ = served
    drive_to(reading, 10.15)
    assert bat["ladder"].level == RESERVE
    r = client.post("/action", json={"action": "FORWARD"}, headers={"x-driver": "brain"}).json()
    assert r["executed"] is False and r["reason"] == BATTERY_RESERVE
    r = client.post("/wheels", json={"left_rad_s": 2.0, "right_rad_s": 2.0},
                    headers={"x-driver": "brain"}).json()
    assert r["executed"] is False and r["reason"] == BATTERY_RESERVE


def test_at_reserve_a_person_drives_slowly(served):
    client, bat, body, reading, _ = served
    r = client.post("/action", json={"action": "FORWARD", "speed": 100, "duration": 0.5},
                    headers={"x-driver": "twin-dpad"}).json()
    assert r["executed"] is True and r["result"]["speed"] == 100, r
    drive_to(reading, 10.15)
    assert bat["ladder"].level == RESERVE
    r = client.post("/action", json={"action": "FORWARD", "speed": 100, "duration": 0.5},
                    headers={"x-driver": "twin-dpad"}).json()
    assert r["executed"] is True, r
    assert r["result"]["speed"] == bat["settings"]["reserve_speed"] == 30


def test_reserve_lets_go_of_the_brains_claim_and_stops(served):
    client, bat, body, reading, _ = served
    client.post("/wheels", json={"left_rad_s": 2.0, "right_rad_s": 2.0},
                headers={"x-driver": "brain"})
    assert client.get("/health").json()["authority_holder"] == "brain"
    # Read at rest only for the test's sake: the brain's standing command is
    # stopped by the check, then the pack is still low at rest.
    body.battery.v = 10.15
    body.pass_time(0.6)
    bat["poll"](force=True)
    body.battery.v = 10.14
    body.pass_time(0.6)
    bat["poll"](force=True)
    if bat["ladder"].check_level:
        reading(10.13)
    assert bat["ladder"].level == RESERVE
    w = client.get("/wheels").json()
    assert (w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]) == (0.0, 0.0)
    assert client.get("/health").json()["authority_holder"] is None


def test_battery_route_names_the_level(served):
    client, bat, body, reading, _ = served
    b = client.get("/battery").json()
    assert b["usable"] is True and b["level"] == OK
    drive_to(reading, 10.15)
    b = client.get("/battery").json()
    assert b["level"] == RESERVE and b["ladder"]["poweroff_in_s"] is not None
    assert client.get("/health").json()["battery"]["level"] == RESERVE


def test_a_sag_while_driving_stops_reads_again_and_clears(served):
    client, bat, body, reading, _ = served
    r = client.post("/wheels", json={"left_rad_s": 2.0, "right_rad_s": 2.0},
                    headers={"x-driver": "twin-dpad"}).json()
    assert r["executed"] is True
    reading(10.05, rest=False)
    reading(10.04, rest=False)
    assert bat["ladder"].check_level == RESERVE
    # The wheels were stopped for the reading at rest, and the brain waits.
    w = client.get("/wheels").json()
    assert (w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]) == (0.0, 0.0)
    r = client.post("/action", json={"action": "FORWARD"}, headers={"x-driver": "brain"}).json()
    assert r["reason"] == BATTERY_CHECK
    reading(11.0)                       # at rest, the pack's own voltage
    assert bat["ladder"].level == OK and bat["ladder"].sags == 1
    r = client.post("/action", json={"action": "LOOK_CENTER"}, headers={"x-driver": "brain"}).json()
    assert r.get("reason") not in (BATTERY_CHECK, BATTERY_RESERVE), r


def test_power_off_refuses_everyone_and_runs_its_steps_in_order_once(served):
    client, bat, body, reading, log = served
    drive_to(reading, 9.95)
    assert bat["ladder"].level == POWEROFF
    assert bat["batt"]["shutdown_done"].wait(10.0)
    assert log.read_text().split() == ["brain", "ros", "vlm", "poweroff"]
    steps = [s["step"] for s in bat["batt"]["shutdown"]]
    assert steps == ["wheels", "brain", "ros", "vlm", "sync", "poweroff"]
    assert bat["batt"]["shutdown"][0]["rc"] == 0          # the wheels at rest first
    for driver in ("brain", "twin-dpad"):
        r = client.post("/action", json={"action": "FORWARD"},
                        headers={"x-driver": driver}).json()
        assert r["executed"] is False and r["reason"] == BATTERY_POWEROFF, driver
    r = client.post("/wheels", json={"left_rad_s": 2.0, "right_rad_s": 2.0},
                    headers={"x-driver": "twin-dpad"}).json()
    assert r["reason"] == BATTERY_POWEROFF
    # A Stop is still everyone's right.
    assert client.post("/stop", headers={"x-driver": "twin-dpad"}).json()["executed"]
    # More readings, more polls: never a second power-off.
    for v in (9.9, 9.8, 9.7):
        reading(v)
    time.sleep(0.2)
    assert log.read_text().split().count("poweroff") == 1


def test_reserve_held_too_long_powers_off_through_the_server(served):
    client, bat, body, reading, log = served
    drive_to(reading, 10.15)
    body.pass_time(bat["settings"]["reserve_max_s"] + 1)
    bat["poll"](force=True)
    assert bat["ladder"].level == POWEROFF
    assert bat["batt"]["shutdown_done"].wait(10.0)
    assert log.read_text().split()[-1] == "poweroff"


def test_the_power_off_command_is_a_stub_off_the_car(monkeypatch):
    monkeypatch.delenv("BATTERY_POWEROFF_CMD", raising=False)
    monkeypatch.delenv("BATTERY_SHUTDOWN_CMDS", raising=False)
    assert server.battery_settings({}, "sim")["poweroff_cmd"] is None
    monkeypatch.setenv("SIM_MOTOR_BOARD", "fake")
    assert server.battery_settings({}, "hardware")["poweroff_cmd"] is None
    monkeypatch.delenv("SIM_MOTOR_BOARD")
    assert server.battery_settings({}, "hardware")["poweroff_cmd"] == "systemctl poweroff"
    monkeypatch.setenv("BATTERY_POWEROFF_CMD", "")
    assert server.battery_settings({}, "hardware")["poweroff_cmd"] is None


def test_a_body_with_no_battery_never_moves_the_ladder(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.delenv("SIM_BATTERY_V", raising=False)
    app = server.create_app()
    client = TestClient(app)
    for _ in range(5):
        app.state.battery["robot"].pass_time(1.0)
        app.state.battery["poll"](force=True)
    assert app.state.battery["ladder"].level == OK
    assert client.get("/battery").json()["usable"] is False


# ---------- nav2 goals ----------

@pytest.fixture
def served_goals(monkeypatch):
    from tests.test_ros_goals import world_with
    from tests.test_ros_world import FakeTruth
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    monkeypatch.setenv("BATTERY_POWEROFF_CMD", "")
    world, bridge = world_with(truth=FakeTruth(1.0, 1.0, 90.0))
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    app = server.create_app()
    client = TestClient(app)
    bat = app.state.battery
    bat["robot"].battery = Pack(12.0)
    return client, bridge, bat


def test_reserve_ends_a_nav2_goal_and_takes_no_new_one(served_goals):
    client, bridge, bat = served_goals
    assert client.post("/world/goal", json={"x_m": 2.0, "y_m": 1.0},
                       headers={"x-driver": "twin-dpad"}).json()["accepted"]
    body = bat["robot"]
    for v in (10.15, 10.14):
        body.battery.v = v
        body.pass_time(0.6)
        bat["poll"](force=True)
    assert bat["ladder"].level == RESERVE
    deadline = time.monotonic() + 3.0
    while bridge.goal is not None and time.monotonic() < deadline:
        time.sleep(0.02)
    assert bridge.goal is None, "the goal outlived Reserve"
    for driver in ("twin-dpad", "brain", ""):
        r = client.post("/world/goal", json={"x_m": 1.0, "y_m": 2.0},
                        headers={"x-driver": driver}).json()
        assert r["accepted"] is False and r["reason"] == BATTERY_RESERVE, driver
    # nav2's own wheel commands are zeroed under drive: direct.
    client.post("/wheels", json={"left_rad_s": 3.0, "right_rad_s": 3.0},
                headers={"x-driver": "ros"})
    w = client.get("/wheels").json()
    assert (w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]) == (0.0, 0.0)


# ---------- the brain ----------

def test_the_brain_client_raises_battery_reserve(served):
    from control.remote_robot import RemoteRobot
    from tests.conftest import ASGI_BASE_URL
    client, bat, body, reading, _ = served
    drive_to(reading, 10.15)
    remote = RemoteRobot(ASGI_BASE_URL, client=client)
    with pytest.raises(BatteryReserve):
        remote.drive_forward()


def test_a_mission_ends_out_of_budget_at_reserve(served):
    from control.mission_runner import OUT_OF_BUDGET, MissionRunner
    from control.remote_robot import RemoteRobot
    from tests.conftest import ASGI_BASE_URL
    client, bat, body, reading, _ = served
    remote = RemoteRobot(ASGI_BASE_URL, client=client)
    runner = MissionRunner(remote, target_object="purple elephant",
                           mission_time_limit_s=600.0, battery_floor_v=None)
    runner.start()
    assert runner.tick()
    drive_to(reading, 10.15)
    for _ in range(20):
        if not runner.tick():
            break
    s = runner.status()
    assert s["outcome"] == OUT_OF_BUDGET, s["log_tail"]
    assert "battery" in s["end_reason"]


def test_a_mission_reads_reserve_from_the_battery_route(served):
    """With a floor set, the brain reads `level` itself and ends at once,
    even if no command of its own was refused yet."""
    from control.mission_runner import OUT_OF_BUDGET, MissionRunner
    from control.remote_robot import RemoteRobot
    from tests.conftest import ASGI_BASE_URL
    client, bat, body, reading, _ = served
    remote = RemoteRobot(ASGI_BASE_URL, client=client)
    runner = MissionRunner(remote, target_object="purple elephant",
                           mission_time_limit_s=600.0, battery_floor_v=9.0)
    runner.start()
    drive_to(reading, 10.15)
    runner.tick()
    s = runner.status()
    assert s["outcome"] == OUT_OF_BUDGET, s["log_tail"]
    assert "reserve" in s["end_reason"]


def test_no_mission_starts_on_a_low_battery(served, monkeypatch):
    from control.brain_server import create_app as brain_app
    from control.remote_robot import RemoteRobot
    from tests.conftest import ASGI_BASE_URL
    client, bat, body, reading, _ = served
    drive_to(reading, 10.40)
    assert bat["ladder"].level == LOW
    remote = RemoteRobot(ASGI_BASE_URL, client=client)
    brain = TestClient(brain_app(robot_factory=lambda: remote))
    r = brain.post("/mission/start", json={"target_object": "purple elephant",
                                           "policy": "frontier"})
    assert r.status_code == 409 and r.json()["detail"].startswith("battery_low"), r.text
    assert brain.get("/mission/status").json().get("running") in (False, None)
