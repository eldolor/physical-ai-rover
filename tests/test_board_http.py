# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_board_http.py

PLAN-ros-alignment.md 3.217: the motor board's commands over its web route,
for a board whose serial receive is dead. The body is the real backend,
`HardwareRobot`, with `command_url` set: commands go to the fake board's
HTTP face (`FakeEsp32Http`), feedback comes back over its pty. Criteria 1-6
and 8 are here in order; 7 (serial untouched) is the rest of the suite.
"""

import random
import time

import pytest
from robot.board_http import COMMAND_STALE_S, COMMAND_TIMEOUT_S, HttpCommandSender
from robot.hardware_robot import HEARTBEAT_MS, WHEEL_RADIUS_M, HardwareRobot
from robot.interface import WheelFeedbackLost
from sim.fake_esp32 import FakeEsp32, FakeEsp32Http
from sim.maps.scaled_house import build_scaled_world
from sim.mock_robot import MockRobot

SPEED = 0.2 / WHEEL_RADIUS_M        # rad/s for 0.2 m/s


def _wait(cond, timeout=2.0, step=0.005):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(step)
    return cond()


def _car(delay_s=0.0):
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    face = FakeEsp32Http(board, delay_s=delay_s)
    robot = HardwareRobot(board.path, sensors=body, command_url=face.url)
    assert _wait(lambda: robot.get_wheel_state()["usable"]), "no first frame"
    return robot, board, face, body


@pytest.fixture
def car():
    made = _car()
    yield made
    made[0].close()
    made[2].close()
    made[1].close()


@pytest.fixture
def slow_car():
    """Every reply held 30 ms: above the 24 ms median measured on the Rover."""
    made = _car(delay_s=0.03)
    yield made
    made[0].close()
    made[2].close()
    made[1].close()


def _drive(robot, seconds, speed=SPEED):
    """Hold a speed as the robot server's wheel loop does: every 50 ms."""
    end = time.monotonic() + seconds
    robot.set_wheel_velocity(speed, speed)
    while time.monotonic() < end:
        time.sleep(0.05)
        robot.advance(0.05)


def test_the_windows_fit_inside_the_boards_heartbeat():
    # One timeout and one retry must land before the board stops itself.
    assert COMMAND_TIMEOUT_S * 1000 < HEARTBEAT_MS / 2 + 1
    assert COMMAND_TIMEOUT_S < COMMAND_STALE_S < 1.0


@pytest.mark.parametrize("url", ["https://10.0.0.1", "http://", "http://u:p@10.0.0.1",
                                 "http://10.0.0.1/js", "http://10.0.0.1/?x=1", "10.0.0.1"])
def test_only_a_bare_http_host_is_a_command_url(url):
    with pytest.raises(ValueError):
        HttpCommandSender(url)


# ---- criterion 1: it drives, and nothing goes down the serial line ----------

def test_it_drives_a_metre_and_the_serial_line_carries_no_command(car):
    robot, board, face, body = car
    start = body.get_wheel_state()
    _drive(robot, 5.0)                      # 0.2 m/s for 5 s
    robot.stop()
    assert _wait(lambda: board.setpoint == [0.0, 0.0])
    time.sleep(0.2)
    def left(state):
        return state["left"]["position_rad"]
    truth = (left(body.get_wheel_state()) - left(start)) * WHEEL_RADIUS_M
    assert truth > 0.9, f"drove {truth:.3f} m"
    assert abs(left(robot.get_wheel_state()) * WHEEL_RADIUS_M - truth) < 0.01
    assert len(face.wheel_commands()) > 50
    assert board.lines_in == len(face.received), "a command went down the pty"
    # The set-up went the same way, heartbeat first.
    assert [c["T"] for _, c in face.received[:2]] == [136, 131]
    assert board.heartbeat_ms == HEARTBEAT_MS


# ---- criterion 2: a stop is not delayed by the network ----------------------

def test_a_stop_returns_at_once_and_lands_within_150_ms(slow_car):
    robot, board, _, _ = slow_car
    for i in range(200):
        robot.set_wheel_velocity(SPEED, SPEED)
        assert _wait(lambda: board.setpoint != [0.0, 0.0], timeout=1.0), i
        t0 = time.monotonic()
        robot.stop()
        took = time.monotonic() - t0
        assert took < 0.020, f"stop() took {took * 1000:.1f} ms on try {i}"
        assert _wait(lambda: board.setpoint == [0.0, 0.0], timeout=0.150 - took, step=0.002), \
            f"the board still drove 150 ms after stop {i}"


# ---- criterion 3: a stale speed never follows a stop ------------------------

def test_the_board_ends_on_the_last_command_issued():
    rng = random.Random(3217)
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    face = FakeEsp32Http(board, delay_s=lambda: rng.choice((0.0, 0.0, 0.0, 0.001, 0.005, 0.04)))
    sender = HttpCommandSender(face.url)
    pick = random.Random(1)
    try:
        for trial in range(1000):
            last = None
            for _ in range(pick.randint(1, 6)):
                speed = pick.choice((0.0, 0.0, 0.1, 0.2, -0.1))
                last = {"T": 1, "L": speed, "R": speed}
                assert sender.submit(last)
                if pick.random() < 0.3:
                    time.sleep(pick.choice((0.001, 0.01)))
            assert sender.flush(2.0), trial
            got = face.wheel_commands()[-1][1]
            assert got == last, f"trial {trial}: issued {last}, the board ended on {got}"
            assert board.setpoint == [last["L"], last["R"]]
    finally:
        sender.close()
        face.close()
        board.close()


# ---- criterion 4: a dead route refuses motion and never traps a stop --------

def test_a_dead_route_refuses_motion_and_the_heartbeat_stops_the_wheels(car):
    robot, board, face, _ = car
    _drive(robot, 0.3)
    last_delivered = face.wheel_commands()[-1][0]
    face.refuse()
    cut = time.monotonic()
    raised = None
    while time.monotonic() - cut < 1.0:
        time.sleep(0.05)
        try:
            robot.advance(0.05)
        except WheelFeedbackLost as e:
            raised = (time.monotonic() - cut, str(e))
            break
    assert raised, "a dead route never refused motion"
    assert raised[0] < COMMAND_STALE_S + COMMAND_TIMEOUT_S, raised
    assert "web route" in raised[1]
    assert robot._cmd == (0.0, 0.0)
    with pytest.raises(WheelFeedbackLost):
        robot.set_wheel_velocity(SPEED, SPEED)
    assert robot.stop() == {"action": "stop"}
    # The board stopped itself, on its own clock.
    assert _wait(lambda: board.heartbeat_stopped, timeout=0.5, step=0.002)
    assert board.setpoint == [0.0, 0.0]
    assert time.monotonic() - last_delivered < 0.25 + raised[0], "the heartbeat was late"
    assert robot.feedback_status()["command"]["refused"] >= 1


def test_a_route_that_comes_back_drives_again(car):
    robot, board, face, _ = car
    face.lose_next(n=6, hang_s=0.3)         # down for longer than the stale window
    robot.set_wheel_velocity(SPEED, SPEED)
    with pytest.raises(WheelFeedbackLost):
        for _ in range(20):
            time.sleep(0.05)
            robot.advance(0.05)
    assert _wait(lambda: not robot._http.stale(), timeout=3.0)
    robot.set_wheel_velocity(SPEED, SPEED)
    assert _wait(lambda: board.setpoint != [0.0, 0.0], timeout=1.0)
    robot.stop()


# ---- criterion 5: one lost request does not trip the 200 ms heartbeat -------

def test_one_lost_request_does_not_trip_the_heartbeat():
    for run in range(20):
        robot, board, face, _ = _car()
        try:
            _drive(robot, 0.3)
            fired = []
            face.lose_next(n=1, hang_s=1.0)
            end = time.monotonic() + 0.8
            while time.monotonic() < end:
                time.sleep(0.05)
                robot.advance(0.05)
                fired.append(board.heartbeat_stopped)
            times = [t for t, c in face.wheel_commands() if c["L"]]
            gap = max(b - a for a, b in zip(times, times[1:]))
            assert not any(fired), f"run {run}: the board's heartbeat fired"
            assert gap < HEARTBEAT_MS / 1000, f"run {run}: {gap * 1000:.0f} ms between T:1s"
            assert robot.feedback_status()["command"]["failed"] == 1
        finally:
            robot.close()
            face.close()
            board.close()


# ---- criterion 6: the wheel loop is not held --------------------------------

def test_a_wheel_command_never_waits_on_the_network(slow_car):
    robot, _, _, _ = slow_car
    took = []
    for i in range(1000):
        t0 = time.perf_counter()
        robot.set_wheel_velocity(SPEED, SPEED)
        took.append(time.perf_counter() - t0)
        if i % 50 == 0:
            time.sleep(0.01)
    took.sort()
    assert took[989] < 0.002, f"p99 {took[989] * 1000:.2f} ms"
    assert robot.feedback_status()["command"]["coalesced"] > 900


# ---- criterion 8: /health says which route ----------------------------------

def test_health_describes_the_route_and_never_judges_it(car):
    robot, _, face, _ = car
    status = robot.feedback_status()
    assert status["command_route"] == "http"
    assert status["command"]["url"] == face.url
    assert _wait(lambda: robot.feedback_status()["command"]["sent"] >= 2)    # the set-up
    for key in ("undelivered_age_s", "failed", "coalesced", "refused", "dropped"):
        assert key in status["command"]


def test_the_serial_route_says_so():
    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    robot = HardwareRobot(board.path, sensors=body)
    try:
        status = robot.feedback_status()
        assert status["command_route"] == "serial" and status["command"] is None
    finally:
        robot.close()
        board.close()


def test_the_factory_reads_the_command_url(monkeypatch):
    from robot.factory import _backend

    body = MockRobot(build_scaled_world(), render=False)
    board = FakeEsp32(body)
    face = FakeEsp32Http(board)
    monkeypatch.setenv("ROBOT_SERIAL", board.path)
    monkeypatch.setenv("ROBOT_COMMAND_URL", face.url)
    monkeypatch.delenv("SIM_MOTOR_BOARD", raising=False)
    robot = _backend({"mode": "hardware"})
    try:
        assert robot.feedback_status()["command_route"] == "http"
        assert _wait(lambda: board.heartbeat_ms == HEARTBEAT_MS)
    finally:
        robot.close()
        face.close()
        board.close()
