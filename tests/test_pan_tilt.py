# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
# GPL-3.0-only: it pins sim/fake_esp32.py's reimplementation of Waveshare's GPL-3.0 firmware.
"""
tests/test_pan_tilt.py

`PLAN-ros-alignment.md` 3.220 -- the Rover's pan-tilt camera, driven
through the motor board. Criteria 1-5 of the section file; each fake-board
rule cites the function in `waveshareteam/ugv_base_ros` @ 2e7df97,
`ROS_Driver/`, that it mirrors.
"""

import json
import os
import time

import pytest

from robot.hardware_robot import (
    LOOK_TILT_DEG, PAN_LIMIT_DEG, PAN_SETTLE_TIMEOUT_S, TILT_LIMITS_DEG, HardwareRobot)
from sim.fake_esp32 import FakeEsp32, FakeEsp32Http, _arduino_map
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import MockRobot


def _wait(pred, timeout=2.0, step=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(step)
    return pred()


@pytest.fixture
def board():
    body = MockRobot(build_starter_world(), render=False)
    b = FakeEsp32(body)
    yield b, body
    b.close()


@pytest.fixture
def rig(board):
    """The host over the pty with the gimbal on, the body as its sensors."""
    b, body = board
    bot = HardwareRobot(b.path, sensors=body, gimbal=True)
    assert _wait(lambda: (bot.gimbal_status() or {}).get("settled"))
    yield b, body, bot
    bot.close()


def _send(board, obj):
    board._handle(json.dumps(obj).encode())


def _frame(board):
    """One baseInfoFeedback() frame, as the board would write it."""
    got = []
    with board.lock:
        write = board._write
        board._write = lambda obj: got.append(obj) or True
        try:
            board._last_feedback_at = -1e9
            board._feedback()
        finally:
            board._write = write
    return got[-1]


# ---------- criterion 4: the fake board follows the firmware ----------

def test_the_simple_move_clamps_and_maps_like_the_firmware(board):
    """gimbalCtrlSimple(): constrainFloat(X, -180, 180), (Y, -30, 90), then
    2047 + map(X, 0, 360, 0, 4095) and 2047 - map(Y, ...), where Arduino's
    map() takes longs: 45.9 deg is 45, and C division truncates."""
    b, _ = board
    _send(b, {"T": 133, "X": 45.9, "Y": -50, "SPD": 0, "ACC": 0})
    assert b.servo_goal == [2047 + 511, 2047 + 341]
    _send(b, {"T": 133, "X": 999, "Y": 999})
    assert b.servo_goal == [2047 + 2047, 2047 - 1023]
    _send(b, {"T": 134, "X": -90, "Y": 0, "SX": 100, "SY": 100})   # gimbalCtrlMove()
    assert b.servo_goal == [2047 - 1023, 2047]
    assert _arduino_map(-30.7, 0, 360, 0, 4095) == -341


def test_the_angle_is_in_the_feedback_only_in_module_type_two(board):
    """baseInfoFeedback(): case 2 adds pan/tilt; changeModuleType() (T:4) is
    RAM only, so a reboot goes back to the boot mission's module; T:900
    writes it there."""
    b, _ = board
    assert b.module_type == 0 and "pan" not in _frame(b)
    _send(b, {"T": 4, "cmd": 2})
    f = _frame(b)
    assert f["pan"] == pytest.approx(0.0) and f["tilt"] == pytest.approx(0.0)
    b.reboot()
    assert b.module_type == 0 and "pan" not in _frame(b)
    _send(b, {"T": 900, "main": 2, "module": 2})
    b.reboot()
    assert b.module_type == 2 and "pan" in _frame(b)


def test_the_command_moves_the_servos_in_any_module_type_at_their_speed(board):
    """uart_ctrl.h dispatches 133 whatever the module; the servo travels."""
    b, body = board
    _send(b, {"T": 133, "X": 90, "Y": 0})
    time.sleep(0.1)
    mid = b.gimbal_angles[0]
    assert 5 < mid < 60
    assert _wait(lambda: abs(b.gimbal_angles[0] - 89.93) < 0.05, 1.0)
    # The body's camera follows the servo, not the command (grid_world's
    # pan is in 90-degree units, clockwise).
    assert body.world.pan == pytest.approx(b.gimbal_angles[0] / 90.0, abs=0.02)


def test_the_stop_holds_where_the_servo_is(board):
    """gimbalCtrlStop(): torque off for SERVO_STOP_DELAY, then on."""
    b, _ = board
    _send(b, {"T": 133, "X": -90, "Y": 0})
    time.sleep(0.12)
    _send(b, {"T": 135})
    held = b.gimbal_angles[0]
    time.sleep(0.3)
    assert b.gimbal_angles[0] == pytest.approx(held, abs=0.5)
    assert -85 < held < -5


def test_a_board_never_commanded_leaves_the_bodys_pan_alone(board):
    b, body = board
    body.look_left()
    time.sleep(0.05)
    assert body.world.pan == -1


def test_the_web_face_runs_the_same_handler(board):
    import urllib.parse
    import urllib.request
    b, _ = board
    face = FakeEsp32Http(b)
    try:
        q = urllib.parse.quote(json.dumps({"T": 133, "X": -45, "Y": 0}))
        urllib.request.urlopen(f"{face.url}/js?json={q}", timeout=2).read()
        assert b.servo_goal[0] == 2047 - 511
    finally:
        face.close()


# ---------- criterion 1: the limits ----------

def test_every_gimbal_command_is_inside_the_limits(rig):
    b, _, bot = rig
    for look in (bot.look_left, bot.look_right, bot.look_center, bot.look_left):
        look()
    bot._point(400, 80)
    bot._point(-400, -80)
    assert _wait(lambda: len(b.gimbal_commands) == 7)
    sent = [c for c in b.gimbal_commands if c.get("T") == 133]
    assert len(sent) == 7                          # set-up's centring + 6
    assert all(abs(c["X"]) <= PAN_LIMIT_DEG for c in sent)
    assert all(TILT_LIMITS_DEG[0] <= c["Y"] <= TILT_LIMITS_DEG[1] for c in sent)
    assert all(c["Y"] == LOOK_TILT_DEG for c in sent[:5])
    assert sent[-2]["X"] == 90 and sent[-1]["X"] == -90


def test_the_set_up_turns_the_report_on_and_centres_the_camera(board):
    b, body = board
    _send(b, {"T": 133, "X": 60, "Y": 0})
    bot = HardwareRobot(b.path, sensors=body, gimbal=True)
    try:
        assert _wait(lambda: b.module_type == 2)
        assert _wait(lambda: (bot.gimbal_status() or {}).get("settled"))
        assert abs(b.gimbal_angles[0]) < 0.1
    finally:
        bot.close()


def test_off_the_looks_are_the_sensors_as_before(board):
    b, body = board
    bot = HardwareRobot(b.path, sensors=body)
    try:
        assert bot.look_left() == {"action": "look_left", "pan": -1}
        assert bot.gimbal_status() is None
        assert b.gimbal_commands == []
        time.sleep(0.1)
        assert b.module_type == 0
    finally:
        bot.close()


# ---------- criterion 2: pan_deg is what the servo reached ----------

def test_at_rest_the_frame_carries_the_measured_pan(rig):
    b, _, bot = rig
    worst = 0.0
    for look, goal in ((bot.look_left, -90), (bot.look_right, 90), (bot.look_center, 0)):
        answer = look()
        assert answer["goal_pan_deg"] == goal
        assert answer["settled"] is False             # it has only just been sent
        assert _wait(lambda: bot.gimbal_status()["settled"], 1.0)
        assert _wait(lambda: b.servo_pos[0] == b.servo_goal[0], 1.0)
        time.sleep(0.1)                               # at rest: one frame since
        for _ in range(10):
            frame = bot.get_camera_frame()
            worst = max(worst, abs(frame["pan_deg"] - b.gimbal_angles[0]))
            time.sleep(0.02)
    assert worst <= 1.0, worst


def test_while_moving_the_report_is_where_the_servo_is_not_the_goal(rig):
    b, _, bot = rig
    bot.look_right()
    time.sleep(0.12)
    g = bot.gimbal_status()
    assert g["settled"] is False
    assert g["pan_deg"] < 80
    assert abs(g["pan_deg"] - b.gimbal_angles[0]) < 270 * 0.06 + 1


def test_a_board_that_does_not_report_the_angle_is_never_panned(rig):
    b, _, bot = rig
    _send(b, {"T": 4, "cmd": 0})
    assert _wait(lambda: not bot.gimbal_status()["measured"], 1.0)
    n = len(b.gimbal_commands)
    answer = bot.look_left()
    assert answer["pan"] is None and answer["reason"].startswith("gimbal_unmeasured")
    assert len(b.gimbal_commands) == n
    assert bot.get_camera_frame()["pan_deg"] is None


# ---------- criterion 3: a lost Wi-Fi command ----------

def test_a_lost_command_is_never_reported_reached_and_raises_a_fault(board):
    b, body = board
    face = FakeEsp32Http(b)
    bot = HardwareRobot(b.path, sensors=body, gimbal=True, command_url=face.url)
    try:
        assert _wait(lambda: (bot.gimbal_status() or {}).get("settled"))
        face.lose_next(10_000, hang_s=0.15)
        t0 = time.monotonic()
        bot.look_left()
        worst = 0.0
        while bot.gimbal_status()["fault"] is None and time.monotonic() - t0 < 3:
            g = bot.gimbal_status()
            if g["pan_deg"] is not None:
                worst = max(worst, abs(g["pan_deg"] - b.gimbal_angles[0]))
            time.sleep(0.02)
        took = time.monotonic() - t0
        g = bot.gimbal_status()
        assert g["fault"] and g["pan_deg"] is None and g["settled"] is False
        assert took <= PAN_SETTLE_TIMEOUT_S + 0.25, took
        assert worst <= 1.0 and abs(b.gimbal_angles[0]) < 0.1     # it never moved
        assert bot.feedback_status()["gimbal"]["fault"]
        # The route comes back: the queued command is retried, reached, cleared.
        face.lose_next(0)
        assert _wait(lambda: bot.gimbal_status()["settled"], 3.0)
        assert bot.gimbal_status()["fault"] is None
        assert bot.gimbal_status()["pan_deg"] == pytest.approx(-89.93, abs=1.0)
    finally:
        bot.close()
        face.close()


def test_a_look_never_changes_a_wheel_command(rig):
    b, _, bot = rig
    bot.set_wheel_velocity(1.0, 1.0)
    bot.look_left()
    assert bot._cmd == (1.0, 1.0)
    bot.stop()


# ---------- criterion 5: the twin's look buttons, through the robot server ----------

def test_the_look_buttons_route_turns_the_camera(sim_programs, monkeypatch):
    """The buttons post LOOK_* to /action (web-twin/app.js manualAction());
    in mode: hardware on the fake board that reaches the gimbal."""
    from robot.server import create_app
    from tests.conftest import ASGI_BASE_URL, asgi_client

    programs = sim_programs()
    programs.apply(monkeypatch)
    monkeypatch.delenv("ROBOT_DRIVE", raising=False)
    monkeypatch.delenv("ROBOT_GIMBAL", raising=False)
    monkeypatch.setenv("WORLD_MODE", "none")
    client = asgi_client(create_app())

    def act(action):
        return client.post(ASGI_BASE_URL + "/action", json={"action": action},
                           headers={"x-driver": "twin-dpad"}).json()

    time.sleep(0.5)
    r = act("LOOK_LEFT")
    assert r["executed"] is True, r
    def gimbal():
        return client.get(ASGI_BASE_URL + "/health").json()["motor_board"]["gimbal"]

    t0 = time.monotonic()
    ok = _wait(lambda: abs((gimbal()["pan_deg"] or 99) + 90) <= 1.0, 1.0)
    took = time.monotonic() - t0
    assert ok and took <= 1.0, (gimbal(), took)
    assert act("LOOK_CENTER")["executed"] is True
    assert _wait(lambda: abs(gimbal()["pan_deg"] if gimbal()["pan_deg"] is not None
                             else 99) <= 1.0, 1.0)
