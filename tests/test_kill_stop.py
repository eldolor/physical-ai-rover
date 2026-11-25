# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""PLAN-ros-alignment.md 3.216 -- a robot-server death mid-move.

With the robot server dead nothing vets a move: the board drives its last
setpoint until its own heartbeat zeroes it. So the room the live vet keeps
must already hold that blind travel and the braking after it (criterion 2),
and the heartbeat must still never trip under a live server, which re-sends
the standing command every wheel period. The ground-truth sweep that judges
the death itself is `evaluations/kill-stop-3216/sweep.py` (criterion 1);
these are the pinned facts it rests on.
"""

from robot import hardware_robot, safety
from robot.interface import VERB_PERIOD_S


def _blind_cm(v: float) -> float:
    """Criterion 2's right side: the creep margin, the travel until the
    board's heartbeat (from the last send, at most one period before the
    death), and braking at the placeholder deceleration."""
    t = hardware_robot.HEARTBEAT_MS / 1000.0 + VERB_PERIOD_S
    return safety.CREEP_MARGIN_CM + 100.0 * (v * t + v * v / (2.0 * safety.DECEL_M_S2))


def test_the_stop_distance_holds_the_blind_travel_after_a_death():
    """Criterion 2, every 0.01 m/s up to the board's 2.0 m/s window. Red on
    the 1.5 s heartbeat (48 cm kept at 0.6 m/s against 132 cm needed)."""
    short = [(v / 100.0, round(safety.stop_distance_cm(v / 100.0), 2), round(_blind_cm(v / 100.0), 2))
             for v in range(1, 201)
             if safety.stop_distance_cm(v / 100.0) < _blind_cm(v / 100.0) - 1e-9]
    assert not short, short[:5]


def test_safety_and_the_board_agree_on_the_heartbeat():
    """robot/safety.py keeps its own copy (it imports no backend)."""
    assert safety.BOARD_HEARTBEAT_S == hardware_robot.HEARTBEAT_MS / 1000.0


def test_the_heartbeat_sits_well_above_the_wheel_loop():
    """Criterion 5: at least four wheel periods, so a live server's re-sends
    never let it trip; and it watches the robot server, not the brain -- it
    is no longer tied to the watchdog's 1.0 s."""
    assert hardware_robot.HEARTBEAT_MS / 1000.0 >= 4 * VERB_PERIOD_S - 1e-9
