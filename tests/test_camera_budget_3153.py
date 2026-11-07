# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.153 criterion 1: a camera read that fails is a transient fault with a
budget. 3.152.B's board run ended a mission `failed` on one `GET /frame`
503 ("timed out": the sim's sensor program stalled in a VE kill's OOM
squeeze); `MissionRunner._tick`'s catch-all ended it on the first robot
error, while B3.2's budget covered only the vision path. Now a frame read
through the mission's gate that raises is `CameraUnavailable`: that tick
takes no further action and stops the car, and `DEFAULT_MAX_CAMERA_FAILURES`
(3) consecutive such ticks end the mission `failed`; a completed tick resets
the count (`status.camera_failures`). Every other robot error still ends it
at once."""

from control.mission_runner import (
    CAMERA_RETRY_S, DEFAULT_MAX_CAMERA_FAILURES, FAILED, FOUND, MissionRunner,
)
from control.remote_robot import RobotTransportError
from tests.conftest import RecordingRobot, fresh_mock_robot, mock_world_for

BUDGET = 150
MOVES = {"drive_forward", "reverse", "turn_left", "turn_right", "set_wheel_velocity"}


class FlakyCamera(RecordingRobot):
    """A recording robot whose frame reads number `fail` (1-based) raise
    the robot server's 503, as `RemoteRobot` raises it."""

    def __init__(self, delegate, fail=(), forever_from=None, on_fail=None):
        super().__init__(delegate)
        self.reads = 0
        self.fail = set(fail)
        self.forever_from = forever_from
        self.on_fail = on_fail

    def get_camera_frame(self) -> dict:
        self.reads += 1
        if self.reads in self.fail or (self.forever_from and self.reads >= self.forever_from):
            self.calls.append("frame_failed")
            if self.on_fail is not None:
                self.on_fail()
            raise RobotTransportError('GET /frame -> HTTP 503: {"detail":"timed out"}')
        return super().get_camera_frame()


def runner_for(robot, **kw):
    return MissionRunner(robot, target_object="red backpack", max_steps=BUDGET,
                         world=mock_world_for(robot), **kw)


def drive(runner, max_ticks=500, seen=None):
    ticks = 0
    while runner.tick():
        if seen is not None:
            seen.append(runner.status())
        ticks += 1
        assert ticks < max_ticks, "runner never terminated"
    return runner.status()


def reference_outcome():
    runner = runner_for(RecordingRobot(fresh_mock_robot()))
    runner.start()
    return drive(runner)


def test_one_failed_frame_read_mid_mission_does_not_fail_it():
    """Red before 3.153: the first 503 ended the mission `failed`."""
    ref = reference_outcome()
    assert ref["outcome"] == FOUND
    robot = FlakyCamera(fresh_mock_robot(), fail={3})
    runner = runner_for(robot)
    runner.start()
    seen = []
    status = drive(runner, seen=seen)
    assert status["outcome"] == FOUND and status["error"] is None
    failed = [s for s in seen if s["camera_failures"] == 1]
    assert failed, "the failed read was never counted"
    assert any("camera unavailable 1/3" in line for line in failed[0]["log_tail"])
    assert status["camera_failures"] == 0, "a completed tick resets the count"
    i = robot.calls.index("frame_failed")
    assert robot.calls[i + 1] == "stop", "the car stops on a blind tick"


def test_a_camera_that_never_comes_back_ends_failed_after_three():
    robot = FlakyCamera(fresh_mock_robot(), forever_from=4)
    runner = runner_for(robot)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FAILED
    assert status["error"].startswith(f"camera unavailable {DEFAULT_MAX_CAMERA_FAILURES} times in a row")
    assert "timed out" in status["error"]
    assert robot.reads == 3 + DEFAULT_MAX_CAMERA_FAILURES, "exactly three failed ticks"
    first = robot.calls.index("frame_failed")
    assert not MOVES & set(robot.calls[first:]), "a failed tick moved the robot"


def test_the_budget_counts_consecutive_ticks_only():
    """Failures 1-2, a good tick, then 2 more: never 3 in a row."""
    robot = FlakyCamera(fresh_mock_robot(), fail={2, 3, 5, 6})
    runner = runner_for(robot)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FOUND


def test_every_other_robot_error_still_ends_the_mission_at_once():
    class DeadWheels(RecordingRobot):
        def drive_forward(self, speed=50, duration=0.5):
            raise RobotTransportError("POST /action -> HTTP 503")
        set_wheel_velocity = None

    robot = DeadWheels(fresh_mock_robot())

    def fail_wheels(left, right):
        raise RobotTransportError("POST /wheels -> HTTP 503")
    robot.set_wheel_velocity = fail_wheels
    runner = runner_for(robot)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FAILED
    assert status["error"].startswith("step failed:")
    assert status["camera_failures"] == 0


def test_a_stop_during_a_failed_read_is_unchanged():
    """stop() lands while the read is in flight, then the read fails: the
    mission is `cancelled` by the stop, the failure is not counted, and no
    second stop is sent by the loser of the race."""
    holder = {}
    robot = FlakyCamera(fresh_mock_robot(), fail={2},
                        on_fail=lambda: holder["runner"].stop())
    runner = holder["runner"] = runner_for(robot)
    runner.start()
    drive(runner)
    status = runner.status()
    assert status["outcome"] == "cancelled"
    assert status["camera_failures"] == 0
    i = robot.calls.index("frame_failed")
    assert robot.calls[i + 1:].count("stop") == 1, "only stop()'s own stop"


def test_a_read_that_fails_at_once_waits_between_tries():
    """The car's stale-frame 503 answers at once: each failed tick under the
    budget idles CAMERA_RETRY_S, so three failures span at least 1 s."""
    robot = FlakyCamera(fresh_mock_robot(), forever_from=2)
    waits = []
    runner = runner_for(robot, idle=waits.append)
    runner.start()
    status = drive(runner)
    assert status["outcome"] == FAILED
    assert waits.count(CAMERA_RETRY_S) == DEFAULT_MAX_CAMERA_FAILURES - 1
