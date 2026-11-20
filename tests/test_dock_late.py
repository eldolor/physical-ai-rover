# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.194: a dock read late (`docs/plans/ros-alignment/3.194-late-dock.md`).

A mission whose pose was not usable at its start (a reload's first seconds,
a stale estimate under load) used to have no trip home at all: `start()`
read the dock once and nothing read it again, so `return: skipped, no_dock`.
These run the real path -- `MissionRunner` -> explore -> `robot/safety.py`
-> `MockRobot`, nav2 stood in for by `tests/test_dock_return.py`'s RecNav --
with a world whose first pose reads are unusable.

L1 (red on `dev` `d291606c`): a saved-map dock read late is driven home to.
L3: a start-pose dock read after the robot moved is never used."""

import math

from brain.dock import DOCK_TOL_DEG, DOCK_TOL_M
from control.mission_runner import FOUND, OUT_OF_BUDGET
from tests.test_dock_return import mission, run, truth_error
from world.interface import unusable_pose


def counting(nav):
    """FakeNav jumps the robot to a goal without touching the wheels; nav2
    drives them, so its moves reach the odometry. This counts each jump as
    wheel path (3.194's start-pose rule reads the odometry)."""
    jump = nav.get_goal

    def get_goal():
        g = nav.grid
        before = (g.x, g.y)
        out = jump()
        nav.robot._path_m += math.hypot(g.x - before[0], g.y - before[1]) * 0.30
        return out
    nav.get_goal = get_goal
    return nav


class LatePose:
    """A world whose first `n` pose reads are unusable; then the real pose,
    carrying `saved` as the saved map's dock (3.156.E's field) when given."""

    def __init__(self, world, n, saved=None):
        self.world, self.n, self.saved, self.calls = world, n, saved, 0

    def __getattr__(self, name):
        return getattr(self.world, name)

    def get_pose(self):
        self.calls += 1
        if self.calls <= self.n:
            return unusable_pose()
        pose = self.world.get_pose()
        if self.saved is not None and pose.get("usable"):
            pose = {**pose, "dock": dict(self.saved)}
        return pose


def late(n, saved=True, **kw):
    runner, nav, grid, robot = mission(**kw)
    counting(nav)
    x, y = grid.x * 0.30, grid.y * 0.30
    dock = {"x_m": x, "y_m": y, "heading_deg": grid.heading_deg}
    runner.world = LatePose(runner.world, n, dock if saved else None)
    return runner, nav, grid, robot, dock


def test_a_saved_dock_read_late_is_driven_home_to():
    """L1: the pose unusable for the first reads (the robot drives off
    meanwhile), then usable with the saved map's dock: the mission still
    goes home, to that dock."""
    runner, nav, grid, robot, dock = late(5)
    s = run(runner)
    assert s["outcome"] == FOUND, s["log_tail"]
    assert s["return"]["state"] == "docked", s["return"]
    assert s["dock"]["source"] == "saved_map"
    assert s["dock"]["late_s"] >= 0
    assert math.hypot(s["dock"]["x_m"] - dock["x_m"], s["dock"]["y_m"] - dock["y_m"]) < 1e-6
    m, deg = truth_error(grid, s["dock"])
    assert m <= DOCK_TOL_M and deg <= DOCK_TOL_DEG, (m, deg)
    assert any("dock read late" in line for line in runner._log), runner._log[:6]


def test_a_start_pose_read_after_the_robot_moved_is_never_the_dock():
    """L3: no saved dock, and the robot has driven off before the pose is
    usable: the pose then is not where it started, so no dock -- never a
    wrong one."""
    runner, nav, grid, robot, _ = late(5, saved=False)
    s = run(runner)
    assert s["outcome"] == FOUND
    assert s["dock"] is None
    assert s["return"] == {"state": "skipped", "reason": "no_dock"}


def test_a_start_pose_read_late_before_any_move_is_the_dock():
    """The start pose read late is still the dock while odometry says the
    robot has not moved since the start."""
    runner, nav, grid, robot, start = late(1, saved=False)
    runner.start()
    assert runner._dock is None
    robot.pass_time(1.1)                        # past the throttle; nothing moved
    runner._late_dock()
    s = runner.status()
    assert s["dock"]["source"] == "start"
    assert abs(s["dock"]["x_m"] - start["x_m"]) < 1e-6
    assert abs(s["dock"]["y_m"] - start["y_m"]) < 1e-6


def test_the_end_reads_once_more_before_saying_no_dock():
    """The pose usable only after the last search read: the end that goes
    home reads it once more, so `no_dock` means no dock was knowable."""
    runner, nav, grid, robot, dock = late(10 ** 6)
    runner.start()
    for _ in range(4):
        runner.tick()
    runner.world.n = runner.world.calls         # usable from the next read
    runner._dock_late_at = runner.clock()       # the throttle just read
    runner._finish(OUT_OF_BUDGET, "decided")
    assert runner.status()["return"]["state"] == "returning"
    assert runner.status()["dock"]["source"] == "saved_map"


def test_a_pose_usable_at_the_start_reads_no_more():
    """L4: the normal path is unchanged -- the dock is read at the start and
    never again, and carries no `late_s`."""
    runner, nav, grid, robot, _ = late(0)
    s = run(runner)
    assert s["return"]["state"] == "docked"
    assert "late_s" not in s["dock"]
    assert runner.world.calls >= 1
    assert not any("dock read late" in line for line in runner._log)


def test_a_server_restart_after_a_move_is_still_moved():
    """Review: odometry that shrank (the robot server restarted, its path
    from 0 again) or once showed motion counts as moved for good."""
    runner, nav, grid, robot, _ = late(10 ** 6, saved=False)
    runner.start()
    robot._path_m += 3.0                        # drove 3 m
    robot.pass_time(1.1)
    runner._late_dock(force=False)              # pose unusable: no dock, but...
    assert runner._dock_moved                   # ...the try sampled the motion
    robot._path_m = runner._dock_odom0["distance_m"]   # a restart reads "no path"
    runner.world.n = 0                          # and the pose comes back usable
    robot.pass_time(1.1)
    runner._late_dock()
    assert runner._dock is None
    runner2, _, _, robot2, _ = late(10 ** 6, saved=False)
    runner2.start()
    robot2._path_m = runner2._dock_odom0["distance_m"] - 1.0   # shrank
    assert not runner2._not_moved()


def test_the_end_takes_no_start_pose_and_no_navigator_reads_nothing():
    """Review: the end's forced read takes a saved dock only (no odometry
    before the wheels stop); with no navigator nothing is read late."""
    runner, nav, grid, robot, _ = late(1, saved=False)
    runner.start()
    runner._late_dock(force=True)
    assert runner._dock is None
    runner, nav, grid, robot, _ = late(1)
    runner._navigator = None
    runner.start()
    calls = runner.world.calls
    runner._late_dock(force=True)
    assert runner._dock is None and runner.world.calls == calls


def test_off_reads_nothing_late():
    runner, nav, grid, robot, _ = late(5, return_to_dock=False)
    s = run(runner)
    assert s["dock"] is None and s["return"] is None


# -- the committed sweep, pinned (L2-L4; tests/dock_late_sweep.py) -----------

def _sweep(arm):
    import json
    from pathlib import Path
    p = Path(__file__).resolve().parents[1] / "evaluations" / "dock-late-394" / f"{arm}.json"
    return json.loads(p.read_text())


def test_the_sweep_returns_every_mission_whose_saved_dock_became_knowable():
    for arm in ("saved-1", "saved-5", "saved-40"):
        s = _sweep(arm)["summary"][arm]
        assert s["missions"] == 60 and s["knowable"] == 60, (arm, s)
        assert s["return_ran"] == 60 and s["no_dock_when_knowable"] == 0, (arm, s)
        assert s["wrong_dock"] == 0 and s["dock_err_max_m"] <= 0.01, (arm, s)


def test_the_sweep_never_uses_a_start_pose_read_after_a_move():
    s = _sweep("start-5")["summary"]["start-5"]
    assert s["missions"] == 60 and s["wrong_dock"] == 0, s
    assert s["docks_used"] == 0, s          # the robot had always moved first


def test_the_sweep_leaves_the_normal_path_as_dev_had_it():
    """L4: the base arm, mission by mission, equal to `dev` `d291606c`'s."""
    import json
    key = lambda r: json.dumps([r["house"], r["start"], r["job"]])   # noqa: E731
    dev = {key(r): r for r in _sweep("dev-base")["rows"]}
    now = {key(r): r for r in _sweep("base")["rows"]}
    assert len(dev) == len(now) == 60 and dev.keys() == now.keys()
    for k in dev:
        for f in ("outcome", "return", "return_reason", "dock", "end_s", "end_err_m"):
            assert dev[k][f] == now[k][f], (k, f)
