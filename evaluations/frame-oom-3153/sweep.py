# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/frame-oom-3153/sweep.py -- 3.153 criterion 2, in process.

    .venv/bin/python evaluations/frame-oom-3153/sweep.py [--out results.json]

For each case, the mission is run twice through `MissionRunner`: once as is
(the reference), once with ONE frame read failing (the robot server's 503,
as `RemoteRobot` raises it) at a seeded read between the second and the
reference's last. Bars (written before running):
* every injected mission ends with the reference's outcome;
* a camera that never comes back from a seeded read ends `failed` after
  exactly 3 consecutive failed ticks, with the robot's pose unchanged from
  the first failed read to the end (a failed tick never moves it).

Explore missions use `tests/test_explore.py`'s harness (FakeNav, sim time);
tiered ones `tests/test_local_vlm_down.py`'s (the brain server's own
`_tiered_vision_fn`, no cloud, with and without a stub local VLM).
"""

import argparse
import json
import logging
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from control.mission_runner import DEFAULT_MAX_CAMERA_FAILURES, FAILED  # noqa: E402
from control.remote_robot import RobotTransportError  # noqa: E402

EXPLORE = [("scaled_house", "red backpack"), ("scaled_house", "toy car"),
           ("home_first_floor", "red backpack"), ("home_first_floor", "toy car"),
           ("complex_house", "red backpack"), ("complex_house", "toy car")]
TIERED = [("procthor:val:2", "basketball", None), ("procthor:val:2", "basketball", "up"),
          ("procthor:val:2", "basketball", "down")]
SEEDS = (0, 1, 2)


def flaky(cls):
    class Flaky(cls):
        fail_at = None          # the 1-based read that fails
        forever = False         # and every one after it
        reads = 0
        pose_at_fail = None

        def get_camera_frame(self):
            self.reads += 1
            if self.fail_at is not None and (self.reads == self.fail_at
                                             or (self.forever and self.reads > self.fail_at)):
                if self.pose_at_fail is None:
                    self.pose_at_fail = (self.world.x, self.world.y, self.world.theta)
                raise RobotTransportError('GET /frame -> HTTP 503: {"detail":"timed out"}')
            return super().get_camera_frame()
    return Flaky


def explore_run(house, target, fail_at=None, forever=False):
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld
    from tests.test_explore import FakeNav
    grid = build_world(house)
    robot = flaky(MockRobot)(grid, render=False)
    robot.fail_at, robot.forever = fail_at, forever
    runner = MissionRunner(robot, target_object=target, policy="explore", navigator=FakeNav(robot),
                           world=MockWorld(grid), max_steps=80,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < 5000:
        ticks += 1
    return _row(runner.status(), robot)


def tiered_run(house, target, ve_mode, fail_at=None, forever=False):
    from _pytest.monkeypatch import MonkeyPatch
    import tests.test_local_vlm_down as t
    made = {}

    class Cam(flaky(t.Camera)):
        def __init__(self, grid):
            super().__init__(grid)
            self.fail_at, self.forever = fail_at, forever
            made["robot"] = self

    mp = MonkeyPatch()
    try:
        mp.setattr(t, "Camera", Cam)
        mp.setattr(t, "HOUSE", house)
        mp.setattr(t, "TARGET", target)
        ve = t.StubVE(target, ve_mode) if ve_mode else None
        got, _ = t._mission(mp, ve)
    finally:
        mp.undo()
    robot = made["robot"]
    return {"outcome": got["outcome"], "step": got["step"], "reads": robot.reads,
            **_moved(robot), "error": None}


def _moved(robot):
    if robot.pose_at_fail is None:
        return {"moved_after_fail": None}
    w = robot.world
    return {"moved_after_fail": (w.x, w.y, w.theta) != robot.pose_at_fail}


def _row(status, robot):
    return {"outcome": status["outcome"], "step": status.get("step"), "reads": robot.reads,
            "error": status.get("error"), **_moved(robot)}


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(Path(__file__).with_name("results.json")))
    args = ap.parse_args(argv)
    logging.disable(logging.WARNING)
    rows = []
    cases = [("explore", h, tg, None) for h, tg in EXPLORE] + [("tiered", h, tg, m) for h, tg, m in TIERED]
    for policy, house, target, mode in cases:
        run = (lambda **kw: explore_run(house, target, **kw)) if policy == "explore" \
            else (lambda **kw: tiered_run(house, target, mode, **kw))
        ref = run()
        for seed in SEEDS:
            rng = random.Random(f"3153-{policy}-{house}-{target}-{mode}-{seed}")
            k = rng.randint(2, max(2, ref["reads"] - 1))
            got = run(fail_at=k)
            rows.append({"kind": "one", "policy": policy, "house": house, "target": target,
                         "ve": mode, "fail_at": k, "ref": ref, "got": got,
                         "same": got["outcome"] == ref["outcome"]})
            print(json.dumps(rows[-1]), flush=True)
        k = random.Random(f"3153-forever-{policy}-{house}-{target}-{mode}").randint(2, max(2, ref["reads"] - 1))
        got = run(fail_at=k, forever=True)
        ok = (got["outcome"] == FAILED and got["reads"] == k + DEFAULT_MAX_CAMERA_FAILURES - 1
              and got["moved_after_fail"] is False)
        rows.append({"kind": "forever", "policy": policy, "house": house, "target": target,
                     "ve": mode, "fail_at": k, "got": got, "ok": ok})
        print(json.dumps(rows[-1]), flush=True)
    one = [r for r in rows if r["kind"] == "one"]
    forever = [r for r in rows if r["kind"] == "forever"]
    verdict = {"one_same": f"{sum(r['same'] for r in one)}/{len(one)}",
               "forever_ok": f"{sum(r['ok'] for r in forever)}/{len(forever)}",
               "met": all(r["same"] for r in one) and all(r["ok"] for r in forever)}
    Path(args.out).write_text(json.dumps({"verdict": verdict, "rows": rows}, indent=1) + "\n")
    print(json.dumps(verdict))
    return 0 if verdict["met"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
