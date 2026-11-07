# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.167: one sweep mission's true pose after every tick, from a given
checkout, so two commits that take the same actions can be compared where
the robot actually went (3.60's sweep, seed 3463).

    .venv/bin/python evaluations/inventory-precision-3167/pose_trace.py <checkout> <mission 1-20> <out.json>
"""
import json
import random
import sys
from pathlib import Path

SEED = 3463


def main(checkout, mission, out):
    sys.path.insert(0, str(Path(checkout).resolve()))
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from tests import inventory_sweep as sw
    from tests.conftest import mock_world_for
    k = int(mission) - 1
    (cx, cy), theta, _ = sw.starts(seed=SEED)[k]
    grid = build_world("complex_house")
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    robot = MockRobot(grid, render=False)
    det = sw.ErringDetector(grid, random.Random(SEED * 100 + k), noisy=True)
    runner = MissionRunner(robot, target_object=sw.ABSENT_TARGET, max_steps=sw.MAX_STEPS,
                           policy="frontier", world=mock_world_for(robot),
                           inventory=True, detections_fn=det)
    trace = []
    runner.start()
    while runner.tick():
        a = runner.memory.actions[-1].action if runner.memory.actions else None
        trace.append([a, round(grid.x, 4), round(grid.y, 4), round(grid.theta, 4)])
    Path(out).write_text(json.dumps(trace) + "\n")


if __name__ == "__main__":
    main(*sys.argv[1:4])
