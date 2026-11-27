# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/explore-sight-3227/sweep.py

3.227 section A's sweep: plain explore, in process, through MissionRunner ->
ExploreAgent -> SafetyController -> MockRobot, nav2 as FakeNav, on 3.80.G's 21
houses. Free: no model, no cloud.

Arms (section A): F dev's plain explore on sim frames; S this branch's `auto`
on sim frames; B the plain explore on a real-camera stand-in (frames stripped
of `detections` / `objects_visible`, provenance not `sim`); R this branch on
the stand-in, sightings from a perception stand-in reading the stripped
detections; G `perception` in the sim with the geometry's identity.

    python evaluations/explore-sight-3227/sweep.py F [S B R G]
"""

import json
import logging
import math
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from brain.perceive import FrameReportedPipeline  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from sim.mock_world import MockWorld  # noqa: E402
from tests.test_explore import FakeNav  # noqa: E402

sys.path.insert(0, str(ROOT / "evaluations/tiered-380/E"))
from pilot import SWEEP_HOUSES  # noqa: E402

TARGET = "red backpack"
CELL_M = 0.3
AT_TARGET_M = 0.9
OUT = Path(__file__).with_name("results")


class StandInRobot(MockRobot):
    """A real camera's frames: no `detections`, no `objects_visible`, and a
    provenance that is not `sim`. The stripped detections are kept on the
    robot (never in the frame) for the perception stand-in alone."""

    last_truth = None

    def get_camera_frame(self):
        frame = super().get_camera_frame()
        self.last_truth = {"detections": frame.pop("detections", None)}
        frame.pop("objects_visible", None)
        frame["metadata"] = {**(frame.get("metadata") or {}), "source": "stand-in"}
        return frame


class StandInPipeline(FrameReportedPipeline):
    """The detector replaced by truth, the consumer real (1.12): perceives the
    detections the stand-in robot stripped from the frame it just served."""

    def __init__(self, robot, target):
        super().__init__(target)
        self.robot = robot

    def perceive(self, frame):
        return super().perceive(self.robot.last_truth or {})


def target_m(grid):
    cells = [c for c, n in grid.objects.items() if n == TARGET]
    return min(math.dist((grid.x, grid.y), (cx + 0.5, cy + 0.5)) for cx, cy in cells) * CELL_M


def build(arm, grid):
    standin = arm in ("B", "R")
    robot = (StandInRobot if standin else MockRobot)(grid, render=False)
    vision_fn = None
    if arm in ("R", "G"):
        from brain.explore_sight import PerceptionSight
        if arm == "R":
            vision_fn = PerceptionSight(StandInPipeline(robot, TARGET))
        else:
            from brain.arrival import geometry_identity
            import functools
            vision_fn = PerceptionSight(FrameReportedPipeline(TARGET),
                                        identity_fn=functools.partial(geometry_identity,
                                                                      target=TARGET))
    return robot, vision_fn


def run(arm, house):
    grid = build_world(house)
    robot, vision_fn = build(arm, grid)
    runner = MissionRunner(robot, target_object=TARGET, policy="explore", vision_fn=vision_fn,
                           navigator=FakeNav(robot), world=MockWorld(grid),
                           mission_time_limit_s=1200,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < 20000:
        ticks += 1
    s = runner.status()
    agent = runner.agent
    return {"arm": arm, "house": house, "outcome": s["outcome"], "sim_s": round(grid.sim_time, 1),
            "goals": s["explore"]["goals_sent"], "end_target_m": round(target_m(grid), 2),
            "sighted": len(getattr(agent, "_sighted", ()) or ()),
            "close_ins": getattr(agent, "close_ins", 0),
            "sight_from": getattr(agent, "sight_from", None),
            "tail": s["log_tail"][-1:]}


def summary(rows):
    at = [r for r in rows if r["end_target_m"] <= AT_TARGET_M]
    return {"missions": len(rows),
            "found": sum(r["outcome"] == "found" for r in rows),
            "arrived_unconfirmed": sum(r["outcome"] == "arrived_unconfirmed" for r in rows),
            "arrived_unconfirmed_at_target": sum(r["outcome"] == "arrived_unconfirmed"
                                                 for r in at),
            "arrived_unconfirmed_away": sum(r["outcome"] == "arrived_unconfirmed"
                                            for r in rows if r["end_target_m"] > AT_TARGET_M),
            "ended_at_target": len(at),
            "sighted_missions": sum(r["sighted"] > 0 for r in rows),
            "outcomes": {o: sum(r["outcome"] == o for r in rows)
                         for o in sorted({r["outcome"] for r in rows})},
            "median_sim_s": statistics.median(r["sim_s"] for r in rows)}


def main(*arms):
    logging.disable(logging.WARNING)
    OUT.mkdir(exist_ok=True)
    for arm in arms or ("F",):
        t, rows = time.time(), []
        for h in SWEEP_HOUSES:
            rows.append(run(arm, h))
            print(json.dumps(rows[-1]), flush=True)
        out = {"arm": arm, **summary(rows), "wall_s": round(time.time() - t), "rows": rows}
        (OUT / f"{arm}.json").write_text(json.dumps(out, indent=1) + "\n")
        print(json.dumps({k: v for k, v in out.items() if k != "rows"}), flush=True)


if __name__ == "__main__":
    main(*sys.argv[1:])
