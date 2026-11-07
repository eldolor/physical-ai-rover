# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.167: why 3.60's inventory sweep (seed 3463) reads 93.8% precision of
reported where 3.60 recorded 97.1% -- pinned on the mechanism, not the
number (docs/plans/ros-alignment/3.167-inventory-precision-regression.md).

The drop arrived with 3.74 part B (`fc5c2122`), which put the depth camera's
floor band in series with the cone and the corridor. In `complex_house`
(16 small things on the floor) the band clamps some FORWARD moves: the same
action ends a little shorter, every later pose is shifted, and the
inventory's thresholds (new view, merge, range gate) fall differently.
Nothing in `brain/inventory.py` changed.
"""
import random

from control.mission_runner import MissionRunner
from robot.safety import SafetyController
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests import inventory_sweep as sw
from tests.conftest import mock_world_for

SEED = 3463


def _mission_one(ticks):
    (cx, cy), theta, _ = sw.starts(seed=SEED)[0]
    grid = build_world("complex_house")
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    robot = MockRobot(grid, render=False)
    det = sw.ErringDetector(grid, random.Random(SEED * 100), noisy=True)
    runner = MissionRunner(robot, target_object=sw.ABSENT_TARGET, max_steps=sw.MAX_STEPS,
                           policy="frontier", world=mock_world_for(robot),
                           inventory=True, detections_fn=det)
    out = []
    runner.start()
    while len(out) < ticks and runner.tick():
        out.append((runner.memory.actions[-1].action, grid.x, grid.y))
    return out


def test_the_recorded_bisect_and_seeds_are_pinned():
    """The bisect's two states, and the four seeds: the band moves a seed's
    precision either way (the parent fails 95% on seed 346) and pooled
    precision does not fall."""
    import json
    from pathlib import Path
    rec = Path(__file__).resolve().parents[1] / "evaluations" / "inventory-precision-3167"
    steps = [json.loads(l) for l in (rec / "bisect.jsonl").read_text().splitlines() if l]
    states = {round(s["score"]["precision_reported"], 4) for s in steps}
    assert states == {0.9706, 0.9381}
    assert [s["verdict"] for s in steps if s["commit"].startswith("fc5c2122")] == ["bad"]
    assert [s["verdict"] for s in steps if s["commit"].startswith("90f81fcb")] == ["good"]
    seeds = [json.loads(l) for l in (rec / "seeds.jsonl").read_text().splitlines() if l]
    seeds += [{"commit": s["commit"][:8], "seed": 3463, "score": s["score"]}
              for s in steps if s["commit"][:8] in ("90f81fcb", "fc5c2122")]

    def pooled(c):
        rows = [s["score"] for s in seeds if s["commit"] == c]
        assert len(rows) == 4
        right = sum(round(r["precision_reported"] * r["reported"]) for r in rows)
        return right / sum(r["reported"] for r in rows)
    assert round(pooled("90f81fcb"), 3) == 0.965 and round(pooled("fc5c2122"), 3) == 0.969
    parent_346 = next(s for s in seeds if s["commit"] == "90f81fcb" and s["seed"] == 346)
    assert parent_346["score"]["precision_reported"] < 0.95


def test_the_floor_band_shortens_a_forward_in_the_sweeps_first_mission(monkeypatch):
    """Mission 1 takes the same actions with the band on and off, but a
    FORWARD the band clamps ends shorter, and the poses part from there."""
    on = _mission_one(16)
    monkeypatch.setattr(SafetyController, "floor_clearance",
                        lambda self, *a, **k: (None, "no_floor_band"))
    off = _mission_one(16)
    assert [a for a, _, _ in on] == [a for a, _, _ in off]
    parted = next(i for i, (p, q) in enumerate(zip(on, off)) if p[1:] != q[1:])
    assert on[parted][0] == "FORWARD"
    prev = on[parted - 1][1:]
    moved = lambda p: ((p[0] - prev[0]) ** 2 + (p[1] - prev[1]) ** 2) ** 0.5   # noqa: E731
    assert moved(on[parted][1:]) < moved(off[parted][1:])
