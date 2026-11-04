# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/soak-find-3143/v2_measure.py

3.143 criterion 3, bar V2: real frames never take fix V's geometry. Whole
tiered missions in process (`policy: tiered`, `tier_search: explore`,
`rooms`, `sim_arrival_confirm: geometry`, as 3.143's ladder A2 builds them
through the brain server's own `_tiered_vision_fn`) on frames that carry no
`metadata.source` -- what a real camera's frames look like to the brain --
with a local tier that says yes, no and unable from a hash of the frame:
the box meets a local no at arrival, the basketball a yes (the frame-question
path is not shown reached here; the differential test covers it). The measurement (2026-11-04): this file run on `dev`
before fix V (`3f0c3adc`, the merge base) wrote `v2-dev-3f0c3adc.json`, and run
on fix V's branch wrote `v2-fixv.json`, byte for byte the same:

    PYTHONPATH=<checkout> python evaluations/soak-find-3143/v2_measure.py <out.json>

The suite's own pin is differential (`tests/test_sim_local_no_real.py`).
"""

import hashlib
import json
import logging
import sys

# A local no at a real frame's arrival (the box) and a yes (the basketball).
PAIRS = (("procthor:val:0", "box"), ("procthor:val:2", "basketball"))
MAX_STEPS = 60
TICK_CAP = 1500


class HashLocal:
    """`LocalVLMClient`'s surface. Its answer is a function of the frame
    alone: unable 1 in 8; else, with the target among the frame's
    detections, yes 2 in 8, otherwise yes 1 in 8; no for the rest."""

    def __init__(self, target):
        self.needle = target.lower()

    def ask(self, frame, box=False):
        h = int(hashlib.sha256(json.dumps(frame, sort_keys=True, default=str).encode())
                .hexdigest()[:8], 16) % 8
        if h == 0:
            return {"status": "unable", "score": None, "reason": "stub"}
        seen = any(self.needle in str(d.get("label", "")).lower()
                   for d in frame.get("detections") or [])
        yes = h <= 2 if seen else h == 1
        got = {"status": "answered", "score": 1.0 if yes else 0.0}
        if box and yes:
            got["box_frac"] = [0.45, 0.4, 0.55, 0.6]
        return got

    __call__ = ask

    def direction(self, frame):
        return {"status": "unsupported", "score": None, "reason": "stub"}

    def close(self):
        pass


def _standin(frame):
    return {"obstacles_ahead": [], "free_space": "clear", "doorway_visible": False,
            "important_objects": [], "safest_direction": "RIGHT",
            "_navigate": {"reasoning": "stand-in", "target_visible": False}}


def run(house, target):
    from control import brain_server
    from control.brain_config import load_brain_config
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld
    from tests.test_explore import FakeNav

    grid = build_world(house)
    robot = MockRobot(grid, render=False)   # unrendered: no metadata.source, a real frame to the brain
    config = load_brain_config()
    config.update(tier_search="explore", sim_arrival_confirm="geometry",
                  tier_local_vlm_url="stub://hash")
    saved = brain_server._local_identity_fn
    brain_server._local_identity_fn = lambda tg, *a, **k: HashLocal(tg)
    try:
        tier = brain_server._tiered_vision_fn(target, _standin, config, simulated=True,
                                              confirm_vision_fn=_standin)
    finally:
        brain_server._local_identity_fn = saved
    runner = MissionRunner(robot, policy="tiered", vision_fn=tier, tier_search="explore",
                           explore_goal_policy="rooms", target_object=target,
                           navigator=FakeNav(robot), world=MockWorld(grid), max_steps=MAX_STEPS,
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           vision_timeout_s=config["vision_timeout_s"],
                           arrival_confirm_timeout_s=config["arrival_confirm_timeout_s"],
                           max_vision_failures=config["max_vision_failures"])
    try:
        runner.start()
        ticks = 0
        while runner.tick() and ticks < TICK_CAP:
            ticks += 1
        s = runner.status()
        local = ((s.get("tier") or {}).get("stats") or {}).get("local") or {}
        return {"house": house, "target": target, "outcome": s["outcome"], "step": s.get("step"),
                "ticks": ticks,
                "arrivals": [[v.get("tier"), v.get("confirmed"), v.get("local_score")]
                             for v in s.get("arrival_verdicts") or []],
                "local": {k: v for k, v in sorted(local.items())
                          if isinstance(v, int) and not k.endswith("_ms")}}
    finally:
        close = getattr(tier, "close", None)
        if callable(close):
            close()


def run_all():
    logging.disable(logging.WARNING)
    try:
        return [run(h, t) for h, t in PAIRS]
    finally:
        logging.disable(logging.NOTSET)


if __name__ == "__main__":
    rows = run_all()
    with open(sys.argv[1], "w") as f:
        json.dump(rows, f, indent=1)
        f.write("\n")
    print(json.dumps(rows))
