# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/soak-find-3143/ladder.py

3.143 criterion 1, step A: the soak's 40 house/target pairs (3.135.C's
`soak.TARGETS`) in process, each arm one change from the one before, with
`tests/procthor_sweep.py`'s caps (200 steps, 5,000 ticks) and its FakeNav
(a goal succeeds iff the true floor connects it; the robot is then there).

  A0  the recorded sweep: policy explore, goal policy rule, no frames
  A1  policy tiered, tier_search explore, a local tier answering from the
      simulator's ground truth (amendment 2: a VE that is never wrong),
      3.98's geometry at arrival, the $0 stand-in's answer to every cloud
      question
  A2  A1 with explore_goal_policy rooms
  A3  A2 with the frames rendered and the local tier the board's VE (run on
      the board: LOCAL_VLM_URL, LOCAL_VLM_SECRET)
  A3n A2 with the local tier a stub answering no (score 0) to every
      question: the worst VE, on the desktop (amendment 1)

The tiered policy is built by the brain server's own `_tiered_vision_fn`
from `config/robot.yaml`'s brain block, as a mission start builds it.

    python evaluations/soak-find-3143/ladder.py A0 [out.json]
"""

import json
import logging
import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "evaluations/jetson-fmea-3135"))

from control.brain_config import load_brain_config  # noqa: E402
from control import brain_server  # noqa: E402
from control.mission_runner import FOUND, MissionRunner  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from sim.mock_world import MockWorld  # noqa: E402
from soak import HOUSES, TARGETS  # noqa: E402
from tests.test_explore import FakeNav  # noqa: E402

LOCAL_IDENTITY_FN = brain_server._local_identity_fn
MAX_STEPS = 200          # tests/procthor_sweep.py's caps
TICK_CAP = 5000
CELL_M = 0.30
ARMS = ("A0", "A1", "A2", "A3", "A3n")


class StandinCloud:
    """evaluations/tiered-380/E's $0 stand-in: every question "not visible,
    RIGHT"; counted."""

    def __init__(self):
        self.calls = 0

    def __call__(self, frame):
        self.calls += 1
        return {"obstacles_ahead": [], "free_space": "clear", "doorway_visible": False,
                "important_objects": [], "safest_direction": "RIGHT",
                "_navigate": {"reasoning": "stand-in", "target_visible": False}}


class NoLocal:
    """A3n's local tier: `LocalVLMClient`'s surface, no to every question."""

    def __init__(self):
        self.asked = 0

    def ask(self, frame, box=False):
        self.asked += 1
        return {"status": "answered", "score": 0.0}

    __call__ = ask

    def direction(self, frame):
        return {"status": "unsupported", "score": None, "reason": "stub"}

    def close(self):
        pass


class TruthLocal(NoLocal):
    """A1/A2's local tier (amendment 2): yes, with the target's box, when the
    frame's own detections (the simulator's ground truth) hold the target;
    no otherwise."""

    HFOV_DEG = 60.0          # sim/renderer.py's FPV_FOV

    def __init__(self, target):
        super().__init__()
        self.needle = target.strip().lower()

    def ask(self, frame, box=False):
        self.asked += 1
        seen = [d for d in frame.get("detections") or []
                if self.needle in str(d.get("label", "")).lower()
                and isinstance(d.get("bearing_deg"), (int, float))]
        if not seen:
            return {"status": "answered", "score": 0.0}
        near = min(seen, key=lambda d: d.get("distance_m") or float("inf"))
        x = 0.5 + near["bearing_deg"] / self.HFOV_DEG       # bearings are the view's
        x = min(max(x, 0.05), 0.95)
        got = {"status": "answered", "score": 1.0}
        if box:
            got["box_frac"] = [x - 0.05, 0.4, x + 0.05, 0.6]
        return got

    __call__ = ask


class SimRobot(MockRobot):
    """MockRobot whose unrendered frames still carry their provenance
    (`metadata.source` "sim"), as the board's rendered frames do: without
    it 3.98's geometry cannot judge an arrival and the stand-in's no decides
    every one (the first A1 run, 0/40)."""

    def get_camera_frame(self):
        frame = super().get_camera_frame()
        frame.setdefault("metadata", {"source": "sim"})
        return frame


def pairs():
    """The 40 pairs: each house's eight targets, houses in soak.HOUSES order."""
    return [(h, t) for h in HOUSES for t in TARGETS[h]]


def target_m(grid, target):
    cells = [c for c, n in grid.objects.items() if n == target]
    return min(math.dist((grid.x, grid.y), (cx + 0.5, cy + 0.5)) for cx, cy in cells) * CELL_M


def run(arm, house, target):
    grid = build_world(house)
    robot = (MockRobot if arm == "A0" else SimRobot)(grid, render=(arm == "A3"))
    nav = FakeNav(robot)
    kw = dict(target_object=target, navigator=nav, world=MockWorld(grid), max_steps=MAX_STEPS,
              clock=lambda: grid.sim_time, idle=robot.pass_time)
    cloud = confirm = tier = None
    if arm == "A0":
        runner = MissionRunner(robot, policy="explore", **kw)
    else:
        config = load_brain_config()
        config.update(tier_search="explore", sim_arrival_confirm="geometry",
                      tier_local_vlm_url=os.environ.get("LOCAL_VLM_URL", "") if arm == "A3"
                      else "")
        cloud, confirm = StandinCloud(), StandinCloud()
        if arm == "A3n":
            config["tier_local_vlm_url"] = "stub://no"
            brain_server._local_identity_fn = lambda *a, **k: NoLocal()
        elif arm in ("A1", "A2"):
            config["tier_local_vlm_url"] = "stub://truth"
            brain_server._local_identity_fn = lambda tg, *a, **k: TruthLocal(tg)
        else:
            brain_server._local_identity_fn = LOCAL_IDENTITY_FN
        tier = brain_server._tiered_vision_fn(target, cloud, config, simulated=True,
                                 confirm_vision_fn=confirm,
                                 local_secret=os.environ.get("LOCAL_VLM_SECRET") or None)
        runner = MissionRunner(robot, policy="tiered", vision_fn=tier, tier_search="explore",
                               explore_goal_policy="rule" if arm == "A1" else "rooms",
                               vision_timeout_s=config["vision_timeout_s"],
                               arrival_confirm_timeout_s=config["arrival_confirm_timeout_s"],
                               max_vision_failures=config["max_vision_failures"], **kw)
    row = {"arm": arm, "house": house, "target": target}
    try:
        runner.start()
        ticks, closest = 0, target_m(grid, target)
        while runner.tick() and ticks < TICK_CAP:
            ticks += 1
            closest = min(closest, target_m(grid, target))
        s = runner.status()
        row.update(outcome=s["outcome"], ticks=ticks, capped=ticks >= TICK_CAP,
                   steps=s.get("step"), goals=(s.get("explore") or {}).get("goals_sent"),
                   closest_m=round(closest, 2), end_m=round(target_m(grid, target), 2),
                   arrivals=[(v.get("tier"), v.get("confirmed"), v.get("local_score"))
                             for v in s.get("arrival_verdicts") or []],
                   cloud_calls=(cloud.calls + confirm.calls) if cloud else 0,
                   # fix V's overrules: 0 on an always-no arm means unstamped frames
                   no_overruled_sim=((s.get("tier") or {}).get("stats") or {}).get(
                       "local", {}).get("no_overruled_sim", 0),
                   tail=s["log_tail"][-2:])
    except Exception as e:  # noqa: BLE001 -- counted, as procthor_sweep counts them
        row.update(outcome="exception", error=f"{type(e).__name__}: {e}")
    finally:
        close = getattr(tier, "close", None)
        if callable(close):
            close()
    return row


def main(argv):
    arm = argv[0]
    if arm not in ARMS:
        raise SystemExit(f"arm must be one of {ARMS}")
    out = Path(argv[1]) if len(argv) > 1 else HERE / f"ladder-{arm}.json"
    logging.disable(logging.WARNING)
    t, rows = time.time(), []
    for h, tg in pairs():
        rows.append(run(arm, h, tg))
        print(json.dumps({k: rows[-1].get(k) for k in
                          ("house", "target", "outcome", "steps", "goals", "closest_m",
                           "arrivals")}), flush=True)
    found = sum(r["outcome"] == FOUND for r in rows)
    summary = {"arm": arm, "pairs": len(rows), "found": found,
               "misses": [(r["house"], r["target"], r["outcome"]) for r in rows
                          if r["outcome"] != FOUND],
               "wall_s": round(time.time() - t)}
    out.write_text(json.dumps({**summary, "rows": rows}, indent=1) + "\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    main(sys.argv[1:])
