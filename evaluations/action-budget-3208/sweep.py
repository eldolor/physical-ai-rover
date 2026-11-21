# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/action-budget-3208/sweep.py

3.208 criterion 3-4: the frontier backpack hunt through the real path
(MissionRunner -> agent -> robot/safety.py -> RemoteRobot -> an in-process
robot/server.py -> MockRobot) in three houses; a no-fault arm (one mission a
house, deterministic) and a fault arm (SEEDS seeds a house) in which each
POST /action is lost with probability DROP, half before the robot sees it
and half after it executed (the answer lost); and the same seeds under a
budget of 1, which is dev's behaviour (the first lost POST ends it). Counts, per decided action,
the move POSTs that reached the robot server.

    python evaluations/action-budget-3208/sweep.py [out.json]
"""

import json
import os
import random
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from tests.test_action_budget_3208 import (  # noqa: E402
    MOVES, count_decisions, drive, moves_reached_per_decision, runner_for, stack,
)

HOUSES = ("starter_house", "scaled_house", "home_first_floor")
SEEDS = range(1, 11)
DROP = 0.05


class _Env:
    def setenv(self, k, v):
        os.environ[k] = v


def one(house, seed, budget=None):
    rng = random.Random(seed) if seed is not None else None

    def plan(i, body):
        if rng is not None and rng.random() < DROP:
            return rng.choice(["before", "after"])
        return None
    t = time.time()
    link, robot, world = stack(plan, house=house, monkeypatch=_Env())
    runner = runner_for(robot, world, **({"max_action_failures": budget} if budget else {}))
    count_decisions(runner, link)
    runner.start()
    seen = []
    status = drive(runner, seen=seen)
    per = moves_reached_per_decision(link)
    counted = sum(1 for line in status["log_tail"] if "robot unreachable" in line)
    return {
        "house": house, "seed": seed, "outcome": status["outcome"],
        "error": status["error"], "steps": status.get("step"),
        "lost": len(link.failed),
        "lost_moves": sum(1 for f in link.failed if f[1] in MOVES),
        "lost_before": sum(1 for f in link.failed if f[2] == "before"),
        "lost_after": sum(1 for f in link.failed if f[2] == "after"),
        "max_action_failures_seen": max((s["action_failures"] for s in seen), default=0),
        "decisions_with_moves": len(per),
        "decisions_sent_twice": sum(1 for n in per.values() if n > 1),
        "step_failed": bool(status["error"] and status["error"].startswith("step failed")),
        "wall_s": round(time.time() - t, 1),
    }


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    rows = []
    for house in HOUSES:
        rows.append({"arm": "no_fault", **one(house, None)})
        print(rows[-1], flush=True)
        for seed in SEEDS:
            rows.append({"arm": "fault", **one(house, seed)})
            print(rows[-1], flush=True)
            # dev's behaviour (the first lost POST ends the mission), the
            # same seeds: a budget of 1.
            rows.append({"arm": "fault_budget1", **one(house, seed, budget=1)})
            print(rows[-1], flush=True)
    summary = {}
    for arm in ("no_fault", "fault", "fault_budget1"):
        r = [x for x in rows if x["arm"] == arm]
        summary[arm] = {
            "missions": len(r),
            "found": sum(x["outcome"] == "found" for x in r),
            "outcomes": {o: sum(x["outcome"] == o for x in r) for o in {x["outcome"] for x in r}},
            "lost_posts": sum(x["lost"] for x in r),
            "step_failed": sum(x["step_failed"] for x in r),
            "decisions_sent_twice": sum(x["decisions_sent_twice"] for x in r),
        }
    result = {"drop": DROP, "houses": HOUSES, "seeds": list(SEEDS), "summary": summary, "rows": rows}
    print(json.dumps(summary, indent=1))
    if out:
        out.write_text(json.dumps(result, indent=1) + "\n")


if __name__ == "__main__":
    main()
