# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/action-budget-3208/sweep.py

3.208 criteria 3 and 4' (amendment 1): the frontier backpack hunt through
the real path (MissionRunner -> agent -> robot/safety.py -> RemoteRobot and
RemoteWorld -> an in-process robot/server.py -> MockRobot) in three houses,
on the runner's simulated clock (tests/test_action_budget_3208.FakeClock):

* no_fault: one mission a house (deterministic);
* k7_all: 5% of ALL robot requests lost (K7's model), SEEDS seeds a house;
* k7_action: 5% of POST /action lost (the FAILED run's model), same seeds;
* *_budget1: the same two with a link budget of one failed tick
  (`link_max_failed_ticks=1`): dev's behaviour, approximately (on dev the
  first lost request ended the mission unless it was a frame read or a scan
  inside the vision call, which spent the camera's or vision's budget).

Half the losses are before the robot sees the request, half after it acted.
Counts, per decided action, the move POSTs that reached the robot server.

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
OTHER_BUDGET_ENDS = ("step failed", "camera unavailable", "vision unavailable")


class _Env:
    def setenv(self, k, v):
        os.environ[k] = v


def one(house, seed, model, budget1=False):
    rng = random.Random(seed)

    def plan(i, method, path, body):
        if model is None or (model == "action" and path != "/action"):
            return None
        if rng.random() < DROP:
            return rng.choice(["before", "after"])
        return None
    t = time.time()
    link, robot, world = stack(plan, house=house, monkeypatch=_Env())
    runner = runner_for(robot, world, **({"link_max_failed_ticks": 1} if budget1 else {}))
    count_decisions(runner, link)
    runner.start()
    seen = []
    status = drive(runner, seen=seen)
    per = moves_reached_per_decision(link)
    err = status["error"] or ""
    return {
        "house": house, "seed": seed, "outcome": status["outcome"], "error": err or None,
        "steps": status.get("step"), "lost": len(link.failed),
        "lost_actions": sum(1 for f in link.failed if f[2] == "/action"),
        "lost_moves": sum(1 for f in link.failed if f[3] in MOVES),
        "lost_before": sum(1 for f in link.failed if f[4] == "before"),
        "lost_after": sum(1 for f in link.failed if f[4] == "after"),
        "max_link_failures": max((s["link_failures"] for s in seen), default=0),
        "max_camera_failures": max((s["camera_failures"] for s in seen), default=0),
        "max_vision_failures": max((s["vision_failures"] for s in seen), default=0),
        "decisions_with_moves": len(per),
        "decisions_sent_twice": sum(1 for n in per.values() if n > 1),
        "ended_outside_link_budget": err.startswith(OTHER_BUDGET_ENDS),
        "wall_s": round(time.time() - t, 1),
    }


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    rows = []
    for house in HOUSES:
        rows.append({"arm": "no_fault", **one(house, None, None)})
        print(rows[-1], flush=True)
        for seed in SEEDS:
            for arm, model, b1 in (("k7_all", "all", False), ("k7_action", "action", False),
                                   ("k7_all_budget1", "all", True),
                                   ("k7_action_budget1", "action", True)):
                rows.append({"arm": arm, **one(house, seed, model, b1)})
                print(rows[-1], flush=True)
    summary = {}
    for arm in ("no_fault", "k7_all", "k7_action", "k7_all_budget1", "k7_action_budget1"):
        r = [x for x in rows if x["arm"] == arm]
        summary[arm] = {
            "missions": len(r),
            "found": sum(x["outcome"] == "found" for x in r),
            "outcomes": {o: sum(x["outcome"] == o for x in r) for o in sorted({x["outcome"] for x in r})},
            "lost_requests": sum(x["lost"] for x in r),
            "ended_outside_link_budget": sum(x["ended_outside_link_budget"] for x in r),
            "decisions_sent_twice": sum(x["decisions_sent_twice"] for x in r),
        }
    result = {"drop": DROP, "houses": HOUSES, "seeds": list(SEEDS), "summary": summary, "rows": rows}
    print(json.dumps(summary, indent=1))
    if out:
        out.write_text(json.dumps(result, indent=1) + "\n")


if __name__ == "__main__":
    main()
