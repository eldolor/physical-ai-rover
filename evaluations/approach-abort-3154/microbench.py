# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/approach-abort-3154/microbench.py

3.154 criteria 1, 3 and 4 (`docs/plans/ros-alignment/3.154-approach-abort-
on-cancel.md`), on the desktop. A live robot server (sim, `scaled_house`)
and the ROS container come up through `tests/demo_explore.py` `stack()`,
whose image is `ROS_IMAGE`. Goals go through the robot server's
`/world/goal` as the brain's do (`x-driver: brain`).

* `switch` (criteria 1 and 3): goal A to a reachable point 1.5-2.5 m away,
  waited until nav2 runs it, then goal B the way the arm says --
  `delete-then-post` (explore on `dev`: `DELETE /world/goal` then `POST`) or
  `post` (explore with the fix: `POST` only). B's state is read for 3 s.
* `stop` (criterion 4): goal A, waited until it runs, then `DELETE` alone;
  canceled within 2 s?

    PICAR_ROS_NETWORK=host ROS_IMAGE=... python evaluations/approach-abort-3154/microbench.py \\
        switch delete-then-post 30 OUT.json
"""

import json
import math
import os
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tests import demo_explore as de  # noqa: E402
from sim.maps import build_world  # noqa: E402

H = {"x-driver": "brain"}
CELL = 0.3


def goal(robot):
    return (robot.get("/world/goal", headers=H).json() or {}).get("goal") or {}


def wait_state(robot, gid, states, timeout_s):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        g = goal(robot)
        if g.get("goal_id") == gid and g.get("state") in states:
            return g, time.monotonic()
        time.sleep(0.05)
    return goal(robot), None


def post(robot, p):
    r = robot.post("/world/goal", json={"x_m": p[0], "y_m": p[1]}, headers=H)
    body = r.json() if r.headers.get("content-type", "").startswith("application/json") else {}
    return r.status_code, body.get("goal_id") or (body.get("goal") or {}).get("goal_id")


def pick(robot, reach, rng, near_m=1.5, far_m=2.5):
    t = robot.get("/world/truth").json()
    x, y = t["x_m"], t["y_m"]
    cands = [((cx + 0.5) * CELL, (cy + 0.5) * CELL) for cx, cy in reach
             if near_m <= math.hypot((cx + 0.5) * CELL - x, (cy + 0.5) * CELL - y) <= far_m]
    return rng.choice(cands) if cands else None


def main():
    mode, arm, n, out = sys.argv[1], sys.argv[2], int(sys.argv[3]), Path(sys.argv[4])
    rng = random.Random(3154)
    world = build_world("scaled_house")
    reach = sorted(de.reachable_cells(world))
    rows = []
    robot = de.stack("scaled_house")
    try:
        for i in range(n):
            row = {"trial": i, "mode": mode, "arm": arm}
            a = pick(robot, reach, rng)
            code, aid = post(robot, a)
            ga, t_run = wait_state(robot, aid, ("active",), 8.0)
            row.update(a=a, a_code=code, a_running=t_run is not None)
            if t_run is None:
                row["skipped"] = f"A never ran: {ga}"
            else:
                time.sleep(0.5)
                if mode == "switch":
                    b = pick(robot, reach, rng)
                    t0 = time.monotonic()
                    if arm == "delete-then-post":
                        robot.delete("/world/goal", headers=H)
                    code, bid = post(robot, b)
                    row.update(b=b, b_code=code)
                    seen, ended, t_end, states = set(), None, None, []
                    end = time.monotonic() + 3.0
                    while time.monotonic() < end:
                        g = goal(robot)
                        if g.get("goal_id") == bid:
                            st = g.get("state")
                            if st not in seen:
                                seen.add(st)
                                states.append([st, round(time.monotonic() - t0, 3)])
                            if st in ("aborted", "canceled", "rejected", "succeeded"):
                                ended, t_end = st, time.monotonic()
                                break
                        time.sleep(0.02)
                    row.update(b_states=states, b_ended=ended,
                               b_ended_s=round(t_end - t0, 3) if t_end else None,
                               b_ran=any(s == "active" for s, _ in states))
                else:
                    t0 = time.monotonic()
                    robot.delete("/world/goal", headers=H)
                    g, t_c = wait_state(robot, aid, ("canceled", "aborted", "succeeded"), 2.0)
                    row.update(stop_state=g.get("state"),
                               stop_s=round(t_c - t0, 3) if t_c else None)
            # Clear before the next trial: whatever is live is cancelled and ends.
            robot.delete("/world/goal", headers=H)
            end = time.monotonic() + 6
            while time.monotonic() < end and goal(robot).get("state") in ("pending", "active"):
                time.sleep(0.1)
            row["cleared"] = goal(robot).get("state")
            rows.append(row)
            print(json.dumps(row), flush=True)
            time.sleep(1.0)
    finally:
        de._kill_ports()
        os.system(f"docker rm -f {de.CONTAINER} >/dev/null 2>&1")
    switched = [r for r in rows if r.get("mode") == "switch" and "skipped" not in r]
    summary = {"mode": mode, "arm": arm, "image": os.environ.get("ROS_IMAGE", "vision-picar-ros:latest"),
               "trials": len(rows), "valid": len(switched) if mode == "switch" else
               sum(1 for r in rows if "skipped" not in r)}
    if mode == "switch":
        summary.update(b_aborted=sum(1 for r in switched if r.get("b_ended") == "aborted"),
                       b_ran=sum(1 for r in switched if r.get("b_ran")),
                       b_aborted_after_s=[r["b_ended_s"] for r in switched if r.get("b_ended") == "aborted"])
    else:
        ok = [r for r in rows if "skipped" not in r]
        summary.update(canceled_within_2s=sum(1 for r in ok if r.get("stop_state") == "canceled"))
    out.write_text(json.dumps({"summary": summary, "rows": rows}, indent=1) + "\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
