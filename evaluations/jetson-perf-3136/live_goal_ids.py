# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/live_goal_ids.py

3.136.G criterion 5, on the desktop: one explore mission on a fresh stack
(`tests.demo_explore.stack()`, ROS_IMAGE = this section's own tag), with
`ros_pace.py` in the container recording nav2's goal statuses and this
script recording `status.explore.sent` as `board_probe.py missions` does.
Checks that every nav2 goal id is matched to a recorded send, and that the
recorded kinds, in order, equal explore's own `goal_sent` events in the
brain's log. Writes `<out>/summary.json`.

    PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:plan-3.136 \\
      python evaluations/jetson-perf-3136/live_goal_ids.py <out_dir> [house] [limit_s]
"""

import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))

import httpx  # noqa: E402

import scorers  # noqa: E402
from tests import demo_explore as de  # noqa: E402


def main():
    out = Path(sys.argv[1]).resolve()
    house = sys.argv[2] if len(sys.argv) > 2 else "scaled_house"
    limit = float(sys.argv[3]) if len(sys.argv) > 3 else 600
    out.mkdir(parents=True, exist_ok=False)
    de.LOGDIR = str(out)
    robot = de.stack(house)
    run_dir = Path(de.RUN_DIR)
    subprocess.run(["docker", "cp", str(HERE / "ros_pace.py"), f"{de.CONTAINER}:/tmp/ros_pace.py"],
                   check=True)
    subprocess.run(["docker", "exec", "-d", de.CONTAINER, "/entrypoint.sh", "python3",
                    "/tmp/ros_pace.py", "/tmp/pace.jsonl"], check=True)
    time.sleep(3)
    brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
    r = brain.post("/mission/start", json={"target_object": de.TARGET, "policy": "explore",
                                           "mission_time_limit_s": limit})
    r.raise_for_status()
    t0, seen, rows, s = time.time(), 0, [], {}
    while time.time() - t0 < limit + 60:
        s = brain.get("/mission/status").json()
        ex = s.get("explore") or {}
        new = [e for e in ex.get("sent") or [] if e[0] > seen]
        if new:
            rows.append({"t": round(time.time(), 3), "mission": 1, "sent": [e[1:] for e in new],
                         "missing": new[0][0] - seen - 1})
            seen = new[-1][0]
        if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(1)
    time.sleep(2)
    subprocess.run(["docker", "cp", f"{de.CONTAINER}:/tmp/pace.jsonl", str(out / "pace.jsonl")],
                   check=True)
    (out / "explore_sends.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    de._kill_ports()
    pace = scorers.jsonl(out / "pace.jsonl")
    nav_ids = []
    for p in sorted((p for p in pace if p.get("k") == "nav"), key=lambda p: p["t"]):
        for gid, _ in p.get("goals") or []:
            if gid not in nav_ids:
                nav_ids.append(gid)
    sent = [e for r in rows for e in r["sent"]]
    by_id = {gid[:scorers.GOAL_ID_HEX]: k for gid, k in sent if gid}
    logged = [json.loads(m.group(1))["kind"]
              for m in re.finditer(r"explore (\{.*\"event\": \"goal_sent\".*\})",
                                   (run_dir / "explore_brain.log").read_text())]
    summary = {"house": house, "outcome": s.get("outcome"), "goals_sent": seen,
               "nav2_goal_ids": len(nav_ids),
               "matched": sum(g in by_id for g in nav_ids),
               "unmatched_ids": [g for g in nav_ids if g not in by_id],
               "sends_missing": sum(r["missing"] for r in rows),
               "kinds_recorded": [k for _, k in sent], "kinds_logged": logged,
               "kinds_equal": [k for _, k in sent] == logged,
               "amended": scorers.nav_goals_amended(pace, rows)}
    summary["criterion_5"] = (summary["nav2_goal_ids"] > 0 and not summary["unmatched_ids"]
                              and summary["kinds_equal"] and summary["sends_missing"] == 0)
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps({k: v for k, v in summary.items() if not k.startswith("kinds_")}, indent=1))


if __name__ == "__main__":
    main()
