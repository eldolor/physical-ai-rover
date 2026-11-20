# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.194 L5, live: a find started through the brain's HTTP API right after a
reload, before the reloaded map's pose is usable, still goes home to the
saved map's dock. The bar is L5 in docs/plans/ros-alignment/3.194-late-dock.md,
written before this ran.

  s1  map session: fresh servers and container, empty maps; one survey
      (explore, SURVEY_LIMIT_S); its end saves the map
  s2  reload session: servers restarted (the robot on the dock), and the
      find is started (`POST /mission/start`, explore, the return on) while
      the container is still coming up -- the pose not yet usable, 3.70.M2
      k 3's state -- then the container's reload is watched and the find
      runs to its end and its return

Rows: evaluations/dock-late-394/live.jsonl. The maps stay under
EXPLORE_LOGDIR (a sim house's floor plan, never committed).

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host python evaluations/dock-late-394/live.py run
"""
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
ROWS = HERE / "live.jsonl"
HOUSE = "scaled_house"
SURVEY_LIMIT_S = 600
FIND_LIMIT_S = 600
WALL_S = FIND_LIMIT_S + 600
TARGET = "red backpack"


def run():
    assert os.environ.get("ROS_IMAGE", "").endswith("plan-3.194"), "run with this branch's env"
    if ROWS.exists():
        raise SystemExit(f"{ROWS} exists: move it aside first (never overwritten)")
    image = os.environ["ROS_IMAGE"]
    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(ROOT / "evaluations" / "map-350"))
    sys.path.insert(0, str(ROOT / "evaluations" / "survey-370"))
    os.environ["ROS_IMAGE"] = "for-import-plan-3.50"        # d3_runs' own guard
    import httpx
    import d3_runs as d3
    import d5_runs as d5
    from d3_runs import de
    d3.IMAGE = image
    os.environ["ROS_IMAGE"] = image
    import s4_live as s4

    def write(row):
        with open(ROWS, "a") as f:
            f.write(json.dumps(row) + "\n")

    def jget(client, path):
        try:
            return client.get(path).json()
        except (httpx.HTTPError, ValueError):
            return {}

    de._kill_ports()
    de._new_run_dir()
    load = s4.Load()
    load.start()
    maps = os.path.join(de.RUN_DIR, f"maps-{HOUSE}")
    inv_dir = os.path.join(de.RUN_DIR, f"inventory-{HOUSE}")
    os.makedirs(maps, exist_ok=True)
    os.makedirs(inv_dir, exist_ok=True)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
    head = subprocess.run(["git", "rev-parse", "--short=8", "HEAD"], capture_output=True,
                          text=True, cwd=ROOT).stdout.strip()
    try:
        # ---- s1: the survey that saves the map ----
        t = time.time()
        robot, brain = d5.servers(HOUSE, inv_dir)
        d3.container(maps, bridge)
        sv = s4.survey(brain, SURVEY_LIMIT_S)
        saved, settled = s4.settle(bridge)
        write({"row": "survey", "house": HOUSE, "head": head, "image": image,
               "outcome": sv.get("outcome"), "seconds": sv.get("seconds"),
               "map_id": (saved or {}).get("map_id"), "saved": (saved or {}).get("saved"),
               "settled_s": settled, "load": load.summary(t)})
        print("survey", sv.get("outcome"), saved, flush=True)

        # ---- s2: restart; the find starts before the reload is usable ----
        t = time.time()
        robot, brain = d5.servers(HOUSE, inv_dir)
        dock_truth = list(d3.truth(robot))
        watch = {}
        up = threading.Thread(target=lambda: watch.update(d3.container(maps, bridge)), daemon=True)
        up.start()
        start, tries, t_start = None, 0, time.time()
        while start is None and time.time() - t_start < 120:
            tries += 1
            try:
                r = brain.post("/mission/start", json={
                    "target_object": TARGET, "policy": "explore", "max_steps": 100000,
                    "mission_time_limit_s": FIND_LIMIT_S})
            except httpx.HTTPError:
                time.sleep(0.5)
                continue
            if r.status_code == 200:
                start = round(time.time() - t, 2)
            else:
                time.sleep(0.5)
        first = jget(brain, "/mission/status")
        pose_at_start = jget(robot, "/world/pose")
        t0, s, end_truth, end_seen = time.time(), {}, None, None
        while time.time() - t0 < WALL_S:
            s = jget(brain, "/mission/status") or s
            if end_seen is None and (s.get("return") or {}).get("state") not in (None, "returning"):
                end_seen = round(time.time() - t0, 1)
            if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
                break
            time.sleep(1.0)
        else:
            brain.post("/mission/stop")
            time.sleep(5)
            s = jget(brain, "/mission/status") or s
        up.join(timeout=200)
        tr = d3.truth(robot)
        end_truth = [round(v, 3) for v in tr]
        dock = s.get("dock") or {}
        log = s.get("log_tail") or []
        write({"row": "find", "head": head, "image": image, "target": TARGET,
               "start_after_restart_s": start, "start_tries": tries,
               "pose_usable_at_start": bool(pose_at_start.get("usable")),
               "first_status_log": (first.get("log_tail") or [])[:4],
               "reload": ((watch.get("health") or {}).get("slam") or {}).get("reload"),
               "slam_states": watch.get("states"),
               "outcome": s.get("outcome"), "end_reason": (s.get("end_reason") or "")[:200],
               "dock": dock, "return": s.get("return"),
               "late_lines": [x for x in log if "dock" in x][:6],
               "dock_truth": [round(v, 3) for v in dock_truth], "end_truth": end_truth,
               "end_off_dock_m": round(math.dist(end_truth[:2], dock_truth[:2]), 3),
               "end_off_dock_deg": round(abs((end_truth[2] - dock_truth[2] + 180) % 360 - 180), 1),
               "seconds": round(time.time() - t0), "load": load.summary(t)})
        print("find", json.dumps({"outcome": s.get("outcome"), "dock": dock,
                                  "return": s.get("return")}), flush=True)
    finally:
        if subprocess.run(["docker", "inspect", de.CONTAINER], capture_output=True).returncode == 0:
            with open(f"{de.RUN_DIR}/ros-end.log", "w") as f:
                subprocess.run(["docker", "logs", de.CONTAINER], stdout=f, stderr=subprocess.STDOUT)
        subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
        de._kill_ports()
        load.stop_ev.set()
        print("load", json.dumps(load.summary()), flush=True)


if __name__ == "__main__":
    if sys.argv[1:2] == ["run"]:
        run()
    else:
        raise SystemExit(__doc__)
