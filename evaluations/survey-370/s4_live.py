# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.70 S4, live: a survey's map, room labels and inventory saved under one
`map_id`, returned equal after a container restart and a reload, and none of
them used after a fall-back. The bars are S4's, written before this ran
(docs/plans/ros-alignment/3.70-survey-and-mission-ends.md).

Per house, on one host map directory (/maps) and one inventory directory:

  s1  an empty store: a survey (job survey, explore, its own time limit); the
      bridge saves v1 at its end, the brain the survey's report with its
      `room_labels`. Recorded: the bridge's map_id and saved version, the
      report's frame_id, labels and landmarks, and the last /slam/map before
      the save (criterion 10's reference)
  s2  robot server, brain and container restarted, the robot on the dock;
      the reload judged; the reloaded /map against s1's (criterion 10's
      99%); the store read on the reloaded pose's saved map BEFORE any
      mission (labels cell for cell, landmarks by id, position, belief,
      votes); then a find for an absent target, its seeded inventory read
      at once (the labels block whole; the landmarks as 3.50.D5 carries them)
  fb  robot server and brain restarted, the robot driven >= 1 m and >= 30 deg
      off the dock with the container up, then only the container restarted:
      the reload must fall back (a new map_id) and a 60 s find on it carries
      no label and no landmark

`uptime` every 10 s throughout (this cloud box: outcome-judged bars only).
The maps and inventories are a SIM house's, under EXPLORE_LOGDIR, never
committed; numbers only go to evaluations/survey-370/S4/.

    eval "$(python tools/plan_section.py env)"
    python tools/desktop_lock.py exclusive --why "3.70.S4 live" -- \\
        python evaluations/survey-370/s4_live.py scaled_house
"""
import json
import math
import os
import subprocess
import sys
import threading
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "evaluations", "map-350"))

IMAGE = os.environ.get("ROS_IMAGE", "")
assert IMAGE.endswith("plan-3.70"), "run with this branch's env"
os.environ["ROS_IMAGE"] = "for-import-plan-3.50"     # d3_runs' own guard
import httpx  # noqa: E402

import d3_runs as d3  # noqa: E402
import d5_runs as d5  # noqa: E402
from d3_runs import de  # noqa: E402

from brain.room_labels import read_saved  # noqa: E402
from control.inventory_store import InventoryStore  # noqa: E402

d3.IMAGE = IMAGE
os.environ["ROS_IMAGE"] = IMAGE
OUT = os.path.join(HERE, "S4")
SURVEY_S = {"scaled_house": 900, "home_first_floor": 1800}
FIND_S = 60
ABSENT = d3.ABSENT


class Load(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.rows, self.stop_ev = [], threading.Event()

    def run(self):
        while True:
            with open("/proc/loadavg") as f:
                self.rows.append((round(time.time()), float(f.read().split()[0])))
            if self.stop_ev.wait(10):
                return

    def summary(self, since=None):
        xs = [v for t, v in self.rows if since is None or t >= since]
        if not xs:
            return None
        s = sorted(xs)
        return {"n": len(xs), "max": s[-1], "p50": s[len(s) // 2],
                "over_4": sum(v > 4 for v in xs)}


def settle(bridge, timeout=180):
    """The mission end's save (or skip) landed and no re-map runs."""
    t0 = time.time()
    h = None
    while time.time() - t0 < timeout:
        h = bridge.get("/health").json()
        m = h["map"]
        if (m["saved"] + m.get("skipped", 0) + m.get("failed", 0) >= 1 and not m.get("saving")
                and h["slam"]["state"] != "remapping"):
            break
        time.sleep(0.5)
    time.sleep(3)                       # the brain's inventory save thread
    return h["map"] if h else None, round(time.time() - t0, 1)


def survey(brain, limit_s):
    r = brain.post("/mission/start", json={"job": "survey", "policy": "explore",
                                           "max_steps": 100000, "mission_time_limit_s": limit_s})
    if r.status_code != 200:
        return {"start": r.status_code, "detail": r.text[:300]}
    t0, st = time.time(), {}
    while time.time() - t0 < limit_s + 120:
        st = brain.get("/mission/status").json()
        if not st.get("running") and st.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(1.0)
    else:
        brain.post("/mission/stop")
    return {"outcome": st.get("outcome"), "seconds": round(time.time() - t0),
            "survey": st.get("survey"), "end": (st.get("log_tail") or [""])[-1]}


def rows_key(report):
    return sorted((r["id"], r["label"], r["x_m"], r["y_m"], r["belief"],
                   json.dumps(r.get("votes"), sort_keys=True))
                  for r in (report.get("reported") or []) + (report.get("candidates") or []))


def find(brain, robot, house, limit_s):
    seed = d5.SeedWatch(brain)
    seed.start()
    de.MISSION_LIMIT_S = limit_s
    m = de.run_mission(robot, house, policy="explore", target=ABSENT, max_steps=100000)
    seed.stop_ev.set()
    time.sleep(3)
    inv = brain.get("/mission/inventory").json().get("inventory") or {}
    return m, seed.snap, inv


def house_runs(house, load):
    res = {"house": house, "image": IMAGE, "survey_limit_s": SURVEY_S[house]}
    maps = os.path.join(de.RUN_DIR, f"maps-{house}")
    inv_dir = os.path.join(de.RUN_DIR, f"inventory-{house}")
    os.makedirs(maps, exist_ok=True)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)

    # ---- s1: the survey ----
    t = time.time()
    robot, brain = d5.servers(house, inv_dir)
    d3.container(maps, bridge)
    s1 = {"survey": survey(brain, SURVEY_S[house])}
    last_map = bridge.get("/slam/map").json()          # before the end's save lands
    s1["map_save"], s1["settled_s"] = settle(bridge)
    map_id = s1["map_save"]["map_id"] if s1["map_save"] else None
    s1["map_id"], s1["saved_version"] = map_id, (s1["map_save"] or {}).get("last_version")
    written = InventoryStore(inv_dir).latest_for(map_id) if map_id else None
    block = read_saved((written or {}).get("room_labels"))
    s1["report"] = None if written is None else {
        "frame_id": written.get("frame_id"), "mission_id": written.get("mission_id"),
        "landmarks": len(rows_key(written)),
        "labels": None if block is None else {
            "names": block["names"], "grid": [block["width"], block["height"], block["resolution_m"]],
            "labelled_cells": sum(c >= 0 for c in block["cells"]), "seeds": len(block["seeds"]),
            "frame_id": block.get("frame_id"), "mission_id": block.get("mission_id")}}
    s1["load"] = load.summary(t)
    res["s1"] = s1
    _dump(house, res)
    print(house, "s1", json.dumps({k: s1[k] for k in ("map_id", "saved_version", "report")}),
          flush=True)
    s1_ok = bool(written and block and map_id and s1["saved_version"]
                 and written.get("frame_id") == map_id and block.get("frame_id") == map_id)

    # ---- s2: restart, reload, the store read, a find ----
    t = time.time()
    robot, brain = d5.servers(house, inv_dir)
    w = d3.container(maps, bridge)
    rel = w["health"]["slam"]["reload"]
    s2 = {"reload": rel, "map_id": w["health"]["map"]["map_id"]}
    m = d3.first_map(bridge)
    s2["map_agreement"] = d3.agreement(last_map, m) if (m and last_map.get("data")) else None
    pose = bridge.get("/slam/pose").json()
    s2["pose_map_id"] = pose.get("map_id")
    got = InventoryStore(inv_dir).latest_for(pose.get("map_id")) if pose.get("map_id") else None
    gb = read_saved((got or {}).get("room_labels"))
    s2["store"] = {
        "same_report": got is not None and got.get("mission_id") == (written or {}).get("mission_id"),
        "labels_cell_for_cell": bool(gb and block and gb["cells"] == block["cells"]
                                     and gb["names"] == block["names"]),
        "labels_block_equal": bool(got and written and got.get("room_labels") == written.get("room_labels")),
        "landmarks_equal": bool(got and written and rows_key(got) == rows_key(written)),
        "landmarks": None if got is None else len(rows_key(got))}
    mission, snap, inv = find(brain, robot, house, FIND_S)
    s2["find"] = {k: mission.get(k) for k in ("outcome", "seconds", "contact_samples", "min_gap_cm")}
    snap = snap or {}
    saved_rows = [r for r in (written or {}).get("reported", []) + (written or {}).get("candidates", [])
                  if r["belief"] >= 0.5]
    carried = [r for r in snap.get("reported", []) + snap.get("candidates", []) if r.get("carried")]
    match = []
    for r in saved_rows:
        best = min(carried, key=lambda c: math.hypot(c["x_m"] - r["x_m"], c["y_m"] - r["y_m"])
                   if c["label"] == r["label"] else 1e9, default=None)
        if best is not None and best["label"] == r["label"]:
            match.append({"id_saved": r["id"], "id_seeded": best["id"], "label": r["label"],
                          "d_m": round(math.hypot(best["x_m"] - r["x_m"], best["y_m"] - r["y_m"]), 4),
                          "d_belief": round(abs(best["belief"] - r["belief"]), 4)})
    s2["seeded"] = {"frame_id": snap.get("frame_id"), "carried_from": snap.get("carried_from"),
                    "at_s": snap.get("at_s"),
                    "labels_block_equal": bool(written and snap.get("room_labels") == written.get("room_labels")),
                    "carried": len(carried), "saved_at_belief_0_5": len(saved_rows),
                    "matched": match}
    s2["end_inventory_labels_equal"] = bool(written and inv.get("room_labels") == written.get("room_labels"))
    s2["load"] = load.summary(t)
    res["s2"] = s2
    _dump(house, res)
    print(house, "s2", json.dumps({k: s2[k] for k in ("reload", "map_agreement", "store")}), flush=True)

    # ---- fb: off the dock, the container restarted only ----
    t = time.time()
    robot, brain = d5.servers(house, inv_dir)
    dock = d3.truth(robot)
    d3.container(maps, bridge)
    for _ in range(4):
        x, y, h = d3.truth(robot)
        if math.hypot(x - dock[0], y - dock[1]) >= 1.0 and abs((h - dock[2] + 180) % 360 - 180) >= 30:
            break
        d3.drive(robot, 1.0, 60)
    x, y, h = d3.truth(robot)
    fb = {"from_dock_m": round(math.hypot(x - dock[0], y - dock[1]), 2),
          "from_dock_deg": round(abs((h - dock[2] + 180) % 360 - 180), 1)}
    w = d3.container(maps, bridge)
    fb["reload"], fb["map_id"] = w["health"]["slam"]["reload"], w["health"]["map"]["map_id"]
    mission, snap, inv = find(brain, robot, house, FIND_S)
    fb["find"] = {k: mission.get(k) for k in ("outcome", "seconds")}
    fb["labels_used"] = bool(inv.get("room_labels")) or bool((snap or {}).get("room_labels"))
    fb["landmarks_used"] = sum(1 for r in inv.get("reported", []) + inv.get("candidates", [])
                               if r.get("carried"))
    fb["frame_id"] = inv.get("frame_id")
    fb["load"] = load.summary(t)
    res["fall_back"] = fb

    res["verdicts"] = {
        "written_as_one": "MET" if s1_ok else "FAILED",
        "map_round_trip": ("MET" if rel.get("result") == "matched" and s2["map_id"] == map_id
                           and (s2["map_agreement"] or 0) >= 0.99 else "FAILED"),
        "labels_cell_for_cell": "MET" if s2["store"]["labels_cell_for_cell"] else "FAILED",
        "landmarks_equal": "MET" if s2["store"]["landmarks_equal"] else "FAILED",
        "fall_back_none_used": ("MET" if fb["reload"].get("result") == "fell_back"
                                and fb["map_id"] != map_id and not fb["labels_used"]
                                and fb["landmarks_used"] == 0
                                and fb["from_dock_m"] >= 1.0 and fb["from_dock_deg"] >= 30
                                else "FAILED"),
    }
    _dump(house, res)
    return res


def _dump(house, res):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{house}.json"), "w") as f:
        json.dump(res, f, indent=1)


def main(argv):
    houses = argv or ["scaled_house"]
    de._kill_ports()
    de._new_run_dir()
    load = Load()
    load.start()
    try:
        for house in houses:
            res = house_runs(house, load)
            print(house, json.dumps(res["verdicts"]), flush=True)
    finally:
        subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
        de._kill_ports()
        load.stop_ev.set()
        print("load", json.dumps(load.summary()), flush=True)


if __name__ == "__main__":
    main(sys.argv[1:])
