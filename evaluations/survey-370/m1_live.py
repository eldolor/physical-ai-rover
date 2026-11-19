# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.70 M1, live: a mission on a first visit starts from an empty map under a
new map_id; after a restart and a reload it runs on the saved map's id,
reloaded -- both read from `GET /mission/status`'s `saved_map` through the
brain's HTTP API. The bars are M1's, written before this ran
(docs/plans/ros-alignment/3.70-survey-and-mission-ends.md).

  a  an empty store: a find for an absent target (FIND_S); when the mission
     first names its map, the bridge's /slam/map known cells (criterion 12:
     its first known cells are the robot's own scans) and the robot server's
     /world/map known cells; the bridge saves v1 at its end
  b  robot server, brain and container restarted, the robot on the dock;
     the reload judged; the same find; its `saved_map`

`uptime` every 10 s (outcome-judged bars only on this box). Maps under
EXPLORE_LOGDIR, never committed; numbers to evaluations/survey-370/M1/.

    eval "$(python tools/plan_section.py env)"
    python tools/desktop_lock.py exclusive --why "3.70.M1 live" -- \\
        python evaluations/survey-370/m1_live.py scaled_house
"""
import json
import os
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import s4_live as s4  # noqa: E402 -- its env setup, d3/d5/de and the load sampler

import httpx  # noqa: E402

from sim.maps import build_world  # noqa: E402

d3, d5, de = s4.d3, s4.d5, s4.de
OUT = os.path.join(s4.HERE, "M1")
FIND_S = 120


def known(cells):
    return sum(c != -1 for c in cells)


class FirstNamed(threading.Thread):
    """Polls /mission/status until `saved_map` appears; then the maps."""

    def __init__(self, brain, bridge, robot, house):
        super().__init__(daemon=True)
        self.brain, self.bridge, self.robot, self.house = brain, bridge, robot, house
        self.got, self.stop_ev = None, threading.Event()

    def run(self):
        t0 = time.time()
        while not self.stop_ev.wait(0.25) and time.time() - t0 < 120 and self.got is None:
            try:
                st = self.brain.get("/mission/status").json()
            except Exception:  # noqa: BLE001 -- not up yet
                continue
            if st.get("saved_map"):
                bm = self.bridge.get("/slam/map").json()
                wm = self.robot.get("/world/map").json()
                house = build_world(self.house)
                self.got = {"saved_map": st["saved_map"], "at_s": round(time.time() - t0, 2),
                            "house_m2": round(len(house.layout) * len(house.layout[0]) * 0.09, 1),
                            "bridge_known_m2": round(known(bm.get("data") or [])
                                                     * float(bm.get("resolution_m") or 0) ** 2, 2),
                            "bridge_known": known(bm.get("data") or []),
                            "bridge_cells": len(bm.get("data") or []),
                            "world_known": known(wm.get("cells") or []),
                            "log": [ln for ln in st.get("log_tail") or [] if ln.startswith("map:")]}
                break
        # Amendment 1: the bridge's first non-empty map after the naming, the
        # world's read between two bridge reads.
        t1 = time.time()
        while self.got and not self.stop_ev.is_set() and time.time() - t1 < 30:
            b1 = self.bridge.get("/slam/map").json()
            if known(b1.get("data") or []):
                wm = self.robot.get("/world/map").json()
                b2 = self.bridge.get("/slam/map").json()
                res = float(b1.get("resolution_m") or 0)
                self.got["first_map"] = {
                    "after_s": round(time.time() - t1, 2),
                    "bridge_known_1": known(b1["data"]), "world_known": known(wm.get("cells") or []),
                    "bridge_known_2": known(b2.get("data") or []),
                    "bridge_known_m2": round(known(b1["data"]) * res ** 2, 2)}
                return
            time.sleep(0.2)


def mission(robot, brain, bridge, house):
    first = FirstNamed(brain, bridge, robot, house)
    first.start()
    de.MISSION_LIMIT_S = FIND_S
    m = de.run_mission(robot, house, policy="explore", target=s4.ABSENT, max_steps=100000)
    first.join(40)
    first.stop_ev.set()
    return {k: m.get(k) for k in ("outcome", "seconds", "contact_samples", "min_gap_cm")}, first.got


def house_runs(house, load):
    res = {"house": house, "image": s4.IMAGE, "find_s": FIND_S}
    maps = os.path.join(de.RUN_DIR, f"m1-maps-{house}")
    inv_dir = os.path.join(de.RUN_DIR, f"m1-inventory-{house}")
    os.makedirs(maps, exist_ok=True)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)

    t = time.time()
    robot, brain = d5.servers(house, inv_dir)
    w = d3.container(maps, bridge)
    a = {"reload": w["health"]["slam"]["reload"], "bridge_map_id": w["health"]["map"]["map_id"]}
    a["mission"], a["named"] = mission(robot, brain, bridge, house)
    a["map_save"], a["settled_s"] = s4.settle(bridge)
    a["load"] = load.summary(t)
    res["a"] = a
    print(house, "a", json.dumps({k: a[k] for k in ("bridge_map_id", "named")}), flush=True)

    t = time.time()
    robot, brain = d5.servers(house, inv_dir)
    w = d3.container(maps, bridge)
    b = {"reload": w["health"]["slam"]["reload"], "bridge_map_id": w["health"]["map"]["map_id"]}
    b["mission"], b["named"] = mission(robot, brain, bridge, house)
    b["load"] = load.summary(t)
    res["b"] = b
    print(house, "b", json.dumps({k: b[k] for k in ("reload", "named")}), flush=True)

    na, nb = a.get("named") or {}, b.get("named") or {}
    sa, sb = na.get("saved_map") or {}, nb.get("saved_map") or {}
    saved_id = (a.get("map_save") or {}).get("map_id")
    res["verdicts"] = {
        # With none: an empty store -> a new map_id, not reloaded, its first
        # known cells the bridge's own (the world's no more than the bridge's).
        "first_visit_new_map": ("MET" if sa.get("frame_id") and sa["frame_id"] == a["bridge_map_id"]
                                and sa.get("reloaded") is False else "FAILED"),
        # The brain's map is the bridge's, built from scans: at the first
        # named moment it holds no more known cells than the bridge's (read
        # just before it), and the bridge's are a part of the house.
        "first_cells_own_scans": ("MET" if na.get("first_map")
                                  and na["first_map"]["bridge_known_1"] <= na["first_map"]["world_known"]
                                  <= na["first_map"]["bridge_known_2"]
                                  and na["first_map"]["bridge_known_m2"] < na["house_m2"]
                                  else "FAILED"),
        "saved_map_reloaded": ("MET" if b["reload"].get("result") == "matched"
                               and sb.get("frame_id") == saved_id == sa.get("frame_id")
                               and sb.get("reloaded") is True else "FAILED"),
        "missions_ran": ("MET" if a["mission"].get("outcome") and b["mission"].get("outcome")
                         and not a["mission"].get("contact_samples")
                         and not b["mission"].get("contact_samples") else "FAILED"),
    }
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{house}.json"), "w") as f:
        json.dump(res, f, indent=1)
    return res


def main(argv):
    houses = argv or ["scaled_house"]
    de._kill_ports()
    de._new_run_dir()
    load = s4.Load()
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
