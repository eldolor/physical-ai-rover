# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3161/live.py -- 3.161.B's live run on the ROS stack
(nav2 + slam_toolbox + the bridge built from this branch), scaled_house,
judged on the simulator's ground truth.

1. A fresh stack (`tests/demo_explore.stack`, PICAR_ROS_NETWORK=host, the
   container with `-v picar-maps:/maps`), the robot on its dock.
2. Three zones (`sweep.place_zones(house, 0)`), drawn through the brain's
   own route (`POST /mission/nogo`), which stores them and sends them to the
   robot server and on to the bridge; the bridge's /health must hold 3.
3. Z6 live: ten of a person's goal taps inside the zones (`x-driver:
   twin-dpad`), each refused `no_go_zone`; one outside, taken.
4. A find for the red backpack through the brain's HTTP API; the truth read
   every 0.5 s: Z2's entries (the true centre inside a zone), Z1's goals
   (`status.explore.sent` against the zones), and `uptime` every 10 s.
5. Z4 live: after the mission's end (its keepout clear) the bridge still
   holds 3 zones; the brain's store, re-read, still holds 3.
6. Z3 live, once: a zone drawn around the robot where it stopped, and a
   survey started; does it leave, and is the zone drawn again (the
   bridge's zone_restored, clear_m)?

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:plan-3.161 \\
        python evaluations/nogo-3161/live.py OUT.json
"""

import json
import math
import os
import subprocess
import sys
import threading
import time

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(1, os.path.dirname(__file__))

import httpx                                             # noqa: E402

from tests import demo_explore as de                     # noqa: E402
import sweep                                             # noqa: E402

HOUSE = "scaled_house"
VOLUME = os.environ.get("PICAR_MAPS_VOLUME", "picar-maps")
LIMIT_S = 900


def _with_volume():
    real = subprocess.run

    def run(args, *a, **k):
        if isinstance(args, list) and args[:3] == ["docker", "run", "-d"]:
            args = args[:3] + ["-v", f"{VOLUME}:/maps"] + args[3:]
        return real(args, *a, **k)
    de.subprocess.run = run


def _uptime(samples, stop):
    while not stop.is_set():
        samples.append([round(time.time(), 1), open("/proc/loadavg").read().split()[0]])
        stop.wait(10)


def _inside(zones, x, y):
    return [z["id"] for z in zones if z["x0"] <= x <= z["x1"] and z["y0"] <= y <= z["y1"]]


def main(out):
    load, stop = [], threading.Event()
    threading.Thread(target=_uptime, args=(load, stop), daemon=True).start()
    _with_volume()
    result = {"house": HOUSE, "image": os.environ.get("ROS_IMAGE")}
    robot = de.stack(HOUSE)
    brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=10)
    try:
        for _ in range(60):
            p = robot.get("/world/pose").json()
            if p.get("usable") and p.get("frame_id"):
                break
            time.sleep(2)
        result["frame_id"] = p.get("frame_id")
        zones = sweep.place_zones(HOUSE, 0)
        v = brain.get("/mission/nogo").json()
        for z in zones:
            v = brain.post("/mission/nogo", json={"frame_id": v["frame_id"], "rev": v["rev"],
                                                  "op": "add", "zone": {k: z[k] for k in
                                                                        ("x0", "y0", "x1", "y1")}}).json()
        zones = v["zones"]
        result["zones"] = zones
        result["sent"] = v.get("sent")
        result["bridge_after_draw"] = bridge.get("/health").json().get("nogo")
        # Z6 live
        taps = []
        for i in range(10):
            z = zones[i % len(zones)]
            f = 0.2 + 0.6 * ((i // len(zones)) / 4)
            x, y = z["x0"] + f * (z["x1"] - z["x0"]), z["y0"] + f * (z["y1"] - z["y0"])
            r = robot.post("/world/goal", json={"x_m": x, "y_m": y},
                           headers={"x-driver": "twin-dpad"}).json()
            taps.append(r.get("reason"))
        result["z6_refused"] = sum(t == "no_go_zone" for t in taps)
        result["z6_taps"] = taps
        robot.delete("/world/goal")
        time.sleep(1)
        # Z1/Z2 live: one find.
        r = brain.post("/mission/start", json={"target_object": de.TARGET, "policy": "explore",
                                               "max_steps": 200, "mission_time_limit_s": LIMIT_S})
        result["start"] = r.status_code
        t0, entries, samples, status = time.time(), [], 0, {}
        while time.time() - t0 < LIMIT_S + 30:
            t = robot.get("/world/truth").json()
            ins = _inside(zones, t["x_m"], t["y_m"])
            samples += 1
            if ins:
                entries.append({"t": round(time.time() - t0, 1), "at": [t["x_m"], t["y_m"]], "zones": ins})
            status = brain.get("/mission/status").json()
            if not status.get("running") and status.get("outcome") not in (None, "idle", "running"):
                break
            time.sleep(0.5)
        ex = status.get("explore") or {}
        result["mission"] = {"outcome": status.get("outcome"), "seconds": round(time.time() - t0),
                             "goals_sent": ex.get("goals_sent"), "nogo": ex.get("nogo"),
                             "truth_samples": samples, "entries": entries,
                             "log_tail": (status.get("log_tail") or [])[-5:]}
        # Explore's goals, from the brain's log (each `explore {json}` line).
        goals = []
        try:
            for line in open(os.path.join(de.RUN_DIR or de.LOGDIR, "explore_brain.log")):
                if "explore {" in line and '"goal_sent"' in line:
                    ev = json.loads(line[line.index("explore {") + 8:])
                    goals.append(ev.get("goal"))
        except Exception as e:  # noqa: BLE001
            result["goal_log_error"] = str(e)
        result["goals"] = goals
        result["goals_in_zone"] = [g for g in goals if g and _inside(zones, *g)]
        time.sleep(2)
        result["bridge_after_mission"] = bridge.get("/health").json().get("nogo")
        result["store_after_mission"] = len(brain.get("/mission/nogo").json()["zones"])
        # Z3 live, once: a zone around where the robot stands now.
        t = robot.get("/world/truth").json()
        p = robot.get("/world/pose").json()
        v = brain.get("/mission/nogo").json()
        v = brain.post("/mission/nogo", json={"frame_id": v["frame_id"], "rev": v["rev"], "op": "add",
                                              "zone": {"x0": p["x_m"] - 0.4, "y0": p["y_m"] - 0.4,
                                                       "x1": p["x_m"] + 0.4, "y1": p["y_m"] + 0.4}}).json()
        around = v["zones"][-1]
        time.sleep(1.5)
        result["z3_bridge_at_draw"] = bridge.get("/health").json().get("nogo")
        r = brain.post("/mission/start", json={"job": "survey", "policy": "explore",
                                               "max_steps": 6, "mission_time_limit_s": 300})
        result["z3_start"] = r.status_code
        t0, left_at, restored = time.time(), None, None
        while time.time() - t0 < 300:
            t = robot.get("/world/truth").json()
            d = math.hypot(max(around["x0"] - t["x_m"], 0, t["x_m"] - around["x1"]),
                           max(around["y0"] - t["y_m"], 0, t["y_m"] - around["y1"]))
            if left_at is None and d > 0:
                left_at = round(time.time() - t0, 1)
            nogo = bridge.get("/health").json().get("nogo") or {}
            last = nogo.get("last") or {}
            if last.get("event") == "zone_restored" and last.get("id") == around["id"]:
                restored = last
                break
            s = brain.get("/mission/status").json()
            if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
                break
            time.sleep(0.3)
        brain.post("/mission/stop")
        result["z3"] = {"zone": around, "left_after_s": left_at, "restored": restored}
    finally:
        stop.set()
        result["loadavg_10s"] = load
        result["peak_load"] = max((float(x[1]) for x in load), default=None)
        with open(out, "w") as fh:
            json.dump(result, fh, indent=1)
        subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
        de._kill_ports()
    print(json.dumps({k: result.get(k) for k in ("z6_refused", "goals_in_zone", "peak_load")}))
    print(json.dumps(result.get("mission")))
    print(json.dumps(result.get("z3")))


if __name__ == "__main__":
    main(sys.argv[1])
