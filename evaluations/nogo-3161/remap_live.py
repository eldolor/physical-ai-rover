# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3161/remap_live.py -- 3.161.B's Z4 re-map, live (bars K1-K4
in the section file, written before the run).

Three stacks on one fresh maps volume, scaled_house, judged on the bridge's
and the brain's own records plus the simulator's truth:

1. Stack 1 (fresh map): place_zones(house, 0)'s 3 zones through the brain's
   POST /mission/nogo; one find, whose end saves the map.
2. Stack 2: the saved map reloads (frame_id F); a person's re-map
   (POST /world/remap, x-driver twin-dpad) until the bridge's re-map record
   reads `mapping`; zones checked; one find (entries counted), whose end
   saves the re-mapped version; zones checked.
3. Stack 3: the re-mapped version reloads; frame_id and zones checked.

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:plan-3.161 \\
        PICAR_MAPS_VOLUME=picar-maps-3161-remap \\
        python evaluations/nogo-3161/remap_live.py OUT.json
"""

import json
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
import live                                              # noqa: E402
import sweep                                             # noqa: E402

HOUSE = "scaled_house"
LIMIT_S = 600


def _clients():
    return (httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60),
            httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=10))


def _wait_pose(robot, need_ready=True, limit=180):
    p, h = {}, {}
    t0 = time.time()
    while time.time() - t0 < limit:
        try:
            p = robot.get("/world/pose").json()
            h = httpx.get(f"http://127.0.0.1:{de.BRIDGE}/health", timeout=10).json()
            if p.get("usable") and p.get("frame_id") and (not need_ready or (h.get("slam") or {}).get("state") == "ready"):
                break
        except Exception:  # noqa: BLE001 -- still coming up
            pass
        time.sleep(2)
    return p, h


def _zones(brain, bridge):
    b = bridge.get("/health").json()
    return {"bridge": (b.get("nogo") or {}).get("zones"),
            "store": len(brain.get("/mission/nogo").json().get("zones") or []),
            "store_frame_id": brain.get("/mission/nogo").json().get("frame_id")}


def _find(robot, brain, zones):
    r = brain.post("/mission/start", json={"target_object": de.TARGET, "policy": "explore",
                                           "max_steps": 200, "mission_time_limit_s": LIMIT_S})
    t0, entries, samples, status = time.time(), [], 0, {}
    while time.time() - t0 < LIMIT_S + 30:
        t = robot.get("/world/truth").json()
        ins = live._inside(zones, t["x_m"], t["y_m"])
        samples += 1
        if ins:
            entries.append({"t": round(time.time() - t0, 1), "at": [t["x_m"], t["y_m"]], "zones": ins})
        status = brain.get("/mission/status").json()
        if not status.get("running") and status.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(0.5)
    return {"start": r.status_code, "outcome": status.get("outcome"),
            "seconds": round(time.time() - t0), "truth_samples": samples, "entries": entries,
            "log_tail": (status.get("log_tail") or [])[-5:]}


def _wait_saved(bridge, before, limit=60):
    """The mission-end save: the bridge's map record (whatever it reports)
    after the find, polled until it changes or the limit."""
    t0, h = time.time(), {}
    while time.time() - t0 < limit:
        h = bridge.get("/health").json()
        if json.dumps(h.get("map"), sort_keys=True) != before:
            break
        time.sleep(2)
    time.sleep(3)
    return bridge.get("/health").json().get("map")


def _teardown():
    subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
    de._kill_ports()
    time.sleep(3)


def main(out):
    load, stop = [], threading.Event()
    threading.Thread(target=live._uptime, args=(load, stop), daemon=True).start()
    live._with_volume()
    res = {"house": HOUSE, "image": os.environ.get("ROS_IMAGE"), "volume": live.VOLUME}
    try:
        # ---- stack 1 ----
        robot = de.stack(HOUSE)
        brain, bridge = _clients()
        p, h = _wait_pose(robot)
        s1 = {"frame_id": p.get("frame_id"), "slam": (h.get("slam") or {}).get("state"),
              "map": h.get("map")}
        v = brain.get("/mission/nogo").json()
        for z in sweep.place_zones(HOUSE, 0):
            v = brain.post("/mission/nogo", json={"frame_id": v["frame_id"], "rev": v["rev"], "op": "add",
                                                  "zone": {k: z[k] for k in ("x0", "y0", "x1", "y1")}}).json()
        zones = v["zones"]
        res["zones"] = zones
        time.sleep(1.5)
        s1["zones_after_draw"] = _zones(brain, bridge)
        before = json.dumps(bridge.get("/health").json().get("map"), sort_keys=True)
        s1["find"] = _find(robot, brain, zones)
        s1["map_after_find"] = _wait_saved(bridge, before)
        res["stack1"] = s1
        _teardown()
        # ---- stack 2 ----
        robot = de.stack(HOUSE)
        brain, bridge = _clients()
        p, h = _wait_pose(robot)
        s2 = {"frame_id_reload": p.get("frame_id"), "slam": (h.get("slam") or {}).get("state"),
              "reload": (h.get("slam") or {}).get("reload"), "map": h.get("map")}
        time.sleep(2)
        s2["zones_after_start"] = _zones(brain, bridge)
        r = robot.post("/world/remap", headers={"x-driver": "twin-dpad"})
        s2["remap_reply"] = {"status": r.status_code, "body": r.json()}
        rec, t0 = {}, time.time()
        while time.time() - t0 < 120:
            hh = bridge.get("/health").json()
            rec = (hh.get("slam") or {}).get("remap") or {}
            if rec.get("result") not in (None, "none", "waiting", "starting"):
                break
            time.sleep(1)
        s2["remap_record"] = rec
        s2["remap_health_slam"] = {k: v for k, v in (hh.get("slam") or {}).items() if k != "reload"}
        time.sleep(2)
        s2["frame_id_after_remap"] = robot.get("/world/pose").json().get("frame_id")
        s2["zones_after_remap"] = _zones(brain, bridge)
        before = json.dumps(bridge.get("/health").json().get("map"), sort_keys=True)
        s2["find"] = _find(robot, brain, zones)
        s2["map_after_find"] = _wait_saved(bridge, before)
        s2["zones_after_find"] = _zones(brain, bridge)
        res["stack2"] = s2
        _teardown()
        # ---- stack 3 ----
        robot = de.stack(HOUSE)
        brain, bridge = _clients()
        p, h = _wait_pose(robot)
        time.sleep(3)
        res["stack3"] = {"frame_id_reload": p.get("frame_id"), "slam": (h.get("slam") or {}).get("state"),
                         "reload": (h.get("slam") or {}).get("reload"), "map": h.get("map"),
                         "zones_after_start": _zones(brain, bridge)}
    finally:
        stop.set()
        res["loadavg_10s"] = load
        res["peak_load"] = max((float(x[1]) for x in load), default=None)
        with open(out, "w") as fh:
            json.dump(res, fh, indent=1, default=str)
        _teardown()
    print(json.dumps({k: res.get(k) for k in ("peak_load",)}))


if __name__ == "__main__":
    main(sys.argv[1])
