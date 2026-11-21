# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3199/live.py -- 3.199 G8, once, on the ROS stack: a fresh
stack (`scaled_house`, `tests/demo_explore.stack`), the first of
`sweep.place_zones("scaled_house", 0)` stored through the brain
(`POST /mission/nogo`, so it reaches the robot server and the bridge as a
person's zone does), then a person's taps (`x-driver: twin-dpad`) beside the
middle of the first edge whose outside is open floor: 0.02, 0.06 and 0.11 m
out (each must be refused `no_go_zone`, with no nav2 goal started), then
0.30 m out (accepted; nav2 drives there; the truth is read every 0.25 s and
must never enter the zone).

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:plan-3.199 \\
        PICAR_MAPS_VOLUME=<fresh> NOGO_DIR=<fresh> \\
        python evaluations/nogo-3199/live.py OUT.json
"""

import json
import math
import os
import sys
import threading
import time

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(1, os.path.join(ROOT, "evaluations", "nogo-3161"))

import httpx                                             # noqa: E402

from tests import demo_explore as de                     # noqa: E402
import sweep                                             # noqa: E402

HOUSE = "scaled_house"
VOLUME = os.environ.get("PICAR_MAPS_VOLUME", "picar-maps")
BAND = (0.02, 0.06, 0.11)
CLEAR_M = 0.30
DRIVE_S = 120


def _with_volume():
    real = de.subprocess.run

    def run(args, *a, **k):
        if isinstance(args, list) and args[:3] == ["docker", "run", "-d"]:
            args = args[:3] + ["-v", f"{VOLUME}:/maps"] + args[3:]
        return real(args, *a, **k)
    de.subprocess.run = run


def _uptime(samples, stop):
    while not stop.is_set():
        with open("/proc/loadavg") as fh:
            samples.append([round(time.time(), 1), fh.read().split()[0]])
        stop.wait(10)


def _open_floor(g, x, y, r=0.35):
    """Every grid cell within r of (x, y) is floor (the simulator's map)."""
    for dx in (-r, 0.0, r):
        for dy in (-r, 0.0, r):
            c = (int(math.floor((x + dx) / sweep.CELL)), int(math.floor((y + dy) / sweep.CELL)))
            if not sweep._floor(g, c):
                return False
    return True


def _side(g, z):
    """The first edge (E, W, N, S) whose middle has open floor CLEAR_M out:
    (unit normal, the edge's middle)."""
    mx, my = (z["x0"] + z["x1"]) / 2, (z["y0"] + z["y1"]) / 2
    for n, mid in (((1, 0), (z["x1"], my)), ((-1, 0), (z["x0"], my)),
                   ((0, 1), (mx, z["y1"])), ((0, -1), (mx, z["y0"]))):
        if _open_floor(g, mid[0] + n[0] * CLEAR_M, mid[1] + n[1] * CLEAR_M):
            return n, mid
    return None, None


def main(out):
    load, stop = [], threading.Event()
    threading.Thread(target=_uptime, args=(load, stop), daemon=True).start()
    _with_volume()
    result = {"house": HOUSE, "image": os.environ.get("ROS_IMAGE")}
    robot = de.stack(HOUSE)
    brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=10)
    try:
        p = {}
        for _ in range(60):
            p = robot.get("/world/pose").json()
            if p.get("usable") and p.get("frame_id"):
                break
            time.sleep(2)
        result["frame_id"] = p.get("frame_id")
        g = sweep.build_world(HOUSE)
        z = sweep.place_zones(HOUSE, 0)[0]
        n, mid = _side(g, z)
        if n is None:
            raise RuntimeError(f"setup: no open side beside {z}")
        v = brain.get("/mission/nogo").json()
        v = brain.post("/mission/nogo", json={"frame_id": v["frame_id"], "rev": v["rev"], "op": "add",
                                              "zone": {k: z[k] for k in ("x0", "y0", "x1", "y1")}}).json()
        if not v.get("zones"):
            raise RuntimeError(f"setup: no zone stored ({v})")
        zone = v["zones"][0]
        result["zone"], result["normal"] = zone, n
        time.sleep(1.5)
        result["bridge_after_draw"] = bridge.get("/health").json().get("nogo")
        taps = []
        for d in BAND:
            x, y = mid[0] + n[0] * d, mid[1] + n[1] * d
            before = bridge.get("/goal").json().get("goal")
            r = robot.post("/world/goal", json={"x_m": x, "y_m": y},
                           headers={"x-driver": "twin-dpad"}).json()
            time.sleep(0.5)
            after = bridge.get("/goal").json().get("goal")
            taps.append({"d_m": d, "at": [x, y], "reply": r, "goal_before": before,
                         "goal_after": after, "reached_nav2": after != before})
        result["band_taps"] = taps
        result["band_refused"] = sum(t["reply"].get("reason") == "no_go_zone" for t in taps)
        result["band_reached_nav2"] = sum(t["reached_nav2"] for t in taps)
        gx, gy = mid[0] + n[0] * CLEAR_M, mid[1] + n[1] * CLEAR_M
        r = robot.post("/world/goal", json={"x_m": gx, "y_m": gy},
                       headers={"x-driver": "twin-dpad"}).json()
        result["clear_tap"] = {"at": [gx, gy], "reply": r}
        t0, entries, closest, nav, samples = time.time(), [], None, {}, 0
        while time.time() - t0 < DRIVE_S:
            t = robot.get("/world/truth").json()
            samples += 1
            d = sweep._dist(zone, t["x_m"], t["y_m"])
            closest = d if closest is None else min(closest, d)
            if sweep._in(zone, t["x_m"], t["y_m"]):
                entries.append([round(time.time() - t0, 2), t["x_m"], t["y_m"]])
            nav = bridge.get("/goal").json().get("goal") or {}
            if time.time() - t0 > 3 and nav.get("state") not in ("pending", "active"):
                break
            time.sleep(0.25)
        t = robot.get("/world/truth").json()
        result["drive"] = {"seconds": round(time.time() - t0, 1), "samples": samples,
                           "entries": entries, "closest_outside_m": closest,
                           "end_truth": [t["x_m"], t["y_m"]],
                           "end_to_goal_m": round(math.hypot(t["x_m"] - gx, t["y_m"] - gy), 4),
                           "goal_status": nav}
    finally:
        stop.set()
        result["load_1min_peak"] = max((float(v) for _, v in load), default=None)
        with open(out, "w") as fh:
            json.dump(result, fh, indent=1, default=str)
        de._kill_ports()
        de.subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)


if __name__ == "__main__":
    main(sys.argv[1])
