# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3195/live.py -- 3.195's live runs on the ROS stack: 3.193's
`evaluations/nogo-3193/live.py` (a fresh stack, `scaled_house`, the three
`sweep.place_zones("scaled_house", 0)` zones, Z6's taps, one find, the
mission-end clear, then 3.193 D2's Z3 drawn mid-survey -- its `_z3` is
called as it is) with ONE change: every truth sample of the find carries the
SLAM pose beside it, so a near miss splits into what SLAM got wrong and what
nav2 did on its own estimate (3.195 M1).

Each sample reads the truth, then `/world/pose` (SLAM's estimate through the
anchor, the frame the zones are drawn in), then the truth again, and takes
the truth at the pose's own instant by interpolating the two reads (the
robot moves up to 0.2 m/s; a 10 ms gap is 2 mm). Per sample: the true and
the SLAM signed distance to each zone (negative inside), and the offset
SLAM - truth.

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:plan-3.195 \\
        PICAR_MAPS_VOLUME=<fresh> NOGO_DIR=<fresh> \\
        python evaluations/nogo-3195/live.py OUT.json
"""

import importlib.util
import json
import math
import os
import subprocess
import sys
import threading
import time

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(1, os.path.join(ROOT, "evaluations", "nogo-3161"))

import httpx                                             # noqa: E402

from tests import demo_explore as de                     # noqa: E402
import sweep                                             # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "live3193", os.path.join(ROOT, "evaluations", "nogo-3193", "live.py"))
L3 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(L3)

HOUSE = L3.HOUSE
LIMIT_S = L3.LIMIT_S
TRUTH_S = L3.TRUTH_S


def signed_dist(z, x, y):
    """Metres from (x, y) to an axis-aligned zone's edge: positive outside,
    negative inside (the depth)."""
    dx, dy = max(z["x0"] - x, 0.0, x - z["x1"]), max(z["y0"] - y, 0.0, y - z["y1"])
    if dx or dy:
        return math.hypot(dx, dy)
    return -min(x - z["x0"], z["x1"] - x, y - z["y0"], z["y1"] - y)


def paired(robot):
    """(truth at the pose's instant, the pose, the read's span in s), or
    None when either is unusable."""
    t0 = time.time()
    a = robot.get("/world/truth").json()
    t1 = time.time()
    p = robot.get("/world/pose").json()
    t2 = time.time()
    b = robot.get("/world/truth").json()
    t3 = time.time()
    if not (a.get("usable") and b.get("usable") and p.get("usable")):
        return None
    ta, tb, tp = (t0 + t1) / 2, (t2 + t3) / 2, (t1 + t2) / 2
    f = 0.0 if tb == ta else (tp - ta) / (tb - ta)
    tx = a["x_m"] + f * (b["x_m"] - a["x_m"])
    ty = a["y_m"] + f * (b["y_m"] - a["y_m"])
    return (tx, ty), (p["x_m"], p["y_m"]), t3 - t0


def sample(zones, truth, pose, t, span):
    td = {z["id"]: signed_dist(z, *truth) for z in zones}
    sd = {z["id"]: signed_dist(z, *pose) for z in zones}
    near = min(td, key=td.get)
    return {"t": round(t, 2), "truth": [round(truth[0], 4), round(truth[1], 4)],
            "slam": [round(pose[0], 4), round(pose[1], 4)],
            "offset_m": [round(pose[0] - truth[0], 4), round(pose[1] - truth[1], 4)],
            "zone": near, "truth_d_m": round(td[near], 4), "slam_d_m": round(sd[near], 4),
            "slam_min_d_m": round(min(sd.values()), 4), "span_s": round(span, 4)}


def main(out):
    load, stop = [], threading.Event()
    threading.Thread(target=L3._uptime, args=(load, stop), daemon=True).start()
    L3._with_volume()
    result = {"house": HOUSE, "image": os.environ.get("ROS_IMAGE"), "truth_every_s": TRUTH_S,
              "volume": L3.VOLUME, "nogo_dir": os.environ.get("NOGO_DIR")}
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
        if len(zones) != 3:
            raise RuntimeError(f"setup: {len(zones)} zones stored, not 3 ({v})")
        result["bridge_after_draw"] = bridge.get("/health").json().get("nogo")
        taps = []
        for i in range(10):
            z = zones[i % len(zones)]
            f = 0.2 + 0.6 * ((i // len(zones)) / 4)
            x, y = z["x0"] + f * (z["x1"] - z["x0"]), z["y0"] + f * (z["y1"] - z["y0"])
            r = robot.post("/world/goal", json={"x_m": x, "y_m": y},
                           headers={"x-driver": "twin-dpad"}).json()
            taps.append(r.get("reason"))
        result["z6_refused"] = sum(t == "no_go_zone" for t in taps)
        robot.delete("/world/goal")
        time.sleep(1)
        r = brain.post("/mission/start", json={"target_object": de.TARGET, "policy": "explore",
                                               "max_steps": 200, "mission_time_limit_s": LIMIT_S})
        result["start"] = r.status_code
        t0, samples, status, skipped = time.time(), [], {}, 0
        while time.time() - t0 < LIMIT_S + 30:
            got = paired(robot)
            if got is None:
                skipped += 1
            else:
                samples.append(sample(zones, got[0], got[1], time.time() - t0, got[2]))
            status = brain.get("/mission/status").json()
            if L3._ended(status):
                break
            time.sleep(TRUTH_S)
        ex = status.get("explore") or {}
        entries = [s for s in samples if s["truth_d_m"] <= 0]
        closest = min(samples, key=lambda s: s["truth_d_m"]) if samples else None
        offs = sorted(math.hypot(*s["offset_m"]) for s in samples)
        result["mission"] = {
            "outcome": status.get("outcome"), "seconds": round(time.time() - t0, 1),
            "goals_sent": ex.get("goals_sent"), "nogo": ex.get("nogo"),
            "truth_samples": len(samples), "unusable_samples": skipped,
            "entries": entries, "closest": closest,
            "closest_outside_m": None if closest is None else closest["truth_d_m"],
            "slam_closest_outside_m": min((s["slam_min_d_m"] for s in samples), default=None),
            "offset_m_median": offs[len(offs) // 2] if offs else None,
            "offset_m_max": offs[-1] if offs else None,
            "log_tail": (status.get("log_tail") or [])[-5:]}
        result["samples"] = samples
        goals = []
        try:
            for line in open(os.path.join(de.RUN_DIR or de.LOGDIR, "explore_brain.log")):
                if "explore {" in line and '"goal_sent"' in line:
                    ev = json.loads(line[line.index("explore {") + 8:])
                    goals.append(ev.get("goal"))
        except OSError:
            pass
        result["goals"] = goals
        result["goals_in_zone"] = sum(bool(g and L3._inside(zones, g[0], g[1])) for g in goals)
        time.sleep(2)
        result["bridge_after_mission"] = bridge.get("/health").json().get("nogo")
        result["store_after_mission"] = len(brain.get("/mission/nogo").json()["zones"])
        L3._z3(robot, brain, bridge, result)
    finally:
        stop.set()
        result["loadavg_10s"] = load
        result["peak_load"] = max((float(x[1]) for x in load), default=None)
        with open(out, "w") as fh:
            json.dump(result, fh, indent=1)
        subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
        de._kill_ports()
    m = result.get("mission") or {}
    print(json.dumps({k: m.get(k) for k in ("outcome", "seconds", "goals_sent", "closest_outside_m",
                                            "slam_closest_outside_m", "offset_m_median", "offset_m_max")}
                     | {"entries": len(m.get("entries") or []), "goals_in_zone": result.get("goals_in_zone"),
                        "peak_load": result.get("peak_load")}))
    z3 = result.get("z3") or {}
    print(json.dumps({k: z3.get(k) for k in ("left_after_s", "seen_suspended", "restored_after_s",
                                             "truth_clear_at_restore_m", "final_outcome", "ended")}))


if __name__ == "__main__":
    main(sys.argv[1])
