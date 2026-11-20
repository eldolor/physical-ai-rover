# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3193/live.py -- 3.193.B's live run on the ROS stack, judged
on the simulator's ground truth: 3.161's `evaluations/nogo-3161/live.py`
(steps 1-5 as written there: a fresh stack, `scaled_house`, the three
`sweep.place_zones` zones, Z6's taps, one find, Z4's clear) with two changes:

* the truth is read every 0.25 s (3.161 and 3.192 read it every 0.5 s);
* Z3 (3.193 D2): the zone is drawn around the robot WHILE a survey runs, once
  its true centre is 1.3 m from the survey's start, so the zone is not over
  the mission's dock (3.161 Part 2's E4 leaves a zone over the dock out of
  force, which is what 3.192's Z3 ran into). The bridge's events are read
  from `/health`'s `nogo` counters every poll and from the bridge's log
  lines at the end, not from `last` alone.

`--old-z3` runs 3.161's Z3 as written instead (3.193 R1: the trace) and
records the bridge's zones after the survey's start and the mission's
`dock_refused`.

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:plan-3.193 \\
        python evaluations/nogo-3193/live.py OUT.json [--old-z3]
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
sys.path.insert(1, os.path.join(ROOT, "evaluations", "nogo-3161"))

import httpx                                             # noqa: E402

from tests import demo_explore as de                     # noqa: E402
import sweep                                             # noqa: E402

HOUSE = "scaled_house"
VOLUME = os.environ.get("PICAR_MAPS_VOLUME", "picar-maps")
LIMIT_S = 900
TRUTH_S = 0.25
AWAY_M = 1.3          # the survey's start to the robot before Z3's zone is drawn
HALF_M = 0.4          # Z3's zone: 0.8 m a side, as 3.161's


def _with_volume():
    real = subprocess.run

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


def _inside(zones, x, y):
    return [z["id"] for z in zones if z["x0"] <= x <= z["x1"] and z["y0"] <= y <= z["y1"]]


def _rect_dist(z, x, y):
    return math.hypot(max(z["x0"] - x, 0, x - z["x1"]), max(z["y0"] - y, 0, y - z["y1"]))


def _ended(s):
    return not s.get("running") and s.get("outcome") not in (None, "idle", "running")


def _add(brain, x, y):
    v = brain.get("/mission/nogo").json()
    r = brain.post("/mission/nogo", json={"frame_id": v["frame_id"], "rev": v["rev"], "op": "add",
                                          "zone": {"x0": x - HALF_M, "y0": y - HALF_M,
                                                   "x1": x + HALF_M, "y1": y + HALF_M}})
    return r.status_code, r.json()


def _bridge_log_events(zid):
    out = subprocess.run(["docker", "logs", de.CONTAINER], capture_output=True, text=True)
    return [ln.strip()[-160:] for ln in (out.stdout + out.stderr).splitlines()
            if zid in ln and ("zone_suspended" in ln or "zone_restored" in ln)]


def _old_z3(robot, brain, bridge, result):
    """3.161's Z3 as written: a zone around the robot where it stands, then a
    survey (R1: is it left out at the survey's start?)."""
    p = robot.get("/world/pose").json()
    code, v = _add(brain, p["x_m"], p["y_m"])
    around = v["zones"][-1] if code == 200 else None
    time.sleep(1.5)
    out = {"add": code, "zone": around, "bridge_at_draw": bridge.get("/health").json().get("nogo")}
    r = brain.post("/mission/start", json={"job": "survey", "policy": "explore",
                                           "max_steps": 6, "mission_time_limit_s": 300})
    out["start"] = r.status_code
    time.sleep(3.0)
    out["bridge_after_start"] = bridge.get("/health").json().get("nogo")
    s = brain.get("/mission/status").json()
    out["explore_nogo"] = (s.get("explore") or {}).get("nogo")
    out["store"] = {k: v for k, v in brain.get("/mission/nogo").json().items()
                    if k in ("dock_conflicts", "rev")}
    t0 = time.time()
    while time.time() - t0 < 120 and not _ended(brain.get("/mission/status").json()):
        time.sleep(1.0)
    brain.post("/mission/stop")
    out["bridge_at_end"] = bridge.get("/health").json().get("nogo")
    if around:
        out["bridge_log"] = _bridge_log_events(around["id"])
    result["z3_old"] = out


def _z3(robot, brain, bridge, result):
    """D2: a zone drawn around the robot while a survey runs."""
    start = robot.get("/world/truth").json()
    r = brain.post("/mission/start", json={"job": "survey", "policy": "explore",
                                           "max_steps": 40, "mission_time_limit_s": 300})
    out = {"start": r.status_code, "start_truth": [start["x_m"], start["y_m"]]}
    t0, around, drawn_at, n0 = time.time(), None, None, None
    left_at, events, refused = None, [], []
    restored_at, restored_clear, last_seen, seen_suspended = None, None, None, False
    while time.time() - t0 < 300:
        s = brain.get("/mission/status").json()
        if _ended(s):
            out["ended"] = s.get("outcome")
            break
        t = robot.get("/world/truth").json()
        if around is None:
            if math.hypot(t["x_m"] - start["x_m"], t["y_m"] - start["y_m"]) >= AWAY_M:
                p = robot.get("/world/pose").json()
                n0 = bridge.get("/health").json().get("nogo") or {}
                code, v = _add(brain, p["x_m"], p["y_m"])
                if code == 200:
                    around, drawn_at = v["zones"][-1], time.time()
                    out["truth_at_draw"] = [t["x_m"], t["y_m"]]
                else:
                    refused.append({"code": code, "detail": v})     # review: once, not per poll
                    out["ended"] = "z3_add_refused"
                    break
            time.sleep(TRUTH_S)
            continue
        if left_at is None and _rect_dist(around, t["x_m"], t["y_m"]) > 0:
            left_at = round(time.time() - drawn_at, 2)
        nogo = bridge.get("/health").json().get("nogo") or {}
        last = nogo.get("last") or {}
        if last != last_seen:
            last_seen = last
            if last.get("id") == around["id"]:
                events.append({**last, "t": round(time.time() - drawn_at, 2),
                               "truth_clear_m": round(_rect_dist(around, t["x_m"], t["y_m"]), 3)})
        # Review: this zone's own events only -- seen suspended, then gone from
        # `suspended` with the restored counter up (another zone's restore
        # moves the same counter).
        if around["id"] in nogo.get("suspended", []):
            if not seen_suspended:
                n0 = nogo                    # review: count restores from here
            seen_suspended = True
        if (seen_suspended and around["id"] not in nogo.get("suspended", [])
                and nogo.get("restored", 0) > n0.get("restored", 0)):
            restored_at = round(time.time() - drawn_at, 2)
            restored_clear = round(_rect_dist(around, t["x_m"], t["y_m"]), 3)
            break
        time.sleep(TRUTH_S)
    brain.post("/mission/stop")
    n1 = bridge.get("/health").json().get("nogo") or {}
    out.update({"zone": around, "refused_adds": refused, "left_after_s": left_at,
                "events": events, "seen_suspended": seen_suspended, "restored_after_s": restored_at,
                "truth_clear_at_restore_m": restored_clear,
                "counters_at_suspended": {k: n0.get(k) for k in ("suspended_count", "restored")} if n0 else None,
                "counters_after": {k: n1.get(k) for k in ("suspended_count", "restored")},
                "bridge_log": _bridge_log_events(around["id"]) if around else []})
    t1 = time.time()
    while time.time() - t1 < 60 and not _ended(brain.get("/mission/status").json()):
        time.sleep(1.0)
    out["final_outcome"] = brain.get("/mission/status").json().get("outcome")
    result["z3"] = out


def main(out, old_z3=False):
    load, stop = [], threading.Event()
    threading.Thread(target=_uptime, args=(load, stop), daemon=True).start()
    _with_volume()
    result = {"house": HOUSE, "image": os.environ.get("ROS_IMAGE"), "truth_every_s": TRUTH_S}
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
        if not zones:
            raise RuntimeError(f"setup: no zone stored ({v})")       # review
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
        t0, entries, samples, status, closest = time.time(), [], 0, {}, None
        while time.time() - t0 < LIMIT_S + 30:
            t = robot.get("/world/truth").json()
            ins = _inside(zones, t["x_m"], t["y_m"])
            samples += 1
            d = min(_rect_dist(z, t["x_m"], t["y_m"]) for z in zones)
            closest = d if closest is None else min(closest, d)
            if ins:
                # the SLAM pose beside the truth: does the robot's own map see it inside too?
                pp = robot.get("/world/pose").json()
                entries.append({"t": round(time.time() - t0, 2), "at": [t["x_m"], t["y_m"]],
                                "slam_pose": [pp.get("x_m"), pp.get("y_m")],
                                "slam_inside": _inside(zones, pp.get("x_m") or 1e9, pp.get("y_m") or 1e9),
                                "zones": ins})
            status = brain.get("/mission/status").json()
            if _ended(status):
                break
            time.sleep(TRUTH_S)
        ex = status.get("explore") or {}
        result["mission"] = {"outcome": status.get("outcome"), "seconds": round(time.time() - t0, 1),
                             "goals_sent": ex.get("goals_sent"), "nogo": ex.get("nogo"),
                             "truth_samples": samples, "entries": entries,
                             "closest_outside_m": None if closest is None else round(closest, 4),
                             "log_tail": (status.get("log_tail") or [])[-5:]}
        goals = []
        try:
            for line in open(os.path.join(de.RUN_DIR or de.LOGDIR, "explore_brain.log")):
                if "explore {" in line and '"goal_sent"' in line:
                    ev = json.loads(line[line.index("explore {") + 8:])
                    goals.append(ev.get("goal"))
        except OSError:
            pass
        result["goals"] = goals
        result["goals_in_zone"] = sum(bool(g and _inside(zones, g[0], g[1])) for g in goals)
        time.sleep(2)
        result["bridge_after_mission"] = bridge.get("/health").json().get("nogo")
        result["store_after_mission"] = len(brain.get("/mission/nogo").json()["zones"])
        (_old_z3 if old_z3 else _z3)(robot, brain, bridge, result)
    finally:
        stop.set()
        result["loadavg_10s"] = load
        result["peak_load"] = max((float(x[1]) for x in load), default=None)
        with open(out, "w") as fh:
            json.dump(result, fh, indent=1)
        subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
        de._kill_ports()
    m = result.get("mission") or {}
    print(json.dumps({k: m.get(k) for k in ("outcome", "seconds", "goals_sent", "closest_outside_m")}
                     | {"entries": len(m.get("entries") or []), "goals_in_zone": result.get("goals_in_zone"),
                        "peak_load": result.get("peak_load")}))
    print(json.dumps(result.get("z3_old") or result.get("z3"))[:2000])


if __name__ == "__main__":
    main(sys.argv[1], old_z3="--old-z3" in sys.argv[2:])
