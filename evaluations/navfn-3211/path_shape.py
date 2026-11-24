# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/navfn-3211/path_shape.py -- 3.211.C: nav2's path shape on
`scaled_house`, `use_astar: false` (F, the image's own nav2.yaml) against
`true` (T, a copy that differs in that line alone, mounted over it). How it
is read is docs/plans/ros-alignment/3.211-navfn-plan-failures.md, 3.211.C,
written before any run.

Each run is R6's instrument (tests/demo_nav_goals.py, 3.15) on a fresh
stack (tests/demo_explore.stack(), which takes the desktop's exclusive
lock): the mapping lap, the six goals, the unreachable goal, the D-pad tap.
Beside it, a poller keeps each goal's first plan and scores it on the
simulator's truth, and a sampler keeps the load. Rows keep numbers only.

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host PICAR_KEEP_LOGS=... python evaluations/navfn-3211/path_shape.py run OUT
    python evaluations/navfn-3211/path_shape.py score OUT
"""

import json
import math
import os
import re
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NAV_YAML = ROOT / "service" / "slam" / "src" / "picar_bringup" / "config" / "nav2.yaml"
ORDER = ["T", "F", "T", "F", "T", "F"]       # interleaved, written before any run
LOAD_MAX = 4.0                               # 3.79
HALF_WIDTH_M = 0.1155                        # tests/demo_nav_goals.HALF_WIDTH_M
# The goal comes back re-anchored to SLAM's current map (review): a few cm
# off what was sent, and the six goals are metres apart.
MATCH_M = 0.25


def arm_yaml(arm, out):
    """F: None (the image's own). T: a copy with use_astar: true, the only change."""
    if arm == "F":
        return None
    text = NAV_YAML.read_text()
    new, n = re.subn(r"(\n\s*use_astar:\s*)false", r"\g<1>true", text)
    assert n == 1, "nav2.yaml must name use_astar: false once"
    p = Path(out) / "nav2-T.yaml"
    p.write_text(new)
    return p


def plan_stats(plan, clearance):
    """Length and truth clearance of one plan (house-frame points)."""
    if len(plan) < 2:
        return None
    length = sum(math.dist(a, b) for a, b in zip(plan, plan[1:]))
    clear = [clearance(x, y) for x, y in plan]
    return {"plan_len_m": round(length, 3), "plan_min_clear_m": round(min(clear), 3),
            "plan_median_clear_m": round(statistics.median(clear), 3), "plan_points": len(plan)}


class FirstPlan(threading.Thread):
    """Polls GET /world/goal until the goal at (x, y) reports a plan; keeps
    the first. The bridge empties its plan when a goal is sent, so a plan
    seen with this goal's coordinates is this goal's."""

    def __init__(self, client, x, y):
        super().__init__(daemon=True)
        self.client, self.x, self.y = client, x, y
        self.plan, self.done = None, threading.Event()

    def run(self):
        end = time.time() + 150
        while not self.done.is_set() and time.time() < end:
            try:
                r = self.client.get("/world/goal").json()
                g = r.get("goal") or {}
                if (r.get("plan") and g.get("x_m") is not None
                        and math.dist((g["x_m"], g["y_m"]), (self.x, self.y)) < MATCH_M):
                    self.plan = r["plan"]
                    return
            except Exception:  # noqa: BLE001 -- a slow reply: look again
                pass
            time.sleep(0.1)


class Load(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.samples, self.done = [], threading.Event()

    def run(self):
        while not self.done.is_set():
            self.samples.append(round(os.getloadavg()[0], 2))
            self.done.wait(10)


def one_run(arm, k, out):
    import httpx

    from tests import demo_explore as de
    from tests import demo_nav_goals as nav
    assert nav.HOUSE == "scaled_house"
    # The clients take PICAR_ROBOT_URL / PICAR_BRIDGE_URL, the stack its
    # ports: both from `plan_section.py env`, or the run drives another robot.
    assert (nav.client().base_url.port, nav.bridge().base_url.port) == (de.ROBOT, de.BRIDGE), \
        "eval \"$(python tools/plan_section.py env)\" first"
    load = Load()
    poll = None
    try:
        de.stack("scaled_house", nav_yaml=arm_yaml(arm, out))
        load.start()                 # review: not while the lock waits or the last run drains
        end = time.time() + 240
        while True:
            r = subprocess.run(["docker", "logs", de.CONTAINER], capture_output=True, text=True)
            if "nav2 active" in r.stdout + r.stderr:
                break
            if time.time() > end:
                raise RuntimeError("nav2 did not come up in 240 s")
            time.sleep(2)
        param = subprocess.run(
            ["docker", "exec", de.CONTAINER, "bash", "-c",
             "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && "
             "ros2 param get /planner_server GridBased.use_astar"],
            capture_output=True, text=True).stdout.strip()
        if param != f"Boolean value is: {arm == 'T'}":
            raise RuntimeError(f"arm {arm}: planner_server says {param!r}")
        robot, br = nav.client(), nav.bridge()
        poll = httpx.Client(base_url=robot.base_url, headers=robot.headers, timeout=10)
        end = time.time() + 35.0                     # as tests/test_nav_live.py
        while not br.get("/slam/pose").json().get("start_truth"):
            if time.time() > end:
                raise RuntimeError("the bridge recorded no start truth")
            time.sleep(0.2)
        nav.map_first(robot)
        br.post("/nav/stats/reset", json={})
        goals = []
        for name, x, y in nav.GOALS:
            poller = FirstPlan(poll, x, y)
            poller.start()
            res = nav.run_goal(robot, x, y)
            poller.done.set()
            poller.join(2)
            ps = plan_stats(poller.plan or [], nav.clearance_m)
            goals.append({"goal": name, **res, **(ps or {"plan_len_m": None})})
            print(arm, k, json.dumps(goals[-1]), flush=True)
        stats = br.get("/nav/stats").json()
        unreach = nav.unreachable(robot)
        pre = nav.preempt(robot, br)
    finally:
        load.done.set()
        if load.is_alive():
            load.join(1)
        if poll is not None:
            poll.close()
    load_max = max(load.samples) if load.samples else None
    return {"arm": arm, "run": k, "use_astar_param": param, "image": os.environ.get("ROS_IMAGE"),
            "load_max": load_max, "load_p50": statistics.median(load.samples) if load.samples else None,
            "valid": load_max is not None and load_max <= LOAD_MAX,
            "goals": goals, "reversals": stats.get("reversals"), "metres": stats.get("metres"),
            "unreachable": {key: unreach.get(key) for key in ("state", "seconds", "wheels_stopped")},
            "preempt": {"tap_executed": pre.get("tap_executed"),
                        "seconds": pre.get("seconds_from_tap_sent_to_canceled")},
            "at": time.strftime("%Y-%m-%dT%H:%M:%S")}


def run(out, order=ORDER):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    os.environ["SIM_MAP"] = "scaled_house"
    sys.path.insert(0, str(ROOT))
    from tests import demo_explore as de
    rows = out / "rows-path-shape.jsonl"
    done = ({(r["arm"], r["run"]) for r in map(json.loads, rows.read_text().splitlines())}
            if rows.exists() else set())
    counts = {}
    try:
        for arm in order:
            counts[arm] = counts.get(arm, 0) + 1
            k = counts[arm]
            if (arm, k) in done:
                continue
            row = one_run(arm, k, out)
            if not row["valid"]:                 # repeated once (3.211.C's method)
                print(arm, k, "invalid at load", row["load_max"], "-- repeated once", flush=True)
                with open(out / "rows-invalid.jsonl", "a") as f:
                    f.write(json.dumps(row) + "\n")
                row = dict(one_run(arm, k, out), repeat=True)
            with open(rows, "a") as f:
                f.write(json.dumps(row) + "\n")
    finally:
        de._kill_ports()


def _med(xs):
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else None


def score(out):
    rows = [json.loads(line) for line in
            (Path(out) / "rows-path-shape.jsonl").read_text().splitlines()]
    by = {a: [r for r in rows if r["arm"] == a] for a in ("T", "F")}
    valid = {a: [r for r in by[a] if r["valid"]] for a in by}
    s = {"runs": {a: len(by[a]) for a in by}, "valid": {a: len(valid[a]) for a in by},
         "load_max": max(r["load_max"] for r in rows),
         "params": {a: sorted({r["use_astar_param"] for r in by[a]}) for a in by}}

    def succ(r):
        return [g for g in r["goals"] if g["state"] == "succeeded"]

    # C1: goals reached.
    s["succeeded"] = {a: [len(succ(r)) for r in by[a]] for a in by}
    s["C1"] = ("MET" if all(len(succ(r)) >= 5 and all(g["end_error_m"] <= 0.20 for g in succ(r))
                            for r in by["F"])
               and sum(s["succeeded"]["F"]) >= sum(s["succeeded"]["T"]) - 1 else "FAILED")
    # C2: no contact.
    # A goal never sent (`not_sent`) has no clearance or time (review): it
    # reads 0, so it fails C2 and C5 rather than crashing the score.
    s["min_clearance"] = {a: [min(g.get("min_clearance_m", 0.0) for g in r["goals"]) for r in by[a]]
                          for a in by}
    s["C2"] = "MET" if all(c >= HALF_WIDTH_M for c in s["min_clearance"]["F"]) else "FAILED"
    # C3: smoothness, valid runs only.
    s["reversals_per_m"] = {a: [round(r["reversals"] / r["metres"], 3) if r["metres"] else None
                                for r in by[a]] for a in by}
    if len(valid["F"]) < 2:
        s["C3"] = "NOT JUDGED"
    else:
        s["C3"] = ("MET" if all(r["metres"] and r["reversals"] / r["metres"] <= 1.0
                                for r in valid["F"]) else "FAILED")
    # C4: the plans' wall and door clearance, all 18 first plans an arm.
    pmin = {a: [g["plan_min_clear_m"] for r in by[a] for g in r["goals"]
                if g.get("plan_min_clear_m") is not None] for a in by}
    s["plans"] = {a: len(pmin[a]) for a in by}
    s["plan_min_clear_median"] = {a: _med(pmin[a]) for a in by}
    s["plan_min_clear_least"] = {a: min(pmin[a]) if pmin[a] else None for a in by}
    s["C4"] = ("MET" if s["plans"]["F"] == 18 and s["plans"]["T"] == 18
               and s["plan_min_clear_median"]["F"] >= s["plan_min_clear_median"]["T"] - 0.02
               and s["plan_min_clear_least"]["F"] >= s["plan_min_clear_least"]["T"] - 0.02
               else "FAILED")
    # C5: length and time.
    ratios = []
    for i in range(len(by["F"][0]["goals"])):
        f = _med([r["goals"][i].get("plan_len_m") for r in by["F"]])
        t = _med([r["goals"][i].get("plan_len_m") for r in by["T"]])
        if f is not None and t:
            ratios.append(f / t)
    s["plan_len_ratio_median"] = round(_med(ratios), 3) if ratios else None
    s["metres_median"] = {a: _med([r["metres"] for r in by[a]]) for a in by}
    s["metres_ratio"] = round(s["metres_median"]["F"] / s["metres_median"]["T"], 3)
    s["goal_seconds_median"] = {a: _med([sum(g.get("seconds", math.inf) for g in r["goals"])
                                         for r in valid[a]]) for a in by}
    secs = s["goal_seconds_median"]
    s["seconds_ratio"] = round(secs["F"] / secs["T"], 3) if secs["F"] and secs["T"] else None
    timed = len(valid["F"]) >= 2 and len(valid["T"]) >= 2
    shape = len(ratios) == 6 and s["plan_len_ratio_median"] <= 1.02 and s["metres_ratio"] <= 1.05
    if not shape or (timed and s["seconds_ratio"] > 1.10):
        s["C5"] = "FAILED"
    else:
        s["C5"] = "MET" if timed else "MET (time NOT JUDGED)"
    # C6: R6 criteria 3 and 4.
    s["C6"] = ("MET" if all(r["unreachable"]["state"] in ("aborted", "rejected")
                            and r["unreachable"]["seconds"] <= 60 and r["unreachable"]["wheels_stopped"]
                            and r["preempt"]["tap_executed"] and r["preempt"]["seconds"] is not None
                            and r["preempt"]["seconds"] <= 1.0 for r in by["F"]) else "FAILED")
    return s


if __name__ == "__main__":
    cmd, out = sys.argv[1], sys.argv[2]
    if cmd == "run":
        run(out)
    else:
        print(json.dumps(score(out), indent=1))
