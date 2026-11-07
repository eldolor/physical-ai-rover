# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.50.D5: criteria 6 (growth over ten sessions; the desktop half) and 7
(sightings carry over, live), on the criteria and the measurement rules
written in the plan ("3.50.D5 live: how it is measured") before this ran.

Per house, on one host directory at /maps and one inventory directory:

  s1       an explore mission for an absent target on an empty store, stopped
           at its time limit; the bridge saves v1 at its end, the brain the
           mission's inventory (its landmarks are session 1's sightings)
  s2..s10  a session each: robot server, brain and container restarted (the
           robot on the dock), the reload judged, then the same mission for
           a shorter time limit. Session 2's inventory is read as soon as it
           has seeded (criterion 7); every session records the saved graph's
           size, the container's memory and slam_toolbox's CPU (criterion 6)
  fb       off the dock (>= 1 m, >= 30 deg), only the container restarted:
           the reload must fall back, and a mission on it carries nothing

The maps and inventories are a SIM house's, kept under EXPLORE_LOGDIR,
never committed (only numbers are). Raw output: evaluations/map-350/D5/.

    eval "$(python tools/plan_section.py env 3.50)"
    python evaluations/map-350/d5_runs.py [scaled_house home_first_floor] [--sessions N]
"""
import json
import math
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import httpx  # noqa: E402

import d3_runs as d3  # noqa: E402
from d3_runs import de  # noqa: E402

OUT = os.path.join(HERE, "D5")
ABSENT = d3.ABSENT
FIRST_S = {"scaled_house": 420, "home_first_floor": 900}   # D3's map runs
LATER_S = {"scaled_house": 240, "home_first_floor": 300}
CELL_M = 0.3
CARRY_BAR_M = 0.15          # criterion 7
ASSOC_M = 1.0               # 3.46 criterion 4's "a true instance" radius
GROWTH_X, GRAPH_MB = 1.5, 50.0   # criterion 6
SAMPLE_S = 5.0
if os.environ.get("D5_SMOKE") == "1":        # a harness check only: never a result
    FIRST_S = {h: 90 for h in FIRST_S}
    LATER_S = {h: 60 for h in LATER_S}
    OUT = os.path.join(HERE, "D5", "smoke")


def servers(house, inv_dir):
    os.environ["INVENTORY_DIR"] = inv_dir
    os.environ["INVENTORY_BUCKET"] = ""       # local only, never S3
    return d3.servers(house)


def graph_mb(maps):
    """The current saved version's graph files (what a reload loads), in MB."""
    try:
        cur = json.load(open(os.path.join(maps, "current.json")))
    except (OSError, ValueError):
        return None, None
    d = os.path.join(maps, cur["map_id"], f"v{cur['version']}")
    total = sum(os.path.getsize(os.path.join(d, f)) for f in os.listdir(d)
                if f.endswith((".posegraph", ".data")))
    return cur, round(total / 1e6, 2)


def store_mb(maps):
    total = sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(maps) for f in fs)
    return round(total / 1e6, 2)


def _slam_pid():
    r = subprocess.run(["docker", "top", de.CONTAINER, "-eo", "pid,args"],
                       capture_output=True, text=True)
    for line in r.stdout.splitlines()[1:]:
        pid, _, args = line.strip().partition(" ")
        if "slam_toolbox_node" in args and "ros2" not in args.split()[0]:
            return int(pid), args.split()[0].rsplit("/", 1)[-1]
    return None, None


def _cpu_ticks(pid):
    with open(f"/proc/{pid}/stat") as f:
        parts = f.read().rsplit(")", 1)[1].split()
    return int(parts[11]) + int(parts[12])          # utime + stime


def _mem_mb():
    r = subprocess.run(["docker", "stats", "--no-stream", "--format", "{{.MemUsage}}",
                        de.CONTAINER], capture_output=True, text=True)
    used = r.stdout.split("/")[0].strip()
    for unit, k in (("GiB", 1024.0), ("MiB", 1.0), ("KiB", 1 / 1024.0), ("B", 1 / 1048576.0)):
        if used.endswith(unit):
            return float(used[:-len(unit)]) * k
    return None


class Monitor(threading.Thread):
    """Every SAMPLE_S: the container's memory, and slam_toolbox's CPU share
    over the window (a window across a node restart is dropped)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.stop_ev = threading.Event()
        self.mem, self.cpu, self.nodes = [], [], set()

    def run(self):
        hz = os.sysconf("SC_CLK_TCK")
        last = None
        while not self.stop_ev.wait(SAMPLE_S):
            m = _mem_mb()
            if m is not None:
                self.mem.append(round(m, 1))
            pid, node = _slam_pid()
            if pid is None:
                last = None
                continue
            self.nodes.add(node)
            try:
                now = (pid, _cpu_ticks(pid), time.monotonic())
            except OSError:
                last = None
                continue
            if last and last[0] == pid:
                self.cpu.append(round((now[1] - last[1]) / hz / (now[2] - last[2]), 4))
            last = now

    def result(self):
        self.stop_ev.set()
        self.join(timeout=30)
        return {"mem_mb_max": max(self.mem, default=None), "mem_mb_p50": _pct(self.mem, 50),
                "slam_cpu_p90": _pct(self.cpu, 90), "slam_cpu_p50": _pct(self.cpu, 50),
                "windows": len(self.cpu), "nodes": sorted(self.nodes)}


def _pct(xs, q):
    if not xs:
        return None
    s = sorted(xs)
    return s[min(len(s) - 1, int(math.ceil(q / 100 * len(s))) - 1)]


class SeedWatch(threading.Thread):
    """Polls GET /mission/inventory until the mission has seeded (or 120 s):
    the carried landmarks before any of this session's sightings merge in."""

    def __init__(self, brain):
        super().__init__(daemon=True)
        self.brain, self.snap, self.stop_ev = brain, None, threading.Event()

    def run(self):
        t0 = time.time()
        while not self.stop_ev.wait(1.0) and time.time() - t0 < 120:
            try:
                inv = self.brain.get("/mission/inventory").json().get("inventory")
            except Exception:  # noqa: BLE001 -- the mission may not be up yet
                continue
            if inv and (inv.get("frame_id") or inv["counts"]["carried"]):
                self.snap = dict(inv, at_s=round(time.time() - t0, 1))
                if inv["counts"]["carried"]:
                    return


def footprint_m(x, y, cells):
    """Distance from (x, y) to the nearest of the object's cells (0 inside)."""
    best = math.inf
    for cx, cy in cells:
        dx = max(cx * CELL_M - x, 0.0, x - (cx + 1) * CELL_M)
        dy = max(cy * CELL_M - y, 0.0, y - (cy + 1) * CELL_M)
        best = min(best, math.hypot(dx, dy))
    return best


def carry_check(landmarks, objects):
    """Criterion 7 on each carried landmark: the nearest true instance of
    its label (cells grouped by name), as footprint and centre distances."""
    cells = {}
    for o in objects:
        cells.setdefault(o["name"].strip().lower(), []).append((o["x"], o["y"]))
    rows = []
    for lm in landmarks:
        if not lm.get("carried"):
            continue
        cs = cells.get(lm["label"], [])
        fp = footprint_m(lm["x_m"], lm["y_m"], cs) if cs else None
        centre = min((math.hypot((cx + .5) * CELL_M - lm["x_m"], (cy + .5) * CELL_M - lm["y_m"])
                      for cx, cy in cs), default=None)
        rows.append({"label": lm["label"], "x_m": lm["x_m"], "y_m": lm["y_m"],
                     "belief": lm["belief"], "footprint_m": None if fp is None else round(fp, 3),
                     "centre_m": None if centre is None else round(centre, 3),
                     "true_instance": fp is not None and fp <= ASSOC_M})
    return rows


def session(i, house, maps, inv_dir, bridge, saved_id, limit_s):
    robot, brain = servers(house, inv_dir)
    w = d3.container(maps, bridge)
    rel = w["health"]["slam"]["reload"]
    out = {"i": i, "reload": rel, "map_id": w["health"]["map"]["map_id"],
           "states": w["states"]}
    st = dict(w["states"])
    out["reload_s"] = (round(st["ready"] - st["checking"], 2)
                       if "ready" in st and "checking" in st else None)
    if saved_id is not None and rel.get("result") != "matched":
        out["error"] = "reload did not match"
        return out, robot
    mon, seed = Monitor(), SeedWatch(brain)
    mon.start()
    seed.start()
    de.MISSION_LIMIT_S = limit_s
    out["mission"] = de.run_mission(robot, house, policy="explore", target=ABSENT,
                                    max_steps=100000)
    seed.stop_ev.set()
    t_end = time.time()
    saved0 = None
    while time.time() - t_end < 20:
        h = bridge.get("/health").json()
        saved0 = h["map"]
        if not h["map"].get("saving") and (saved_id is not None or h["map"]["saved"] >= 1):
            break
        time.sleep(0.5)
    time.sleep(3)                             # the inventory's save thread
    out["monitor"] = mon.result()
    out["map_health"] = saved0
    out["slam"] = {k: bridge.get("/health").json()["slam"].get(k)
                   for k in ("state", "mode", "starts", "remap")}
    out["graph"], out["graph_mb"] = graph_mb(maps)
    out["store_mb"] = store_mb(maps)
    inv = brain.get("/mission/inventory").json().get("inventory") or {}
    out["inventory"] = {k: inv.get(k) for k in ("map_id", "frame_id", "carried_from",
                                                "carry_seq", "counts")}
    objects = [o for o in robot.get("/sim/objects").json()["objects"] if not o["mover"]]
    if seed.snap:
        out["seeded"] = {k: seed.snap.get(k) for k in ("frame_id", "carried_from", "carry_seq",
                                                       "counts", "at_s")}
        out["carried_at_seed"] = carry_check(seed.snap["reported"] + seed.snap["candidates"],
                                             objects)
    out["carried_at_end"] = carry_check(inv.get("reported", []) + inv.get("candidates", []),
                                        objects)
    out["final_error"] = d3.error(robot)
    return out, robot


def fall_back(house, maps, inv_dir, bridge, saved_id):
    """Criterion 7's second half: off the dock, a fall-back carries nothing."""
    robot, brain = servers(house, inv_dir)
    dock = d3.truth(robot)
    for _ in range(4):
        x, y, h = d3.truth(robot)
        if math.hypot(x - dock[0], y - dock[1]) >= 1.0 and abs((h - dock[2] + 180) % 360 - 180) >= 30:
            break
        d3.drive(robot, 1.0, 60)
    x, y, h = d3.truth(robot)
    out = {"from_dock_m": round(math.hypot(x - dock[0], y - dock[1]), 2),
           "from_dock_deg": round(abs((h - dock[2] + 180) % 360 - 180), 1)}
    w = d3.container(maps, bridge)
    out["reload"] = w["health"]["slam"]["reload"]
    out["map_id"] = w["health"]["map"]["map_id"]
    seed = SeedWatch(brain)
    seed.start()
    de.MISSION_LIMIT_S = 60
    out["mission"] = de.run_mission(robot, house, policy="explore", target=ABSENT,
                                    max_steps=100000)
    seed.stop_ev.set()
    time.sleep(3)
    inv = brain.get("/mission/inventory").json().get("inventory") or {}
    out["inventory"] = {k: inv.get(k) for k in ("map_id", "frame_id", "carried_from",
                                                "carry_seq", "counts")}
    out["old_landmarks_used"] = sum(1 for lm in inv.get("reported", []) + inv.get("candidates", [])
                                    if lm.get("carried"))
    out["ok"] = (out["reload"].get("result") == "fell_back" and out["map_id"] != saved_id
                 and out["old_landmarks_used"] == 0 and not (inv.get("counts") or {}).get("carried"))
    return out


def verdicts(res):
    s = res["sessions"]
    s1 = s[0]
    v = {}
    s2 = s[1] if len(s) > 1 else {}
    rows = s2.get("carried_at_seed") or []
    true_rows = [r for r in rows if r["true_instance"]]
    v["7-carry"] = bool(true_rows) and all(r["footprint_m"] <= CARRY_BAR_M for r in true_rows)
    v["7-fallback"] = bool(res.get("fall_back", {}).get("ok"))
    last = s[-1]
    ok = len(s) >= 10 and all(x.get("monitor") for x in s)
    if ok:
        base_cpu, base_mem = s1["monitor"]["slam_cpu_p90"], s1["monitor"]["mem_mb_max"]
        v["6-scan"] = all(x["monitor"]["slam_cpu_p90"] is not None
                          and x["monitor"]["slam_cpu_p90"] <= GROWTH_X * base_cpu for x in s)
        v["6-memory"] = all(x["monitor"]["mem_mb_max"] <= GROWTH_X * base_mem for x in s)
        v["6-graph"] = last["graph_mb"] is not None and last["graph_mb"] <= GRAPH_MB
    return {k: "MET" if x else "FAILED" for k, x in v.items()}


def house_runs(house, n):
    res = {"house": house, "image": d3.IMAGE, "sessions": []}
    maps = os.path.join(de.RUN_DIR, f"maps-{house}")
    inv_dir = os.path.join(de.RUN_DIR, f"inventory-{house}")
    os.makedirs(maps, exist_ok=True)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
    s, _ = session(1, house, maps, inv_dir, bridge, None, FIRST_S[house])
    res["sessions"].append(s)
    saved_id = s["map_health"]["map_id"] if s.get("map_health", {}).get("saved") else None
    assert saved_id, f"session 1 saved nothing: {s.get('map_health')}"
    for i in range(2, n + 1):
        s, _ = session(i, house, maps, inv_dir, bridge, saved_id, LATER_S[house])
        res["sessions"].append(s)
        print(house, i, json.dumps({k: s.get(k) for k in ("reload_s", "graph_mb", "store_mb")}),
              json.dumps(s.get("monitor")), flush=True)
        _dump(house, res)
    res["fall_back"] = fall_back(house, maps, inv_dir, bridge, saved_id)
    res["verdicts"] = verdicts(res)
    return res


def _dump(house, res):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{house}.json"), "w") as f:
        json.dump(res, f, indent=1)


def main(argv):
    n = 10
    if "--sessions" in argv:
        k = argv.index("--sessions")
        n = int(argv[k + 1])
        argv = argv[:k] + argv[k + 2:]
    houses = argv or ["scaled_house", "home_first_floor"]
    de.heavy_lock()
    de._kill_ports()
    de._new_run_dir()
    print("run dir", de.RUN_DIR, flush=True)
    for house in houses:
        res = house_runs(house, n)
        res["sessions_asked"] = n
        _dump(house, res)
        print(house, json.dumps(res["verdicts"]), flush=True)
    subprocess.run(["docker", "logs", de.CONTAINER], stdout=open(f"{de.RUN_DIR}/ros-last.log", "w"),
                   stderr=subprocess.STDOUT)
    subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
    de._kill_ports()


if __name__ == "__main__":
    main(sys.argv[1:])
