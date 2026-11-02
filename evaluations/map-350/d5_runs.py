"""3.50.D5's live half: criteria 6 and 7, measured as the plan's "3.50.D5
live half: amendment 1 and the method" says (written before this ran).

    eval "$(python tools/plan_section.py env)"
    python evaluations/map-350/d5_runs.py [scaled_house home_first_floor] [--sessions N]

One chain per house:

  1      session 1 maps from an empty store (explore, an absent target,
         MAP_RUN_S); its inventory is saved on the map's frame
  R      a plain reload: a mission reads back the carried landmarks
         (criterion 7), judged on the truth; not one of criterion 6's ten
  2..10  reload, a person's re-map at the start, explore MAP_RUN_S; the
         mission's end saves the next version (criterion 6: graph size,
         /pose lag p90, container memory peak, reload time)
  F      the robot driven off the dock, the container restarted: a fall-back,
         and a mission there carries 0 landmarks (criterion 7)

The reload time's bar is the Jetson's (a separate bench); here it is recorded.
Maps and inventories stay under EXPLORE_LOGDIR (a floor plan of the SIM
house; only numbers are committed). Raw output: evaluations/map-350/D5/.
"""
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("d4_runs", os.path.join(HERE, "d4_runs.py"))
d4 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(d4)
d3, de = d4.d3, d4.de

OUT = os.path.join(HERE, "D5")
CELL = 0.3
SESSIONS = 10
READBACK_S = 60.0
CARRY_M = 0.15
MEM_EVERY_S = 5.0
GROWTH = 1.5
GRAPH_MB = 50.0
HOUSES = ("scaled_house", "home_first_floor")
# 3.50.D5's trial (the user, 2026-11-01): which arm, passed to the bridge.
# Every arm names both settings: since the trial the bridge's defaults are
# "both", so an arm that left one out would run it.
ARMS = {"base": {"PICAR_SLAM_LIFELONG": "0", "PICAR_MAP_CAP": "0"},
        "lifelong": {"PICAR_SLAM_LIFELONG": "1", "PICAR_MAP_CAP": "0"},
        "cap": {"PICAR_SLAM_LIFELONG": "0", "PICAR_MAP_CAP": "1.5"},
        "both": {"PICAR_SLAM_LIFELONG": "1", "PICAR_MAP_CAP": "1.5"}}
ARM = os.environ.get("D5_ARM", "base")
_network = d3._network


def _armed_network():
    """d3.container's docker flags, plus this arm's bridge settings."""
    return _network() + [a for k, v in ARMS[ARM].items() for a in ("-e", f"{k}={v}")]


d3._network = _armed_network
# A smoke run of the harness only (shorter missions; never a result).
SMOKE_S = int(os.environ.get("D5_SMOKE_S", "0"))


# ---------- what a session measures beside the mission ----------

def _ros(cmd):
    return ["docker", "exec", de.CONTAINER, "bash", "-c",
            "source /opt/ros/humble/setup.bash; source /underlay/install/setup.bash; "
            f"source /ws/install/setup.bash; export ROS_DOMAIN_ID={de.ROS_DOMAIN}; {cmd}"]


_MEM = re.compile(r"([\d.]+)\s*([KMG]i?B)")
_UNIT = {"KiB": 1 / 1024, "KB": 1 / 1024, "MiB": 1.0, "MB": 1.0, "GiB": 1024.0, "GB": 1024.0}


def _mib(text):
    m = _MEM.search(text)
    return float(m.group(1)) * _UNIT[m.group(2)] if m else None


class Probe:
    """Over one mission: the container's memory (docker stats, every 5 s)
    and every /pose's lag behind its scan (d5_rec_node.py in the container)."""

    def __init__(self, secs):
        self.secs, self.mem, self.lags, self.errors = secs, [], [], []
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._stats, daemon=True),
                         threading.Thread(target=self._rec, daemon=True)]

    def __enter__(self):
        for t in self._threads:
            t.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        for t in self._threads:
            t.join(self.secs + 120)

    def _stats(self):
        while not self._stop.is_set():
            r = subprocess.run(["docker", "stats", "--no-stream", "--format", "{{.MemUsage}}",
                                de.CONTAINER], capture_output=True, text=True)
            v = _mib(r.stdout)
            if v is not None:
                self.mem.append((round(time.time(), 1), round(v, 1)))
            self._stop.wait(MEM_EVERY_S)

    def _rec(self):
        try:
            subprocess.run(["docker", "cp", os.path.join(HERE, "d5_rec_node.py"),
                            f"{de.CONTAINER}:/tmp/d5_rec_node.py"], check=True, capture_output=True)
            r = subprocess.run(_ros(f"python3 /tmp/d5_rec_node.py {self.secs} /tmp/d5.json"),
                               capture_output=True, text=True)
            # The node prints only once its file is written: a non-zero exit
            # after that (rclpy's shutdown) loses nothing.
            if not any(ln.startswith("poses ") for ln in r.stdout.splitlines()):
                raise RuntimeError(f"recorder exited {r.returncode}: {r.stderr.strip()[-300:]}")
            got = subprocess.run(["docker", "exec", de.CONTAINER, "cat", "/tmp/d5.json"],
                                 capture_output=True, text=True, check=True).stdout
            self.lags = json.loads(got)["lags"]
        except Exception as e:  # noqa: BLE001 -- the session reports it
            self.errors.append(f"{type(e).__name__}: {e}")

    def summary(self):
        lags = sorted(x[1] for x in self.lags)
        mem = [m for _, m in self.mem]
        return {"poses": len(lags),
                "lag_p50_ms": round(1000 * lags[len(lags) // 2], 1) if lags else None,
                "lag_p90_ms": round(1000 * lags[min(len(lags) - 1, int(0.9 * len(lags)))], 1)
                if lags else None,
                "lag_max_ms": round(1000 * lags[-1], 1) if lags else None,
                "mem_peak_mib": max(mem) if mem else None,
                "mem_samples": len(mem), "errors": self.errors}


def graph_mb(maps):
    """The version current.json names: graph.posegraph + graph.data, in MB."""
    with open(os.path.join(maps, "current.json")) as f:
        cur = json.load(f)
    d = os.path.join(maps, cur["map_id"], f"v{cur['version']}")
    parts = {ext: os.path.getsize(os.path.join(d, "graph" + ext)) / 1e6
             for ext in (".posegraph", ".data")}
    return {"map_id": cur["map_id"], "version": cur["version"],
            "mb": round(sum(parts.values()), 2), "parts_mb": {k: round(v, 2) for k, v in parts.items()}}


def reload_s(watch):
    """`docker run` to `ready` (amendment 2): the watch's clock starts at the
    container's start, and `starting` is often over before the bridge first
    answers, so the stricter bound is the one always readable."""
    st = dict(watch["states"])
    return st.get("ready")


# ---------- criterion 7: landmarks against the truth ----------

def truth_dist(objects, label, x, y):
    """3.46's placement rule: to the nearest cell of an object with this label
    (0 inside it), in the house frame; inf when the house has none."""
    best = math.inf
    for o in objects:
        if str(o["name"]).strip().lower() != label:
            continue
        dx = max(o["x"] * CELL - x, 0.0, x - (o["x"] + 1) * CELL)
        dy = max(o["y"] * CELL - y, 0.0, y - (o["y"] + 1) * CELL)
        best = min(best, math.hypot(dx, dy))
    return best


def judge(objects, items):
    out = []
    for d in items:
        e = truth_dist(objects, d["label"], d["x_m"], d["y_m"])
        out.append({"label": d["label"], "x_m": d["x_m"], "y_m": d["y_m"], "reported": d["reported"],
                    "belief": d["belief"], "truth_m": None if math.isinf(e) else round(e, 3)})
    return out


def inventory(brain):
    return (brain.get("/mission/inventory").json() or {}).get("inventory") or {}


def readback(robot, brain):
    """A mission on the reloaded map: its inventory once it holds carried
    landmarks (at most READBACK_S), then stopped."""
    r = brain.post("/mission/start", json={"target_object": d3.ABSENT, "policy": "explore",
                                           "max_steps": 100000, "mission_time_limit_s": 600})
    if r.status_code != 200:
        return {"start": r.status_code, "detail": r.text[:300]}
    t0, inv = time.time(), {}
    while time.time() - t0 < READBACK_S:
        inv = inventory(brain)
        if inv.get("carried_from"):
            break
        time.sleep(1.0)
    waited = round(time.time() - t0, 1)
    brain.post("/mission/stop")
    time.sleep(2.0)
    items = inv.get("reported", []) + inv.get("candidates", [])
    carried = [d for d in items if d.get("carried")]
    objects = robot.get("/sim/objects").json()["objects"]
    return {"waited_s": waited, "carried_from": inv.get("carried_from"),
            "frame_id": inv.get("frame_id"), "carried": judge(objects, carried),
            "own": len(items) - len(carried)}


def saved_inventory(inv_dir):
    """The newest local report (session 1's, before the read-back)."""
    files = sorted((os.path.join(inv_dir, f) for f in os.listdir(inv_dir) if f.endswith(".json")),
                   key=os.path.getmtime)
    if not files:
        return None
    with open(files[-1]) as f:
        return json.load(f)


# ---------- one chain ----------

def chain(house, n_sessions):
    maps = os.path.join(de.RUN_DIR, f"maps-{house}")
    inv_dir = os.path.join(de.RUN_DIR, f"inventory-{house}")
    os.makedirs(inv_dir, exist_ok=True)
    os.environ["INVENTORY_DIR"] = inv_dir
    os.environ["INVENTORY_BUCKET"] = ""          # local only (3.55)
    secs = SMOKE_S or d4.MAP_RUN_S[house]
    out = {"house": house, "image": d3.IMAGE, "mission_s": secs, "sessions": []}

    # Session 1: the map run, probed like the others.
    os.makedirs(maps, exist_ok=True)
    bridge = d4.httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
    robot, brain = d4.servers(house)
    w = d3.container(maps, bridge)
    de.MISSION_LIMIT_S = secs
    with Probe(secs + 30) as p, d4.Sampler(robot, bridge, house) as s:
        mission = de.run_mission(robot, house, policy="explore", target=d3.ABSENT, max_steps=100000)
    t = time.time()
    while time.time() - t < 30 and d4.health(bridge)["map"]["saved"] < 1:
        time.sleep(0.3)
    time.sleep(3.0)                              # the inventory's async save
    s1 = {"n": 1, "watch": {k: v for k, v in w.items() if k != "health"},
          "reload_s": reload_s(w), "reload": w["health"]["slam"]["reload"],
          "mission": mission, "saved": d4.health(bridge)["map"], "probe": p.summary(),
          "sampler": s.summary()}
    s1["graph"] = graph_mb(maps)
    sl = d4.health(bridge)["slam"]
    out["bridge_settings"] = {"lifelong": sl.get("lifelong"), "map_cap": sl.get("map_cap"),
                              "mode_node": sl.get("mode"),
                              # which executable really maps (the arm's check)
                              "lifelong_running": subprocess.run(
                                  ["docker", "exec", de.CONTAINER, "pgrep", "-f",
                                   "lifelong_slam_toolbox_node"], capture_output=True).returncode == 0}
    saved = saved_inventory(inv_dir)
    s1["inventory"] = {"frame_id": (saved or {}).get("frame_id"),
                       "reported": len((saved or {}).get("reported", [])),
                       "candidates": len((saved or {}).get("candidates", []))}
    out["sessions"].append(s1)
    saved_id = s1["graph"]["map_id"]
    objects = robot.get("/sim/objects").json()["objects"]
    out["session1_placement"] = judge(objects, (saved or {}).get("reported", [])
                                      + (saved or {}).get("candidates", []))
    print(f"    session 1: {s1['graph']['mb']} MB, lag p90 {s1['probe']['lag_p90_ms']} ms, "
          f"mem {s1['probe']['mem_peak_mib']} MiB, {s1['inventory']}", flush=True)

    # R: the plain reload's read-back (criterion 7).
    robot, brain, bridge, rs = d4.reload(house, maps)
    rs["reload_s"] = reload_s(rs["watch"])
    if rs["reload"].get("result") == "matched":
        rs["readback"] = readback(robot, brain)
    out["readback"] = rs
    print(f"    read-back: {rs['reload'].get('result')}, "
          f"{len((rs.get('readback') or {}).get('carried', []))} carried", flush=True)

    # Sessions 2..N: re-mapped at the start (D4-c's procedure).
    for i in range(2, n_sessions + 1):
        robot, brain, bridge, sess = d4.reload(house, maps)
        sess["n"], sess["reload_s"] = i, reload_s(sess["watch"])
        capped = sess["reload"].get("result") == "capped"
        if sess["reload"].get("result") != "matched" and not capped:
            sess["error"] = "reload did not match"
            out["sessions"].append(sess)
            print(f"    session {i}: reload {sess['reload'].get('result')}", flush=True)
            continue
        with Probe(secs + 40) as p, d4.Sampler(robot, bridge, house) as s:
            if capped:                           # a fresh map from the dock: nothing to re-map
                sess["remap"] = {"result": "capped"}
            else:
                r = robot.post("/world/remap", headers={"x-driver": "twin-dpad"})
                sess["remap_request"] = {"code": r.status_code, "body": r.json()}
                t = time.time()
                while time.time() - t < 20 and d4.remap_rec(bridge).get("result") not in (
                        "mapping", "failed", "failed_localising", "deferred"):
                    time.sleep(0.3)
                sess["remap"] = d4.remap_rec(bridge)
            m0 = d4.health(bridge)["map"]
            ends_before = m0["saved"] + m0["skipped"] + m0["failed"]
            de.MISSION_LIMIT_S = secs
            sess["mission"] = de.run_mission(robot, house, policy="explore", target=d3.ABSENT,
                                             max_steps=100000)
        t = time.time()
        while time.time() - t < 30:
            m1 = d4.health(bridge)["map"]
            if m1["saved"] + m1["skipped"] + m1["failed"] > ends_before:
                break
            time.sleep(0.3)
        sess["saved"] = d4.health(bridge)["map"]
        sess["probe"], sess["sampler"] = p.summary(), s.summary()
        sess["graph"] = graph_mb(maps)
        out["sessions"].append(sess)
        print(f"    session {i}: {sess['graph']['mb']} MB v{sess['graph']['version']}, "
              f"lag p90 {sess['probe']['lag_p90_ms']} ms, mem {sess['probe']['mem_peak_mib']} MiB, "
              f"reload {sess['reload_s']} s, remap {sess['remap'].get('result')}", flush=True)

    # F: off the dock, a fall-back; a mission there carries nothing.
    robot, brain, bridge, fs = d4.reload(house, maps)
    if fs["reload"].get("result") == "matched":
        dock = d3.truth(robot)
        for _ in range(3):
            x, y, h = d3.truth(robot)
            if math.hypot(x - dock[0], y - dock[1]) >= 1.0 and abs((h - dock[2] + 180) % 360 - 180) >= 30:
                break
            d3.drive(robot, 1.0, 60)
        x, y, h = d3.truth(robot)
        fs["off_dock"] = {"from_dock_m": round(math.hypot(x - dock[0], y - dock[1]), 2),
                          "from_dock_deg": round(abs((h - dock[2] + 180) % 360 - 180), 1)}
        w = d3.container(maps, bridge)
        fs["after"] = {"reload": w["health"]["slam"]["reload"], "map_id": w["health"]["map"]["map_id"]}
        fs["readback"] = readback(robot, brain)
    out["fall_back"] = fs
    out["criteria"], out["verdict"] = verdicts(out, saved_id)
    return out


def verdicts(out, saved_id):
    ss = [s for s in out["sessions"] if s.get("probe")]
    first, last = ss[0], ss[-1]
    c = {}
    c["ten_sessions"] = len(ss) == SESSIONS and all(
        (s.get("remap") or {}).get("result") in ("mapping", "capped") for s in ss[1:])
    c["capped_sessions"] = [s["n"] for s in ss if (s.get("remap") or {}).get("result") == "capped"]
    c["graph_mb_max"] = max(s["graph"]["mb"] for s in ss)
    c["lag_p90_ms_max"] = max((s["probe"]["lag_p90_ms"] or 0) for s in ss)
    c["mem_peak_mib_max"] = max((s["probe"]["mem_peak_mib"] or 0) for s in ss)
    lag1, lagN = first["probe"]["lag_p90_ms"], last["probe"]["lag_p90_ms"]
    mem1, memN = first["probe"]["mem_peak_mib"], last["probe"]["mem_peak_mib"]
    c["lag_ratio"] = round(lagN / lag1, 3) if lag1 and lagN else None
    c["mem_ratio"] = round(memN / mem1, 3) if mem1 and memN else None
    c["graph_ratio"] = round(last["graph"]["mb"] / first["graph"]["mb"], 3)
    c["graph_mb"] = last["graph"]["mb"]
    c["reload_s_max_laptop"] = max((s["reload_s"] for s in out["sessions"][1:] if s.get("reload_s")),
                                   default=None)
    c["contacts"] = sum(s["sampler"]["contacts"] for s in ss)
    c["jumps"] = sum(len(s["sampler"]["jumps"]) for s in ss)
    graph_ok = (c["graph_mb"] <= GRAPH_MB if out["house"] == "scaled_house"
                else c["graph_ratio"] <= GROWTH)
    six = (c["ten_sessions"] and c["lag_ratio"] is not None and c["lag_ratio"] <= GROWTH
           and c["mem_ratio"] is not None and c["mem_ratio"] <= GROWTH and graph_ok)
    rb = out["readback"].get("readback") or {}
    rep = [d for d in rb.get("carried", []) if d["reported"]]
    c["carried"] = len(rb.get("carried", []))
    c["carried_reported"] = len(rep)
    c["carried_reported_within"] = sum(d["truth_m"] is not None and d["truth_m"] <= CARRY_M for d in rep)
    c["carried_same_frame"] = rb.get("frame_id") == saved_id
    fb = out["fall_back"]
    fb_rb = fb.get("readback") or {}
    c["fell_back"] = (fb.get("after") or {}).get("reload", {}).get("result") == "fell_back"
    c["fall_back_carried"] = len(fb_rb.get("carried", [])) if fb_rb else None
    seven = (c["carried"] > 0 and c["carried_reported"] > 0
             and c["carried_reported_within"] == c["carried_reported"] and c["carried_same_frame"]
             and c["fell_back"] and c["fall_back_carried"] == 0 and fb_rb.get("carried_from") is None)
    return c, {"6 (laptop part; reload's bar on the Jetson)": "MET" if six else "FAILED",
               "7": "MET" if seven else "FAILED"}


def main(argv):
    global SESSIONS
    if "--sessions" in argv:
        SESSIONS = int(argv[argv.index("--sessions") + 1])
    houses = [a for a in argv if a in HOUSES] or list(HOUSES)
    d4.check_image()
    os.makedirs(OUT, exist_ok=True)
    de.heavy_lock()
    de._kill_ports()
    de._new_run_dir()
    print(f"run dir {de.RUN_DIR}", flush=True)
    for house in houses:
        print(f"--- {house} {time.strftime('%H:%M:%S')}", flush=True)
        try:
            res = chain(house, SESSIONS)
        except Exception as e:  # noqa: BLE001 -- recorded, the next house goes on
            import traceback
            res = {"house": house, "error": f"{type(e).__name__}: {e}",
                   "trace": traceback.format_exc()[-2000:]}
        res["sessions_asked"], res["arm"] = SESSIONS, ARM
        want = ARMS[ARM]
        got = res.get("bridge_settings") or {}
        # The bridge's own reading, not the shell's: an arm it did not run is no result.
        lifelong = want["PICAR_SLAM_LIFELONG"] == "1"
        if res.get("verdict") and (got.get("lifelong") != lifelong
                                   or got.get("lifelong_running") != lifelong
                                   or got.get("map_cap") != float(want["PICAR_MAP_CAP"])):
            res["verdict"] = {k: f"INVALID (bridge read {got}; arm {ARM})" for k in res["verdict"]}
        name = f"{house}{'' if ARM == 'base' else f'-{ARM}'}{'' if SESSIONS == 10 else f'-n{SESSIONS}'}.json"
        out_dir = os.path.join(OUT, "smoke") if SMOKE_S else OUT
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, name), "w") as f:
            json.dump(dict(res, smoke_s=SMOKE_S or None), f, indent=1, default=str)
        print(f"    {res.get('verdict') or res.get('error')}", flush=True)
        print(f"    {res.get('criteria')}", flush=True)
    d4.subprocess_cleanup()


if __name__ == "__main__":
    main(sys.argv[1:])
