# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/autonomy-3158/B/rehearsal.py -- 3.158.B: `sheet.json`'s missions
through the brain's HTTP API on `tests/demo_explore.py`'s stack (its
exclusive desktop lock held for this process only), the cloud and VE stand-ins
answering from the simulator's report ($0, no network).

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    ROS_IMAGE=vision-picar-ros:plan-3.156 python evaluations/autonomy-3158/B/rehearsal.py OUT.jsonl

A batch: one stack, missions from where OUT.jsonl stopped, until `--batch-s`
has passed or a mission ends off the dock (only a fresh robot server puts the
robot back). Exit 0: the schedule is finished; 3: run another batch.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import httpx

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

os.environ.update(SIM_BATTERY_V="12.0", SIM_BATTERY_DRAIN_V_PER_S="0.0006",
                  SIM_BATTERY_CHARGE_V_PER_S="0.02", TIER_SEARCH="explore",
                  TIER_LOCAL_DIRECTION="false", METRICS_URL="", INVENTORY_BUCKET="")
for k in ("VISION_SHARED_SECRET", "VISION_SECRET", "RECHARGE_RESUME"):
    os.environ.pop(k, None)

from brain.local_vlm import model_size  # noqa: E402
from sim.grid_world import CELL_WALL  # noqa: E402
from sim.maps import build_world  # noqa: E402
from tests import demo_explore as de  # noqa: E402
from tests.demo_local_tier import free_port  # noqa: E402
from tests.drop_sweep import drop_truth  # noqa: E402
from tests.footprint_sweep import CELL_CM, chassis, gap, penetration, square  # noqa: E402

CELL_M = 0.3
FOUND_RADIUS_M = 0.9          # 3.80.G2's arrival at the true target
DOCK_M, DOCK_DEG = 0.10, 10.0  # 3.156's docked
LIMITS = {"find": 1200, "survey": 1800}
OVER_S = 600                  # past the limit, the return's own bound and more
START_V = 11.9                # the charge stand-in brings an idle robot back to this
CHARGE_WAIT_S = 240
HFOV = 60.0
VOLUME = "picar-maps-3158"
_run = subprocess.run


def _with_maps(cmd, **kw):
    """3.50.D's saved map: `stack()` has no volume of its own."""
    if isinstance(cmd, list) and cmd[:2] == ["docker", "run"]:
        cmd = cmd[:2] + ["-v", f"{VOLUME}:/maps"] + cmd[2:]
    return _run(cmd, **kw)


de.subprocess.run = _with_maps
ROBOT = None


def seen(target):
    """The target in the simulator's report now: (bearing_deg or None)."""
    for d in httpx.get(f"http://127.0.0.1:{de.ROBOT}/frame", timeout=10).json().get("detections") or ():
        if target.lower() in str(d.get("label", "")).lower():
            return float(d.get("bearing_deg") or 0.0)
    return None


class Standins(BaseHTTPRequestHandler):
    """VE's /identity and /health, and the cloud's /navigate, from the report."""

    calls = {"local": 0, "cloud": 0}

    def log_message(self, *a):
        pass

    def _send(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self._send({"service": "local_vlm", "ready": True, "alive": True, "engines": "standin (sim report)"})

    def do_POST(self):
        req = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
        path = self.path.rstrip("/")
        if path == "/identity":
            Standins.calls["local"] += 1
            b = seen(req.get("target") or "")
            if b is None:
                return self._send({"status": "answered", "score": 0.0, "parsed": True, "box": None, "ms": 1.0})
            w, h = model_size(640, 480)
            x = b / HFOV + 0.5
            return self._send({"status": "answered", "score": 0.95, "parsed": True, "ms": 1.0,
                               "box": [max(0.0, (x - 0.02) * w), 0.4 * h, min(w, (x + 0.02) * w), 0.6 * h]
                               if req.get("box") else None})
        if path == "/navigate":
            Standins.calls["cloud"] += 1
            v = seen(req.get("target_object") or "") is not None
            return self._send({"target_visible": v, "target_direction": "center" if v else "not_visible",
                               "target_reached": False, "obstacle_ahead": False, "room_guess": "unclear",
                               "action": "FORWARD" if v else "RIGHT", "reasoning": "stand-in (sim report)",
                               "distance_estimate": "unknown"})
        self._send({"status": "error", "score": None, "reason": "stand-in: not served"})


class Truth:
    """Contacts per incident, the stairwell and rooms entered, on the sim's truth."""

    def __init__(self, house, robot):
        self.world = build_world(house)
        self.walls = {(x, y) for y, row in enumerate(self.world.layout)
                      for x, c in enumerate(row) if c == CELL_WALL}
        self.room = {c: n for n, cs in self.world.rooms.items() for c in cs}
        self.robot = robot
        self.was_touching = False
        self.contacts = self.inside = self.samples = 0
        self.min_gap = math.inf
        self.fell, self.overhang = False, 0.0
        self.rooms = set()

    def sample(self):
        t = self.robot.get("/world/truth").json()
        objs = {(o["x"], o["y"]) for o in self.robot.get("/sim/objects").json()["objects"]}
        x, y = t["x_m"] / CELL_M, t["y_m"] / CELL_M
        a = chassis(x, y, math.radians(t["heading_deg"]) - math.pi / 2)
        touching = False
        for c in self.walls | objs:
            if abs(c[0] + 0.5 - x) > 1.6 or abs(c[1] + 0.5 - y) > 1.6:
                continue
            g = gap(a, square(*c)) * CELL_CM
            self.min_gap = min(self.min_gap, g)
            touching = touching or g < 1.0
            self.inside += penetration(a, square(*c)) > 0
        self.contacts += touching and not self.was_touching
        self.was_touching = touching
        self.samples += 1
        self.world.x, self.world.y, self.world.theta = x, y, math.radians(t["heading_deg"]) - math.pi / 2
        fell, over, _ = drop_truth(self.world)
        self.fell = self.fell or fell
        self.overhang = max(self.overhang, round(over, 2))
        r = self.room.get((int(x), int(y)))
        if r:
            self.rooms.add(r)
        return t


def volts(robot):
    return robot.get("/battery").json().get("v")


def target_m(pose, world, target):
    x, y = pose["x_m"] / CELL_M, pose["y_m"] / CELL_M
    cells = [c for c, n in world.objects.items() if n == target]
    return min(math.dist((x, y), (c[0] + 0.5, c[1] + 0.5)) for c in cells) * CELL_M if cells else None


def mission(robot, brain, house, m):
    limit = LIMITS[m["job"]]
    body = ({"job": "survey", "policy": "explore"} if m["job"] == "survey"
            else {"target_object": m["target"], "policy": "tiered"})
    t_wait = time.time()
    while True:
        r = brain.post("/mission/start", json=body)
        if r.status_code != 503 or time.time() - t_wait > 120:
            break
        time.sleep(3)
    if r.status_code != 200:
        return {"start": r.status_code, "detail": r.text[:300]}
    tr, t0, s, vmin = Truth(house, robot), time.time(), {}, math.inf
    start, pose = None, None
    v0 = volts(robot)
    while time.time() - t0 < limit + OVER_S:
        pose = tr.sample()
        start = start or pose
        v = volts(robot)
        vmin = min(vmin, v if v is not None else math.inf)
        s = brain.get("/mission/status").json()
        if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(0.5)
    else:
        brain.post("/mission/stop")
    secs = round(time.time() - t0)
    pose = tr.sample()
    err = math.hypot(pose["x_m"] - start["x_m"], pose["y_m"] - start["y_m"])
    deg = abs((pose["heading_deg"] - start["heading_deg"] + 180.0) % 360.0 - 180.0)
    ret = s.get("return") or {}
    dist = target_m(pose, tr.world, m["target"]) if m.get("target") else None
    stats = (s.get("tier") or {}).get("stats") or {}
    capped = secs >= limit + OVER_S
    return {"outcome": s.get("outcome"), "end_reason": (s.get("end_reason") or "")[:200],
            "error": s.get("error"), "capped": capped, "seconds": secs, "ret_state": ret.get("state"),
            "ret_reason": ret.get("reason"), "ret_seconds": ret.get("elapsed_s"),
            "dock_err_m": round(err, 3), "dock_err_deg": round(deg, 1),
            "docked": ret.get("state") == "docked" and err <= DOCK_M and deg <= DOCK_DEG,
            "end_target_m": None if dist is None else round(dist, 2),
            "contacts": tr.contacts, "inside_samples": tr.inside, "samples": tr.samples,
            "min_gap_cm": round(tr.min_gap, 1), "fell": tr.fell, "overhang_cm": tr.overhang,
            "rooms_truth": sorted(tr.rooms), "rooms_entered": ((s.get("survey") or {}).get("rooms_entered")),
            "v_start": v0, "v_end": volts(robot), "v_min": None if vmin == math.inf else vmin,
            "budget": s.get("budget"), "cloud_triggers": stats.get("triggers"),
            "local_calls": stats.get("local_calls"), "standin_calls": dict(Standins.calls),
            "start_truth": [round(start["x_m"], 3), round(start["y_m"], 3)],
            "log_tail": (s.get("log_tail") or [])[-3:], "run_dir": de.RUN_DIR}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--batch-s", type=float, default=4500.0)
    ap.add_argument("--limit", type=int, default=None, help="at most this many missions (a smoke test)")
    a = ap.parse_args()
    sheet = json.loads(Path(__file__).with_name("sheet.json").read_text())
    out = Path(a.out)
    done = [json.loads(x) for x in out.read_text().splitlines()] if out.exists() else []
    sched = sheet["schedule"]
    if a.limit is not None:
        sched = sched[:len(done) + a.limit]
    if len(done) >= len(sched):
        return 0
    if any(r["phase"] == "stretch" and r.get("stretch_ended") for r in done):
        return 0
    port = free_port()
    httpd = ThreadingHTTPServer(("127.0.0.1", port), Standins)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    os.environ.update(LOCAL_VLM_URL=f"http://127.0.0.1:{port}", VISION_URL=f"http://127.0.0.1:{port}")
    robot = de.stack(sheet["house"])
    brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
    t_batch = time.time()
    try:
        for i in range(len(done), len(sched)):
            t = time.time()
            while (volts(robot) or 0) < START_V and time.time() - t < CHARGE_WAIT_S:
                time.sleep(2)              # spaced for the charge (3.158.A, D)
            m = sched[i]
            load = os.getloadavg()[0]
            rec = {"i": i, **m, "load": round(load, 2), **mission(robot, brain, sheet["house"], m)}
            if m["phase"] == "stretch":
                rec["stretch_ended"] = bool(not rec.get("docked") or rec.get("outcome") == "failed"
                                            or rec.get("contacts") or rec.get("fell"))
            with out.open("a") as f:
                f.write(json.dumps(rec) + "\n")
            print(i, m.get("target") or m["job"], rec.get("outcome"), rec.get("ret_state"),
                  rec.get("seconds"), "contacts", rec.get("contacts"), flush=True)
            if rec.get("stretch_ended"):
                return 0
            if not rec.get("docked") or time.time() - t_batch >= a.batch_s:
                return 0 if i + 1 >= len(sched) else 3
    finally:
        de._kill_ports()
        os.system(f"docker logs {de.CONTAINER} > {de.RUN_DIR}/ros.log 2>&1; docker rm -f {de.CONTAINER} >/dev/null 2>&1")
        httpd.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
