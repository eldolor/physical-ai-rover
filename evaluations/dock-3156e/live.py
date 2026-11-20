# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/dock-3156e/live.py -- 3.156.E's live bars G1, G3, G4 (live half)
and G5 on `tests/demo_explore.py`'s stack (robot server, brain, the ROS
container with slam_toolbox and nav2), missions through the brain's HTTP API,
judged on the simulator's ground truth. B's `live.py` is copied in spirit,
not edited; the cloud and VE tiers are 3.158's stand-ins answering from the
simulator's report ($0, no network).

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    python evaluations/dock-3156e/live.py OUT.jsonl

(`demo_explore.stack()` takes the desktop's exclusive lock itself.)

Per house, two stacks, both a fresh robot server (the robot at the house's
start: the dock) and a fresh container with the saved map in this step's
own volume (`picar-maps-3156e`, removed at the start and the end; a map is a
floor plan and is never committed):

1. `map`: one explore survey from the dock; its end saves the map (3.50).
2. `reload`: the container reloads it at the dock (3.50.D3). Then each
   mission: a person's nav2 goal (`x-driver: twin-dpad`) moves the robot
   1-4 m off the dock, its authority lapses, and a `tiered` find with
   `tier_search: cloud` starts there (it does not explore). Recorded:
   `status.dock` and its source (G1), the return's end and the true pose
   error at the end against the dock's ground truth (the map session's
   start: G1, G3), contacts on the chassis footprint (G3), the nav2 goals
   sent before the end decision (G4: the robot server's access log between
   the mission's start and end markers, plus every goal id seen polling
   `/world/goal`), and `autonomy_lapsed` (G5). `drill` missions: once the
   return is driving, a person's Stop (`POST /stop`, no driver); the return
   `cancelled`, true displacement over the next 10 s (G5).

scaled_house's reload session runs `FINDS` finds (G1, and half of G3) then
`DRILLS` drills (G5); home_first_floor's runs `FINDS` finds (G3's other half).

The supplementary G4 run (E's live results): `LIVE_FIND_LIMIT_S=300
LIVE_FINDS=3 LIVE_DRILLS=0`. G5's re-run: `LIVE_HOUSES=scaled_house
LIVE_FINDS=0 LIVE_DRILLS=5 LIVE_SEED=2` (the Stop once the return has moved
10 cm or turned 15 degrees).
"""

import json
import math
import os
import random
import subprocess
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# No paid call and no upload can leave this run: the cloud tier is a local
# stand-in, and every credential a child process could reach is dropped.
for k in list(os.environ):
    if k.startswith(("ANTHROPIC_", "AWS_")) or k in (
            "APP_SHARED_SECRET", "LOCAL_SECRET", "VISION_SECRET", "VISION_SHARED_SECRET",
            "RECHARGE_RESUME"):
        os.environ.pop(k)
os.environ.update(TIER_SEARCH="cloud", METRICS_URL="", INVENTORY_BUCKET="",
                  EXPLORE_ROBOT_ACCESS_LOG="1")

import base64  # noqa: E402
import io  # noqa: E402
from http.server import BaseHTTPRequestHandler  # noqa: E402

import httpx  # noqa: E402
from PIL import Image  # noqa: E402

from brain.local_vlm import model_size  # noqa: E402
from sim.grid_world import CELL_WALL  # noqa: E402
from sim.maps import build_world  # noqa: E402
from tests import demo_explore as de  # noqa: E402
from tests.demo_local_tier import free_port  # noqa: E402
from tests.footprint_sweep import CELL_CM, chassis, gap, square  # noqa: E402

VOLUME = "picar-maps-3156e"
MAP_LIMIT_S = 420           # the map session's survey
FIND_LIMIT_S = int(os.environ.get("LIVE_FIND_LIMIT_S", 90))   # a find's limit: it ends, then goes home
WALL_S = 600                # one mission's wall-clock cap (the limit, the return and more)
FINDS = int(os.environ.get("LIVE_FINDS", 5))
DRILLS = int(os.environ.get("LIVE_DRILLS", 5))
OFF_M = (1.0, 4.0)          # G1: a person's goal first moves the robot 1-4 m off the dock
# Another run's starts: an offset added to the kind's seed (finds 0, drills 1),
# so LIVE_SEED=1 would repeat the default drills' cells as finds. The G5
# re-run used 2 (drills from seed 3, cells no earlier run used).
SEED = int(os.environ.get("LIVE_SEED", 0))
STOP_WHEN_M, STOP_WHEN_DEG = 0.10, 15.0      # G5: a Stop only once the return truly moves
DOCK_M, DOCK_DEG = 0.15, 15.0   # C2's every-run bars, as G1 and G3 name them
_run = subprocess.run


def _with_maps(cmd, **kw):
    """3.50.D's saved map: `stack()` mounts no volume of its own."""
    if isinstance(cmd, list) and cmd[:2] == ["docker", "run"]:
        cmd = cmd[:2] + ["-v", f"{VOLUME}:/maps"] + cmd[2:]
    return _run(cmd, **kw)


de.subprocess.run = _with_maps


HFOV = 60.0


def seen(target):
    """The target in the simulator's report now: its bearing, or None."""
    try:
        report = httpx.get(f"http://127.0.0.1:{de.ROBOT}/frame", timeout=10).json().get("detections") or ()
    except (httpx.HTTPError, ValueError):
        return None
    for d in report:
        if target.lower() in str(d.get("label", "")).lower():
            return float(d.get("bearing_deg") or 0.0)
    return None


class Standins(BaseHTTPRequestHandler):
    """3.158.B's stand-ins, copied: VE's /identity and the cloud's /navigate,
    answered from the simulator's report ($0, no network)."""

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
            w, h = model_size(*Image.open(io.BytesIO(base64.b64decode(req["image_b64"]))).size)
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


def post_ok(client, path, **kw):
    """A POST that records a failure instead of ending the run."""
    try:
        client.post(path, **kw)
        return None
    except httpx.HTTPError as e:
        return f"{type(e).__name__}: {e}"[:200]


def get_json(client, path, tries=4):
    """One reply's JSON, riding out a hiccup (a timeout, a plain-text 503):
    one bad sample must not end an hour's run (review, 2026-11-20)."""
    for i in range(tries):
        try:
            r = client.get(path)
            r.raise_for_status()           # a 503's JSON body is not a sample
            return r.json()
        except (httpx.HTTPError, ValueError):
            if i == tries - 1:
                raise
            time.sleep(0.5)


class Truth:
    def __init__(self, robot, house):
        self.robot = robot
        w = build_world(house)
        self.walls = {(x, y) for y, row in enumerate(w.layout) for x, c in enumerate(row)
                      if c == CELL_WALL}

    def pose(self):
        t = get_json(self.robot, "/world/truth")
        return t["x_m"], t["y_m"], t["heading_deg"]

    def gap_cm(self):
        x, y, h = self.pose()
        cx, cy = x / 0.3, y / 0.3
        objs = {(o["x"], o["y"]) for o in get_json(self.robot, "/sim/objects")["objects"]}
        A = chassis(cx, cy, math.radians(h) - math.pi / 2)
        g = math.inf
        for c in self.walls | objs:
            if abs(c[0] + 0.5 - cx) > 1.6 or abs(c[1] + 0.5 - cy) > 1.6:
                continue
            g = min(g, gap(A, square(*c)) * CELL_CM)
        return g


def off_dock_cells(house, dock, n, seed):
    """Reachable cells with a cell of clearance (nav2's inflation), 1-4 m from the dock."""
    w = build_world(house)
    reach = de.reachable_cells(w)
    ok = [c for c in reach
          if all((c[0] + dx, c[1] + dy) in reach for dx in (-1, 0, 1) for dy in (-1, 0, 1))
          and OFF_M[0] <= math.hypot((c[0] + 0.5) * 0.3 - dock[0], (c[1] + 0.5) * 0.3 - dock[1]) <= OFF_M[1]]
    rng = random.Random(f"dock-3156e-{house}-{seed + SEED}")
    rng.shuffle(ok)
    return ok[:n]


def goto(robot, x_m, y_m, timeout_s=180):
    """A person's nav2 goal: named a person, or the robot server refuses it
    as a second autonomous driver (E's results, 'For the live run')."""
    before = (get_json(robot, "/world/goal").get("goal") or {}).get("goal_id")
    r = robot.post("/world/goal", json={"x_m": x_m, "y_m": y_m}, headers={"x-driver": "twin-dpad"})
    try:
        reply = r.json()
    except ValueError:
        reply = {}
    if r.status_code != 200 or not reply.get("accepted"):
        return f"refused: {r.status_code} {r.text[:120]}"
    # This goal's own id when the reply names it, so the last mission's
    # finished goal is never read as this one's end (review, 2026-11-20).
    want = reply.get("goal_id") or (reply.get("goal") or {}).get("goal_id")
    end, seen = time.time() + timeout_s, False
    while time.time() < end:
        g = (get_json(robot, "/world/goal").get("goal") or {})
        gid = g.get("goal_id")
        mine = gid == want if want else (gid is not None and gid != before)
        if mine and g.get("state") not in (None, "pending", "active"):
            return g.get("state")
        if seen and not mine:
            return "vanished"              # e.g. the SLAM session changed under it
        seen = seen or mine
        time.sleep(1.0)
    robot.delete("/world/goal")
    return "timeout"


def marker(robot, tag):
    """A line in the robot server's access log that delimits a mission's goals."""
    try:
        robot.get(f"/health?m={tag}")
    except httpx.HTTPError:
        pass


def goal_posts(log_path, start_tag, end_tag):
    """POST /world/goal lines between two markers of the robot's access log."""
    try:
        lines = Path(log_path).read_text(errors="replace").splitlines()
    except OSError:
        return None
    # The marker whole (`find-1` must not match `find-10`).
    i0 = next((i for i, x in enumerate(lines) if f"m={start_tag} " in x), None)
    i1 = next((i for i, x in enumerate(lines) if f"m={end_tag} " in x), None)
    if i0 is None or i1 is None:
        return None
    return sum('"POST /world/goal' in x for x in lines[i0:i1])


def start_mission(brain, body, wait_s=60):
    t = time.time()
    while True:
        r = brain.post("/mission/start", json=body)
        if r.status_code == 200 or time.time() - t > wait_s:
            return r
        time.sleep(2.0)                # a person's or the last mission's authority lapsing


def mission(robot, brain, truth, body, tag, dock_truth=None, drill=False, wall_s=WALL_S):
    # The goal already there (the person's, or the last mission's) is not
    # this mission's: `goals_seen` leaves it out (Thermos, 2026-11-20).
    prior = (get_json(robot, "/world/goal").get("goal") or {}).get("goal_id")
    marker(robot, f"start-{tag}")
    r = start_mission(brain, body)
    if r.status_code != 200:
        return {"start": r.status_code, "detail": r.text[:300]}
    t0 = time.time()
    rec = {"contacts": 0, "min_gap_cm": math.inf, "return_min_gap_cm": math.inf, "samples": 0,
           "load_max": 0.0, "goals_seen": {}, "prior_goal": prior}
    s, ended_at, ret_pose = {}, None, None
    while time.time() - t0 < wall_s:
        g = truth.gap_cm()
        rec["samples"] += 1
        rec["min_gap_cm"] = min(rec["min_gap_cm"], g)
        rec["contacts"] += g < 1.0
        rec["load_max"] = max(rec["load_max"], os.getloadavg()[0])
        s = get_json(brain, "/mission/status")
        ret = s.get("return") or {}
        gid = ((get_json(robot, "/world/goal").get("goal") or {}).get("goal_id"))
        if gid and gid != prior and gid not in rec["goals_seen"]:
            rec["goals_seen"][gid] = {"t": round(time.time() - t0, 2), "return": bool(ret)}
        if ret and ended_at is None:
            ended_at = round(time.time() - t0, 2)
            marker(robot, f"end-{tag}")
        if ret.get("state") == "returning":
            rec["return_min_gap_cm"] = min(rec["return_min_gap_cm"], g)
            now = truth.pose()
            ret_pose = ret_pose or now
            far = math.hypot(now[0] - ret_pose[0], now[1] - ret_pose[1])
            turned = abs((now[2] - ret_pose[2] + 180) % 360 - 180)
            # The Stop only once the return truly moves (review, 2026-11-20:
            # the first run's Stops all landed before any motion).
            if (drill and "drill_at" not in rec and ret.get("phase") in ("driving", "creep", "turn")
                    and (far >= STOP_WHEN_M or turned >= STOP_WHEN_DEG)):
                rec.update(drill_at=round(time.time() - t0, 1), drill_phase=ret.get("phase"),
                           drill_return_moved_m=round(far, 3), drill_return_turned_deg=round(turned, 1))
                p0 = truth.pose()
                err = post_ok(robot, "/stop")
                if err:
                    rec["stop_error"] = err
                t1, moved, spun, track = time.time(), 0.0, 0.0, []
                while time.time() - t1 < 10.0:
                    p = truth.pose()
                    track.append([round(time.time() - t1, 2), round(p[0], 4), round(p[1], 4), round(p[2], 2)])
                    moved = max(moved, math.hypot(p[0] - p0[0], p[1] - p0[1]))
                    spun = max(spun, abs((p[2] - p0[2] + 180) % 360 - 180))
                    time.sleep(0.2)
                # From the pose at the Stop's sending: any coasting counts.
                rec["moved_after_stop_cm"] = round(moved * 100, 1)
                rec["turned_after_stop_deg"] = round(spun, 1)
                rec["after_stop_track"] = track
        if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(0.25)
    else:
        rec["wall_capped"] = True
        err = post_ok(brain, "/mission/stop")
        if err:
            rec["mission_stop_error"] = err
        for _ in range(60):            # the end the stop made, not the last sample
            s = get_json(brain, "/mission/status")
            if not s.get("running"):
                break
            time.sleep(0.5)
    if ended_at is None:
        marker(robot, f"end-{tag}")
    marker(robot, f"done-{tag}")
    ret = s.get("return") or {}
    dock = s.get("dock") or {}
    x, y, h = truth.pose()
    rec.update(outcome=s.get("outcome"), end_reason=(s.get("end_reason") or "")[:160],
               ret=ret, dock=dock, end_decided_s=ended_at, seconds=round(time.time() - t0),
               end_truth=[round(x, 3), round(y, 3), round(h, 1)])
    if dock_truth:
        rec["err_m"] = round(math.hypot(x - dock_truth[0], y - dock_truth[1]), 3)
        rec["err_deg"] = round(abs((h - dock_truth[2] + 180) % 360 - 180), 1)
        rec["docked_in_bars"] = (ret.get("state") == "docked" and rec["err_m"] <= DOCK_M
                                 and rec["err_deg"] <= DOCK_DEG)
    for k in ("min_gap_cm", "return_min_gap_cm"):
        rec[k] = None if rec[k] == math.inf else round(rec[k], 1)
    rec["load_max"] = round(rec["load_max"], 2)
    return rec


def container_logs(tag):
    os.system(f"docker logs {de.CONTAINER} > {de.RUN_DIR}/ros-{tag}.log 2>&1")


def saved_versions():
    r = _run(["docker", "exec", de.CONTAINER, "sh", "-c", "ls -d /maps/*/v* 2>/dev/null"],
             capture_output=True, text=True)
    return r.stdout.split()


def main(argv):
    out = Path(argv[0])
    port = free_port()
    httpd = ThreadingHTTPServer(("127.0.0.1", port), Standins)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    os.environ.update(LOCAL_VLM_URL=f"http://127.0.0.1:{port}", VISION_URL=f"http://127.0.0.1:{port}")

    def emit(r):
        print(json.dumps(r), flush=True)
        with open(out, "a") as f:
            f.write(json.dumps(r) + "\n")

    for house in os.environ.get("LIVE_HOUSES", "scaled_house,home_first_floor").split(","):
        _run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
        _run(["docker", "volume", "rm", "-f", VOLUME], capture_output=True)
        # 1. map: one survey from the dock; its end saves the map.
        robot = de.stack(house)
        try:
            brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
            truth = Truth(robot, house)
            dock_truth = truth.pose()
            rec = mission(robot, brain, truth, {"job": "survey", "policy": "explore", "max_steps": 600,
                                                "mission_time_limit_s": MAP_LIMIT_S},
                          f"map-{house}", dock_truth=dock_truth, wall_s=MAP_LIMIT_S + 400)
            for _ in range(60):
                if saved_versions():
                    break
                time.sleep(2)
            emit({"kind": "map", "house": house, "dock_truth": dock_truth, "saved": saved_versions(),
                  "run_dir": de.RUN_DIR, **rec})
            Standins.calls.update(local=0, cloud=0)
        finally:
            container_logs("map")
            de._kill_ports()
            _run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
        # 2. reload: missions started off the dock.
        robot = de.stack(house)
        try:
            brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
            truth = Truth(robot, house)
            start = truth.pose()
            t = time.time()
            pose = {}
            while time.time() - t < 120:
                pose = robot.get("/world/pose").json()
                if pose.get("usable") and pose.get("dock"):
                    break
                time.sleep(2)
            emit({"kind": "reload", "house": house, "start_truth": start, "dock_truth": dock_truth,
                  "world_pose": pose, "run_dir": de.RUN_DIR})
            plan = [("find", c) for c in off_dock_cells(house, dock_truth, FINDS, 0)]
            if house == "scaled_house":
                plan += [("drill", c) for c in off_dock_cells(house, dock_truth, DRILLS, 1)]
            for i, (kind, c) in enumerate(plan):
                tag = f"{house}-{kind}-{i}"
                arrived = goto(robot, (c[0] + 0.5) * 0.3, (c[1] + 0.5) * 0.3)
                p = truth.pose()
                body = {"target_object": de.TARGET, "policy": "tiered", "max_steps": 400,
                        "mission_time_limit_s": FIND_LIMIT_S}
                rec = mission(robot, brain, truth, body, tag, dock_truth=dock_truth,
                              drill=kind == "drill")
                rec["goal_posts_before_end"] = goal_posts(f"{de.RUN_DIR}/explore_robot.log",
                                                          f"start-{tag}", f"end-{tag}")
                rec["goal_posts_after_end"] = goal_posts(f"{de.RUN_DIR}/explore_robot.log",
                                                         f"end-{tag}", f"done-{tag}")
                emit({"kind": kind, "house": house, "i": i, "start_cell": c, "start_goal": arrived,
                      "start_truth": [round(v, 3) for v in p],
                      "start_off_dock_m": round(math.hypot(p[0] - dock_truth[0], p[1] - dock_truth[1]), 2),
                      "standin_calls": dict(Standins.calls), "run_dir": de.RUN_DIR, **rec})
                Standins.calls.update(local=0, cloud=0)
        finally:
            container_logs("reload")
            de._kill_ports()
            _run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
    _run(["docker", "volume", "rm", "-f", VOLUME], capture_output=True)
    httpd.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
