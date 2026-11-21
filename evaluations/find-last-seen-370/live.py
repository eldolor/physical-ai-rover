# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.70.M2's live half: a survey, then finds that should go first to where the
survey last saw the target, on the ROS stack through the brain's HTTP API.
The bars are "M2's live half: how it is measured" in
docs/plans/ros-alignment/3.70-survey-and-mission-ends.md, written before this ran.

  s1  map session: fresh servers and container, empty maps and inventory; one
      survey (explore, S1's 1800 s); its end saves the map and the report
  s2  reload session: everything restarted on the dock, the reload judged;
      per plan entry k 0-4 (sweep.plan, the sim's targets and starts): a
      person's nav2 goal to the start, the inventory put back to the survey's
      files, then a find for X (explore, 600 s, return to the dock on)

Rows: evaluations/find-last-seen-370/live.jsonl (one per session or find).
The maps and inventories stay under EXPLORE_LOGDIR (a sim house's floor
plan, never committed).

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host python tools/desktop_lock.py exclusive --why "3.70.M2 live" -- \\
        python evaluations/find-last-seen-370/live.py run
    python evaluations/find-last-seen-370/live.py score
"""
import glob
import importlib.util
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
ROWS = HERE / "live.jsonl"
HOUSE = "complex_house"
SURVEY_LIMIT_S = 1800
FIND_LIMIT_S = 600
WALL_S = FIND_LIMIT_S + 600      # the limit, the return home and more
FINDS = 5                        # plan entries k 0-4
RECALL_AT_M = 0.30
APPROACH_M = 0.60
FOUND_M = 0.9
CONTACT_CM = 1.0


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def first_act(events, reported, target):
    """L1 on one find's explore events: the first recall action (a recall goal
    sent, or a recall faced where it stands) before any other goal, its place
    within RECALL_AT_M of a reported landmark of X, its goal within APPROACH_M
    of that place. Returns a dict for the row."""
    acts = [e for e in events
            if e.get("event") == "goal_sent" or (e.get("event") == "recall" and e.get("here"))]
    first = acts[0] if acts else None
    if first is None:
        return {"first_act": None, "first_at": None, "first_goal": None,
                "first_goal_m": None, "first_recall_ok": False}
    kind = "recall" if first["event"] == "recall" or first.get("kind") == "recall" else first.get("kind")
    at = first.get("at")
    lms = [(r["x_m"], r["y_m"]) for r in reported if r["label"] == target]
    near = (kind == "recall" and at is not None
            and any(math.dist(at, p) <= RECALL_AT_M for p in lms))
    goal = first.get("goal")
    goal_m = None if not near else (0.0 if goal is None else round(math.dist(goal, at), 2))
    return {"first_act": kind, "first_at": at, "first_goal": goal, "first_goal_m": goal_m,
            "first_recall_ok": bool(near and goal_m is not None and goal_m <= APPROACH_M + 0.005)}


def events_from_log(text):
    """The brain's `explore {json}` lines, in order."""
    out = []
    for line in text.splitlines():
        i = line.find("explore {")
        if i < 0:
            continue
        try:
            out.append(json.loads(line[i + len("explore "):]))
        except ValueError:
            continue
    return out


def score(path=ROWS):
    rows = [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]
    s1 = next((r for r in rows if r.get("row") == "survey"), None)
    s2 = next((r for r in rows if r.get("row") == "reload"), None)
    finds = [r for r in rows if r.get("row") == "find"]
    started = [f for f in finds if f.get("outcome") != "start_refused"]
    map_id = (s1 or {}).get("map_id")
    pre = {
        "survey_saved_one_map": bool(s1 and map_id and s1.get("report_frame_id") == map_id
                                     and s1.get("targets")),
        "reload_matched": bool(s2 and (s2.get("reload") or {}).get("result") == "matched"
                               and s2.get("map_id") == map_id),
        # A refused start is a failed find (L3), not a broken instrument.
        "every_find_on_the_reload": bool(started) and all(
            (f.get("saved_map") or {}).get("frame_id") == map_id
            and (f.get("saved_map") or {}).get("reloaded") is True for f in started),
    }
    n = len(finds)
    l1 = sum(bool(f.get("first_recall_ok")) for f in finds)
    found_ok = sum(f.get("outcome") == "found" and f.get("found_m") is not None
                   and f["found_m"] <= FOUND_M for f in finds)
    # A found with no measured distance fails L2 (not shown within 0.9 m)
    # but is not a wrong found: it is reported apart (Thermos).
    wrong = sum(f.get("outcome") == "found" and f.get("found_m") is not None
                and f["found_m"] > FOUND_M for f in finds)
    unmeasured = sum(f.get("outcome") == "found" and f.get("found_m") is None for f in finds)
    contacts = sum(f.get("contacts", 0) for f in finds)
    failed = sum(f.get("outcome") in ("failed", "start_refused") for f in finds)
    valid = all(pre.values()) and n == FINDS

    def med(k):
        xs = sorted(f[k] for f in finds if f.get(k) is not None)
        return None if not xs else (xs[len(xs) // 2] if len(xs) % 2 else (xs[len(xs) // 2 - 1] + xs[len(xs) // 2]) / 2)

    verdict = (lambda ok: ("MET" if ok else "FAILED") if valid else "INVALID")
    return {
        "preconditions": pre, "finds": n, "valid": valid,
        "L1_first_goal": {"recall_first": l1, "of": n, "verdict": verdict(l1 == FINDS)},
        "L2_found": {"found": found_ok, "of": n, "verdict": verdict(found_ok == FINDS)},
        "L3_stability": {"contacts": contacts, "wrong_found": wrong, "failed": failed,
                         "found_unmeasured": unmeasured,
                         # Beside, no bar: a find the harness had to stop (Thermos).
                         "wall_capped": sum(bool(f.get("wall_capped")) for f in finds),
                         "verdict": verdict(contacts == 0 and wrong == 0 and failed == 0)},
        "beside": {"median_seconds": med("seconds"), "median_goals": med("goals"),
                   "median_distance_m": med("distance_m"),
                   "returns": [(f.get("ret") or {}).get("state") for f in finds],
                   # Outcome bars on a cloud box (3.189): recorded, not gated.
                   "load_max": max([r.get("load_max") or 0 for r in rows]
                                   + [(r.get("load") or {}).get("max") or 0 for r in rows] or [0])},
    }


# ---------------------------------------------------------------- the run


def run(rows=ROWS, finds=FINDS, find_limit_s=FIND_LIMIT_S, move=None, before_find=None):
    """M2's live half; 3.70.M3 reuses it with its own `rows`, one find, its
    limit, and `move(m, truth)` -> {"p0", "p1"} cells: X moved through the
    robot server's `/sim/objects/move` after the inventory is put back and
    before the find starts (None: nothing moved, M2's run). 3.205.A:
    `before_find(inv_dir, m)` -> dict, run right after the inventory is put
    back (it drops X's rows), its answer kept in the find's row as
    `before_find`."""
    assert os.environ.get("ROS_IMAGE", "").endswith("plan-3.70"), "run with this branch's env"
    rows = Path(rows)
    wall_s = find_limit_s + 600
    if rows.exists():
        raise SystemExit(f"{rows} exists: move it aside first (never overwritten)")
    image = os.environ["ROS_IMAGE"]
    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(ROOT / "evaluations" / "map-350"))
    os.environ["ROS_IMAGE"] = "for-import-plan-3.50"        # d3_runs' own guard
    import httpx
    import d3_runs as d3
    import d5_runs as d5
    from d3_runs import de
    from control.inventory_store import InventoryStore
    from sim.grid_world import CELL_WALL
    from sim.maps import build_world
    from tests.footprint_sweep import CELL_CM, chassis, gap, square
    d3.IMAGE = image
    os.environ["ROS_IMAGE"] = image
    s4 = _load("s4_live", ROOT / "evaluations" / "survey-370" / "s4_live.py")
    sweep = _load("find_sweep_370", HERE / "sweep.py")

    world = build_world(HOUSE)
    walls = {(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row) if c == CELL_WALL}

    def write(row):
        with open(rows, "a") as f:
            f.write(json.dumps(row) + "\n")

    def gap_cm(robot):
        t = robot.get("/world/truth").json()
        cx, cy = t["x_m"] / 0.3, t["y_m"] / 0.3
        objs = {(o["x"], o["y"]) for o in robot.get("/sim/objects").json()["objects"]}
        A = chassis(cx, cy, math.radians(t["heading_deg"]) - math.pi / 2)
        g = math.inf
        for c in walls | objs:
            if abs(c[0] + 0.5 - cx) > 1.6 or abs(c[1] + 0.5 - cy) > 1.6:
                continue
            g = min(g, gap(A, square(*c)) * CELL_CM)
        return g, (t["x_m"], t["y_m"], t["heading_deg"])

    def jget(client, path):
        """One read; a failed one is {} (review: one bad reply must not end
        a two-hour run)."""
        try:
            return client.get(path).json()
        except (httpx.HTTPError, ValueError):
            return {}

    def save_logs(tag):
        """The container's log before it is removed (review)."""
        if subprocess.run(["docker", "inspect", de.CONTAINER], capture_output=True).returncode == 0:
            with open(f"{de.RUN_DIR}/ros-{tag}.log", "w") as f:
                subprocess.run(["docker", "logs", de.CONTAINER], stdout=f, stderr=subprocess.STDOUT)

    def goto(robot, x_m, y_m, timeout_s=240):
        before = (jget(robot, "/world/goal").get("goal") or {}).get("goal_id")
        r = robot.post("/world/goal", json={"x_m": x_m, "y_m": y_m}, headers={"x-driver": "twin-dpad"})
        try:
            reply = r.json()
        except ValueError:
            reply = {}
        if r.status_code != 200 or not reply.get("accepted"):
            return f"refused: {r.status_code} {r.text[:120]}"
        want = reply.get("goal_id") or (reply.get("goal") or {}).get("goal_id")
        end, seen = time.time() + timeout_s, False
        while time.time() < end:
            g = jget(robot, "/world/goal").get("goal") or {}
            gid = g.get("goal_id")
            mine = gid == want if want else (gid is not None and gid != before)
            if mine and g.get("state") not in (None, "pending", "active"):
                return g.get("state")
            if seen and not mine:
                return "vanished"
            seen = seen or mine
            time.sleep(1.0)
        robot.delete("/world/goal")
        return "timeout"

    def brain_log():
        logs = sorted(glob.glob(f"{de.RUN_DIR}/brain-*.log"), key=os.path.getmtime)
        return logs[-1] if logs else None

    def restore(inv_dir, keep):
        for name in os.listdir(inv_dir):
            if name not in keep:
                p = os.path.join(inv_dir, name)
                if os.path.isfile(p):
                    os.remove(p)

    def find(robot, brain, k, target, reported):
        log = brain_log()
        off = os.path.getsize(log) if log else 0
        r = brain.post("/mission/start", json={"target_object": target, "policy": "explore",
                                               "max_steps": 100000,
                                               "mission_time_limit_s": find_limit_s})
        if r.status_code != 200:
            return {"outcome": "start_refused", "start": r.status_code, "detail": r.text[:300]}
        objs = [(o["x"], o["y"]) for o in (jget(robot, "/sim/objects").get("objects") or [])
                if o.get("name") == target]
        t0 = time.time()
        rec = {"contacts": 0, "min_gap_cm": math.inf, "samples": 0, "load_max": 0.0,
               "distance_m": 0.0, "saved_map": None, "found_m": None, "end_seen_s": None}
        last, s = None, {}
        while time.time() - t0 < wall_s:
            try:
                g, p = gap_cm(robot)
                s = brain.get("/mission/status").json()
            except (httpx.HTTPError, ValueError, KeyError, TypeError):
                time.sleep(0.5)
                continue
            rec["samples"] += 1
            rec["min_gap_cm"] = min(rec["min_gap_cm"], g)
            rec["contacts"] += g < CONTACT_CM
            rec["load_max"] = max(rec["load_max"], os.getloadavg()[0])
            if last is not None and rec["end_seen_s"] is None:
                rec["distance_m"] += math.dist(last[:2], p[:2])
            last = p
            if rec["saved_map"] is None and s.get("saved_map"):
                rec["saved_map"] = s["saved_map"]
            ended = s.get("return") or (s.get("outcome") not in (None, "idle", "running"))
            if ended and rec["end_seen_s"] is None:
                rec["end_seen_s"] = round(time.time() - t0, 2)
                rec["end_truth"] = [round(v, 3) for v in p]
                if objs:
                    rec["found_m"] = round(min(math.dist(p[:2], ((ox + 0.5) * 0.3, (oy + 0.5) * 0.3))
                                               for ox, oy in objs), 3)
            if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
                break
            time.sleep(0.25)
        else:
            rec["wall_capped"] = True
            brain.post("/mission/stop")
            time.sleep(5)
            s = jget(brain, "/mission/status") or s
        time.sleep(4)                                      # the inventory's save thread
        text = ""
        if log:
            with open(log, errors="replace") as f:
                f.seek(off)
                text = f.read()
        ev = events_from_log(text)
        goals = [e for e in ev if e.get("event") == "goal_sent"]
        rec.update(first_act(ev, reported, target))
        rec.update(outcome=s.get("outcome"), end_reason=(s.get("end_reason") or "")[:200],
                   seconds=round(time.time() - t0), mission_s=(s.get("budget") or {}).get("elapsed_s"),
                   goals=len(goals), recall_goals=sum(g.get("kind") == "recall" for g in goals),
                   goal_kinds=[g.get("kind") for g in goals][:30],
                   resight=sum(e.get("event") == "resight" for e in ev),
                   ret={k: (s.get("return") or {}).get(k) for k in ("state", "reason")},
                   target_cells=objs, steps=s.get("step"))
        rec["min_gap_cm"] = None if rec["min_gap_cm"] == math.inf else round(rec["min_gap_cm"], 1)
        rec["distance_m"] = round(rec["distance_m"], 2)
        rec["load_max"] = round(rec["load_max"], 2)
        return rec

    de._kill_ports()
    de._new_run_dir()
    load = s4.Load()
    load.start()
    maps = os.path.join(de.RUN_DIR, f"maps-{HOUSE}")
    inv_dir = os.path.join(de.RUN_DIR, f"inventory-{HOUSE}")
    os.makedirs(maps, exist_ok=True)
    os.makedirs(inv_dir, exist_ok=True)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
    head = subprocess.run(["git", "rev-parse", "--short=8", "HEAD"], capture_output=True,
                          text=True, cwd=ROOT).stdout.strip()
    try:
        # ---- s1: the survey ----
        t = time.time()
        robot, brain = d5.servers(HOUSE, inv_dir)
        d3.container(maps, bridge)
        sv = s4.survey(brain, SURVEY_LIMIT_S)
        saved, settled = s4.settle(bridge)
        map_id = (saved or {}).get("map_id")
        # The brain saves in its own thread and process: wait for the file
        # (review: a fixed sleep could read nothing and end the run here).
        written, t_rep = None, time.time()
        while map_id and written is None and time.time() - t_rep < 120:
            written = InventoryStore(inv_dir).latest_for(map_id)
            if written is None:
                time.sleep(2)
        reported = (written or {}).get("reported") or []
        plan = sweep.plan(reported)[:finds] if reported else []
        row = {"row": "survey", "house": HOUSE, "head": head, "image": image,
               "outcome": sv.get("outcome"), "seconds": sv.get("seconds"), "end": sv.get("end"),
               "map_id": map_id, "saved_version": (saved or {}).get("last_version"),
               "settled_s": settled, "report_frame_id": (written or {}).get("frame_id"),
               "reported": len(reported),
               "reported_rows": [{k: r[k] for k in ("id", "label", "x_m", "y_m", "belief")}
                                 for r in reported],
               "targets": sweep.targets(reported) if reported else [],
               "plan": plan, "load": load.summary(t)}
        write(row)
        print("survey", json.dumps({k: row[k] for k in ("outcome", "seconds", "map_id", "reported",
                                                         "targets")}), flush=True)
        if not plan:
            return
        keep = set(os.listdir(inv_dir))

        # ---- s2: reload, then the finds ----
        t = time.time()
        save_logs("s1")
        robot, brain = d5.servers(HOUSE, inv_dir)
        w = d3.container(maps, bridge)
        write({"row": "reload", "reload": w["health"]["slam"]["reload"],
               "map_id": w["health"]["map"]["map_id"], "dock_truth": list(d3.truth(robot)),
               "load": load.summary(t)})
        print("reload", json.dumps(w["health"]["slam"]["reload"]), flush=True)
        for m in plan:
            t = time.time()
            sx, sy = (m["start"][0] + 0.5) * 0.3, (m["start"][1] + 0.5) * 0.3
            try:
                reached = goto(robot, sx, sy)
            except (httpx.HTTPError, ValueError, KeyError, TypeError) as e:   # Thermos
                reached = f"error: {e!r}"[:160]
            t_ = jget(robot, "/world/truth")
            st = (t_.get("x_m", math.nan), t_.get("y_m", math.nan), t_.get("heading_deg", math.nan))
            # The last find's save has landed (its sleep); put the survey's back.
            restore(inv_dir, keep)
            prep = before_find(inv_dir, m) if before_find is not None else None
            moved = None
            if move is not None:
                try:
                    moved = move(m, t_)
                    r_ = robot.post("/sim/objects/move", json={"src": moved["p0"], "dst": moved["p1"]})
                    moved["status"] = r_.status_code
                except (httpx.HTTPError, ValueError, KeyError, TypeError) as e:
                    moved = {"error": f"{e!r}"[:160]}
                if moved.get("status") != 200:
                    write({"row": "find", "k": m["k"], "target": m["target"], "outcome": "not_moved",
                           "moved": moved})
                    continue
            rec = find(robot, brain, m["k"], m["target"], reported)
            if prep is not None:
                rec["before_find"] = prep
            if moved is not None:
                rec["moved"] = moved
                head_rep = InventoryStore(inv_dir).latest_for(map_id) or {}
                rec["end_landmarks"] = [
                    {k: d[k] for k in ("label", "x_m", "y_m", "belief")}
                    for d in (head_rep.get("reported") or []) + (head_rep.get("candidates") or [])
                    if d.get("label") == m["target"]]
            rec.update(row="find", k=m["k"], target=m["target"], start_room=m["room"],
                       start_cell=m["start"], start_goal=reached,
                       start_truth=[round(v, 3) for v in st],
                       start_off_m=round(math.dist(st[:2], (sx, sy)), 3),
                       load=load.summary(t))
            write(rec)
            print("find", m["k"], m["target"], json.dumps(
                {k: rec.get(k) for k in ("outcome", "first_act", "first_recall_ok", "found_m",
                                         "seconds", "goals", "contacts", "min_gap_cm", "ret")}),
                flush=True)
        restore(inv_dir, keep)
    finally:
        save_logs("end")
        subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
        de._kill_ports()
        load.stop_ev.set()
        print("load", json.dumps(load.summary()), flush=True)


if __name__ == "__main__":
    if sys.argv[1:2] == ["run"]:
        run()
    elif sys.argv[1:2] == ["score"]:
        print(json.dumps(score(Path(sys.argv[2]) if len(sys.argv) > 2 else ROWS), indent=1))
    else:
        raise SystemExit(__doc__)
