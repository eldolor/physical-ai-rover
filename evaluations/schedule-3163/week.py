# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/schedule-3163/week.py -- 3.163.B's T3 (with T1 and T5 inside it),
as amendment 10 fixed it before any run
(docs/plans/ros-alignment/3.163-scheduled-missions.md).

On 3.158.B's stack (`evaluations/autonomy-3158/B/rehearsal.py`: ROS, nav2, the
cloud and VE stand-ins at $0, footprint truth), `SIM_MAP=home_first_floor`.
The brain's scheduler reads its clock through `SCHEDULE_CLOCK_FILE`; this
harness writes the offset there to jump the idle gaps, and every run itself
plays out in real time.

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    ROS_IMAGE=vision-picar-ros:latest python evaluations/schedule-3163/week.py week
    ... week.py hand      # the same 15 jobs that fired, by hand, on a fresh stack
    ... week.py wall      # T1 at real wall time: 3 finds, scheduled 2 minutes ahead

Writes `<mode>.jsonl` beside this file (numbers only: no map, no frame).
The schedules and their record go to a temporary directory, never the repo.
`demo_explore.stack()` takes the desktop's exclusive lock itself.
"""

import importlib.util
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path

import httpx

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))

_spec = importlib.util.spec_from_file_location(
    "rehearsal_3158", ROOT / "evaluations/autonomy-3158/B/rehearsal.py")
rh = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rh)          # its environment: the pack, the stand-ins' tiers
rh.VOLUME = "picar-maps-3163"         # this section's own saved map
de = rh.de

HOUSE = "home_first_floor"
MONDAY = datetime(2026, 11, 16)
GRACE_S = 600
TICK_S = 10
SCHEDULES = [
    {"id": "survey", "job": "survey", "target": None, "days": list(range(7)), "at": "09:00", "enabled": True},
    {"id": "backpack", "job": "find", "target": "red backpack", "days": list(range(7)), "at": "13:00",
     "enabled": True},
    {"id": "table", "job": "find", "target": "side table", "days": list(range(7)), "at": "17:00",
     "enabled": True},
]
# Amendment 10: (day, schedule id) -> the injected skip and its expected end.
INJECT = {(1, "backpack"): ("off_dock", "not_docked"),
          (2, "table"): ("battery_low", "battery_not_normal"),
          (3, "survey"): ("off_dock", "not_docked"),
          (4, "backpack"): ("holding", "person_holding"),
          (5, "survey"): ("stop", "person_stop_latched"),
          (6, "backpack"): ("brain_down_inside", "fired"),
          (6, "table"): ("brain_down_through", "missed_down")}
PERSON = {"x-driver": "twin-dpad"}


class Clock:
    """The scheduler's clock: the wall time plus the offset in the file."""

    def __init__(self, path):
        self.path = Path(path)
        self.set_offset(0.0)

    def set_offset(self, s):
        self.path.write_text(f"{s:.3f}")

    def jump_to(self, fake: datetime):
        self.set_offset((fake - datetime.now()).total_seconds())

    def now(self) -> datetime:
        return datetime.now() + timedelta(seconds=float(self.path.read_text()))


def start_brain(env_extra):
    """The brain alone, as `demo_explore.stack()` starts it (a restart, T5)."""
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(SIM_MAP=HOUSE, ROBOT_DRIVE="ros", WORLD_MODE="ros",
               ROS_BRIDGE_URL=f"http://127.0.0.1:{de.BRIDGE}",
               ROBOT_URL=f"http://127.0.0.1:{de.ROBOT}", PICAR_LOG_LEVEL="INFO", **env_extra)
    log = open(f"{de.RUN_DIR}/explore_brain_restart_{int(time.time())}.log", "w")
    subprocess.Popen([sys.executable, "-m", "uvicorn", "control.brain_server:app", "--port", str(de.BRAIN),
                      "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=env,
                     stdout=log, stderr=subprocess.STDOUT)
    end = time.time() + 60
    while time.time() < end:
        try:
            if httpx.get(f"http://127.0.0.1:{de.BRAIN}/health", timeout=2).status_code == 200:
                return
        except httpx.HTTPError:
            pass
        time.sleep(0.5)
    raise RuntimeError("the brain never came back")


def kill_brain():
    for pid in de._listeners(de.BRAIN):
        subprocess.run(["kill", "-9", pid])
    end = time.time() + 20
    while de._listeners(de.BRAIN) and time.time() < end:
        time.sleep(0.2)


def final_line(brain, due: datetime, sid: str):
    due_s = due.isoformat()
    for r in brain.get("/schedules").json().get("runs", []):
        if r["due"] == due_s and r["id"] == sid:
            return r
    return None


def wait_final(brain, due, sid, timeout):
    end = time.time() + timeout
    while time.time() < end:
        try:
            line = final_line(brain, due, sid)
        except httpx.HTTPError:
            line = None
        if line:
            return line
        time.sleep(0.5)
    return None


def nudge(robot, action):
    """A person's D-pad move of 0.3 m or so."""
    return robot.post("/action", json={"action": action, "duration": 1.0}, headers=PERSON).json()


def lapse(robot):
    end = time.time() + 10
    while robot.get("/health").json().get("authority_holder") and time.time() < end:
        time.sleep(0.1)


def follow(robot, brain, m, started_wall):
    """`rehearsal.mission()`'s measuring loop, for a mission someone else started."""
    limit = rh.LIMITS[m["job"]]
    tr, s, vmin = rh.Truth(HOUSE, robot), {}, math.inf
    start, pose, found_m = None, None, None
    v0 = rh.volts(robot)
    rh.Standins.calls.update(local=0, cloud=0)
    while time.time() - started_wall < limit + rh.OVER_S:
        pose = tr.sample()
        start = start or pose
        v = rh.volts(robot)
        vmin = min(vmin, v if v is not None else math.inf)
        s = brain.get("/mission/status").json()
        if found_m is None and m.get("target") and (
                s.get("outcome") == "found" or (s.get("return") or {}).get("after") == "found"):
            found_m = rh.target_m(pose, tr.world, m["target"])
        if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(0.5)
    else:
        brain.post("/mission/stop")
    pose = tr.sample()
    err = math.hypot(pose["x_m"] - start["x_m"], pose["y_m"] - start["y_m"])
    deg = abs((pose["heading_deg"] - start["heading_deg"] + 180.0) % 360.0 - 180.0)
    ret = s.get("return") or {}
    rooms_all = sorted(set(tr.room.values()))
    return {"outcome": s.get("outcome"), "end_reason": (s.get("end_reason") or "")[:200],
            "started_by": s.get("started_by"), "seconds": round(time.time() - started_wall),
            "ret_state": ret.get("state"), "dock_err_m": round(err, 3), "dock_err_deg": round(deg, 1),
            "docked": ret.get("state") == "docked" and err <= rh.DOCK_M and deg <= rh.DOCK_DEG,
            "found_target_m": None if found_m is None else round(found_m, 2),
            "found_right": found_m is not None and found_m <= rh.FOUND_RADIUS_M,
            "contacts": tr.contacts, "inside_samples": tr.inside, "samples": tr.samples,
            "min_gap_cm": round(tr.min_gap, 1), "fell": tr.fell,
            "rooms_truth": sorted(tr.rooms), "rooms_total": len(rooms_all),
            "v_start": v0, "v_end": rh.volts(robot), "v_min": None if vmin == math.inf else vmin,
            "run_dir": de.RUN_DIR}


def wait_charge(robot):
    """Spaced for the charge, as 3.158.B is. Amendment 11: a pack the charger
    stand-in has not brought back by then is topped up, and the run says so."""
    t = time.time()
    while (rh.volts(robot) or 0) < rh.START_V and time.time() - t < rh.CHARGE_WAIT_S:
        time.sleep(2)
    v = rh.volts(robot) or 0
    if v < rh.START_V:
        robot.post("/sim/battery", json={"v": rh.START_V})
        time.sleep(15)                  # the ladder reads the new pack
        return round(v, 2)
    return None


def week(out: Path, env_extra, clock: Clock):
    robot = rh.ROBOT_CLIENT
    brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
    clock.jump_to(MONDAY.replace(hour=8))
    time.sleep(TICK_S + 2)                 # the scheduler notes the dock
    r = brain.put("/schedules", json={"enabled": True, "schedules": SCHEDULES})
    r.raise_for_status()
    done = {(x["day"], x["id"]) for x in map(json.loads, out.read_text().splitlines())} if out.exists() else set()
    for day in range(7):
        for sched in SCHEDULES:
            if (day, sched["id"]) in done:
                continue
            hh, mm = map(int, sched["at"].split(":"))
            due = (MONDAY + timedelta(days=day)).replace(hour=hh, minute=mm)
            inject, expect = INJECT.get((day, sched["id"]), (None, "fired"))
            topped = wait_charge(robot)
            rec = {"day": day, "id": sched["id"], "job": sched["job"], "target": sched["target"],
                   "due": due.isoformat(), "inject": inject, "expect": expect,
                   "load": round(os.getloadavg()[0], 2), "topped_up_from_v": topped}
            holder = None
            if inject == "off_dock":
                before = robot.get("/world/truth").json()
                rec["nudge"] = nudge(robot, "FORWARD").get("executed")
                lapse(robot)
            elif inject == "battery_low":
                robot.post("/sim/battery", json={"v": 10.3})
                time.sleep(30)             # the ladder's two samples
            elif inject == "stop":
                robot.post("/stop", headers=PERSON)
            elif inject == "holding":
                stop_hold = threading.Event()

                def hold():
                    while not stop_hold.is_set():
                        robot.post("/action", json={"action": "LOOK_LEFT"}, headers=PERSON)
                        time.sleep(0.4)
                holder = threading.Thread(target=hold, daemon=True)
                holder.start()
                time.sleep(1.0)
            if inject in ("brain_down_inside", "brain_down_through"):
                clock.jump_to(due - timedelta(seconds=40))
                kill_brain()
                back = (due + timedelta(seconds=300) if inject == "brain_down_inside"
                        else due + timedelta(seconds=GRACE_S + 60))
                clock.jump_to(back)
                start_brain(env_extra)
                real_due = None
            else:
                clock.jump_to(due - timedelta(seconds=20))
                real_due = time.time() + 20
            if expect == "fired":
                t_run = None
                end = time.time() + 120
                while time.time() < end:
                    s = brain.get("/mission/status").json()
                    if s.get("running") and s.get("started_by") == f"schedule:{sched['id']}":
                        t_run = time.time()
                        break
                    time.sleep(0.25)
                rec["t1_s"] = None if (t_run is None or real_due is None) else round(t_run - real_due, 1)
                rec["started"] = t_run is not None
                if t_run is not None:
                    rec.update(follow(robot, brain, sched, t_run))
                line = wait_final(brain, due, sched["id"], 30)
            else:
                # Amendment 11: wait until the scheduler has tried it (run 1
                # jumped past the grace before any tick saw the due time).
                seen, end = [], time.time() + 20 + 6 * TICK_S
                while time.time() < end and not seen:
                    time.sleep(1.0)
                    waiting = brain.get("/schedules").json().get("waiting", [])
                    seen = [w["reason"] for w in waiting
                            if w["id"] == sched["id"] and w["due"] == due.isoformat()]
                rec["waiting"] = seen
                clock.jump_to(due + timedelta(seconds=GRACE_S + 15))
                line = wait_final(brain, due, sched["id"], 60)
                s = brain.get("/mission/status").json()
                rec["started"] = bool(s.get("running")) and s.get("started_by") == f"schedule:{sched['id']}"
                if rec["started"] and s.get("running"):
                    rec.update(follow(robot, brain, sched, time.time()))   # it should not have
            rec["line"] = line
            got = None if line is None else (line.get("reason") or line["outcome"])
            rec["ok_end"] = got == expect or (expect == "fired" and line is not None
                                              and line["outcome"] == "fired")
            # Undo the injection.
            if holder is not None:
                stop_hold.set()
                holder.join()
                lapse(robot)
            if inject == "off_dock":
                rec["nudge_back"] = nudge(robot, "REVERSE").get("executed")
                lapse(robot)
                after = robot.get("/world/truth").json()
                rec["nudge_back_err_m"] = round(math.hypot(after["x_m"] - before["x_m"],
                                                           after["y_m"] - before["y_m"]), 3)
            elif inject == "battery_low":
                robot.post("/sim/battery", json={"v": 12.0})
                time.sleep(15)
            elif inject == "stop":
                robot.post("/autonomy/resume", headers={"x-driver": "brain"})
            with out.open("a") as f:
                f.write(json.dumps(rec) + "\n")
            print(day, sched["id"], inject, got, rec.get("outcome"), rec.get("t1_s"),
                  "contacts", rec.get("contacts"), "docked", rec.get("docked"), flush=True)


def hand(out: Path):
    """Amendment 10: the 15 jobs that fired in the week, by hand, same order."""
    robot = rh.ROBOT_CLIENT
    brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
    fired = [r for r in map(json.loads, (HERE / "week.jsonl").read_text().splitlines())
             if r["expect"] == "fired"]
    done = len(out.read_text().splitlines()) if out.exists() else 0
    for r in fired[done:]:
        topped = wait_charge(robot)
        m = {"job": r["job"], "target": r["target"]}
        rec = {"day": r["day"], "id": r["id"], "job": r["job"], "target": r["target"],
               "load": round(os.getloadavg()[0], 2), "topped_up_from_v": topped,
               **rh.mission(robot, brain, HOUSE, m)}
        rec["found_right"] = rec.get("found_target_m") is not None and rec["found_target_m"] <= rh.FOUND_RADIUS_M
        with out.open("a") as f:
            f.write(json.dumps(rec) + "\n")
        print(r["day"], r["id"], rec.get("outcome"), "contacts", rec.get("contacts"), flush=True)


def wall(out: Path, clock: Clock):
    """T1 at real wall time: 3 finds, each scheduled two minutes ahead."""
    robot = rh.ROBOT_CLIENT
    brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
    clock.set_offset(0.0)
    for i in range(3):
        topped = wait_charge(robot)
        due = (datetime.now() + timedelta(minutes=2)).replace(second=0, microsecond=0)
        sid = f"wall{i}"
        brain.put("/schedules", json={"enabled": True, "schedules": [
            {"id": sid, "job": "find", "target": "red backpack", "days": list(range(7)),
             "at": due.strftime("%H:%M"), "enabled": True}]}).raise_for_status()
        real_due = due.timestamp()
        t_run = None
        end = real_due + 180
        while time.time() < end:
            s = brain.get("/mission/status").json()
            if s.get("running") and s.get("started_by") == f"schedule:{sid}":
                t_run = time.time()
                break
            time.sleep(0.25)
        rec = {"i": i, "due": due.isoformat(), "t1_s": None if t_run is None else round(t_run - real_due, 1),
               "load": round(os.getloadavg()[0], 2), "topped_up_from_v": topped}
        if t_run is not None:
            rec.update(follow(robot, brain, {"job": "find", "target": "red backpack"}, t_run))
        with out.open("a") as f:
            f.write(json.dumps(rec) + "\n")
        print("wall", i, rec["t1_s"], rec.get("outcome"), flush=True)


def main():
    mode = sys.argv[1]
    tmp = Path(tempfile.mkdtemp(prefix="schedule-3163-"))
    clock = Clock(tmp / "clock")
    env_extra = {"SCHEDULE_DIR": str(tmp / "sched"), "SCHEDULE_CLOCK_FILE": str(clock.path)}
    if mode == "hand":
        env_extra = {"SCHEDULE_DIR": str(tmp / "sched")}       # no schedules at all
    os.environ.update(env_extra)
    port = rh.free_port()
    httpd = rh.ThreadingHTTPServer(("127.0.0.1", port), rh.Standins)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    os.environ.update(LOCAL_VLM_URL=f"http://127.0.0.1:{port}", VISION_URL=f"http://127.0.0.1:{port}")
    rh.ROBOT_CLIENT = de.stack(HOUSE)
    try:
        if mode == "week":
            week(HERE / "week.jsonl", env_extra, clock)
            # Every due time's ends, from the brain's own record: none twice.
            lines = [json.loads(x) for x in (tmp / "sched" / "runs.jsonl").read_text().splitlines()]
            ends = [(x["due"], x["id"], x["outcome"], x.get("reason")) for x in lines]
            finals = [e for e in ends if e[2] != "firing"]
            keys = [(e[0], e[1]) for e in finals]
            (HERE / "week-runs.json").write_text(json.dumps(
                {"ends": finals, "twice": sorted({k for k in keys if keys.count(k) > 1}),
                 "fired": sum(e[2] == "fired" for e in finals)}, indent=1) + "\n")
            wall(HERE / "wall.jsonl", clock)
        elif mode == "hand":
            hand(HERE / "hand.jsonl")
        elif mode == "wall":
            wall(HERE / "wall.jsonl", clock)
    finally:
        de._kill_ports()
        os.system(f"docker logs {de.CONTAINER} > {de.RUN_DIR}/ros.log 2>&1; "
                  f"docker rm -f {de.CONTAINER} >/dev/null 2>&1")
        httpd.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
