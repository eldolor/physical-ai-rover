# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/lat.py

3.136.C's latencies, P7, P6 and P5 (amendment 7 in the section says what
each one measures and why), on 3.135.C's full stack: `stack.py`'s processes
(this section's ports, the split body with `lat_body.py`'s truth and drop,
the fake board, the fake D500, `drive: ros`), the ROS container (SLAM +
nav2), the local VLM (the board's `picar-local-vlm`, VE, as found; on the
desktop a stand-in, `--desktop`), the YOLOE + CLIP loop at 4 Hz (the board)
and a $0 cloud stand-in; the brain on tiered find missions with explore
searching. Every stack's robot server and brain load `shim_lat/`.

    python evaluations/jetson-perf-3136/lat.py <out> [--p7 20] [--p6 20] [--p5 40]
    python evaluations/jetson-perf-3136/lat.py <out> --desktop --plant   # criterion 3

  P7  missions on QUICK's pairs (each a fresh stack, `p7_limit_s` each) until
      `--p7` arrival verdicts (`status.arrival_verdicts[].verdict_s`).
      With `--desktop`, one stack and the same target again once found (the
      robot already there): the measurer's check, not the board's number.
  P6  one stack in `home_first_floor`, a mission always running: `--p6`
      times a person's `POST /stop` while the mission's nav2 goal is active
      and the wheels turn (wheels zero from the truth; the goal's end from
      `GET /world/goal` polled every 25 ms), then `--p6` times during a
      person's FORWARD verb.
  P5  the same stack: `--p5` times, the mission's nav2 goal driving the
      robot forward with nothing within 75 cm ahead, an object moved in front
      (`/lat/drop`: the true travel to contact 8-16 cm, inside the 20 cm
      stop); from the first scan the robot server held that shows it
      (`shim_lat`'s scan rows, nearest return ahead < 0.40 m) to the forward
      speed zero (nav2 may go on pivoting in place, which 3.19 allows).

What lands in <out>: `p7.jsonl`, `p6.jsonl`, `p5.jsonl` (one row a trial;
the scorer, `lat_score.py`, reads the truth and shim logs they name),
`stacks.jsonl` (one row a stack: its wheel loop at the end, its directory),
and per stack `s<k>/` (truth.jsonl, lat_shim.jsonl, pace.jsonl,
explore_sends.jsonl, explore_cancels.jsonl, person.jsonl, the logs).
"""

from __future__ import annotations

import argparse
from http.client import HTTPException
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.parse
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FMEA = HERE.parent / "jetson-fmea-3135"
sys.path.insert(0, str(FMEA))
sys.path.insert(0, str(HERE))

import soak  # noqa: E402
from soak import STACK_OOM_ADJ, Soak, gz, http_ok, mem_available_mb, stack_env, wait  # noqa: E402
from stack import PY, Stack, StackError, http  # noqa: E402

QUICK = (("procthor:val:0", "red backpack"), ("procthor:val:0", "box"), ("procthor:val:3", "box"),
         ("home_first_floor", "dining table"), ("procthor:val:0", "dog bed"),
         ("procthor:val:3", "red backpack"))
P65_HOUSE = "home_first_floor"
P65_TARGETS = ("toilet", "desk", "coffee table", "armchair", "sofa", "chair")
IMAGE = os.environ.get("LAT_IMAGE", "vision-picar-ros:latest")
CONTAINER = os.environ.get("LAT_CONTAINER", "picar-ros-3136")
BRIDGE_PORT = int(os.environ.get("PICAR_BRIDGE_PORT", 8090))
BRIDGE = f"http://127.0.0.1:{BRIDGE_PORT}"
ROS_DOMAIN = os.environ.get("ROS_DOMAIN_ID", "")
VE_PORT = 8005
CLEAR_CM = 74.0          # a trial starts with nothing in true travel ahead (footprint truth looks 75 cm out)
DROP_LO, DROP_HI = 8.0, 16.0
FWD_AT_DROP_M_S = 0.02    # a drop counts only if the robot was driving forward then
POLL_GOAL_S = 0.025
# One trial's transport failures (a reply lost under load): the trial is
# recorded as failed and the next one tried, never the run ended.
TRIAL_ERRORS = (OSError, HTTPException, ValueError, StackError)

VE_STANDIN = """
import sys, uvicorn
from service.local_vlm.driver import EdgeDriver
from service.local_vlm.server import create_app
fake = [sys.executable, "tests/fake_edgellm_driver.py"]
class Driver(EdgeDriver):
    # tests/demo_local_tier.py's: the arrival says yes, a frame's question no.
    def ask(self, jpeg, target, deadline_s, box=False, question="identity"):
        if question == "direction":
            return super().ask(jpeg, "plain thing", deadline_s, question="direction")
        return super().ask(jpeg, "plain thing" if box else "yes thing", deadline_s, box=box)
vocab = {6: "[{", 7: "[]", 9: "x", 10: '"bbox_2d": [', 11: "282", 12: ", ",
         13: "40", 14: "362", 15: "160", 16: "]}]"}
import os
app = create_app(lambda: Driver(fake, dict(os.environ), vocab.get))
uvicorn.run(app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
"""


class LatStack(Stack):
    """3.135's stack with `lat_body.py` as its body."""

    def _spawn(self, role, argv, env=None):
        if role == "body":
            argv = [PY, str(HERE / "lat_body.py"), *argv[2:]]
        return super()._spawn(role, argv, env)


class ExploreLog:
    """`status.explore`'s sends and cancels, as board_probe.py's missions
    probe writes them (amendments 5-6), for nav2's validity item."""

    def __init__(self, d: Path):
        self.sends = open(d / "explore_sends.jsonl", "a", buffering=1)
        self.cancels = open(d / "explore_cancels.jsonl", "a", buffering=1)
        self.seen, self.seen_c, self.mission = 0, 0, 0

    def new_mission(self) -> None:
        self.mission += 1
        self.seen = self.seen_c = 0

    def take(self, s: dict) -> None:
        ex = (s or {}).get("explore") or {}
        new = [e for e in ex.get("sent") or [] if e[0] > self.seen]
        if new:
            self.sends.write(json.dumps({"t": round(time.time(), 3), "mission": self.mission,
                                         "sent": [e[1:] for e in new],
                                         "missing": new[0][0] - self.seen - 1}) + "\n")
            self.seen = new[-1][0]
        new_c = [e for e in ex.get("cancels") or [] if e[0] > self.seen_c]
        if new_c:
            self.cancels.write(json.dumps({"t": round(time.time(), 3), "mission": self.mission,
                                           "cancels": new_c,
                                           "missing": new_c[0][0] - self.seen_c - 1}) + "\n")
            self.seen_c = new_c[-1][0]

    def close(self) -> None:
        self.sends.close()
        self.cancels.close()


class Lat(Soak):
    def __init__(self, out: Path, *, desktop: bool, plant: bool, n7: int, n6: int, n5: int,
                 p7_limit_s: float, p7_missions: int):
        super().__init__(out, 6 * 3600, perception=not desktop)
        self.desktop, self.plant = desktop, plant
        self.n7, self.n6, self.n5 = n7, n6, n5
        self.p7_limit_s, self.p7_missions = p7_limit_s, p7_missions
        self.plant_file = self.out / "plant"
        self.plant_file.write_text("")
        self.k = 0
        self.stacks = open(self.out / "stacks.jsonl", "a", buffering=1)
        self.person = None
        self.xlog: Optional[ExploreLog] = None
        self.target = ""

    # ---------------------------------------------------------------- setup

    def setup(self) -> None:
        self.phase("setup")
        if self.desktop:
            if http_ok(f"http://127.0.0.1:{VE_PORT}/health"):
                raise StackError(f":{VE_PORT} already answers")
            self.spawn("ve", [PY, "-c", VE_STANDIN, str(VE_PORT)],
                       env=dict(os.environ, PYTHONPATH=str(ROOT)))
            wait(lambda: http("GET", f"http://127.0.0.1:{VE_PORT}/health", timeout=2)[1].get("ready"),
                 60, "the VE stand-in")
        else:
            ve = self.ve_state()
            (self.out / "ve_before.json").write_text(json.dumps(ve, indent=1))
            if not (isinstance(ve.get("health"), dict) and ve["health"].get("ready")):
                raise StackError(f"the board's local VLM is not ready: {ve}")
            if not self.vlm_secret:
                raise StackError("no APP_SHARED_SECRET in the VLM's env file")
        if self.sh(["docker", "ps", "-aq", "-f", f"name=^{CONTAINER}$"]).stdout.strip():
            raise StackError(f"a container named {CONTAINER} exists already")
        for port in (BRIDGE_PORT, soak.STUB_PORT):
            if http_ok(f"http://127.0.0.1:{port}/health"):
                raise StackError(f":{port} already answers -- someone else's service")
        self.spawn("stub", [PY, "evaluations/tiered-380/E/standin_cloud.py", str(soak.STUB_PORT),
                            str(self.out / "cloud_calls.jsonl")])
        wait(lambda: http_ok(f"http://127.0.0.1:{soak.STUB_PORT}/health"), 30, "the cloud stand-in")
        if self.perception:
            self.switch_file.write_text("")
            p = self.spawn("perception", [PY, "tools/jetson/perception_loop.py", str(6 * 3600)],
                           env=self.perception_env())
            log = self.out / "perception.log"
            wait(lambda: "loaded 1 pipeline" in log.read_text(errors="replace"), 600,
                 "perception loaded", proc=p)
            self.oom_adj(p.pid, STACK_OOM_ADJ)
        self.say(f"setup done; MemAvailable {mem_available_mb()} MB")

    # ---------------------------------------------------------------- stacks

    def up(self, house: str) -> Stack:
        self.k += 1
        d = self.out / f"s{self.k:03d}"
        env = stack_env(self.vlm_secret)
        env.update({"PYTHONPATH": str(HERE / "shim_lat"), "LAT_SHIM_LOG": str(d / "lat_shim.jsonl"),
                    "LAT_PLANT_FILE": str(self.plant_file), "ROS_BRIDGE_URL": BRIDGE})
        if self.desktop:
            env.pop("LOCAL_VLM_SECRET", None)
        st = self.stack = LatStack(d, house=house, vision_url=f"http://127.0.0.1:{soak.STUB_PORT}",
                                   extra_env=env)
        st.started = time.time()
        st.start()
        if not self.desktop:
            for p in st.procs.values():
                self.oom_adj(p.pid, STACK_OOM_ADJ)
        self.sh(["docker", "rm", "-f", CONTAINER])
        r = self.sh(["docker", "run", "-d", "--name", CONTAINER, "--network", "host",
                     "-e", "APP_SHARED_SECRET", "-e", f"ROBOT_URL={st.urls['robot']}",
                     "-e", f"BRAIN_URL={st.urls['brain']}", "-e", f"BRIDGE_PORT={BRIDGE_PORT}",
                     "-e", "BRIDGE_HOST=127.0.0.1",
                     *(["-e", f"ROS_DOMAIN_ID={ROS_DOMAIN}"] if ROS_DOMAIN else []),
                     IMAGE, "ros2", "launch", "picar_bringup", "picar.launch.py"],
                    timeout=120, env=dict(os.environ, APP_SHARED_SECRET=st.secret))
        if r.returncode != 0:
            raise StackError(f"docker run: {r.stderr.strip()[:300]}")
        wait(lambda: http_ok(BRIDGE + "/health"), 180, "the ROS bridge")
        wait(lambda: '"start_truth": {' in json.dumps(
            http("GET", BRIDGE + "/slam/pose", secret=st.secret, timeout=3)[1]), 120, "SLAM's start pose")
        self.sh(["docker", "cp", str(HERE / "ros_pace.py"), f"{CONTAINER}:/tmp/ros_pace.py"])
        self.sh(["docker", "exec", "-d", CONTAINER, "/entrypoint.sh", "python3", "/tmp/ros_pace.py",
                 "/tmp/pace.jsonl"])
        self.xlog = ExploreLog(d)
        self.person = open(d / "person.jsonl", "a", buffering=1)
        self.say(f"stack s{self.k:03d} up in {house}; MemAvailable {mem_available_mb()} MB")
        return st

    def down(self, phase: str) -> None:
        st, d = self.stack, self.out / f"s{self.k:03d}"
        rec = {"k": self.k, "dir": d.name, "phase": phase, "house": st.house if st else None,
               "t0": getattr(st, "started", None), "t1": round(time.time(), 3)}
        try:
            code, h = st.robot("GET", "/health", timeout=10)
            if code == 200 and isinstance(h, dict):
                rec["wheel_loop"] = h.get("wheel_loop")
        except Exception as e:  # noqa: BLE001
            rec["wheel_loop_error"] = str(e)
        rec["perception_alive"] = (self.helpers["perception"].poll() is None
                                   if "perception" in self.helpers else None)
        try:
            ve = http("GET", f"http://127.0.0.1:{VE_PORT}/health", timeout=5)[1]
            rec["ve_ready"] = bool(ve.get("ready")) if isinstance(ve, dict) else False
        except TRIAL_ERRORS as e:
            rec["ve_ready"], rec["ve_error"] = False, str(e)
        try:
            self.sh(["docker", "exec", CONTAINER, "pkill", "-f", "ros_pace.py"])
            time.sleep(1)
            self.sh(["docker", "cp", f"{CONTAINER}:/tmp/pace.jsonl", str(d / "pace.jsonl")])
        except Exception as e:  # noqa: BLE001
            rec["pace_error"] = str(e)
        for f in (self.xlog, self.person):
            if f is not None:
                f.close()
        self.xlog = self.person = None
        self.teardown_mission(d, st)
        self.stack = None
        self.stacks.write(json.dumps(rec) + "\n")

    def act(self, kind: str, **kw) -> None:
        """A person's action, both clocks: nav2's ends near it are the person's."""
        self.person.write(json.dumps({"k": kind, "t": round(time.time(), 4),
                                      "m": round(time.monotonic(), 4), **kw}) + "\n")

    # -------------------------------------------------------------- missions

    def status(self) -> dict:
        try:
            code, s = self.stack.brain("GET", "/mission/status", timeout=10)
        except Exception:  # noqa: BLE001
            return {}
        s = s if code == 200 and isinstance(s, dict) else {}
        if s and self.xlog is not None:
            self.xlog.take(s)
        return s

    def start(self, target: str) -> None:
        if self.perception and target != self.target:
            self.next_target(target)
        self.target = target
        self.xlog.new_mission()
        code, body = self.stack.brain("POST", "/mission/start",
                                      {"target_object": target, "policy": "tiered"}, timeout=600)
        if code != 200:
            raise StackError(f"mission start: {code} {str(body)[:300]}")

    def run_mission(self, target: str, limit_s: float) -> dict:
        t0 = time.time()
        self.start(target)
        s, stopped = {}, False
        while True:
            time.sleep(1.0)
            if time.time() - t0 > limit_s and not stopped:
                try:
                    self.stack.brain("POST", "/mission/stop", {}, timeout=10)
                    stopped = True
                except TRIAL_ERRORS as e:          # tried again on the next pass
                    self.say(f"the cap's Stop: {e}")
            if time.time() - t0 > limit_s + 120:
                raise StackError("the mission did not end 120 s after its stop")
            s = self.status()
            if not s and not self.stack.alive("brain"):
                raise StackError("the brain died mid-mission")
            if s and not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
                break
        for _ in range(10):          # the end's own goal cancel (amendment 6's record)
            time.sleep(1.0)
            if (self.status().get("explore") or {}).get("goal_end_done") is not False:
                break
        return {"t0": round(t0, 3), "t1": round(time.time(), 3), "outcome": s.get("outcome"),
                "stopped_by_harness": stopped, "verdicts": s.get("arrival_verdicts") or []}

    def ensure_mission(self, i: int) -> bool:
        """A mission running (started if none is); False if the status read
        failed or the start was refused (the trial is skipped, not the run)."""
        s = self.status()
        if not s:
            return False
        if not s.get("running"):
            try:
                self.start(P65_TARGETS[i % len(P65_TARGETS)])
            except TRIAL_ERRORS as e:
                self.say(f"mission start: {e}")
                return False
        return True

    def end_mission(self) -> bool:
        """The mission stopped and ended (P6's verb is a person's alone)."""
        try:
            self.stack.brain("POST", "/mission/stop", {}, timeout=10)
        except TRIAL_ERRORS:
            pass
        end = time.monotonic() + 30
        while time.monotonic() < end:
            s = self.status()
            if s and not s.get("running"):
                return True
            time.sleep(0.5)
        return False

    @staticmethod
    def counted(verdicts: list) -> list:
        return [v for v in verdicts if isinstance(v.get("verdict_s"), (int, float))]

    # ------------------------------------------------------------------- P7

    def p7(self, f, n: int, tag: str) -> int:
        got, m = 0, 0
        if self.desktop:
            st_house, target = "procthor:val:0", "red backpack"
            if self.stack is None:
                self.up(st_house)
            while got < n and m < self.p7_missions:
                m += 1
                try:
                    rec = self.run_mission(target, self.p7_limit_s)
                except TRIAL_ERRORS as e:
                    self.say(f"P7 desktop mission {m}: {e}")
                    continue
                for v in self.counted(rec["verdicts"]):
                    f.write(json.dumps({"tag": tag, "stack": self.k, "house": st_house,
                                        "target": target, **v, "outcome": rec["outcome"]}) + "\n")
                    got += 1
            return got
        while got < n and m < self.p7_missions:
            house, target = QUICK[m % len(QUICK)]
            m += 1
            try:
                self.up(house)
                rec = self.run_mission(target, self.p7_limit_s)
                for v in self.counted(rec["verdicts"]):
                    f.write(json.dumps({"tag": tag, "stack": self.k, "house": house,
                                        "target": target, **v, "outcome": rec["outcome"]}) + "\n")
                    got += 1
                self.say(f"P7 s{self.k:03d} {house}/{target} -> {rec['outcome']}, "
                         f"{len(rec['verdicts'])} verdict(s); {got}/{n}")
            except TRIAL_ERRORS as e:
                self.say(f"P7 s{self.k:03d}: {e}")
            finally:
                if self.stack is not None:
                    self.down("p7")
        return got

    # ------------------------------------------------------------- P6 / P5

    def now(self) -> dict:
        try:
            got = self.stack.now()
        except TRIAL_ERRORS:
            return {}
        return got if isinstance(got, dict) else {}

    def moving_forward(self, hold_s: float = 0.3, within_s: float = 5.0) -> bool:
        end, since = time.monotonic() + within_s, None
        while time.monotonic() < end:
            sp = self.now().get("sp") or [0, 0]
            if all(v > 0.02 for v in sp):
                since = since or time.monotonic()
                if time.monotonic() - since >= hold_s:
                    return True
            else:
                since = None
            time.sleep(0.02)
        return False

    def ahead_cm(self) -> float:
        """The true travel to contact ahead; 0 when it cannot be read (not clear)."""
        try:
            code, a = http("GET", self.stack.urls["body"] + "/lat/ahead", timeout=3)
        except TRIAL_ERRORS:
            return 0.0
        if code != 200 or not isinstance(a, dict):
            return 0.0
        return math.inf if a.get("T_cm") is None else a["T_cm"]

    def face_clear(self) -> bool:
        """Turn in place (a person's 45 deg LEFT) until CLEAR_CM is free ahead."""
        for _ in range(9):
            if self.ahead_cm() >= CLEAR_CM:
                return True
            self.act("verb", action="LEFT")
            try:
                self.stack.robot("POST", "/action", {"action": "LEFT", "angle": 45},
                                 driver="twin-dpad", timeout=15)
            except TRIAL_ERRORS:
                pass
            time.sleep(0.5)
        return self.ahead_cm() >= CLEAR_CM

    def goal(self) -> dict:
        try:
            return (self.stack.robot("GET", "/world/goal", timeout=3)[1] or {}).get("goal") or {}
        except Exception:  # noqa: BLE001
            return {}

    def stop_now(self, tm: Optional[float] = None) -> dict:
        """A person's Stop. A reply lost or late is a reading (the trial
        stands: its wheels and goal are read from the truth and the poller)."""
        tm, tw = tm or time.monotonic(), time.time()
        self.act("stop")
        try:
            code, _ = self.stack.robot("POST", "/stop", {}, timeout=10)
        except TRIAL_ERRORS as e:
            code = f"error: {e}"
        return {"t_stop_m": round(tm, 4), "t_stop_w": round(tw, 4),
                "stop_reply_s": round(time.monotonic() - tm, 4), "stop_status": code}

    def p6_goal(self, f, n: int, tag: str) -> int:
        """A person's Stop while the mission's nav2 goal is active and the
        wheels turn. The 25 ms poller runs first and must have read the goal
        active; the setpoints are read just before the Stop goes out; a trial
        that is not so is retried, not counted."""
        got, tries = 0, 0
        while got < n and tries < 4 * n:
            tries += 1
            try:
                if not self.ensure_mission(tries):
                    time.sleep(2.0)
                    continue
                end = time.monotonic() + 120
                ready = False
                while time.monotonic() < end:
                    g = self.goal()
                    sp = self.now().get("sp") or [0, 0]
                    if g.get("state") == "active" and any(abs(v) > 0.02 for v in sp):
                        ready = True
                        break
                    time.sleep(0.1)
                if not ready:
                    continue
                seen, done, t_ref = {}, threading.Event(), {}
                active_read = threading.Event()

                def poll():
                    while not done.is_set():
                        g = self.goal()
                        st = g.get("state") if g else None
                        t = time.monotonic()
                        if st is None:            # a failed read says nothing
                            time.sleep(POLL_GOAL_S)
                            continue
                        if st == "active":
                            active_read.set()
                            t_ref.setdefault("gid", g.get("goal_id") or g.get("id"))
                        if "t" in t_ref:
                            seen.setdefault(str(st), round(t - t_ref["t"], 4))
                            if st != "active":
                                return
                            if t - t_ref["t"] > 5:
                                return
                        time.sleep(POLL_GOAL_S)
                th = threading.Thread(target=poll, daemon=True)
                th.start()
                if not active_read.wait(3.0):
                    done.set()
                    continue
                sp = self.now().get("sp")
                if not sp or not any(abs(v) > 0.02 for v in sp):
                    done.set()
                    continue
                t_ref["t"] = time.monotonic()           # the poller times from here
                row = self.stop_now(t_ref["t"])
                th.join(7)
                done.set()
                ended = min((v for k, v in seen.items() if k != "active"), default=None)
                f.write(json.dumps({"tag": tag, "kind": "goal", "stack": self.k,
                                    "goal_id": t_ref.get("gid"), "sp_at_stop": sp, **row,
                                    "goal_states": seen, "goal_end_s": ended}) + "\n")
                got += 1
                time.sleep(2.0)
            except TRIAL_ERRORS as e:
                self.say(f"P6 goal try {tries}: {e}")
                time.sleep(2.0)
        return got

    def p6_verb(self, f, n: int, tag: str) -> int:
        """A person's Stop during a person's FORWARD: the mission is stopped
        first, so the wheels turning are the verb's alone."""
        got, tries = 0, 0
        while got < n and tries < 4 * n:
            tries += 1
            try:
                if not self.ensure_mission(tries):
                    time.sleep(2.0)
                    continue
                time.sleep(8.0)                      # the mission's load under way
                self.act("mission_stop")
                if not self.end_mission():
                    continue
                if not self.face_clear():
                    continue
                if any(abs(v) > 0.02 for v in self.now().get("sp") or [1]):
                    continue                         # something else still drives
                box: dict = {}

                def verb():
                    try:
                        box["reply"] = self.stack.robot(
                            "POST", "/action", {"action": "FORWARD", "speed": 50, "duration": 3.0},
                            driver="twin-dpad", timeout=15)
                    except TRIAL_ERRORS as e:
                        box["reply"] = (None, str(e))
                self.act("verb", action="FORWARD")
                th = threading.Thread(target=verb, daemon=True)
                th.start()
                if not self.moving_forward():
                    th.join(15)
                    continue
                sp = self.now().get("sp")
                row = self.stop_now()
                th.join(15)
                reply = box.get("reply") or (None, None)
                f.write(json.dumps({"tag": tag, "kind": "verb", "stack": self.k, "sp_at_stop": sp,
                                    **row, "verb_status": reply[0],
                                    "verb_reason": (reply[1] or {}).get("reason")
                                    if isinstance(reply[1], dict) else str(reply[1])[:200]}) + "\n")
                got += 1
                time.sleep(2.0)
            except TRIAL_ERRORS as e:
                self.say(f"P6 verb try {tries}: {e}")
                time.sleep(2.0)
        return got

    def p5(self, f, n: int, tag: str) -> int:
        """The mission's own nav2 drive is the moving robot (under `drive:
        ros` only `ros` writes the wheels: a person's `/wheels` is refused).
        A drop whose setpoints (read in the drop's own instant) were not
        forward is put back and retried, not counted."""
        got, tries = 0, 0
        while got < n and tries < 4 * n:
            tries += 1
            try:
                if not self.ensure_mission(tries):
                    time.sleep(2.0)
                    continue
                drop, why = None, ""
                q = urllib.parse.urlencode({"lo": DROP_LO, "hi": DROP_HI, "clear": CLEAR_CM,
                                            "avoid": self.target})
                end = time.monotonic() + 120
                while time.monotonic() < end and drop is None:
                    if not self.moving_forward(hold_s=0.5, within_s=10.0):
                        why = "never moving forward"
                        self.ensure_mission(tries)
                        continue
                    code, body = http("POST", self.stack.urls["body"] + "/lat/drop?" + q, timeout=3)
                    if code == 200 and isinstance(body, dict):
                        drop = body
                    else:
                        why = str(body)[:120]
                        time.sleep(0.03)
                if drop is None:
                    self.say(f"P5 try {tries}: no drop ({why})")
                    continue
                self.act("drop", until=round(time.time() + 2.5, 3))
                time.sleep(2.5)
                restored = False
                for _ in range(5):
                    code, _ = http("POST", self.stack.urls["body"] + "/lat/restore",
                                   {"src": drop["src"], "dst": drop["dst"]}, timeout=5)
                    if code == 200:
                        restored = True
                        break
                    time.sleep(1.0)
                moving = (sum(drop.get("sp") or [0, 0]) / 2) > FWD_AT_DROP_M_S
                f.write(json.dumps({"tag": tag, "stack": self.k, "drop": drop, "moving": moving,
                                    "restored": restored}) + "\n")
                if not restored:
                    raise StackError(f"the dropped object at {drop['dst']} could not be put back")
                got += moving
                time.sleep(3.0)
            except TRIAL_ERRORS as e:
                self.say(f"P5 try {tries}: {e}")
                if "could not be put back" in str(e):
                    raise
                time.sleep(2.0)
        return got

    # ------------------------------------------------------------------ run

    def set_plant(self, words: str) -> None:
        self.plant_file.write_text(words + "\n")

    def run(self) -> int:
        try:
            self.setup()
            (self.out / "run_start").write_text(f"{time.time()}\n")
            arms = [("base", "")] + ([("plant", None)] if self.plant else [])
            with open(self.out / "p7.jsonl", "a", buffering=1) as f:
                for tag, _ in arms:
                    self.set_plant("verdict" if tag == "plant" else "")
                    self.say(f"P7 {tag}: {self.p7(f, self.n7, tag)} verdicts")
            self.set_plant("")
            if self.stack is not None:
                self.down("p7")
            if self.n6 or self.n5:
                self.up(P65_HOUSE)
                try:
                    with open(self.out / "p6.jsonl", "a", buffering=1) as f6, \
                            open(self.out / "p5.jsonl", "a", buffering=1) as f5:
                        for tag, _ in arms:
                            self.set_plant("stop" if tag == "plant" else "")
                            self.say(f"P6 goal {tag}: {self.p6_goal(f6, self.n6, tag)}")
                            self.say(f"P6 verb {tag}: {self.p6_verb(f6, self.n6, tag)}")
                            self.set_plant("zero" if tag == "plant" else "")
                            self.say(f"P5 {tag}: {self.p5(f5, self.n5, tag)}")
                        self.set_plant("")
                finally:
                    try:
                        self.stack.brain("POST", "/mission/stop", {}, timeout=10)
                    except TRIAL_ERRORS:
                        pass
                    time.sleep(3)
                    self.down("p65")
            (self.out / "run_end").write_text(f"{time.time()}\n")
            return 0
        finally:
            self.set_plant("")
            self.teardown()

    def teardown(self) -> None:
        if self.stack is not None:
            try:
                self.down("teardown")
            except Exception as e:  # noqa: BLE001
                self.say(f"teardown stack: {e}")
                try:
                    self.stack.stop()
                except Exception:  # noqa: BLE001
                    pass
                self.stack = None
        self.docker_rm()
        if not self.desktop:
            try:
                (self.out / "ve_after.json").write_text(json.dumps(self.ve_state(), indent=1))
            except Exception as e:  # noqa: BLE001
                self.say(f"ve_after: {e}")
        for name, p in self.helpers.items():
            if p.poll() is not None:
                continue
            try:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=20)
            except (ProcessLookupError, PermissionError, subprocess.TimeoutExpired):
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        self.say("torn down")

    def docker_rm(self) -> None:
        try:
            self.sh(["docker", "rm", "-f", CONTAINER])
        except (OSError, subprocess.SubprocessError) as e:
            self.say(f"docker rm: {e}")

    def teardown_mission(self, mdir: Path, st) -> None:
        mdir.mkdir(parents=True, exist_ok=True)
        try:
            with open(mdir / "ros.log", "w") as f:
                subprocess.run(["docker", "logs", CONTAINER], stdout=f, stderr=subprocess.STDOUT,
                               timeout=60)
        except (OSError, subprocess.SubprocessError) as e:
            self.say(f"docker logs: {e}")
        try:
            self.docker_rm()
        finally:
            if st is not None:
                st.stop()
            gz(mdir / "ros.log")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--desktop", action="store_true", help="a VE stand-in, no perception loop")
    ap.add_argument("--plant", action="store_true", help="criterion 3: each row again, 100 ms planted")
    ap.add_argument("--p7", type=int, default=20)
    ap.add_argument("--p6", type=int, default=20)
    ap.add_argument("--p5", type=int, default=40)
    ap.add_argument("--p7-limit-s", type=float, default=600.0)
    ap.add_argument("--p7-missions", type=int, default=40)
    a = ap.parse_args(argv)

    def on_signal(signum, _frame):
        raise SystemExit(128 + signum)
    for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(s, on_signal)
    return Lat(Path(a.out), desktop=a.desktop, plant=a.plant, n7=a.p7, n6=a.p6, n5=a.p5,
               p7_limit_s=a.p7_limit_s, p7_missions=a.p7_missions).run()


if __name__ == "__main__":
    raise SystemExit(main())
