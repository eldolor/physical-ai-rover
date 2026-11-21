# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/sim-chaos-3202/chaos.py

3.202 (S7): the chaos soak in the simulator. ONE stack for the whole run
(3.135's `stack.py`: the split body with its ground-truth recorder, two
sensor servers, the fake D500 on a pty, the shipped robot server and brain),
the brain reaching the robot through `fault_proxy.py`, the cloud a local
$0 stand-in (`hang_cloud.py`, only while K10 runs; a closed port otherwise).
Missions run back to back; a fault kind is drawn (seeded, uniform over
`--kinds`) and fired 2-4 minutes after the previous fault's end, until
`--faults` have fired. What each kind does and its bar: the section file
(3.202-sim-chaos-soak.md, "The faults").

    python evaluations/sim-chaos-3202/chaos.py <out> [--faults 40] [--seed 3202]
        [--house starter_house] [--kinds K1,K2,...] [--gap 120,240] [--plant watchdog]
        [--stop-last]

The run only records; `score.py <out>` judges it. What lands in <out>:
  run.json        the arguments, ports, config values the bars read
  faults.jsonl    one line per fault: kind, stamps (monotonic), mission ids
  missions.jsonl  one line per mission: id, policy, stamps, end status, tags
  rss.jsonl       every 5 s: each stack role's pid and VmRSS (kB), OOM kills
  truth.jsonl     the body's ground truth (20 ms), the whole run
  proxy.jsonl     every request through the proxy
  *.log           each role's log
"""

from __future__ import annotations

import argparse
import json
import os
import random
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FMEA = ROOT / "evaluations" / "jetson-fmea-3135"
sys.path.insert(0, str(FMEA))
sys.path.insert(0, str(HERE))

import measure as m  # noqa: E402
from fmea import robot_health, status, wait_forward  # noqa: E402
from stack import PORTS, PY, Stack, StackError, http  # noqa: E402

PROXY_PORT = PORTS["robot"] + 6          # this section's block of ten, past the stack's five
STUB_PORT = PORTS["robot"] + 7
PROXY_URL = f"http://127.0.0.1:{PROXY_PORT}"
STUB_URL = f"http://127.0.0.1:{STUB_PORT}"

MID_MOVE = ("K1", "K2", "K3", "K4", "K5", "K6")
WHOLE = ("K7", "K8", "K9", "K10", "K11")
KINDS = MID_MOVE + WHOLE
TARGET = "red backpack"
MISSION_LIMIT_S = 1260.0      # criterion 4(c), amendment 1: the brain's own 1200 s limit + 60 s
RECOVERY_BAR_S = 90.0         # criterion 3 (judged in score.py; the wait runs a little past it)
FIRE_TRIES = 10               # criterion 1: a FORWARD that lasts 0.15 s, in 10 tries
HOLD_S = {"K2": 30.0, "K4": 10.0, "K5": 5.0, "K6": 10.0}
WATCH_S = 6.0
K7_LINK = {"latency_ms": 200, "jitter_ms": 50, "drop": 0.05}


def now() -> float:
    return time.monotonic()


class Jsonl:
    def __init__(self, path: Path):
        self.f = open(path, "a", buffering=1)
        self.lock = threading.Lock()

    def write(self, rec: dict) -> None:
        with self.lock:
            self.f.write(json.dumps(rec, default=str) + "\n")

    def close(self) -> None:
        self.f.close()


class RssSampler:
    """Every 5 s: VmRSS of each stack role (and the proxy), the OOM counter."""

    def __init__(self, st: Stack, extra: dict, out: Jsonl, period_s: float = 5.0):
        self.st, self.extra, self.out, self.period = st, extra, out, period_s
        self.mission: Optional[str] = None
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    @staticmethod
    def rss_kb(pid: int) -> Optional[int]:
        try:
            with open(f"/proc/{pid}/status") as f:
                for line in f:
                    if line.startswith("VmRSS:"):
                        return int(line.split()[1])
        except OSError:
            return None
        return None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:                       # the main thread changes both dicts: copy, and never die
                roles = {r: p.pid for r, p in list(self.st.procs.items()) if p.poll() is None}
                roles.update({r: p.pid for r, p in list(self.extra.items()) if p.poll() is None})
                try:
                    oom = next(int(l.split()[1]) for l in open("/proc/vmstat")
                               if l.startswith("oom_kill "))
                except (OSError, StopIteration):
                    oom = None
                self.out.write({"t": round(now(), 3), "mission": self.mission, "oom_kill": oom,
                                "rss": {r: [pid, self.rss_kb(pid)] for r, pid in roles.items()}})
            except Exception as e:     # noqa: BLE001 -- a missed sample, recorded
                self.out.write({"t": round(now(), 3), "sampler_error": f"{type(e).__name__}: {e}"})
            self._stop.wait(self.period)

    def start(self) -> "RssSampler":
        self._t.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._t.join(timeout=10)


class Chaos:
    def __init__(self, out: Path, *, faults: int, seed: int, house: str, kinds: tuple,
                 gap: tuple, plant: str = "", hard_cap_s: float = 6 * 3600,
                 stop_last: bool = False):
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.n_faults, self.house, self.kinds, self.gap = faults, house, kinds, gap
        self.plant, self.hard_cap_s, self.stop_last = plant, hard_cap_s, stop_last
        self.rng = random.Random(seed)
        self.seed = seed
        self.missions = Jsonl(self.out / "missions.jsonl")
        self.faults = Jsonl(self.out / "faults.jsonl")
        self.events = Jsonl(self.out / "events.jsonl")
        self.cur: Optional[dict] = None          # the current mission's record
        self.fired = 0
        self.unfired = 0
        self.extra: dict = {}                    # proxy, stub: this harness's own processes
        self.st: Optional[Stack] = None
        self.sampler: Optional[RssSampler] = None

    # ------------------------------------------------------------ helpers

    def note(self, what: str, **kw) -> None:
        self.events.write({"t": round(now(), 3), "what": what, **kw})

    def _spawn_extra(self, role: str, argv: list, env: Optional[dict] = None) -> subprocess.Popen:
        log = open(self.out / f"{role}.log", "ab")
        p = subprocess.Popen(argv, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True, env=env or dict(os.environ))
        self.extra[role] = p
        return p

    def _stop_extra(self, role: str) -> None:
        p = self.extra.pop(role, None)
        if p is not None and p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass

    def proxy(self, **faults) -> dict:
        code, body = http("POST", PROXY_URL + "/__fault", faults, timeout=5)
        if code != 200:
            raise StackError(f"the proxy refused {faults}: {code} {body}")
        self.note("proxy", faults=body.get("faults"))
        return body

    def proxy_clear(self) -> dict:
        return self.proxy(latency_ms=0, jitter_ms=0, drop=0.0, dead=False, dup_action=False)

    def start_stub(self) -> None:
        self._spawn_extra("stub", [PY, str(FMEA / "hang_cloud.py"), str(STUB_PORT),
                                   str(self.out / "cloud_calls.jsonl")])
        end = now() + 10
        import socket
        while now() < end:
            with socket.socket() as s:
                if s.connect_ex(("127.0.0.1", STUB_PORT)) == 0:
                    return
            time.sleep(0.1)
        raise StackError("the hanging cloud stand-in never listened")

    # ----------------------------------------------------------- missions

    def start_mission(self, policy: str = "frontier", tags: Optional[list] = None,
                      fault: str = "") -> dict:
        body = {"target_object": TARGET, "policy": policy}
        if fault:
            body["fault"] = fault
        t0 = now()
        code, got = self.st.brain("POST", "/mission/start", body, timeout=30)
        if code != 200:
            raise StackError(f"mission start: {code} {str(got)[:300]}")
        mid = (got.get("status") or {}).get("mission_id") if isinstance(got, dict) else None
        end = now() + 30
        while mid is None and now() < end:      # the start answer should name it; else ask
            mid = status(self.st).get("mission_id")
            if mid is None:
                time.sleep(0.5)
        if mid is None:
            raise StackError("the started mission has no mission_id after 30 s")
        rec = {"id": mid, "policy": policy, "t0": round(t0, 3),
               "tags": list(tags or []), "house": self.house}
        self.cur = rec
        if self.sampler:
            self.sampler.mission = rec["id"]
        self.note("mission_start", id=rec["id"], policy=policy, tags=rec["tags"])
        return rec

    def end_mission(self, s: dict, how: str) -> None:
        rec = self.cur
        if rec is None:
            return
        rec.update({"t1": round(now(), 3), "how": how,
                    **{k: s.get(k) for k in ("outcome", "end_state", "end_reason", "cancelled_by",
                                             "step", "error", "found", "mission_id")}})
        n = self.st.now() if self.st else None
        if isinstance(n, dict):
            rec["end_truth"] = {k: n.get(k) for k in ("t", "x", "y", "th")}
        self.missions.write(rec)
        self.cur = None
        if self.sampler:
            self.sampler.mission = None

    def poll_end(self) -> Optional[dict]:
        """The current mission's status once it has ended (None while it runs)."""
        if self.cur is None:
            return None
        s = status(self.st)
        if s.get("mission_id") not in (None, self.cur["id"]):
            self.stop_mission()
            return {"outcome": "lost", "error": f"status names mission {s.get('mission_id')}"}
        if s.get("running") is False and s.get("outcome") not in (None, "idle", "running"):
            return s
        if now() - self.cur["t0"] > MISSION_LIMIT_S:
            self.cur["hung"] = True
            self.stop_mission()
            return {**status(self.st), "outcome_before_stop": s.get("outcome"), "hung": True}
        return None

    def wait_mission(self, limit_s: float = MISSION_LIMIT_S + 60) -> dict:
        end = now() + limit_s
        while now() < end:
            s = self.poll_end()
            if s is not None:
                return s
            time.sleep(1.0)
        return {"outcome": "harness_timeout"}

    def stop_mission(self) -> None:
        try:
            if status(self.st).get("running"):
                self.st.brain("POST", "/mission/stop", {}, timeout=10)
                end = now() + 45                # the brain's own close waits are 10 s + 30 s
                while status(self.st).get("running") and now() < end:
                    time.sleep(0.5)
        except OSError as e:
            self.note("stop_error", error=str(e))

    # ------------------------------------------------------------ recovery

    def recover(self, frec: dict, cleared_at: float) -> None:
        """Criterion 3: restart what the fault killed (systemd's job on the
        board), end the interrupted mission, and get a new one driving. The
        new mission becomes the run's current mission."""
        try:
            for role, starter in (("robot", self.st.start_robot), ("brain", self.st.start_brain)):
                if not self.st.alive(role):
                    starter()
                    frec.setdefault("restarted", []).append(role)
            if self.cur is not None:
                s = status(self.st)
                if s.get("running"):
                    self.stop_mission()
                    s = {**status(self.st), "stopped_by_harness": True}
                self.end_mission(s, "fault")
            self.st.robot("POST", "/autonomy/resume", {})
            self.start_mission("frontier", tags=["recovery"])
            frec["recovery_mission"] = self.cur["id"]
            wait_forward(self.st, max(5.0, cleared_at + RECOVERY_BAR_S + 30 - now()))
            frec["recovery_s"] = round(now() - cleared_at, 2)
        except Exception as e:                     # noqa: BLE001 -- recovery is scored, not fatal
            frec["recovery_error"] = f"{type(e).__name__}: {e}"

    # --------------------------------------------------------------- faults

    def fire_mid_move(self, kind: str) -> Optional[dict]:
        """Wait for a FORWARD still on 0.15 s in, inside the current mission,
        and fire. None (unfired) if the mission ends or no FORWARD lasts."""
        for _ in range(FIRE_TRIES):
            if self.cur is None:
                self.start_mission("frontier")
            try:
                wait_forward(self.st, 120.0)
            except StackError:
                s = self.poll_end()
                if s is not None:
                    self.end_mission(s, "hung" if s.get("hung") else "natural")
                continue
            time.sleep(0.15)
            n = self.st.now()
            if isinstance(n, dict) and m.is_forward(n.get("sp")):
                break
        else:
            return None
        frec = {"kind": kind, "mission": self.cur["id"] if self.cur else None,
                "health_before": _health_counts(robot_health(self.st))}
        self.cur.setdefault("tags", []).append(kind)
        hold = HOLD_S.get(kind, 0.0)
        if kind == "K1":
            t = self.st.signal("brain", signal.SIGKILL)
            cleared = t
        elif kind == "K2":
            t = self.st.signal("brain", signal.SIGSTOP)
            time.sleep(hold)
            cleared = frec["cont_at"] = self.st.signal("brain", signal.SIGCONT)
        elif kind == "K3":
            t = self.st.signal("robot", signal.SIGKILL)
            cleared = t
        elif kind == "K4":
            t = self.st.signal("lidar", signal.SIGSTOP)
            time.sleep(hold)
            cleared = frec["cont_at"] = self.st.signal("lidar", signal.SIGCONT)
        elif kind == "K5":
            code, body = http("POST", f"{self.st.urls['body']}/fmea/silence?s={hold}")
            if code != 200 or not isinstance(body, dict):
                raise StackError(f"the body refused the silence: {code} {str(body)[:200]}")
            t = body["silent_from"]
            time.sleep(hold + 0.2)
            cleared = frec["cont_at"] = body["silent_until"]
        elif kind == "K6":
            t = now()
            self.proxy(dead=True)
            time.sleep(hold)
            self.proxy(dead=False)
            cleared = frec["cont_at"] = now()
        else:
            raise ValueError(kind)
        frec["fault_at"] = round(t, 4)
        frec["cleared_at"] = round(cleared, 4)
        # The watch window the bars read (as fmea.py), measured from the fault.
        time.sleep(max(0.0, t + WATCH_S - now()) if kind in ("K1", "K3", "K6") else 1.0)
        if kind == "K2":
            time.sleep(max(0.0, cleared + 10.5 - now()))     # "no drive in the 10 s after SIGCONT"
        if kind == "K3":
            time.sleep(4.0)                                  # the brain finds the robot gone
        s = status(self.st)
        frec["status_after"] = {k: s.get(k) for k in ("running", "outcome", "end_state", "end_reason",
                                                      "mission_id", "error")}
        frec["health_after"] = _health_counts(robot_health(self.st))
        self.recover(frec, cleared)
        return frec

    def run_whole(self, kind: str) -> dict:
        """A fault that spans one whole mission (K7-K11)."""
        if self.cur is not None:                     # amendment 1: preempted, not waited for
            s0 = self.poll_end()
            if s0 is None:
                self.cur.setdefault("tags", []).append("preempted")
                self.stop_mission()
                s0 = status(self.st)
                self.end_mission(s0, "preempted")
            else:
                self.end_mission(s0, "hung" if s0.get("hung") else "natural")
        frec = {"kind": kind}
        policy, fault = "frontier", ""
        if kind == "K7":
            self.proxy(**K7_LINK, seed=self.rng.randrange(1 << 30))
        elif kind == "K8":
            self.proxy(dup_action=True)
        elif kind == "K9":
            policy = "vision"                        # the stub is not running: refused
        elif kind == "K10":
            policy = "vision"
            self.start_stub()
        elif kind == "K11":
            fault = "tick_hang"
        frec["proxy_before"] = http("GET", PROXY_URL + "/__fault", timeout=5)[1]
        frec["health_before"] = _health_counts(robot_health(self.st))
        try:
            self.start_mission(policy, tags=[kind], fault=fault)
        except Exception:
            self.proxy_clear()
            self._stop_extra("stub")
            raise
        frec["mission"] = self.cur["id"]
        frec["fault_at"] = self.cur["t0"]
        if kind == "K11":                            # the stamp the K11 bar reads from
            end = now() + 60
            while now() < end:
                st = status(self.st)
                if (st.get("step") or 0) >= 1:
                    frec["step1_at"] = round(now(), 4)
                    break
                if st.get("running") is False:      # ended before its first step: invalid
                    frec["ended_before_step1"] = True
                    break
                time.sleep(0.1)
        s = self.wait_mission()
        frec["cleared_at"] = round(now(), 4)
        frec["proxy_after"] = http("GET", PROXY_URL + "/__fault", timeout=5)[1]
        self.proxy_clear()
        self._stop_extra("stub")
        frec["health_after"] = _health_counts(robot_health(self.st))
        frec["status_after"] = {k: s.get(k) for k in ("running", "outcome", "end_state", "end_reason",
                                                      "mission_id", "error")}
        self.end_mission(s, "hung" if s.get("hung") else "natural")
        # The recovery for a whole-mission fault: the next mission drives.
        try:
            self.st.robot("POST", "/autonomy/resume", {})
            self.start_mission("frontier", tags=["recovery"])
            frec["recovery_mission"] = self.cur["id"]
            wait_forward(self.st, RECOVERY_BAR_S + 30)
            frec["recovery_s"] = round(now() - frec["cleared_at"], 2)
        except Exception as e:                       # noqa: BLE001
            frec["recovery_error"] = f"{type(e).__name__}: {e}"
        return frec

    def _settle(self) -> None:
        """After a harness error: links clear, stubs gone, dead roles back,
        no mission running -- so the next draw starts from a known state."""
        for step in (self.proxy_clear, lambda: self._stop_extra("stub")):
            try:
                step()
            except Exception as e:          # noqa: BLE001
                self.note("settle_error", error=f"{type(e).__name__}: {e}")
        try:
            for role, starter in (("robot", self.st.start_robot), ("brain", self.st.start_brain)):
                if not self.st.alive(role):
                    starter()
                    self.note("settle_restart", role=role)
            if self.st.alive("lidar"):
                self.st.signal("lidar", signal.SIGCONT)
            if self.st.alive("brain"):
                self.st.signal("brain", signal.SIGCONT)
            if self.cur is not None:
                self.stop_mission()
                self.end_mission(status(self.st), "harness_error")
        except Exception as e:              # noqa: BLE001
            self.note("settle_error", error=f"{type(e).__name__}: {e}")
            self.cur = None
        time.sleep(5.0)

    # ------------------------------------------------------------------ run

    def config_values(self) -> dict:
        sys.path.insert(0, str(ROOT))
        from control.brain_config import load_brain_config
        cfg = load_brain_config()
        import yaml
        robot = yaml.safe_load(open(ROOT / "config" / "robot.yaml"))
        return {k: cfg.get(k) for k in ("vision_timeout_s", "max_vision_failures", "tick_timeout_s",
                                        "drill_tick_timeout_s", "drill_vision_timeout_s",
                                        "mission_time_limit_s", "allow_drills")} | {
            "watchdog_timeout_s": (robot.get("safety") or {}).get("watchdog_timeout_s")}

    def run(self) -> int:
        t_start = now()
        self.sampler = None
        run = {"started": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "t_start": round(t_start, 3),
               "faults": self.n_faults, "seed": self.seed, "house": self.house,
               "kinds": list(self.kinds), "gap": list(self.gap), "plant": self.plant or None,
               "ports": PORTS, "proxy": PROXY_URL, "stub": STUB_URL, "host": os.uname().nodename,
               "git": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                                     text=True).stdout.strip(),
               "config": self.config_values(), "target": TARGET, "stop_last": self.stop_last,
               "planted_config": {"drill_tick_timeout_s": 600.0}
               if "dead_man" in self.plant.split(",") else None}
        (self.out / "run.json").write_text(json.dumps(run, indent=1))
        rss = Jsonl(self.out / "rss.jsonl")
        try:
            self._spawn_extra("proxy", [PY, str(HERE / "fault_proxy.py"), str(PROXY_PORT),
                                        f"http://127.0.0.1:{PORTS['robot']}"],
                              dict(os.environ, FAULT_PROXY_LOG=str(self.out / "proxy.jsonl")))
            time.sleep(0.5)
            plants = [p for p in self.plant.split(",") if p]
            stack_plant = ",".join(p for p in plants if p != "dead_man")
            self.st = Stack(self.out, house=self.house, plant=stack_plant, vision_url=STUB_URL,
                            extra_env={"ROBOT_URL": PROXY_URL, "LOCAL_VLM_URL": ""})
            if "dead_man" in plants:                 # amendment 2: this section's own brain plant
                st = self.st

                def _start_brain_planted():
                    st._spawn("brain", [PY, str(HERE / "dead_man_plant.py"), str(PORTS["brain"])])
                    st._wait_health("brain", st.urls["brain"])
                st.start_brain = _start_brain_planted
            self.st.start()
            self.proxy_clear()
            self.sampler = RssSampler(self.st, self.extra, rss).start()
            due = now() + self.rng.uniform(*self.gap)
            kind = self.rng.choice(self.kinds)
            while self.fired < self.n_faults and now() - t_start < self.hard_cap_s:
                if now() >= due:
                    self.note("fault_due", kind=kind)
                    try:
                        frec = self.fire_mid_move(kind) if kind in MID_MOVE else self.run_whole(kind)
                    except Exception as e:           # noqa: BLE001 -- the fault errors; the soak goes on
                        frec = {"kind": kind, "errored": f"{type(e).__name__}: {e}"}
                        self.note("fault_error", kind=kind, error=frec["errored"])
                        self._settle()
                    if frec is None:
                        self.unfired += 1
                        self.faults.write({"kind": kind, "unfired": True, "t": round(now(), 3)})
                    else:
                        self.fired += 1
                        frec["n"] = self.fired
                        self.faults.write(frec)
                        self.note("fault_done", kind=kind, n=self.fired,
                                  recovery_s=frec.get("recovery_s"),
                                  recovery_error=frec.get("recovery_error"))
                    due = now() + self.rng.uniform(*self.gap)
                    kind = self.rng.choice(self.kinds)
                    continue
                if self.cur is None:
                    try:
                        self.start_mission("frontier")
                    except Exception as e:           # noqa: BLE001 -- recorded; settle and retry
                        self.note("start_error", error=f"{type(e).__name__}: {e}")
                        self._settle()
                        continue
                s = self.poll_end()
                if s is not None:
                    self.end_mission(s, "hung" if s.get("hung") else "natural")
                    continue
                time.sleep(1.0)
            s_last = self.poll_end() if self.cur is not None else None
            if s_last is not None:                   # it ended on its own meanwhile
                self.end_mission(s_last, "hung" if s_last.get("hung") else "natural")
            if self.cur is not None and self.stop_last:   # criterion 7's single-fault runs
                self.stop_mission()
                self.end_mission(status(self.st), "stopped_at_end")
            elif self.cur is not None:               # the last mission runs to its end
                s = self.wait_mission()
                self.end_mission(s, "hung" if s.get("hung") else "natural")
            end = {"ended": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "t_end": round(now(), 3),
                   "fired": self.fired, "unfired": self.unfired,
                   "alive": {r: self.st.alive(r) for r in self.st.procs},
                   "health": {"robot": _ok(self.st.robot("GET", "/health", timeout=5)),
                              "brain": _ok(self.st.brain("GET", "/health", timeout=5))}}
        except BaseException as e:                    # noqa: BLE001 -- recorded, never lost (SIGTERM too)
            end = {"error": f"{type(e).__name__}: {e}", "t_end": round(now(), 3),
                   "fired": self.fired, "unfired": self.unfired}
        finally:
            if self.sampler:
                self.sampler.stop()
            try:
                if self.cur is not None:
                    self.stop_mission()
            finally:
                if self.st is not None:
                    self.st.stop()
                for role in list(self.extra):
                    self._stop_extra(role)
                rss.close()
        (self.out / "end.json").write_text(json.dumps(end, indent=1, default=str))
        return 0 if "error" not in end else 3


def _ok(resp: tuple) -> bool:
    return resp[0] == 200


def _health_counts(h: dict) -> dict:
    return {"refusal_counts": h.get("refusal_counts"), "wheel_loop": h.get("wheel_loop"),
            "last_refusal": h.get("last_refusal")}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--faults", type=int, default=40)
    ap.add_argument("--seed", type=int, default=3202)
    ap.add_argument("--house", default="starter_house")
    ap.add_argument("--kinds", default=",".join(KINDS))
    ap.add_argument("--gap", default="120,240")
    ap.add_argument("--plant", default="")
    ap.add_argument("--hard-cap-s", type=float, default=6 * 3600)
    ap.add_argument("--stop-last", action="store_true",
                    help="stop the mission running after the last fault (criterion 7's runs)")
    a = ap.parse_args(argv)
    kinds = tuple(k for k in a.kinds.split(",") if k)
    bad = set(kinds) - set(KINDS)
    if bad:
        raise SystemExit(f"unknown kinds {sorted(bad)}")
    lo, hi = (float(v) for v in a.gap.split(","))

    def _term(signum, frame):
        raise SystemExit(f"signal {signum}")
    signal.signal(signal.SIGTERM, _term)        # the stack is torn down, end.json written
    signal.signal(signal.SIGHUP, _term)
    return Chaos(Path(a.out), faults=a.faults, seed=a.seed, house=a.house, kinds=kinds,
                 gap=(lo, hi), plant=a.plant, hard_cap_s=a.hard_cap_s,
                 stop_last=a.stop_last).run()


if __name__ == "__main__":
    raise SystemExit(main())
