# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/local-vlm-3152/crash_loop.py

3.152 criterion 10 (as amendment 2 reads it), ON the board (through
`board.sh`, under the board lock and 3.136's lease): one candidate restart
delay D for `picar-local-vlm`, measured in a 15-min OOM crash loop beside
3.135.C's live stack (`Soak`: the split body, the fake board and D500, the
ROS container, YOLOE + CLIP, the $0 cloud stand-in) running tiered missions
back to back with VE as the local tier.

* The candidate is a runtime drop-in (`/run/systemd/system/...d/`, cleared
  at boot): `StartLimitIntervalSec=0`, `RestartSec=D`. The unit as found is
  recorded first, and the drop-in is taken out and the unit put back as
  found at the end, whatever happened.
* `hog.py`, its own process at `oom_score_adj` 100, grows until the kernel
  kills VE, then holds 256 MB more: each VE load is killed again. The window
  is `WINDOW_S` from when it starts holding; then it is sent SIGTERM.
* At 1 Hz: `MemAvailable`, `oom_kill`, VE's /health, the unit's
  `NRestarts`, and, while a mission runs, both servers' /health and the
  brain's `local_vlm`.
* After the window: VE must be ready within D + 60 s with no
  `reset-failed`. The 60 s after that is the memory bar's loaded window
  (>= 500 MB), and the running mission's brain must read `local_vlm` `up`.
  Then the mission is stopped and the run ends.

    python evaluations/local-vlm-3152/crash_loop.py <out> <D seconds | shipped>

`shipped` (amendment 7): no drop-in; the unit as installed, its start limit
included. VE back after the window is recorded but is not a bar for it.
"""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "evaluations/jetson-fmea-3135"))

import soak  # noqa: E402
from soak import http, http_ok, mem_available_mb  # noqa: E402
from stack import StackError  # noqa: E402

UNIT = "picar-local-vlm"
VE_MEMCG = f"{UNIT}.service"
DROPIN_DIRS = (Path(f"/etc/systemd/system/{UNIT}.service.d"),
               Path(f"/run/systemd/system/{UNIT}.service.d"))
DROPIN = DROPIN_DIRS[1] / "zz-3152-crash-loop.conf"
WINDOW_S = 15 * 60
MARGIN_MB = 256
GROW_GIVE_UP_S = 300
LOADED_S = 60                 # amendment 2: the memory bar's window after VE is back
MIN_KILLS = 3                 # amendment 2: fewer is no crash loop (invalid)
RUN_CEILING_S = 50 * 60       # the whole run; the board hold's own --max is 60 min
LOADED_MIN_ROWS = 30          # amendment 4: samples of a running mission in the loaded window
STACK_PORTS = (10350, 10351, 10352, 10353, 10354)   # stack.py's PORTS defaults       # the whole run; the board hold's own --max is 60 min
SHOW = "MainPID,NRestarts,ActiveState,SubState,Result,StartLimitIntervalUSec,StartLimitBurst,RestartUSec"


def oom_kills() -> int:
    with open("/proc/vmstat") as f:
        for line in f:
            if line.startswith("oom_kill "):
                return int(line.split()[1])
    return -1


class CrashLoop(soak.Soak):
    def __init__(self, out: Path, delay_s):
        if (out / "samples.jsonl").exists() or (out / "missions.jsonl").exists():
            raise StackError(f"{out} holds an earlier run: give each run its own directory")
        super().__init__(out, WINDOW_S + 2 * soak.MISSION_CAP_S)
        self.shipped = delay_s == "shipped"
        self.delay_s = None if self.shipped else int(delay_s)
        self.window = {"start": None, "end": None}
        self.stop_all = threading.Event()
        self.finished = threading.Event()
        self.samples = open(out / "samples.jsonl", "a", buffering=1)
        self.as_found: dict = {}
        self.hog_proc: subprocess.Popen | None = None
        self.t_begin = time.time()
        self.notes: list = []
        self.baseline_ok = False
        self.mission_live = False        # from a mission seen running until its teardown
        self.in_teardown = False
        self.main_done = threading.Event()
        self.loaded_from: float | None = None
        self.signalled = False
        self.live_lock = threading.Lock()   # mission_live and in_teardown together (review 4)
        self.teardowns = 0                  # bumped by each teardown: a sample across one is dropped

    # ------------------------------------------------------- guarded reads

    def ve_ready(self) -> bool:
        try:
            code, body = http("GET", soak.VE_URL + "/health", timeout=3)
            return code == 200 and isinstance(body, dict) and body.get("ready") is True
        except Exception:  # noqa: BLE001 -- refused while VE is down: not ready
            return False

    def brain_status(self) -> dict | None:
        st = getattr(self, "stack", None)
        if st is None:
            return None
        try:
            code, s = st.brain("GET", "/mission/status", timeout=3)
            return s if code == 200 and isinstance(s, dict) else None
        except Exception:  # noqa: BLE001 -- a brain not up yet, or gone
            return None

    def unit_show(self) -> dict:
        try:
            r = self.sh(["systemctl", "show", "-p", SHOW, UNIT], timeout=20)
            return dict(x.split("=", 1) for x in r.stdout.splitlines() if "=" in x)
        except Exception:  # noqa: BLE001
            return {}

    def dropins(self) -> list:
        return sorted(str(p) for d in DROPIN_DIRS if d.exists() for p in d.glob("*"))

    def oom_events(self) -> list | None:
        """The kernel's `oom-kill:` lines since the run began, from the journal
        with their wall times (amendment 4: dmesg's ring buffer drops early
        lines after many task dumps); None if the journal could not be read."""
        try:
            r = self.sh(["sudo", "-n", "journalctl", "-k", "-o", "json", "--no-pager",
                         "--since", f"@{int(self.t_begin) - 5}"], timeout=60)
        except Exception:  # noqa: BLE001
            return None
        if r.returncode != 0:
            return None
        # The kernel's own time of each line (review 5: journald's receive time
        # lags under an OOM storm): its monotonic stamp, put on the wall clock.
        offset = time.time() - time.clock_gettime(time.CLOCK_MONOTONIC)
        out = []
        for raw in r.stdout.splitlines():
            try:
                e = json.loads(raw)
            except ValueError:
                continue
            msg = e.get("MESSAGE")
            if not isinstance(msg, str) or "oom-kill:" not in msg:
                continue
            try:
                t = int(e["_SOURCE_MONOTONIC_TIMESTAMP"]) / 1e6 + offset
            except (KeyError, ValueError, TypeError):
                t = None
            out.append((t, msg))
        return out

    # ------------------------------------------------------------ the unit

    def record_as_found(self) -> None:
        found = {"show": self.unit_show(),
                         "cat": self.sh(["systemctl", "cat", UNIT]).stdout,
                         "dropins": self.dropins(),
                         "active": self.sh(["systemctl", "is-active", UNIT]).stdout.strip(),
                         "enabled": self.sh(["systemctl", "is-enabled", UNIT]).stdout.strip()}
        (self.out / "unit_as_found.json").write_text(json.dumps(found, indent=1))
        if str(DROPIN) in found["dropins"]:
            raise StackError(f"{DROPIN} exists already: a run before this one was not put back")
        if found["active"] != "active":
            raise StackError(f"{UNIT} is {found['active']} as found, not active")
        # Set only now (review 3): a refused run changes nothing in put_back.
        self.as_found = found

    def refuse_busy(self) -> None:
        """Soak.setup's and stack.py's refusals, made before board.sh's net is
        armed (review 5): a container of that name, or anything on the ports
        the soak and its mission stack use, is someone else's."""
        if self.sh(["docker", "ps", "-aq", "-f", f"name=^{soak.CONTAINER}$"]).stdout.strip():
            raise StackError(f"a container named {soak.CONTAINER} exists already")
        for port in (8090, soak.STUB_PORT) + STACK_PORTS:
            if self.sh(["ss", "-ltnH", f"sport = :{port}"]).stdout.strip():
                raise StackError(f":{port} is already served -- someone else's service")

    def install_dropin(self) -> None:
        if self.shipped:
            show = self.unit_show()
            (self.out / "unit_candidate.json").write_text(json.dumps(show, indent=1))
            self.say(f"shipped unit, no drop-in: {show}")
            return
        body = ("# 3.152 criterion 10: a runtime crash-loop candidate; removed at the run's end.\n"
                "[Unit]\nStartLimitIntervalSec=0\n[Service]\n"
                f"RestartSec={self.delay_s}\n")
        self.sh(["sudo", "-n", "mkdir", "-p", str(DROPIN.parent)], check=True)
        r = subprocess.run(["sudo", "-n", "tee", str(DROPIN)], input=body, text=True,
                           capture_output=True, timeout=20)
        if r.returncode != 0:
            raise StackError(f"writing the drop-in: {r.stderr.strip()[:200]}")
        self.sh(["sudo", "-n", "systemctl", "daemon-reload"], check=True)
        show = self.unit_show()
        (self.out / "unit_candidate.json").write_text(json.dumps(show, indent=1))
        want = {f"{self.delay_s}s", f"{self.delay_s // 60}min"}
        if show.get("StartLimitIntervalUSec") not in ("0", "infinity") or show.get("RestartUSec") not in want:
            raise StackError(f"the candidate did not load: {show}")
        self.say(f"candidate loaded: {show}")

    def put_back(self) -> None:
        """Every step on its own, so one that fails never skips the rest:
        the drop-in out, the counter reset, VE started (it was active as
        found), then the unit read back against as found."""
        if not self.as_found:
            # Nothing was recorded, so nothing was changed: never start or reset
            # a VE that was not active as found (review 2).
            return
        steps = []
        for argv in (["sudo", "-n", "rm", "-f", str(DROPIN)],
                     ["sudo", "-n", "systemctl", "daemon-reload"],
                     ["sudo", "-n", "systemctl", "reset-failed", UNIT]):
            try:
                steps.append([argv[-1], self.sh(argv, timeout=60).returncode])
            except Exception as e:  # noqa: BLE001
                steps.append([argv[-1], repr(e)])
        try:
            if self.sh(["systemctl", "is-active", UNIT], timeout=20).stdout.strip() != "active":
                steps.append(["start", self.sh(["sudo", "-n", "systemctl", "start", UNIT],
                                               timeout=120).returncode])
        except Exception as e:  # noqa: BLE001
            steps.append(["start", repr(e)])
        end = time.time() + 180
        while time.time() < end and not self.ve_ready():
            time.sleep(2)
        show = self.unit_show()
        back = {"steps": steps, "show": show, "dropins": self.dropins(), "ve_ready": self.ve_ready()}
        try:
            back["active"] = self.sh(["systemctl", "is-active", UNIT], timeout=20).stdout.strip()
        except Exception as e:  # noqa: BLE001
            back["active"] = repr(e)
        if self.as_found:
            keys = ("StartLimitIntervalUSec", "StartLimitBurst", "RestartUSec")
            back["settings_as_found"] = all(show.get(k) == self.as_found["show"].get(k) for k in keys)
            back["dropins_as_found"] = back["dropins"] == self.as_found["dropins"]
            back["active_as_found"] = back["active"] == self.as_found["active"]
        (self.out / "unit_put_back.json").write_text(json.dumps(back, indent=1))
        self.say(f"unit put back: {back}")

    # ------------------------------------------------------- the hog, timed

    def run_hog(self) -> None:
        """Waits for a mission running, starts hog.py, waits for it to hold,
        keeps the window, then ends it. Its process goes whatever happens."""
        try:
            while not self.stop_all.is_set():
                if (self.brain_status() or {}).get("running"):
                    break
                self.stop_all.wait(2.0)
            if self.stop_all.is_set():
                return
            self.hog_proc = subprocess.Popen(
                [sys.executable, str(HERE / "hog.py"), str(self.out / "hog.json"), str(MARGIN_MB),
                 str(GROW_GIVE_UP_S), str(WINDOW_S + 120), str(os.getpid())],
                stdout=open(self.out / "hog.log", "w"), stderr=subprocess.STDOUT)
            self.say(f"hog started (pid {self.hog_proc.pid})")
            while not self.stop_all.is_set() and self.hog_proc.poll() is None:
                rec = self.hog_record()
                if rec.get("holding_t"):
                    self.window["start"] = rec["holding_t"]
                    self.say(f"hog holding {rec.get('held_mb')} MB; window {WINDOW_S} s")
                    break
                self.stop_all.wait(1.0)
            if self.window["start"]:
                # Polled (review 2): a hog the kernel killed ends the window early.
                while not self.stop_all.is_set() and time.time() < self.window["start"] + WINDOW_S:
                    if self.hog_proc.poll() is not None:
                        self.notes.append(f"the hog ended {time.time() - self.window['start']:.0f} s "
                                          f"into the window (exit {self.hog_proc.returncode})")
                        break
                    self.stop_all.wait(1.0)
            else:
                self.notes.append(f"the hog never held: {self.hog_record()}")
        finally:
            self.end_hog()

    def hog_record(self) -> dict:
        try:
            return json.loads((self.out / "hog.json").read_text())
        except (OSError, ValueError):
            return {}

    def end_hog(self) -> None:
        p = self.hog_proc
        if p is not None and p.poll() is None:
            p.terminate()
            try:
                p.wait(20)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(5)
        if self.window["end"] is None:
            self.window["end"] = time.time()

    def teardown_mission(self, mdir, st) -> None:
        """The harness stopping a mission's stack is not an outage (review 3):
        the servers are not sampled from here until the next mission runs."""
        with self.live_lock:
            self.in_teardown = True
            self.mission_live = False
            self.teardowns += 1
        super().teardown_mission(mdir, st)

    @staticmethod
    def up(url: str) -> bool:
        """`http_ok` also for a reply cut off mid-way (review 3)."""
        try:
            return http_ok(url)
        except Exception:  # noqa: BLE001 -- a server that died mid-reply is down
            return False

    # --------------------------------------------------------- the sampler

    def sample(self) -> None:
        while not self.stop_all.is_set():
            try:
                row = {"t": round(time.time(), 3), "mem_mb": mem_available_mb(),
                       "oom_kill": oom_kills(), "ve_ready": self.ve_ready()}
                row.update({k: v for k, v in self.unit_show().items()
                            if k in ("MainPID", "NRestarts", "ActiveState", "SubState", "Result")})
                s = self.brain_status()
                row["running"] = bool(s and s.get("running"))
                with self.live_lock:
                    if row["running"] and not self.in_teardown:
                        self.mission_live = True
                    live, gen = self.mission_live, self.teardowns
                st = getattr(self, "stack", None)
                row["mission_live"] = live
                if live and st is not None:
                    # A brain that stops answering reads as down here, never as
                    # "not running" (review 2): live until the mission returns.
                    robot_ok = self.up(st.urls["robot"] + "/health")
                    brain_ok = self.up(st.urls["brain"] + "/health")
                    with self.live_lock:
                        across = gen != self.teardowns   # review 5: the harness stopped it meanwhile
                    if across:
                        row["mission_live"] = False
                    else:
                        row["robot_ok"], row["brain_ok"] = robot_ok, brain_ok
                if row["running"]:
                    lv = ((s.get("tier") or {}).get("stats") or {}).get("local_vlm") or {}
                    row["local_vlm"] = lv.get("state")
                self.samples.write(json.dumps(row) + "\n")
            except Exception as e:  # noqa: BLE001 -- a lost sample, never a lost sampler
                self.samples.write(json.dumps({"t": round(time.time(), 3), "error": repr(e)}) + "\n")
            self.stop_all.wait(1.0)

    # --------------------------------------------------------- the finisher

    def finish(self) -> None:
        """After the window: VE back (at most D + 60 s), the loaded window,
        then the mission stopped -- asked every 5 s until no mission runs."""
        try:
            ceiling = self.t_begin + RUN_CEILING_S
            while (not self.stop_all.is_set() and self.window["end"] is None
                   and time.time() < ceiling):
                self.stop_all.wait(1.0)
            if self.stop_all.is_set() or self.window["end"] is None:
                if time.time() >= ceiling and "the run's ceiling came first" not in self.notes:
                    self.notes.append("the run's ceiling came first")
                return
            end = self.window["end"] + (self.delay_s or 0) + 60
            while time.time() < end and not self.ve_ready() and not self.stop_all.is_set():
                self.stop_all.wait(1.0)
            # The loaded window: inside a running mission with VE ready (at
            # most 300 s on), then LOADED_S of it (amendment 3).
            end = time.time() + 300
            while time.time() < end and not self.stop_all.is_set() and not (
                    self.ve_ready() and (self.brain_status() or {}).get("running")):
                self.stop_all.wait(1.0)
            self.loaded_from = time.time()
            self.stop_all.wait(LOADED_S + 5)
        finally:
            self.finished.set()
            # Stop whatever mission runs, until the mission loop has ended.
            while not self.main_done.is_set():
                st = getattr(self, "stack", None)
                if st is not None and (self.brain_status() or {}).get("running"):
                    try:
                        st.brain("POST", "/mission/stop", {}, timeout=10)
                    except Exception:  # noqa: BLE001 -- asked again
                        pass
                self.main_done.wait(5.0)

    # ------------------------------------------------------------- the run

    def run(self) -> int:
        threads = []
        try:
            self.record_as_found()
            self.refuse_busy()
            (self.out / "setup_started").write_text(f"{time.time()}\n")   # board.sh's net may clean up
            # The journal readable now: else no kill could be attributed.
            self.baseline_ok = self.oom_events() is not None
            self.setup()
            self.install_dropin()
            for fn in (self.sample, self.finish, self.run_hog):
                threads.append(threading.Thread(target=fn, daemon=True))
                threads[-1].start()
            sched, k, errors = soak.schedule(64), 0, 0
            with open(self.out / "missions.jsonl", "a", buffering=1) as f:
                while not self.finished.is_set():
                    if time.time() - self.t_begin > RUN_CEILING_S:
                        if "the run's ceiling came first" not in self.notes:
                            self.notes.append("the run's ceiling came first")
                        break
                    dead = {n: p.returncode for n, p in self.helpers.items() if p.poll() is not None}
                    if dead:
                        self.notes.append(f"a long-lived helper ended: {dead}")
                        break
                    house, target = sched[k % len(sched)]
                    self.say(f"m{k:03d} {house} / {target}; MemAvailable {mem_available_mb()} MB")
                    with self.live_lock:
                        self.in_teardown = False
                    rec = self.mission(k, house, target)
                    with self.live_lock:
                        self.mission_live = False
                    f.write(json.dumps(rec, default=str) + "\n")
                    self.say(f"m{k:03d} -> {rec.get('outcome')} {rec.get('error') or ''}")
                    k += 1
                    errors = errors + 1 if rec.get("error") else 0
                    if errors >= soak.MAX_ERRORS_IN_A_ROW:
                        self.notes.append(f"{errors} missions in a row errored")
                        break
                    if rec.get("error") and not self.finished.is_set():
                        time.sleep(30)
            self.main_done.set()
            self.stop_all.set()
            for t in threads:
                t.join(timeout=30)
            res = self.score()
            # 0 met, 1 valid but a bar missed, 2 invalid (amendment 2: re-run once).
            if not all(res["valid"].values()):
                return 2
            return 0 if all(v for v in res["met"].values() if v is not None) else 1
        finally:
            self.main_done.set()
            self.stop_all.set()
            self.end_hog()               # memory back before anything else is torn down
            for t in threads:
                t.join(timeout=5)
            try:
                self.put_back()
            finally:
                self.teardown()

    # ------------------------------------------------------------ the score

    def score(self) -> dict:
        rows = [r for r in (json.loads(x) for x in open(self.out / "samples.jsonl")) if "error" not in r]
        ws, we = self.window["start"], self.window["end"]
        win = [r for r in rows if ws and we and ws <= r["t"] <= we]
        hog = self.hog_record()
        hog_pid = str(self.hog_proc.pid) if self.hog_proc else None
        events = self.oom_events() or []
        kills = []
        for t, x in events:
            pid = (re.search(r"[,\s]pid=(\d+)", x) or [None, None])[1]
            kills.append({"t": t, "ve": VE_MEMCG in x, "hog": pid is not None and pid == hog_pid,
                          "task": (re.search(r"task=([^,]+)", x) or [None, None])[1],
                          "pid": pid, "line": x[-300:]})
        hog_t0 = hog.get("t0")
        before_hog = [k for k in kills if hog_t0 and k["t"] is not None and k["t"] < hog_t0]
        ve_kills = sum(1 for k in kills if k["ve"] and ws and we and k["t"] is not None
                       and ws - 5 <= k["t"] <= we + 5)
        count_rises = [(a, b) for a, b in zip(rows, rows[1:]) if a["oom_kill"] >= 0
                       and b["oom_kill"] > a["oom_kill"]]
        rose_by = sum(b["oom_kill"] - a["oom_kill"] for a, b in count_rises)
        window_kills = sum(b["oom_kill"] - a["oom_kill"] for a, b in count_rises
                           if ws and we and ws <= b["t"] <= we)
        nr = [int(r["NRestarts"]) for r in win if str(r.get("NRestarts", "")).isdigit()]
        restarts = (max(nr) - min(nr)) if nr else 0
        after = [r for r in rows if we and r["t"] > we]
        back = next((r["t"] for r in after if r["ve_ready"]), None)
        lf = self.loaded_from
        loaded = [r for r in rows if lf and lf <= r["t"] <= lf + LOADED_S and r.get("running")]
        lv_after = [r.get("local_vlm") for r in loaded]
        missions = [json.loads(x) for x in open(self.out / "missions.jsonl")]
        overlapping = [m for m in missions if ws and we and (m.get("t0") or m.get("setup_t"))
                       and (m.get("t0") or m.get("setup_t")) <= we
                       and (m.get("t1") or time.time()) >= ws]
        late = sum(((m.get("wheel_loop") or {}).get("late_ticks") or 0) for m in overlapping)
        # Servers through the crash-loop window and the loaded window (amendment 4).
        def in_windows(t):
            return bool((ws and we and ws <= t <= we) or (lf and lf <= t <= lf + LOADED_S))
        live = [r for r in rows if in_windows(r["t"]) and r.get("mission_live")]
        res = {
            "delay_s": self.delay_s, "window": [ws, we], "window_s": round(we - ws, 1) if ws and we else None,
            "hog": hog, "notes": self.notes, "kills": kills, "ve_kills": ve_kills,
            "kills_before_hog": len(before_hog),
            "oom_kill_rose_by": rose_by, "kills_in_window": window_kills,
            "loaded_rows": len(loaded),
            "restarts_in_window": restarts,
            "late_wheel_ticks_overlapping": late,
            "missions_overlapping": len(overlapping),
            "errored_missions_overlapping": sum(1 for m in overlapping if m.get("error")),
            "missions_without_wheel_loop": sum(1 for m in overlapping if not m.get("wheel_loop")),
            "live_samples_in_window": len(live),
            "robot_down_samples": sum(1 for r in live if r.get("robot_ok") is False),
            "brain_down_samples": sum(1 for r in live if r.get("brain_ok") is False),
            "mem_min_window_mb": min((r["mem_mb"] for r in win if r["mem_mb"] is not None), default=None),
            "loaded_from": lf,
            "mem_min_loaded_mb": min((r["mem_mb"] for r in loaded if r["mem_mb"] is not None),
                                     default=None),
            "ve_back_s": round(back - we, 1) if back and we else None,
            # Amendment 7: VE's availability, ranked after the stack's bars.
            "ve_ready_share_window": (round(sum(1 for r in win if r.get("ve_ready")) / len(win), 3)
                                      if win else None),
            "late_per_hour": (round(late / ((we - ws) / 3600), 1) if ws and we and we > ws else None),
            "local_vlm_loaded": lv_after[-10:],
        }
        res["valid"] = {   # amendment 3's complete list
            "a_crash_loop": ((window_kills >= MIN_KILLS and ve_kills >= MIN_KILLS
                              and restarts >= MIN_KILLS)
                             # Amendment 7: the shipped unit's loop is the one its
                             # start limit allows -- valid once the limit stopped it.
                             or (self.shipped and ve_kills >= 1 and any(
                                 r.get("Result") == "start-limit-hit" for r in win))),
            "kill_lines_complete": len(kills) >= rose_by,
            "loaded_window_in_a_mission": len(loaded) >= LOADED_MIN_ROWS,
            "no_kill_before_the_hog": not before_hog,
            "every_kill_line_placed": all(k["t"] is not None for k in kills),
            "a_full_window": res["window_s"] is not None and res["window_s"] >= WINDOW_S - 5,
            "no_errored_mission": res["errored_missions_overlapping"] == 0,
            "wheel_loop_read": res["missions_without_wheel_loop"] == 0 and bool(overlapping),
            "dmesg_baseline": self.baseline_ok,
        }
        res["met"] = {
            "every_kill_ve": bool(kills) and all(k["ve"] or k["hog"] for k in kills),
            "zero_late_ticks": late == 0,
            "servers_answered": res["robot_down_samples"] == 0 and res["brain_down_samples"] == 0,
            "mem_loaded_500": res["mem_min_loaded_mb"] is not None and res["mem_min_loaded_mb"] >= 500,
            "ve_back_within": (None if self.shipped else    # amendment 7: not a bar for it
                               res["ve_back_s"] is not None and res["ve_back_s"] <= self.delay_s + 60),
            "brain_up_after": bool(lv_after) and all(x == "up" for x in lv_after[-5:]),
        }
        # A kill of a stack process is a measured failure, never invalid (amendment 3).
        if (not res["met"]["every_kill_ve"] and kills and self.baseline_ok and not before_hog
                and all(k["t"] is not None for k in kills)):
            # Every line is this run's (the baseline was read), so the kill is
            # real: a failure as measured, whatever else made the run invalid.
            res["valid"] = {k: True for k in res["valid"]}
            res["valid_note"] = "a process other than VE or the hog was killed: failed, as measured"
        (self.out / "result.json").write_text(json.dumps(res, indent=1) + "\n")
        self.say(f"valid: {res['valid']}; met: {res['met']}")
        return res


def main(argv) -> int:
    loop = CrashLoop(Path(argv[0]), argv[1] if argv[1] == "shipped" else int(argv[1]))

    def on_signal(signum, _frame):
        if loop.signalled:               # review 3: a second signal never cuts the put-back short
            return
        loop.signalled = True
        loop.stop_all.set()              # the hog lets go first
        loop.end_hog()
        raise SystemExit(128 + signum)
    for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(s, on_signal)
    try:
        return loop.run()
    except SystemExit:
        raise
    except BaseException as e:  # noqa: BLE001 -- a harness error, not a measured failure
        print(f"harness error: {type(e).__name__}: {e}", flush=True)
        (Path(argv[0]) / "harness_error.txt").write_text(f"{type(e).__name__}: {e}\n")
        return 3


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
