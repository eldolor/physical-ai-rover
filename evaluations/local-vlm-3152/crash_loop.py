# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/local-vlm-3152/crash_loop.py

3.152 criterion 10, ON the board (through `board.sh`, under the board lock
and 3.136's lease): one candidate restart delay D for `picar-local-vlm`,
measured in a 15-min OOM crash loop beside 3.135.C's live stack (`Soak`:
the split body, the fake board and D500, the ROS container, YOLOE + CLIP,
the $0 cloud stand-in) running tiered missions back to back with VE as the
local tier.

* The unit gets a temporary drop-in, `StartLimitIntervalSec=0` and
  `RestartSec=D`. The shipped unit and any drop-ins found are recorded
  first, and the drop-in is removed and the unit put back as found at the
  end, whatever happened.
* The hog grows until the kernel's first OOM kill, which names VE's driver
  at `oom_score_adj` 1000. It then holds that size plus `MARGIN_MB`, so each
  VE load is killed again: a crash loop at the unit's pace.
* At 1 Hz: `MemAvailable`, the kernel's `oom_kill` count, both servers'
  /health, VE's /health, the unit's `NRestarts` and VE's PIDs (the unit's
  main process and its children: the service and its Edge-LLM driver).
  Every dmesg kill line new since the start is VE's only if its PID was.
  Each kill's moment is the sample where `oom_kill` rose.
* After `WINDOW_S` the hog ends. VE must answer ready within D + 60 s with no
  `reset-failed`, and the running mission's brain must read `local_vlm`
  `up`.

    python evaluations/local-vlm-3152/crash_loop.py <out> <D seconds>
"""

from __future__ import annotations

import json
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
DROPIN_DIR = Path(f"/etc/systemd/system/{UNIT}.service.d")
DROPIN = DROPIN_DIR / "zz-3152-crash-loop.conf"
WINDOW_S = 15 * 60
CHUNK = 32 * 1024 * 1024
MARGIN_MB = 256
GROW_GIVE_UP_S = 300        # no kill by then: the hog stops (an invalid run)
FLOOR_MB = 150              # the hog never grows past this much MemAvailable without a kill
SHOW = "MainPID,NRestarts,ActiveState,SubState,Result,StartLimitIntervalUSec,StartLimitBurst,RestartUSec"


def oom_kills() -> int:
    with open("/proc/vmstat") as f:
        for line in f:
            if line.startswith("oom_kill "):
                return int(line.split()[1])
    return -1


class CrashLoop(soak.Soak):
    def __init__(self, out: Path, delay_s: int):
        super().__init__(out, WINDOW_S + 2 * soak.MISSION_CAP_S)
        self.delay_s = int(delay_s)
        self.window = {"start": None, "end": None}
        self.stop_all = threading.Event()
        self.samples = open(out / "samples.jsonl", "a", buffering=1)
        self.dropin_installed = False
        self.ve_pids: set = set()
        self.finished = threading.Event()
        self.dmesg0: set = set()

    # ------------------------------------------------------------ the unit

    def unit_show(self) -> dict:
        r = self.sh(["systemctl", "show", "-p", SHOW, UNIT])
        return dict(x.split("=", 1) for x in r.stdout.splitlines() if "=" in x)

    def ve_tree(self, main: int) -> set:
        """The unit's main PID and every descendant, from /proc."""
        pids, todo = set(), [main] if main > 0 else []
        while todo:
            p = todo.pop()
            pids.add(p)
            try:
                for task in Path(f"/proc/{p}/task").iterdir():
                    kids = (task / "children").read_text().split()
                    todo.extend(int(k) for k in kids if int(k) not in pids)
            except OSError:
                pass
        return pids

    def dmesg_kills(self) -> list:
        out = self.sh(["sudo", "-n", "dmesg"]).stdout
        return [line for line in out.splitlines() if "Killed process" in line]

    def record_as_found(self) -> None:
        found = {"show": self.unit_show(),
                 "cat": self.sh(["systemctl", "cat", UNIT]).stdout,
                 "dropins": sorted(p.name for p in DROPIN_DIR.glob("*")) if DROPIN_DIR.exists() else [],
                 "active": self.sh(["systemctl", "is-active", UNIT]).stdout.strip(),
                 "enabled": self.sh(["systemctl", "is-enabled", UNIT]).stdout.strip()}
        (self.out / "unit_as_found.json").write_text(json.dumps(found, indent=1))
        if DROPIN.name in found["dropins"]:
            raise StackError(f"{DROPIN} exists already: a run before this one was not put back")
        self.as_found = found

    def install_dropin(self) -> None:
        body = ("# 3.152 criterion 10: a temporary crash-loop candidate; removed at the run's end.\n"
                "[Unit]\nStartLimitIntervalSec=0\n[Service]\n"
                f"RestartSec={self.delay_s}\n")
        r = subprocess.run(["sudo", "-n", "install", "-D", "-m", "644", "/dev/stdin", str(DROPIN)],
                           input=body, text=True, capture_output=True)
        if r.returncode != 0:
            raise StackError(f"installing the drop-in: {r.stderr.strip()[:200]}")
        self.dropin_installed = True
        self.sh(["sudo", "-n", "systemctl", "daemon-reload"], check=True)
        show = self.unit_show()
        (self.out / "unit_candidate.json").write_text(json.dumps(show, indent=1))
        if show.get("StartLimitIntervalUSec") not in ("0", "infinity") \
                or show.get("RestartUSec") != f"{self.delay_s}s" and \
                show.get("RestartUSec") != f"{self.delay_s // 60}min":
            raise StackError(f"the candidate did not load: {show}")
        self.say(f"candidate loaded: {show}")

    def put_back(self) -> None:
        """The drop-in out, the unit's counter reset and VE started again only
        if it was active as found; then the unit read back against as found."""
        if self.dropin_installed or DROPIN.exists():
            self.sh(["sudo", "-n", "rm", "-f", str(DROPIN)])
            self.sh(["sudo", "-n", "systemctl", "daemon-reload"])
            self.dropin_installed = False
        self.sh(["sudo", "-n", "systemctl", "reset-failed", UNIT])
        if getattr(self, "as_found", {}).get("active") == "active":
            if self.sh(["systemctl", "is-active", UNIT]).stdout.strip() != "active":
                self.sh(["sudo", "-n", "systemctl", "start", UNIT])
            end = time.time() + 180
            while time.time() < end and not self.ve_ready():
                time.sleep(2)
        show = self.unit_show()
        back = {"show": show, "active": self.sh(["systemctl", "is-active", UNIT]).stdout.strip(),
                "ve_ready": self.ve_ready(),
                "dropins": sorted(p.name for p in DROPIN_DIR.glob("*")) if DROPIN_DIR.exists() else []}
        found = getattr(self, "as_found", None)
        if found:
            keys = ("StartLimitIntervalUSec", "StartLimitBurst", "RestartUSec")
            back["settings_as_found"] = all(show.get(k) == found["show"].get(k) for k in keys)
            back["dropins_as_found"] = back["dropins"] == found["dropins"]
            back["active_as_found"] = back["active"] == found["active"]
        (self.out / "unit_put_back.json").write_text(json.dumps(back, indent=1))
        self.say(f"unit put back: {back}")

    def ve_ready(self) -> bool:
        code, body = http("GET", soak.VE_URL + "/health", timeout=3)
        return code == 200 and isinstance(body, dict) and body.get("ready") is True

    # ------------------------------------------------------------- the hog

    def hog(self) -> None:
        held, kills0 = [], oom_kills()
        t0 = time.time()
        rec = {"t0": round(t0, 3), "kills_at_start": kills0}
        try:
            while not self.stop_all.is_set():          # grow to the first kill
                if oom_kills() > kills0:
                    rec["first_kill_t"] = round(time.time(), 3)
                    rec["first_kill_mb"] = len(held) * CHUNK // 2**20
                    break
                if time.time() - t0 > GROW_GIVE_UP_S:
                    rec["gave_up"] = "no OOM kill while growing"
                    return
                avail = mem_available_mb() or 0
                if avail < FLOOR_MB:
                    time.sleep(0.25)                   # at the floor: wait for the kernel
                    continue
                b = bytearray(CHUNK)
                b[::4096] = b"\x01" * len(b[::4096])   # resident
                held.append(b)
                time.sleep(0.05)
            for _ in range(MARGIN_MB * 2**20 // CHUNK):   # each reload killed again
                b = bytearray(CHUNK)
                b[::4096] = b"\x01" * len(b[::4096])
                held.append(b)
            rec["held_mb"] = len(held) * CHUNK // 2**20
            self.window["start"] = time.time()
            self.say(f"hog holding {rec['held_mb']} MB; the window runs {WINDOW_S} s")
            self.stop_all.wait(max(0.0, WINDOW_S - (time.time() - rec["first_kill_t"])))
        finally:
            held.clear()
            self.window["end"] = time.time()
            rec["released_t"] = round(self.window["end"], 3)
            (self.out / "hog.json").write_text(json.dumps(rec, indent=1))

    # --------------------------------------------------------- the sampler

    def sample(self) -> None:
        while not self.stop_all.is_set():
            st = getattr(self, "stack", None)
            row = {"t": round(time.time(), 3), "mem_mb": mem_available_mb(), "oom_kill": oom_kills(),
                   "ve_ready": self.ve_ready()}
            try:
                row.update({k: v for k, v in self.unit_show().items()
                            if k in ("MainPID", "NRestarts", "ActiveState", "SubState")})
                tree = self.ve_tree(int(row.get("MainPID") or 0))
                self.ve_pids |= tree
                row["ve_pids"] = sorted(tree)
            except (OSError, subprocess.SubprocessError, ValueError):
                pass
            if st is not None:
                row["robot_ok"] = http_ok(st.urls["robot"] + "/health")
                row["brain_ok"] = http_ok(st.urls["brain"] + "/health")
                try:
                    code, s = st.brain("GET", "/mission/status", timeout=3)
                    lv = ((s or {}).get("tier") or {}).get("stats", {}).get("local_vlm") \
                        if code == 200 else None
                    row["local_vlm"] = (lv or {}).get("state")
                    row["running"] = (s or {}).get("running")
                except Exception:  # noqa: BLE001 -- a sample that fails is recorded as missing
                    row["local_vlm"] = None
            self.samples.write(json.dumps(row) + "\n")
            self.stop_all.wait(1.0)

    def finish(self) -> None:
        """After the window: wait for VE back (at most D + 60 s), then 30 s of
        samples for the brain's state, then stop the running mission so the
        run ends."""
        while not self.stop_all.is_set() and not self.window["end"]:
            self.stop_all.wait(1.0)
        if self.stop_all.is_set():
            return
        end = self.window["end"] + self.delay_s + 60
        while time.time() < end and not self.ve_ready() and not self.stop_all.is_set():
            time.sleep(1.0)
        self.stop_all.wait(30.0)
        self.finished.set()
        st = getattr(self, "stack", None)
        if st is not None:
            try:
                st.brain("POST", "/mission/stop", {}, timeout=10)
            except Exception as e:  # noqa: BLE001 -- the mission's own cap still ends it
                self.say(f"stopping the mission after the window: {e}")

    # ------------------------------------------------------------- the run

    def run(self) -> int:
        threads = []
        try:
            self.record_as_found()
            self.dmesg0 = set(self.dmesg_kills())
            self.setup()
            self.install_dropin()
            for fn in (self.sample, self.finish):
                threads.append(threading.Thread(target=fn, daemon=True))
                threads[-1].start()
            hog = None
            sched = soak.schedule(64)
            k = 0
            with open(self.out / "missions.jsonl", "a", buffering=1) as f:
                while True:
                    house, target = sched[k % len(sched)]
                    if hog is None:
                        # The hog starts once this mission is running.
                        def start_hog():
                            end = time.time() + 900
                            while time.time() < end and not self.stop_all.is_set():
                                st = getattr(self, "stack", None)
                                if st is not None:
                                    code, s = st.brain("GET", "/mission/status", timeout=3)
                                    if code == 200 and (s or {}).get("running"):
                                        break
                                time.sleep(2)
                            self.hog()
                        hog = threading.Thread(target=start_hog, daemon=True)
                        hog.start()
                        threads.append(hog)
                    self.say(f"m{k:03d} {house} / {target}; MemAvailable {mem_available_mb()} MB")
                    rec = self.mission(k, house, target)
                    f.write(json.dumps(rec, default=str) + "\n")
                    self.say(f"m{k:03d} -> {rec.get('outcome')} {rec.get('error') or ''}")
                    k += 1
                    if self.finished.is_set():
                        break
                    if rec.get("error"):
                        time.sleep(30)
            self.score()
            return 0
        finally:
            self.stop_all.set()
            for t in threads:
                t.join(timeout=5)
            try:
                self.put_back()
            finally:
                self.teardown()

    def score(self) -> None:
        rows = [json.loads(x) for x in open(self.out / "samples.jsonl")]
        ws, we = self.window["start"], self.window["end"]
        hog = json.loads((self.out / "hog.json").read_text())
        kills = self.kill_lines()
        win = [r for r in rows if ws and we and ws <= r["t"] <= we]
        # Each kill's moment: the sample where the kernel's count rose.
        kill_ts = [b["t"] for a, b in zip(rows, rows[1:])
                   if a["oom_kill"] >= 0 and b["oom_kill"] > a["oom_kill"]]
        outside = [r["mem_mb"] for r in win if r["mem_mb"] is not None
                   and not any(abs(r["t"] - kt) <= 5 for kt in kill_ts)]
        after = [r for r in rows if we and r["t"] > we]
        back = next((r["t"] for r in after if r["ve_ready"]), None)
        lv_after = [r.get("local_vlm") for r in after if back and r["t"] >= back and r.get("running")]
        missions = [json.loads(x) for x in open(self.out / "missions.jsonl")]
        late = sum(((m.get("wheel_loop") or {}).get("late_ticks") or 0) for m in missions)
        res = {
            "delay_s": self.delay_s, "window": [ws, we], "hog": hog,
            "kills": kills, "kill_times": kill_ts,
            "kills_in_window": sum(1 for t in kill_ts if ws and we and ws <= t <= we),
            "every_kill_ve": all(k["ve"] for k in kills),
            "late_wheel_ticks_all_missions": late,
            "robot_down_samples": sum(1 for r in win if r.get("robot_ok") is False),
            "brain_down_samples": sum(1 for r in win if r.get("brain_ok") is False),
            "mem_min_outside_kills_mb": min(outside) if outside else None,
            "mem_min_window_mb": min((r["mem_mb"] for r in win if r["mem_mb"] is not None), default=None),
            "nrestarts": [win[0].get("NRestarts"), win[-1].get("NRestarts")] if win else None,
            "ve_back_s": round(back - we, 1) if back and we else None,
            "local_vlm_after_back": lv_after[-5:],
        }
        res["met"] = {
            "every_kill_ve": res["every_kill_ve"] and bool(kills),
            "zero_late_ticks": late == 0,
            "servers_answered": res["robot_down_samples"] == 0 and res["brain_down_samples"] == 0,
            "mem_floor_500": res["mem_min_outside_kills_mb"] is not None
                             and res["mem_min_outside_kills_mb"] >= 500,
            "ve_back_within": res["ve_back_s"] is not None and res["ve_back_s"] <= self.delay_s + 60,
            "brain_up_after": bool(lv_after) and lv_after[-1] == "up",
        }
        (self.out / "result.json").write_text(json.dumps(res, indent=1) + "\n")
        self.say(f"result: {res['met']}")

    def kill_lines(self) -> list:
        """dmesg's kill lines new since the run began; VE's if the PID was
        one of the unit's (main process or a descendant) at a sample."""
        kills = []
        for line in self.dmesg_kills():
            if line in self.dmesg0:
                continue
            m = re.search(r"Killed process (\d+) \(([^)]+)\)", line)
            if m:
                pid = int(m.group(1))
                kills.append({"pid": pid, "name": m.group(2), "ve": pid in self.ve_pids,
                              "line": line[:300]})
        return kills


def main(argv) -> int:
    def on_signal(signum, _frame):
        raise SystemExit(128 + signum)
    for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(s, on_signal)
    return CrashLoop(Path(argv[0]), int(argv[1])).run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
