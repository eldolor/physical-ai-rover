# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/soak.py

3.135.C, criterion 1 (the memory soak, F1), the BOARD side: back-to-back
tiered missions with `tier_search: explore` for `--seconds` (14,400: 4 h),
each in the next house of `HOUSES` with that house's next target (never the
previous mission's), on the stack `stack.py` starts (this section's ports,
the split body with `truth_body.py`'s ground truth, the fake board and the
fake D500), with the ROS container (SLAM + nav2), the board's own local VLM
(`picar-local-vlm`, VE, as found: never started, stopped or held here), the
YOLOE + CLIP loop (`tools/jetson/perception_loop.py` at 4 Hz, switched to
each mission's target with the release, 3.81.E's new-target path) and a $0
cloud stand-in (`evaluations/tiered-380/E/standin_cloud.py`). How each
choice reads against the bars is amendment 3 in the section file.

    python evaluations/jetson-fmea-3135/soak.py <out> [--seconds 14400]

Run through `soak.sh` (the board lock's check, the desktop off, the twin's
units stopped), which `run.py soak` runs through 3.136's `board_run`.

What lands in <out>:
  account.jsonl   tools/jetson/memory_account.py every 5 s (root): MemAvailable,
                  every process's RSS and the GPU's nvmap share; `phase` names the
                  mission (`m007`) or `setup` / `between` / `end`
  missions.jsonl  one line per mission: house, target, stamps (epoch and
                  monotonic), outcome, the robot server's wheel loop at the end,
                  contacts and the longest hole in the truth log, and `encoded`:
                  the models that encoded its target's text (3.142)
  text-cache/     the perception loop's text cache, emptied before each mission
  m<k>/           that mission's stack logs, truth.jsonl.gz, ros.log.gz
  perception.log, cloud_calls.jsonl, soak.log, ve_before.json, ve_after.json
"""

from __future__ import annotations

import argparse
import gzip
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from http.client import HTTPException
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import measure as m  # noqa: E402
from stack import PY, Stack, StackError, http  # noqa: E402

# Amendment 3: the five houses, and each one's eight targets -- the first
# eight of PREFERENCE present in the house, so the list is a rule, not a
# pick (the in-process explore sweep's rate on these 40 pairs is reported
# beside the live one: 39/40, procthor:val:3's box exhausted).
HOUSES = ("home_first_floor", "complex_house", "procthor:val:0", "procthor:val:2",
          "procthor:val:3")
PREFERENCE = ("red backpack", "basketball", "plunger", "garbage bag", "cart", "laundry hamper",
              "dog bed", "box", "house plant", "garbage can", "shoes", "slippers", "toy car",
              "water bottle", "pet bowl", "laptop bag", "laundry basket", "umbrella", "mug",
              "keys", "book", "guitar", "headphones", "towel", "tv remote", "trash can", "sofa",
              "chair", "dining table", "toilet", "desk", "coffee table", "armchair", "arm chair",
              "bookcase", "refrigerator", "fridge", "tv console", "tv stand", "bed", "dresser")
TARGETS = {
    "home_first_floor": ("red backpack", "sofa", "chair", "dining table", "toilet", "desk",
                         "coffee table", "armchair"),
    "complex_house": ("red backpack", "shoes", "slippers", "toy car", "water bottle", "pet bowl",
                      "laptop bag", "laundry basket"),
    "procthor:val:0": ("red backpack", "plunger", "dog bed", "box", "house plant", "garbage can",
                       "sofa", "chair"),
    "procthor:val:2": ("red backpack", "basketball", "plunger", "laundry hamper", "dog bed",
                       "house plant", "garbage can", "sofa"),
    "procthor:val:3": ("red backpack", "plunger", "garbage bag", "box", "garbage can", "sofa",
                       "chair", "dining table"),
}

IMAGE = os.environ.get("SOAK_IMAGE", "vision-picar-ros:latest")
CONTAINER = os.environ.get("SOAK_CONTAINER", "picar-ros-3135")
BRIDGE = "http://127.0.0.1:8090"
VE_URL = "http://127.0.0.1:8005"
STUB_PORT = int(os.environ.get("SOAK_STUB_PORT", 8099))
STACK_OOM_ADJ = -900               # picar-stack's OOMScoreAdjust (3.80.B4)
MISSION_CAP_S = 1200 + 600         # mission_time_limit_s + slack: past it the harness stops it
POLL_S = 2.0
MAX_ERRORS_IN_A_ROW = 5


def schedule(n: int, houses=HOUSES, targets=TARGETS) -> list:
    """The first `n` missions as (house, target): houses in turn, each
    house's targets in turn, a target equal to the previous mission's
    skipped (so every mission is a new target)."""
    ptr = {h: 0 for h in houses}
    out, prev = [], None
    for k in range(n):
        h = houses[k % len(houses)]
        ts = targets[h]
        t = ts[ptr[h] % len(ts)]
        ptr[h] += 1
        if t == prev:
            t = ts[ptr[h] % len(ts)]
            ptr[h] += 1
        out.append((h, t))
        prev = t
    return out


def stack_env(vlm_secret: str) -> dict:
    """What the board's `picar-stack` runs with, beyond `stack.py`'s own
    environment: ROS drive and world, the local tier, explore searching."""
    env = {"ROBOT_DRIVE": "ros", "WORLD_MODE": "ros", "LOCAL_VLM_URL": VE_URL,
           "TIER_SEARCH": "explore"}
    if vlm_secret:
        env["LOCAL_VLM_SECRET"] = vlm_secret
    return env


def read_vlm_secret(path: Path = Path.home() / ".vision-picar-local-vlm.env") -> str:
    """The VLM unit's own secret (its env file's APP_SHARED_SECRET), so the
    brain's local tier is answered, never 401'd into the cloud. Never logged."""
    try:
        for line in path.read_text().splitlines():
            k, _, v = line.partition("=")
            if k.strip() == "APP_SHARED_SECRET":
                return v.strip().strip('"').strip("'")
    except OSError:
        pass
    return ""


class Soak:
    def __init__(self, out: Path, seconds: float, *, houses=HOUSES, perception: bool = True):
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.seconds = seconds
        self.houses = houses
        self.perception = perception
        self.helpers: dict = {}
        self.stack: Optional[Stack] = None
        self.log = open(self.out / "soak.log", "a", buffering=1)
        self.phase_file = self.out / "phase"
        self.switch_file = self.out / "targets.txt"
        # 3.142: the perception loop's text cache is the run's own, emptied
        # before each switch, so every mission encodes its target (run 1 read
        # the board's ~/.cache, where earlier benches had left targets).
        self.text_cache = self.out / "text-cache"
        self.vlm_secret = read_vlm_secret()

    # ---------------------------------------------------------------- utils

    def say(self, msg: str) -> None:
        self.log.write(f"{time.strftime('%T')} {msg}\n")

    def phase(self, name: str) -> None:
        tmp = self.phase_file.with_suffix(".tmp")
        tmp.write_text(name + "\n")
        os.replace(tmp, self.phase_file)

    def sh(self, cmd: list, check: bool = False, timeout: float = 120,
           env: Optional[dict] = None) -> subprocess.CompletedProcess:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=check,
                              env=env)

    def oom_adj(self, pid: int, value: int) -> None:
        r = subprocess.run(["sudo", "-n", "tee", f"/proc/{pid}/oom_score_adj"], input=f"{value}\n",
                           capture_output=True, text=True, timeout=10)
        if r.returncode != 0:
            self.say(f"oom_score_adj {value} for {pid} failed: {r.stderr.strip()[:200]}")

    def spawn(self, name: str, argv: list, env: Optional[dict] = None,
              log: Optional[str] = None) -> subprocess.Popen:
        f = open(self.out / (log or f"{name}.log"), "ab")
        p = subprocess.Popen(argv, cwd=ROOT, env=env or os.environ.copy(), stdout=f,
                             stderr=subprocess.STDOUT, start_new_session=True)
        self.helpers[name] = p
        return p

    def ve_state(self) -> dict:
        r = self.sh(["systemctl", "show", "-p", "MainPID,NRestarts,ActiveState,ActiveEnterTimestamp",
                     "picar-local-vlm"])
        st = dict(line.split("=", 1) for line in r.stdout.splitlines() if "=" in line)
        code, body = http("GET", VE_URL + "/health", timeout=5)
        st["health"] = body if code == 200 else {"status": code}
        return st

    # ---------------------------------------------------------------- setup

    def setup(self) -> None:
        self.phase("setup")
        ve = self.ve_state()
        (self.out / "ve_before.json").write_text(json.dumps(ve, indent=1))
        if not (isinstance(ve.get("health"), dict) and ve["health"].get("ready")):
            raise StackError(f"the board's local VLM is not ready: {ve}")
        if not self.vlm_secret:
            # Without it the brain would send the stack's own secret, VE would
            # 401 every question into the cloud stand-in, and VE carry no load.
            raise StackError("no APP_SHARED_SECRET in the VLM's env file: the local tier "
                             "would be refused")
        if self.sh(["docker", "ps", "-aq", "-f", f"name=^{CONTAINER}$"]).stdout.strip():
            raise StackError(f"a container named {CONTAINER} exists already")
        for port in (8090, STUB_PORT):
            if http_ok(f"http://127.0.0.1:{port}/health"):
                raise StackError(f":{port} already answers -- someone else's service")
        # The memory account (root: nvmap's debugfs and the container's processes).
        self.spawn("account", ["sudo", "-n", "python3", "tools/jetson/memory_account.py", "sample",
                               str(self.out / "account.jsonl"), "--interval", "5",
                               "--seconds", str(int(self.seconds + 4 * 3600)),
                               "--phase-file", str(self.phase_file)])
        self.spawn("stub", [PY, "evaluations/tiered-380/E/standin_cloud.py", str(STUB_PORT),
                            str(self.out / "cloud_calls.jsonl")])
        wait(lambda: http_ok(f"http://127.0.0.1:{STUB_PORT}/health"), 30, "the cloud stand-in")
        if self.perception:
            self.switch_file.write_text("")
            p = self.spawn("perception", [PY, "tools/jetson/perception_loop.py",
                                          str(int(self.seconds + 4 * 3600))],
                           env=self.perception_env())
            log = self.out / "perception.log"
            wait(lambda: "loaded 1 pipeline" in log.read_text(errors="replace"), 600,
                 "perception loaded", proc=p)
            self.oom_adj(p.pid, STACK_OOM_ADJ)   # it is the brain's perception on the car
        self.say(f"setup done; MemAvailable {mem_available_mb()} MB")

    def perception_env(self) -> dict:
        return dict(os.environ, PERCEPTION_SWITCH_FILE=str(self.switch_file),
                    PERCEPTION_SWITCH_RELEASE="1", PICAR_TEXT_CACHE=str(self.text_cache),
                    PYTHONPATH=str(ROOT), PYTHONUNBUFFERED="1")

    def next_target(self, target: str) -> None:
        """Switch the perception loop to `target` with nothing cached, so its
        first predict encodes the text in `brain.text_encoder`'s children."""
        try:              # a purge that fails must fail the mission, not credit
            shutil.rmtree(self.text_cache)   # an old file as this mission's encode
        except FileNotFoundError:
            pass
        self.text_cache.mkdir(parents=True)
        with open(self.switch_file, "a") as f:
            f.write(target + "\n")

    def encoded(self, target: str) -> list:
        """The models that encoded `target`'s own text since the last switch
        (YOLOE's prompt is `[target]`, CLIP's scoring list starts with it).
        Under `crop_path: soft_gate` CLIP's anchor lookup is `[target]` too
        and counts; CLIP is reported only, so nothing is judged on it."""
        models = set()
        for f in sorted(self.text_cache.glob("*.json")):
            try:
                rec = json.loads(f.read_text())
                if rec["texts"][0] == target:
                    models.add(rec["model"])
            except (OSError, ValueError, KeyError, TypeError, IndexError):
                continue
        return sorted(models)

    # -------------------------------------------------------------- mission

    def mission(self, k: int, house: str, target: str) -> dict:
        mdir = self.out / f"m{k:03d}"
        rec: dict = {"k": k, "house": house, "target": target, "setup_t": round(time.time(), 3)}
        self.phase(f"m{k:03d}")
        try:                      # VE's process: a change between missions is a restart
            r = self.sh(["systemctl", "show", "-p", "MainPID,NRestarts", "picar-local-vlm"])
            rec["ve"] = dict(x.split("=", 1) for x in r.stdout.splitlines() if "=" in x)
        except (OSError, subprocess.SubprocessError):
            pass
        st = None
        try:
            st = self.stack = Stack(mdir, house=house,
                                    vision_url=f"http://127.0.0.1:{STUB_PORT}",
                                    extra_env=stack_env(self.vlm_secret))
            st.start()
            for p in st.procs.values():
                self.oom_adj(p.pid, STACK_OOM_ADJ)
            self.sh(["docker", "rm", "-f", CONTAINER])
            r = self.sh(["docker", "run", "-d", "--name", CONTAINER, "--network", "host",
                         "-e", "APP_SHARED_SECRET", "-e", "ROBOT_URL", "-e", "BRAIN_URL",
                         IMAGE, "ros2", "launch", "picar_bringup", "picar.launch.py"],
                        timeout=120, env=dict(os.environ, APP_SHARED_SECRET=st.secret,
                                              ROBOT_URL=st.urls["robot"],
                                              BRAIN_URL=st.urls["brain"]))
            if r.returncode != 0:
                raise StackError(f"docker run: {r.stderr.strip()[:300]}")
            wait(lambda: http_ok(BRIDGE + "/health"), 180, "the ROS bridge")
            wait(lambda: '"start_truth": {' in json.dumps(
                http("GET", BRIDGE + "/slam/pose", secret=st.secret, timeout=3)[1]),
                120, "SLAM's start pose")
            rec["ready_t"] = round(time.time(), 3)
            if self.perception:
                self.next_target(target)
            rec["t0"], rec["m0"] = round(time.time(), 3), round(time.monotonic(), 4)
            code, body = st.brain("POST", "/mission/start",
                                  {"target_object": target, "policy": "tiered"}, timeout=600)
            if code != 200:
                raise StackError(f"mission start: {code} {str(body)[:300]}")
            s, stopped, last = {}, False, {}
            while True:
                time.sleep(POLL_S)
                # The cap first: a brain alive but not answering must still end here.
                age = time.time() - rec["t0"]
                if age > MISSION_CAP_S + 120:
                    rec["status_last"] = last
                    raise StackError("the mission did not end 120 s after the harness's cap")
                if age > MISSION_CAP_S and not stopped:
                    try:
                        st.brain("POST", "/mission/stop", {}, timeout=10)
                        stopped = True
                    except (OSError, HTTPException) as e:     # tried again on the next poll
                        self.say(f"m{k:03d} the cap's Stop: {e}")
                    rec["stopped_by_harness"] = True
                try:
                    code, s = st.brain("GET", "/mission/status", timeout=10)
                except (OSError, HTTPException):
                    code, s = None, {}
                if code != 200 or not isinstance(s, dict):
                    s = {}
                    if not st.alive("brain"):
                        raise StackError("the brain died mid-mission")
                    continue
                last = s
                if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
                    break
            rec["t1"], rec["m1"] = round(time.time(), 3), round(time.monotonic(), 4)
            rec["outcome"] = s.get("outcome")
            rec.setdefault("stopped_by_harness", False)
            rec["status"] = {k_: s.get(k_) for k_ in ("outcome", "end_state", "end_reason", "step",
                                                      "error", "cancelled_by", "report",
                                                      "arrival_verdicts", "cloud")}
            code, h = st.robot("GET", "/health", timeout=10)
            if code == 200 and isinstance(h, dict):
                rec["wheel_loop"] = h.get("wheel_loop")
        except Exception as e:  # noqa: BLE001 -- one mission's failure is a recorded row
            rec["error"] = f"{type(e).__name__}: {e}"
            rec.setdefault("t1", round(time.time(), 3))
            rec.setdefault("m1", round(time.monotonic(), 4))
            if st is not None and "wheel_loop" not in rec:
                try:
                    code, h = st.robot("GET", "/health", timeout=5)
                    if code == 200 and isinstance(h, dict):
                        rec["wheel_loop"] = h.get("wheel_loop")
                except OSError:
                    pass
        finally:
            # Read at the end: the encode lands ~7-42 s into a mission (3.142).
            if self.perception and "t0" in rec:
                rec["encoded"] = self.encoded(target)
            self.phase("between")
            self.teardown_mission(mdir, st)
            self.stack = None
        truth = mdir / "truth.jsonl"
        rows = m.read_truth(truth) if truth.exists() else []
        rec["contacts"] = m.contacts(rows)
        rec["gap_unknown"] = sum(1 for r in rows if r.get("gap_cm") is None)
        sp = m.span(rows)
        rec["truth_hole_s"] = (m.gaps_s(rows, rec["m0"], rec["m1"]) if "m0" in rec else None)
        rec["truth_hole_all_s"] = m.gaps_s(rows, sp[0], sp[1]) if sp else None
        rec["truth_rows"] = len(rows)
        gz(truth)
        return rec

    def teardown_mission(self, mdir: Path, st: Optional[Stack]) -> None:
        mdir.mkdir(parents=True, exist_ok=True)
        try:
            with open(mdir / "ros.log", "w") as f:
                subprocess.run(["docker", "logs", CONTAINER], stdout=f, stderr=subprocess.STDOUT,
                               timeout=60)
        except (OSError, subprocess.SubprocessError) as e:
            self.say(f"docker logs: {e}")
        try:
            self.docker_rm()
        finally:                  # the stack goes whatever docker does
            if st is not None:
                st.stop()
            gz(mdir / "ros.log")

    def docker_rm(self) -> None:
        try:
            self.sh(["docker", "rm", "-f", CONTAINER])
        except (OSError, subprocess.SubprocessError) as e:
            self.say(f"docker rm: {e}")

    # ------------------------------------------------------------------ run

    def run(self) -> int:
        try:
            self.setup()
            t_start = time.time()
            (self.out / "run_start").write_text(f"{t_start}\n")
            plan = schedule(10_000, self.houses)
            k, errors = 0, 0
            with open(self.out / "missions.jsonl", "a", buffering=1) as f:
                while time.time() - t_start < self.seconds:
                    # The 4 h's long-lived load and instruments, still there: a
                    # soak without them would score a lighter workload.
                    dead = {n: p.returncode for n, p in self.helpers.items() if p.poll() is not None}
                    if dead:
                        raise StackError(f"a long-lived helper ended: {dead}")
                    house, target = plan[k]
                    self.say(f"m{k:03d} {house} / {target}; MemAvailable {mem_available_mb()} MB")
                    rec = self.mission(k, house, target)
                    f.write(json.dumps(rec, default=str) + "\n")
                    self.say(f"m{k:03d} -> {rec.get('outcome')} {rec.get('error') or ''}")
                    k += 1
                    # A stack that keeps failing is a broken run, not thousands of rows.
                    errors = errors + 1 if rec.get("error") else 0
                    if errors >= MAX_ERRORS_IN_A_ROW:
                        raise StackError(f"{errors} missions in a row ended in an error")
                    if errors:
                        time.sleep(30)
            (self.out / "run_end").write_text(f"{time.time()}\n")
            return 0
        finally:
            self.teardown()

    def teardown(self) -> None:
        self.phase("end")
        if self.stack is not None:
            try:
                self.teardown_mission(self.out / "teardown", self.stack)
            except Exception as e:  # noqa: BLE001
                self.say(f"teardown stack: {e}")
            finally:
                try:
                    self.stack.stop()
                except Exception as e:  # noqa: BLE001
                    self.say(f"teardown stack stop: {e}")
        self.docker_rm()
        try:
            (self.out / "ve_after.json").write_text(json.dumps(self.ve_state(), indent=1))
        except Exception as e:  # noqa: BLE001
            self.say(f"ve_after: {e}")
        time.sleep(6)            # the sampler reads `end` and stops by itself
        for name in ("perception", "stub", "account"):
            p = self.helpers.get(name)
            if p is None or p.poll() is not None:
                continue
            if name == "account":           # root's: only sudo can signal it, by its own file
                pat = f"memory_account.py sample {self.out}/account.jsonl"
                for sig in ("-TERM", "-KILL"):
                    subprocess.run(["sudo", "-n", "pkill", sig, "-f", pat], timeout=10)
                    try:
                        p.wait(timeout=15)
                        break
                    except subprocess.TimeoutExpired:
                        continue
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


def gz(path: Path) -> None:
    if not path.exists():
        return
    with open(path, "rb") as src, gzip.open(str(path) + ".gz", "wb", compresslevel=6) as dst:
        shutil.copyfileobj(src, dst)
    path.unlink()


def http_ok(url: str) -> bool:
    try:
        return http("GET", url, timeout=2)[0] == 200
    except OSError:
        return False


def wait(cond, timeout_s: float, what: str, proc: Optional[subprocess.Popen] = None) -> None:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if proc is not None and proc.poll() is not None:
            raise StackError(f"{what}: the process exited ({proc.returncode})")
        try:
            if cond():
                return
        except (OSError, ValueError):
            pass
        time.sleep(1.0)
    raise StackError(f"{what} not reached in {timeout_s:.0f} s")


def mem_available_mb() -> Optional[int]:
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) // 1024
    except OSError:
        pass
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--seconds", type=float, default=14400.0)
    ap.add_argument("--houses", default=",".join(HOUSES))
    ap.add_argument("--no-perception", action="store_true")
    a = ap.parse_args(argv)
    houses = tuple(h for h in a.houses.split(",") if h)
    unknown = [h for h in houses if h not in TARGETS]
    if unknown:
        raise SystemExit(f"no target list for {unknown}")

    def on_signal(signum, _frame):
        raise SystemExit(128 + signum)     # the finally's teardown runs
    for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(s, on_signal)
    return Soak(Path(a.out), a.seconds, houses=houses, perception=not a.no_perception).run()


if __name__ == "__main__":
    raise SystemExit(main())
