# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/vlm_rows.py

3.135.B's VE rows (amendment 8), BOARD only: one case run once on the full
stack, as 3.135.C's soak mission runs it -- `stack.py`'s stack with
`drive: ros`, `world: ros`, `tier_search: explore`, the ROS container
(`ros.py`), the board's own VE (`picar-local-vlm`, as found) reached through
`vlm_proxy.py` with the unit's own secret, the $0 cloud stand-in
(`evaluations/tiered-380/E/standin_cloud.py`, or a closed port), and the
stack's processes at `picar-stack`'s `oom_score_adj` (-900, sudo). A tiered
mission for the red backpack in `scaled_house`.

| Case | Fault | Plant (criterion 4) |
|---|---|---|
| F2 | `tools/jetson/memory_hog.py` (to the first OOM kill) 0.15 s into nav2's FORWARD | `ve_oom0` (a unit drop-in; the user's yes first) |
| F3f | SIGKILL VE's driver while a frame question is open (the proxy) | `vlm_absent` |
| F3a | SIGKILL VE's driver while the arrival's question is open; the stand-in up | `vlm_absent` |
| F3a2 | as F3a, the stand-in answering from the simulator's truth (`truth_cloud.py`, amendment 17) | `vlm_absent` |
| F3r | as F3a, the cloud refused (a closed port) | `vlm_absent` |
| F17g | a person's `/stop` 0.15 s into nav2's FORWARD | `no_latch` |
| F17q | a person's `/stop` while a frame question is open | `no_latch` |
| F17a | a person's `/stop` while the arrival's question is open | `no_latch` |

    python evaluations/jetson-fmea-3135/vlm_rows.py F3f --out DIR [--plant vlm_absent]
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import measure as m  # noqa: E402
import ros as rosmod  # noqa: E402
from fmea import (GOAL_MOVE_WAIT_S, MemWatch, robot_health, status, wait_end,  # noqa: E402
                  wait_forward)
from soak import read_vlm_secret  # noqa: E402
from stack import CLOSED_PORT, PORTS, PY, Stack, StackError, http  # noqa: E402

VE_URL = "http://127.0.0.1:8005"
VE_UNIT = "picar-local-vlm"
PROXY_PORT = int(os.environ.get("FMEA_VLM_PROXY_PORT", 10357))
STUB_PORT = int(os.environ.get("FMEA_STUB_PORT", 10358))
STACK_OOM_ADJ = -900
QUESTION_WAIT_S = 900.0        # a frame question opens within seconds; an arrival may take minutes
VE_BACK_BAR_S = 60.0
WATCH_AFTER_S = 120.0          # F3f: the mission is watched this long after the kill, then stopped
MISSION_WAIT_S = 1200 + 300

STOP_ZERO_BAR_S = 0.2
CANCEL_WAIT_S = 30.0
NEXT_ANSWER_WAIT_S = 240.0

ROWS = {
    "F2": {"title": "Memory runs out (OOM) during a mission", "fault": "oom", "when": "goal",
           "cloud": "standin", "plant": "ve_oom0",
           "reading": "exactly one OOM kill in the window and it is VE's (dmesg names VE's process); "
                      "the robot server, the brain and the ROS container alive after it; 0 late "
                      "wheel ticks from the hog's start to 30 s after the kill; the mission ends "
                      "with an outcome (within 1,500 s); 0 contacts"},
    "F3f": {"title": "Local VLM crashes mid-question (a frame's)", "fault": "kill_ve", "when": "frame",
            "cloud": "standin", "plant": "vlm_absent",
            "reading": "the mission is not failed by it over the 120 s after the kill; VE's /health "
                       "ready again <= 60 s after the kill; the next mission's first frame question "
                       "answered 200 by VE; 0 contacts"},
    "F3a": {"title": "Local VLM crashes mid-question (the arrival's)", "fault": "kill_ve",
            "when": "arrival", "cloud": "standin", "plant": "vlm_absent",
            "reading": "the mission ends found; VE back <= 60 s; the next mission's first frame "
                       "question answered; 0 contacts"},
    "F3a2": {"title": "Local VLM crashes mid-question (the arrival's), a truthful cloud",
             "fault": "kill_ve", "when": "arrival", "cloud": "truth", "plant": "vlm_absent",
             "reading": "amendment 17: the mission ends found, confirmed by the cloud (an arrival "
                        "verdict of the cloud's, confirmed); VE back <= 60 s; the next mission's first "
                        "frame question answered; 0 contacts"},
    "F3r": {"title": "Local VLM crashes at the arrival, the cloud refused", "fault": "kill_ve",
            "when": "arrival", "cloud": "refused", "plant": "vlm_absent",
            "reading": "the mission ends arrived_unconfirmed; VE back <= 60 s; the next mission's "
                       "first frame question answered; 0 contacts"},
    "F17g": {"title": "A person's Stop under full load (a nav2 goal)", "fault": "stop", "when": "goal",
             "cloud": "standin", "plant": "no_latch",
             "reading": "wheels zero <= 0.2 s from the /stop sent; the mission ends cancelled by stop "
                        "within 30 s; 0 paid calls (stand-in /navigate) after the Stop; 0 contacts"},
    "F17q": {"title": "A person's Stop during a VLM question", "fault": "stop", "when": "frame",
             "cloud": "standin", "plant": "no_latch", "reading": "as F17g"},
    "F17a": {"title": "A person's Stop during the arrival's question", "fault": "stop",
             "when": "arrival", "cloud": "standin", "plant": "no_latch", "reading": "as F17g"},
    "F24": {"title": "The battery reaches Reserve, then Power-off, mid-mission", "fault": "battery",
            "when": "goal", "cloud": "standin", "plant": "no_reserve",
            "reading": "amendment 18, 3.137 criterion 1's bars: 0 autonomous motion after Reserve (true "
                       "path <= 2 cm from the Reserve latch to the power-off); the stub called exactly "
                       "once, after the brain's mission, ROS and VE are down and the wheels at 0, in "
                       "the logged order; 0 contacts"},
}

# F24 (amendment 18): the pack set at the fault, crossing Reserve (10.2 V) ~30 s later and
# Power-off (10.0 V) ~40 s after that; the shutdown steps in 3.137's order.
F24_V0, F24_DRAIN = 10.35, 0.005
F24_WAIT_S = 240.0
F24_MOVE_BAR_CM = 2.0
F24_STEPS = ["wheels", "brain", "ros", "vlm", "sync", "poweroff"]


VE_UNIT_PROPS = ("OOMScoreAdjust", "Restart", "RestartUSec", "StartLimitBurst", "StartLimitIntervalUSec",
                 "ExecStart", "ConditionPathExists", "EnvironmentFiles", "UnitFileState", "DropInPaths")


def ve_unit_settings() -> dict:
    """F24 (the orchestrator's condition, 2026-11-04): VE's unit as found, read
    before a run that stops and restarts it, and compared after."""
    out = sh("systemctl", "show", "-p", ",".join(VE_UNIT_PROPS), VE_UNIT).stdout
    props = dict(line.split("=", 1) for line in out.splitlines() if "=" in line)
    if "ExecStart" in props:            # its runtime fields change with every start, not the setting
        props["ExecStart"] = re.sub(r" ; (start_time|stop_time|pid|code|status)=[^;}]*", "", props["ExecStart"])
    return props


def ladder_of(battery: object) -> dict:
    """`GET /battery`'s ladder block (`reserve_s`, `shutdown`, `poweroff_cmd`
    live under `ladder`, beside the top level's `v` and `level`)."""
    return ((battery or {}).get("ladder") or {}) if isinstance(battery, dict) else {}


def f24_env(brain_url: str) -> dict:
    """The robot server's shutdown steps and its stubbed power-off (3.137)."""
    steps = [["brain", f'curl -sf -m 20 -X POST -H "x-app-secret: $APP_SHARED_SECRET" '
                       f'-H "Content-Type: application/json" -d "{{}}" {brain_url}/mission/stop'],
             ["ros", f"docker stop -t 10 {rosmod.CONTAINER}"],
             ["vlm", f"sudo -n systemctl stop {VE_UNIT}"]]
    return {"BATTERY_SHUTDOWN_CMDS": json.dumps(steps), "BATTERY_POWEROFF_CMD": ""}


def judge(row: str, ms: dict) -> dict:
    spec = ROWS[row]
    r = []

    def bar(ok, why):
        if not ok:
            r.append(why)

    if ms.get("fault_at") is None:
        return {"pass": False, "valid": False, "reasons": ["the fault was never injected"]}
    valid = True
    if spec["when"] == "goal":
        if ms.get("forward_at_fault") is not True or ms.get("goal_at_fault") is not True:
            valid = False
            r.append(f"invalid: not driving forward on an active goal at the fault "
                     f"(forward {ms.get('forward_at_fault')}, goal {ms.get('goal_at_fault')})")
    else:
        if not ms.get("question_hit"):
            valid = False
            r.append(f"invalid: no {spec['when']} question was open at the fault")
    bar(ms.get("contacts") == 0, f"{ms.get('contacts')} contact samples")
    bar(ms.get("gap_unknown", 1) == 0, f"{ms.get('gap_unknown')} truth samples with no gap (contact unjudged)")
    bar(ms.get("truth_hole_s", 9) <= 0.2, f"truth log has a {ms.get('truth_hole_s')} s hole")
    out = ms.get("outcome")
    if spec["fault"] == "oom":
        bar(ms.get("oom_kills") == 1, f"{ms.get('oom_kills')} OOM kills in the window (bar 1, VE's)")
        bar(ms.get("killed_ve") is True, f"the kill named {ms.get('killed')}, not VE")
        bar(ms.get("alive_after") == {"robot": True, "brain": True, "ros": True},
            f"alive after: {ms.get('alive_after')}")
        bar(ms.get("late_ticks") == 0, f"{ms.get('late_ticks')} late wheel ticks in the window (bar 0)")
        bar(out not in (None, "running"), f"the mission ended {out}")
    if spec["fault"] == "kill_ve":
        h = ms.get("handed_on")
        bar(h is not None and h >= 1, f"{h} questions handed to the cloud while VE was gone (bar >= 1)")
        b = ms.get("ve_back_s")
        bar(b is not None and b <= VE_BACK_BAR_S, f"VE ready {b} s after the kill (bar 60)")
        bar(ms.get("next_answer") is True, "the next mission's first frame question was not answered")
        if row == "F3f":
            bar(ms.get("failed_in_watch") is False, f"the mission failed after the kill ({out})")
        if row == "F3a":
            bar(out == "found", f"the mission ended {out}, not found")
        if row == "F3a2":
            v = (ms.get("status") or {}).get("arrival_verdicts") or []
            bar(out == "found", f"the mission ended {out}, not found")
            bar(bool(v) and v[-1].get("tier") == "cloud" and v[-1].get("confirmed") is True,
                f"the deciding arrival verdict was not the cloud's yes ({v[-1] if v else None})")
            if ms.get("standin_blind"):
                valid = False
                r.append(f"invalid: the stand-in answered {ms['standin_blind']} questions with no pose")
        if row == "F3r":
            bar(out == "arrived_unconfirmed", f"the mission ended {out}, not arrived_unconfirmed")
    if spec["fault"] == "battery":
        if ms.get("reserve_at") is None or ms.get("poweroff_at") is None:
            valid = False
            r.append(f"invalid: the ladder did not reach Reserve and Power-off "
                     f"(reserve {ms.get('reserve_at')}, poweroff {ms.get('poweroff_at')})")
        if ms.get("running_at_reserve") is not True:
            valid = False
            r.append("invalid: the mission was not running when Reserve latched")
        mv = ms.get("moved_after_reserve_cm")
        bar(mv is not None and mv <= F24_MOVE_BAR_CM,
            f"moved {mv} cm after Reserve (bar {F24_MOVE_BAR_CM:g})")
        steps = ms.get("shutdown") or []
        names = [x.get("step") for x in steps]
        bar(names.count("poweroff") == 1, f"the power-off stub called {names.count('poweroff')} times (bar 1)")
        bar(names == F24_STEPS, f"the shutdown order {names}, not {F24_STEPS}")
        bad = [x for x in steps if x.get("step") in ("wheels", "brain", "ros", "vlm") and x.get("rc") != 0]
        bar(not bad, f"shutdown steps not clean: {bad}")
        bar(ms.get("down_at_poweroff") == {"mission": False, "ros": False, "vlm": False},
            f"still up at the power-off: {ms.get('down_at_poweroff')}")
    if spec["fault"] == "stop":
        z = ms.get("wheels_zero_from_stop_s")
        bar(z is not None and z <= STOP_ZERO_BAR_S, f"wheels zero {z} s after the /stop (bar 0.2)")
        bar(out == "cancelled" and ms.get("cancelled_by") == "stop",
            f"the mission ended {out} (by {ms.get('cancelled_by')}), not cancelled by stop")
        bar(ms.get("paid_after_stop") == 0, f"{ms.get('paid_after_stop')} paid calls after the Stop (bar 0)")
    return {"pass": not r, "valid": valid, "reasons": r}


def sh(*argv, timeout=30, input=None) -> subprocess.CompletedProcess:
    return subprocess.run(list(argv), capture_output=True, text=True, timeout=timeout, input=input)


def ve_ready() -> bool:
    try:
        code, body = http("GET", VE_URL + "/health", timeout=2)
        return code == 200 and isinstance(body, dict) and body.get("ready") is True
    except OSError:
        return False


def reset_start_limit() -> None:
    """Amendment 16: `picar-local-vlm` allows 4 starts in 900 s (StartLimitBurst=4),
    and every kill_ve / oom run restarts VE (the ve_oom0 plant twice more). Before
    each run its start-rate counter is flushed (`systemctl reset-failed`, as
    systemd.unit documents), so a run's one restart is never refused by the
    runs before it. A single fault restarts VE once; the limit is not under test."""
    r = sh("sudo", "-n", "systemctl", "reset-failed", VE_UNIT)
    if r.returncode != 0:
        raise StackError(f"reset-failed {VE_UNIT}: {r.stderr.strip()[:200]}")


def ve_driver_pid() -> int | None:
    """VE's driver: the unit's main process's child (`edgellm_identity`)."""
    main = sh("systemctl", "show", "-p", "MainPID", "--value", VE_UNIT).stdout.strip()
    if not main or main == "0":
        return None
    kids = sh("pgrep", "-P", main).stdout.split()
    return int(kids[0]) if kids else None


def open_questions(kind: str) -> list:
    try:
        code, body = http("GET", f"http://127.0.0.1:{PROXY_PORT}/fmea/open", timeout=1)
    except OSError:
        return []
    return [q for q in (body or {}).get("open", []) if q.get("kind") == kind] if code == 200 else []


def proxy_log(path: Path) -> list:
    if not path.exists():
        return []
    out = []
    for line in path.read_text().splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            pass
    return out


DROPIN = f"/etc/systemd/system/{VE_UNIT}.service.d/99-fmea-3135-oom0.conf"
UNIT_OOM_ADJ = "1000"           # the unit's own (3.80.B), checked after the drop-in goes


def ve_oom_dropin(on: bool) -> dict:
    """F2's planted check (the user's yes, 2026-11-03): VE's OOMScoreAdjust
    at 0 by a drop-in, the unit restarted under it; `on=False` removes it,
    restarts the unit and checks it is back at its own 1000 and ready."""
    if on:
        sh("sudo", "-n", "mkdir", "-p", str(Path(DROPIN).parent))
        sh("sudo", "-n", "tee", DROPIN, input="[Service]\nOOMScoreAdjust=0\n")
    else:
        sh("sudo", "-n", "rm", "-f", DROPIN)
        sh("sudo", "-n", "rmdir", "--ignore-fail-on-non-empty", str(Path(DROPIN).parent))
    sh("sudo", "-n", "systemctl", "daemon-reload")
    sh("sudo", "-n", "systemctl", "restart", VE_UNIT, timeout=120)
    end = time.monotonic() + 300
    while not ve_ready() and time.monotonic() < end:
        time.sleep(2.0)
    adj = sh("systemctl", "show", "-p", "OOMScoreAdjust", "--value", VE_UNIT).stdout.strip()
    res = {"on": on, "unit_oom_score_adjust": adj, "ready": ve_ready()}
    if not on and (adj != UNIT_OOM_ADJ or Path(DROPIN).exists()):
        raise StackError(f"the ve_oom0 drop-in was not undone: {res}")
    if on and adj != "0":
        raise StackError(f"the ve_oom0 drop-in did not take: {res}")
    return res


def absent_count(status: dict) -> Optional[int]:
    """The tiers' own count of questions they found VE absent for and handed
    on (`status.tier.stats` `local.absent` + `local_frame.absent`, 3.80), or
    None when the status carries no stats yet."""
    stats = ((status or {}).get("tier") or {}).get("stats") or {}
    if not stats:
        return None
    return int((stats.get("local") or {}).get("absent", 0)) + int((stats.get("local_frame") or {}).get("absent", 0))


def oom_count() -> int:
    for line in Path("/proc/vmstat").read_text().splitlines():
        if line.startswith("oom_kill "):
            return int(line.split()[1])
    raise StackError("no oom_kill in /proc/vmstat")


def run_row(row: str, out: Path, plant: str = "", house: str = "scaled_house") -> dict:
    spec = ROWS[row]
    if spec["cloud"] == "truth" and house != "scaled_house":
        raise SystemExit("the truthful stand-in knows scaled_house's target only (amendment 17)")
    out.mkdir(parents=True, exist_ok=True)
    rec = {"row": row, "title": spec["title"], "reading": spec["reading"], "plant": plant or None,
           "house": house, "host": os.uname().nodename, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    ms: dict = {"fault_at": None}
    plog, calls_log = out / "vlm_proxy.jsonl", out / "cloud_calls.jsonl"
    helpers: dict = {}
    secret = read_vlm_secret()
    mw, st, ros = MemWatch(), None, None
    stop_epoch = None
    with mw:
        try:
            if not secret:
                raise StackError("no APP_SHARED_SECRET in the VLM's env file")
            if plant == "ve_oom0":
                ms["plant_dropin"] = ve_oom_dropin(True)
            if spec["fault"] in ("kill_ve", "oom"):
                reset_start_limit()
            if spec["fault"] == "battery":
                ms["ve_unit_before"] = ve_unit_settings()
            if not ve_ready():
                raise StackError("VE is not ready at the start")
            ms["ve_before"] = sh("systemctl", "show", "-p", "MainPID,NRestarts", VE_UNIT).stdout.split()
            helpers["proxy"] = subprocess.Popen([PY, str(HERE / "vlm_proxy.py"), str(PROXY_PORT), VE_URL, str(plog)],
                                                cwd=ROOT, stdout=open(out / "proxy.log", "ab"),
                                                stderr=subprocess.STDOUT, start_new_session=True)
            if spec["cloud"] == "standin":
                helpers["stub"] = subprocess.Popen(
                    [PY, str(ROOT / "evaluations/tiered-380/E/standin_cloud.py"), str(STUB_PORT), str(calls_log)],
                    cwd=ROOT, stdout=open(out / "stub.log", "ab"), stderr=subprocess.STDOUT, start_new_session=True)
            elif spec["cloud"] == "truth":             # amendment 17: answers from the simulator's truth
                helpers["stub"] = subprocess.Popen(
                    [PY, str(HERE / "truth_cloud.py"), str(STUB_PORT), str(calls_log),
                     f"http://127.0.0.1:{PORTS['body']}"],
                    cwd=ROOT, stdout=open(out / "stub.log", "ab"), stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(1.5)
            env = {**rosmod.ROS_ENV, "TIER_SEARCH": "explore", "LOCAL_VLM_URL": f"http://127.0.0.1:{PROXY_PORT}",
                   "LOCAL_VLM_SECRET": secret,
                   # amendment 12: the arrival's cloud step is the cloud (stand-in), not the sim's geometry
                   "SIM_ARRIVAL_CONFIRM": "cloud"}
            if spec["fault"] == "battery":
                env.update(f24_env(f"http://127.0.0.1:{PORTS['brain']}"))
            url = (f"http://127.0.0.1:{STUB_PORT}" if spec["cloud"] in ("standin", "truth")
                   else f"http://127.0.0.1:{CLOSED_PORT}")
            # ve_oom0 is the unit's drop-in, not a process plant: the stack runs as shipped.
            st = Stack(out, house=house, plant="" if plant == "ve_oom0" else plant, vision_url=url,
                       extra_env=env, brain_floor_off=spec["fault"] == "battery")
            st.start()
            for p in st.procs.values():
                sh("sudo", "-n", "tee", f"/proc/{p.pid}/oom_score_adj", input=f"{STACK_OOM_ADJ}\n")
            ros = rosmod.Ros(st, out)
            ros.up()
            h0 = robot_health(st)
            code, body = st.brain("POST", "/mission/start", {"target_object": "red backpack", "policy": "tiered"},
                                  timeout=600)
            if code != 200:
                raise StackError(f"mission start: {code} {str(body)[:300]}")
            ms["start_at"] = time.monotonic()
            # ---- when: the fault's moment
            if spec["when"] == "goal":
                # "During an explore goal": an active goal and the board driving forward
                # (amendment 10; under tiered explore the brain's own verbs share the goal).
                for _ in range(10):
                    wait_forward(st, GOAL_MOVE_WAIT_S, goal=True)
                    time.sleep(0.15)
                    n = st.now()
                    if isinstance(n, dict) and m.is_forward(n.get("sp")) and rosmod.goal_state(st) == "active":
                        ms["goal_at_fault"] = True
                        ms["holder_at_fault"] = robot_health(st).get("authority_holder")
                        break
                else:
                    raise StackError("no FORWARD on an active goal lasted 0.15 s in 10 tries: not fired")
            else:
                end = time.monotonic() + QUESTION_WAIT_S
                while time.monotonic() < end:
                    if open_questions(spec["when"]):
                        break
                    s = status(st)
                    if s.get("running") is False:
                        raise StackError(f"the mission ended ({s.get('outcome')}) before a "
                                         f"{spec['when']} question opened")
                    time.sleep(0.05)
                else:
                    raise StackError(f"no {spec['when']} question opened in {QUESTION_WAIT_S:.0f} s")
            # ---- the fault
            if spec["fault"] == "kill_ve":
                pid = ve_driver_pid()
                if pid is None:
                    raise StackError("VE's driver not found")
                ms["killed_pid"] = pid
                ms["absent_before"] = absent_count(status(st))
                ms["fault_at"] = time.monotonic()
                sh("kill", "-9", str(pid))
            elif spec["fault"] == "battery":
                # The orchestrator's condition: the power-off is 3.137's logged stub on
                # this stack (a real one runs only under mode: hardware), or no fault.
                code, b0 = st.robot("GET", "/battery", timeout=3)
                ms["poweroff_cmd_at_start"] = ladder_of(b0).get("poweroff_cmd") if code == 200 else None
                if ms["poweroff_cmd_at_start"] != "stub":
                    raise StackError(f"the power-off is not the stub here ({code}, {b0}): not run")
                ms["fault_at"] = time.monotonic()
                code, _ = http("POST", st.urls["body"] + f"/fmea/battery?v={F24_V0}&drain={F24_DRAIN}", timeout=5)
                if code != 200:
                    raise StackError(f"the body's battery: {code}")
                end = ms["fault_at"] + F24_WAIT_S
                while time.monotonic() < end:
                    code, b = st.robot("GET", "/battery", timeout=3)
                    now = time.monotonic()
                    if code == 200 and isinstance(b, dict):
                        b = ladder_of(b)
                        if ms.get("reserve_at") is None and b.get("reserve_s") is not None:
                            ms["reserve_at"] = now - float(b["reserve_s"])
                            # A mission that had already ended moves nothing after Reserve:
                            # no test of the guard (and the plant would slip past it).
                            ms["running_at_reserve"] = bool(status(st).get("running"))
                        steps = b.get("shutdown") or []
                        if any(x.get("step") == "poweroff" for x in steps):
                            ms["poweroff_at"] = now
                            ms["down_at_poweroff"] = {
                                "mission": bool(status(st).get("running")),
                                "ros": bool(rosmod.docker("ps", "-q", "-f", f"name=^{rosmod.CONTAINER}$").stdout.strip()),
                                "vlm": sh("systemctl", "is-active", VE_UNIT).stdout.strip() == "active"}
                            time.sleep(2.0)                  # a second stub would land here
                            ms["battery_end"] = st.robot("GET", "/battery", timeout=3)[1]
                            ms["shutdown"] = ladder_of(ms["battery_end"]).get("shutdown")
                            break
                    time.sleep(0.2)
            elif spec["fault"] == "stop":
                stop_epoch = time.time()
                ms["fault_at"] = ms["stop_at"] = time.monotonic()
                ms["stop_code"] = st.robot("POST", "/stop", {}, timeout=10)[0]       # unnamed: a person
                ms["stop_answer_s"] = round(time.monotonic() - ms["stop_at"], 3)
            elif spec["fault"] == "oom":
                ms["oom_before"] = oom_count()
                main = sh("systemctl", "show", "-p", "MainPID", "--value", VE_UNIT).stdout.strip()
                ms["ve_pids"] = [int(x) for x in (main, ve_driver_pid()) if x and str(x) != "0"]
                hb = robot_health(st)
                ms["fault_at"] = time.monotonic()
                hog = subprocess.run([PY, str(ROOT / "tools/jetson/memory_hog.py"), "120", str(ve_driver_pid() or "")],
                                     cwd=ROOT, capture_output=True, text=True, timeout=200)
                (out / "hog.log").write_text(hog.stdout + hog.stderr)
                ms["hog_end_at"] = time.monotonic()
                time.sleep(30.0)
                ha = robot_health(st)
                ms["oom_kills"] = oom_count() - ms["oom_before"]
                dm = sh("sudo", "-n", "dmesg").stdout
                kills = re.findall(r"Killed process (\d+) \(([^)]+)\)", dm)
                ms["killed"] = kills[-ms["oom_kills"]:] if ms["oom_kills"] else []
                ms["killed_ve"] = (ms["oom_kills"] == 1 and len(ms["killed"]) == 1
                                   and int(ms["killed"][0][0]) in ms["ve_pids"])
                ms["alive_after"] = {"robot": st.alive("robot"), "brain": st.alive("brain"),
                                     "ros": bool(rosmod.docker("ps", "-q", "-f", f"name=^{rosmod.CONTAINER}$").stdout.strip())}
                if hb and ha:
                    ms["late_ticks"] = ha["wheel_loop"]["late_ticks"] - hb["wheel_loop"]["late_ticks"]
            # ---- what the fault left
            if spec["fault"] == "kill_ve":
                end = ms["fault_at"] + 180
                down_seen = False
                while time.monotonic() < end:
                    if not ve_ready():
                        down_seen = True
                    elif down_seen:
                        ms["ve_back_s"] = round(time.monotonic() - ms["fault_at"], 1)
                        break
                    time.sleep(0.5)
                # Handed on while VE was gone: the tiers' absent count, kill to VE back (or now).
                after = absent_count(status(st))
                # Amendment 16: no stats at the kill (its first question) is a baseline of 0;
                # no stats after it stays unreadable (None), never 0 handed on.
                if after is not None:
                    ms["handed_on"] = after - (ms.get("absent_before") or 0)
                if row == "F3f":
                    watch_end = ms["fault_at"] + WATCH_AFTER_S
                    failed = False
                    while time.monotonic() < watch_end:
                        s = status(st)
                        if s.get("running") is False and s.get("outcome") not in (None, "idle", "running"):
                            failed = s.get("outcome") == "failed"
                            ms["outcome"] = s.get("outcome")
                            break
                        time.sleep(1.0)
                    ms["failed_in_watch"] = failed
                    if status(st).get("running"):
                        st.brain("POST", "/mission/stop", {}, timeout=10)
                        ms["outcome"] = ms.get("outcome") or "stopped_by_harness"
                    wait_end(st, 60)
                else:
                    s1 = wait_end(st, MISSION_WAIT_S)
                    ms["outcome"] = s1.get("outcome") if s1.get("running") is False else None
                    ms["status"] = {k: s1.get(k) for k in ("outcome", "end_state", "end_reason", "error",
                                                           "arrival_verdicts", "cancelled_by")}
                # the next mission's first frame question, answered by VE
                st.robot("POST", "/autonomy/resume", {})
                t_next = time.monotonic()
                code, _ = st.brain("POST", "/mission/start", {"target_object": "red backpack", "policy": "tiered"},
                                   timeout=600)
                end = time.monotonic() + NEXT_ANSWER_WAIT_S
                ms["next_answer"] = False
                while code == 200 and time.monotonic() < end:
                    if any(q.get("kind") == "frame" and q.get("status") == 200 and q.get("t0", 0) >= t_next
                           for q in proxy_log(plog)):
                        ms["next_answer"] = True
                        ms["next_answer_s"] = round(time.monotonic() - t_next, 1)
                        break
                    time.sleep(1.0)
                st.brain("POST", "/mission/stop", {}, timeout=10)
                wait_end(st, 60)
            if spec["fault"] == "stop":
                s1 = wait_end(st, CANCEL_WAIT_S)
                ms["outcome"] = s1.get("outcome") if s1.get("running") is False else "running"
                ms["cancelled_by"] = s1.get("cancelled_by")
                ms["status"] = {k: s1.get(k) for k in ("outcome", "end_state", "end_reason", "cancelled_by")}
                time.sleep(max(0.0, ms["stop_at"] + CANCEL_WAIT_S - time.monotonic()))
                if ms["outcome"] == "running":                 # the planted build drives on: end it
                    st.brain("POST", "/mission/stop", {}, timeout=10)
                    wait_end(st, 60)
            if spec["fault"] == "oom":
                s1 = wait_end(st, MISSION_WAIT_S)
                ms["outcome"] = s1.get("outcome") if s1.get("running") is False else "running"
                ms["status"] = {k: s1.get(k) for k in ("outcome", "end_state", "end_reason", "error")}
            h1 = robot_health(st)
            if h0 and h1:
                ms["wheel_loop"] = {"late": h1["wheel_loop"]["late_ticks"] - h0["wheel_loop"]["late_ticks"],
                                    "moving": h1["wheel_loop"]["moving_ticks"] - h0["wheel_loop"]["moving_ticks"],
                                    "max_dt_s": h1["wheel_loop"]["max_dt_s"]}
        except Exception as e:             # noqa: BLE001 -- recorded, never a lost result
            rec["error"] = f"{type(e).__name__}: {e}"
        finally:
            try:
                if ros is not None:
                    ros.down()
            except Exception as e:  # noqa: BLE001 -- the drop-in and the record still follow
                rec["teardown_error"] = f"ros down: {type(e).__name__}: {e}"
            try:
                if st is not None:
                    st.stop()
                for p in helpers.values():
                    if p.poll() is None:
                        os.killpg(p.pid, signal.SIGKILL)
            except Exception as e:  # noqa: BLE001
                rec["teardown_error"] = f"stack stop: {type(e).__name__}: {e}"
            if plant == "ve_oom0":
                try:
                    ms["plant_removed"] = ve_oom_dropin(False)
                except Exception as e:     # noqa: BLE001 -- never lose the run; the board check says so too
                    rec["error"] = f"the plant's removal: {type(e).__name__}: {e}"
            if spec["fault"] == "battery" and sh("systemctl", "is-active", VE_UNIT).stdout.strip() != "active":
                try:                                      # F24's shutdown stopped VE: put it back
                    reset_start_limit()
                    ms["ve_restarted"] = sh("sudo", "-n", "systemctl", "start", VE_UNIT, timeout=120).returncode
                except Exception as e:  # noqa: BLE001 -- the VE-ready check below records it
                    rec["teardown_error"] = f"VE restart: {type(e).__name__}: {e}"
            end = time.monotonic() + 180                  # VE as found before the next run
            while not ve_ready() and time.monotonic() < end:
                time.sleep(1.0)
            ms["ve_after"] = sh("systemctl", "show", "-p", "MainPID,NRestarts", VE_UNIT).stdout.split()
            ms["ve_ready_after"] = ve_ready()
            if ms.get("ve_unit_before") is not None:
                ms["ve_unit_after"] = ve_unit_settings()
                ms["ve_unit_as_found"] = ms["ve_unit_after"] == ms["ve_unit_before"]
    rows = m.read_truth(out / "truth.jsonl") if (out / "truth.jsonl").exists() else []
    plg = proxy_log(plog)
    t = ms.get("fault_at")
    if t is not None:
        ms["forward_at_fault"] = m.forward_at(rows, t)
        if spec["when"] in ("frame", "arrival"):
            hit = [q for q in plg if q.get("kind") == spec["when"] and q["t0"] <= t <= q.get("t1", 9e18)]
            ms["question_hit"] = bool(hit)
            ms["question_at_fault"] = hit[0] if hit else None
        if spec["fault"] == "battery" and ms.get("reserve_at") is not None:
            ms["moved_after_reserve_cm"] = round(m.travel_cm(rows, ms["reserve_at"], ms.get("poweroff_at")), 2)
        if spec["fault"] == "stop":
            ms["wheels_zero_from_stop_s"] = m.wheels_zero_s(rows, t)
            calls = proxy_log(calls_log)
            ms["paid_after_stop"] = sum(1 for c in calls if c.get("path") == "/navigate"
                                        and stop_epoch is not None and c.get("t", 0) > stop_epoch)
    sp = m.span(rows)
    ms["truth_hole_s"] = m.gaps_s(rows, sp[0], sp[1]) if sp else float("inf")
    ms["contacts"] = m.contacts(rows)
    ms["gap_unknown"] = sum(1 for r in rows if r.get("gap_cm") is None)
    ms["local_questions"] = {k: sum(1 for q in plg if q.get("kind") == k) for k in ("frame", "arrival", "direction")}
    if spec["cloud"] == "truth":                       # amendment 17: answers given blind are the instrument's
        ms["standin_blind"] = sum(1 for c in proxy_log(calls_log) if "pose" in c and c["pose"] is None)
    if mw.rows:
        ms["mem_available_min_mb"] = min(r[1] for r in mw.rows)
        ms.setdefault("oom_kills_window", mw.rows[-1][2] - mw.rows[0][2])
    rec["measures"] = ms
    rec["verdict"] = judge(row, ms)
    if ms.get("ve_unit_as_found") is False:
        rec["error"] = rec.get("error") or "VE's unit settings changed over the run (the board not as found)"
    if not ms.get("ve_ready_after"):
        rec["error"] = rec.get("error") or "VE was not ready after the case (the board not as found)"
    rec["ended"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    (out / "result.json").write_text(json.dumps(rec, indent=1, default=str))
    return rec


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("row", choices=sorted(ROWS))
    ap.add_argument("--out", required=True)
    ap.add_argument("--plant", default="")
    ap.add_argument("--house", default="scaled_house")
    a = ap.parse_args(argv)
    rec = run_row(a.row, Path(a.out), a.plant, a.house)
    print(json.dumps({"row": a.row, "plant": a.plant, "verdict": rec["verdict"], "error": rec.get("error"),
                      "measures": {k: v for k, v in rec["measures"].items() if k != "status"}}, default=str))
    return 0 if "error" not in rec else 3


if __name__ == "__main__":
    raise SystemExit(main())
