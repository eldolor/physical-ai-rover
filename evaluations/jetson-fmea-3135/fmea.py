"""
evaluations/jetson-fmea-3135/fmea.py

One FMEA row, run once, on the machine this runs on (the desktop for
criterion 4, the board through `run.py` for 3.135.B): start the stack
(`stack.py`), get the robot moving through the real path, inject the row's
fault from outside the processes, clear it, score the row's measures from
ground truth (`measure.py`), try the recovery (a new mission), and write
`<out>/result.json`.

    python evaluations/jetson-fmea-3135/fmea.py F9 --out DIR [--plant lidar_fresh] [--house scaled_house]

The rows built in 3.135.A (amendment 1). Each bar is the row's as written in
the section; how each is read off the truth is in `ROWS[...]["reading"]`:

| Row | Fault | Moving by | Plant (criterion 4) |
|---|---|---|---|
| F6 | SIGKILL the brain mid-FORWARD | a frontier mission | `watchdog` |
| F7 | SIGSTOP the brain 30 s mid-FORWARD, then SIGCONT | a frontier mission | `watchdog` |
| F8 | SIGKILL the robot server mid-FORWARD | a frontier mission | `heartbeat` |
| F9 | SIGSTOP the fake lidar 10 s mid-FORWARD | a frontier mission | `lidar_fresh` |
| F10 | the fake board's feedback frames silenced 5 s mid-FORWARD | a frontier mission | `feedback_fresh` |
| F13 | SIGKILL a person's D-pad client mid-press (a standing `/wheels`) | the client | `watchdog` |
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import measure as m  # noqa: E402
from stack import PY, Stack, StackError, http  # noqa: E402

LIDAR_STALE_S = 0.3        # robot/lidar_ld19.py STALE_S
FEEDBACK_STALE_S = 0.25    # robot/hardware_robot.py FEEDBACK_STALE_S
WATCH_S = 6.0              # the truth window after a stop fault, to rest
MOVE_WAIT_S = 90.0

ROWS = {
    "F6": {"title": "Brain killed mid-move", "plant": "watchdog", "mover": "mission",
           "reading": "wheels zero <= 1.1 s after the SIGKILL; true path from the kill over "
                      "the next 6 s <= 15 cm; 0 contacts"},
    "F7": {"title": "Brain hung (alive, silent)", "plant": "watchdog", "mover": "mission",
           "hold_s": 30.0,
           "reading": "as F6 from the SIGSTOP; and after SIGCONT the stale mission does not "
                      "drive: no non-zero wheel setpoint in the 10 s after SIGCONT; 0 contacts"},
    "F8": {"title": "Robot server killed", "plant": "heartbeat", "mover": "mission",
           "reading": "the fake board's setpoints zero <= 1.6 s after the SIGKILL; the "
                      "brain's mission does not end found; 0 contacts"},
    "F9": {"title": "Lidar stops (no scans)", "plant": "lidar_fresh", "mover": "mission",
           "hold_s": 10.0,
           "reading": "true path from the SIGSTOP + 0.3 s (the scan's freshness bar) to the "
                      "SIGCONT <= 2 cm; 0 contacts"},
    "F10": {"title": "Wheel feedback stops", "plant": "feedback_fresh", "mover": "mission",
            "hold_s": 5.0,
            "reading": "true path from the silence + 0.25 s (feedback's freshness bar) to its "
                       "end <= 2 cm; the robot server refused something as no_feedback; "
                       "0 contacts"},
    "F13": {"title": "Twin / tunnel link lost while a person drives", "plant": "watchdog",
            "mover": "dpad",
            "reading": "wheels zero <= 1.1 s after the client's SIGKILL; 0 contacts "
                       "(no mission runs, so 'the brain does not take over mid-press' is not "
                       "exercised here)"},
}


def judge(row: str, ms: dict) -> dict:
    """The row's pass from its measures; every failed bar is a reason."""
    r = []

    def bar(ok, why):
        if not ok:
            r.append(why)

    if ms.get("fault_at") is None:
        return {"pass": False, "valid": False,
                "reasons": ["the fault was never injected (the robot never moved)"]}
    valid = ms.get("forward_at_fault") is True
    if not valid:
        r.append("invalid: the board was not driving forward when the fault fired")
    bar(ms.get("gap_unknown", 0) == 0, f"{ms.get('gap_unknown')} truth samples with no gap (contact unjudged)")
    bar(ms.get("truth_hole_s", 9) <= 0.2, f"truth log has a {ms.get('truth_hole_s')} s hole")
    bar(ms.get("contacts") == 0, f"{ms.get('contacts')} contact samples")
    z = ms.get("wheels_zero_s")
    if row in ("F6", "F7", "F13"):
        bar(z is not None and z <= 1.1, f"wheels zero after {z} s (bar 1.1)")
    if row in ("F6", "F7"):
        bar(ms["travel_cm"] <= 15.0, f"moved {ms['travel_cm']} cm after the fault (bar 15)")
    if row == "F7":
        bar(ms["drive_after_cont"] == 0,
            f"{ms['drive_after_cont']} driving samples in the 10 s after SIGCONT (bar 0)")
    if row == "F8":
        bar(z is not None and z <= 1.6, f"wheels zero after {z} s (bar 1.6)")
        bar(ms.get("outcome") != "found", f"the mission ended {ms.get('outcome')}")
    if row in ("F9", "F10"):
        bar(ms["travel_stale_cm"] <= 2.0, f"moved {ms['travel_stale_cm']} cm while stale (bar 2)")
    if row == "F10":
        bar((ms.get("no_feedback_refusals") or 0) >= 1, "no refusal named no_feedback")
    return {"pass": not r, "valid": valid, "reasons": r}


# ------------------------------------------------------------------ helpers

class MemWatch:
    """MemAvailable (MB) and the OOM-kill counter, once a second."""

    def __init__(self):
        self.rows, self._stop = [], threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop.is_set():
            mem = next(int(l.split()[1]) // 1024 for l in open("/proc/meminfo")
                       if l.startswith("MemAvailable:"))
            oom = next(int(l.split()[1]) for l in open("/proc/vmstat") if l.startswith("oom_kill "))
            self.rows.append((time.monotonic(), mem, oom))
            self._stop.wait(1.0)

    def __enter__(self):
        self._t.start()
        return self

    def __exit__(self, *a):
        self._stop.set()
        self._t.join(timeout=3)


def wait_forward(st: Stack, timeout_s: float = MOVE_WAIT_S) -> float:
    """Poll the truth until the board drives both wheels forward; the stamp."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        n = st.now()
        if isinstance(n, dict) and m.is_forward(n.get("sp")):
            return time.monotonic()
        time.sleep(0.02)
    raise StackError(f"the robot never drove forward in {timeout_s:.0f} s")


def start_mission(st: Stack) -> dict:
    code, body = st.brain("POST", "/mission/start",
                          {"target_object": "red backpack", "policy": "frontier"})
    if code != 200:
        raise StackError(f"mission start: {code} {str(body)[:300]}")
    return body


def status(st: Stack) -> dict:
    try:
        code, body = st.brain("GET", "/mission/status", timeout=5)
        return body if code == 200 and isinstance(body, dict) else {"error": f"{code}"}
    except OSError as e:
        return {"error": str(e)}


def robot_health(st: Stack) -> dict:
    try:
        code, body = st.robot("GET", "/health", timeout=5)
        return body if code == 200 and isinstance(body, dict) else {}
    except OSError:
        return {}


def recover(st: Stack, rec: dict) -> None:
    """The progress half (section 7.4): a new mission running, no person.
    The desktop restarts a killed role itself (systemd's job on the board)."""
    t0 = time.monotonic()
    try:
        for role, starter in (("robot", st.start_robot), ("brain", st.start_brain)):
            if not st.alive(role):
                starter()
                rec.setdefault("restarted", []).append(role)
        if status(st).get("running"):
            st.brain("POST", "/mission/stop", {})
            end = time.monotonic() + 40       # the brain's own close waits are 10 s + 30 s
            while status(st).get("running") and time.monotonic() < end:
                time.sleep(0.5)
            if status(st).get("running"):
                raise StackError("the old mission was still running 40 s after its Stop")
        st.robot("POST", "/autonomy/resume", {})
        start_mission(st)
        wait_forward(st, 60.0)
        rec["recovery_s"] = round(time.monotonic() - t0, 2)
    except Exception as e:                 # noqa: BLE001 -- recovery is reported, not the row
        rec["recovery_error"] = f"{type(e).__name__}: {e}"


# --------------------------------------------------------------------- run

def run_row(row: str, out: Path, plant: str = "", house: str = "scaled_house") -> dict:
    spec = ROWS[row]
    out.mkdir(parents=True, exist_ok=True)
    rec = {"row": row, "title": spec["title"], "reading": spec["reading"], "plant": plant or None,
           "house": house, "host": os.uname().nodename, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    ms: dict = {"fault_at": None}
    dpad = None
    mw, st = MemWatch(), Stack(out, house=house, plant=plant)
    with mw:
        try:
            st.start()                     # a stack that will not start is a recorded error
            h0 = robot_health(st)
            if spec["mover"] == "mission":
                start_mission(st)
            else:
                dpad = subprocess.Popen([PY, str(HERE / "dpad_client.py"), st.urls["robot"], st.secret],
                                        stdout=open(out / "dpad.log", "w"), stderr=subprocess.STDOUT,
                                        start_new_session=True)
            for _ in range(10):            # fire 0.15 s into a FORWARD that is still on
                wait_forward(st)
                time.sleep(0.15)
                n = st.now()
                if isinstance(n, dict) and m.is_forward(n.get("sp")):
                    break
            else:
                raise StackError("no FORWARD lasted 0.15 s in 10 tries: the fault was not fired")
            t = time.monotonic()
            hold = spec.get("hold_s", 0.0)
            if row == "F6":
                t = st.signal("brain", signal.SIGKILL)
            elif row == "F7":
                t = st.signal("brain", signal.SIGSTOP)
                time.sleep(hold)
                ms["cont_at"] = st.signal("brain", signal.SIGCONT)
                time.sleep(10.5)
            elif row == "F8":
                t = st.signal("robot", signal.SIGKILL)
            elif row == "F9":
                t = st.signal("lidar", signal.SIGSTOP)
                time.sleep(hold)
                ms["cont_at"] = st.signal("lidar", signal.SIGCONT)
            elif row == "F10":
                code, body = http("POST", f"{st.urls['body']}/fmea/silence?s={hold}")
                if code != 200 or not isinstance(body, dict):
                    raise StackError(f"the body refused the silence: {code} {str(body)[:200]}")
                t = body["silent_from"]
                time.sleep(hold + 0.2)
                ms["cont_at"] = body["silent_until"]
            elif row == "F13":
                t = time.monotonic()
                os.killpg(dpad.pid, signal.SIGKILL)
                dpad.wait(timeout=5)
            ms["fault_at"] = t
            time.sleep(WATCH_S if row in ("F6", "F8", "F13") else 1.0)
            if row == "F8":
                time.sleep(4.0)            # the brain finds the robot gone
            h1 = robot_health(st)
            s1 = status(st)
            ms["outcome"] = s1.get("outcome")
            ms["end_state"], ms["end_reason"] = s1.get("end_state"), s1.get("end_reason")
            ms["status"] = {k: s1.get(k) for k in ("running", "outcome", "end_state", "end_reason",
                                                   "cancelled_by", "step", "last_action", "error")}
            if h0 and h1:
                rc0, rc1 = h0.get("refusal_counts") or {}, h1.get("refusal_counts") or {}
                ms["refusals"] = {k: rc1.get(k, 0) - rc0.get(k, 0) for k in rc1
                                  if rc1.get(k, 0) != rc0.get(k, 0)}
                ms["no_feedback_refusals"] = ms["refusals"].get("no_feedback", 0)
                ms["wheel_loop"] = {"late": h1["wheel_loop"]["late_ticks"] - h0["wheel_loop"]["late_ticks"],
                                    "moving": h1["wheel_loop"]["moving_ticks"]
                                    - h0["wheel_loop"]["moving_ticks"],
                                    "max_dt_s": h1["wheel_loop"]["max_dt_s"]}
                ms["last_refusal"] = h1.get("last_refusal")
            recover(st, rec)
        except Exception as e:             # noqa: BLE001 -- recorded, never a lost result
            rec["error"] = f"{type(e).__name__}: {e}"
        finally:
            if dpad is not None and dpad.poll() is None:
                os.killpg(dpad.pid, signal.SIGKILL)
            st.stop()
    rows = m.read_truth(out / "truth.jsonl") if (out / "truth.jsonl").exists() else []
    t = ms["fault_at"]
    if t is not None:
        ms["forward_at_fault"] = m.forward_at(rows, t)
        ms["wheels_zero_s"] = m.wheels_zero_s(rows, t)
        ms["travel_cm"] = m.travel_cm(rows, t, t + WATCH_S)
        holes = [m.gaps_s(rows, t - 1.0, t + WATCH_S)]      # every window a bar reads
        if row in ("F9", "F10"):
            stale = LIDAR_STALE_S if row == "F9" else FEEDBACK_STALE_S
            ms["travel_stale_cm"] = m.travel_cm(rows, t + stale, ms["cont_at"])
            holes.append(m.gaps_s(rows, t, ms["cont_at"]))
        if row == "F7":
            ms["drive_after_cont"] = m.nonzero_after(rows, ms["cont_at"], ms["cont_at"] + 10.0)
            holes.append(m.gaps_s(rows, ms["cont_at"], ms["cont_at"] + 10.0))
        sp = m.span(rows)
        holes.append(m.gaps_s(rows, sp[0], sp[1]))           # contacts read the whole log
        ms["truth_hole_s"] = max(holes)
    ms["contacts"] = m.contacts(rows)
    ms["gap_unknown"] = sum(1 for r in rows if r.get("gap_cm") is None)
    ms["truth_span"] = m.span(rows)
    if mw.rows:
        ms["mem_available_min_mb"] = min(r[1] for r in mw.rows)
        ms["oom_kills"] = mw.rows[-1][2] - mw.rows[0][2]
    rec["measures"] = ms
    rec["verdict"] = judge(row, ms)
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
    print(json.dumps({"row": a.row, "plant": a.plant, "verdict": rec["verdict"],
                      "error": rec.get("error"),
                      "measures": {k: v for k, v in rec["measures"].items() if k != "status"}},
                     default=str))
    return 0 if "error" not in rec else 3


if __name__ == "__main__":
    raise SystemExit(main())
