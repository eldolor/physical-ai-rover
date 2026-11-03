# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/sys_rows.py

3.135.B's clock row (amendment 7), one case run once, on `stack.py`'s stack
(A's: no ROS, no VE, the cloud a closed port), a frontier mission for the
red backpack in `scaled_house`:

| Case | Fault | Plant (criterion 4) |
|---|---|---|
| F16c | none: the fault-free control, whose outcome the others must match | -- |
| F16p | the wall clock stepped +5 min 0.15 s into a FORWARD, put back after the mission | `wall_clock` |
| F16m | the wall clock stepped -5 min 0.15 s into a FORWARD, put back after the mission | `wall_clock` |

The step is `sudo date -s` with NTP off (`timedatectl set-ntp false`); after
the case the clock is stepped back by the same amount and NTP turned on
again, and the case waits for `NTPSynchronized=yes` (recorded; sudo:
approved by the user 2026-11-01). On a machine where `FMEA_CLOCK=fake`
(the desktop's planted check: no sudo there) the step is not made on the
system clock: the planted robot server reads `FMEA_CLOCK_OFFSET_FILE`.

    python evaluations/jetson-fmea-3135/sys_rows.py F16p --out DIR [--plant wall_clock]
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import measure as m  # noqa: E402
from fmea import MOVE_WAIT_S, MemWatch, robot_health, start_mission, wait_end, wait_forward  # noqa: E402
from stack import Stack, StackError  # noqa: E402

STEP_S = 300.0
AFTER_S = 3.0              # the window a guard that read the jump would fire in
MISSION_WAIT_S = 1200 + 300
NTP_WAIT_S = 120.0

ROWS = {
    "F16c": {"title": "Wall clock (fault-free control)", "step_s": 0.0, "plant": "",
             "reading": "the outcome every stepped case must match"},
    "F16p": {"title": "Wall clock jumps +5 min", "step_s": STEP_S, "plant": "wall_clock",
             "reading": "no watchdog stop in the 3 s after the step (the robot server's watchdog "
                        "refusal count unchanged); no 503 or stale-frame line in the brain's log after "
                        "it; the mission does not end out_of_budget, and ends as the control's runs "
                        "(found); 0 contacts"},
    "F16m": {"title": "Wall clock jumps -5 min", "step_s": -STEP_S, "plant": "wall_clock",
             "reading": "as F16p, the step -5 min"},
}


def judge(row: str, ms: dict, control: str | None = "found") -> dict:
    r = []

    def bar(ok, why):
        if not ok:
            r.append(why)

    out = ms.get("outcome")
    if out is None:
        return {"pass": False, "valid": False, "reasons": ["the mission never ended"]}
    bar(ms.get("contacts") == 0, f"{ms.get('contacts')} contact samples")
    bar(ms.get("gap_unknown", 1) == 0, f"{ms.get('gap_unknown')} truth samples with no gap (contact unjudged)")
    bar(ms.get("truth_hole_s", 9) <= 0.2, f"truth log has a {ms.get('truth_hole_s')} s hole")
    if row == "F16c":
        return {"pass": not r, "valid": True, "reasons": r}
    valid = ms.get("stepped") is True
    if not valid:
        r.append("invalid: the clock was not stepped")
    bar(ms.get("watchdog_after_step") == 0,
        f"{ms.get('watchdog_after_step')} watchdog stops in the 3 s after the step (bar 0)")
    bar(ms.get("brain_503_lines") == 0, f"{ms.get('brain_503_lines')} 503/stale lines in the brain's log")
    bar(out != "out_of_budget", "the mission ended out_of_budget")
    bar(out == control, f"the mission ended {out}, the control's {control}")
    return {"pass": not r, "valid": valid, "reasons": r}


def sudo(*argv: str) -> subprocess.CompletedProcess:
    return subprocess.run(["sudo", "-n", *argv], capture_output=True, text=True, timeout=30)


class Clock:
    """The step, on the system clock (the board) or a fake offset (the desktop)."""

    def __init__(self, out: Path):
        self.fake = os.environ.get("FMEA_CLOCK") == "fake"
        self.offset_file = out / "clock_offset"
        self.offset_file.write_text("0\n")
        self.stepped = 0.0
        self.ntp_off = False

    def step(self, s: float) -> float:
        t = time.monotonic()
        if self.fake:
            self.stepped += s
            self.offset_file.write_text(f"{self.stepped}\n")
            return t
        if not self.ntp_off:
            r = sudo("timedatectl", "set-ntp", "false")
            if r.returncode != 0:
                raise StackError(f"set-ntp false: {r.stderr.strip()[:200]}")
            self.ntp_off = True
        t = time.monotonic()
        r = sudo("date", "-s", f"{'+' if s >= 0 else '-'}{abs(int(s))} seconds")
        if r.returncode != 0:
            raise StackError(f"date -s: {r.stderr.strip()[:200]}")
        self.stepped += s
        return t

    def restore(self) -> dict:
        """The step undone and NTP back; how it ended."""
        res = {}
        if self.stepped:
            if self.fake:
                self.offset_file.write_text("0\n")
            else:
                r = sudo("date", "-s", f"{'-' if self.stepped >= 0 else '+'}{abs(int(self.stepped))} seconds")
                res["step_back_rc"] = r.returncode
            self.stepped = 0.0
        if self.ntp_off:
            res["ntp_on_rc"] = sudo("timedatectl", "set-ntp", "true").returncode
            end = time.monotonic() + NTP_WAIT_S
            while time.monotonic() < end:
                q = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                                   capture_output=True, text=True, timeout=10).stdout.strip()
                if q == "yes":
                    break
                time.sleep(2.0)
            res["ntp_synchronized"] = q
            self.ntp_off = False
        return res


def run_row(row: str, out: Path, plant: str = "", house: str = "scaled_house") -> dict:
    spec = ROWS[row]
    out.mkdir(parents=True, exist_ok=True)
    rec = {"row": row, "title": spec["title"], "reading": spec["reading"], "plant": plant or None,
           "house": house, "host": os.uname().nodename, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    ms: dict = {"outcome": None, "stepped": False}
    clock = Clock(out)
    mw, st = MemWatch(), None
    with mw:
        try:
            st = Stack(out, house=house, plant=plant,
                       extra_env={"FMEA_CLOCK_OFFSET_FILE": str(clock.offset_file)})
            st.start()
            start_mission(st)
            ms["start_at"] = time.monotonic()
            if spec["step_s"]:
                for _ in range(10):
                    wait_forward(st, MOVE_WAIT_S)
                    time.sleep(0.15)
                    n = st.now()
                    if isinstance(n, dict) and m.is_forward(n.get("sp")):
                        break
                h0 = robot_health(st)
                log0 = (out / "brain.log").stat().st_size if (out / "brain.log").exists() else 0
                ms["step_at"] = clock.step(spec["step_s"])
                ms["stepped"] = True
                ms["fault_at"] = ms["step_at"]
                time.sleep(AFTER_S)
                h1 = robot_health(st)
                rc0, rc1 = (h0.get("refusal_counts") or {}), (h1.get("refusal_counts") or {})
                ms["watchdog_after_step"] = rc1.get("watchdog", 0) - rc0.get("watchdog", 0) if h1 else None
                ms["last_refusal_after_step"] = h1.get("last_refusal")
            s1 = wait_end(st, MISSION_WAIT_S)
            ms["end_at"] = time.monotonic()
            ms["outcome"] = s1.get("outcome") if s1.get("running") is False else None
            ms["mission_s"] = round(ms["end_at"] - ms["start_at"], 1)
            ms["status"] = {k: s1.get(k) for k in ("outcome", "end_state", "end_reason", "error", "step")}
            if spec["step_s"]:
                text = (out / "brain.log").read_bytes()[log0:].decode(errors="replace")
                ms["brain_503_lines"] = sum(1 for line in text.splitlines()
                                            if "503" in line or "stale" in line.lower())
        except Exception as e:             # noqa: BLE001 -- recorded, never a lost result
            rec["error"] = f"{type(e).__name__}: {e}"
        finally:
            try:
                rec["clock_restore"] = clock.restore()
            finally:
                if st is not None:
                    st.stop()
    rows = m.read_truth(out / "truth.jsonl") if (out / "truth.jsonl").exists() else []
    sp = m.span(rows)
    ms["truth_hole_s"] = m.gaps_s(rows, sp[0], sp[1]) if sp else float("inf")
    ms["contacts"] = m.contacts(rows)
    ms["gap_unknown"] = sum(1 for r in rows if r.get("gap_cm") is None)
    if ms.get("step_at") is not None:
        ms["forward_at_fault"] = m.forward_at(rows, ms["step_at"])
    if mw.rows:
        ms["mem_available_min_mb"] = min(r[1] for r in mw.rows)
        ms["oom_kills"] = mw.rows[-1][2] - mw.rows[0][2]
    rec["measures"] = ms
    control = os.environ.get("FMEA_F16_CONTROL", "found")
    rec["verdict"] = judge(row, ms, control)
    if rec.get("clock_restore", {}).get("ntp_synchronized") not in (None, "yes"):
        rec["error"] = f"NTP not synchronized after the case: {rec['clock_restore']}"
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
