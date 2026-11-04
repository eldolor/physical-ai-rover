# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/cloud_rows.py

3.135.B's cloud rows (amendment 5), one case run once: a mission on
`stack.py`'s stack (no ROS; the brain's defaults but no local tier, and
`SIM_ARRIVAL_CONFIRM=cloud` so the simulator's arrival asks the cloud as a
real frame's would), the cloud a local $0 stand-in that refuses, answers or
hangs. Same `result.json` shape as `fmea.py` (`measures`, `verdict`).

| Case | The cloud | Policy | Plant (criterion 4) |
|---|---|---|---|
| F11a | refused from the start (a closed port) | tiered | `cloud_class` |
| F11b | answers (the 3.80.E stand-in), killed mid-mission, back after the end | tiered | `cloud_class` |
| F11c | refused from the start | vision | -- |
| F12a | hangs from the start (`hang_cloud.py`) | tiered | `cloud_deadline` |
| F12b | hangs from the start | vision | `cloud_deadline` |

    python evaluations/jetson-fmea-3135/cloud_rows.py F11a --out DIR [--plant cloud_class]
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import measure as m  # noqa: E402
from fmea import MemWatch, robot_health, status, wait_end  # noqa: E402
from stack import CLOSED_PORT, PY, Stack, StackError, http  # noqa: E402

STUB_PORT = int(os.environ.get("FMEA_STUB_PORT", 10358))    # this section's block, past the stack's
MISSION_WAIT_S = 1200 + 300        # mission_time_limit_s + slack
VISION_WAIT_S = 300.0              # B3.2: 3 failures x 20 s, and the frames between
SWITCH_AFTER_S = 60.0              # F11b: the cloud goes once it has answered, or by then
LATE_WAIT_S = 90.0                 # F11b: 3.53's late confirmation (probe every 15 s)
PARK_BAR_S = 30.0

ROWS = {
    "F11a": {"title": "Cloud unreachable from the start (tiered)", "cloud": "refused",
             "policy": "tiered", "plant": "cloud_class",
             "reading": "the mission ends arrived_unconfirmed (no tier here could confirm, so found "
                        "would be wrong, 3.47), never failed; 0 contacts"},
    "F11b": {"title": "Cloud unreachable mid-mission (tiered)", "cloud": "switch",
             "policy": "tiered", "plant": "cloud_class",
             "reading": "the cloud answers, then is gone; the mission ends arrived_unconfirmed or "
                        "found (found only by a cloud yes landed before it went), never failed; with "
                        "the cloud back, the late confirmation lands and no wheel setpoint is "
                        "non-zero from the mission's end to 10 s after it; 0 contacts"},
    "F11c": {"title": "Cloud unreachable from the start (vision)", "cloud": "refused",
             "policy": "vision", "plant": "",
             "reading": "the mission ends failed; true path over the mission <= 0.5 cm (without "
                        "moving); 0 contacts"},
    "F12a": {"title": "Cloud hangs (tiered)", "cloud": "hang", "policy": "tiered",
             "plant": "cloud_deadline",
             "reading": "never two paid calls open at once (open_paid <= 1); the mission ends "
                        "arrived_unconfirmed, its end <= 30 s after the last paid call was sent "
                        "(the arrival parks); 0 contacts"},
    "F12b": {"title": "Cloud hangs (vision)", "cloud": "hang", "policy": "vision",
             "plant": "cloud_deadline",
             "reading": "never two paid calls open at once; B3.2's budget ends the mission failed, "
                        "within 300 s of its start; 0 contacts"},
}


def judge(row: str, ms: dict) -> dict:
    r = []

    def bar(ok, why):
        if not ok:
            r.append(why)

    out = ms.get("outcome")
    if out is None and ms.get("timed_out"):
        # Waited out its whole budget: a guard that let it run on is a failed row, not a lost run.
        return {"pass": False, "valid": True,
                "reasons": [f"the mission had not ended {ms.get('mission_s')} s after its start"]}
    if out is None:
        return {"pass": False, "valid": False, "reasons": ["the mission never ended"]}
    bar(ms.get("contacts") == 0, f"{ms.get('contacts')} contact samples")
    bar(ms.get("gap_unknown", 1) == 0, f"{ms.get('gap_unknown')} truth samples with no gap (contact unjudged)")
    bar(ms.get("truth_hole_s", 9) <= 0.2, f"truth log has a {ms.get('truth_hole_s')} s hole")
    if row == "F11a":
        bar(out == "arrived_unconfirmed", f"the mission ended {out}, not arrived_unconfirmed")
    if row == "F11b":
        ok = out == "arrived_unconfirmed" or (out == "found" and ms.get("cloud_yes_before_switch"))
        bar(ok, f"the mission ended {out}" + (" (no cloud yes before the switch)" if out == "found" else ""))
        bar(ms.get("late_confirmation") is not None if out == "arrived_unconfirmed" else True,
            "no late confirmation within 90 s of the cloud's return")
        bar(ms.get("drive_after_end") == 0,
            f"{ms.get('drive_after_end')} driving samples after the mission's end (bar 0)")
    if row == "F11c":
        bar(out == "failed", f"the mission ended {out}, not failed")
        bar(ms.get("travel_mission_cm", 99) <= 0.5, f"moved {ms.get('travel_mission_cm')} cm (bar 0.5)")
    if row in ("F12a", "F12b"):
        bar(ms.get("max_open_paid", 9) <= 1, f"{ms.get('max_open_paid')} paid calls open at once (bar 1)")
    if row == "F12a":
        bar(out == "arrived_unconfirmed", f"the mission ended {out}, not arrived_unconfirmed")
        p = ms.get("park_s")
        bar(p is not None and p <= PARK_BAR_S, f"ended {p} s after the last paid call (bar 30)")
    if row == "F12b":
        bar(out == "failed", f"the mission ended {out}, not failed")
        d = ms.get("mission_s")
        bar(d is not None and d <= VISION_WAIT_S, f"ended {d} s after its start (bar 300)")
    return {"pass": not r, "valid": True, "reasons": r}


def calls(path: Path) -> list:
    if not path.exists():
        return []
    out = []
    for line in path.read_text().splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            continue
    return out


def run_row(row: str, out: Path, plant: str = "", house: str = "scaled_house") -> dict:
    spec = ROWS[row]
    out.mkdir(parents=True, exist_ok=True)
    rec = {"row": row, "title": spec["title"], "reading": spec["reading"], "plant": plant or None,
           "house": house, "host": os.uname().nodename, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    ms: dict = {"outcome": None}
    calls_log = out / "cloud_calls.jsonl"
    cloud = None

    def cloud_up(kind: str):
        argv = ([PY, str(HERE / "hang_cloud.py"), str(STUB_PORT), str(calls_log)] if kind == "hang" else
                [PY, str(ROOT / "evaluations/tiered-380/E/standin_cloud.py"), str(STUB_PORT), str(calls_log)])
        p = subprocess.Popen(argv, cwd=ROOT, stdout=open(out / "cloud.log", "ab"), stderr=subprocess.STDOUT,
                             start_new_session=True)
        end = time.monotonic() + 20
        import socket
        while time.monotonic() < end:
            with socket.socket() as s:
                if s.connect_ex(("127.0.0.1", STUB_PORT)) == 0:
                    return p
            time.sleep(0.2)
        raise StackError("the local cloud did not listen in 20 s")

    def cloud_down(p):
        if p is not None and p.poll() is None:
            t = time.monotonic()
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(timeout=5)
            return t
        return None

    mw, st = MemWatch(), None
    with mw:
        try:
            url = (f"http://127.0.0.1:{CLOSED_PORT}" if spec["cloud"] == "refused"
                   else f"http://127.0.0.1:{STUB_PORT}")
            st = Stack(out, house=house, plant=plant, vision_url=url,
                       extra_env={"LOCAL_VLM_URL": "", "SIM_ARRIVAL_CONFIRM": "cloud"})
            if spec["cloud"] != "refused":
                cloud = cloud_up("hang" if spec["cloud"] == "hang" else "standin")
            st.start()
            h0 = robot_health(st)
            code, body = st.brain("POST", "/mission/start",
                                  {"target_object": "red backpack", "policy": spec["policy"]}, timeout=60)
            if code != 200:
                raise StackError(f"mission start: {code} {str(body)[:300]}")
            ms["start_at"] = time.monotonic()
            if spec["cloud"] == "switch":
                end = ms["start_at"] + SWITCH_AFTER_S
                while time.monotonic() < end and not any(c.get("path") == "/navigate" for c in calls(calls_log)):
                    time.sleep(0.5)
                ms["answered_before_switch"] = sum(1 for c in calls(calls_log) if c.get("path") == "/navigate")
                s0 = status(st)
                ms["cloud_yes_before_switch"] = any(
                    (v or {}).get("tier") == "cloud" and (v or {}).get("confirmed") is True
                    for v in (s0.get("arrival_verdicts") or []))
                ms["switch_at"] = cloud_down(cloud)
                cloud = None
            s1 = wait_end(st, VISION_WAIT_S if spec["policy"] == "vision" else MISSION_WAIT_S)
            ms["end_at"] = time.monotonic()
            ms["outcome"] = s1.get("outcome") if s1.get("running") is False else None
            ms["timed_out"] = ms["outcome"] is None
            ms["mission_s"] = round(ms["end_at"] - ms["start_at"], 1)
            ms["status"] = {k: s1.get(k) for k in ("outcome", "end_state", "end_reason", "error", "step",
                                                   "arrival_verdicts", "cloud", "late_confirmation")}
            if spec["cloud"] == "switch" and ms["outcome"] == "arrived_unconfirmed":
                cloud = cloud_up("standin")            # the cloud is back: 3.53 asks once
                end = time.monotonic() + LATE_WAIT_S
                while time.monotonic() < end:
                    lc = status(st).get("late_confirmation")
                    if isinstance(lc, dict) and lc.get("state") not in (None, "waiting"):
                        ms["late_confirmation"] = lc
                        break
                    time.sleep(1.0)
                ms["late_at"] = time.monotonic()
            time.sleep(10.5)
            h1 = robot_health(st)
            if h0 and h1:
                ms["wheel_loop"] = {"late": h1["wheel_loop"]["late_ticks"] - h0["wheel_loop"]["late_ticks"],
                                    "moving": h1["wheel_loop"]["moving_ticks"] - h0["wheel_loop"]["moving_ticks"],
                                    "max_dt_s": h1["wheel_loop"]["max_dt_s"]}
        except Exception as e:             # noqa: BLE001 -- recorded, never a lost result
            rec["error"] = f"{type(e).__name__}: {e}"
        finally:
            cloud_down(cloud)
            if st is not None:
                st.stop()
    rows = m.read_truth(out / "truth.jsonl") if (out / "truth.jsonl").exists() else []
    cs = calls(calls_log)
    paid = [c for c in cs if "closed" not in c and (c.get("paid") or c.get("path") == "/navigate")]
    ms["paid_calls"] = len(paid)
    ms["max_open_paid"] = max((c.get("open_paid", 1) for c in paid), default=0)
    if spec["cloud"] == "hang" and ms.get("end_at") is not None:
        before = [c["t"] for c in paid if c["t"] <= ms["end_at"]]      # hang_cloud's stamps are monotonic
        ms["park_s"] = round(ms["end_at"] - max(before), 1) if before else None
    if ms.get("end_at") is not None and ms.get("start_at") is not None:
        ms["travel_mission_cm"] = m.travel_cm(rows, ms["start_at"], ms["end_at"])
        last = ms.get("late_at", ms["end_at"]) + 10.0
        ms["drive_after_end"] = m.nonzero_after(rows, ms["end_at"] + 1.0, last)
    sp = m.span(rows)
    ms["truth_hole_s"] = m.gaps_s(rows, sp[0], sp[1]) if sp else float("inf")
    ms["contacts"] = m.contacts(rows)
    ms["gap_unknown"] = sum(1 for r in rows if r.get("gap_cm") is None)
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
    print(json.dumps({"row": a.row, "plant": a.plant, "verdict": rec["verdict"], "error": rec.get("error"),
                      "measures": {k: v for k, v in rec["measures"].items() if k != "status"}}, default=str))
    return 0 if "error" not in rec else 3


if __name__ == "__main__":
    raise SystemExit(main())
