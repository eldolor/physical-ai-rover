# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/tiered-wedge-3148/desktop.py

3.148's runs on the desktop (`docs/plans/ros-alignment/3.148-tiered-wedge-
without-cloud.md`). Rows F11a, F11b and F12a are 3.135.B's own
(`evaluations/jetson-fmea-3135/cloud_rows.py` `run_row`, unchanged). C0 is
this section's control: the same stack and mission (`stack.py`, ground
truth, no ROS, no local tier, `SIM_ARRIVAL_CONFIRM=cloud`) with a cloud that
answers, `truth_cloud.py` ($0, from the simulator's truth). Every run records
its outcome, where it ended (true pose), its length and its paid calls; the
summary is `summary.json` beside the runs.

Runs on this section's ports (`plan_section.py env`), under the desktop lock:

    python tools/desktop_lock.py exclusive --why "3.148 A1" -- \\
        python evaluations/tiered-wedge-3148/desktop.py F11a --runs 10 --out OUT
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

# 3.148's ports, set before stack.py reads them at import.
for k, v in {"PICAR_ROBOT_PORT": "10480", "PICAR_BRAIN_PORT": "10481", "PICAR_BODY_PORT": "10482",
             "PICAR_SENSOR_PORT": "10483", "PICAR_SENSOR2_PORT": "10484",
             "FMEA_STUB_PORT": "10486"}.items():
    os.environ.setdefault(k, v)

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
FMEA = ROOT / "evaluations/jetson-fmea-3135"
sys.path.insert(0, str(FMEA))

import cloud_rows  # noqa: E402
import measure as m  # noqa: E402
from fmea import wait_end  # noqa: E402
from stack import PY, Stack, StackError  # noqa: E402

ROWS = ("F11a", "F11b", "F12a", "C0")


def end_pose(out: Path) -> dict | None:
    """The true pose at the last truth sample, and the gap then."""
    path = out / "truth.jsonl"
    if not path.exists():
        return None
    rows = m.read_truth(path)
    if not rows:
        return None
    r = rows[-1]
    return {k: r.get(k) for k in ("x", "y", "th", "gap_cm")}


def run_control(out: Path, house: str = "scaled_house") -> dict:
    """C0: `run_row`'s mission with `truth_cloud.py` as the cloud."""
    out.mkdir(parents=True, exist_ok=True)
    port = cloud_rows.STUB_PORT
    calls_log = out / "cloud_calls.jsonl"
    rec: dict = {"row": "C0", "house": house, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    ms: dict = {"outcome": None}
    st, cloud = None, None
    try:
        st = Stack(out, house=house, vision_url=f"http://127.0.0.1:{port}",
                   extra_env={"LOCAL_VLM_URL": "", "SIM_ARRIVAL_CONFIRM": "cloud"})
        cloud = subprocess.Popen([PY, str(FMEA / "truth_cloud.py"), str(port), str(calls_log),
                                  st.urls["body"]], cwd=ROOT, stdout=open(out / "cloud.log", "ab"),
                                 stderr=subprocess.STDOUT, start_new_session=True)
        end = time.monotonic() + 20
        while time.monotonic() < end:
            with socket.socket() as s:
                if s.connect_ex(("127.0.0.1", port)) == 0:
                    break
            time.sleep(0.2)
        else:
            raise StackError("truth_cloud did not listen in 20 s")
        st.start()
        code, body = st.brain("POST", "/mission/start",
                              {"target_object": "red backpack", "policy": "tiered"}, timeout=60)
        if code != 200:
            raise StackError(f"mission start: {code} {str(body)[:300]}")
        ms["start_at"] = time.monotonic()
        s1 = wait_end(st, cloud_rows.MISSION_WAIT_S)
        ms["end_at"] = time.monotonic()
        ms["outcome"] = s1.get("outcome") if s1.get("running") is False else None
        ms["mission_s"] = round(ms["end_at"] - ms["start_at"], 1)
        ms["status"] = {k: s1.get(k) for k in ("outcome", "end_reason", "error", "step",
                                               "arrival_verdicts")}
        time.sleep(2.0)
    except Exception as e:  # noqa: BLE001 -- recorded, never a lost result
        rec["error"] = f"{type(e).__name__}: {e}"
    finally:
        if cloud is not None and cloud.poll() is None:
            os.killpg(cloud.pid, signal.SIGKILL)
            cloud.wait(timeout=5)
        if st is not None:
            st.stop()
    rows = m.read_truth(out / "truth.jsonl") if (out / "truth.jsonl").exists() else []
    ms["contacts"] = m.contacts(rows)
    if ms.get("end_at") is not None:
        ms["travel_mission_cm"] = m.travel_cm(rows, ms["start_at"], ms["end_at"])
    cs = cloud_rows.calls(calls_log)
    ms["paid_calls"] = sum(1 for c in cs if c.get("path") == "/navigate")
    rec["measures"] = ms
    (out / "result.json").write_text(json.dumps(rec, indent=1, default=str))
    return rec


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("row", choices=ROWS)
    ap.add_argument("--runs", type=int, default=10)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    out = Path(a.out)
    if out.exists() and any(out.iterdir()):
        raise SystemExit(f"{out} is not empty: give each batch its own directory")
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for i in range(a.runs):
        d = out / f"{a.row}-{i:02d}"
        rec = run_control(d) if a.row == "C0" else cloud_rows.run_row(a.row, d)
        ms = rec.get("measures") or {}
        row = {"run": i, "outcome": ms.get("outcome"), "mission_s": ms.get("mission_s"),
               "travel_cm": ms.get("travel_mission_cm"), "contacts": ms.get("contacts"),
               "paid_calls": ms.get("paid_calls"), "max_open_paid": ms.get("max_open_paid"),
               "park_s": ms.get("park_s"), "end_reason": ((ms.get("status") or {}).get("end_reason")
                                                          or "")[:160],
               "end_pose": end_pose(d), "error": rec.get("error"),
               "verdict": (rec.get("verdict") or {}).get("met")}
        rows.append(row)
        print(json.dumps(row), flush=True)
        (out / "summary.json").write_text(json.dumps({"row": a.row, "runs": rows}, indent=1))
        time.sleep(5)          # the stack's ports free before the next run
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
