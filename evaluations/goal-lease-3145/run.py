# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/goal-lease-3145/run.py

3.145's desktop live checks on 3.135's FMEA harness (`evaluations/
jetson-fmea-3135/`, used as it is: its rows, bars and truth recorder). That
harness is 3.135's; the only additions -- the `no_lease` plant and F6g/F7g
pointing at it -- are made in this process, never in its files.

    python evaluations/goal-lease-3145/run.py rows [--runs 3]       # criterion 2
    python evaluations/goal-lease-3145/run.py plant                 # criterion 3
    python evaluations/goal-lease-3145/run.py explore [--runs 3]    # criterion 4

Every one is timed: run it under the exclusive desktop lock
(`tools/desktop_lock.py exclusive --why ... -- ...`).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
HARNESS = HERE.parent / "jetson-fmea-3135"
sys.path.insert(0, str(HARNESS))
os.environ.setdefault("FMEA_ROS_CONTAINER", "picar-ros-3145")

import fmea  # noqa: E402
import plant_bugs  # noqa: E402
import ros as rosmod  # noqa: E402
import stack  # noqa: E402

PLANT = "no_lease"
ROWS = ("F6g", "F7g")


def _wire() -> None:
    """The plant 3.135's harness does not know, added in this process."""
    stack.ROBOT_PLANTS.add(PLANT)
    shipped = stack.Stack.start_robot

    def start_robot(self) -> None:
        if PLANT not in self.plants:
            return shipped(self)
        if self.plants & stack.ROBOT_PLANTS - {PLANT}:
            raise stack.StackError(f"{PLANT} does not combine with another robot plant")
        self._spawn("robot", [stack.PY, str(HERE / "robot_plant.py"), str(stack.PORTS["robot"])],
                    dict(self.env))
        self._wait_health("robot", self.urls["robot"])

    stack.Stack.start_robot = start_robot
    for row in ROWS:                         # the lease is now these rows' guard
        fmea.ROWS[row]["plant"] = PLANT


def rows(out: Path, runs: int) -> dict:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    res = []
    for row in ROWS:
        for i in range(runs):
            rec = fmea.run_row(row, out / "runs" / f"{stamp}-{row}-{i + 1}")
            ms = rec.get("measures") or {}
            res.append({"row": row, "run": i + 1, "error": rec.get("error"),
                        "verdict": rec["verdict"], "recovery_s": rec.get("recovery_s"),
                        **{k: ms.get(k) for k in ("wheels_zero_s", "travel_cm", "drive_after_cont",
                                                  "contacts", "outcome", "end_reason",
                                                  "driver_at_fault", "last_refusal")}})
            print(json.dumps(res[-1], default=str), flush=True)
    valid = [r for r in res if r["error"] is None and r["verdict"].get("valid")]
    doc = {"stamp": stamp, "criterion": 2, "runs": res,
           "met": {row: bool([r for r in valid if r["row"] == row])
                   and all(r["verdict"]["pass"] for r in valid if r["row"] == row)
                   and len([r for r in valid if r["row"] == row]) >= runs
                   for row in ROWS}}
    (out / f"rows-{stamp}.json").write_text(json.dumps(doc, indent=1, default=str))
    return doc


def explore(out: Path, runs: int) -> dict:
    """Criterion 4: plain explore missions end found, 0 contacts, 0 lapses."""
    stamp = time.strftime("%Y%m%d-%H%M%S")
    res = []
    for i in range(runs):
        d = out / "runs" / f"{stamp}-explore-{i + 1}"
        d.mkdir(parents=True, exist_ok=True)
        rec = {"run": i + 1}
        st = ros = None
        try:
            st = stack.Stack(d, house="scaled_house", extra_env=rosmod.ROS_ENV)
            st.start()
            ros = rosmod.Ros(st, d)
            ros.up()
            t0 = time.monotonic()
            fmea.start_mission(st, "explore")
            s = fmea.wait_end(st, fmea.FOUND_WAIT_S)
            rec.update(outcome=s.get("outcome"), end_reason=s.get("end_reason"),
                       mission_s=round(time.monotonic() - t0, 1),
                       goal_lease=fmea.robot_health(st).get("goal_lease"))
        except Exception as e:  # noqa: BLE001 -- recorded, never a lost result
            rec["error"] = f"{type(e).__name__}: {e}"
        finally:
            try:
                if ros is not None:
                    ros.down()
            finally:
                if st is not None:
                    st.stop()
        rows_ = fmea.m.read_truth(d / "truth.jsonl") if (d / "truth.jsonl").exists() else []
        rec["contacts"] = fmea.m.contacts(rows_)
        rec["met"] = (rec.get("error") is None and rec.get("outcome") == "found"
                      and rec["contacts"] == 0 and (rec.get("goal_lease") or {}).get("lapses") == 0)
        res.append(rec)
        print(json.dumps(rec, default=str), flush=True)
    doc = {"stamp": stamp, "criterion": 4, "runs": res, "met": all(r["met"] for r in res)}
    (out / f"explore-{stamp}.json").write_text(json.dumps(doc, indent=1, default=str))
    return doc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("rows", "plant", "explore"))
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--out", default=str(HERE / "desktop"))
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    _wire()
    if a.what == "rows":
        doc = rows(out, a.runs)
        return 0 if all(doc["met"].values()) else 1
    if a.what == "plant":
        return plant_bugs.main(["--rows", ",".join(ROWS), "--out", str(out)])
    return 0 if explore(out, a.runs)["met"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
