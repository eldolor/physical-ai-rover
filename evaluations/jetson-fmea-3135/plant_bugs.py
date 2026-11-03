# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/plant_bugs.py

3.135 criterion 4, "the harness tells the truth", on the DESKTOP's local
stack: each row built in 3.135.A run once against a build with the guard it
tests switched off (a planted bug) must FAIL its row. Each row is also run
once as shipped, beside it, so the reader sees what the plant changed (the
base run is reported, not scored: criterion 3 is the board's).

| Row | Planted bug | Where |
|---|---|---|
| F6, F7, F13 | the watchdog disabled | robot_plant.py `watchdog` |
| F8 | the fake board ignores its heartbeat | truth_body.py `heartbeat` |
| F9 | the lidar freshness check removed | robot_plant.py `lidar_fresh` |
| F10 | the wheel-feedback freshness check removed | robot_plant.py `feedback_fresh` |

A row whose planted run PASSES is not scored until the harness is fixed
(criterion 4); this file says so, it does not hide it.

    python evaluations/jetson-fmea-3135/plant_bugs.py [--out DIR] [--rows F6,F9]

Run it under the exclusive desktop lock (its bars are timed): `run.py plant` does.
"""

from __future__ import annotations

import argparse
import gzip
import json
import shutil
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent

import fmea  # noqa: E402
import rows as rowsmod  # noqa: E402

ORDER = ("F13", "F9", "F10", "F8", "F6", "F7")
# 3.135.B (amendment 5): the goal rows and the cloud cases with a plant.
ORDER_B = ("F6g", "F7g", "F4", "F5", "F11a", "F12a", "F12b", "F11b")


def summarise(row: str, base: dict, planted: dict) -> dict:
    keys = ("wheels_zero_s", "travel_cm", "travel_stale_cm", "drive_after_cont", "contacts",
            "no_feedback_refusals", "outcome", "recovery_s", "driver_at_fault", "stop_answer_s",
            "wheels_zero_from_stop_s", "drive_after_unpause", "recovery_outcome", "max_open_paid",
            "park_s", "mission_s", "late_confirmation", "drive_after_end", "travel_mission_cm")

    def pick(r):
        ms = r.get("measures") or {}
        return {k: ms.get(k) for k in keys if k in ms}

    def clean(r):                          # a run that errored or was invalid proves nothing
        return r.get("error") is None and r["verdict"].get("valid") is True

    caught = (not planted["verdict"]["pass"]) and clean(planted)
    # Caught is criterion 4 as written. It tells the guard apart only if the
    # shipped build passes the same row: a row whose base fails too (its guard
    # does not hold today) is caught but proves nothing about the harness.
    discriminates = caught and base["verdict"]["pass"] and clean(base)
    spec = rowsmod.runner(row).ROWS[row]
    return {"row": row, "plant": spec["plant"], "reading": spec["reading"],
            "base": {"pass": base["verdict"]["pass"], "reasons": base["verdict"]["reasons"],
                     "error": base.get("error"), "recovery_s": base.get("recovery_s"), **pick(base)},
            "planted": {"pass": planted["verdict"]["pass"], "reasons": planted["verdict"]["reasons"],
                        "error": planted.get("error"), **pick(planted)},
            "caught": caught, "discriminates": discriminates}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(HERE / "desktop"))
    ap.add_argument("--rows", default=",".join(ORDER))
    ap.add_argument("--record", default="planted.json", help="the record's name in --out")
    a = ap.parse_args(argv)
    out = Path(a.out)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    results = []
    for row in a.rows.split(","):
        runs = {}
        mod = rowsmod.runner(row)
        for kind, plant in (("base", ""), ("planted", mod.ROWS[row]["plant"])):
            runs[kind] = mod.run_row(row, out / "runs" / f"{stamp}-{row}-{kind}", plant=plant)
            print(json.dumps({"row": row, "kind": kind, "verdict": runs[kind]["verdict"],
                              "error": runs[kind].get("error")}), flush=True)
        results.append(summarise(row, runs["base"], runs["planted"]))
        for kind in runs:                  # scored above; kept small for the record
            t = out / "runs" / f"{stamp}-{row}-{kind}" / "truth.jsonl"
            if t.exists():
                with open(t, "rb") as src, gzip.open(f"{t}.gz", "wb") as dst:
                    shutil.copyfileobj(src, dst)
                t.unlink()
    doc = {"stamp": stamp, "criterion": 4, "rows": results,
           "met": all(r["caught"] for r in results),
           "not_scored_until_fixed": [r["row"] for r in results if not r["caught"]],
           "base_fails_too": [r["row"] for r in results if r["caught"] and not r["discriminates"]]}
    (out / a.record).write_text(json.dumps(doc, indent=1))
    print(json.dumps({"met": doc["met"], "not_caught": doc["not_scored_until_fixed"]}))
    return 0 if doc["met"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
