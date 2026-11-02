# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/run.py

3.135's entry point on the desktop.

    python evaluations/jetson-fmea-3135/run.py plant [--wait]          # criterion 4, desktop, exclusive lock
    python evaluations/jetson-fmea-3135/run.py smoke [--sha REV] [--wait]  # 3.135.A's board smoke run (no sudo)
    python evaluations/jetson-fmea-3135/run.py found                   # the board's as-found state, read only

Board runs reuse 3.136.A's runner, not a copy (`evaluations/jetson-perf-3136/`):
`boardrun.locked()` (the desktop lock, then `board_lock.py run --why`),
`run.board_run()` (as-found snapshot, the commit pushed to the board,
telemetry, the command, fetch, restore, the as-found check: criterion 5)
with `bench_dir="bench-3135"`, so the commit under test lands in
`~/bench-3135`, never in 3.136's checkout.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PERF = ROOT / "evaluations" / "jetson-perf-3136"
sys.path.insert(0, str(PERF))

import boardrun as br  # noqa: E402  (3.136.A's)
import run as perf_run  # noqa: E402  (3.136.A's board_run)
import validity  # noqa: E402  (3.136.A's telemetry_counts)

PY = sys.executable
BENCH_DIR = "bench-3135"
# The smoke: one fault row (F13, the D-pad link lost) on a stack of its own,
# on this section's ports, beside the board's untouched twin stack. It
# needs no sudo and changes no unit; it is not scored (criterion 3 is B's).
SMOKE_ROW = "F13"
SMOKE_CMD = (f"nvpmodel -q 2>&1 | grep -m1 'NV Power Mode' > $OUT/nvpmodel.txt; "
             f"date +%s.%N > $OUT/run_start; "
             f".venv/bin/python evaluations/jetson-fmea-3135/fmea.py {SMOKE_ROW} --out $OUT/fmea "
             f"> $OUT/fmea.log 2>&1; echo $? > $OUT/fmea_exit; date +%s.%N > $OUT/run_end")


def smoke_summary(rec: dict, out: Path) -> dict:
    b = out / "board"
    tele = br.read_sysfs(b / "telemetry/sysfs.jsonl") if (b / "telemetry/sysfs.jsonl").exists() else []

    def num(name):
        p = b / name
        return float(p.read_text()) if p.exists() and p.read_text().strip() else None

    tc = validity.telemetry_counts(tele, num("run_start"), num("run_end"))
    res = json.loads((b / "fmea/result.json").read_text()) if (b / "fmea/result.json").exists() else {}
    return {"sha": rec.get("sha"), "cmd_exit": rec.get("cmd_exit"), "cmd_s": rec.get("cmd_s"),
            "fmea_exit": (b / "fmea_exit").read_text().strip() if (b / "fmea_exit").exists() else None,
            "row": res.get("row"), "host": res.get("host"), "verdict": res.get("verdict"),
            "error": res.get("error"), "recovery_s": res.get("recovery_s"),
            "measures": {k: v for k, v in (res.get("measures") or {}).items() if k != "status"},
            "power_mode": (b / "nvpmodel.txt").read_text().strip() if (b / "nvpmodel.txt").exists() else None,
            "telemetry": tc, "desktop_load_samples": len(rec.get("desktop_load") or []),
            "restore_actions": rec.get("restore_actions"), "restore_error": rec.get("restore_error"),
            "as_found_differences": rec.get("as_found_differences"),
            "left_as_found": rec.get("left_as_found")}


def smoke_inner(a) -> int:
    out = Path(a.out)
    rec = perf_run.board_run("fmea-smoke", SMOKE_CMD, out, sha=a.sha, allow_sudo=False,
                             stop_twin=False, bench_dir=BENCH_DIR)
    s = smoke_summary(rec, out)
    (out / "summary.json").write_text(json.dumps(s, indent=1))
    print(json.dumps(s, indent=1))
    ok = (rec.get("cmd_exit") == 0 and s["fmea_exit"] == "0" and s["left_as_found"]
          and s["host"] and s["host"] != __import__("os").uname().nodename)
    print("smoke:", "OK" if ok else "NOT OK")
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("smoke")
    s.add_argument("--sha", default="HEAD")
    s.add_argument("--out", default=str(HERE / "board" / "smoke"))
    s.add_argument("--wait", action="store_true", help="wait for the desktop lock")
    i = sub.add_parser("_smoke")
    i.add_argument("--sha", required=True)
    i.add_argument("--out", required=True)
    p = sub.add_parser("plant")
    p.add_argument("--wait", action="store_true", help="wait for the desktop lock")
    p.add_argument("--out", default=str(HERE / "desktop"))
    sub.add_parser("found")
    a = ap.parse_args(argv)
    if a.cmd == "smoke":
        sha = subprocess.run(["git", "rev-parse", a.sha], cwd=ROOT, capture_output=True, text=True,
                             check=True).stdout.strip()
        # Not timed (nothing is scored): the shared desktop lock.
        return br.locked([PY, str(Path(__file__).resolve()), "_smoke", "--sha", sha, "--out", a.out],
                         "3.135.A board smoke", desktop_mode="shared", desktop_wait=a.wait)
    if a.cmd == "_smoke":
        return smoke_inner(a)
    if a.cmd == "plant":
        # Timed stops (1.1 s bars): the exclusive desktop lock, as 3.136's plant.
        return subprocess.call([PY, str(ROOT / "tools/desktop_lock.py"), "exclusive",
                                *([] if a.wait else ["--no-wait"]), "--why", "3.135.A planted bugs",
                                "--", PY, str(HERE / "plant_bugs.py"), "--out", a.out], cwd=ROOT)
    if a.cmd == "found":
        print(json.dumps(br.snapshot(), indent=1))
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
