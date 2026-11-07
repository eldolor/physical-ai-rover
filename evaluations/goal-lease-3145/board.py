# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/goal-lease-3145/board.py

3.145's criterion 6: F6g and F7g on the Jetson exactly as 3.135.B ran them
(`evaluations/jetson-fmea-3135/run.py row`: `rows.py`, its scored runs, its
as-found check and restore), with two differences made in this process,
never in 3.135's files:
  * the commit under test lands in `~/bench-3145`, not 3.135's checkout;
  * the ROS container runs this section's own image tag (built on the board
    from `dev`), never `:latest`.

    python evaluations/goal-lease-3145/board.py F6g [--runs 5] [--sha HEAD]
"""

from __future__ import annotations

import argparse
import importlib.util
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
HARNESS = ROOT / "evaluations" / "jetson-fmea-3135"
sys.path.insert(0, str(HARNESS))

# 3.135's run.py imports 3.136's, which is also `run`: load 3.135's under a
# name of its own, so its `import run as perf_run` finds 3.136's.
_spec = importlib.util.spec_from_file_location("fmea_run_3135", HARNESS / "run.py")
fmea_run = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fmea_run)

BENCH_DIR = "bench-3145"
IMAGE = "vision-picar-ros:plan-3.145"
CONTAINER = "picar-ros-3145"
ROWS = ("F6g", "F7g")


def _wire() -> None:
    fmea_run.BENCH_DIR = BENCH_DIR
    shipped = fmea_run.row_cmd

    def row_cmd(row, runs, plant="", house="scaled_house"):
        return (f"export FMEA_ROS_IMAGE={IMAGE} FMEA_ROS_CONTAINER={CONTAINER}; "
                + shipped(row, runs, plant, house))

    fmea_run.row_cmd = row_cmd


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("row", choices=ROWS)
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--sha", default="HEAD")
    ap.add_argument("--out", default=None)
    ap.add_argument("--inner", action="store_true", help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    if a.inner:
        _wire()
        return fmea_run.row_inner(argparse.Namespace(row=a.row, runs=a.runs, plant="",
                                                     house="scaled_house", sha=a.sha, out=a.out))
    sha = subprocess.run(["git", "rev-parse", a.sha], cwd=ROOT, capture_output=True, text=True,
                         check=True).stdout.strip()
    out = a.out or str(HERE / "board" / f"{a.row}-{time.strftime('%Y%m%d-%H%M')}")
    argv_ = [fmea_run.PY, str(Path(__file__).resolve()), a.row, "--inner", "--sha", sha,
             "--runs", str(a.runs), "--out", out]
    # As 3.135.B: every number is the board's, so the shared desktop lock and
    # the board lock for this row only.
    return fmea_run.br.locked(argv_, f"3.145 c6 {a.row}", desktop_mode="shared")


if __name__ == "__main__":
    sys.exit(main())
