# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/local-vlm-3152/board.py

3.152 criterion 10's arms from the desktop, one arm per board hold, each through
3.136's `board_run` (the commit pushed to ~/bench-3152, the lease beside the
run, the twin's units stopped and the board put back as found):

    python evaluations/local-vlm-3152/board.py d30    # RestartSec=30
    python evaluations/local-vlm-3152/board.py d60    # RestartSec=60
    python evaluations/local-vlm-3152/board.py d120   # RestartSec=120

The board lock waits in its queue behind another session's hold, so two
sections taking the board in turn alternate; the desktop lock (shared: no
number here is timed on the desktop) is taken inside it without waiting.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PY = sys.executable
sys.path.insert(0, str(ROOT / "evaluations/jetson-perf-3136"))

import run as perf_run  # noqa: E402  (3.136's board_run)

BENCH_DIR = "bench-3152"
# A hold's ceiling per arm: setup (~10 min), the 15-min window, D + 60 s, the
# mission's stop and the put-back, with room.
MAX_S = {"d30": 3600, "d60": 3600, "d120": 3600}
WAIT_S = 12 * 3600                 # how long to queue for the board
BUSY = 75                          # either lock's busy exit
RETRY_S = 120


def inner(a) -> int:
    out = Path(a.out)
    rec = perf_run.board_run(f"3152-{a.arm}", f"bash evaluations/local-vlm-3152/board.sh $OUT {a.arm}",
                             out, sha=a.sha, allow_sudo=True, stop_twin=True, bench_dir=BENCH_DIR)
    keep = ("sha", "cmd_exit", "cmd_s", "restore_actions", "restore_error",
            "as_found_differences", "left_as_found", "lease", "left_to_lease", "fetch_error")
    (out / "summary.json").write_text(json.dumps({k: rec.get(k) for k in keep}, indent=1) + "\n")
    print(f"cmd_exit {rec.get('cmd_exit')}, left as found {rec.get('left_as_found')}")
    return 0 if rec.get("cmd_exit") == 0 and rec.get("left_as_found") else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("arm", choices=("d30", "d60", "d120", "_d30", "_d60", "_d120"))
    ap.add_argument("--sha", default="HEAD")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    if a.arm.startswith("_"):
        a.arm = a.arm[1:]
        if not a.out:
            raise SystemExit("the inner arm needs --out")
        return inner(a)
    sha = subprocess.run(["git", "rev-parse", a.sha], cwd=ROOT, capture_output=True, text=True,
                         check=True).stdout.strip()
    out = str(Path(a.out).resolve()) if a.out else str(HERE / "board" / f"{a.arm}-{time.strftime('%Y%m%d-%H%M')}")
    why = f"3.152 {a.arm}"
    argv_ = [PY, str(Path(__file__).resolve()), f"_{a.arm}", "--sha", sha, "--out", out]
    # The board lock outside, queued; the desktop lock inside and never waited
    # on, so neither is held while waiting for the other (review): a busy
    # desktop (an exclusive timed run) gives the board back, and the queue is
    # joined again a little later.
    end = time.monotonic() + WAIT_S
    while True:
        rc = subprocess.call([PY, str(ROOT / "tools/jetson/board_lock.py"), "run", "--why", why,
                              "--max", str(MAX_S[a.arm]), "--wait", str(WAIT_S), "--",
                              PY, str(ROOT / "tools/desktop_lock.py"), "shared", "--no-wait",
                              "--why", why, "--", *argv_], cwd=ROOT)
        if rc != BUSY or time.monotonic() > end:
            return rc
        print(f"{why}: a lock was busy; queueing again in {RETRY_S} s", file=sys.stderr, flush=True)
        time.sleep(RETRY_S)


if __name__ == "__main__":
    raise SystemExit(main())
