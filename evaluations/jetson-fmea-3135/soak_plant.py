# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/soak_plant.py

3.135.C's planted check (criterion 4's spirit, amendment 3): the real
sampler (`tools/jetson/memory_account.py sample`, as the soak runs it, here
without root) over two stand-in processes on the desktop, through four short
"hours" of back-to-back stand-in missions, scored by `soak_score.bar_b` with
the hour scaled down. The planted leak (`stand-in leak`, +25 MB every
mission) must fail (b)'s per-role bar; the steady stand-in must not.

    python evaluations/jetson-fmea-3135/soak_plant.py --out DIR [--hour-s 40]

Writes DIR/planted.json; exit 0 only if the leak was caught and the steady
stand-in passed (caught AND discriminates).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import soak_score as ss  # noqa: E402

PY = sys.executable


def run(out: Path, hour_s: float = 40.0, mission_s: float = 10.0) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="soak-plant-") as work:   # only planted.json is kept
        res = _run(Path(work), hour_s, mission_s)
    (out / "planted.json").write_text(json.dumps(res, indent=1))
    return res


def _run(out: Path, hour_s: float, mission_s: float) -> dict:
    total = 4 * hour_s
    phase = out / "phase"
    phase.write_text("setup\n")
    procs = []
    try:
        procs.append(subprocess.Popen(
            [PY, "-m", "tools.jetson.memory_account", "sample", str(out / "account.jsonl"),
             "--interval", "1", "--seconds", str(total + 30), "--phase-file", str(phase)],
            cwd=ROOT))
        for name, leak in (("leak", 25), ("steady", 0)):
            procs.append(subprocess.Popen(
                [PY, str(HERE / "soak_standin.py"), "--name", name, "--base-mb", "50",
                 "--leak-mb", str(leak), "--every-s", str(mission_s), "--seconds", str(total + 20)]))
        time.sleep(2.0)
        rs = time.time()
        (out / "run_start").write_text(f"{rs}\n")
        k = 0
        with open(out / "missions.jsonl", "w") as f:
            while time.time() - rs < total:
                phase.write_text(f"m{k:03d}\n")
                t0 = time.time()
                time.sleep(mission_s - 0.5)
                t1 = time.time()
                f.write(json.dumps({"k": k, "house": "stand-in", "target": "none", "setup_t": t0,
                                    "ready_t": t0, "t0": t0, "t1": t1, "outcome": "found",
                                    "wheel_loop": {"late_ticks": 0, "moving_ticks": 1},
                                    "contacts": 0, "truth_hole_s": 0.0,
                                    "truth_hole_all_s": 0.0, "gap_unknown": 0}) + "\n")
                time.sleep(0.5)
                k += 1
        (out / "run_end").write_text(f"{time.time()}\n")
        phase.write_text("end\n")
        time.sleep(2.5)
    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    p.kill()
    samples = [ss.compact(r) for r in ss.jsonl(out / "account.jsonl")]
    missions = ss.jsonl(out / "missions.jsonl")
    b = ss.bar_b(ss.mission_mins(samples, missions), rs, hour_s)
    leak, steady = b["growth"].get("stand-in leak"), b["growth"].get("stand-in steady")
    caught = leak is not None and "stand-in leak" in b["over"] and not b["met"]
    passes = steady is not None and "stand-in steady" not in b["over"]
    res = {"hour_s": hour_s, "mission_s": mission_s, "missions": len(missions),
           "samples": len(samples), "leak": leak, "steady": steady,
           "caught": caught, "steady_passes": passes, "discriminates": caught and passes,
           "b_reasons": b["reasons"],
           "when": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(HERE / "desktop" / "soak-plant"))
    ap.add_argument("--hour-s", type=float, default=40.0)
    a = ap.parse_args(argv)
    res = run(Path(a.out), a.hour_s)
    print(json.dumps(res, indent=1))
    return 0 if res["discriminates"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
