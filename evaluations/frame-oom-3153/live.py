# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/frame-oom-3153/live.py -- 3.153 criterion 3, the desktop's
split-body stack (3.135's `stack.py`: sim mode, the fake motor board and
lidar, the cloud a closed port).

    .venv/bin/python tools/desktop_lock.py exclusive --why "3.153 live" -- \\
        .venv/bin/python evaluations/frame-oom-3153/live.py [--out DIR]

Two tiered find missions (red backpack, scaled_house) through the brain's
HTTP API: a reference, then the same with both sensor programs stopped
(`SIGSTOP`) for 4 s once the mission has taken 3 steps -- past the sim's
3 s frame read (`sim/body_client.py` FRAME_TIMEOUT_S), the cause 3.152.B
found -- then resumed (`SIGCONT`). Bars (criterion 3): the perturbed mission
does not end `failed` on it, its status shows `camera_failures` rising and
back to 0, and it ends with the reference's outcome.
"""

import argparse
import json
import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "evaluations" / "jetson-fmea-3135"))
sys.path.insert(0, str(ROOT))

from fmea import status, wait_end  # noqa: E402
from stack import Stack, StackError  # noqa: E402

STOP_S = 4.0
START_AFTER_STEPS = 3
END_WAIT_S = 900


def one(out: Path, perturb: bool) -> dict:
    rec = {"perturb": perturb, "camera_failures_seen": [], "error": None}
    with Stack(out, house="scaled_house") as st:
        code, body = st.brain("POST", "/mission/start",
                              {"target_object": "red backpack", "policy": "tiered"}, timeout=600)
        if code != 200:
            raise StackError(f"mission start: {code} {str(body)[:300]}")
        t0 = time.monotonic()
        if perturb:
            end = t0 + 300
            while time.monotonic() < end:
                s = status(st)
                if (s.get("step") or 0) >= START_AFTER_STEPS or not s.get("running", True):
                    break
                time.sleep(0.2)
            rec["stopped_at_step"] = s.get("step")
            rec["stop_at_s"] = round(st.signal("sensors", signal.SIGSTOP) - t0, 2)
            st.signal("sensors2", signal.SIGSTOP)
            stop_end = time.monotonic() + STOP_S
            while time.monotonic() < stop_end:
                rec["camera_failures_seen"].append(status(st).get("camera_failures"))
                time.sleep(0.25)
            st.signal("sensors", signal.SIGCONT)
            rec["cont_at_s"] = round(st.signal("sensors2", signal.SIGCONT) - t0, 2)
            end = time.monotonic() + 30
            while time.monotonic() < end:
                cf = status(st).get("camera_failures")
                rec["camera_failures_seen"].append(cf)
                if cf == 0 and max(x or 0 for x in rec["camera_failures_seen"]) > 0:
                    break
                time.sleep(0.25)
        s = wait_end(st, END_WAIT_S)
        rec.update(outcome=s.get("outcome"), end_reason=s.get("end_reason") or s.get("error"),
                   step=s.get("step"), camera_failures_end=s.get("camera_failures"),
                   wall_s=round(time.monotonic() - t0, 1),
                   log_tail=[l for l in s.get("log_tail") or [] if "camera" in l][-6:])
        seen = [x for x in rec["camera_failures_seen"] if x is not None]
        rec["camera_failures_max"] = max(seen) if seen else None
    return rec


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(Path(__file__).with_name("live")))
    args = ap.parse_args(argv)
    out = Path(args.out)
    ref = one(out / "reference", False)
    print(json.dumps(ref), flush=True)
    got = one(out / "perturbed", True)
    print(json.dumps(got), flush=True)
    verdict = {"not_failed": got["outcome"] != "failed",
               "rose_and_reset": bool(got["camera_failures_max"]) and 0 in
               got["camera_failures_seen"][got["camera_failures_seen"].index(got["camera_failures_max"]):],
               "same_outcome": got["outcome"] == ref["outcome"]}
    verdict["met"] = all(verdict.values())
    (out / "result.json").write_text(json.dumps({"verdict": verdict, "reference": ref,
                                                  "perturbed": got}, indent=1) + "\n")
    print(json.dumps(verdict))
    return 0 if verdict["met"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
