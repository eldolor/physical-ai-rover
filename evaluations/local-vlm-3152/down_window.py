# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/local-vlm-3152/down_window.py

3.152 criteria 1-2, measured: one whole tiered mission with VE down from its
first question (tests/test_local_vlm_down.py's StubVE and mission), against
the same mission with no local tier, and the real client's refused-connection
time (50 asks to a closed port). Run on this branch and on dev before 3.152;
writes JSON to the path given.

    python evaluations/local-vlm-3152/down_window.py OUT.json
"""

import json
import logging
import os
import statistics
import subprocess
import sys
import time

sys.path.insert(0, os.getcwd())

import pytest  # noqa: E402

import tests.test_local_vlm_down as t  # noqa: E402
from brain.local_vlm import LocalVLMAbsent, LocalVLMClient  # noqa: E402


def main(out):
    logging.disable(logging.WARNING)
    mp = pytest.MonkeyPatch()
    ve = t.StubVE(t.TARGET, "down")
    down, _ = t._mission(mp, ve)
    none, _ = t._mission(mp, None)
    keep = ("outcome", "step", "ticks", "arrivals", "cloud_calls", "triggers")
    client = LocalVLMClient("http://127.0.0.1:9", t.TARGET, deadline_s=3.0)
    frame = {"image_base64": t._tiny_jpeg_b64()}
    ms = []
    for _ in range(50):
        t0 = time.perf_counter()
        try:
            client.ask(frame, box=True)
        except LocalVLMAbsent:
            pass
        ms.append((time.perf_counter() - t0) * 1000)
    client.close()
    rev = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True,
                         text=True).stdout.strip()
    rec = {"rev": rev, "house": t.HOUSE, "target": t.TARGET, "max_steps": t.MAX_STEPS,
           "ve_questions_sent": ve.asked, "ve_probes": ve.probes,
           "down": {k: down[k] for k in keep}, "no_local_tier": {k: none[k] for k in keep},
           "decisions_equal": {k: down[k] for k in keep} == {k: none[k] for k in keep},
           "down_local_frame": down["local_frame"], "down_local_frame_ms": down["local_frame_ms"],
           "down_local_vlm": down.get("local_vlm"),
           "refused_ask_ms": {"n": len(ms), "p50": round(statistics.median(ms), 2),
                              "p90": round(sorted(ms)[int(0.9 * len(ms)) - 1], 2),
                              "max": round(max(ms), 2)}}
    with open(out, "w") as f:
        json.dump(rec, f, indent=1, default=str)
        f.write("\n")
    print(json.dumps({k: rec[k] for k in ("rev", "ve_questions_sent", "decisions_equal",
                                          "refused_ask_ms")}))


if __name__ == "__main__":
    main(sys.argv[1])
