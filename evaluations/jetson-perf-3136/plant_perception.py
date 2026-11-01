"""
evaluations/jetson-perf-3136/plant_perception.py

Criterion 3 for P1/P2/P10: `tools/jetson/perception_loop.py` itself, run on
the desktop (which has no perception models) with `brain.perceive.pipeline_for`
replaced by a stand-in whose `perceive()` takes STUB_MS, plus 100 ms when
PLANT=1. The loop's own timing, log format and 4 Hz pacing are the shipped
ones; the scorer (`scorers.perception_ms`) must read the 100 ms back.

    PLANT=0|1 python evaluations/jetson-perf-3136/plant_perception.py <seconds>

Run from a directory holding `recordings/<walk>/<file>` for the frames in
`tools/jetson/bench_frames.json` (`plant.py` writes stand-in JPEGs there).
"""

import os
import runpy
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import brain.perceive  # noqa: E402

STUB_S = float(os.environ.get("STUB_MS", "30")) / 1000
PLANT_S = float(os.environ.get("PLANT_DELAY_S", "0.1")) if os.environ.get("PLANT") == "1" else 0.0


class StubPipeline:
    def perceive(self, frame):
        time.sleep(STUB_S + PLANT_S)
        return SimpleNamespace(status="absent")

    def close(self):
        pass


brain.perceive.pipeline_for = lambda target, *a, **k: StubPipeline()
sys.argv = [str(ROOT / "tools/jetson/perception_loop.py"), *sys.argv[1:]]
runpy.run_path(sys.argv[0], run_name="__main__")
