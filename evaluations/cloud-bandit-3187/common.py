# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.187 shared: the pinned labelled set (3.177.A's `set.json`, refused unless
it hashes to the pinned hash), 3.92's per-frame local answers, and the files
this section writes. Free; reads files only."""

import importlib.util
import json
import os
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parent.parent
RECORDINGS = Path(os.environ.get("PICAR_RECORDINGS", "/home/anshu/vision-picar-data/recordings"))
CAL = ROOT / "evaluations" / "tier-calibration-392"

# 3.177.A's module, loaded by path under its own name: both directories have
# a `common.py`, and a plain import would pick up this one.
_spec = importlib.util.spec_from_file_location(
    "visual_promise_3177_common", ROOT / "evaluations" / "visual-promise-3177" / "common.py")
vp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(vp)

SPLIT_SEED = 3651    # 3.65's split seed, carried
PILOT_SEED = 3187    # the pilot's frames (A, 2026-11-18)
PILOT_N = 20
# What /navigate ships (cloudformation/serverless.yaml NavigateModelId).
SHIPPED_MODEL = "us.anthropic.claude-opus-4-5-20251101-v1:0"
# brain/tiered.py's LOCAL_YES_AT and DEFAULT_LOCAL_NO_BELOW (3.80.D's rule).
LOCAL_YES_AT, LOCAL_NO_BELOW = 0.5, 0.1


def load_set() -> dict:
    return vp.load_set()


def obj(target: str) -> str:
    return vp.obj(target)


def walk_of(key: str) -> str:
    return key.split("/")[0]


def jsonl(path: Path) -> list:
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()]


def local_kind(row: dict) -> str:
    """A VE answer as `yes`, `no` or `unable` (the band counts as unable),
    by brain/tiered.py `_local_kind`'s thresholds."""
    s = row.get("score")
    if row.get("status") != "answered" or not isinstance(s, (int, float)):
        return "unable"
    return "yes" if s >= LOCAL_YES_AT else "no" if s <= LOCAL_NO_BELOW else "unable"
