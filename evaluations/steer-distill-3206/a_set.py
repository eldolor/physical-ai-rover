# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/steer-distill-3206/a_set.py

3.206.A (amendment 6): the larger steering test set, and the local order
scored on it. No model is asked anything: the tiers' answers are 3.92's
committed rows.

    .venv/bin/python evaluations/steer-distill-3206/a_set.py --draw     # a-draw.json (seed 3206)
    .venv/bin/python evaluations/steer-distill-3206/a_set.py --score    # a-scores.json

The draw: 140 frames, uniform over the frames people labelled in view that
are not in S1, at most 12 a walk. Labels are added by viewing, to
`a-labels.json`, in `labels-s1.json`'s shape.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import random
from collections import Counter
from pathlib import Path
from typing import Optional

HERE = Path(__file__).parent
ROOT = HERE.parents[1]


def _by_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


s3214 = _by_path("score_3214_a", ROOT / "evaluations/teacher-probe-3214/score.py")
mf = s3214.mf
third = s3214.third

SEED, N_NEW, PER_WALK = 3206, 140, 12
BORDER = 0.05                        # of the frame's width, either side of a line
STOP_AT = 0.90                       # criterion 1: unflagged right at or above this stops the section
DRAW = HERE / "a-draw.json"
LABELS = HERE / "a-labels.json"
S1_LABELS = ROOT / "evaluations/tiered-380/I/labels-s1.json"
T392 = ROOT / "evaluations/tier-calibration-392"
NEIGHBOURS = {"LEFT": {"LEFT", "FORWARD"}, "RIGHT": {"RIGHT", "FORWARD"},
              "FORWARD": {"LEFT", "FORWARD", "RIGHT"}}


def draw() -> list:
    """The 140 new frames, as `[{"walk", "frame", "target"}]`, sorted."""
    man = mf.build()
    pool = sorted(k for k, v in man["frames"].items()
                  if "I2261" in v["sets"] and v["visible"] and k not in man["steer"])
    rng = random.Random(SEED)
    rng.shuffle(pool)
    out, per = [], Counter()
    for key in pool:
        walk, frame = key.split("/")
        if per[walk] < PER_WALK:
            out.append({"walk": walk, "frame": frame, "target": man["frames"][key]["target"]})
            per[walk] += 1
        if len(out) == N_NEW:
            break
    return sorted(out, key=lambda r: (r["walk"], r["frame"]))


def label_row(row: dict, left: float, right: float) -> dict:
    """A viewed frame's label from the target's left and right extent, as
    fractions of the width (amendment 6)."""
    centre = (left + right) / 2
    return {**row, "left_frac": round(left, 3), "right_frac": round(right, 3),
            "centre_x_frac": round(centre, 3), "truth": third(centre),
            "border": min(abs(centre - 1 / 3), abs(centre - 2 / 3)) <= BORDER,
            "wide": right - left > 0.5}


def box_frac(row: dict) -> Optional[float]:
    """The centre of a 3.92 row's box as a fraction of the frame's width;
    None when the row holds no box. Perception's rows carry the frame's
    size; VE's box is in the pixels of the image it saw."""
    box = row.get("box")
    if not box:
        return None
    width = row.get("w") or s3214.model_size(*_size(row["key"]))[0]
    return min(1.0, max(0.0, (box[0] + box[2]) / 2 / width))


def _size(key: str) -> tuple:
    return _SIZES[key]


def _rows(name: str) -> dict:
    return {r["key"]: r for r in mf._jsonl(T392 / name)}


_PERCEPTION = _rows("perception.jsonl")
_VE = _rows("qwen-VE.jsonl")
_SIZES = {k: (r["w"], r["h"]) for k, r in _PERCEPTION.items()}


def local_order(key: str) -> Optional[str]:
    """3.80.I's order: perception's detection (P >= 0.8) gives its box's
    third; else VE's yes with a box gives its; else no answer."""
    p = _PERCEPTION[key]
    if p["status"] == "detected" and p.get("box"):
        return third(box_frac(p))
    v = _VE[key]
    if v["status"] == "answered" and (v["score"] or 0) >= s3214.LOCAL_YES_AT and v.get("box"):
        return third(box_frac(v))
    return None


def test_set() -> list:
    """S1's 60 and A's labelled new frames: `[{"key", "truth", "border", "set"}]`."""
    out = [{"key": f"{r['walk']}/{r['frame']}", "truth": r["truth"], "border": r["border"],
            "set": "S1"} for r in json.loads(S1_LABELS.read_text())["rows"]]
    if LABELS.exists():
        out += [{"key": f"{r['walk']}/{r['frame']}", "truth": r["truth"], "border": r["border"],
                 "set": "new"} for r in json.loads(LABELS.read_text())["rows"]]
    return out


def tally(frames: list, answer) -> dict:
    """Right, silent and wrong-side counts for `answer(key)` over `frames`;
    on border frames either adjacent third is right."""
    right = silent = 0
    for f in frames:
        a = answer(f["key"])
        ok = NEIGHBOURS[f["truth"]] if f["border"] else {f["truth"]}
        silent += a is None
        right += a in ok
    n = len(frames)
    wrong = n - right - silent
    half = 1.96 * math.sqrt(right / n * (1 - right / n) / n) if n else None
    return {"frames": n, "right": right, "silent": silent, "wrong_side": wrong,
            "right_share": round(right / n, 4) if n else None,
            "half_width_95": round(half, 4) if n else None}


def scores() -> dict:
    frames = test_set()
    clear = [f for f in frames if not f["border"]]
    edge = [f for f in frames if f["border"]]
    out = {"frames": len(frames), "new": sum(f["set"] == "new" for f in frames),
           "unflagged": tally(clear, local_order), "border": tally(edge, local_order),
           "unflagged_s1_only": tally([f for f in clear if f["set"] == "S1"], local_order),
           "unflagged_new_only": tally([f for f in clear if f["set"] == "new"], local_order)}
    out["criterion_1_stop"] = (out["unflagged"]["right_share"] or 0) >= STOP_AT
    return out


def main(argv: Optional[list] = None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--draw", action="store_true")
    ap.add_argument("--score", action="store_true")
    args = ap.parse_args(argv)
    if args.draw:
        rows = draw()
        DRAW.write_text(json.dumps({"seed": SEED, "per_walk": PER_WALK, "rows": rows}, indent=1) + "\n")
        print(len(rows), "frames;", dict(Counter(r["target"] for r in rows)))
    if args.score:
        out = scores()
        (HERE / "a-scores.json").write_text(json.dumps(out, indent=1) + "\n")
        print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
