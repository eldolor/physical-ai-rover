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
SPOT_N, SPOT_MAX_DIFFER = 20, 2      # amendment 6: 3 or more unflagged differences void the labels


def accepted(truth: str, centre: float, border: bool) -> set:
    """The answers counted right: the truth, and on a border frame also the
    third across the line its centre is near -- never the far third."""
    if not border:
        return {truth}
    return {"LEFT", "FORWARD"} if abs(centre - 1 / 3) <= abs(centre - 2 / 3) else {"FORWARD", "RIGHT"}


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
    # Each edge is clipped to the frame before the middle is taken, as the
    # shipped reading does (`brain.local_vlm._box_frac`).
    x0, x1 = (min(float(width), max(0.0, v)) for v in (box[0], box[2]))
    return (x0 + x1) / 2 / width


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


def _labels() -> dict:
    return json.loads(LABELS.read_text()) if LABELS.exists() else {"rows": [], "dropped": []}


def spot_check_draw() -> list:
    """The 20 labelled new frames shown to the user (amendment 6), by `i`."""
    rows = _labels()["rows"]
    pick = sorted(random.Random(SEED).sample(range(len(rows)), SPOT_N))
    return [rows[k]["i"] for k in pick]


def test_set() -> list:
    """S1's 60 and A's labelled new frames:
    `[{"key", "truth", "centre", "border", "wide", "set"}]`."""
    def one(r, name):
        return {"key": f"{r['walk']}/{r['frame']}", "truth": r["truth"],
                "centre": r["centre_x_frac"], "border": r["border"], "wide": r["wide"], "set": name}
    labels = _labels()
    out = ([one(r, "S1") for r in json.loads(S1_LABELS.read_text())["rows"]]
           + [one(r, "new") for r in labels["rows"]])
    # A person's correction to an S1 label is applied here only;
    # `labels-s1.json` stays as 3.80.I recorded it.
    fixes = labels.get("s1_corrections", {})
    unknown = set(fixes) - {f["key"] for f in out if f["set"] == "S1"}
    if unknown:
        raise ValueError(f"s1_corrections names frames that are not in S1: {sorted(unknown)}")
    for f in out:
        if f["set"] == "S1" and f["key"] in fixes:
            f["truth"] = fixes[f["key"]]["truth"]
    return out


def tally(frames: list, answer) -> dict:
    """Right, silent and wrong-side counts for `answer(key)` over `frames`."""
    right = silent = 0
    for f in frames:
        a = answer(f["key"])
        silent += a is None
        right += a in accepted(f["truth"], f["centre"], f["border"])
    n = len(frames)
    wrong = n - right - silent
    half = 1.96 * math.sqrt(right / n * (1 - right / n) / n) if n else None
    return {"frames": n, "right": right, "silent": silent, "wrong_side": wrong,
            "right_share": round(right / n, 4) if n else None,
            "half_width_95": round(half, 4) if n else None}


def scores() -> dict:
    labels = _labels()
    frames = test_set()
    clear = [f for f in frames if not f["border"]]
    edge = [f for f in frames if f["border"]]
    u = tally(clear, local_order)
    dropped = [{"key": f"{r['walk']}/{r['frame']}",
                "local_order": local_order(f"{r['walk']}/{r['frame']}")} for r in labels["dropped"]]
    spot = labels.get("spot_check")
    return {
        "frames": len(frames), "new": sum(f["set"] == "new" for f in frames),
        "unflagged": u, "border": tally(edge, local_order),
        "unflagged_s1_only": tally([f for f in clear if f["set"] == "S1"], local_order),
        "unflagged_new_only": tally([f for f in clear if f["set"] == "new"], local_order),
        # Reported: frames where the target fills the view have no clear third.
        "unflagged_wide": tally([f for f in clear if f["wide"]], local_order),
        "unflagged_not_wide": tally([f for f in clear if not f["wide"]], local_order),
        # Drawn frames people labelled in view where no target could be placed:
        # not in the tallies above, counted here with the local order's answer.
        "dropped": dropped,
        "unflagged_right_share_with_dropped_as_silent":
            round(u["right"] / (u["frames"] + len(dropped)), 4),
        # Amendment 6: the new frames' labels stand only after the user's spot-check.
        "provisional": spot is None or not spot.get("labels_stand"),
        "spot_check": spot,
        "criterion_1_stop": u["right"] >= STOP_AT * u["frames"],
    }


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
        if not LABELS.exists():
            raise SystemExit(f"refused: {LABELS.name} is missing -- the new frames are not labelled")
        out = scores()
        (HERE / "a-scores.json").write_text(json.dumps(out, indent=1) + "\n")
        print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
