# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/steer-distill-3206/c_labels.py

3.206.C (amendments 4-6): the training labels. The teacher is the 72B: its
box on a frame people labelled in view is the label, kept when the 235B's
recorded box on the same frame overlaps it (IoU >= 0.5). Nothing is asked
of any model and no frame is read.

    .venv/bin/python evaluations/steer-distill-3206/c_labels.py --extract ROWS-QWEN.JSONL
        # once: the 72B's boxes on the in-view frames, from 3.214's full rows
        # (the deploy bucket's private copy) -> c-72b-boxes.json
    .venv/bin/python evaluations/steer-distill-3206/c_labels.py --build
        # c-labels.json, c-scores.json
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import random
from collections import Counter
from pathlib import Path
from typing import Optional

HERE = Path(__file__).parent


def _by_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


a = _by_path("a_3206_c", HERE / "a_set.py")
b2 = _by_path("b2_3206_c", HERE / "b2_bedrock.py")
s3214, mf, third = a.s3214, a.mf, a.third
parse_box = s3214.parse_box
model_size = s3214.model_size

MIN_IOU = 0.5
AUDIT_N, AUDIT_MIN_RIGHT = 50, 48
MIN_KEPT_SHARE, MIN_KEPT_RIGHT = 0.70, 0.95          # criterion 2
BOXES_72B = HERE / "c-72b-boxes.json"
LABELS = HERE / "c-labels.json"
SCORES = HERE / "c-scores.json"


def exact_frac(box: Optional[list], width: float, height: float) -> Optional[list]:
    """A pixel box as fractions of the frame, each edge clipped to it; None
    for no box or one with no area left. Unrounded: what the check compares."""
    if not box:
        return None
    x0, x1 = (min(float(width), max(0.0, v)) / width for v in (box[0], box[2]))
    y0, y1 = (min(float(height), max(0.0, v)) / height for v in (box[1], box[3]))
    if x1 <= x0 or y1 <= y0:
        return None
    return [x0, y0, x1, y1]


def frac_box(box: Optional[list], width: float, height: float) -> Optional[list]:
    """`exact_frac`, rounded to 4 places for the record."""
    f = exact_frac(box, width, height)
    return [round(v, 4) for v in f] if f else None


def outside(box: Optional[list], width: float, height: float) -> bool:
    """Whether a box reaches past the image it was drawn on."""
    return bool(box) and (box[0] < 0 or box[1] < 0 or box[2] > width or box[3] > height)


def iou(p: list, q: list) -> float:
    w = min(p[2], q[2]) - max(p[0], q[0])
    h = min(p[3], q[3]) - max(p[1], q[1])
    if w <= 0 or h <= 0:
        return 0.0
    inter = w * h
    return inter / ((p[2] - p[0]) * (p[3] - p[1]) + (q[2] - q[0]) * (q[3] - q[1]) - inter)


def student_box(frac: list, size: tuple) -> list:
    """The label as the student writes a box: whole pixels of the image it
    sees (`model_size` of the frame's size), Qwen2.5-VL's own format."""
    w, h = model_size(*size)
    return [round(frac[0] * w), round(frac[1] * h), round(frac[2] * w), round(frac[3] * h)]


def extract(rows_path: Path) -> dict:
    """key -> {"size", "box"} for every unmirrored frame people labelled in
    view: the 72B's first box in the pixels of the image it saw, or None."""
    frames = mf.build()["frames"]
    want = {k for k, v in frames.items() if v["visible"]}
    out, errored = {}, set()
    for r in mf._jsonl(rows_path):
        key = r["key"]
        if r["mirror"] or key not in want:
            continue
        if "error" in r:                       # a later row may answer it
            errored.add(key)
            continue
        if key in out:
            raise ValueError(f"{key}: answered twice in {rows_path.name}")
        out[key] = {"size": r["size"], "box": parse_box(r["text"] or "")}
    missing = sorted(want - set(out))
    if missing:
        raise ValueError(f"{len(missing)} in-view frames have no answer from the 72B "
                         f"({len(errored & set(missing))} of them errors): {missing[:3]}")
    return dict(sorted(out.items()))


def boxes_235b() -> dict:
    """key -> the 235B's first box in 0-1000, or None, from the committed rows."""
    rows = {**b2.reused(), **b2.pilot_rows(), **b2.b1.latest(b2.b1._load(b2.ROWS))}
    return {k: parse_box(r["text"] or "") for k, r in rows.items()}


def test_keys() -> set:
    """Every frame of the test set's draw: S1's 60 and all 140 of A's."""
    s1 = {f"{r['walk']}/{r['frame']}" for r in json.loads(a.S1_LABELS.read_text())["rows"]}
    drawn = {f"{r['walk']}/{r['frame']}" for r in json.loads(a.DRAW.read_text())["rows"]}
    return s1 | drawn


def judge(key: str, t72: dict, q235: dict) -> dict:
    """One in-view frame under the label check. A frame either model was
    never asked is an error here, never a silent "no box"."""
    if key not in q235:
        raise KeyError(f"{key}: no answered row from the 235B")
    rec = t72[key]
    size = tuple(rec["size"])
    seen = model_size(*size)
    teacher, other = exact_frac(rec["box"], *seen), exact_frac(q235[key], 1000.0, 1000.0)
    overlap = iou(teacher, other) if teacher and other else None
    label = student_box(teacher, size) if teacher else None
    why = ("no 72B box" if not teacher else "no 235B box" if not other
           else "IoU under 0.5" if overlap < MIN_IOU
           else "the label has no area in the student's pixels"
           if label[2] <= label[0] or label[3] <= label[1] else None)
    return {"key": key, "size": list(size),
            "teacher": frac_box(rec["box"], *seen), "other": frac_box(q235[key], 1000.0, 1000.0),
            "iou": round(overlap, 4) if overlap is not None else None,
            "teacher_past_the_image": outside(rec["box"], *seen),
            "kept": why is None, "why_not": why, "label": label if why is None else None}


def build() -> tuple:
    frames = mf.build()["frames"]
    t72 = json.loads(BOXES_72B.read_text())
    q235 = boxes_235b()
    held = test_keys()
    train = [k for k, v in frames.items() if "I2261" in v["sets"] and v["visible"] and k not in held]
    rows = [{**judge(k, t72, q235), "target": frames[k]["target"]} for k in train]
    kept = [r for r in rows if r["kept"]]
    audit = sorted(random.Random(a.SEED).sample([r["key"] for r in kept], AUDIT_N))

    # Criterion 2, on A's new unflagged frames only.
    def crit(test_frames: list) -> dict:
        got = [(f, judge(f["key"], t72, q235)) for f in test_frames]
        k = [(f, j) for f, j in got if j["kept"]]
        right = sum((third((j["teacher"][0] + j["teacher"][2]) / 2)
                     in a.accepted(f["truth"], f["centre"], f["border"])) for f, j in k)
        n = len(got)
        return {"frames": n, "kept": len(k), "kept_right": right,
                "kept_share": round(len(k) / n, 4) if n else None,
                "kept_right_share": round(right / len(k), 4) if k else None,
                "not_kept": dict(Counter(j["why_not"] for _, j in got if not j["kept"])),
                "wrong": sorted(f["key"] for f, j in k
                                if third((j["teacher"][0] + j["teacher"][2]) / 2)
                                not in a.accepted(f["truth"], f["centre"], f["border"]))}
    ts = a.test_set()
    new_clear = [f for f in ts if f["set"] == "new" and not f["border"]]
    c2 = crit(new_clear)
    scores = {
        "training_frames": len(rows), "kept": len(kept),
        "not_kept": dict(Counter(r["why_not"] for r in rows if not r["kept"])),
        "kept_by_target": dict(Counter(r["target"] for r in kept)),
        "frames_by_target": dict(Counter(r["target"] for r in rows)),
        "criterion_2_new_unflagged": c2,
        "criterion_2_met": (c2["kept_share"] or 0) >= MIN_KEPT_SHARE
        and c2["kept_right"] >= MIN_KEPT_RIGHT * c2["kept"] and c2["kept"] > 0,
        "reported_new_border": crit([f for f in ts if f["set"] == "new" and f["border"]]),
        "reported_s1_unflagged": crit([f for f in ts if f["set"] == "S1" and not f["border"]]),
        # Reported: kept labels whose 72B box reached past the image and was clipped.
        "kept_with_a_clipped_teacher_box": sorted(r["key"] for r in kept
                                                  if r["teacher_past_the_image"]),
        "audit": audit_record(audit),
    }
    labels = {"how": "3.206.C: the 72B's box on each in-view training frame (fractions of the frame; "
                     "`label` in the student's pixels), kept when the 235B's box overlaps it "
                     f"(IoU >= {MIN_IOU}). No frame is in this file.",
              "rows": rows}
    return labels, scores


AUDIT = HERE / "c-audit.json"


def audit_record(keys: list) -> dict:
    """Criterion 3: the user's judgement of the 50 drawn labels, from
    `c-audit.json` (`{"marks": {key: "Y" | "N" | "X"}}`); unjudged until it
    exists. Only "Y" (the box is on the target) counts as right."""
    out = {"n": AUDIT_N, "keys": keys, "judged": False, "right": None, "criterion_3_met": None}
    if AUDIT.exists():
        marks = json.loads(AUDIT.read_text())["marks"]
        if set(marks) != set(keys):
            raise ValueError("c-audit.json's frames are not the 50 drawn")
        right = sum(v == "Y" for v in marks.values())
        out.update(judged=True, right=right, criterion_3_met=right >= AUDIT_MIN_RIGHT,
                   not_right=sorted(k for k, v in marks.items() if v != "Y"))
    return out


def main(argv: Optional[list] = None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--extract", metavar="ROWS")
    ap.add_argument("--build", action="store_true")
    args = ap.parse_args(argv)
    if not (args.extract or args.build):
        ap.error("give --extract ROWS or --build")
    if args.extract:
        out = extract(Path(args.extract))
        BOXES_72B.write_text(json.dumps(out, indent=0) + "\n")
        print(len(out), "in-view frames;", sum(v["box"] is not None for v in out.values()), "with a box")
    if args.build:
        labels, scores = build()
        LABELS.write_text(json.dumps(labels, indent=0) + "\n")
        SCORES.write_text(json.dumps(scores, indent=1) + "\n")
        print(json.dumps({k: v for k, v in scores.items() if k != "audit"}, indent=1))


if __name__ == "__main__":
    main()
