# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/teacher-probe-3214/score.py

3.214: the job's stored replies -> answers -> the section's criteria.
The judgement lives here, not on the rented GPU: the 72B's replies are
scored with the student's own `p_localised` and `parse_box`, Super's by
amendment 1's rules, and both against the people's labels and VE's
committed answers on the same frames.

    .venv/bin/python evaluations/teacher-probe-3214/score.py qwen  rows-qwen.jsonl
    .venv/bin/python evaluations/teacher-probe-3214/score.py super rows-super.jsonl

A run is judged only whole (every task asked, no error row); a pilot's
rows go to `pilot-<model>*`. Writes `<model>.jsonl` (one answer per task: key, target, label, yes,
score, box's x fraction -- never a frame) and `<model>-scores.json`.
"""

from __future__ import annotations

import functools
import importlib.util
import json
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Optional

HERE = Path(__file__).parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from brain.local_vlm import model_size                         # noqa: E402
from brain.tiered import LOCAL_YES_AT                           # noqa: E402
from service.local_vlm.driver import parse_box                  # noqa: E402
from tools.jetson.bench_identity_llama import p_localised      # noqa: E402

_spec = importlib.util.spec_from_file_location("manifest_3214", HERE / "manifest.py")
mf = importlib.util.module_from_spec(_spec)      # by path: no bare `manifest` on sys.path
_spec.loader.exec_module(mf)

SWAP = {"LEFT": "RIGHT", "RIGHT": "LEFT", "FORWARD": "FORWARD"}
# Criteria 1-9's numbers (3.214, written 2026-11-22 before any run).
MAX_MISTAKES = 59
MIN_PRECISION = 0.95
MIN_RECALL_QWEN = 0.901
MIN_DISAGREE_RIGHT, MIN_DISAGREE_N = 0.75, 30
MIN_KEPT_RIGHT, MIN_KEPT_SHARE, MAX_S2_BOXES = 0.95, 0.70, 2
MIN_PARSED = 0.95
FAIL_PILOT = 5               # a pilot is five tasks


def third(frac: Optional[float]) -> Optional[str]:
    """3.80.I's thirds (common.third)."""
    if frac is None:
        return None
    return "LEFT" if frac < 1 / 3 else "RIGHT" if frac > 2 / 3 else "FORWARD"


def object_of(key: str) -> str:
    """The object a walk's frames belong to (3.188's six: the burgundy and
    the red backpack are one bag)."""
    return _objects().get(key.split("/")[0], "?")


@functools.lru_cache(maxsize=1)
def _objects() -> dict:
    return {w: rec["object"] for w, rec in
            json.loads((mf.FT / "frames.json").read_text())["walks"].items()}


# ------------------------------------------------------ replies -> answers
def qwen_answer(row: dict) -> dict:
    """One 72B reply as VE's answer is read: `p_localised`'s score, a yes
    at `LOCAL_YES_AT`, the box's centre as a fraction of the width the
    model saw (`model_size`, VE's pixel bounds)."""
    if "error" in row:
        return {"status": "error", "score": None, "yes": False, "frac_x": None}
    score, parsed, _ = p_localised(row["lp"])
    box = parse_box(row["text"])
    frac = None
    if box and row.get("size"):
        frac = (box[0] + box[2]) / 2 / model_size(*row["size"])[0]
    return {"status": "answered" if parsed else "unparsed", "score": round(score, 4),
            "yes": bool(parsed and score >= LOCAL_YES_AT), "frac_x": frac}


_NUM = r"(-?\d+(?:\.\d+)?)"
_BOX = re.compile(r'"(?:bbox_2d|bbox|box)"\s*:\s*\[\s*' + r"\s*,\s*".join([_NUM] * 4) + r"\s*\]")


def super_box_frac(text: str) -> Optional[float]:
    """The first box in Super's grounding reply, its centre as a fraction
    of the width. Amendment 2: read as Qwen3-VL's 0-1000 relative
    coordinates -- what its repository's own rule resolves to on NVIDIA's
    frames, and what the `auto` mode would misread on ours (under 1000 px
    wide)."""
    m = _BOX.search(text or "")
    if not m:
        return None
    x1, _, x2, _ = (float(v) for v in m.groups())
    if x2 <= x1:
        return None
    return min(1.0, max(0.0, (x1 + x2) / 2 / 1000.0))


def super_answer(row: dict) -> dict:
    """Amendment 1: a yes is an identity reply that starts with "yes"; the
    steer is the grounding box's third (`gated`: only where the identity
    answer is yes)."""
    if "error" in row:
        return {"status": "error", "yes": False, "frac_x": None, "parsed_identity": False,
                "parsed_grounding": False}
    said = (row["identity"]["text"] or "").strip().lower()
    frac = super_box_frac(row["grounding"]["text"])
    return {"status": "answered", "yes": bool(re.match(r"yes\b", said)), "frac_x": frac,
            "parsed_identity": bool(re.match(r"(yes|no)\b", said)),
            "parsed_grounding": frac is not None}


# ------------------------------------------------------------- criteria ---
def identity(answers: list) -> dict:
    """`answers`: `[{"key", "visible", "yes"}]`. A mistake is a false yes
    or an in-view frame without a yes (3.188.A's count)."""
    tp = sum(a["yes"] and a["visible"] for a in answers)
    fy = sum(a["yes"] and not a["visible"] for a in answers)
    missed = sum(a["visible"] and not a["yes"] for a in answers)
    in_view = sum(a["visible"] for a in answers)
    return {"frames": len(answers), "in_view": in_view, "yes": tp + fy, "false_yes": fy,
            "missed": missed, "mistakes": fy + missed,
            "precision": round(tp / (tp + fy), 4) if tp + fy else None,
            "recall": round(tp / in_view, 4) if in_view else None}


def per_object(answers: list) -> dict:
    by = defaultdict(list)
    for a in answers:
        by[object_of(a["key"])].append(a)
    return {obj: identity(rows) for obj, rows in sorted(by.items())}


def disagreement(teacher: list, student: list) -> dict:
    """Frames where the two give different answers: how often the teacher
    is the one who is right (criterion 5)."""
    theirs = {a["key"]: a["yes"] for a in student}
    differ = [a for a in teacher if a["key"] in theirs and theirs[a["key"]] != a["yes"]]
    right = sum(a["yes"] == a["visible"] for a in differ)
    return {"frames": len(differ), "teacher_right": right,
            "share": round(right / len(differ), 4) if differ else None}


def steering(direct: dict, mirrored: dict, steer: dict) -> dict:
    """Criterion 6 (3.206's criterion 2). `direct`/`mirrored`: key -> the
    answered third or None; `steer`: key -> {"set", "truth"}. A label is
    kept when both answers exist and the mirrored one is the original with
    left and right swapped."""
    s1 = [k for k, v in steer.items() if v["set"] == "S1"]
    s2 = [k for k, v in steer.items() if v["set"] == "S2"]
    kept = [k for k in s1 if direct.get(k) and mirrored.get(k) == SWAP[direct[k]]]
    right = sum(direct[k] == steer[k]["truth"] for k in kept)
    return {"s1": len(s1), "answered": sum(direct.get(k) is not None for k in s1),
            "right_all": sum(direct.get(k) == steer[k]["truth"] for k in s1),
            "kept": len(kept), "kept_right": right,
            "kept_share": round(len(kept) / len(s1), 4) if s1 else None,
            "kept_right_share": round(right / len(kept), 4) if kept else None,
            "s2": len(s2), "s2_boxes": sum(direct.get(k) is not None for k in s2)}


def ve_answers() -> list:
    """VE's committed answers on I-2261 (3.92), as `identity` reads them."""
    return [{"key": r["key"], "visible": bool(r["visible"]),
             "yes": r["status"] == "answered" and (r["score"] or 0) >= LOCAL_YES_AT}
            for r in mf._jsonl(mf.VE_2261)]


def complete(answers: dict, man: dict, model: str) -> dict:
    """A run is judged only whole: every task asked, none an error. A
    partial run's counts would be held against bars set for the full sets,
    and an error row reads as a "no"."""
    want = {t["id"] for t in mf.tasks(man, model)}
    missing = sorted(want - set(answers))
    errors = sorted(i for i in want & set(answers) if answers[i]["status"] == "error")
    return {"expected": len(want), "missing": len(missing), "errors": len(errors),
            "whole": not missing and not errors, "first_missing": missing[:3],
            "first_errors": errors[:3]}


def _judged(criteria: dict, cover: dict) -> dict:
    """The criteria, each False when the run is not whole."""
    return criteria if cover["whole"] else {k: False for k in criteria}


def verdicts_qwen(answers: dict, man: dict) -> dict:
    """`answers`: task id -> `qwen_answer`'s dict."""
    frames = man["frames"]
    rows = [{"key": k, "visible": frames[k]["visible"], "yes": answers[k]["yes"]}
            for k in frames if "I2261" in frames[k]["sets"] and k in answers]
    pooled, objects = identity(rows), per_object(rows)
    ve = ve_answers()
    ve_objects = per_object(ve)
    look = {tag: sum(answers[k]["yes"] for k in frames if tag in frames[k]["sets"] and k in answers)
            for tag in ("teal", "taupe")}
    dis = disagreement(rows, ve)
    direct = {k: third(answers[k]["frac_x"]) if answers[k]["yes"] else None
              for k in man["steer"] if k in answers}
    mirrored = {k: third(answers[k + "#mirror"]["frac_x"]) if answers[k + "#mirror"]["yes"] else None
                for k in man["steer"] if k + "#mirror" in answers}
    st = steering(direct, mirrored, man["steer"])
    worse = [o for o in objects if objects[o]["false_yes"] > ve_objects[o]["false_yes"]
             or objects[o]["missed"] > ve_objects[o]["missed"]]
    cover = complete(answers, man, "qwen")
    return {
        "coverage": cover,
        "pooled": pooled, "per_object": objects, "ve_pooled": identity(ve),
        "ve_per_object": ve_objects, "lookalike_yes": look, "disagreement": dis, "steering": st,
        "criteria": _judged({
            "1 mistakes <= 59": pooled["mistakes"] <= MAX_MISTAKES,
            "2 precision >= 95% and recall >= 90.1%":
                (pooled["precision"] or 0) >= MIN_PRECISION
                and (pooled["recall"] or 0) >= MIN_RECALL_QWEN,
            "3 lookalikes 0 yes": look["teal"] == 0 and look["taupe"] == 0,
            "4 no object worse": not worse,
            "5 right on >= 75% of >= 30 disagreements":
                dis["frames"] >= MIN_DISAGREE_N and (dis["share"] or 0) >= MIN_DISAGREE_RIGHT,
            "6 steering kept >= 70%, right >= 95% of kept, S2 boxes <= 2":
                (st["kept_share"] or 0) >= MIN_KEPT_SHARE
                and (st["kept_right_share"] or 0) >= MIN_KEPT_RIGHT
                and st["s2_boxes"] <= MAX_S2_BOXES,
        }, cover),
        "objects_worse": worse,
    }


def verdicts_super(answers: dict, man: dict) -> dict:
    """`answers`: task id -> `super_answer`'s dict."""
    frames = man["frames"]
    asked = [a for a in answers.values()]
    parsed_i = sum(a["parsed_identity"] for a in asked) / len(asked) if asked else 0.0
    # Amendment 3: a box is expected only where the people's label says the
    # target is in view; elsewhere "no box" is the right reply.
    expect = [answers[i] for i in answers if frames[i.split("#")[0]]["visible"]]
    parsed_g = sum(a["parsed_grounding"] for a in expect) / len(expect) if expect else 0.0
    rows = [{"key": k, "visible": frames[k]["visible"], "yes": answers[k]["yes"]}
            for k in frames if "I299" in frames[k]["sets"] and k in answers]
    mine = identity(rows)
    keys = {r["key"] for r in rows}
    ve = identity([a for a in ve_answers() if a["key"] in keys])
    teal = sum(answers[k]["yes"] for k in frames if "teal" in frames[k]["sets"] and k in answers)

    def thirds(gated: bool):
        def one(a):
            return third(a["frac_x"]) if (a["yes"] or not gated) else None
        direct = {k: one(answers[k]) for k in man["steer"] if k in answers}
        mirrored = {k: one(answers[k + "#mirror"]) for k in man["steer"] if k + "#mirror" in answers}
        return steering(direct, mirrored, man["steer"])

    gated, ungated = thirds(True), thirds(False)
    cover = complete(answers, man, "super")
    return {
        "coverage": cover,
        "parsed": {"identity": round(parsed_i, 4), "grounding_in_view": round(parsed_g, 4),
                   "in_view_asked": len(expect)},
        "i299": mine, "ve_i299": ve, "teal_yes": teal,
        "steering_gated": gated, "steering_ungated": ungated,
        "criteria": _judged({
            "7 parseable on >= 95% in each mode": parsed_i >= MIN_PARSED and parsed_g >= MIN_PARSED,
            "8 fewer mistakes than VE on I-299, precision >= 95%, teal 0":
                mine["mistakes"] < ve["mistakes"]
                and (mine["precision"] or 0) >= MIN_PRECISION and teal == 0,
            "9 steering kept >= 70%, right >= 95% of kept, S2 boxes <= 2":
                (gated["kept_share"] or 0) >= MIN_KEPT_SHARE
                and (gated["kept_right_share"] or 0) >= MIN_KEPT_RIGHT
                and gated["s2_boxes"] <= MAX_S2_BOXES,
        }, cover),
    }


def main(model: str, rows_path: str) -> None:  # pragma: no cover -- needs a run's rows
    man = mf.build()
    rows = mf._jsonl(Path(rows_path))
    # A pilot's rows are scored into pilot-* files, never over the run's record.
    stem = model if len(rows) > 2 * FAIL_PILOT else f"pilot-{model}"
    read = qwen_answer if model == "qwen" else super_answer
    answers = {r["id"]: read(r) for r in rows}
    out = [{"id": r["id"], "key": r["key"], "target": r["target"], "mirror": r["mirror"],
            "visible": man["frames"][r["key"]]["visible"], **answers[r["id"]]} for r in rows]
    (HERE / f"{stem}.jsonl").write_text("".join(json.dumps(o) + "\n" for o in out))
    verdict = (verdicts_qwen if model == "qwen" else verdicts_super)(answers, man)
    (HERE / f"{stem}-scores.json").write_text(json.dumps(verdict, indent=1) + "\n")
    print(json.dumps(verdict["criteria"], indent=1))


if __name__ == "__main__":  # pragma: no cover
    main(*sys.argv[1:3])
