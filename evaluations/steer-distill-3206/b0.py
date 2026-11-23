# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/steer-distill-3206/b0.py

3.206.B0 (amendment 2): open steering teachers scored from answers already
committed -- nothing is asked again, nothing is bought. Criterion 2 on S1's
60 in-view frames: a label is kept when the mirrored answer is the original
with left and right swapped (for a pair, when both teachers keep the frame
and agree); >= 95% right on what is kept, >= 70% of the frames kept.

* the 72B: 3.214's `qwen.jsonl`, its box's third gated by its own yes
  (amendment 1, as 3.214's criterion 6 read it);
* Florence-2-base and -large: 3.80.R's board rows, `box_answer` and the
  mirror's (`fl-*-phrase.scaled.jsonl`).

    .venv/bin/python evaluations/steer-distill-3206/b0.py   # writes b0-scores.json
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parents[1]
PROBE = ROOT / "evaluations/teacher-probe-3214"
R380 = ROOT / "evaluations/tiered-380/R"

_spec = importlib.util.spec_from_file_location("score_3214", PROBE / "score.py")
s3214 = importlib.util.module_from_spec(_spec)   # by path: two `score` modules exist
_spec.loader.exec_module(s3214)

SWAP = s3214.SWAP
MIN_KEPT_RIGHT, MIN_KEPT_SHARE = 0.95, 0.70          # criterion 2, unchanged


def s1_truth() -> dict:
    """S1's key -> the people's direction (3.214's manifest, from 3.80.I)."""
    steer = s3214.mf.build()["steer"]
    return {k: v["truth"] for k, v in steer.items() if v["set"] == "S1"}


def qwen_answers(truth: dict) -> tuple:
    """The 72B's (direct, mirrored) thirds on S1, gated by its own yes."""
    rows = {r["id"]: r for r in s3214.mf._jsonl(PROBE / "qwen.jsonl")}

    def ans(row):
        return s3214.third(row["frac_x"]) if row["yes"] else None
    return ({k: ans(rows[k]) for k in truth}, {k: ans(rows[k + "#mirror"]) for k in truth})


def florence_answers(name: str, truth: dict) -> tuple:
    """Florence's (direct, mirrored) `box_answer` on S1 from 3.80.R's rows.
    Refuses rows whose recorded truth disagrees with the manifest's."""
    rows = {}
    for r in s3214.mf._jsonl(R380 / f"{name}.scaled.jsonl"):
        if r["set"] == "S1":
            rows[r["key"]] = r
    missing = sorted(set(truth) - set(rows))
    if missing:
        raise ValueError(f"{name}: {len(missing)} S1 frames missing, first {missing[0]}")
    for k in truth:
        if rows[k]["truth"] != truth[k]:
            raise ValueError(f"{name}: {k} truth {rows[k]['truth']} != {truth[k]}")
    return ({k: rows[k].get("box_answer") for k in truth},
            {k: (rows[k].get("mirror") or {}).get("box_answer") for k in truth})


def kept(direct: dict, mirrored: dict) -> dict:
    """key -> label, where both answers exist and the mirror is the swap."""
    return {k: d for k, d in direct.items()
            if d is not None and mirrored.get(k) == SWAP[d]}


def pair(a: dict, b: dict) -> dict:
    """Frames both teachers keep, with the same label."""
    return {k: v for k, v in a.items() if b.get(k) == v}


def score(labels: dict, truth: dict) -> dict:
    right = sum(labels[k] == truth[k] for k in labels)
    share = len(labels) / len(truth)
    right_share = right / len(labels) if labels else 0.0
    return {"frames": len(truth), "kept": len(labels), "kept_right": right,
            "kept_share": round(share, 4), "kept_right_share": round(right_share, 4),
            "meets_criterion_2": share >= MIN_KEPT_SHARE and right_share >= MIN_KEPT_RIGHT}


def arms() -> dict:
    truth = s1_truth()
    k72 = kept(*qwen_answers(truth))
    kb = kept(*florence_answers("fl-base-phrase", truth))
    kl = kept(*florence_answers("fl-large-phrase", truth))
    labels = {"72B": k72, "Florence-2-base": kb, "Florence-2-large": kl,
              "72B + Florence-2-base": pair(k72, kb), "72B + Florence-2-large": pair(k72, kl)}
    return {name: score(lab, truth) for name, lab in labels.items()}


def main() -> None:
    out = arms()
    (HERE / "b0-scores.json").write_text(json.dumps(out, indent=1) + "\n")
    for name, s in out.items():
        print(f"{name:24s} kept {s['kept']}/{s['frames']} ({s['kept_share']:.1%}), "
              f"right {s['kept_right']} ({s['kept_right_share']:.1%}) "
              f"{'MET' if s['meets_criterion_2'] else 'FAILED'}")


if __name__ == "__main__":
    main()
