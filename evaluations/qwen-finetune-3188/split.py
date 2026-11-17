# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/qwen-finetune-3188/split.py

3.188.A: the leave-one-object-out folds for fine-tuning Qwen2.5-VL-3B on
frame identity, and the stock engine's numbers on each held-out object.

Reads only committed records: 3.92's `qwen-VE.jsonl` (VE's answer on every
frame of the pinned labelled set, with the frame's human label). No
frames, models or network. Writes `split.json`, pinned by
`tests/test_qwen_finetune_split.py`.

    .venv/bin/python evaluations/qwen-finetune-3188/split.py

Each fold holds out one OBJECT: every frame of every walk whose target is
that object. The fold's model trains on the other five objects' frames
and is judged only on the held-out object's. A walk never straddles a
fold, because a walk has one target.
"""

import hashlib
import json
from pathlib import Path

HERE = Path(__file__).parent
ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "evaluations" / "tier-calibration-392" / "qwen-VE.jsonl"
YES_AT = 0.5                                  # LOCAL_YES_AT, as 3.92 scored it
OBJECT = {"burgundy backpack": "red backpack"}   # one bag (3.92's obj())


def obj(target: str) -> str:
    t = target.strip().lower()
    return OBJECT.get(t, t)


def rows():
    return [json.loads(l) for l in SRC.read_text().splitlines() if l.strip()]


def stock(rs) -> dict:
    """VE's frame answers on these rows, scored as 3.92 criterion 3 did:
    yes at score >= 0.5; an unanswered frame is neither yes nor right."""
    yes = [r for r in rs if r.get("score") is not None and r["score"] >= YES_AT]
    tp = sum(r["visible"] for r in yes)
    in_view = sum(r["visible"] for r in rs)
    return {"frames": len(rs), "in_view": in_view, "yes": len(yes),
            "false_yes": len(yes) - tp, "false_no": in_view - tp,
            "precision": round(tp / len(yes), 4) if yes else None,
            "recall": round(tp / in_view, 4) if in_view else None,
            "false_yes_keys": sorted(r["key"] for r in yes if not r["visible"]),
            "false_no_keys": sorted(r["key"] for r in rs if r["visible"] and r not in yes)}


def main() -> dict:
    rs = rows()
    keys = sorted(r["key"] for r in rs)
    objects = sorted({obj(r["target"]) for r in rs})
    folds = {}
    for o in objects:
        held = [r for r in rs if obj(r["target"]) == o]
        train = [r for r in rs if obj(r["target"]) != o]
        folds[o] = {"held_out_walks": sorted({r["key"].split("/")[0] for r in held}),
                    "train_frames": len(train),
                    "train_in_view": sum(r["visible"] for r in train),
                    "stock": stock(held)}
    pooled = stock(rs)
    out = {"source": SRC.relative_to(ROOT).as_posix(),
           "set_hash": hashlib.sha256("\n".join(keys).encode()).hexdigest()[:12],
           "frames": len(rs), "objects": objects, "yes_at": YES_AT,
           "stock_pooled": {k: v for k, v in pooled.items() if not k.endswith("_keys")},
           "folds": folds}
    (HERE / "split.json").write_text(json.dumps(out, indent=1) + "\n")
    return out


if __name__ == "__main__":
    s = main()
    print(f"{s['frames']} frames, set {s['set_hash']}, stock pooled {s['stock_pooled']}")
    for o, f in s["folds"].items():
        st = f["stock"]
        print(f"  {o:32s} held {st['frames']:4d} in view {st['in_view']:3d}  "
              f"false yes {st['false_yes']:2d}  false no {st['false_no']:2d}  "
              f"P {st['precision']}  R {st['recall']}  train {f['train_frames']}")
