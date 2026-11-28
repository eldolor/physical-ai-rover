# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/qwen-finetune-3188/b_score.py

3.188.B (amendment 2): the jobs' stored replies -> answers -> 3.188's
criteria 1-6 and 3.206's criteria 4-10. The judgement lives here, not on
the rented GPU: each reply is read as VE's is (`p_localised`'s score, a
yes at `LOCAL_YES_AT`, `parse_box`'s box), with 3.214's reader.

    .venv/bin/python evaluations/qwen-finetune-3188/b_score.py ROWS.jsonl [ROWS.jsonl ...]

Writes `b-answers.jsonl` (one answer per model and task: id, model, yes,
score, the box in the student's pixels -- never a frame) and
`b-scores.json`. With no rows it re-scores the committed answers.
A model's criteria are judged only on a whole run: every task answered,
no error row.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Optional

HERE = Path(__file__).parent
ROOT = HERE.parents[1]


def _by_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


bm = _by_path("b_manifest_3188_s", HERE / "b_manifest.py")
a = _by_path("a_set_3206_b", ROOT / "evaluations/steer-distill-3206/a_set.py")
s3214, mf = bm.s3214, bm.mf
third, SWAP, identity = s3214.third, s3214.SWAP, s3214.identity

ANSWERS = HERE / "b-answers.jsonl"
SCORES = HERE / "b-scores.json"
# 3.188's criteria (written 2026-11-17, before any training).
RECALL_SLACK = 0.02
MISTAKE_SHARE = 0.75
MIN_PRECISION = 0.95
WORST_OBJECT, WORST_MIN_PRECISION = "grey backpack", 0.90
# 3.206's criteria 4-7 (written 2026-11-21; amendment 4) and A's figures.
MIN_OWN_RIGHT, MIN_LOCAL_RIGHT, MIN_MIRROR = 0.80, 0.90, 0.90
# 3.214's measured cost of the 72B's boxes (its four pilots and its run).
TEACHER_USD = 0.61 + 1.44


def answer(row: dict) -> dict:
    """One reply as VE's answer is read; the box kept in the student's pixels."""
    got = s3214.qwen_answer(row)
    box = None if "error" in row else s3214.parse_box(row["text"] or "")
    return {"id": row["id"], "model": row["model"], "status": got["status"],
            "score": got["score"], "yes": got["yes"], "box": box,
            "size": row.get("size")}


def box_third(ans: Optional[dict]) -> Optional[str]:
    """The answer's steer: the third of its box's centre, each edge clipped
    to the image the model saw (`brain.local_vlm._box_frac`), only on a yes."""
    if not ans or not ans["yes"] or not ans["box"] or not ans["size"]:
        return None
    w = bm.model_size(*ans["size"])[0]
    x0, x1 = (min(float(w), max(0.0, v)) for v in (ans["box"][0], ans["box"][2]))
    return third((x0 + x1) / 2 / w)


def load_rows(paths: list) -> list:
    """Every row of every file; a task answered twice by one model keeps
    the later answer only when the earlier one was an error."""
    out = {}
    for p in paths:
        for r in mf._jsonl(Path(p)):
            k = (r["model"], r["id"])
            if k in out and "error" not in out[k]:
                if "error" in r:
                    continue
                raise ValueError(f"{k} answered twice")
            out[k] = r
    return list(out.values())


def merged(old: list, new: list) -> list:
    """The committed answers with a new run's added: a new answer replaces
    an old one only where the old one was an error; any other clash is a
    model answering a task twice, refused."""
    out = {(x["model"], x["id"]): x for x in old}
    for x in new:
        k = (x["model"], x["id"])
        if k in out and out[k]["status"] != "error" and out[k] != x:
            raise ValueError(f"{k} is already answered differently in {ANSWERS.name}")
        out[k] = x
    return sorted(out.values(), key=lambda x: (x["model"], x["id"]))


def by_model(answers: list) -> dict:
    out = defaultdict(dict)
    for x in answers:
        out[x["model"]][x["id"]] = x
    return dict(out)


def whole(got: dict, tasks: list) -> dict:
    want = {t["id"] for t in tasks}
    missing = sorted(want - set(got))
    errors = sorted(i for i in want & set(got) if got[i]["status"] == "error")
    return {"expected": len(want), "missing": len(missing), "errors": len(errors),
            "whole": not missing and not errors, "first_missing": missing[:3]}


def _ident(got: dict, keys: list, frames: dict) -> dict:
    return identity([{"key": k, "visible": frames[k]["visible"], "yes": got[k]["yes"]} for k in keys])


def model_for(key: str, folds: dict) -> str:
    """The fold model that never saw this frame's object."""
    obj = s3214.object_of(key)
    return next(f"fold-{f}" for f, rec in folds.items() if rec["held_out"] == obj)


def scores(answers: list, spend: Optional[dict] = None) -> dict:
    b = bm.build()
    folds = {f: rec for f, rec in b["folds"].items() if f != "all"}
    frames = mf.build()["frames"]
    got = by_model(answers)
    cover = {"stock": whole(got.get("stock", {}), b["stock"])}
    for f, rec in b["folds"].items():
        cover[f"fold-{f}"] = whole(got.get(f"fold-{f}", {}), rec["asked"])
    out = {"coverage": cover}
    if spend:
        out["cost"] = {"gpu_usd": spend.get("spent_usd"),
                       "teacher_usd_per_kept_label": round(TEACHER_USD / 314, 4),
                       "jobs": [{k: j.get(k) for k in ("id", "run", "folds", "stage", "running_secs", "usd")}
                                for j in spend.get("jobs", [])]}
    fold_whole = all(cover[f"fold-{f}"]["whole"] for f in folds)
    if not cover["stock"]["whole"]:
        out["note"] = "the stock run is not whole: nothing is judged"
        return out

    stock = got["stock"]
    in_2261 = sorted(k for k, v in frames.items() if "I2261" in v["sets"])
    look = json.loads((mf.FT / "lookalikes.json").read_text())
    teal, taupe = look["teal_bin"]["keys"], look["taupe_bag"]["keys"]
    per_stock = {rec["held_out"]: _ident(stock, [k for k in in_2261 if s3214.object_of(k) == rec["held_out"]], frames)
                 for rec in folds.values()}
    out["stock"] = {"pooled": _ident(stock, in_2261, frames), "per_object": per_stock,
                    "teal_yes": sum(stock[k]["yes"] for k in teal),
                    "taupe_yes": sum(stock[k]["yes"] for k in taupe),
                    "ve_pooled": identity(s3214.ve_answers())}
    out["stock_steering"] = steering(lambda k: stock.get(k), lambda k: stock.get(k + "#mirror"))
    if not fold_whole:
        out["note"] = "a fold run is not whole: the fold criteria are not judged"
        return out

    # One answer per frame of I-2261: its object's fold model.
    held = {k: got[model_for(k, folds)][k] for k in in_2261}
    per_fold = {rec["held_out"]: _ident(held, [k for k in in_2261 if s3214.object_of(k) == rec["held_out"]], frames)
                for rec in folds.values()}
    pooled = _ident(held, in_2261, frames)
    teal_yes = sum(got[model_for(k, folds)][k]["yes"] for k in teal)
    taupe_yes = sum(got["fold-" + _fold_of(bm.TAUPE_FOLD_OBJECT, folds)][k]["yes"] for k in taupe)
    c1 = {o: per_fold[o]["false_yes"] <= per_stock[o]["false_yes"]
          and (per_fold[o]["recall"] or 0) >= (per_stock[o]["recall"] or 0) - RECALL_SLACK
          for o in per_fold}
    crit_3188 = {
        "1 no object worse": all(c1.values()),
        "2 mistakes <= 0.75 x stock's": pooled["mistakes"] <= MISTAKE_SHARE * out["stock"]["pooled"]["mistakes"],
        "3 pooled precision >= 95%": (pooled["precision"] or 0) >= MIN_PRECISION,
        "4 grey backpack precision >= 90%": (per_fold[WORST_OBJECT]["precision"] or 0) >= WORST_MIN_PRECISION,
        "5 teal 0 of 34, taupe 0 of 7": teal_yes == 0 and taupe_yes == 0,
    }
    out["folds"] = {"pooled": pooled, "per_object": per_fold, "per_object_c1": c1,
                    "teal_yes": teal_yes, "taupe_yes": taupe_yes,
                    "mistake_bar": MISTAKE_SHARE * out["stock"]["pooled"]["mistakes"]}
    out["criteria_3188"] = crit_3188

    def fold_ans(k):
        return got[model_for(k.split("#")[0], folds)].get(k)
    st = steering(lambda k: fold_ans(k), lambda k: fold_ans(k + "#mirror"))
    out["fold_steering"] = st
    base = a.scores()["unflagged"]
    out["criteria_3206"] = {
        "4 own box right >= 80% of unflagged": st["own"]["right_share"] >= MIN_OWN_RIGHT,
        "5 local order right >= 90% and above A by its half-width":
            st["local"]["right_share"] >= MIN_LOCAL_RIGHT
            and st["local"]["right_share"] > base["right_share"] + base["half_width_95"],
        "6 local order wrong side <= A's": st["local"]["wrong_side"] <= base["wrong_side"],
        "7 mirror consistency >= 90%, LEFT and RIGHT both used":
            st["mirror"]["share"] >= MIN_MIRROR and st["mirror"]["both_words"],
        "8 no object's local order right count below A's":
            all(st["local_per_object"][o]["right"] >= st["a_per_object"][o]["right"]
                for o in st["a_per_object"]),
        "9 3.188's criteria 1-5": all(crit_3188.values()),
    }
    if "all" in got and cover["fold-all"]["whole"]:
        allm = got["fold-all"]
        out["all_objects_reported"] = {
            "steering": steering(lambda k: allm.get(k), lambda k: allm.get(k + "#mirror")),
            "teal_yes_seen_walk": sum(allm[k]["yes"] for k in teal),
            "taupe_yes": sum(allm[k]["yes"] for k in taupe)}
    return out


def _fold_of(obj: str, folds: dict) -> str:
    return next(f for f, rec in folds.items() if rec["held_out"] == obj)


def steering(direct, mirrored) -> dict:
    """3.206's criteria 4-8 on its test set (197 labelled frames).
    `direct(key)` / `mirrored(key)`: the answer for the frame / its mirror."""
    frames = a.test_set()
    clear = [f for f in frames if not f["border"]]
    own = lambda k: box_third(direct(k))                     # noqa: E731

    def local(k):
        p = a._PERCEPTION[k]
        if p["status"] == "detected" and p.get("box"):
            return third(a.box_frac(p))
        return own(k)

    consistent, words = 0, set()
    for f in clear:
        d, m = own(f["key"]), box_third(mirrored(f["key"]))
        words.add(d)
        consistent += d is not None and m is not None and m == SWAP[d]
    objs = sorted({s3214.object_of(f["key"]) for f in clear})
    return {
        "own": a.tally(clear, own), "own_border": a.tally([f for f in frames if f["border"]], own),
        "local": a.tally(clear, local),
        "local_border": a.tally([f for f in frames if f["border"]], local),
        "mirror": {"frames": len(clear), "consistent": consistent,
                   "share": round(consistent / len(clear), 4),
                   "both_words": {"LEFT", "RIGHT"} <= words},
        "local_per_object": {o: a.tally([f for f in clear if s3214.object_of(f["key"]) == o], local)
                             for o in objs},
        "a_per_object": {o: a.tally([f for f in clear if s3214.object_of(f["key"]) == o], a.local_order)
                         for o in objs},
    }


def main(argv=None) -> None:
    paths = list(argv if argv is not None else sys.argv[1:])
    if paths:
        ANSWERS.write_text("".join(json.dumps(x) + "\n" for x in merged(
            mf._jsonl(ANSWERS) if ANSWERS.exists() else [], [answer(r) for r in load_rows(paths)])))
    answers = mf._jsonl(ANSWERS)
    spend = json.loads((HERE / "b-spend.json").read_text()) if (HERE / "b-spend.json").exists() else None
    out = scores(answers, spend)
    SCORES.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: out.get(k) for k in ("coverage", "criteria_3188", "criteria_3206", "note")}, indent=1))


if __name__ == "__main__":
    main()
