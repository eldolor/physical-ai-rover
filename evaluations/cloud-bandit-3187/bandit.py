# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/cloud-bandit-3187/bandit.py

3.187.D: the learned bandit (A2's two decisions; a logistic model of each
arm's outcome, the direct method, fitted on the training walks only),
judged on the held-out walks against the tree, with F0 and value of
information (G) beside it. The criteria, the context, the grid and the
verdict rule are D0 in `docs/plans/ros-alignment/3.187-cloud-call-bandit.md`,
written before any fit. Free: reads committed records, calls nothing.
Writes `bandit-scores.json`.

    python evaluations/cloud-bandit-3187/bandit.py
"""

import argparse
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from common import ROOT, local_kind, walk_of  # noqa: E402

# G's harness: the data, the frame accounting, the totals, the bars.
_spec = importlib.util.spec_from_file_location("cloud_bandit_3187_voi_score_d", HERE / "voi" / "score.py")
voi = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(voi)
fit_logistic, predict_logistic, logit = voi.fit_logistic, voi.predict_logistic, voi.logit
COST, LAMBDAS, MUS, D_MIN = voi.COST, voi.LAMBDAS, voi.MUS, voi.D_MIN

sys.path.insert(0, str(ROOT))
from brain.perceive import coco_class_for  # noqa: E402

RECENT = 5               # "the gate passed in the 5 frames before"
VOI_CELL = "lambda=50,mu=0"   # G's judged cell, reported beside


# -- context (D0: policy-independent features only) -------------------------

def contexts(data) -> dict:
    """{key: context vector}, per walk in frame order. Nothing here depends
    on what any policy asked."""
    out = {}
    by_walk = {}
    for f in data["frames"]:
        by_walk.setdefault(walk_of(f["key"]), []).append(f)
    for walk, fs in by_walk.items():
        streak, history = 0, []
        coco = float(coco_class_for(fs[0]["target"]) is not None)
        for i, f in enumerate(fs):
            prow = data["per"][f["key"]]
            det = prow["status"] == "detected"
            streak = streak + 1 if det else 0
            recent = float(any(history[-RECENT:]))
            out[f["key"]] = voi.gate_x(prow) + [math.log1p(streak), recent, math.log1p(i), coco]
            history.append(det)
    return out


def ve_part(vrow, with_score: bool) -> list:
    kind = local_kind(vrow)
    vy, vn = float(kind == "yes"), float(kind == "no")
    if not with_score:
        return [vy]
    ls = float(logit(vrow["score"])) if isinstance(vrow.get("score"), (int, float)) else 0.0
    return [vy, vn, vy * ls, vn * ls]


# -- the models ------------------------------------------------------------

def _fit(X, y):
    X, y = np.array(X, float), np.array(y, float)
    if len(set(y.tolist())) < 2:
        # A constant outcome: a model that predicts it (smoothed).
        p = (y.sum() + 1) / (len(y) + 2)
        return ("const", p)
    return ("lr", fit_logistic(X, y))


def _pred(m, x) -> float:
    if m[0] == "const":
        return float(m[1])
    return float(predict_logistic(m[1], np.array([x], float))[0])


class Bandit:
    """D0's per-arm outcome models, fitted on `label_keys` (labels, VE) and
    `cloud_keys` (the frames of those whose answer is in the sample)."""

    def __init__(self, data, ctx, label_keys, cloud_keys):
        per, ve, vis, ans = data["per"], data["ve"], data["visible"], data["ans"]
        ks = sorted(label_keys)
        b0 = {k: per[k]["status"] == "detected" for k in ks}
        self.r0 = _fit([ctx[k] for k in ks], [b0[k] == vis[k] for k in ks])

        def b1(k):
            kind = local_kind(ve[k])
            return b0[k] if kind == "unable" else kind == "yes"
        self.rL = _fit([ctx[k] for k in ks], [b1(k) == vis[k] for k in ks])
        dec = [k for k in ks if local_kind(ve[k]) != "unable"]
        self.pv = _fit([ctx[k] for k in dec], [local_kind(ve[k]) == "yes" for k in dec])
        self.sLk = _fit([ctx[k] + ve_part(ve[k], False) for k in dec], [b1(k) == vis[k] for k in dec])
        self.sL2 = _fit([ctx[k] + ve_part(ve[k], True) for k in dec], [b1(k) == vis[k] for k in dec])
        ck = sorted(k for k in cloud_keys if k in ans)
        cright = {k: bool(ans[k]["target_visible"]) == vis[k] for k in ck}
        self.rC = _fit([ctx[k] for k in ck], [cright[k] for k in ck])
        cdec = [k for k in ck if local_kind(ve[k]) != "unable"]
        self.sCk = _fit([ctx[k] + ve_part(ve[k], False) for k in cdec], [cright[k] for k in cdec])
        self.sC2 = _fit([ctx[k] + ve_part(ve[k], True) for k in cdec], [cright[k] for k in cdec])
        self.rtt_s = float(np.mean([ve[k]["rtt_ms"] for k in ks])) / 1000

    def decide(self, x, vrow, lam, mu, first) -> dict:
        c_paid, c_loc = lam * COST, mu * self.rtt_s
        v_none = _pred(self.r0, x)
        v_cloud = _pred(self.rC, x) - c_paid
        py = _pred(self.pv, x)
        v_local = -c_loc
        for yes, pr in ((1.0, py), (0.0, 1 - py)):
            xk = x + [yes]
            v_local += pr * max(_pred(self.sLk, xk), _pred(self.sCk, xk) - c_paid)
        # Ties go to the cheaper arm (list order); the first frame asks.
        arms = ([] if first else [("none", v_none)]) + [("local", v_local), ("cloud", v_cloud)]
        arm = max(arms, key=lambda a: a[1])[0]
        if arm == "none":
            return {"local": False, "cloud": False}
        if arm == "cloud":
            return {"local": False, "cloud": True}
        if local_kind(vrow) == "unable":
            stay, go = v_none, v_cloud
        else:
            x2 = x + ve_part(vrow, True)
            stay, go = _pred(self.sL2, x2), _pred(self.sC2, x2) - c_paid
        return {"local": True, "cloud": go > stay}


# -- scoring ---------------------------------------------------------------

def bandit_rows(data, ctx, model_for, keys, lam, mu):
    first = {}
    for f in data["frames"]:
        first.setdefault(walk_of(f["key"]), f["key"])
    out = []
    for k in keys:
        w = walk_of(k)
        is_first = first[w] == k
        asked = model_for(w).decide(ctx[k], data["ve"][k], lam, mu, is_first)
        r = voi.frame_score(data, k, asked)
        r["first"] = is_first
        r["right"] = _final_right(data, k, asked)
        out.append(r)
    return out


def _final_right(data, k, asked) -> bool:
    b = data["per"][k]["status"] == "detected"
    if asked["local"]:
        kind = local_kind(data["ve"][k])
        if kind != "unable":
            b = kind == "yes"
    a = data["ans"].get(k)
    if asked["cloud"] and a is not None:
        b = bool(a["target_visible"])
    return b == data["visible"][k]


def sign_test(data, rows, fixed, keys) -> dict:
    """Frames where the bandit and the tree end differently right, and the
    exact two-sided sign test on them (D0: a description, not a bar)."""
    fx = {k: _final_right(data, k, fixed[k]) for k in keys}
    br = {r_k: r["right"] for r_k, r in zip(keys, rows)}
    better = sum(br[k] and not fx[k] for k in keys)
    worse = sum(fx[k] and not br[k] for k in keys)
    n = better + worse
    if n == 0:
        p = 1.0
    else:
        tail = sum(math.comb(n, i) for i in range(0, min(better, worse) + 1)) / 2 ** n
        p = min(1.0, 2 * tail)
    return {"bandit_right_tree_wrong": better, "tree_right_bandit_wrong": worse, "p_two_sided": round(p, 4)}


def criteria(v, f, f0) -> dict:
    """D0's criteria 1-3 for the bandit's totals `v` against the tree `f`;
    3.65's wording against F0 beside it."""
    b = voi.bars(v, f)
    out = {"C1a": b["G1"], "C1b": b["G2"], "C1b_i": b["G2_i"], "C1b_ii": b["G2_ii"],
           "C2": b["G3"], "C2_abandoned": b["G3_abandoned"],
           "C2_walks_checked": sum(1 for c in f["per_walk"].values() if c["useful"] > 0),
           "vs_F0_3_65": {"useful_ge": v["useful"] >= f0["useful"],
                          "paid_30pct_fewer": v["paid"] <= 0.7 * f0["paid"]}}
    return out


def run(data) -> dict:
    data["visible"] = {f["key"]: bool(f["visible"]) for f in data["frames"]}
    ctx = contexts(data)
    smp = voi.samples(data)
    all_keys = [f["key"] for f in data["frames"]]
    train_keys = [k for k in all_keys if walk_of(k) in data["train"]]
    halves = {"train": data["train"], "held_out": data["held_out"]}
    full = {s: Bandit(data, ctx, train_keys, [k for k in train_keys if k in smp[s]]) for s in smp}
    lowo = {}
    for w in sorted(data["train"]):
        ks = [k for k in train_keys if walk_of(k) != w]
        lowo[w] = Bandit(data, ctx, ks, [k for k in ks if k in smp["W"]])

    fixed = {}
    for s, sk in smp.items():
        for h, ws in halves.items():
            keys = [k for k in all_keys if k in sk and walk_of(k) in ws]
            fixed[f"{s}/{h}"] = {arm: voi.total(voi.fixed_rows(data, arm, keys)) for arm in ("F", "F0")}

    out = {"cost_per_call": COST, "lambdas": LAMBDAS, "mus": MUS,
           "features": ["detected", "logit_p", "log1p_streak", f"passed_in_last_{RECENT}",
                        "log1p_frames_since_start", "coco_word"],
           "fixed": fixed, "cells": {}}
    for lam in LAMBDAS:
        for mu in MUS:
            cell = f"lambda={lam},mu={mu}"
            out["cells"][cell] = {}
            for s, sk in smp.items():
                for h, ws in halves.items():
                    keys = [k for k in all_keys if k in sk and walk_of(k) in ws]
                    mf = (lambda w, s=s: full[s]) if h == "held_out" else (lambda w: lowo[w])
                    rows = bandit_rows(data, ctx, mf, keys, lam, mu)
                    v = voi.total(rows)
                    fx = fixed[f"{s}/{h}"]
                    c = criteria(v, fx["F"], fx["F0"])
                    firsts = [r for r in rows if r["first"]]
                    c["C3"] = all(r["local"] or r["paid"] for r in firsts)
                    c["C3_walks_checked"] = len(firsts)
                    c["met"] = c["C1a"] and c["C1b"] and c["C2"] and c["C3"]
                    v["criteria"] = c
                    v["sign_test_vs_tree"] = sign_test(data, rows, data["fixed"]["F"], keys)
                    out["cells"][cell][f"{s}/{h}"] = v

    passing = [(lam, mu) for lam in LAMBDAS for mu in MUS
               if out["cells"][f"lambda={lam},mu={mu}"]["W/train"]["criteria"]["met"]]
    if passing:
        lam, mu = max(passing)
        why = "the passing cell with the highest lambda, then mu (training walks, leave one walk out)"
    else:
        lam, mu = max(((lam, mu) for lam in LAMBDAS for mu in MUS),
                      key=lambda c: (out["cells"][f"lambda={c[0]},mu={c[1]}"]["W/train"]["net"], c))
        why = "no cell passed on the training walks: the highest training net"
    cell = f"lambda={lam},mu={mu}"
    out["judged"] = {"cell": cell, "why": why,
                     "passing_on_train": [f"lambda={a},mu={b}" for a, b in passing]}
    w_ho, p_ho = out["cells"][cell]["W/held_out"], out["cells"][cell]["P/held_out"]
    out["verdict"] = verdict(w_ho["criteria"], p_ho["criteria"])
    out["voi_beside"] = voi_beside()
    return out


def verdict(w: dict, p: dict) -> str:
    """D0's rule."""
    if not (w["C1a"] and w["C2"] and w["C3"] and p["C1a"] and p["C2"]):
        return "failed"
    if w["C1b"]:
        return "met"
    if w["C1b_i"] == "too few to judge":
        return "sample too small to tell"
    return "failed"


def voi_beside() -> dict:
    """G's VOI at its judged cell, from the committed `voi/scores.json`."""
    s = json.loads((HERE / "voi" / "scores.json").read_text())
    return {"cell": s["judged"]["cell"], "verdict": s["verdict"],
            **{sh: s["cells"][s["judged"]["cell"]][sh] for sh in ("W/held_out", "P/held_out")}}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, default=HERE / "bandit-scores.json")
    args = ap.parse_args()
    out = run(voi.load())
    args.out.write_text(json.dumps(voi.finite(out), indent=1, allow_nan=False) + "\n")
    j = out["judged"]["cell"]
    brief = {"judged": out["judged"], "verdict": out["verdict"]}
    for sh in ("W/held_out", "P/held_out", "W/train"):
        brief[sh] = {"bandit": {k: v for k, v in out["cells"][j][sh].items() if k != "per_walk"},
                     **{a: {k: v for k, v in out["fixed"][sh][a].items() if k != "per_walk"}
                        for a in ("F", "F0")}}
    print(json.dumps(brief, indent=1))


if __name__ == "__main__":
    main()
