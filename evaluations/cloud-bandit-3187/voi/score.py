# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/cloud-bandit-3187/voi/score.py

3.187.G: value of information (VOI) over the tier's calibrated confidences,
scored against today's fixed rule (`replay.py`'s `fixed-F.jsonl`, the tree;
`fixed-F0.jsonl`, cloud-only triggers) on the frames that carry a cloud
answer. The bars, the samples and the verdict rule are G0 in
`docs/plans/ros-alignment/3.187-cloud-call-bandit.md`, written first.
Free: reads committed records, calls nothing. Writes `scores.json`.

    python evaluations/cloud-bandit-3187/voi/score.py
"""

import argparse
import importlib.util
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

VOI = Path(__file__).resolve().parent
sys.path.insert(0, str(VOI.parent))
from common import CAL, HERE, SHIPPED_MODEL, jsonl, load_set, local_kind, walk_of  # noqa: E402

# 3.92's logistic fit, loaded by path (its directory has no package).
_spec = importlib.util.spec_from_file_location("tier_calibration_392_score", CAL / "score.py")
cal392 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cal392)
fit_logistic, predict_logistic, logit = cal392.fit_logistic, cal392.predict_logistic, cal392.logit

COST = 0.010645                       # B2's measured mean dollars per paid call
LAMBDAS = (0, 5, 10, 25, 50, 100)     # useful answers per dollar (A2's grid)
MUS = (0, 0.01, 0.05)                 # per local second (A2's grid)
D_MIN = 20                            # amendment 1's "too few to judge"
CATCH, FLAG = 0.50, 0.10              # branch (b)(i)
FEWER_LOCAL = 0.30                    # branch (b)(ii)


# -- data ------------------------------------------------------------------

def load(answers=None, fixed_dir=None):
    d = load_set()
    split = json.loads((HERE / "split.json").read_text())
    per = {r["key"]: r for r in jsonl(CAL / "perception.jsonl")}
    ve = {r["key"]: r for r in jsonl(CAL / "qwen-VE.jsonl")}
    ans = {r["key"]: r for r in jsonl(answers or VOI / "cloud-answers.jsonl")}
    fdir = fixed_dir or VOI
    fixed = {arm: {r["key"]: r for r in jsonl(fdir / f"fixed-{arm}.jsonl")} for arm in ("F", "F0")}
    frames = sorted(d["frames"], key=lambda f: f["key"])
    return {"set": d, "frames": frames, "held_out": set(split["held_out"]),
            "train": set(split["train"]), "per": per, "ve": ve, "ans": ans, "fixed": fixed}


def samples(data) -> dict:
    """G0's two samples: W, every frame with an answer; P, the shipped
    model's (a walk row by `SHIPPED_MODEL`, or a pilot answer)."""
    ans = data["ans"]
    w = {k for k in ans}
    p = {k for k, a in ans.items() if a.get("source") == "pilot" or a.get("model_id") == SHIPPED_MODEL}
    return {"W": w, "P": p}


# -- features and calibrations (labels only) --------------------------------

def gate_x(prow) -> list:
    det = prow["status"] == "detected"
    return [float(det), float(logit(prow["p"])) if det else 0.0]


def ve_x(prow, vrow) -> list:
    det = float(prow["status"] == "detected")
    kind = local_kind(vrow)
    vy, vn = float(kind == "yes"), float(kind == "no")
    ls = float(logit(vrow["score"])) if isinstance(vrow.get("score"), (int, float)) else 0.0
    return gate_x(prow) + [vy, vn, vy * ls, vn * ls, det * vy, det * vn]


def kind_x(prow, yes: bool) -> list:
    det = float(prow["status"] == "detected")
    return gate_x(prow) + [float(yes), det * float(yes)]


class Model:
    """m0, mv, m1, m1k and the cloud's two rates, fitted on `keys` only."""

    def __init__(self, data, label_keys, cloud_keys):
        per, ve, vis = data["per"], data["ve"], data["visible"]
        ks = sorted(label_keys)
        y = np.array([vis[k] for k in ks], float)
        self.m0 = fit_logistic(np.array([gate_x(per[k]) for k in ks]), y)
        dec = [k for k in ks if local_kind(ve[k]) != "unable"]
        yd = np.array([vis[k] for k in dec], float)
        self.mv = fit_logistic(np.array([gate_x(per[k]) for k in dec]),
                               np.array([local_kind(ve[k]) == "yes" for k in dec], float))
        self.m1 = fit_logistic(np.array([ve_x(per[k], ve[k]) for k in dec]), yd)
        self.m1k = fit_logistic(np.array([kind_x(per[k], local_kind(ve[k]) == "yes") for k in dec]), yd)
        ans = data["ans"]
        ck = [k for k in cloud_keys if k in ans]
        pos = [k for k in ck if vis[k]]
        neg = [k for k in ck if not vis[k]]
        # Laplace-smoothed, so a half with no error is not read as perfect.
        self.tpr = (sum(ans[k]["target_visible"] for k in pos) + 1) / (len(pos) + 2)
        self.tnr = (sum(not ans[k]["target_visible"] for k in neg) + 1) / (len(neg) + 2)
        self.rtt_s = float(np.mean([ve[k]["rtt_ms"] for k in ks])) / 1000

    @staticmethod
    def _p(w, x):
        return float(predict_logistic(w, np.array([x]))[0])

    def q0(self, prow):
        return self._p(self.m0, gate_x(prow))

    def q1(self, prow, vrow):
        return self._p(self.m1, ve_x(prow, vrow))

    def pv(self, prow):
        return self._p(self.mv, gate_x(prow))

    def q1k(self, prow, yes):
        return self._p(self.m1k, kind_x(prow, yes))

    def cloud_right(self, q):
        return q * self.tpr + (1 - q) * self.tnr


def right(belief: bool, q: float) -> float:
    return q if belief else 1 - q


def voi_decide(m: Model, prow, vrow, lam: float, mu: float, first: bool) -> dict:
    """One frame's two choices (G0). Returns what was asked."""
    c_paid, c_loc = lam * COST, mu * m.rtt_s
    b0 = prow["status"] == "detected"
    q0 = m.q0(prow)
    v_none = right(b0, q0)
    v_cloud = m.cloud_right(q0) - c_paid
    py = m.pv(prow)
    v_local = -c_loc
    for yes, pr in ((True, py), (False, 1 - py)):
        q = m.q1k(prow, yes)
        v_local += pr * max(right(yes, q), m.cloud_right(q) - c_paid)
    # Ties go to the cheaper arm; the first frame always asks (amendment 3).
    arms = ([] if first else [("none", v_none)]) + [("local", v_local), ("cloud", v_cloud)]
    arm = max(arms, key=lambda a: a[1])[0]
    if arm == "none":
        return {"local": False, "cloud": False}
    if arm == "cloud":
        return {"local": False, "cloud": True}
    kind = local_kind(vrow)
    if kind == "unable":
        b, q = b0, q0
    else:
        b, q = kind == "yes", m.q1(prow, vrow)
    return {"local": True, "cloud": m.cloud_right(q) - c_paid > right(b, q)}


# -- scoring ---------------------------------------------------------------

def frame_score(data, k, asked) -> dict:
    """A2's accounting of one frame, given what was asked on it."""
    prow, vrow, a = data["per"][k], data["ve"][k], data["ans"].get(k)
    vis = data["visible"][k]
    b0 = prow["status"] == "detected"
    b = b0
    if asked["local"]:
        kind = local_kind(vrow)
        if kind != "unable":
            b = kind == "yes"
    if asked["cloud"] and a is not None:
        b = bool(a["target_visible"])
    ve_kind = local_kind(vrow)
    in_d = (ve_kind != "unable" and (ve_kind == "yes") != vis
            and a is not None and bool(a["target_visible"]) == vis)
    return {"useful": b == vis and b0 != vis, "harmful": b != vis and b0 == vis,
            "paid": bool(asked["cloud"]), "local": bool(asked["local"]),
            "local_s": vrow["rtt_ms"] / 1000 if asked["local"] else 0.0,
            "d": in_d, "d_right": in_d and b == vis, "walk": walk_of(k)}


def total(rows) -> dict:
    t = defaultdict(float)
    per_walk = defaultdict(lambda: defaultdict(float))
    for r in rows:
        for f in ("useful", "harmful", "paid", "local", "local_s", "d", "d_right"):
            t[f] += r[f]
            per_walk[r["walk"]][f] += r[f]
    out = {f: (round(t[f], 1) if f == "local_s" else int(t[f]))
           for f in ("useful", "harmful", "paid", "local", "local_s", "d", "d_right")}
    out["frames"] = len(rows)
    out["net"] = out["useful"] - out["harmful"]
    out["dollars"] = round(out["paid"] * COST, 4)
    out["per_walk"] = {w: {f: (round(v, 1) if f == "local_s" else int(v)) for f, v in c.items()}
                       | {"dollars": round(c["paid"] * COST, 4)}
                       for w, c in sorted(per_walk.items())}
    return out


def bars(v: dict, f: dict) -> dict:
    """G1-G3 for VOI's totals `v` against the tree's `f` on the same frames."""
    g1 = v["net"] >= f["net"]
    if f["d"] < D_MIN:
        b1 = None   # too few to judge
    else:
        b1 = v["d_right"] >= CATCH * v["d"] and v["paid"] <= FLAG * f["local"]
    b2 = v["local"] <= (1 - FEWER_LOCAL) * f["local"] and v["paid"] <= f["paid"]
    g2 = bool(b1) or b2
    helped = [w for w, c in f["per_walk"].items() if c["useful"] > 0]
    abandoned = [w for w in helped if v["per_walk"].get(w, {}).get("useful", 0) == 0]
    return {"G1": g1, "G2": g2, "G2_i": "too few to judge" if b1 is None else b1, "G2_ii": b2,
            "G3": not abandoned, "G3_abandoned": abandoned,
            "met": g1 and g2 and not abandoned}


def fixed_rows(data, arm, keys):
    fx = data["fixed"][arm]
    return [frame_score(data, k, fx[k]) for k in keys]


def voi_rows(data, model_for, keys, lam, mu):
    """`model_for(walk)` gives the model that scores that walk."""
    first = {}
    for f in data["frames"]:
        first.setdefault(walk_of(f["key"]), f["key"])
    out = []
    for k in keys:
        m = model_for(walk_of(k))
        is_first = first[walk_of(k)] == k
        asked = voi_decide(m, data["per"][k], data["ve"][k], lam, mu, is_first)
        r = frame_score(data, k, asked)
        r["first"] = is_first
        out.append(r)
    return out


def run(data) -> dict:
    data["visible"] = {f["key"]: bool(f["visible"]) for f in data["frames"]}
    smp = samples(data)
    all_keys = [f["key"] for f in data["frames"]]
    train_keys = [k for k in all_keys if walk_of(k) in data["train"]]
    halves = {"train": data["train"], "held_out": data["held_out"]}
    out = {"cost_per_call": COST, "lambdas": LAMBDAS, "mus": MUS,
           "samples": {}, "cells": {}, "fixed": {}}

    # Leave one training walk out (cell choice, W only), and the full fit.
    # Each sample's held-out rule uses its own answers' cloud rates (P's are
    # the shipped model's); P/train is scored with the W-fitted folds and
    # is reported, never judged (G1 records both readings).
    full = {s: Model(data, train_keys, [k for k in train_keys if k in smp[s]]) for s in smp}
    lowo = {}
    for w in sorted(data["train"]):
        ks = [k for k in train_keys if walk_of(k) != w]
        lowo[w] = Model(data, ks, [k for k in ks if k in smp["W"]])

    for s, sk in smp.items():
        out["samples"][s] = {
            "frames": len(sk),
            **{h: sum(walk_of(k) in ws for k in sk) for h, ws in halves.items()},
            "cloud_tpr_train": round(full[s].tpr, 4), "cloud_tnr_train": round(full[s].tnr, 4)}
        for h, ws in halves.items():
            keys = [k for k in all_keys if k in sk and walk_of(k) in ws]
            out["fixed"][f"{s}/{h}"] = {arm: total(fixed_rows(data, arm, keys)) for arm in ("F", "F0")}

    # Every cell, on both halves of both samples (held out: the full fit;
    # train: leave one walk out).
    for lam in LAMBDAS:
        for mu in MUS:
            cell = f"lambda={lam},mu={mu}"
            out["cells"][cell] = {}
            for s, sk in smp.items():
                for h, ws in halves.items():
                    keys = [k for k in all_keys if k in sk and walk_of(k) in ws]
                    mf = (lambda w, s=s: full[s]) if h == "held_out" else (lambda w: lowo[w])
                    rows = voi_rows(data, mf, keys, lam, mu)
                    v = total(rows)
                    v["bars"] = bars(v, out["fixed"][f"{s}/{h}"]["F"])
                    # G4: each walk's first frame asks at least the local
                    # tier (by construction; checked). A walk whose first
                    # frame is not in the sample has nothing to check.
                    firsts = [r for r in rows if r["first"]]
                    v["bars"]["G4"] = all(r["local"] or r["paid"] for r in firsts)
                    v["bars"]["G4_walks_checked"] = len(firsts)
                    out["cells"][cell][f"{s}/{h}"] = v

    # The judged cell: chosen on W's training walks only (G0).
    passing = [(lam, mu) for lam in LAMBDAS for mu in MUS
               if out["cells"][f"lambda={lam},mu={mu}"]["W/train"]["bars"]["met"]]
    if passing:
        lam, mu = max(passing)
        why = "the passing cell with the highest lambda, then mu (training walks, leave one walk out)"
    else:
        lam, mu = max(((lam, mu) for lam in LAMBDAS for mu in MUS),
                      key=lambda c: (out["cells"][f"lambda={c[0]},mu={c[1]}"]["W/train"]["net"], c))
        why = "no cell passed on the training walks: the highest training net"
    cell = f"lambda={lam},mu={mu}"
    out["judged"] = {"cell": cell, "why": why, "passing_on_train": [f"lambda={a},mu={b}" for a, b in passing]}
    w_ho, p_ho = out["cells"][cell]["W/held_out"], out["cells"][cell]["P/held_out"]
    out["verdict"] = verdict(w_ho["bars"], p_ho["bars"], out["fixed"]["W/held_out"]["F"]["d"])
    out["cloud_accuracy"] = cloud_accuracy(data, smp)
    return out


def verdict(w: dict, p: dict, w_d: int) -> str:
    """G0's rule. A failed G1 or G3 (W or P) is "C needed" whatever the
    sample size; then a G2 that can be neither met nor failed is "too
    small"; then G2 decides."""
    if not (w["G1"] and w["G3"] and p["G1"] and p["G3"]):
        return "C needed"
    if w_d < D_MIN and not w["G2_ii"]:
        return "sample too small to tell"
    return "VOI enough: C is not needed" if w["G2"] else "C needed"


def cloud_accuracy(data, smp) -> dict:
    """The answers against the labels, by kind (G0's report)."""
    ans, vis = data["ans"], data["visible"]
    out = {}
    groups = {"P (shipped model)": smp["P"], "W minus P (September, unrecorded model)": smp["W"] - smp["P"]}
    for name, ks in groups.items():
        ks = [k for k in ks if k in ans]
        out[name] = {"frames": len(ks),
                     "right": sum(bool(ans[k]["target_visible"]) == vis[k] for k in ks)}
    both = [k for k, a in ans.items() if a.get("source") == "pilot" and "walk_target_visible" in a]
    out["pilot frames with a walk answer too"] = {
        "frames": len(both),
        "agree": sum(bool(ans[k]["target_visible"]) == bool(ans[k]["walk_target_visible"]) for k in both)}
    return out


def finite(x):
    """`x` with every non-finite float as None (strict JSON)."""
    if isinstance(x, dict):
        return {k: finite(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [finite(v) for v in x]
    if isinstance(x, float) and not math.isfinite(x):
        return None
    return x


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--answers", type=Path, default=None)
    ap.add_argument("--fixed-dir", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=VOI / "scores.json")
    args = ap.parse_args()
    out = run(load(args.answers, args.fixed_dir))
    args.out.write_text(json.dumps(finite(out), indent=1, allow_nan=False) + "\n")
    j = out["judged"]["cell"]
    brief = {"judged": out["judged"], "verdict": out["verdict"], "samples": out["samples"],
             "cloud_accuracy": out["cloud_accuracy"]}
    for sh in ("W/held_out", "P/held_out"):
        brief[sh] = {"VOI": {k: v for k, v in out["cells"][j][sh].items() if k != "per_walk"},
                     **{a: {k: v for k, v in out["fixed"][sh][a].items() if k != "per_walk"}
                        for a in ("F", "F0")}}
    print(json.dumps(brief, indent=1))


if __name__ == "__main__":
    main()
