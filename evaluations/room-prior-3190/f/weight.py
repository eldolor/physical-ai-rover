# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/room-prior-3190/f/weight.py -- 3.190.F: a cloud room prior
weighted more in goal choice. Each candidate's learned score gets
`w x prior(type) / max(rates)` added (`bonus`); `w = 0` is today's choice.
Built on 3.190.A's `arms.py` (F-large greedy, mode R, the prior read from
`T.PRIOR`), with two labellers: `names` (every room's true type known from
the start: a person's names on a saved map) and `stand-in` (A's own). Free:
no call is made; the priors are 3.116.C's cached replies. The bars are the
section file's "F", written before any run.

    python evaluations/room-prior-3190/f/weight.py select OUT_DIR     # 100 selection houses, every w
    python evaluations/room-prior-3190/f/weight.py test OUT_DIR W     # 588 test houses, W against 0
    python evaluations/room-prior-3190/f/weight.py score OUT_DIR      # select.json / score.json
"""
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
A_DIR = os.path.dirname(HERE)
sys.path.insert(0, A_DIR)
import arms                                                   # noqa: E402  (3.190.A)
from arms import A, T, TYPES, arm_prior, paired                # noqa: E402
from features import Labeller                                  # noqa: E402
from brain.goal_policy import FEATURES, ROOM_FEATURES          # noqa: E402

GRID = (0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 1000.0)
LABELLERS = ("names", "stand-in")
PRIOR_COL = len(FEATURES) + ROOM_FEATURES.index("prior")
KEYS = ("outcome", "goals", "metres")

# Per mission, per worker: what the goal choice read and sent first.
_STATE: dict = {}


def bonus(prior_col: np.ndarray, rates: dict, w: float) -> np.ndarray:
    """`w x prior / max(rates)` per candidate; 0 everywhere when the target
    has no rates, no rate above 0, or w is 0 (today's choice, exactly)."""
    top = max((float(v) for v in (rates or {}).values()), default=0.0)
    if w == 0 or top <= 0:
        return np.zeros(len(prior_col), dtype=np.float64)
    return w * np.asarray(prior_col, dtype=np.float64) / top


def top_types(rates: dict) -> set:
    if not rates:
        return set()
    hi = max(float(v) for v in rates.values())
    return {t for t, v in rates.items() if float(v) == hi}


class Names(Labeller):
    """Every room's true type known from the start (no share of its floor
    need be mapped): what a person's names on a saved map give."""

    def known_rooms(self, m: dict) -> set:
        return {n for n, c in self.cells.items() if c and n not in self.drop}


def _labels(kind: str):
    def make(grid, index):
        lab = (Names if kind == "names" else Labeller)(grid, T.CELL)
        _STATE["truth"] = Names(grid, T.CELL)
        return lab
    return make


def _install() -> None:
    """Wrap the harness's room features (to see the pool's goals and the
    target's rates) and its scorer (to add the bonus and see the first
    choice). Idempotent; the pool's initializer."""
    if getattr(T, "_f3190", False):
        return
    rf, sn = T.room_features, T.score_np

    def room_features(m, pose, goals, *, trail, types, prior, **kw):
        _STATE["goals"], _STATE["rates"] = list(goals), prior
        return rf(m, pose, goals, trail=trail, types=types, prior=prior, **kw)

    def score_np(params, kind, X, C=None):
        s = np.asarray(sn(params, kind, X, C), dtype=np.float64)
        s = s + bonus(np.asarray(X)[:, PRIOR_COL], _STATE.get("rates"), _STATE.get("w", 0.0))
        if _STATE.get("first") is None:
            i = int(np.argmax(np.where(np.isfinite(s), s, -1e9)))
            _STATE["first"] = tuple(_STATE["goals"][i])
        return s

    T.room_features, T.score_np, T._f3190 = room_features, score_np, True


def _use_prior(arm: str) -> None:
    for t in arms.PROCTHOR:
        T.PRIOR.pop(t, None)
    T.PRIOR.update(arm_prior(arm))


def _job(args):
    s, h, arm, w, lab = args
    _install()
    _use_prior(arm)
    _STATE.clear()
    _STATE["w"] = w
    spec, drop, _ = A._spec("stand-in")
    try:
        r = T.mission(s, h, "R", spec, 0, drop, labels=_labels(lab))
    except Exception as e:  # noqa: BLE001 -- a house that cannot start is a row
        r = {"split": s, "index": h, "mode": "R", "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST}
    r.pop("choices", None)
    rates = T.PRIOR.get(r.get("target"))
    r.update(arm=arm, w=w, labeller=lab, prior=rates)
    # The first goal against the target's top type(s), on the true rooms; the
    # cloud's rates name the top type for every arm (the question F2 asks is
    # "did it go where the cloud said first").
    cloud = arm_prior("cloud").get(r.get("target"))
    truth, first = _STATE.get("truth"), _STATE.get("first")
    tops = top_types(cloud)
    if truth is not None and tops:
        r["has_top"] = any(T.room_type(n) in tops for n, c in truth.cells.items() if c)
        if first is not None:
            t = truth.types(None, [first], known=set(truth.cells))[0]
            r.update(first_goal=[round(first[0], 3), round(first[1], 3)], first_type=t,
                     first_in_top=t in tops)
    return r


def _head() -> str:
    return subprocess.run(["git", "rev-parse", "--short=8", "HEAD"], cwd=A_DIR,
                          capture_output=True, text=True, check=True).stdout.strip()


def _name(lab: str, arm: str, w: float) -> str:
    return f"{lab}-{arm}-w{w:g}.json"


def _run(out: str, which: str, cells: list) -> None:
    """Resumable per cell, only on the commit the run began on."""
    os.makedirs(out, exist_ok=True)
    meta, head = os.path.join(out, "run.json"), _head()
    if os.path.exists(meta):
        began = json.load(open(meta))["commit"]
        if began != head:
            raise SystemExit(f"{out} began at {began}, HEAD is {head}: start a new OUT_DIR")
    else:
        T.save_atomic(meta, lambda p: json.dump({"commit": head, "houses": which}, open(p, "w")))
    hs = A.houses(which)
    todo = [c for c in cells if not os.path.exists(os.path.join(out, _name(*c)))]
    if not todo:
        return
    with ProcessPoolExecutor(T.WORKERS, initializer=_install) as ex:
        for lab, arm, w in todo:
            rows = list(ex.map(_job, [(s, h, arm, w, lab) for s, h in hs], chunksize=2))
            T.save_atomic(os.path.join(out, _name(lab, arm, w)),
                          lambda p: json.dump(rows, open(p, "w")))
            print(lab, arm, w, T.summary(rows), flush=True)


def cmd_select(out: str) -> None:
    cells = [("names", "none", 0.0)]
    cells += [("names", arm, w) for w in GRID for arm in ("cloud", "wrong")]
    _run(out, "select", cells)


def cmd_test(out: str, w: str) -> None:
    w = float(w)
    if w not in GRID or w == 0:
        raise SystemExit(f"w must be a nonzero grid value {GRID}")
    cells = []
    for v in (0.0, w):
        cells += [("names", "cloud", v), ("names", "wrong", v), ("names", "none", v),
                  ("stand-in", "cloud", v), ("stand-in", "none", v)]
    _run(out, "test", cells)
    T.save_atomic(os.path.join(out, "w.json"), lambda p: json.dump({"w": w}, open(p, "w")))


# ---------- scoring ----------

def load(out: str) -> dict:
    rows = {}
    for f in sorted(os.listdir(out)):
        if re.fullmatch(r"(names|stand-in)-(cloud|wrong|none)-w[0-9.]+\.json", f):
            rs = json.load(open(os.path.join(out, f)))
            r0 = rs[0]
            rows[(r0["labeller"], r0["arm"], r0["w"])] = rs
    for (lab, arm, w), rs in rows.items():
        want = arm_prior(arm)
        for r in rs:
            if (r.get("labeller"), r.get("arm"), r.get("w")) != (lab, arm, w) or (
                    r.get("target") is not None and r.get("prior") != want.get(r["target"])):
                raise SystemExit(f"{lab}/{arm}/{w}: row {r['index']} read another arm -- not scoring")
    return rows


def first_rate(rows: list) -> dict:
    n = [r for r in rows if r["outcome"] != "no_target" and r.get("has_top")]
    k = sum(bool(r.get("first_in_top")) for r in n)
    return {"n": len(n), "in_top": k, "rate": round(k / len(n), 4) if n else None}


def stats(rows: list) -> dict:
    return {**T.summary(rows), "first_goal": first_rate(rows)}


def differ(a: list, b: list) -> int:
    kb = {(r["split"], r["index"]): r for r in b}
    return sum(1 for r in a if r["outcome"] != "no_target"
               and [r.get(k) for k in KEYS] != [kb[(r["split"], r["index"])].get(k) for k in KEYS])


def select(rows: dict) -> dict:
    none = T.summary(rows[("names", "none", 0.0)])
    table, ok = [], []
    for w in GRID:
        c, wr = stats(rows[("names", "cloud", w)]), stats(rows[("names", "wrong", w)])
        guard = (wr["found"] >= 0.98 * none["found"]
                 and wr["median_metres_found"] <= 1.25 * none["median_metres_found"])
        table.append({"w": w, "cloud": c, "wrong": wr, "wrong_guard": guard})
        if guard:
            ok.append((c["mean_reward"], -w, w))
    best = max(ok)[2] if ok else 0.0
    return {"none": none, "grid": table, "w_star": best,
            "stops": best == 0.0}


def score(rows: dict, w: float) -> dict:
    g = lambda lab, arm, v: rows[(lab, arm, v)]                 # noqa: E731
    s = {f"{lab}/{arm}/w{v:g}": stats(rs) for (lab, arm, v), rs in sorted(rows.items())}
    c0, cw = stats(g("names", "cloud", 0.0)), stats(g("names", "cloud", w))
    ww, n0 = stats(g("names", "wrong", w)), T.summary(g("names", "none", 0.0))
    p1 = paired(g("names", "cloud", w), g("names", "cloud", 0.0))
    f1 = {"paired_cloud_w_minus_cloud_0": p1, "metres": cw["median_metres_found"],
          "metres_bar": c0["median_metres_found"]}
    f1["met"] = p1["ci95"][0] > 0 and f1["metres"] <= f1["metres_bar"]
    f2 = {"rate": cw["first_goal"]["rate"], "bar": round(c0["first_goal"]["rate"] + 0.10, 4),
          "w0": c0["first_goal"]}
    f2["met"] = cw["first_goal"]["rate"] >= c0["first_goal"]["rate"] + 0.10
    f3 = {"found": cw["found"], "bar": round(0.99 * c0["found"], 2),
          "met": cw["found"] >= 0.99 * c0["found"]}
    p4 = paired(g("names", "wrong", w), g("names", "none", 0.0))
    f4 = {"found": ww["found"], "found_bar": round(0.98 * n0["found"], 2),
          "metres": ww["median_metres_found"], "metres_bar": round(1.25 * n0["median_metres_found"], 3),
          "paired_wrong_w_minus_none": p4, "paired_bar": -0.05}
    f4["met"] = (ww["found"] >= 0.98 * n0["found"]
                 and ww["median_metres_found"] <= 1.25 * n0["median_metres_found"]
                 and p4["mean"] >= -0.05)
    s0, sw = T.summary(g("stand-in", "cloud", 0.0)), T.summary(g("stand-in", "cloud", w))
    p5 = paired(g("stand-in", "cloud", w), g("stand-in", "cloud", 0.0))
    f5 = {"found": sw["found"], "found_bar": round(0.99 * s0["found"], 2),
          "paired_cloud_w_minus_cloud_0": p5}
    f5["met"] = sw["found"] >= 0.99 * s0["found"] and p5["ci95"][1] >= 0
    f6 = {lab: differ(g(lab, "none", w), g(lab, "none", 0.0)) for lab in LABELLERS}
    f6["met"] = all(v == 0 for v in f6.values())
    bars = {"F1": f1, "F2": f2, "F3": f3, "F4": f4, "F5": f5, "F6": f6}
    return {"model_id": arms.MODEL_ID, "w": w, "summary": s, **bars,
            "every_bar_met": w > 0 and all(b["met"] for b in bars.values()),
            "reported": {
                "paired_cloud_w_minus_none_names": paired(g("names", "cloud", w),
                                                          g("names", "none", 0.0)),
                "paired_wrong_0_minus_none_names": paired(g("names", "wrong", 0.0),
                                                          g("names", "none", 0.0)),
                "by_category_names": arms.by_category(g("names", "cloud", w),
                                                      g("names", "cloud", 0.0))}}


def cmd_score(out: str) -> dict:
    rows = load(out)
    meta = json.load(open(os.path.join(out, "run.json")))
    if meta["houses"] == "select":
        report = {"commit": meta["commit"], **select(rows)}
        path = os.path.join(out, "select.json")
    else:
        w = json.load(open(os.path.join(out, "w.json")))["w"]
        report = {"commit": meta["commit"], **score(rows, w)}
        path = os.path.join(out, "score.json")
    T.save_atomic(path, lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps({k: v for k, v in report.items() if k != "summary"}, indent=1))
    return report


if __name__ == "__main__":
    cmds = {"select": (cmd_select, 3), "test": (cmd_test, 4), "score": (cmd_score, 3)}
    if len(sys.argv) < 2 or sys.argv[1] not in cmds or len(sys.argv) != cmds[sys.argv[1]][1]:
        raise SystemExit(__doc__)
    cmds[sys.argv[1]][0](*sys.argv[2:])
