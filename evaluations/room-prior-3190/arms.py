# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/room-prior-3190/arms.py -- 3.190.A, criteria 1-3: the cloud
room prior (3.116.C's cached replies) against no prior and against a wrong
prior. F-large with the stand-in labeller, mode R, the 588 test houses,
three arms on one commit (amendment 1). Free: no call is made.

    python evaluations/room-prior-3190/arms.py run OUT_DIR     # resumable, per arm
    python evaluations/room-prior-3190/arms.py score OUT_DIR
"""
import json
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "evaluations", "goal-learn-396"))
sys.path.insert(0, os.path.join(ROOT, "evaluations", "room-prior-3116"))
import ask as A                                               # noqa: E402
import train as T                                             # noqa: E402
from llm_prior import PROCTHOR, TYPES, _load                  # noqa: E402

# The model whose cached replies were measured -- a fixed record, never the
# service's live default (`ROOM_MODEL_ID` follows an env var; review).
MODEL_ID = "us.anthropic.claude-opus-5-5"

ARMS = ("cloud", "none", "wrong")
DRAWS = 10_000
C9_LLM = os.path.join(ROOT, "evaluations", "room-prior-3116", "c9", "llm-R.json")


def cloud_prior() -> dict:
    got = _load()[MODEL_ID]
    out = {}
    for t in PROCTHOR:
        p = (got.get(t) or {}).get("prior")
        if p is None:
            raise SystemExit(f"no parsed cloud prior for {t!r}")
        out[t] = dict(p)
    return out


def swap(p: dict) -> dict:
    """The highest and lowest types swapped; ties go to the first in TYPES
    order (3.116's scorer's rule, amendment 1)."""
    hi = max(TYPES, key=lambda k: p[k])
    lo = min(TYPES, key=lambda k: p[k])
    w = dict(p)
    w[hi], w[lo] = p[lo], p[hi]
    return w


def arm_prior(arm: str) -> dict:
    """{category: rates} the arm puts in T.PRIOR; `none` puts nothing."""
    if arm == "none":
        return {}
    c = cloud_prior()
    if arm == "cloud":
        return c
    if arm == "wrong":
        return {t: swap(p) for t, p in c.items()}
    raise ValueError(f"unknown arm {arm!r}")


def _use(arm: str) -> None:
    """The pool's initializer: each worker sets its own prior (spawn-safe)."""
    for t in PROCTHOR:
        T.PRIOR.pop(t, None)
    T.PRIOR.update(arm_prior(arm))


def _job(args):
    s, h, arm = args
    spec, drop, labels = A._spec("stand-in")
    try:
        r = T.mission(s, h, "R", spec, 0, drop, labels=labels)
    except Exception as e:  # noqa: BLE001 -- a house that cannot start is a row, not a lost run
        r = {"split": s, "index": h, "mode": "R", "outcome": "exception",
             "error": f"{type(e).__name__}: {e}", "goals": 0, "metres": 0.0,
             "reward": -T.CRASH_COST}
    r.pop("choices", None)
    # Which prior this row read, so a file can never pass for another arm.
    r.update(arm=arm, prior=T.PRIOR.get(r.get("target")))
    return r


def _head() -> str:
    return subprocess.run(["git", "rev-parse", "--short=8", "HEAD"], cwd=ROOT,
                          capture_output=True, text=True, check=True).stdout.strip()


def cmd_run(out: str) -> None:
    """Resumable per arm, but only on the commit the run began on: the
    arms must share one commit (amendment 1; review)."""
    os.makedirs(out, exist_ok=True)
    meta, head = os.path.join(out, "run.json"), _head()
    if os.path.exists(meta):
        began = json.load(open(meta))["commit"]
        if began != head:
            raise SystemExit(f"{out} began at {began}, HEAD is {head}: start a new OUT_DIR")
    else:
        T.save_atomic(meta, lambda p: json.dump({"commit": head}, open(p, "w")))
    hs = A.houses("test")
    for arm in ARMS:
        path = os.path.join(out, f"{arm}-R.json")
        if os.path.exists(path):
            continue
        with ProcessPoolExecutor(T.WORKERS, initializer=_use, initargs=(arm,)) as ex:
            rows = list(ex.map(_job, [(s, h, arm) for s, h in hs], chunksize=2))
        T.save_atomic(path, lambda p: json.dump(rows, open(p, "w")))
        print(arm, T.summary(rows), flush=True)


def paired(a: list, b: list, draws: int = DRAWS, seed: int = 0) -> dict:
    """Mean per-house reward a - b over houses with a target in both, and
    its 95% bootstrap interval."""
    ka = {(r["split"], r["index"]): r for r in a if r["outcome"] != "no_target"}
    kb = {(r["split"], r["index"]): r for r in b if r["outcome"] != "no_target"}
    keys = sorted(set(ka) & set(kb))
    if set(ka) != set(kb):
        raise SystemExit("the arms ran different houses -- not scoring")
    d = np.asarray([ka[k]["reward"] - kb[k]["reward"] for k in keys], dtype=float)
    rng = np.random.default_rng(seed)
    means = d[rng.integers(0, len(d), size=(draws, len(d)))].mean(axis=1)
    lo, hi = np.percentile(means, [2.5, 97.5])
    return {"n": len(d), "mean": round(float(d.mean()), 4),
            "ci95": [round(float(lo), 4), round(float(hi), 4)]}


def check(rows: dict) -> None:
    for arm, rs in rows.items():
        want = arm_prior(arm)
        for r in rs:
            t = r.get("target")
            if t is None:
                continue
            if r.get("arm") != arm or r.get("prior") != want.get(t):
                raise SystemExit(f"{arm} row {r['index']} read another prior -- not scoring")


def by_category(a: list, b: list) -> dict:
    out = {}
    kb = {(r["split"], r["index"]): r for r in b}
    for t in PROCTHOR:
        d = [r["reward"] - kb[(r["split"], r["index"])]["reward"] for r in a if r.get("target") == t]
        out[t] = {"n": len(d), "mean": round(float(np.mean(d)), 4) if d else None}
    return out


def differ_c9(rows: list):
    if not os.path.exists(C9_LLM):
        return None
    old = {(r["split"], r["index"]): r for r in json.load(open(C9_LLM))}
    f = ("outcome", "goals", "metres")
    n = [r["index"] for r in rows if r["outcome"] != "no_target"
         and [r.get(k) for k in f] != [old[(r["split"], r["index"])].get(k) for k in f]]
    return {"differ": len(n), "of": sum(r["outcome"] != "no_target" for r in rows),
            "houses": n[:20]}


def score(rows: dict) -> dict:
    check(rows)
    s = {a: T.summary(rs) for a, rs in rows.items()}
    c, n, w = s["cloud"], s["none"], s["wrong"]
    p_cn = paired(rows["cloud"], rows["none"])
    p_wn = paired(rows["wrong"], rows["none"])
    c1 = {"paired_cloud_minus_none": p_cn, "goals": c["median_goals_found"],
          "goals_bar": n["median_goals_found"]}
    c1["met"] = p_cn["ci95"][0] > 0 and c1["goals"] <= c1["goals_bar"]
    # Verdicts against the unrounded bars; only the reported bar is rounded (review).
    f2, f3, m3 = 0.99 * n["found"], 0.98 * n["found"], 1.25 * n["median_metres_found"]
    c2 = {"found": c["found"], "found_bar": round(f2, 2), "met": c["found"] >= f2}
    c3 = {"found": w["found"], "found_bar": round(f3, 2),
          "metres": w["median_metres_found"], "metres_bar": round(m3, 3),
          "met": w["found"] >= f3 and w["median_metres_found"] <= m3}
    return {"model_id": MODEL_ID, "summary": s, "criterion_1": c1, "criterion_2": c2,
            "criterion_3": c3,
            "reported": {"paired_wrong_minus_none": p_wn,
                         "by_category_cloud_minus_none": by_category(rows["cloud"], rows["none"]),
                         "by_category_wrong_minus_none": by_category(rows["wrong"], rows["none"]),
                         "cloud_vs_3116_c9": differ_c9(rows["cloud"])}}


def cmd_score(out: str) -> dict:
    rows = {a: json.load(open(os.path.join(out, f"{a}-R.json"))) for a in ARMS}
    report = score(rows)
    T.save_atomic(os.path.join(out, "score.json"),
                  lambda p: json.dump(report, open(p, "w"), indent=1))
    print(json.dumps(report, indent=1))
    return report


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] not in ("run", "score"):
        raise SystemExit(__doc__)
    {"run": cmd_run, "score": cmd_score}[sys.argv[1]](sys.argv[2])
