# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/visual-promise-3177/score.py

3.177.A: criteria 1-3 from `set.json` and `clip-cpu.jsonl`. Every AUROC is
within a walk (a positive is compared only with a negative of its own walk);
"pooled" counts every within-walk pair together. numpy only. Writes
`scores.json` and prints a summary. `--partial` scores whatever rows exist
(a smoke test; criterion 1 then reads unmet).

    .venv/bin/python evaluations/visual-promise-3177/score.py [--partial]
"""

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import HERE, load_set, obj, texts   # noqa: E402

K, KS = 5, (3, 5, 10)
BAR_2A, BAR_2C, CONTROL_BAND, CONTROL_GAP = 0.65, 0.55, (0.43, 0.57), 0.10
BOOT, SEED = 2000, 0
ONSET_MAX = 15


def labels(visible, k):
    """(positives, negatives): out of view with the target in view within the
    next k frames; out of view with none within k on either side."""
    v = np.asarray(visible, bool)
    n = len(v)
    pos = [i for i in range(n) if not v[i] and v[i + 1:i + k + 1].any()]
    neg = [i for i in range(n) if not v[i] and not v[max(0, i - k):i + k + 1].any()]
    return pos, neg


def pairs(score, pos, neg):
    """(positive-over-negative wins, ties half; pairs)."""
    if not pos or not neg:
        return 0.0, 0
    p = np.asarray(score, float)[pos][:, None]
    q = np.asarray(score, float)[neg][None, :]
    return float((p > q).sum() + 0.5 * (p == q).sum()), len(pos) * len(neg)


def ratio(wins_n):
    w, n = wins_n
    return w / n if n else None


def add(a, b):
    return a[0] + b[0], a[1] + b[1]


def build_walks(d, rows):
    """[{walk, object, visible, scores: {variant: (frames, objects)}}] for walks with every row."""
    by_walk = {}
    for r in d["frames"]:
        by_walk.setdefault(r["key"].split("/")[0], []).append(r)
    out = []
    for walk, frames in by_walk.items():
        if not all(f["key"] in rows for f in frames):
            continue
        rs = [rows[f["key"]] for f in frames]
        full = np.array([x["full"] for x in rs])
        tiles = np.array([x["tiles"] for x in rs]).max(axis=1)
        out.append({"walk": walk, "object": obj(frames[0]["target"]),
                    "visible": [f["visible"] for f in frames],
                    "objects": rs[0]["objects"], "full": full, "tiles": tiles})
    return out


def variant_scores(w, variant):
    """{name: per-frame score}: the walk's own words, each other object, the margin."""
    m = w[variant]
    i = w["objects"].index(w["object"])
    others = [j for j in range(len(w["objects"])) if j != i]
    return {"target": m[:, i], "others": {w["objects"][j]: m[:, j] for j in others},
            "margin": m[:, i] - m[:, others].mean(axis=1)}


def evaluate(walks, variant, k):
    per_walk, per_obj, ctrl = {}, {}, {}
    for w in walks:
        pos, neg = labels(w["visible"], k)
        if not pos or not neg:
            continue
        s = variant_scores(w, variant)
        t, m = pairs(s["target"], pos, neg), pairs(s["margin"], pos, neg)
        c = {o: pairs(v, pos, neg) for o, v in s["others"].items()}
        per_walk[w["walk"]] = {"object": w["object"], "pos": len(pos), "neg": len(neg),
                               "target": ratio(t), "margin": ratio(m),
                               "control": float(np.mean([ratio(x) for x in c.values()])),
                               "_t": t, "_m": m, "_c": c}
        o = per_obj.setdefault(w["object"], {"t": (0.0, 0), "m": (0.0, 0), "pos": 0, "neg": 0, "walks": 0})
        o["t"], o["m"] = add(o["t"], t), add(o["m"], m)
        o["pos"] += len(pos); o["neg"] += len(neg); o["walks"] += 1
        oc = ctrl.setdefault(w["object"], {})
        for name, x in c.items():
            oc[name] = add(oc.get(name, (0.0, 0)), x)
    objects = {o: {"walks": v["walks"], "pos": v["pos"], "neg": v["neg"],
                   "target": ratio(v["t"]), "margin": ratio(v["m"]),
                   "control": float(np.mean([ratio(x) for x in ctrl[o].values()]))}
               for o, v in sorted(per_obj.items())}
    pooled_t = ratio(sum_pairs(per_walk, "_t"))
    out = {
        "k": k, "variant": variant,
        "walks": len(per_walk), "pos": sum(w["pos"] for w in per_walk.values()),
        "neg": sum(w["neg"] for w in per_walk.values()),
        "balanced_target": mean_of(objects, "target"),
        "balanced_margin": mean_of(objects, "margin"),
        "balanced_control": mean_of(objects, "control"),
        "pooled_target": pooled_t, "pooled_margin": ratio(sum_pairs(per_walk, "_m")),
        "ci_target": boot(per_walk, "_t"), "ci_margin": boot(per_walk, "_m"),
        "per_object": objects,
        "per_walk": {k_: {kk: vv for kk, vv in v.items() if not kk.startswith("_")}
                     for k_, v in per_walk.items()},
    }
    return out


def sum_pairs(per_walk, key):
    tot = (0.0, 0)
    for w in per_walk.values():
        tot = add(tot, w[key])
    return tot


def mean_of(objects, key):
    vals = [v[key] for v in objects.values() if v[key] is not None]
    return float(np.mean(vals)) if vals else None


def boot(per_walk, key):
    """95% interval of the pooled AUROC, resampling walks with replacement."""
    ws = [w[key] for w in per_walk.values()]
    if not ws:
        return None
    rng = np.random.default_rng(SEED)
    vals = []
    for _ in range(BOOT):
        tot = (0.0, 0)
        for i in rng.integers(0, len(ws), len(ws)):
            tot = add(tot, ws[i])
        vals.append(tot[0] / tot[1])
    return [float(np.percentile(vals, 2.5)), float(np.percentile(vals, 97.5))]


def onset_curve(walks):
    """Mean within-walk z-score of the target's score by frames to the next
    in-view frame (1..ONSET_MAX), and over the far frames (k = K)."""
    by = {j: [] for j in range(1, ONSET_MAX + 1)}
    far = []
    for w in walks:
        pos, neg = labels(w["visible"], K)
        if not pos or not neg:
            continue
        s = variant_scores(w, "full")["target"]
        out = ~np.asarray(w["visible"], bool)
        z = (s - s[out].mean()) / (s[out].std() or 1.0)
        v = w["visible"]
        for i in range(len(v)):
            if v[i]:
                continue
            nxt = next((j for j in range(i + 1, len(v)) if v[j]), None)
            if nxt is not None and nxt - i <= ONSET_MAX:
                by[nxt - i].append(z[i])
        far += [z[i] for i in neg]
    return {"by_frames_to_onset": {j: [round(float(np.mean(x)), 3), len(x)] if x else None
                                   for j, x in by.items()},
            "far": [round(float(np.mean(far)), 3), len(far)] if far else None}


def main():
    partial = "--partial" in sys.argv
    d = load_set()
    path = HERE / "clip-cpu.jsonl"
    rows = {}
    for line in path.read_text().splitlines() if path.exists() else []:
        if line.strip():
            r = json.loads(line)
            rows[r["key"]] = r
    devices = sorted({r["device"] for r in rows.values()})
    if devices and devices != ["cpu"]:
        raise SystemExit(f"rows from {devices}: P24 forbids mixing devices")
    c1 = {"hash": d["hash"], "frames": d["n"], "rows": len(rows), "devices": devices,
          "met": len(rows) == d["n"] and set(rows) == {r["key"] for r in d["frames"]} and devices == ["cpu"]}
    if not c1["met"] and not partial:
        raise SystemExit(f"criterion 1 unmet: {c1}")
    walks = build_walks(d, rows)
    main_ = evaluate(walks, "full", K)
    objs = main_["per_object"]
    c2 = {"2a": main_["balanced_target"], "2b_ci": main_["ci_target"],
          "2c_floor": min((v["target"] for v in objs.values()), default=None)}
    c2["met_2a"] = c2["2a"] is not None and c2["2a"] >= BAR_2A
    c2["met_2b"] = c2["2b_ci"] is not None and c2["2b_ci"][0] > 0.5
    c2["met_2c"] = c2["2c_floor"] is not None and c2["2c_floor"] >= BAR_2C
    ctl = main_["balanced_control"]
    c3 = {"control": ctl, "gap": None if ctl is None or c2["2a"] is None else c2["2a"] - ctl}
    c3["met"] = (ctl is not None and CONTROL_BAND[0] <= ctl <= CONTROL_BAND[1]
                 and c3["gap"] >= CONTROL_GAP)
    ms = [r["ms"] for r in rows.values()]
    s = {
        "criterion_1": c1, "criterion_2": c2, "criterion_3": c3,
        "met": c1["met"] and c2["met_2a"] and c2["met_2b"] and c2["met_2c"] and c3["met"],
        "main": main_,
        "recorded": {
            "k": {k: evaluate(walks, "full", k) for k in KS if k != K},
            "tiles": evaluate(walks, "tiles", K),
            "onset": onset_curve(walks),
            "cpu_ms_full_frame": ({"p50": float(np.median(ms)), "p90": float(np.percentile(ms, 90))}
                                  if ms else None),
            "texts": texts(d),
        },
    }
    out = HERE / ("scores-partial.json" if partial else "scores.json")
    out.write_text(json.dumps(s, indent=1, default=str) + "\n")
    print(json.dumps({"criterion_1": c1, "criterion_2": c2, "criterion_3": c3, "met": s["met"],
                      "per_object": objs}, indent=1, default=str))


if __name__ == "__main__":
    main()
