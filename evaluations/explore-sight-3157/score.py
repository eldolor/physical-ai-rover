# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.157.B: what explore's sightings would be on the labelled rig walks,
scored on bars B1-B5 (docs/plans/ros-alignment/3.157-perception-into-explore.md,
"3.157.A", written before this ran).

Nothing is re-run: each frame's records are 3.92's own (perception.jsonl,
qwen-VE.jsonl), put in the scene shape the tiered policy hands explore and
read by `ExploreAgent._hits` itself, under `sight_from: perception`. Free.

    PICAR_RECORDINGS=<dir holding the 21 labelled walks> \\
      .venv/bin/python evaluations/explore-sight-3157/score.py
"""
import json
import math
import sys
import types
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "evaluations" / "tier-calibration-392"))

import common                                                  # noqa: E402  (3.92's frame order)
from brain.explore import ExploreAgent                         # noqa: E402
from brain.local_vlm import model_size                         # noqa: E402
from brain.perceive import DETECTED                            # noqa: E402
from brain.tiered import DEFAULT_HFOV_DEG, LOCAL_YES_AT, _local_kind  # noqa: E402

REC = ROOT / "evaluations" / "tier-calibration-392"
HFOV = DEFAULT_HFOV_DEG                     # 66: perceive.py's default, the bars' unit
NO_BELOW = 0.1                              # tier_local_no_below (3.92)
KS = (3, 5)
BARS = {"B1": 0.70, "B2k3": 0.90, "B2k5": 0.95, "B3p50": 3.0, "B3p90": 6.0,
        "B4frame": 0.02, "B4k5": 0.01, "B5": 19, "B5deg": 6.0}


def load(name):
    return {r["key"]: r for r in map(json.loads, (REC / name).read_text().splitlines()) if r}


def perception_scene(p):
    """`_perception` as the tiered policy reports it: the pipeline's bearing,
    `(box centre / width - 0.5) * hfov + pan` (perceive.py), pan 0 on the rig."""
    if p["status"] != "detected" or not p.get("box"):
        return {"status": p["status"]}
    x1, _, x2, _ = p["box"]
    return {"status": DETECTED, "bearing_deg": ((x1 + x2) / 2 / p["w"] - 0.5) * HFOV,
            "pan_deg": 0.0, "p": p["p"]}


def local_landed(q, w, h):
    """`_tier.local_landed` for VE's answer on this frame: a yes with a box
    carries its body bearing as `TieredVision._bearing_of` reads it (box
    fractions of `model_size`, pan 0, no turn since)."""
    kind = _local_kind(q, NO_BELOW, LOCAL_YES_AT)
    out = {"answer": kind if kind in ("yes", "no") else "unable"}
    if kind == "yes" and q.get("box"):
        mw, mh = model_size(w, h)
        x1, _, x2, _ = (min(1.0, max(0.0, float(v) / d)) for v, d in zip(q["box"], (mw, mh, mw, mh)))
        out["bearing_deg"] = ((x1 + x2) / 2 - 0.5) * HFOV
    return out


def hits(scene, target):
    """explore's own `_hits` on this scene (sight_from: perception)."""
    me = types.SimpleNamespace(memory=types.SimpleNamespace(target_object=target),
                               sight_from="perception", _scene=scene,
                               _sighted_by_local=False, _local_yes_at=None,
                               clock=lambda: 0.0, _pan=lambda: 0.0)
    return ExploreAgent._hits(me, {"detections": []})


def pct(xs, q):
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, math.ceil(q / 100 * len(s)) - 1))] if s else None


def main():
    P, Q = load("perception.jsonl"), load("qwen-VE.jsonl")
    labels = {r["key"]: r for r in json.loads((HERE / "labels-bearing.json").read_text())["rows"]}
    frames = []
    for key, _, target, visible in common.items():
        p, q = P[key], Q[key]
        perc = perception_scene(p)
        land = local_landed(q, p["w"], p["h"])
        h_perc = hits({"_perception": perc}, target)
        h_both = hits({"_perception": perc, "_tier": {"local_landed": land}}, target)
        h_local = hits({"_perception": {"status": "absent"}, "_tier": {"local_landed": land}}, target)
        frames.append({"key": key, "walk": key.split("/")[0], "target": target, "visible": visible,
                       "perc": h_perc[0]["bearing_deg"] if h_perc else None,
                       "both": h_both[0]["bearing_deg"] if h_both else None,
                       "local": h_local[0]["bearing_deg"] if h_local else None})
    out = {"frames": len(frames), "in_view": sum(f["visible"] for f in frames),
           "hfov_deg": HFOV, "bars": BARS, "sources": {}}
    walks = defaultdict(list)
    for f in frames:
        walks[f["walk"]].append(f)
    for src in ("perc", "both", "local"):
        r = {}
        vis = [f for f in frames if f["visible"]]
        r["B1_in_view_rate"] = round(sum(f[src] is not None for f in vis) / len(vis), 4)
        for k in KS:
            got = tot = 0
            for fs in walks.values():
                for i in range(len(fs) - k + 1):
                    win = fs[i:i + k]
                    if all(f["visible"] for f in win):
                        tot += 1
                        got += any(f[src] is not None for f in win)
            r[f"B2_k{k}"] = {"windows": tot, "with_sighting": got,
                             "rate": round(got / tot, 4) if tot else None}
        # B3: bearing error on S1.
        errs, dropped = [], 0
        for f in frames:
            lab = labels.get(f["key"])
            if not lab or lab["set"] != "S1":
                continue
            if f[src] is not None:
                truth = (lab["centre"] - 0.5) * HFOV
                errs.append(round(abs(f[src] - truth), 2))
        r["B3"] = {"sighted": len(errs), "of": sum(l["set"] == "S1" for l in labels.values()),
                   "p50": pct(errs, 50), "p90": pct(errs, 90), "max": max(errs, default=None),
                   "errors": sorted(errs)}
        # B4: out of view.
        oov = [f for f in frames if not f["visible"]]
        r["B4_frame"] = {"false": sum(f[src] is not None for f in oov), "of": len(oov),
                         "rate": round(sum(f[src] is not None for f in oov) / len(oov), 4)}
        bad = tot = 0
        for fs in walks.values():
            for i in range(len(fs) - 5 + 1):
                win = fs[i:i + 5]
                if not any(f["visible"] for f in win):
                    tot += 1
                    bad += sum(f[src] is not None for f in win) >= 2
        r["B4_k5"] = {"windows": tot, "two_or_more": bad, "rate": round(bad / tot, 4) if tot else None}
        # B5: the first sighting per walk.
        firsts = {}
        for w, fs in sorted(walks.items()):
            first = next((f for f in fs if f[src] is not None), None)
            row = {"frame": None if first is None else first["key"].split("/")[1],
                   "in_view": None if first is None else first["visible"]}
            if first is not None and first["visible"]:
                lab = labels.get(first["key"])
                if lab:
                    row["error_deg"] = round(abs(first[src] - (lab["centre"] - 0.5) * HFOV), 2)
                    row["right"] = row["error_deg"] <= BARS["B5deg"]
                else:
                    row["error_deg"], row["right"] = None, None     # unlabelled: not judged
            else:
                row["right"] = False
            firsts[w] = row
        r["B5"] = {"right": sum(bool(x["right"]) for x in firsts.values()),
                   "unlabelled": sum(x["right"] is None for x in firsts.values()),
                   "of": len(firsts), "walks": firsts}
        per = defaultdict(lambda: [0, 0, 0, 0])
        for f in frames:
            c = per[f["target"]]
            if f["visible"]:
                c[0] += 1
                c[1] += f[src] is not None
            else:
                c[2] += 1
                c[3] += f[src] is not None
        r["per_object"] = {t: {"in_view": c[0], "sighted": c[1], "out_of_view": c[2], "false": c[3]}
                           for t, c in sorted(per.items())}
        out["sources"][src] = r
    p = out["sources"]["perc"]
    out["verdicts"] = {
        "B1": p["B1_in_view_rate"] >= BARS["B1"],
        "B2": p["B2_k3"]["rate"] >= BARS["B2k3"] and p["B2_k5"]["rate"] >= BARS["B2k5"],
        "B3": p["B3"]["sighted"] > 0 and p["B3"]["p50"] <= BARS["B3p50"] and p["B3"]["p90"] <= BARS["B3p90"],
        "B4": p["B4_frame"]["rate"] <= BARS["B4frame"] and p["B4_k5"]["rate"] <= BARS["B4k5"],
        "B5": p["B5"]["right"] >= BARS["B5"],
    }
    out["verdicts"] = {k: "MET" if v else "FAILED" for k, v in out["verdicts"].items()}
    (HERE / "scores.json").write_text(json.dumps(out, indent=1))
    for src, r in out["sources"].items():
        print(src, json.dumps({k: (v if not isinstance(v, dict) else
                                   {kk: vv for kk, vv in v.items() if kk not in ("errors", "walks")})
                               for k, v in r.items() if k != "per_object"}))
    print("verdicts (perception alone):", out["verdicts"])


if __name__ == "__main__":
    main()
