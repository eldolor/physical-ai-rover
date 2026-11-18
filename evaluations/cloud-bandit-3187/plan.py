# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/cloud-bandit-3187/plan.py

3.187.A, free and offline: from the pinned set (3.177.A, hash 00d12dbc44f6)
and 3.92's per-frame local rows, write

  * `split.json`  -- the walks held out and the walks trained on (seed 3651);
  * `pilot.json`  -- B's 20 frames (seed 3187);
  * `local-counts.json` -- the recount this repo can make without the frames.

The rules are in the docstrings below and in the section file, written
before any cloud answer was read. With `--check`, recomputes all three and
fails on any difference; an existing file is never overwritten without
`--force`.

    .venv/bin/python evaluations/cloud-bandit-3187/plan.py [--check | --force]
"""

import json
import random
import sys
from collections import defaultdict

from common import (CAL, HERE, LOCAL_NO_BELOW, LOCAL_YES_AT, PILOT_N, PILOT_SEED, SPLIT_SEED,
                    jsonl, load_set, local_kind, obj, walk_of)

MAX_DRAWS = 100


def walks_by_object(d: dict) -> dict:
    """object -> {walk: {"n": frames, "visible": [keys], "absent": [keys]}}."""
    out = defaultdict(dict)
    for r in d["frames"]:
        w = out[obj(r["target"])].setdefault(walk_of(r["key"]), {"n": 0, "visible": [], "absent": []})
        w["n"] += 1
        w["visible" if r["visible"] else "absent"].append(r["key"])
    return out


def split(d: dict) -> dict:
    """By walk, never by frame. One `random.Random(3651)`; objects in sorted
    order; each object's walks sorted by name, then shuffled; the first
    max(1, round(n/3)) held out (none when an object has one walk). Where it
    allows, both halves of an object must hold a frame with the target in
    view: if not, the object's walks are shuffled again by the same generator,
    up to 100 draws; an object that cannot meet it keeps its first draw and is
    listed in `in_view_one_side`."""
    rng = random.Random(SPLIT_SEED)
    held, train, one_side = [], [], []
    for o, walks in sorted(walks_by_object(d).items()):
        names = sorted(walks)
        k = max(1, round(len(names) / 3)) if len(names) >= 2 else 0
        first = None
        for _ in range(MAX_DRAWS):
            order = names[:]
            rng.shuffle(order)
            first = first or order
            h, t = order[:k], order[k:]
            if k == 0 or (any(walks[w]["visible"] for w in h) and any(walks[w]["visible"] for w in t)):
                break
        else:
            order, one_side = first, one_side + [o]
            h, t = order[:k], order[k:]
        held += h
        train += t
    frames = {walk_of(r["key"]) for r in d["frames"]}
    assert sorted(held + train) == sorted(frames)
    n = lambda ws: sum(1 for r in d["frames"] if walk_of(r["key"]) in ws)  # noqa: E731
    v = lambda ws: sum(r["visible"] for r in d["frames"] if walk_of(r["key"]) in ws)  # noqa: E731
    return {"seed": SPLIT_SEED, "set_hash": d["hash"],
            "held_out": sorted(held), "train": sorted(train),
            "held_out_frames": n(set(held)), "train_frames": n(set(train)),
            "held_out_in_view": v(set(held)), "train_in_view": v(set(train)),
            "in_view_one_side": one_side}


def pilot(d: dict) -> dict:
    """B's 20 frames. One `random.Random(3187)`. Objects in sorted order,
    20 // objects frames each (3 for the pinned set's 6), plus one more to
    each of the 20 % objects with the most frames (2: the basket, the red
    bag). An object's walks are shuffled once and its picks cycle over them,
    passing over a walk with no frame left; pick i of an object asks for a frame with the target in view when
    i is even and one without when odd, taking the other kind when its walk
    has none left; the frame is a uniform choice among that walk's remaining
    frames of that kind. From every walk, held out or not: the pilot fits
    nothing, and C answers every frame anyway."""
    rng = random.Random(PILOT_SEED)
    by = walks_by_object(d)
    base, extra = divmod(PILOT_N, len(by))
    quota = {o: base for o in by}
    for o in sorted(by, key=lambda o: (-sum(w["n"] for w in by[o].values()), o))[:extra]:
        quota[o] += 1
    assert sum(quota.values()) == PILOT_N
    target = {r["key"]: r["target"] for r in d["frames"]}
    picked = []
    for o in sorted(by):
        walks = sorted(by[o])
        rng.shuffle(walks)
        pools = {w: {"visible": by[o][w]["visible"][:], "absent": by[o][w]["absent"][:]} for w in walks}
        for i in range(quota[o]):
            # The next walk in the cycle with a frame left (a used-up walk is passed over).
            left = [walks[(i + j) % len(walks)] for j in range(len(walks))]
            w = next((w for w in left if pools[w]["visible"] or pools[w]["absent"]), None)
            if w is None:
                raise SystemExit(f"{o}: fewer frames than its quota of {quota[o]}")
            want = "visible" if i % 2 == 0 else "absent"
            kind = want if pools[w][want] else ("absent" if want == "visible" else "visible")
            key = rng.choice(pools[w][kind])
            pools[w][kind].remove(key)
            picked.append({"key": key, "object": o, "target": target[key], "visible": kind == "visible"})
    return {"seed": PILOT_SEED, "set_hash": d["hash"], "n": len(picked),
            "walks": len({walk_of(p["key"]) for p in picked}),
            "in_view": sum(p["visible"] for p in picked), "frames": picked}


def local_counts(d: dict) -> dict:
    """What the repo can recount without the frames or walk logs: 3.92.A's
    perception (YOLOE + CLIP, the gate) and VE rows, per frame."""
    perc = {r["key"]: r for r in jsonl(CAL / "perception.jsonl")}
    ve = {r["key"]: r for r in jsonl(CAL / "qwen-VE.jsonl")}
    keys = [r["key"] for r in d["frames"]]
    assert set(perc) == set(keys) == set(ve), "3.92's rows are not the pinned set's frames"
    c = defaultdict(int)
    for r in d["frames"]:
        k, v = r["key"], r["visible"]
        det = perc[k]["status"] == "detected"
        kind = local_kind(ve[k])
        c["frames"] += 1
        c["in_view"] += v
        c["gate_passed"] += det
        c["gate_missed_in_view"] += v and not det       # 3.65's first kind of useful call
        c["gate_false_pass"] += det and not v           # 3.65's second kind
        c[f"ve_{kind}"] += 1
        if kind != "unable":
            right = (kind == "yes") == v
            c["ve_wrong"] += not right
            c["ve_fixes_gate_miss"] += v and not det and kind == "yes"
            c["ve_fixes_gate_false_pass"] += det and not v and kind == "no"
            c["ve_breaks_gate_right"] += (det == v) and not right
            c["ve_disagrees_gate"] += det != (kind == "yes")
    return {"set_hash": d["hash"], "thresholds": f"VE yes >= {LOCAL_YES_AT}, no <= {LOCAL_NO_BELOW} (brain/tiered.py, config/robot.yaml)", **c}


def main():
    d = load_set()
    out = {"split.json": split(d), "pilot.json": pilot(d), "local-counts.json": local_counts(d)}
    if "--check" in sys.argv:
        for name, new in out.items():
            if json.loads((HERE / name).read_text()) != new:
                raise SystemExit(f"{name} differs from what the rule produces")
        print("split, pilot and counts unchanged")
        return
    for name, new in out.items():
        if (HERE / name).exists() and "--force" not in sys.argv:
            raise SystemExit(f"{name} exists; --check it, or --force")
    old_pilot = HERE / "pilot.json"
    if (list(HERE.glob("pilot-answers*.jsonl")) and old_pilot.exists()
            and json.loads(old_pilot.read_text()) != out["pilot.json"]):
        raise SystemExit("B's answers belong to the existing pilot.json; never change it")
    for name, new in out.items():
        (HERE / name).write_text(json.dumps(new, indent=1) + "\n")
    s, p = out["split.json"], out["pilot.json"]
    print("held out", len(s["held_out"]), "walks,", s["held_out_frames"], "frames;",
          "train", len(s["train"]), "walks,", s["train_frames"], "frames")
    print("pilot", p["n"], "frames from", p["walks"], "walks,", p["in_view"], "in view")
    print(json.dumps(out["local-counts.json"]))


if __name__ == "__main__":
    main()
