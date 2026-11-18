# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/cloud-bandit-3187/count.py

3.187.A's recount of the cloud's side, run where the recordings are (the
desktop): for every frame of the pinned set, what its walk's `walk.jsonl`
row says the cloud did on it. Free: reads files, calls nothing. Writes
`cloud-counts.json`.

    PICAR_RECORDINGS=<recordings dir> .venv/bin/python evaluations/cloud-bandit-3187/count.py

A labelled frame is, by its row (the first row filing that frame):

  * `sync`  -- the row's `navigate` carries `target_visible` and no `_tier`:
    a walk under a cloud-every-step policy, so the answer is this frame's
    (`sync_by_model` splits it by `navigate.model_id`);
  * `tier_called` -- `_tier.cloud_called` true: the cloud was sent this frame
    (under `tier_async_cloud` its answer lands on a later row);
  * `tier_local` -- `_tier` present, no call on this frame;
  * `no_answer` -- a row with neither (`navigate` empty, an error, a stub);
  * `no_row` -- no row files the frame.

`with_shipped_answer` counts `sync` rows answered by the shipped model
(`SHIPPED_MODEL`, Opus 4.5); `need_minimum` is every other frame: the least
C must replay. 3.65's "373 carrying a cloud answer" (2026-10-09) is compared
with `sync + tier_called`.
"""

import json
from collections import Counter, defaultdict

from common import HERE, RECORDINGS, SHIPPED_MODEL, load_set, walk_of


def rows_by_file(walk: str) -> dict:
    out = {}
    path = RECORDINGS / walk / "walk.jsonl"
    if not path.exists():
        return out
    for line in path.read_text().splitlines():
        try:
            e = json.loads(line) if line.strip() else None
        except json.JSONDecodeError:
            continue
        if isinstance(e, dict) and e.get("file"):
            out.setdefault(e["file"], e)
    return out


def classify(row) -> tuple:
    """(kind, model_id) for one labelled frame's row, or ("no_row", None)."""
    if row is None:
        return "no_row", None
    nav = row.get("navigate") or {}
    tier = nav.get("_tier") or row.get("_tier")
    if tier:
        return ("tier_called" if tier.get("cloud_called") else "tier_local"), None
    if "target_visible" in nav and not nav.get("error"):
        return "sync", nav.get("model_id")
    return "no_answer", None


def main():
    d = load_set()
    rows = {}
    kinds, by_model = Counter(), Counter()
    per_walk = defaultdict(Counter)
    for r in d["frames"]:
        walk, frame = r["key"].split("/", 1)
        if walk not in rows:
            rows[walk] = rows_by_file(walk)
        kind, model = classify(rows[walk].get(frame))
        kinds[kind] += 1
        per_walk[walk][kind] += 1
        if kind == "sync":
            by_model[model or "unrecorded"] += 1
        if kind == "sync" and model == SHIPPED_MODEL:
            per_walk[walk]["with_shipped_answer"] += 1
    shipped = by_model.get(SHIPPED_MODEL, 0)
    out = {"set_hash": d["hash"], "recordings": str(RECORDINGS), "frames": d["n"],
           "walks_without_log": sorted(w for w, rs in rows.items() if not rs),
           **{k: kinds.get(k, 0) for k in ("sync", "tier_called", "tier_local", "no_answer", "no_row")},
           "sync_by_model": dict(by_model), "with_shipped_answer": shipped,
           "need_minimum": d["n"] - shipped,
           "comparable_to_3_65_373": kinds["sync"] + kinds["tier_called"],
           "tiered_walks": sorted(w for w, c in per_walk.items() if c["tier_called"] or c["tier_local"]),
           "per_walk": {w: dict(c) for w, c in sorted(per_walk.items())}}
    assert sum(kinds.values()) == d["n"] and len({walk_of(r["key"]) for r in d["frames"]}) == len(rows)
    (HERE / "cloud-counts.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: v for k, v in out.items() if k != "per_walk"}, indent=1))


if __name__ == "__main__":
    main()
