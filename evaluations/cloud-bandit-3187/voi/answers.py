# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/cloud-bandit-3187/voi/answers.py

3.187.G's data step, run where the recordings are (`PICAR_RECORDINGS`): the
cloud's recorded answer on every frame of the pinned set, as data, not
frames (G0). Free: reads files, calls nothing. Writes `cloud-answers.jsonl`,
one row per frame with an answer:

  * `key`, `target_visible`, `target_reached`, `target_direction`, `action`
    (short enums; no reply text, no image);
  * `model_id`, `prompt_variant` (None where the walk did not record them);
  * `source`: `walk` (the walk's own row, `count.py`'s `sync` rule) or
    `pilot` (B's answer, which wins where both exist; the walk's answer is
    kept as `walk_target_visible`);
  * `t`: seconds since the walk's first row, when the row carries a numeric
    `captured_at` / `ts` (the replay's clock; else the frame index).

    PICAR_RECORDINGS=<dir> python evaluations/cloud-bandit-3187/voi/answers.py
"""

import json
import sys
from pathlib import Path

VOI = Path(__file__).resolve().parent
sys.path.insert(0, str(VOI.parent))
from common import HERE, RECORDINGS, jsonl, load_set, rows_by_file  # noqa: E402

from count import classify  # noqa: E402  3.187.A's one rule for a row

OUT = VOI / "cloud-answers.jsonl"
CLOCK_FIELDS = ("captured_at", "ts")


def stamp(row: dict):
    for f in CLOCK_FIELDS:
        v = row.get(f)
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            return float(v) / (1000.0 if v > 1e11 else 1.0)    # ms or s since the epoch
    return None


def main():
    d = load_set()
    pilot = {r["key"]: r for r in jsonl(HERE / "pilot-answers.jsonl")
             if isinstance(r.get("target_visible"), bool)}
    rows, starts, out, missing_walks = {}, {}, [], set()
    for f in sorted(d["frames"], key=lambda f: f["key"]):
        walk, name = f["key"].split("/", 1)
        if walk not in rows:
            rows[walk], _ = rows_by_file(walk)
            if not rows[walk]:
                missing_walks.add(walk)
            stamps = [s for s in (stamp(r) for r in rows[walk].values()) if s is not None]
            starts[walk] = min(stamps) if stamps else None
        row = rows[walk].get(name)
        kind, model, variant = classify(row)
        rec = None
        if kind == "sync":
            nav = row["navigate"]
            rec = {"key": f["key"], "target_visible": bool(nav["target_visible"]),
                   "target_reached": nav.get("target_reached") is True,
                   "target_direction": nav.get("target_direction"),
                   "action": nav.get("action") or nav.get("reply_action"),
                   "model_id": model, "prompt_variant": variant, "source": "walk"}
            s = stamp(row)
            if s is not None and starts[walk] is not None:
                rec["t"] = round(s - starts[walk], 3)
        p = pilot.get(f["key"])
        if p is not None:
            base = rec or {"key": f["key"]}
            if rec is not None:
                base["walk_target_visible"] = rec["target_visible"]
            # One answer per row: the pilot recorded no reached/direction,
            # so the walk's (another model's) are not carried over.
            base.update({"target_visible": p["target_visible"], "action": p.get("action"),
                         "target_reached": p.get("target_reached") is True,
                         "target_direction": p.get("target_direction"),
                         "model_id": p["model_id"], "prompt_variant": None, "source": "pilot"})
            rec = base
        if rec is not None:
            out.append(rec)
    if missing_walks:
        raise SystemExit(f"walks without a walk.jsonl under {RECORDINGS}: {sorted(missing_walks)}")
    OUT.write_text("".join(json.dumps(r) + "\n" for r in out))
    print(json.dumps({"frames": d["n"], "with_answer": len(out),
                      "pilot": sum(r["source"] == "pilot" for r in out),
                      "with_clock": sum("t" in r for r in out)}))


if __name__ == "__main__":
    main()
