# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.205.A N8: one live find on a carried map with no memory of the target.
M2's live harness (`evaluations/find-last-seen-370/live.py`): a survey on the
ROS stack, everything restarted on the dock (the reload judged), plan entry
k 0's target X, the inventory put back to the survey's files and then every
row naming X dropped (`sweep.drop_target`), X not moved; then the find
through the brain's HTTP API (1200 s, the sim's limit). Bars: "3.205.A:
criteria", N8.

    eval "$(python tools/plan_section.py env)"
    docker tag "$ROS_IMAGE" vision-picar-ros:s3205-plan-3.70   # the harness's image guards
    ROS_IMAGE=vision-picar-ros:s3205-plan-3.70 PICAR_ROS_NETWORK=host python tools/desktop_lock.py exclusive --why "3.205.A live" -- \\
        python evaluations/carried-no-memory-3205/live.py run
    python evaluations/carried-no-memory-3205/live.py score
"""
import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ROWS = HERE / "live.jsonl"
FOUND_M = 0.9


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_SWEEP = []


def _drop(inv_dir, m):
    if not _SWEEP:
        _SWEEP.append(_load("carried_3205", HERE / "sweep.py"))
    return {"dropped": _SWEEP[0].drop_target(inv_dir, m["target"])}


def run(rows=ROWS):
    # One find only: the drop rewrites the survey's own files, which the
    # harness's put-back (it removes files added since) does not undo (review).
    live = _load("live_370", ROOT / "evaluations" / "find-last-seen-370" / "live.py")
    live.run(rows=rows, finds=1, find_limit_s=1200.0, before_find=_drop)   # image tagged *-plan-3.70 (the harness's guards)


def score(path=ROWS):
    rows = [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]
    s1 = next((r for r in rows if r.get("row") == "survey"), {})
    s2 = next((r for r in rows if r.get("row") == "reload"), {})
    finds = [r for r in rows if r.get("row") == "find"]
    f = finds[0] if len(finds) == 1 else {}
    map_id = s1.get("map_id")
    pre = {"survey_saved_one_map": bool(map_id and s1.get("report_frame_id") == map_id and s1.get("targets")),
           "reload_matched": (s2.get("reload") or {}).get("result") == "matched" and s2.get("map_id") == map_id,
           "dropped": ((f.get("before_find") or {}).get("dropped") or 0) >= 1,
           "find_on_the_reload": (f.get("saved_map") or {}).get("frame_id") == map_id
           and (f.get("saved_map") or {}).get("reloaded") is True,
           "no_recall": f.get("recall_goals") == 0}
    wrong = f.get("outcome") == "found" and (f.get("found_m") is None or f["found_m"] > FOUND_M)
    # The orchestrator, 2026-11-21: a live run above ~4 (CLAUDE.md 6, 3.79) is
    # invalid, not met. Review: every part must carry its load; none missing reads 0.
    loads = [(r.get("load") or {}).get("max") for r in (s1, s2, f)]
    load_max = None if any(v is None for v in loads) else max(loads)
    load_ok = load_max is not None and load_max <= 4.0
    met = all(pre.values()) and f.get("contacts") == 0 and not wrong
    return {"preconditions": pre, "valid": all(pre.values()) and load_ok, "load_max": load_max,
            "load_ok": load_ok,
            "N8": {"outcome": f.get("outcome"), "found_m": f.get("found_m"), "contacts": f.get("contacts"),
                   "wrong_found": wrong, "resight": f.get("resight"),
                   "met": met and load_ok,
                   "verdict": ("MET" if met else "FAILED") if load_ok and all(pre.values()) else
                              "INVALID (load)" if all(pre.values()) else "INVALID"},
            "beside": {k: f.get(k) for k in ("target", "seconds", "mission_s", "goals", "goal_kinds",
                                             "distance_m", "min_gap_cm", "ret", "end_reason", "load_max")},
            "survey": {k: s1.get(k) for k in ("outcome", "seconds", "reported", "map_id")}}


if __name__ == "__main__":
    if sys.argv[1:2] == ["run"]:
        run()
    elif sys.argv[1:2] == ["score"]:
        print(json.dumps(score(Path(sys.argv[2]) if len(sys.argv) > 2 else ROWS), indent=1))
    else:
        raise SystemExit(__doc__)
