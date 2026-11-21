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
    PICAR_ROS_NETWORK=host python tools/desktop_lock.py exclusive --why "3.205.A live" -- \\
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


def _drop(inv_dir, m):
    sweep = _load("carried_3205", HERE / "sweep.py")
    return {"dropped": sweep.drop_target(inv_dir, m["target"])}


def run(rows=ROWS):
    live = _load("live_370", ROOT / "evaluations" / "find-last-seen-370" / "live.py")
    live.run(rows=rows, finds=1, find_limit_s=1200.0, before_find=_drop, section="3.205")


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
    return {"preconditions": pre, "valid": all(pre.values()),
            "N8": {"outcome": f.get("outcome"), "found_m": f.get("found_m"), "contacts": f.get("contacts"),
                   "wrong_found": wrong, "resight": f.get("resight"),
                   "met": all(pre.values()) and f.get("contacts") == 0 and not wrong},
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
