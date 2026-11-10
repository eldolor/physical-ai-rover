"""3.172.B's scorer: B1-B6 from live.py's rows and each batch's brain log.

    python evaluations/rehearsal-fixes-3172/score_B.py ROWS.jsonl

Aisle seconds use 3.158.B's method (DIAGNOSIS.md): dwell between explore's
events whose pose lies in the dining room's chair aisle. The target's room is
the truth room holding the most of the target's cells; "entered" is the row's
truth track (`rooms_truth`).
"""
import collections
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sim.maps import build_world  # noqa: E402

BOX = (7.9, 9.75, 9.75, 11.45)  # x0, y0, x1, y1 (m), B/DIAGNOSIS.md


def aisle_seconds(rows):
    """Per row: seconds explore's poses spent in BOX (one brain log per batch)."""
    out = {}
    by_dir = collections.OrderedDict()
    for r in rows:
        by_dir.setdefault(r["run_dir"], []).append(r)
    for d, rs in by_dir.items():
        lines = open(Path(d) / "explore_brain.log", errors="replace").read().splitlines()
        starts = [i for i, l in enumerate(lines) if "mission started:" in l]
        for r, i0, i1 in zip(rs, starts, starts[1:] + [len(lines)]):
            ev = []
            for l in lines[i0:i1]:
                if '"event"' in l and " explore " in l:
                    try:
                        e = json.loads(l[l.index("{"):])
                    except ValueError:
                        continue
                    e["_t"] = float(l.split()[0])
                    ev.append(e)
            dwell = 0.0
            for a, b in zip(ev, ev[1:]):
                p = a.get("pose")
                if p and BOX[0] <= p[0] <= BOX[2] and BOX[1] <= p[1] <= BOX[3]:
                    dwell += b["_t"] - a["_t"]
            out[r["i"]] = dwell
    return out


def target_room(world, name):
    rooms = collections.Counter()
    for c, n in world.objects.items():
        if n == name:
            for room, cells in world.rooms.items():
                if c in cells:
                    rooms[room] += 1
    return rooms.most_common(1)[0][0] if rooms else None


def main(path):
    rows = [json.loads(l) for l in open(path)]
    world = build_world("home_first_floor_stairs")
    n_rooms = len(world.rooms)
    aisle = aisle_seconds(rows)
    finds = [r for r in rows if r["job"] == "find"]
    surveys = [r for r in rows if r["job"] == "survey"]
    print("i job target outcome s aisle_s rooms target_room entered")
    entered = 0
    for r in rows:
        tr = target_room(world, r.get("target")) if r["job"] == "find" else None
        hit = tr in (r["rooms_truth"] or []) if tr else None
        entered += bool(hit)
        print(r["i"], r["job"], r.get("target"), r["outcome"], r["seconds"],
              round(aisle[r["i"]]), len(r["rooms_truth"] or []), tr, hit)
    found = sum(r["outcome"] == "found" for r in finds)
    wrong = sum(r["outcome"] == "found" and (r["found_target_m"] or 9) > 1.0 for r in finds)
    bad = sum(r["contacts"] + bool(r["fell"]) + (r["outcome"] == "failed") for r in rows) + wrong
    med = statistics.median(aisle.values())
    print(f"B1 median aisle s, all {len(rows)}: {med:.0f} (bar < 60)")
    print(f"B2 found {found} of {len(finds)} = {100 * found / len(finds):.1f}% (bar >= 32.1%)")
    print(f"B3 entered the target's room {entered} of {len(finds)} (bar >= 8)")
    print(f"B4 surveys' rooms of {n_rooms}: {[len(r['rooms_truth'] or []) for r in surveys]} (bar each >= 11)")
    print(f"B5 contacts+falls+wrong+failed {bad} (wrong {wrong}) (bar 0)")
    print(f"B6 docked {sum(r['docked'] for r in rows)} of {len(rows)}")


if __name__ == "__main__":
    main(sys.argv[1])
