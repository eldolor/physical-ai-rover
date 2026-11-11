#!/usr/bin/env python3
"""3.173.C: S1, S2, S4, S6 from `c_surveys.py`'s rows, and the reported measure:
every `small_room` region each survey sent, from its brain log.

    python evaluations/small-rooms-3173/c_score.py ROWS.jsonl

Per region: its truth room, area, when it was sent (s after the mission's first
goal), how its goal ended, and its time cost -- from its send to the next goal
of any kind sent after it (the look around included). "Laundry/pantry reached"
is the truth track (`rooms_truth`); "when" is the first small-room goal into
that room that ended `succeeded`.
"""
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from sim.maps import build_world  # noqa: E402

HOUSE = "home_first_floor_stairs"
SMALL = ("laundry", "pantry")


def events(log):
    out = []
    for line in open(log, errors="replace"):
        k = line.find("explore {")
        if k < 0:
            continue
        try:
            out.append(json.loads(line[k + len("explore "):]))
        except ValueError:
            pass
    return out


def main(path):
    rows = [json.loads(x) for x in open(path)]
    w = build_world(HOUSE)
    room = {c: n for n, cs in sorted(w.rooms.items(), key=lambda kv: -len(kv[1])) for c in cs}
    n_rooms = len(w.rooms)

    def where(g):
        return room.get((int(g[0] / 0.3), int(g[1] / 0.3)))

    entered = []
    for r in rows:
        ev = events(Path(r["run_dir"]) / "explore_brain.log")
        sent = [e for e in ev if e.get("event") == "goal_sent"]
        t0 = sent[0]["t"] if sent else 0.0
        ended = {}
        for e in ev:
            if e.get("event") == "goal_ended" and e.get("kind") == "small_room":
                ended.setdefault(tuple(e["goal"]), e)
        regions, when = [], {}
        for k, e in enumerate(sent):
            if e.get("kind") != "small_room":
                continue
            nxt = next((x["t"] for x in sent[k + 1:]), None)
            end = ended.get(tuple(e["goal"]), {})
            name = where(e["goal"])
            regions.append({"room": name, "area_m2": e.get("area_m2"), "sent_s": round(e["t"] - t0),
                            "ended": end.get("state"), "cost_s": round(nxt - e["t"]) if nxt else None})
            if name in SMALL and end.get("state") == "succeeded" and name not in when:
                when[name] = round(end["t"] - t0)
        got = set(r.get("rooms_truth") or [])
        entered.append(len(got))
        print(f"survey {r['i']}: {r['outcome']}, {r['seconds']} s, rooms {len(got)} of {n_rooms}, "
              f"docked {r['docked']}, contacts {r['contacts']}, fell {r['fell']}, load {r['load']}")
        for s in SMALL:
            print(f"  {s}: {'entered' if s in got else 'NOT entered'}"
                  + (f", small-room goal reached at {when[s]} s" if s in when else ""))
        for g in regions:
            print(f"  region {g['room']} {g['area_m2']} m2: sent at {g['sent_s']} s, {g['ended']}, cost {g['cost_s']} s")
        corner = sum(g["cost_s"] or 0 for g in regions if g["room"] not in SMALL)
        print(f"  time on regions outside laundry/pantry: {corner} s")
    n = len(rows)
    s1 = {s: sum(s in (r.get("rooms_truth") or []) for r in rows) for s in SMALL}
    print()
    print(f"S1: laundry {s1['laundry']} of {n}, pantry {s1['pantry']} of {n}; bar each >= 4 of 5:",
          "met" if n >= 5 and min(s1.values()) >= 4 else "FAILED")
    med = statistics.median(entered) if entered else 0
    print(f"S2: rooms entered median {med} of {n_rooms}; bar >= 12:", "met" if med >= 12 else "FAILED")
    c = sum(r["contacts"] for r in rows)
    f = sum(bool(r["fell"]) for r in rows)
    print(f"S4: contacts {c}, falls {f}, lowest true gap {min(r['min_gap_cm'] for r in rows)} cm; bar 0:",
          "met" if c == 0 and f == 0 else "FAILED")
    bad = [r["i"] for r in rows if r["outcome"] == "failed" or not r["docked"]]
    print(f"S6: failed or not docked {bad}; bar none:", "met" if not bad else "FAILED")
    print(f"loads at start: {[r['load'] for r in rows]}")


if __name__ == "__main__":
    main(sys.argv[1])
