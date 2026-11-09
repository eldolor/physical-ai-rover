#!/usr/bin/env python3
"""3.158.B: every criterion of step B's protocol, from rows.jsonl and sheet.json.

Bars as 3.158.A wrote them, on sim truth. Run: python score.py [rows.jsonl]
"""
import json
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).parent
LATE_S = 60                       # 3.156 C5: docked by the limit, never later than it + 60 s
LIMIT_S = {"find": 1200, "survey": 1800}
RESERVE_V = 10.2                  # robot/battery_ladder.py's default


def pct(n, d):
    return f"{n} of {d} ({100.0 * n / d:.1f}%)" if d else "0 of 0"


def nearest_rank(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs) + 0.999999) - 1)]


def main():
    rows = [json.loads(x) for x in Path(sys.argv[1] if len(sys.argv) > 1 else HERE / "rows.jsonl").read_text().splitlines()]
    sheet = json.loads((HERE / "sheet.json").read_text())
    rooms = set(sheet["rooms"])
    c = [r for r in rows if r["phase"] == "counted"]
    d = [r for r in rows if r["phase"] == "stretch"]

    def right(r):
        return r["outcome"] == "found" and (r.get("found_target_m") or 99) <= 0.9

    def wrong(rs):
        return sum(r["outcome"] == "found" and not right(r) for r in rs)

    def late(r):
        return not r["docked"] or r["seconds"] > LIMIT_S[r["job"]] + LATE_S

    finds, present, absent, surveys = ([r for r in c if r["job"] == "find"],
                                       [r for r in c if r["job"] == "find" and r["present"]],
                                       [r for r in c if r["job"] == "find" and not r["present"]],
                                       [r for r in c if r["job"] == "survey"])
    n1 = sum(map(right, present))
    rate = n1 / len(present)
    room_pct = [100.0 * len(rooms & set(r["rooms_truth"])) / len(rooms) for r in surveys]
    secs = [r["seconds"] for r in finds]
    hours = sum(r["seconds"] for r in c) / 3600.0
    dfinds = [r for r in d if r["job"] == "find"]
    n4 = sum(map(right, dfinds))
    sc = lambda ok: "met" if ok else "FAILED"
    def row(k, val, ok):
        print(f"| {k} | {val} | {ok if isinstance(ok, str) else sc(ok)} |")
    print("| # | Measured | Result |\n|---|---|---|")
    row("C1", pct(n1, len(present)) + " of present finds; bar 80%", rate >= 0.8)
    row("C2", f"{wrong(rows)} wrong found over all {len(rows)}; bar 0", wrong(rows) == 0)
    row("C3", f"{sum(r['outcome'] in ('exhausted', 'out_of_budget') for r in absent)} of {len(absent)}: "
              + ", ".join(sorted({r['outcome'] for r in absent})), len(absent) == 4 and all(r['outcome'] in ('exhausted', 'out_of_budget') for r in absent))
    ne = sum(r["outcome"] == "exhausted" for r in surveys)
    row("C4", f"{ne} of {len(surveys)} exhausted (" + ", ".join(sorted({r['outcome'] for r in surveys})) + "); bar 4 of 5", ne >= 4)
    row("C5", f"median {st.median(room_pct):.1f}%, lowest {min(room_pct):.1f}% ({', '.join(f'{p:.0f}' for p in room_pct)}) of {len(rooms)}; bars 90%, 75%", st.median(room_pct) >= 90 and min(room_pct) >= 75)
    row("C6", "the stand-in reads the simulator's names; says nothing of naming", "reported only")
    row("C7", f"{sum(r['contacts'] for r in c)} contacts; lowest true gap {min(r['min_gap_cm'] for r in c):.1f} cm; bar 0", sum(r['contacts'] for r in c) == 0)
    row("C8", f"{sum(bool(r['fell']) for r in c)} falls, overhang max {max(r['overhang_cm'] for r in c):.1f} cm; bar 0", not any(r['fell'] or r['overhang_cm'] for r in c))
    row("C9", "0 by construction (the harness sends no person's input)", "not read")
    nd = sum(r["docked"] for r in c)
    row("C10", pct(nd, len(c)) + f"; misses i={[r['i'] for r in c if not r['docked']]}; bar 95%", nd / len(c) >= 0.95)
    nf = sum(r["outcome"] == "failed" for r in c)
    row("C11", pct(nf, len(c)) + " `failed`; bar 5%", nf / len(c) <= 0.05)
    p90, med = nearest_rank(secs, 0.9), st.median(secs)
    sm = max(r["seconds"] for r in surveys)
    row("C12", f"finds (all {len(finds)}, return included) median {med:.0f} s, p90 {p90} s; bars 600, 1200. Surveys max {sm} s; bar 2100", med <= 600 and p90 <= 1200 and sm <= 2100)
    vm = min(r["v_min"] for r in c)
    lt = [r["i"] for r in c if late(r)]
    row("C13", f"lowest {vm:.2f} V, Reserve {RESERVE_V} V: reached by {sum(r['v_min'] <= RESERVE_V for r in c)}. Return reserve (docked by the limit + {LATE_S} s) missed by i={lt}; Wh not read", vm > RESERVE_V and not lt)
    nh = sum(map(right, finds)) / hours
    row("C14", f"{nh:.2f} finds per hour ({sum(map(right, finds))} in {hours:.2f} h of missions); bar 3", nh >= 3)
    row("C15", "the board's", "not read")
    row("D1", "0 by construction", "not read")
    row("D2", f"{sum(r['contacts'] for r in d)} contacts, {sum(bool(r['fell']) for r in d)} falls, overhang max {max(r['overhang_cm'] for r in d):.1f} cm", not any(r['contacts'] or r['fell'] or r['overhang_cm'] for r in d))
    row("D3", f"{wrong(d)} wrong found", wrong(d) == 0)
    row("D4", f"{pct(n4, len(dfinds))}; bar C1's rate minus 10 points = {100 * rate - 10:.1f}%, and >= 6 finds", 100.0 * n4 / len(dfinds) >= 100 * rate - 10 and n4 >= 6)
    row("D5", pct(sum(r['docked'] for r in d), len(d)) + " docked", all(r['docked'] for r in d))
    row("D6", f"lowest {min(r['v_min'] for r in d):.2f} V; services at the end not read by the harness",
        ("reserve met" if min(r["v_min"] for r in d) > RESERVE_V else "reserve FAILED") + "; services not read")
    row("D7", f"{sum(r['outcome'] == 'failed' for r in d)} `failed`", not any(r['outcome'] == 'failed' for r in d))
    calls = [r["standin_calls"]["local"] for r in rows if r["job"] == "find"]
    print(f"\nStand-in local calls per find (`standin_calls.local`; the brain's own `local_calls` stat "
          f"read {sorted({r['local_calls'] for r in rows if r['job'] == 'find'})}): median {st.median(calls):.0f}, "
          f"min {min(calls)}, max {max(calls)}; cloud calls {sum(r['standin_calls']['cloud'] for r in rows)}. "
          f"Surveys {sorted({r['standin_calls']['local'] for r in rows if r['job'] == 'survey'})} (survey runs `explore`, not tiered).")
    print(f"Stretch: {len(d)} missions, {sum(r['job'] == 'survey' for r in d)} surveys, "
          f"{sum(r['seconds'] for r in d) / 3600:.2f} h of missions; `stretch_ended` {[r['i'] for r in d if r.get('stretch_ended')]}.")
    print(f"Highest load at a start {max(r['load'] for r in rows)} (i={max(rows, key=lambda r: r['load'])['i']}).")


if __name__ == "__main__":
    main()
