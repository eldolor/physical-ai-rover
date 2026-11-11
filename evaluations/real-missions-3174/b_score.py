#!/usr/bin/env python3
"""3.174.B: score B1-B7 from the live run (the plan's bar table).

    .venv/bin/python evaluations/real-missions-3174/b_score.py REC.jsonl CONTROL.jsonl \\
        [--kept ~/picar-kept/3174b]

REC/CONTROL are b_live.py's harness rows; the recorder's rows, tracks and walks and the
truth tracks are read from `--kept` (local only). Only this scorer reads the simulator's
truth. Rooms from the track (B4): the saved map's frame is fitted once to the truth frame
(a rigid fit over every time-matched pair of every mission, as the user's rooms drawn on
the saved map would be), then each sample names the truth room under it.
"""
import argparse
import importlib.util
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
from control.walk_eval import compute_metrics  # noqa: E402
from control.walk_store import LocalWalkStore  # noqa: E402
from sim.maps import build_world  # noqa: E402

_spec = importlib.util.spec_from_file_location("recorder_3174", HERE / "recorder.py")
rec = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rec)

CELL_M = 0.3
FIELDS = ("mission_id", "job", "outcome", "end_state", "end_reason", "seconds", "ret_state",
          "refusals", "late_ticks", "v_start", "v_end", "v_min", "distance_m", "pose_samples",
          "pose_coverage", "frames_kept", "gap_s", "report", "intervention_lines", "health",
          "battery", "unmeasurable")
MATCH_S = 0.5


def load(p):
    p = Path(p).expanduser()
    return [json.loads(x) for x in p.read_text().splitlines() if x.strip()] if p.exists() else []


def fit(pairs):
    """Rigid (rotation, translation) taking map-frame points onto truth points."""
    a = np.array([p for p, _ in pairs], float)
    b = np.array([q for _, q in pairs], float)
    ca, cb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    r = (u @ vt).T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = (u @ vt).T
    return r, cb - r @ ca


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("control")
    ap.add_argument("--kept", default="~/picar-kept/3174b")
    a = ap.parse_args(argv)
    kept = Path(a.kept).expanduser()
    hr, hc = load(a.rec), load(a.control)
    rrows = {r["mission_id"]: r for r in load(kept / "batch-rec" / "rows.jsonl")}
    house = json.loads((ROOT / "evaluations" / "autonomy-3158" / "B" / "sheet.json").read_text())["house"]
    world = build_world(house)
    room_of = {c: n for n, cs in world.rooms.items() for c in cs}
    report, ok = {}, {}

    # B1: one row a mission, every field, the harness's outcome, reason and time.
    b1 = []
    for h in hr:
        r = rrows.get(h.get("mission_id"))
        # Review: a field is present when its key is; None is a value (`end_state`
        # of a fault, `ret_state` with no return). A reading not had says `missing`.
        miss = [f for f in FIELDS if r is None or f not in r]
        miss += [f for f in ("report", "health", "battery", "intervention_lines")
                 if r is not None and r.get(f) == "missing"]
        same = r is not None and r["outcome"] == h["outcome"] and \
            r["end_reason"] == (h["end_reason"] or "")[:200] and abs(r["seconds"] - h["seconds"]) <= 3
        b1.append({"mission": h.get("mission_id"), "row": r is not None, "absent_fields": miss,
                   "same": same, "seconds": [h.get("seconds"), r and r["seconds"]]})
    ok["B1"] = len(hr) == 6 and all(x["row"] and not x["absent_fields"] and x["same"] for x in b1)
    report["B1"] = b1

    # B2, B3.
    cov = [rrows[h["mission_id"]]["pose_coverage"] for h in hr if h.get("mission_id") in rrows]
    ok["B2"] = len(cov) == len(hr) and min(cov) >= 0.95
    report["B2"] = cov
    b3 = []
    for h in hr:
        r = rrows.get(h.get("mission_id"))
        if r and h.get("truth_dist_m"):
            b3.append(round(abs(r["distance_m"] - h["truth_dist_m"]) / h["truth_dist_m"], 4))
    ok["B3"] = len(b3) == len(hr) and max(b3) <= 0.10
    report["B3"] = {"rel_err": b3, "truth_m": [h.get("truth_dist_m") for h in hr],
                    "track_m": [rrows.get(h.get("mission_id"), {}).get("distance_m") for h in hr]}

    # B4: one rigid fit of the saved map's frame, then rooms per mission.
    tracks, truths, pairs = {}, {}, []
    for h in hr:
        mid, r = h.get("mission_id"), rrows.get(h.get("mission_id"))
        if not r:
            continue
        tr = load(kept / "batch-rec" / "track" / f"{mid[:rec.WALK_MAX - 4]}.jsonl")
        truth = json.loads((kept / "truth" / f"{mid}.json").read_text())
        frames = [s.get("frame_id") or s.get("map_id") for s in tr]
        main_frame = max(set(frames), key=frames.count) if frames else None
        tr = [s for s, f in zip(tr, frames) if f == main_frame]
        tracks[mid], truths[mid] = tr, truth
        ts = np.array([q[0] for q in truth])
        for s in tr:
            t = r["started_at"] + s["t"]
            k = int(np.argmin(np.abs(ts - t)))
            if abs(ts[k] - t) <= MATCH_S:
                pairs.append(((s["x_m"], s["y_m"]), (truth[k][1], truth[k][2])))
    rot, off = fit(pairs)
    resid = [float(np.hypot(*((rot @ np.array(p) + off) - np.array(q)))) for p, q in pairs]

    def room_at(x, y):
        X, Y = rot @ np.array([x, y]) + off
        return room_of.get((int(math.floor(X / CELL_M)), int(math.floor(Y / CELL_M))))
    agree = total = seen_agree = seen = 0
    b4 = []
    for h in hr:
        mid = h.get("mission_id")
        if mid not in tracks:
            continue
        got = set(rec.rooms_from_track(tracks[mid], room_at))
        want = set(h.get("rooms_truth") or [])
        for n in world.rooms:
            total += 1
            agree += (n in got) == (n in want)
        seen += len(got | want)
        seen_agree += len(got & want)
        b4.append({"mission": mid, "missed": sorted(want - got), "extra": sorted(got - want)})
    ok["B4"] = total > 0 and agree / total >= 0.95
    report["B4"] = {"agree": agree, "pairs": total, "share": round(agree / total, 4) if total else None,
                    # Review: the stricter share, over rooms the track or truth entered
                    # (reported beside the bar, which stays as written)
                    "share_entered": round(seen_agree / seen, 4) if seen else None,
                    "fit_pairs": len(pairs), "fit_resid_m_median": round(float(np.median(resid)), 3),
                    "fit_resid_m_p90": round(float(np.percentile(resid, 90)), 3), "missions": b4}

    # B5: frames per second, decodable, readable by walk_eval and make_sets.
    from PIL import Image
    store = LocalWalkStore(kept / "batch-rec" / "walks")
    b5 = []
    for h in hr:
        r = rrows.get(h.get("mission_id"))
        if not r:
            continue
        bad, metrics_ok, drawn = 0, True, True
        for w in r["walks"]:
            d = kept / "batch-rec" / "walks" / w
            ents = [json.loads(x) for x in (d / "walk.jsonl").read_text().splitlines()]
            for e in ents:
                try:
                    Image.open(d / e["file"]).verify()
                except Exception:  # noqa: BLE001 -- counted
                    bad += 1
            metrics_ok &= compute_metrics(ents)["frames"] == len(ents)
            with tempfile.TemporaryDirectory() as tmp:
                # make_sets reads labels.json in the walk: a scratch walk of links to
                # the kept frames, with stand-in labels; the kept walk untouched
                (Path(tmp) / "w").mkdir()
                for e in ents:
                    (Path(tmp) / "w" / e["file"]).symlink_to(d / e["file"])
                (Path(tmp) / "w" / "labels.json").write_text(json.dumps(
                    {"description": "x", "target_visible_labels":
                     {e["file"]: i % 2 == 0 for i, e in enumerate(ents)}}))
                p = subprocess.run([sys.executable, str(ROOT / "evaluations" / "tiered-380" / "U" / "make_sets.py"),
                                    "--out", str(Path(tmp) / "sets"), "--recordings", tmp, "--walk", "w"],
                                   capture_output=True, text=True, cwd=ROOT)
                drawn &= p.returncode == 0
        b5.append({"mission": h["mission_id"], "per_s": round(r["frames_kept"] / max(r["seconds"], 1), 3),
                   "frames": r["frames_kept"], "bad": bad, "walk_eval": metrics_ok, "make_sets": drawn,
                   "listed": all(w in store.list_walks() for w in r["walks"])})
    ok["B5"] = len(b5) == len(hr) and all(x["per_s"] >= 0.45 and not x["bad"] and x["walk_eval"]
                                          and x["make_sets"] and x["listed"] for x in b5)
    report["B5"] = b5

    # B6, B7: no harm, against the control.
    def rate(rows):
        late, mov = sum(r.get("late_ticks") or 0 for r in rows), sum(r.get("moving_ticks") or 0 for r in rows)
        return round(1000 * late / mov, 3) if mov else None
    finds_r = [h for h in hr if h["job"] == "find"]
    rr, rc = rate(finds_r), rate(hc)
    camf = [h.get("camera_failures") for h in hr]
    ok["B6"] = rr is not None and rc is not None and rr <= rc + 1 and all(c == 0 for c in camf)
    report["B6"] = {"late_per_1000_rec_finds": rr, "late_per_1000_rec_all": rate(hr),
                    "late_per_1000_control": rc, "camera_failures_rec": camf,
                    "camera_failures_control": [h.get("camera_failures") for h in hc]}
    fr = sum(h["outcome"] == "found" for h in finds_r)
    fc = sum(h["outcome"] == "found" for h in hc)
    harm = sum((h.get("contacts") or 0) + bool(h.get("fell")) for h in hr + hc)
    ok["B7"] = fr >= fc and harm == 0
    report["B7"] = {"found_rec": f"{fr} of {len(finds_r)}", "found_control": f"{fc} of {len(hc)}",
                    "contacts": [h.get("contacts") for h in hr + hc],
                    "falls": [h.get("fell") for h in hr + hc]}
    print(json.dumps({"ok": ok, **report}, indent=1))


if __name__ == "__main__":
    main()
