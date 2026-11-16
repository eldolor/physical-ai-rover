# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.178.F's diagnosis: why does a pass never reach a report?

    python evaluations/watch-list-3178/diagnose_f.py --seed 3178 --out diagnose-f-3178.json
    python evaluations/watch-list-3178/diagnose_f.py --seed 4178 --out diagnose-f-4178.json

TUNING seeds only (B's 3178/3179, D's 4178/4179); never E's fresh 5178/5179
or F's own (6178/6179). The mission is `sweep.run_one`'s with the list on (E's placed
house, 150 steps, B's noise and 5% list-labelled false labels), with two
recorders that change nothing the robot or the inventory does:

* the detector records, per frame, every list instance a TRUE sim detection
  saw within the inventory's `MAX_RANGE_M` (4 m; the pass needs one within
  `PASS_RANGE_M`), its true range and bearing, and
  what 3.46's erring detector kept of it (dropped, relabelled, kept);
* the inventory records, per kept list-labelled detection, the range the
  lidar gave it, where it was placed, the instance distance, the landmark it
  joined and whether that frame counted a look.

For each pass that did not reach a report, every frame that saw its
instance is kept, and a cause is assigned by the first stage the sighting
did not get through (`classify`).
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import multiprocessing
import random
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))

from brain import inventory as inv  # noqa: E402
from brain.goal_pose import _wrap  # noqa: E402
from brain.watch_list import item_for  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from tests import inventory_sweep as base  # noqa: E402
from tests.conftest import mock_world_for  # noqa: E402

def load_sweep():
    """`sweep.py` under a unique module name (other evaluations have a
    `sweep.py` too), once per process."""
    name = "watch_sweep_3178"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, HERE / "sweep.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
    return sys.modules[name]


sw = load_sweep()
LIST = sw.LIST


class TracingDetector(sw.ListErringDetector):
    """`ListErringDetector`, plus a per-frame record of each list instance's
    true sighting and what the erring detector made of it."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.frames: list = []     # one entry per call that returned detections
        self.last = None

    def __call__(self, frame):
        g = self.grid
        view = g.theta + g.pan * base.PAN_ANGLE_RAD
        true = {}                  # instance -> (range_m, camera bearing)
        for d in frame.get("detections") or []:
            item = item_for(d.get("label"), LIST)
            # Every sighting the inventory could place (its MAX_RANGE_M), as
            # the sweep's view count (3.178.F) reads them.
            if item is None or d["distance_m"] > inv.MAX_RANGE_M:
                continue
            a = view + math.radians(d["bearing_deg"])
            r = d["distance_m"] / base.CELL_M
            cell = (int(math.floor(g.x + r * math.cos(a))), int(math.floor(g.y + r * math.sin(a))))
            for k in self.item_insts[item]:
                if cell in self.insts[k][1] or sw._near_cell(cell, self.insts[k][1]):
                    have = true.get(k)
                    if have is None or d["distance_m"] < have[0]:
                        true[k] = (d["distance_m"], d["bearing_deg"])
        clean = inv.frame_detections(frame)
        out = super().__call__(frame)
        if out is None:
            self.last = None
            return None
        rec = {"true": {}}
        for k, (rng, bearing) in true.items():
            item = item_for(self.insts[k][0], LIST)
            # The clean (pre-noise) box nearest the true bearing, then what
            # the erring detector made of the frame near that bearing.
            boxes = [d for d in clean or [] if item_for(d["label"], LIST) == item]
            box = min(boxes, key=lambda d: abs(d["bearing_deg"] - bearing), default=None)
            kept = [d for d in out if item_for(d["label"], LIST) == item
                    and box is not None and abs(d["bearing_deg"] - box["bearing_deg"]) <= 10.0]
            relabelled = (box is not None and not kept
                          and any(abs(d["bearing_deg"] - box["bearing_deg"]) <= 10.0
                                  and item_for(d["label"], LIST) != item for d in out))
            rec["true"][k] = {"range_m": round(rng, 3), "bearing_deg": round(bearing, 2),
                              "box": box is not None, "kept": len(kept),
                              "relabelled": relabelled}
        self.frames.append(rec)
        self.last = rec
        return out


class TracingInventory(inv.Inventory):
    """`Inventory`, plus a record of each list-labelled detection's path."""

    def __init__(self, det: TracingDetector, insts: list):
        super().__init__()
        self.det, self.insts = det, insts
        self.ii = sw.item_instances(insts)
        self.trace: list = []

    def observe(self, detections, pose, scan, pan_deg=0.0, fov_deg=inv.DEFAULT_FOV_DEG,
                room="unknown", step=None):
        before = self.frames
        rows = []
        if detections is not None and pose and pose.get("usable"):
            returns = inv.scan_returns(scan)
            for d in detections:
                label = str(d.get("label", "")).strip().lower()
                item = item_for(label, LIST)
                if item is None or d.get("bearing_deg") is None:
                    continue
                bearing = _wrap(pan_deg + float(d["bearing_deg"]))
                rng = inv.range_at(returns, bearing)
                row = {"label": label, "item": item, "bearing": round(bearing, 2),
                       "range_m": None if rng is None else round(rng, 3)}
                if rng is not None and rng <= inv.MAX_RANGE_M:
                    x, y = inv._place(pose, bearing, rng)
                    row["xy"] = (round(x, 3), round(y, 3))
                    row["to_inst"] = {k: round(sw.dist_to_cells(x, y, self.insts[k][1]), 3)
                                      for k in self.ii[item]}
                    # The instance's own distance along that bearing: is the
                    # lidar's return the item, or something nearer/farther?
                    row["inst_range"] = {k: round(min(math.hypot((i + 0.5) * base.CELL_M - pose["x_m"],
                                                                 (j + 0.5) * base.CELL_M - pose["y_m"])
                                                      for i, j in self.insts[k][1]), 3)
                                         for k in self.ii[item]}
                rows.append(row)
        super().observe(detections, pose, scan, pan_deg=pan_deg, fov_deg=fov_deg,
                        room=room, step=step)
        if self.frames == before:
            return
        for row in rows:
            if "xy" in row:
                x, y = row["xy"]
                lm = next((lm for lm in self.landmarks if lm.near(x, y, 0.002)), None)
                if lm is not None:
                    row["lm"] = lm.id
                    row["look"] = self.frames in lm._looks and lm._looks[self.frames] == row["label"]
        self.trace.append({"frame": self.frames, "step": step,
                           "pose": {k: round(float(pose[k]), 3) for k in ("x_m", "y_m", "heading_deg")},
                           "det": self.det.last, "rows": rows})


def trace_one(job) -> dict:
    k, start, target, seed, max_steps, placed = job
    (cx, cy), theta, room = start
    grid = build_world("complex_house")
    if placed:
        grid.objects = {**grid.objects, **{tuple(p["cell"]): p["item"] for p in placed}}
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    robot = MockRobot(grid, render=False)
    insts = sw.instances(placed)
    det = TracingDetector(grid, random.Random(seed), list_rng=random.Random(seed * 7 + 1),
                          insts=insts)
    runner = MissionRunner(robot, target_object=target, max_steps=max_steps,
                           policy="frontier", world=mock_world_for(robot),
                           detections_fn=det, watch_list=LIST)
    tinv = TracingInventory(det, insts)
    runner.inventory = tinv
    runner.start()
    while runner.tick():
        pass
    report = runner.inventory_report()
    r = {"nearest": det.nearest, "reported": report.get("reported") or [],
         "watch": {"label": report.get("watch")}}
    sc = sw.score_mission(r, "label", insts)
    lost = [i for i in sc["passes"] if i not in sc["reached"]]
    lms = {lm.id: lm for lm in tinv.landmarks}
    out = {"mission": k + 1, "start_room": room, "passes": len(sc["passes"]),
           "reached": len(sc["reached"]), "lost": []}
    for i in lost:
        out["lost"].append(explain(i, insts, det, tinv, lms))
    return out


def explain(i, insts, det, tinv, lms) -> dict:
    item = item_for(insts[i][0], LIST)
    cells = insts[i][1]
    seen = [t for t in tinv.trace if t["det"] and i in t["det"]["true"]]
    poses = [t["pose"] for t in seen]
    kept_frames = [t for t in seen if t["det"]["true"][i]["kept"]]
    rows = [(t, row) for t in kept_frames for row in t["rows"]
            if row["item"] == item and i in row.get("to_inst", {i: None})]
    placed = [(t, row) for t, row in rows if "xy" in row]
    near = [(t, row) for t, row in placed if row["to_inst"][i] <= sw.PIN_M]
    touched = sorted({row["lm"] for _, row in near if "lm" in row})
    looks_near = [(t, row) for t, row in near if row.get("look")]
    lm_rows = []
    for lid in touched:
        lm = lms.get(lid)
        if lm is None:
            lm_rows.append({"id": lid, "gone": True})
            continue
        lm_rows.append({"id": lid, "label": lm.label, "belief": round(lm.belief, 3),
                        "votes": dict(lm.votes), "misses": lm.misses,
                        "to_inst": round(sw.dist_to_cells(lm.x, lm.y, cells), 3),
                        "reported": lm.belief >= inv.REPORT_BELIEF})
    e = {"inst": i, "item": item, "cell": sorted(cells)[0],
         "true_frames": len(seen), "true_views": sw.views(poses),
         "true_ranges": [t["det"]["true"][i]["range_m"] for t in seen],
         "kept_frames": len(kept_frames), "kept_views": sw.views([t["pose"] for t in kept_frames]),
         "relabelled": sum(t["det"]["true"][i]["relabelled"] for t in seen),
         "unranged": sum(1 for _, row in rows if "xy" not in row),
         "placed_d": [row["to_inst"][i] for _, row in placed],
         "near_views": sw.views([t["pose"] for t, _ in near]),
         "looks_near": len(looks_near), "landmarks": lm_rows,
         # Each frame that saw it: the pose, the truth, and the inventory's rows of its item.
         "frames": [{"frame": t["frame"], "pose": t["pose"], "true": t["det"]["true"][i],
                     "rows": [{k: row.get(k) for k in ("label", "bearing", "range_m", "lm", "look")}
                              | {"to_inst": (row.get("to_inst") or {}).get(i),
                                 "inst_range": (row.get("inst_range") or {}).get(i)}
                              for row in t["rows"] if row["item"] == item]}
                    for t in seen]}
    e["cause"] = classify(e)
    return e


def classify(e: dict) -> str:
    """The first stage the sighting did not get through."""
    if e["true_views"] < 2:
        return "one viewpoint: the camera saw it from fewer than two new views"
    if e["kept_views"] < 2:
        return "detector: fewer than two views survived the erring detector"
    if e["near_views"] < 2:
        return "placement: fewer than two views ranged within 0.5 m"
    if any(lm.get("reported") is False and item_for(lm["label"], LIST) == e["item"]
           for lm in e["landmarks"]):
        return "belief: a landmark of its item under REPORT_BELIEF"
    if any(item_for(lm.get("label"), LIST) != e["item"] for lm in e["landmarks"]
           if not lm.get("gone")):
        return "fusion: joined a landmark labelled otherwise"
    return "other"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--missions", type=int, default=sw.MISSIONS)
    ap.add_argument("--steps", type=int, default=sw.MAX_STEPS)
    ap.add_argument("--placed", default="placements.json")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    if args.seed not in (3178, 4178):
        raise SystemExit("tuning seeds only: B's 3178 or D's 4178 (never E's or F's fresh seeds)")
    placed = tuple(json.loads(Path(__file__).with_name(args.placed).read_text())["placed"])
    out = {"seed": args.seed, "steps": args.steps}
    for key, target, seed in (("search", base.ABSENT_TARGET, args.seed),
                              ("present", sw.PRESENT_TARGET, args.seed + 1)):
        jobs = [(k, s, target, seed * 100 + k, args.steps, placed)
                for k, s in enumerate(sw.starts(args.missions, seed, placed))]
        # fork: the workers share this process's module table (`load_sweep`).
        with ProcessPoolExecutor(args.workers, mp_context=multiprocessing.get_context("fork")) as ex:
            out[key] = list(ex.map(trace_one, jobs))
    path = Path(args.out)
    if not path.is_absolute():
        path = Path(__file__).with_name(args.out)
    path.write_text(json.dumps(out, indent=1, sort_keys=True, default=str) + "\n")
    for key in ("search", "present"):
        for m in out[key]:
            for e in m["lost"]:
                print(f"{key} {m['mission']:2d} {e['item']:<8} views {e['true_views']} kept "
                      f"{e['kept_views']} near {e['near_views']} placed {e['placed_d']} "
                      f"lms {e['landmarks']} -> {e['cause']}", file=sys.stderr)


if __name__ == "__main__":
    main()
