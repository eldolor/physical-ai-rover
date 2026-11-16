# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/last-seen-3175c/measure.py -- 3.175.C: "where did you last see
it?" against real chains, in process. The bars C1-C5 are in
`docs/plans/ros-alignment/3.175-memory-when-things-move.md`.

For each house, MISSIONS finds for a name that is in no house (so each mission
maps the house and names everything it sees) run on one saved map's chain:
`MissionRunner` -> explore -> `MockRobot` (rendered frames, for 3.162's
thumbnails), 3.175.A's stand-in nav, an `InventoryStore` directory and a
`ReportStore` fed through the brain's own `_write_saw`. Then:

* every label a find would recall from the head is asked through the REAL
  brain app (`TestClient`), its stores pointed at the chain, with a robot that
  counts every attribute read and a world that counts every call (C4);
* the truth for C1 is a real `MissionRunner` seeded from the same store with
  that label as its target: the places it hands explore's `recall_from`;
* C2 fetches each answer's thumbnail through the brain's route;
* C3 asks every simulator label never in the head, and five made-up names,
  and asks once with an empty store;
* C5 times the route with the report store at its 50.

    python tools/desktop_lock.py shared --why "3.175.C measure" -- \\
        python evaluations/last-seen-3175c/measure.py run OUT.json WORKDIR
    python evaluations/last-seen-3175c/measure.py score OUT.json
"""

import json
import math
import shutil
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

HOUSES = ("complex_house", "scaled_house", "home_first_floor")
FRAME = "map-3175c"
ABSENT = "unicorn"                # in no house: a find for it explores to its end
MISSIONS = 2
LIMIT_S = 1200.0
TICK_CAP = 20000
MADE_UP = ("unicorn", "zeppelin", "saxophone", "giraffe", "hovercraft")
TIMED_ASKS = 40


class CountingRobot:
    """Any attribute read is counted (and fails): the route must not touch it."""

    def __init__(self):
        object.__setattr__(self, "reads", [])

    def __getattr__(self, name):
        self.reads.append(name)
        raise AttributeError(name)


class CountingWorld:
    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        def call(*a, **k):
            self.calls.append(name)
            return {"usable": False}
        return call


def _saved_map_world(grid):
    from sim.mock_world import MockWorld

    class SavedMap(MockWorld):
        def get_pose(self):
            p = super().get_pose()
            return dict(p, frame_id=FRAME) if p.get("usable") else p
    return SavedMap(grid)


def run_mission(house, target, chain, writer, name, cells=None, seed_only=False):
    """One find on the chain. `seed_only`: build and seed it, return the
    places it hands explore (the truth for C1), and drive nothing."""
    import logging
    logging.disable(logging.WARNING)
    from control.brain_server import _write_saw
    from control.inventory_store import InventoryStore
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from tests.dock_sweep import inflated_nav_class

    grid = build_world(house)
    world = _saved_map_world(grid)
    if cells is not None:
        world._cells = list(cells)
    robot = MockRobot(grid, render=not seed_only, exact_verbs=True)
    store = InventoryStore(str(chain))
    runner = MissionRunner(robot, policy="explore", navigator=inflated_nav_class()(robot),
                           world=world, target_object=target,
                           mission_time_limit_s=LIMIT_S, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=False)
    runner.inventory_prior = store.latest_for
    if not seed_only:                 # a truth runner saves nothing to the chain
        runner.inventory_sink = lambda report: store.save(name, report)
        runner.report_id = name
        runner.saw_sink = lambda payload: _write_saw(lambda: writer, name, payload)
    runner.start()
    if seed_only:
        runner._seed_inventory(world.get_pose())
        return [list(p) for p in getattr(runner.agent, "recall", [])]
    ticks = 0
    while runner.tick() and ticks < TICK_CAP:
        ticks += 1
    s = runner.status()
    return {"house": house, "mission": name, "outcome": s["outcome"],
            "search_s": round(grid.sim_time, 1), "ticks": ticks}, list(world._cells)


def _client(chain, saw):
    import os
    from fastapi.testclient import TestClient
    from control import brain_server
    os.environ["WHAT_I_SAW_DIR"] = str(saw)
    os.environ["INVENTORY_DIR"] = str(chain)
    robot, world = CountingRobot(), CountingWorld()
    app = brain_server.create_app(robot_factory=lambda: robot, world_factory=lambda r: world)
    return TestClient(app), robot, world


def run(out, work):
    from brain.what_i_saw import ReportStore, ReportWriter
    from sim.maps import build_world
    work = Path(work)
    result = {"houses": {}, "missions": []}
    for house in HOUSES:
        d = work / house
        shutil.rmtree(d, ignore_errors=True)
        chain, saw = d / "chain", d / "saw"
        chain.mkdir(parents=True)
        writer = ReportWriter(ReportStore(saw))
        cells = None
        for i in range(MISSIONS):
            row, cells = run_mission(house, ABSENT, chain, writer, f"{house}-m{i}", cells)
            assert writer.flush(30), "the report writer did not finish"
            result["missions"].append(row)
            print(row, flush=True)
        client, robot, world = _client(chain, saw)
        head = json.loads(max(chain.glob("*.json"), key=lambda p: json.loads(p.read_text())["carry_seq"]).read_text())
        labels = sorted({str(r["label"]) for r in head["reported"] + head["candidates"]})
        asks = []
        for label in labels:
            got = client.get("/mission/last-seen", params={"name": label}).json()
            truth = run_mission(house, label, chain, None, "truth", seed_only=True)
            thumb_ok = None
            w = got.get("when")
            if w and w.get("thumb"):
                r = client.get(f"/mission/reports/{w['mission_id']}/{w['thumb']}")
                thumb_ok = r.status_code == 200 and r.content[:3] == b"\xff\xd8\xff"
            asks.append({"label": label, "status": got["status"],
                         "places": [[p["x_m"], p["y_m"]] for p in got["places"]],
                         "truth": truth, "when": w, "thumb_ok": thumb_ok})
        all_labels = sorted(set(build_world(house).objects.values()))
        never = [n for n in all_labels if not any(n in lb for lb in labels)]
        unseen = {n: client.get("/mission/last-seen", params={"name": n}).json()["status"]
                  for n in never + list(MADE_UP)}
        result["houses"][house] = {"head_carry_seq": head["carry_seq"], "labels": labels,
                                   "asks": asks, "unseen": unseen,
                                   "robot_reads": list(robot.reads), "world_calls": list(world.calls),
                                   "reports_kept": len(list(saw.iterdir()))}
    # C3's empty store, and C5's full one: the last house's reports copied to 50.
    empty = work / "empty"
    shutil.rmtree(empty, ignore_errors=True)
    (empty / "chain").mkdir(parents=True)
    (empty / "saw").mkdir()
    client, robot, world = _client(empty / "chain", empty / "saw")
    result["empty"] = client.get("/mission/last-seen", params={"name": "mug"}).json()["status"]
    full = work / HOUSES[0]
    saw = full / "saw"
    src = sorted(p for p in saw.iterdir() if p.is_dir() and not p.name.startswith("."))
    from brain.what_i_saw import KEEP
    n = len(src)
    for i in range(KEEP - n):
        dst = saw / f"copy-{i}"
        shutil.copytree(src[i % n], dst)
        rep = json.loads((dst / "report.json").read_text())
        rep["seq"] = 1000 + i
        rep["mission_id"] = dst.name
        (dst / "report.json").write_text(json.dumps(rep))
    client, robot, world = _client(full / "chain", saw)
    label = result["houses"][HOUSES[0]]["labels"][0]
    times = []
    for _ in range(TIMED_ASKS):
        t0 = time.perf_counter()
        assert client.get("/mission/last-seen", params={"name": label}).status_code == 200
        times.append((time.perf_counter() - t0) * 1000)
    result["timing"] = {"reports": len([p for p in saw.iterdir() if p.is_dir()]),
                        "ms": [round(t, 2) for t in times],
                        "robot_reads": list(robot.reads), "world_calls": list(world.calls)}
    Path(out).write_text(json.dumps(result, indent=1))
    print(json.dumps(score_result(result), indent=1))


def _p95(xs):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(math.ceil(0.95 * len(xs))) - 1)]


def score_result(r):
    from brain.explore import RECALL_MAX
    c1 = c1_ok = c2 = c2_ok = 0
    c1_bad, c2_bad = [], []
    for house, h in r["houses"].items():
        for a in h["asks"]:
            if not a["truth"]:
                # A find would not recall it: never answered as a place.
                if a["status"] == "seen":
                    c1_bad.append(f"{house}:{a['label']} (seen, not recalled)")
                continue
            c1 += 1
            # explore keeps RECALL_MAX places; the answer lists every one.
            ok = a["status"] == "seen" and len(a["places"][:RECALL_MAX]) == len(a["truth"]) and all(
                math.dist(p, q) <= 1e-3 for p, q in zip(a["places"], a["truth"]))
            c1_ok += ok
            if not ok:
                c1_bad.append(f"{house}:{a['label']}")
            if a["when"] is not None:
                c2 += 1
                good = a["thumb_ok"] is True
                c2_ok += good
                if not good:
                    c2_bad.append(f"{house}:{a['label']}")
    unseen = [s for h in r["houses"].values() for s in h["unseen"].values()]
    reads = sum(len(h["robot_reads"]) + len(h["world_calls"]) for h in r["houses"].values())
    reads += len(r["timing"]["robot_reads"]) + len(r["timing"]["world_calls"])
    seen_without_when = sum(1 for h in r["houses"].values() for a in h["asks"]
                            if a["truth"] and a["when"] is None)
    return {"C1": {"recalled_labels": c1, "right": c1_ok, "wrong": c1_bad},
            "C2": {"with_report": c2, "thumb_jpeg": c2_ok, "bad": c2_bad,
                   "recalled_without_kept_report": seen_without_when},
            "C3": {"asked": len(unseen), "not_seen": unseen.count("not_seen"),
                   "empty_store": r["empty"]},
            "C4": {"robot_reads_and_world_calls": reads},
            "C5": {"reports": r["timing"]["reports"], "median_ms": round(statistics.median(r["timing"]["ms"]), 2),
                   "p95_ms": round(_p95(r["timing"]["ms"]), 2)},
            "missions": [(m["mission"], m["outcome"], m["search_s"]) for m in r["missions"]]}


def score(path):
    print(json.dumps(score_result(json.loads(Path(path).read_text())), indent=1))


if __name__ == "__main__":
    if sys.argv[1] == "run":
        run(sys.argv[2], sys.argv[3])
    else:
        score(sys.argv[2])
