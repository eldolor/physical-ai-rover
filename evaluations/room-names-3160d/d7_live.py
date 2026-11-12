#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.160.D's D7: one find through the brain's HTTP API on the live stack, on a saved
map whose rooms a person named first (`POST /mission/rooms/names`).

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    ROS_IMAGE=vision-picar-ros:plan-3.156 python evaluations/room-names-3160d/d7_live.py OUT.jsonl

3.158.B's rehearsal (`evaluations/autonomy-3158/B/rehearsal.py`: its house, its
stand-ins, its truth recorder) on volume `picar-maps-3160d`, a copy of 3.172.E's
saved map. Each cut room the editor shows is named as a person would, with the
true room under its centre: the map frame is put on the simulator's by the dock
pose, read in both. The names are kept outside the repo (ROOM_NAMES_DIR,
`~/picar-kept/3160d/room-names`); OUT holds counts and outcomes only.
"""
import json
import math
import os
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

import httpx

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0] / "autonomy-3158" / "B"))
os.environ["ROOM_NAMES_DIR"] = str(Path("~/picar-kept/3160d/room-names").expanduser())
import rehearsal as R          # noqa: E402

R.VOLUME = "picar-maps-3160d"
TARGET = "washer"               # laundry: a room 3.172.E's missions on this map never reached
READY_S = 180


def _dir(h):                    # compass degrees (0 = -y, 90 = +x) as a unit vector
    return math.sin(math.radians(h)), -math.cos(math.radians(h))


def to_truth(map_pose, truth):
    """Map-frame metres -> the simulator's metres, from one pose read in both."""
    a = math.atan2(*reversed(_dir(truth["heading_deg"]))) - math.atan2(*reversed(_dir(map_pose["heading_deg"])))
    c, s = math.cos(a), math.sin(a)

    def f(x, y):
        dx, dy = x - map_pose["x_m"], y - map_pose["y_m"]
        return truth["x_m"] + c * dx - s * dy, truth["y_m"] + s * dx + c * dy
    return f


def name_rooms(robot, brain, house):
    t0 = time.time()
    while True:
        v = brain.get("/mission/rooms").json()
        if v.get("frame_id") and v.get("rooms") or time.time() - t0 > READY_S:
            break
        time.sleep(3)
    if not v.get("frame_id"):
        return {"named": 0, "why": v.get("closed")}
    f = to_truth(robot.get("/world/pose").json(), robot.get("/world/truth").json())
    truth = R.Truth(house, robot)
    named, missed = 0, 0
    for _ in range(64):
        v = brain.get("/mission/rooms").json()
        todo = [r for r in v["rooms"] if r["key"].startswith("c")]
        pick = None
        for r in todo:
            x, y = f(r["x_m"], r["y_m"])
            name = truth.room.get((int(x / R.CELL_M), int(y / R.CELL_M)))
            if name:
                pick = (r, name)
                break
        if pick is None:
            missed = len(todo)
            break
        r, name = pick
        resp = brain.post("/mission/rooms/names", json={
            "frame_id": v["frame_id"], "cut_hash": v["cut_hash"], "rev": v["rev"],
            "op": "name", "room": r["key"], "name": name})
        if resp.status_code != 200:
            return {"named": named, "why": f"{resp.status_code} {resp.text[:200]}"}
        named += 1
    v = brain.get("/mission/rooms").json()
    return {"named": named, "unnamed_left": missed,
            "person_rooms": sum(r["source"] == "person" for r in v["rooms"]),
            "names": sorted({r["name"] for r in v["rooms"] if r["source"] == "person"})}


def main(out):
    port = R.free_port()
    httpd = ThreadingHTTPServer(("127.0.0.1", port), R.Standins)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    os.environ.update(LOCAL_VLM_URL=f"http://127.0.0.1:{port}", VISION_URL=f"http://127.0.0.1:{port}")
    sheet = json.loads((Path(R.__file__).with_name("sheet.json")).read_text())
    robot = R.de.stack(sheet["house"])
    brain = httpx.Client(base_url=f"http://127.0.0.1:{R.de.BRAIN}", timeout=60)
    try:
        rec = {"load": round(os.getloadavg()[0], 2), **name_rooms(robot, brain, sheet["house"])}
        print("named", rec, flush=True)
        if rec["named"]:
            rec.update(R.mission(robot, brain, sheet["house"], {"job": "find", "target": TARGET}))
            s = brain.get("/mission/status").json()
            rec["room_labels"] = s.get("room_labels")
            rec["goals_sent"] = (s.get("explore") or {}).get("goals_sent")
        rec.pop("log_tail", None)
        with open(out, "a") as fh:
            fh.write(json.dumps({"target": TARGET, **rec}) + "\n")
        print(json.dumps({k: rec.get(k) for k in ("named", "outcome", "contacts", "room_labels",
                                                  "seconds", "docked")}), flush=True)
    finally:
        R.de._kill_ports()
        os.system(f"docker logs {R.de.CONTAINER} > {R.de.RUN_DIR}/ros.log 2>&1; "
                  f"docker rm -f {R.de.CONTAINER} >/dev/null 2>&1")
        httpd.shutdown()


if __name__ == "__main__":
    main(sys.argv[1])
