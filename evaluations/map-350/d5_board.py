# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.50.D5 criterion 6's last bar: reload time on the Jetson, at most 10 s,
of the store left by the desktop's tenth session (d5_runs.py).

Run ON THE BOARD, under its lock, from a copy of this checkout, with the
store copied to a temp dir (a SIM house's map: never committed or uploaded;
deleted afterwards):

    python tools/jetson/board_lock.py run --why "3.50.D5 criterion 6 reload" -- \\
      ssh picar-jetson 'cd ~/bench-350D5 && ~/vision-picar-service/.venv/bin/python \\
        evaluations/map-350/d5_board.py <store dir> <out.json>'

A robot server of its own (sim, the store's house, on 9500, the robot on the
dock) and the container on 9505 / ROS domain 50, so picar-stack (8000/8001)
is left running. The reload time is D3's: `checking` -> `ready` as read at
10 Hz on the bridge's /health; container start -> `ready` is recorded too.
"""
import json
import os
import subprocess
import sys
import time

import httpx

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ROBOT, BRIDGE, DOMAIN, NAME = 9500, 9505, 50, "picar-ros-350-d5"
IMAGE = os.environ.get("ROS_IMAGE", "vision-picar-ros:latest")
TRIALS = 3
BAR_S = 10.0


def busy(port):
    return bool(subprocess.run(["ss", "-ltnH", f"sport = :{port}"], capture_output=True,
                               text=True).stdout.strip())


def trial(store, bridge):
    subprocess.run(["docker", "rm", "-f", NAME], capture_output=True)
    t0 = time.time()
    subprocess.run(["docker", "run", "-d", "--name", NAME, "--network", "host", "-v", f"{store}:/maps",
                    "-e", f"BRIDGE_PORT={BRIDGE}", "-e", "BRIDGE_HOST=127.0.0.1",
                    "-e", f"ROBOT_URL=http://127.0.0.1:{ROBOT}", "-e", "BRAIN_URL=http://127.0.0.1:9",
                    "-e", f"ROS_DOMAIN_ID={DOMAIN}", IMAGE, "ros2", "launch", "picar_bringup",
                    "picar.launch.py"], capture_output=True, check=True)
    states = []
    while time.time() - t0 < 180:
        try:
            h = bridge.get("/health").json()
        except Exception:  # noqa: BLE001 -- still coming up
            time.sleep(0.1)
            continue
        st = h["slam"]["state"]
        if not states or states[-1][0] != st:
            states.append((st, round(time.time() - t0, 2)))
        if st in ("ready", "failed"):
            s = dict(states)
            return {"states": states, "reload": h["slam"]["reload"], "map_id": h["map"]["map_id"],
                    "reload_s": round(s["ready"] - s["checking"], 2)
                    if "ready" in s and "checking" in s else None,
                    "start_to_ready_s": s.get("ready")}
        time.sleep(0.1)
    return {"states": states, "error": "not ready in 180 s"}


def main(store, out):
    store = os.path.abspath(store)
    cur = json.load(open(os.path.join(store, "current.json")))
    meta = json.load(open(os.path.join(store, cur["map_id"], f"v{cur['version']}", "meta.json")))
    house = meta.get("house") or os.environ.get("SIM_MAP", "home_first_floor")
    for p in (ROBOT, BRIDGE):
        if busy(p):
            raise SystemExit(f"STOP: :{p} is in use")
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(SIM_MAP=house, ROBOT_DRIVE="ros", WORLD_MODE="ros",
               ROS_BRIDGE_URL=f"http://127.0.0.1:{BRIDGE}", PICAR_LOG_LEVEL="WARNING")
    robot = subprocess.Popen([sys.executable, "-m", "uvicorn", "robot.server:app", "--port",
                              str(ROBOT), "--host", "127.0.0.1", "--log-level", "warning"],
                             cwd=ROOT, env=env, stdout=open(out + ".robot.log", "w"),
                             stderr=subprocess.STDOUT)
    res = {"image": IMAGE, "house": house, "store": cur,
           "graph_mb": round(sum(os.path.getsize(os.path.join(store, cur["map_id"],
                                                              f"v{cur['version']}", f))
                                 for f in ("graph.posegraph", "graph.data")) / 1e6, 2),
           "trials": []}
    try:
        time.sleep(5)
        bridge = httpx.Client(base_url=f"http://127.0.0.1:{BRIDGE}", timeout=10)
        for _ in range(TRIALS):
            res["trials"].append(trial(store, bridge))
            print(json.dumps(res["trials"][-1]), flush=True)
        times = [t.get("reload_s") for t in res["trials"]]
        res["verdict"] = ("MET" if all(t is not None and t <= BAR_S for t in times)
                          and all(t["reload"].get("result") == "matched" for t in res["trials"])
                          else "FAILED")
    finally:
        subprocess.run(["docker", "logs", NAME], stdout=open(out + ".ros.log", "w"),
                       stderr=subprocess.STDOUT)
        subprocess.run(["docker", "rm", "-f", NAME], capture_output=True)
        robot.terminate()
        robot.wait(timeout=20)
        with open(out, "w") as f:
            json.dump(res, f, indent=1)
    print("verdict", res.get("verdict"), flush=True)


if __name__ == "__main__":
    main(*sys.argv[1:3])
