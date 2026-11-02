"""3.50.D5 step 4, ON THE JETSON: criterion 6's reload bar (the slowest of
three reloads at most 10 s, `docker run` -> `ready`, amendment 2) with the
chosen setup (the bridge's defaults: lifelong on, cap 1.5).

Each store is one saved version copied from the laptop's D5 run; for each,
three times: a fresh robot server (the simulator, the robot on the dock), a
fresh copy of the store, the container started and its /health watched at
10 Hz until `ready` with the start's truth taken (d3_runs.container's rule).
A reload that does not read `matched` is recorded as such, not as a time.

Run from a copy of this branch on the board, with the board lock held and
picar-stack stopped:

    python evaluations/map-350/d5_board_reload.py OUT.json home_first_floor \\
        LABEL=/path/to/map-xxxxxxxx/vN [LABEL=...]

The stores are a floor plan of the SIM house; they are deleted at the end.
"""
import json
import os
import shutil
import socket
import subprocess
import sys
import time

import httpx

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
IMAGE = "vision-picar-ros:plan-3.50"
CONTAINER = "picar-ros-350"
ROBOT, BRAIN, BRIDGE, DOMAIN = 9500, 9501, 9505, 50
WORK = os.path.expanduser("~/vp-350-maps")
REPS = 3


def preflight():
    """Refuse under no hold of this section's, or with a port taken: a stale
    server answering /health would time the reload against the wrong robot."""
    try:
        owner = open(os.path.expanduser("~/.picar-board.owner")).read()
    except OSError:
        owner = ""
    if "3.50" not in owner:
        raise SystemExit(f"hold the board lock for 3.50 first (owner: {owner.strip() or 'none'})")
    for port in (ROBOT, BRIDGE):
        with socket.socket() as sk:
            if sk.connect_ex(("127.0.0.1", port)) == 0:
                raise SystemExit(f"port {port} is in use")


def stop(p):
    p.terminate()
    try:
        p.wait(10)
    except subprocess.TimeoutExpired:
        p.kill()
        p.wait(10)


def remove(maps):
    """The store, even files the container wrote as root."""
    shutil.rmtree(maps, ignore_errors=True)
    if os.path.exists(maps):
        subprocess.run(["docker", "run", "--rm", "-v", f"{os.path.dirname(maps)}:/w", IMAGE,
                        "rm", "-rf", f"/w/{os.path.basename(maps)}"], capture_output=True)
    if os.path.exists(maps):
        raise RuntimeError(f"could not remove {maps}")


def robot_server(house, log):
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(SIM_MAP=house, ROBOT_DRIVE="ros", WORLD_MODE="ros",
               ROS_BRIDGE_URL=f"http://127.0.0.1:{BRIDGE}", PICAR_LOG_LEVEL="WARNING")
    p = subprocess.Popen([sys.executable, "-m", "uvicorn", "robot.server:app", "--port", str(ROBOT),
                          "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=env,
                         stdout=open(log, "w"), stderr=subprocess.STDOUT)
    for _ in range(300):
        try:
            if httpx.get(f"http://127.0.0.1:{ROBOT}/health", timeout=2).status_code == 200:
                return p
        except httpx.HTTPError:
            pass
        time.sleep(0.1)
    p.kill()
    raise RuntimeError("robot server did not come up")


def store(src, dst):
    """A store holding only `src` (…/map-xxxxxxxx/vN), current.json naming it."""
    shutil.rmtree(dst, ignore_errors=True)
    meta = json.load(open(os.path.join(src, "meta.json")))
    shutil.copytree(src, os.path.join(dst, meta["map_id"], f"v{meta['version']}"))
    with open(os.path.join(dst, "current.json"), "w") as f:
        json.dump({"map_id": meta["map_id"], "version": meta["version"]}, f)
    return meta


def reload_once(maps, log):
    subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{BRIDGE}", timeout=5)
    t0 = time.time()
    subprocess.run(["docker", "run", "-d", "--name", CONTAINER, "-v", f"{maps}:/maps",
                    "--network", "host", "-e", f"BRIDGE_PORT={BRIDGE}", "-e", "BRIDGE_HOST=127.0.0.1",
                    "-e", f"ROBOT_URL=http://127.0.0.1:{ROBOT}", "-e", f"BRAIN_URL=http://127.0.0.1:{BRAIN}",
                    "-e", f"ROS_DOMAIN_ID={DOMAIN}", IMAGE, "ros2", "launch", "picar_bringup",
                    "picar.launch.py"], capture_output=True, check=True)
    states, out = [], {}
    while time.time() - t0 < 180:
        try:
            h = bridge.get("/health").json()
            pending = bridge.get("/slam/pose").json().get("start_truth_pending")
        except (httpx.HTTPError, ValueError):
            time.sleep(0.1)
            continue
        st = h["slam"]["state"]
        if not states or states[-1][0] != st:
            states.append((st, round(time.time() - t0, 2)))
        if st == "failed" or (st == "ready" and pending is False):
            out = {"ready_s": round(time.time() - t0, 2) if st == "ready" else None,
                   "reload": h["slam"].get("reload"), "lifelong": h["slam"].get("lifelong"),
                   "map_cap": h["slam"].get("map_cap")}
            break
        time.sleep(0.1)
    else:
        out = {"ready_s": None, "timeout": True}
    out["states"] = states
    try:
        e = httpx.get(f"http://127.0.0.1:{ROBOT}/world/error", timeout=5).json()
        out["error"] = {k: e.get(k) for k in ("usable", "position_error_m", "heading_error_deg")}
    except (httpx.HTTPError, ValueError):
        out["error"] = None
    subprocess.run(["docker", "logs", CONTAINER], stdout=open(log, "w"), stderr=subprocess.STDOUT)
    subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
    return out


def main(argv):
    out_path, house, specs = argv[0], argv[1], [a.split("=", 1) for a in argv[2:]]
    preflight()
    os.makedirs(WORK, exist_ok=True)
    res = {"house": house, "image": IMAGE, "host": os.uname().nodename, "stores": {}}
    try:
        for label, src in specs:
            runs = []
            for rep in range(1, REPS + 1):
                maps = os.path.join(WORK, f"{label}-{rep}")
                meta = store(src, maps)
                robot = robot_server(house, os.path.join(WORK, f"robot-{label}-{rep}.log"))
                try:
                    r = reload_once(maps, os.path.join(WORK, f"ros-{label}-{rep}.log"))
                finally:
                    stop(robot)
                    remove(maps)
                r["matched"] = (r.get("reload") or {}).get("result") == "matched"
                print(label, rep, r.get("ready_s"), (r.get("reload") or {}).get("result"), flush=True)
                runs.append(r)
            mb = round(sum(meta["files"].values()) / 1e6, 2)
            times = [r["ready_s"] for r in runs if r["matched"] and r["ready_s"] is not None]
            res["stores"][label] = {"map_id": meta["map_id"], "version": meta["version"], "graph_mb": mb,
                                    "runs": runs, "all_matched": len(times) == REPS,
                                    "slowest_s": max(times) if times else None}
            with open(out_path, "w") as f:          # after each store: a later failure keeps it
                json.dump(res, f, indent=1)
    finally:
        subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
    print(json.dumps({k: {x: v[x] for x in ("graph_mb", "all_matched", "slowest_s")}
                      for k, v in res["stores"].items()}))


if __name__ == "__main__":
    main(sys.argv[1:])
