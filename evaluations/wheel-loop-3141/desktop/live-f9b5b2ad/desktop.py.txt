# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/wheel-loop-3141/desktop.py

3.141.A on the desktop: 3.136.B's robot-server stack (`steady.sh`) on this
section's ports, with the stall tracer (`tracer/sitecustomize.py`) in the
robot server only:

* the simulated body as its own programs (`SIM_MOTOR_BOARD=fake`, body and
  two sensor servers) and the fake D500 on a pty (`SIM_LIDAR=fake`), read
  by the robot server in `mode: hardware` with `drive: ros`;
* the ROS container (`vision-picar-ros:latest`, host network, this
  section's domain), SLAM + nav2;
* the brain on back-to-back tiered find missions in `procthor:val:12`
  (explore searching, the local tier on, `rooms` goal choice,
  `find_room_labels: frame`), the cloud and VE as $0 stand-ins;
* a 1 Hz `/world/map` poller (3.136's P12 load) and the robot server's
  `/health` `wheel_loop` every 5 s.

No YOLOE + CLIP and no real VE: the desktop has no Jetson's GPU. Run under
the desktop lock's exclusive hold (rule 7; it takes it itself):

    python evaluations/wheel-loop-3141/desktop.py <out_dir> [window_s]
"""

import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import httpx

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PY = sys.executable
sys.path.insert(0, str(ROOT / "tools"))

ROBOT = int(os.environ["PICAR_ROBOT_PORT"])
BRAIN = int(os.environ["PICAR_BRAIN_PORT"])
BODY = int(os.environ["PICAR_BODY_PORT"])
SENS = (int(os.environ["PICAR_SENSOR_PORT"]), int(os.environ["PICAR_SENSOR2_PORT"]))
BRIDGE = int(os.environ["PICAR_BRIDGE_PORT"])
CLOUD, VLM = BRIDGE + 1, BRIDGE + 2
CONTAINER = os.environ["PICAR_ROS_CONTAINER"]
DOMAIN = os.environ["ROS_DOMAIN_ID"]
# The board ran `:latest` (3.136.B); `plan_section.py env` names a per-branch
# tag this section never builds.
IMAGE = os.environ.get("TRACE_3141_IMAGE", "vision-picar-ros:latest")
HOUSE = os.environ.get("HOUSE", "procthor:val:12")
TARGETS = os.environ.get("TARGETS", "garbage can|house plant|laundry hamper|basketball|dog bed|"
                         "box|plunger|cart|garbage bag").split("|")


def wait(what, fn, timeout):
    end = time.time() + timeout
    while time.time() < end:
        try:
            if fn():
                return
        except Exception:  # noqa: BLE001 -- still coming up
            pass
        time.sleep(1)
    raise RuntimeError(f"{what} not up in {timeout} s")


def main():
    out = Path(sys.argv[1]).resolve()
    window = float(sys.argv[2]) if len(sys.argv) > 2 else 1800
    out.mkdir(parents=True, exist_ok=False)
    import desktop_lock
    desktop_lock.hold_for_life("exclusive", f"3.141.A desktop stall trace {out.name}")
    procs = []
    stop = threading.Event()

    def spawn(name, cmd, env=None):
        p = subprocess.Popen(cmd, cwd=ROOT, env=env or base,
                             stdout=open(out / f"{name}.log", "w"), stderr=subprocess.STDOUT,
                             start_new_session=True)
        procs.append(p)
        return p

    base = {k: v for k, v in os.environ.items()
            if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET", "PYTHONPATH")}
    lidar = out / "lidar.pty"
    base.update(SIM_MAP=HOUSE, ROBOT_DRIVE="ros", WORLD_MODE="ros", ROBOT_MODE="hardware",
                SIM_MOTOR_BOARD="fake", SIM_LIDAR="fake", ROBOT_LIDAR=str(lidar),
                SIM_BODY_URL=f"http://127.0.0.1:{BODY}",
                SIM_SENSORS_URL=",".join(f"http://127.0.0.1:{p}" for p in SENS),
                SIM_BODY_SHM=f"picar_sim_body_{DOMAIN}",
                ROS_BRIDGE_URL=f"http://127.0.0.1:{BRIDGE}",
                VISION_URL=f"http://127.0.0.1:{CLOUD}", LOCAL_VLM_URL=f"http://127.0.0.1:{VLM}",
                TIER_SEARCH="explore", FIND_ROOM_LABELS="frame", EXPLORE_GOAL_POLICY="rooms",
                INVENTORY_BUCKET="", PICAR_LOG_LEVEL="WARNING")
    try:
        subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
        spawn("cloud", [PY, str(ROOT / "evaluations/tiered-380/E/standin_cloud.py"), str(CLOUD),
                        str(out / "cloud_calls.jsonl")])
        spawn("vlm", [PY, str(HERE / "standin_vlm.py"), str(VLM), "1.0"])
        spawn("body", [PY, "-m", "uvicorn", "sim.body_server:app", "--port", str(BODY),
                       "--host", "127.0.0.1", "--log-level", "warning"])
        wait("body", lambda: httpx.get(f"http://127.0.0.1:{BODY}/health").status_code == 200, 60)
        for i, p in enumerate(SENS):
            spawn(f"sensor{i}", [PY, "-m", "uvicorn", "sim.sensor_server:app", "--port", str(p),
                                 "--host", "127.0.0.1", "--log-level", "warning"])
        spawn("lidar", [PY, "-m", "sim.fake_lidar", "--link", str(lidar)])
        wait("lidar pty", lidar.exists, 10)
        renv = dict(base, PYTHONPATH=str(HERE / "tracer"),
                    TRACE_3141_LOG=str(out / "trace.jsonl"))
        spawn("robot", [PY, "-m", "uvicorn", "robot.server:app", "--port", str(ROBOT),
                        "--host", "127.0.0.1", "--log-level", "warning"], renv)
        spawn("brain", [PY, "-m", "uvicorn", "control.brain_server:app", "--port", str(BRAIN),
                        "--host", "127.0.0.1", "--log-level", "warning"],
              dict(base, ROBOT_URL=f"http://127.0.0.1:{ROBOT}", PICAR_LOG_LEVEL="INFO"))
        robot = httpx.Client(base_url=f"http://127.0.0.1:{ROBOT}", timeout=30)
        wait("wheels", lambda: robot.get("/wheels").json().get("usable"), 90)
        wait("vlm", lambda: httpx.get(f"http://127.0.0.1:{VLM}/health").json()["ready"], 60)
        subprocess.run(["docker", "run", "-d", "--name", CONTAINER, "--network", "host",
                        "-e", f"BRIDGE_PORT={BRIDGE}", "-e", "BRIDGE_HOST=127.0.0.1",
                        "-e", f"ROBOT_URL=http://127.0.0.1:{ROBOT}",
                        "-e", f"BRAIN_URL=http://127.0.0.1:{BRAIN}",
                        "-e", f"ROS_DOMAIN_ID={DOMAIN}",
                        IMAGE,
                        "ros2", "launch", "picar_bringup", "picar.launch.py"],
                       capture_output=True, check=True)
        wait("bridge", lambda: httpx.get(f"http://127.0.0.1:{BRIDGE}/health").status_code == 200, 180)
        wait("start_truth", lambda: "start_truth" in httpx.get(
            f"http://127.0.0.1:{BRIDGE}/slam/pose").json(), 120)
        wait("map", lambda: robot.get("/world/map").json().get("usable"), 120)
        (out / "window_start").write_text(f"{time.time()}\n")

        def missions():
            b = httpx.Client(base_url=f"http://127.0.0.1:{BRAIN}", timeout=600)
            i = 0
            with open(out / "missions.jsonl", "a", buffering=1) as f:
                while not stop.is_set():
                    target = TARGETS[i % len(TARGETS)]
                    i += 1
                    t0 = time.time()
                    r = b.post("/mission/start", json={"target_object": target, "policy": "tiered"})
                    if r.status_code != 200:
                        f.write(json.dumps({"target": target, "start_status": r.status_code,
                                            "body": r.text[:300], "t0": t0}) + "\n")
                        stop.wait(5)
                        continue
                    s = {}
                    while True:
                        if stop.is_set():
                            b.post("/mission/stop")
                        s = b.get("/mission/status").json()
                        if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
                            break
                        time.sleep(1)
                    f.write(json.dumps({"target": target, "t0": round(t0, 3),
                                        "t1": round(time.time(), 3), "outcome": s.get("outcome"),
                                        "step": s.get("step"), "error": s.get("error")}) + "\n")

        def mappoll():
            etag = None
            with open(out / "mappoll.jsonl", "a", buffering=1) as f:
                while not stop.is_set():
                    t0 = time.time()
                    h = {"If-None-Match": etag} if etag else {}
                    r = robot.get("/world/map", headers=h)
                    etag = r.headers.get("etag", etag)
                    f.write(json.dumps({"t": round(t0, 3), "ms": round((time.time() - t0) * 1000, 1),
                                        "status": r.status_code, "bytes": len(r.content)}) + "\n")
                    stop.wait(1.0)

        def wheel():
            with open(out / "wheel.log", "a", buffering=1) as f:
                while not stop.is_set():
                    w = robot.get("/health").json()["wheel_loop"]
                    f.write(f"{time.time():.3f} {w['moving_ticks']} {w['late_ticks']} {w['max_dt_s']}\n")
                    stop.wait(5)

        threads = [threading.Thread(target=fn, daemon=True) for fn in (missions, mappoll, wheel)]
        for t in threads:
            t.start()
        time.sleep(window)
        (out / "window_end").write_text(f"{time.time()}\n")
        (out / "health_end.json").write_text(json.dumps(robot.get("/health").json(), indent=1))
        stop.set()
        for t in threads:
            t.join(timeout=60)
    finally:
        stop.set()
        with open(out / "ros.log", "w") as f:
            subprocess.run(["docker", "logs", CONTAINER], stdout=f, stderr=subprocess.STDOUT)
        subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
        for p in reversed(procs):
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        time.sleep(3)
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if lidar.exists() or lidar.is_symlink():
            lidar.unlink()
        shutil.copy(__file__, out / "desktop.py.txt")


if __name__ == "__main__":
    main()
