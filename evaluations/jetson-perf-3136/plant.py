"""
evaluations/jetson-perf-3136/plant.py

3.136 criterion 3, "the harness tells the truth": each timed row whose
measurer exists, run on the DESKTOP's local stack twice -- as it is, then
with 100 ms planted where the row measures -- through the same scorer the
board run uses (`scorers.py`). The row passes when the scored statistic
moves by 100 ms +- 10%.

| Row | Measured by | Planted where |
|---|---|---|
| P1/P2/P10 | perception_loop.py's frame lines, p90 | inside `perceive()` (plant_perception.py) |
| P4 | `/health` wheel_loop `max_dt_s`, late ticks | one wheel-loop tick blocks the event loop (plant_server.py) |
| P8 | bench_local_vlm.py's HTTP round trip, p90 | the VLM's reply (a stand-in service) |
| P12 map | the harness's `/world/map` timing, p90 | inside `world.get_map()` (plant_server.py) |

    python evaluations/jetson-perf-3136/plant.py [--out DIR]

Run it under the desktop lock, exclusive (timing): `run.py plant` does.
Ports: this section's (`plan_section.py env`), so it meets no other stack.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))

import scorers  # noqa: E402
import validity  # noqa: E402

PY = sys.executable
DELAY_MS = 100.0
TOL = 0.10
WHEEL_PERIOD_S = 0.05     # robot.server.WHEEL_LOOP_INTERVAL_S
# The smallest valid JPEG header frame_dict() reads a width from is not
# needed: the stand-in pipeline never decodes. Any bytes with a .jpg name do.
STUB_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEB"
    "AQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQH/wAALCAABAAEBAREA/8QAFAABAAAAAAAAAAAAAAAAAAAACf/EABQQ"
    "AQAAAAAAAAAAAAAAAAAAAAD/2gAIAQEAAD8AKp//2Q==")


def verdict(base: float, planted: float) -> dict:
    d = planted - base
    return {"base": round(base, 2), "planted": round(planted, 2), "delay_ms": round(d, 2),
            "error_pct": round(100 * (d - DELAY_MS) / DELAY_MS, 1),
            "met": abs(d - DELAY_MS) <= TOL * DELAY_MS}


# --------------------------------------------------------- P1 / P2 / P10

def check_perception(work: Path, seconds: float) -> dict:
    frames = json.loads((ROOT / "tools/jetson/bench_frames.json").read_text())
    for f in frames:
        p = work / "recordings" / f["walk"] / f["file"]
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(STUB_JPEG)
    runs = {}
    for name, plant in (("base", "0"), ("planted", "1")):
        t0 = time.time()
        r = subprocess.run([PY, str(HERE / "plant_perception.py"), str(seconds)], cwd=work,
                           capture_output=True, text=True, env={**os.environ, "PLANT": plant},
                           timeout=seconds + 120)
        t1 = time.time()
        (work / f"perception-{name}.log").write_text(r.stdout + r.stderr)
        ms = scorers.perception_ms(r.stdout, t0, t1)
        runs[name] = {"frames": len(ms), "p50": scorers.percentile(ms, 50),
                      "p90": scorers.percentile(ms, 90), "exit": r.returncode}
    if not (runs["base"]["frames"] and runs["planted"]["frames"]):
        return {"runs": runs, "met": False, "error": "no frames scored"}
    v = verdict(runs["base"]["p90"], runs["planted"]["p90"])
    return {"statistic": "p90 ms", "runs": runs, **v}


# ------------------------------------------------------ P4 and P12 (map)

class Server:
    def __init__(self, port: int, plant_dir: Path, log: Path):
        self.port, self.plant = port, plant_dir
        self.url = f"http://127.0.0.1:{port}"
        env = {**os.environ, "PLANT_DIR": str(plant_dir), "ROBOT_MODE": "sim", "WORLD_MODE": "sim",
               "SIM_MAP": "scaled_house"}
        for k in ("APP_SHARED_SECRET", "ROBOT_DRIVE", "SIM_MOTOR_BOARD", "SIM_LIDAR"):
            env.pop(k, None)    # the plain in-process sim, open (require_secret inert)
        self.proc = subprocess.Popen([PY, str(HERE / "plant_server.py"), str(port)], env=env,
                                     stdout=open(log, "w"), stderr=subprocess.STDOUT,
                                     start_new_session=True)
        end = time.time() + 60
        while time.time() < end:
            try:
                self.get("/health")
                return
            except OSError:
                if self.proc.poll() is not None:
                    raise RuntimeError(f"plant server exited {self.proc.returncode}; see {log}")
                time.sleep(0.5)
        raise RuntimeError("plant server did not come up")

    def get(self, path: str) -> dict:
        with urllib.request.urlopen(self.url + path, timeout=10) as r:
            return json.loads(r.read())

    def post(self, path: str, body: dict) -> dict:
        req = urllib.request.Request(self.url + path, data=json.dumps(body).encode(),
                                     headers={"content-type": "application/json",
                                              "x-driver": "brain"})
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read())

    def close(self):
        try:
            os.killpg(self.proc.pid, signal.SIGTERM)
            self.proc.wait(timeout=10)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def _turning(srv: Server, stop: threading.Event) -> None:
    """A standing pivot, refreshed inside the 1 s watchdog: the wheel loop
    counts only moving ticks, and a pivot is never clamped by a wall."""
    while not stop.is_set():
        try:
            srv.post("/wheels", {"left_rad_s": -2.0, "right_rad_s": 2.0})
        except OSError:
            pass
        stop.wait(0.3)
    try:
        srv.post("/wheels", {"left_rad_s": 0.0, "right_rad_s": 0.0})
    except OSError:
        pass


def check_wheel(srv: Server, settle_s: float = 10.0) -> dict:
    stop = threading.Event()
    th = threading.Thread(target=_turning, args=(srv, stop), daemon=True)
    th.start()
    try:
        time.sleep(2.0)
        h0 = srv.get("/health")
        time.sleep(settle_s)
        h1 = srv.get("/health")
        base = scorers.wheel_delta(h0, h1)
        (srv.plant / "wheel").write_text("1")
        end = time.time() + 10
        while (srv.plant / "wheel").exists() and time.time() < end:
            time.sleep(0.05)
        time.sleep(1.0)
        h2 = srv.get("/health")
        planted = scorers.wheel_delta(h1, h2)
    finally:
        stop.set()
        th.join(timeout=5)
    # The validity rule must refuse the planted window (one late tick).
    flagged = validity.judge({"kind": "full", "window": [0, 1], "bench": [0, 1],
                              "wheel": planted, "oom_kills": 0, "nav": {"ok": 1, "n": 1},
                              "power_mode": "NV Power Mode: 15W", "desktop_load": [1],
                              "temps": 1, "clocks": 1})
    # max_dt_s is a running maximum since the server started, so the delay
    # is read against the loop's nominal period: the planted tick's gap is
    # the period plus the block. It must also be the tick that set the max.
    v = verdict(WHEEL_PERIOD_S * 1000, planted["max_dt_s"] * 1000)
    return {"statistic": "max_dt_s (ms)", "base_window": base, "planted_window": planted,
            "planted_late_ticks": planted["late"], "base_late_ticks": base["late"],
            "validity_flags_planted": not flagged["valid"], "validity_reasons": flagged["reasons"],
            **v, "met": v["met"] and planted["late"] >= 1 and base["late"] == 0
            and planted["max_dt_s"] > base["max_dt_s"] and not flagged["valid"]}


def check_map(srv: Server, seconds: float) -> dict:
    runs = {}
    for name in ("base", "planted"):
        if name == "planted":
            (srv.plant / "map").write_text("1")
        poller = scorers.MapPoller(srv.url, period_s=0.1)
        poller.start()
        time.sleep(seconds)
        runs[name] = scorers.map_summary(poller.stop())
        (srv.plant / "map").unlink(missing_ok=True)
    v = verdict(runs["base"]["p90_ms"], runs["planted"]["p90_ms"])
    return {"statistic": "p90 ms", "runs": runs, **v}


# ------------------------------------------------------------------- P8

class StubVLM(BaseHTTPRequestHandler):
    delay_s = 0.0

    def do_POST(self):  # noqa: N802
        self.rfile.read(int(self.headers.get("content-length", 0)))
        if StubVLM.delay_s:
            time.sleep(StubVLM.delay_s)
        body = json.dumps({"status": "answered", "score": 0.9, "driver_ms": 1.0}).encode()
        self.send_response(200)
        self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


def check_vlm(work: Path, port: int, n_frames: int = 220) -> dict:
    walk = work / "vlm-corpus" / "stub-walk"
    walk.mkdir(parents=True, exist_ok=True)
    names = [f"frame-{i:04d}.jpg" for i in range(n_frames)]
    for n in names:
        (walk / n).write_bytes(STUB_JPEG)
    (walk / "labels.json").write_text(json.dumps({
        "description": "a stand-in target", "target_visible_labels": {n: i % 2 == 0 for i, n in enumerate(names)},
        "adjudicated": names}))
    httpd = ThreadingHTTPServer(("127.0.0.1", port), StubVLM)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    runs = {}
    try:
        for name, delay in (("base", 0.0), ("planted", DELAY_MS / 1000)):
            StubVLM.delay_s = delay
            out = work / f"vlm-{name}.json"
            r = subprocess.run([PY, "-m", "tools.jetson.bench_local_vlm", "frames",
                                "--url", f"http://127.0.0.1:{port}", "--recordings", str(walk.parent),
                                "--walk", walk.name, "--out", str(out)],
                               cwd=ROOT, capture_output=True, text=True, timeout=600)
            if r.returncode != 0:
                return {"met": False, "error": r.stderr[-800:]}
            runs[name] = scorers.vlm_latency(json.loads(out.read_text()))
    finally:
        httpd.shutdown()
    v = verdict(runs["base"]["p90"], runs["planted"]["p90"])
    return {"statistic": "p90 ms", "runs": runs, **v}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(HERE / "desktop"))
    ap.add_argument("--seconds", type=float, default=60.0, help="each perception run (>= 200 frames at 4 Hz)")
    ap.add_argument("--map-seconds", type=float, default=25.0)
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    port = int(os.environ.get("PICAR_ROBOT_PORT", "10360"))
    vlm_port = int(os.environ.get("PICAR_SENSOR2_PORT", "10364"))
    work = Path(tempfile.mkdtemp(prefix="plant-3136-"))
    load0 = os.getloadavg()
    res = {"started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
           "rev": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                                 text=True).stdout.strip(),
           "delay_ms": DELAY_MS, "tolerance": TOL, "rows": {}}
    try:
        res["rows"]["P1/P2/P10 perception"] = check_perception(work, a.seconds)
        print("perception", json.dumps(res["rows"]["P1/P2/P10 perception"]), flush=True)
        plant_dir = work / "plant"
        plant_dir.mkdir()
        srv = Server(port, plant_dir, work / "server.log")
        try:
            res["rows"]["P4 wheel loop"] = check_wheel(srv)
            print("wheel", json.dumps(res["rows"]["P4 wheel loop"]), flush=True)
            res["rows"]["P12 map reply"] = check_map(srv, a.map_seconds)
            print("map", json.dumps(res["rows"]["P12 map reply"]), flush=True)
        finally:
            srv.close()
        res["rows"]["P8 local VLM"] = check_vlm(work, vlm_port)
        print("vlm", json.dumps(res["rows"]["P8 local VLM"]), flush=True)
    finally:
        res["desktop_load"] = {"start": load0, "end": os.getloadavg()}
        res["met"] = bool(res["rows"]) and all(r.get("met") for r in res["rows"].values()) \
            and len(res["rows"]) == 4
        (out / "planted.json").write_text(json.dumps(res, indent=1))
        shutil.rmtree(work, ignore_errors=True)
    print("criterion 3 (rows built in 3.136.A):", "MET" if res["met"] else "FAILED")
    return 0 if res["met"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
