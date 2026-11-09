# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.171's live runs (docs/plans/ros-alignment/3.171-small-stop-defects.md, 3.171.D).

A real robot server and brain per run, `frontier` in the simulator, read only
through their HTTP APIs, the shipped config (a silence reads as a hang after
24 s). The power-off command is a line appended to a file: nothing here can
power off this machine. Between the brain and the robot sits a small proxy that
can lose chosen requests, by dropping the connection with no reply. One JSON
line per run in results.jsonl; the summary in summary.json. Run from the repo
root, under the exclusive desktop lock:

    python tools/desktop_lock.py exclusive --why "3.171 live" -- \
        python evaluations/small-stop-3171/run.py
"""
import collections
import contextlib
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import httpx
import yaml

REPO = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO))
from tests.conftest import free_port  # noqa: E402

RUNS = {"local_brain_power_off": int(os.environ.get("RUNS", "10")),
        "lost_final_stop": int(os.environ.get("RUNS", "10")),
        "normal_end": int(os.environ.get("CONTROL_RUNS", "3")),
        "control_close_lost_too": int(os.environ.get("CONTROL_RUNS", "3"))}
SECRET = "live-run-secret-3171"
ABSENT = "purple unicorn"
BRAIN_APP = 'import os\nfrom control.brain_server import create_app\napp = create_app(os.environ["BRAIN_CONFIG"])\n'
AUTH = {"x-app-secret": SECRET}
LOCAL = {"x-driver": "twin-local-brain", **AUTH}
STOP = ("POST", "/stop")
CLOSE = ("POST", "/autonomy/mission-closed")
TRIP_READ_S = 26.0      # the shipped trip_after_s (24 s), plus 2 s
SPIN = {"left_rad_s": -1.0, "right_rad_s": 1.0}
REST = {"left_rad_s": 0.0, "right_rad_s": 0.0}


class Proxy:
    """Forwards the brain's calls to the robot server, losing the ones in `lose`."""

    def __init__(self, target):
        self.target = target
        self.lose = set()
        self.lost = collections.Counter()
        self.closed_replies = []
        self.client = httpx.Client(timeout=30)
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *args):
                pass

            def forward(self):
                body = self.rfile.read(int(self.headers.get("content-length") or 0))
                key = (self.command, self.path.split("?")[0])
                if key in outer.lose:
                    outer.lost[key] += 1
                    self.close_connection = True
                    self.connection.close()
                    return
                headers = {k: v for k, v in self.headers.items()
                           if k.lower() not in ("host", "content-length", "connection")}
                r = outer.client.request(self.command, outer.target + self.path,
                                         content=body, headers=headers)
                if key == CLOSE:
                    outer.closed_replies.append(r.json().get("closed"))
                self.send_response(r.status_code)
                self.send_header("content-type", r.headers.get("content-type", "application/json"))
                self.send_header("content-length", str(len(r.content)))
                self.end_headers()
                self.wfile.write(r.content)

            do_GET = do_POST = do_DELETE = do_PUT = forward

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = True
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.client.close()


def _serve(args, env, cwd, log):
    return subprocess.Popen([sys.executable, "-m", "uvicorn", *args, "--log-level", "warning"],
                            cwd=cwd, env=env, stdout=log, stderr=log)


def _up(url):
    end = time.monotonic() + 60
    while time.monotonic() < end:
        with contextlib.suppress(httpx.HTTPError):
            if httpx.get(url + "/health", timeout=1).status_code == 200:
                return
        time.sleep(0.05)
    raise RuntimeError(f"{url} never answered")


@contextlib.contextmanager
def stack(env_extra=None):
    """A robot server, a proxy and a brain that reaches the robot through it."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        (tmp / "robot.yaml").write_text((REPO / "config" / "robot.yaml").read_text())
        (tmp / "brain_app.py").write_text(BRAIN_APP)
        rp, bp = free_port(), free_port()
        robot_url, brain_url = f"http://127.0.0.1:{rp}", f"http://127.0.0.1:{bp}"
        proxy = Proxy(robot_url)
        env = {**os.environ, "PYTHONPATH": str(REPO), "ROBOT_CONFIG_PATH": str(tmp / "robot.yaml"),
               "BRAIN_CONFIG": str(tmp / "robot.yaml"), "ROBOT_URL": proxy.url,
               "VISION_URL": "http://127.0.0.1:9", "INTERVENTION_DIR": str(tmp / "iv"),
               "APP_SHARED_SECRET": SECRET, "ROBOT_SHARED_SECRET": SECRET,
               "BATTERY_SHUTDOWN_CMDS": "[]",
               "BATTERY_POWEROFF_CMD": f"echo poweroff >> {tmp / 'marks'}", **(env_extra or {})}
        log = open(tmp / "log", "w")
        procs = [_serve(["robot.server:app", "--host", "127.0.0.1", "--port", str(rp)], env, REPO, log)]
        try:
            _up(robot_url)
            procs.append(_serve(["brain_app:app", "--host", "127.0.0.1", "--port", str(bp),
                                 "--app-dir", str(tmp)], env, tmp, log))
            _up(brain_url)
            yield robot_url, brain_url, proxy
        finally:
            for p in reversed(procs):
                with contextlib.suppress(ProcessLookupError):
                    p.kill()
                p.wait()
            log.close()
            proxy.close()


def status(brain):
    return httpx.get(brain + "/mission/status", headers=AUTH, timeout=5).json()


def start(brain):
    r = httpx.post(brain + "/mission/start", headers=AUTH, timeout=30,
                   json={"target_object": ABSENT, "policy": "frontier"})
    r.raise_for_status()


def wait_steps(brain, n, timeout=30):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s = status(brain)
        if s.get("step", 0) >= n:
            return s
        time.sleep(0.03)
    raise RuntimeError(f"never reached step {n}: {s}")


def wait_end(brain, timeout=60):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s = status(brain)
        if not s.get("running"):
            return s
        time.sleep(0.05)
    raise RuntimeError("the mission never ended")


def wheels(robot):
    w = httpx.get(robot + "/wheels", headers=AUTH, timeout=5).json()
    return w["left"]["velocity_rad_s"], w["right"]["velocity_rad_s"]


def odometry(robot):
    return httpx.get(robot + "/odometry", headers=AUTH, timeout=5).json()


def watchdog(robot):
    return httpx.get(robot + "/health", headers=AUTH, timeout=5).json()["watchdog"]


def ladder(robot):
    return httpx.get(robot + "/battery", headers=AUTH, timeout=5).json()["ladder"]


def post_wheels(robot, body):
    return httpx.post(robot + "/wheels", json=body, headers=LOCAL, timeout=5).json()


def emit(rec, out):
    rec["load_1m"] = round(os.getloadavg()[0], 2)
    out.write(json.dumps(rec) + "\n")
    out.flush()
    print(json.dumps(rec), flush=True)


def local_brain_power_off(out):
    """Criteria A1 and A4: a mission drains the pack to Power-off, then a local
    brain tries the wheels. While the pack is good the same call turns them."""
    with stack({"SIM_BATTERY_V": "11.5", "SIM_BATTERY_DRAIN_V_PER_S": "0.1"}) as (robot, brain, _):
        before = post_wheels(robot, SPIN)
        turning = wheels(robot)
        post_wheels(robot, REST)
        start(brain)
        wait_steps(brain, 3)
        s = wait_end(brain, 90)
        end = time.monotonic() + 60
        while time.monotonic() < end and ladder(robot)["level"] != "poweroff":
            time.sleep(0.1)
        end = time.monotonic() + 40
        while time.monotonic() < end and not any(st["step"] == "poweroff"
                                                 for st in ladder(robot)["shutdown"]):
            time.sleep(0.05)
        level = ladder(robot)["level"]
        odo0 = odometry(robot)
        refused, spun = 0, 0
        for _ in range(20):
            r = post_wheels(robot, SPIN)
            refused += r.get("executed") is False and r.get("reason") == "battery_poweroff"
            spun += wheels(robot) != (0, 0)
        time.sleep(1.0)
        still = odometry(robot) == odo0
        zero = post_wheels(robot, REST)
        emit({"how": "local_brain_power_off", "level": level, "refused": refused, "spun": spun,
              "still": still, "zero_accepted": zero.get("executed"), "ok_pack_executed": before.get("executed"),
              "ok_pack_turning": turning != (0, 0), "outcome": s.get("outcome"), "steps": s.get("step"),
              "ok": level == "poweroff" and refused == 20 and spun == 0 and still
              and zero.get("executed") is True and before.get("executed") is True
              and turning != (0, 0) and s.get("step", 0) > 0}, out)


def _ended(how, lose, out, want_trip=False):
    with stack() as (robot, brain, proxy):
        start(brain)
        wait_steps(brain, 3)
        proxy.lose |= lose
        live_before = watchdog(robot)["mission_live"]
        assert httpx.post(brain + "/mission/stop", headers=AUTH, timeout=30).status_code == 200
        s = wait_end(brain)
        time.sleep(TRIP_READ_S)
        wd = watchdog(robot)
        v = wheels(robot)
        closed = proxy.closed_replies
        tripped = wd["trip"] is not None
        rec = {"how": how, "outcome": s.get("outcome"), "steps": s.get("step"),
               "live_before": live_before, "lost_stops": proxy.lost[STOP], "lost_closes": proxy.lost[CLOSE],
               "close_replies": closed, "mission_live": wd["mission_live"], "trip": wd["trip"],
               "wheels": v, "trip_after_s": wd["trip_after_s"], "last_fired": wd["last_fired"]}
        ok = s.get("step", 0) > 0 and v == (0, 0)
        if want_trip:
            ok = ok and tripped and wd["mission_live"] is True and proxy.lost[CLOSE] > 0
        else:
            ok = ok and not tripped and wd["mission_live"] is False and closed == [True]
            if STOP in lose:
                ok = ok and proxy.lost[STOP] > 0
        rec["ok"] = bool(ok)
        emit(rec, out)


def lost_final_stop(out):
    """Criterion B1: the defect, through the brain's HTTP API."""
    _ended("lost_final_stop", {STOP}, out)


def normal_end(out):
    """Criterion B6: nothing lost: it ends as it did, one close."""
    _ended("normal_end", set(), out)


def control_close_lost_too(out):
    """The harness can see the defect: with the close lost too, the trip comes."""
    _ended("control_close_lost_too", {STOP, CLOSE}, out, want_trip=True)


def main():
    with open(OUT / "results.jsonl", "w") as out:
        for fn in (local_brain_power_off, lost_final_stop, normal_end, control_close_lost_too):
            for _ in range(RUNS[fn.__name__]):
                fn(out)
    rows = [json.loads(line) for line in (OUT / "results.jsonl").read_text().splitlines()]
    summary = {}
    for r in rows:
        row = summary.setdefault(r["how"], {"n": 0, "ok": 0, "outcomes": {}, "max_load_1m": 0.0})
        row["n"] += 1
        row["ok"] += bool(r["ok"])
        row["outcomes"][r["outcome"]] = row["outcomes"].get(r["outcome"], 0) + 1
        row["max_load_1m"] = max(row["max_load_1m"], r["load_1m"])
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary, indent=1))
    return 0 if all(v["ok"] == v["n"] for v in summary.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
