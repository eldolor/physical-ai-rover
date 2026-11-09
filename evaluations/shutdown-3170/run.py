# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.170's live runs (docs/plans/ros-alignment/3.170-twin-shutdown-button.md).

A real robot server and brain per run, `frontier` in the simulator, read only
through their HTTP APIs. The power-off command is a harmless line appended to a
file: nothing here can power off this machine. One JSON line per run in
results.jsonl; the summary in summary.json. Run from the repo root, under the
exclusive desktop lock:

    python tools/desktop_lock.py exclusive --why "3.170 live" -- \
        python evaluations/shutdown-3170/run.py
"""
import concurrent.futures
import contextlib
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import httpx
import yaml

REPO = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO))
from tests.conftest import free_port  # noqa: E402

N = int(os.environ.get("RUNS", "10"))
SECRET = "live-run-secret-3170"
ABSENT = "purple unicorn"
BRAIN_APP = 'import os\nfrom control.brain_server import create_app\napp = create_app(os.environ["BRAIN_CONFIG"])\n'
AUTH = {"x-app-secret": SECRET}
PERSON = {"x-driver": "twin-dpad", **AUTH}


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
def stack(with_brain=True, brain_step=True):
    """A robot server and (optionally) a brain of this run's own. Yields the
    two URLs, the robot's process and the file the steps append to."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        marks = tmp / "marks"
        cfg = yaml.safe_load((REPO / "config" / "robot.yaml").read_text())
        (tmp / "robot.yaml").write_text(yaml.safe_dump(cfg))
        (tmp / "brain_app.py").write_text(BRAIN_APP)
        rp, bp = free_port(), free_port()
        brain_url = f"http://127.0.0.1:{bp}"
        stop_brain = (f"curl -sS -m 20 -o /dev/null -w '%{{http_code}}' -X POST -H 'x-app-secret: {SECRET}' "
                      f"{brain_url}/mission/stop | grep -q 200 && echo brain >> {marks}")
        env = {**os.environ, "PYTHONPATH": str(REPO), "ROBOT_CONFIG_PATH": str(tmp / "robot.yaml"),
               "BRAIN_CONFIG": str(tmp / "robot.yaml"), "ROBOT_URL": f"http://127.0.0.1:{rp}",
               "VISION_URL": "http://127.0.0.1:9", "INTERVENTION_DIR": str(tmp / "iv"),
               "APP_SHARED_SECRET": SECRET, "ROBOT_SHARED_SECRET": SECRET,
               "BATTERY_SHUTDOWN_CMDS": json.dumps([["brain", stop_brain]] if brain_step else []),
               "BATTERY_POWEROFF_CMD": f"echo poweroff >> {marks}"}
        log = open(tmp / "log", "w")
        robot = _serve(["robot.server:app", "--host", "127.0.0.1", "--port", str(rp)], env, REPO, log)
        brain = None
        try:
            _up(f"http://127.0.0.1:{rp}")
            if with_brain:
                brain = _serve(["brain_app:app", "--host", "127.0.0.1", "--port", str(bp), "--app-dir", str(tmp)],
                               env, tmp, log)
                _up(brain_url)
            yield f"http://127.0.0.1:{rp}", brain_url, robot, marks
        finally:
            for p in (brain, robot):
                if p is None:
                    continue
                with contextlib.suppress(ProcessLookupError):
                    p.kill()
                p.wait()
            log.close()


def press(robot, headers=PERSON):
    return httpx.post(robot + "/power/shutdown", headers=headers, timeout=15)


def request_of(robot):
    return httpx.get(robot + "/battery", headers=AUTH, timeout=5).json()["ladder"]


def wait_done(robot, bound=40):
    end = time.monotonic() + bound
    while time.monotonic() < end:
        lad = request_of(robot)
        if any(s["step"] == "poweroff" for s in lad["shutdown"]):
            return lad
        time.sleep(0.03)
    return request_of(robot)


def marks_of(path):
    return path.read_text().split() if path.exists() else []


def status(brain):
    return httpx.get(brain + "/mission/status", headers=AUTH, timeout=5).json()


def start(brain):
    r = httpx.post(brain + "/mission/start", headers=AUTH, timeout=30,
                   json={"target_object": ABSENT, "policy": "frontier"})
    r.raise_for_status()


def wait_steps(brain, n, timeout=20):
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


def emit(rec, out):
    out.write(json.dumps(rec) + "\n")
    out.flush()
    print(json.dumps(rec), flush=True)


def once(out):
    """Criterion 3: 20 at once, a retry after, on a live server."""
    with stack(with_brain=False, brain_step=False) as (robot, _, _, marks):
        with concurrent.futures.ThreadPoolExecutor(20) as pool:
            replies = list(pool.map(lambda _i: press(robot).json(), range(20)))
        lad = wait_done(robot)
        retry = press(robot).json()
        time.sleep(0.5)
        started = sum(r["started"] for r in replies)
        off = marks_of(marks).count("poweroff")
        emit({"crit": 3, "how": "burst_and_retry", "started": started, "retry": retry["started"],
              "poweroff_ran": off, "ok": started == 1 and retry["started"] is False and off == 1
              and lad["level"] == "ok"}, out)


def mission(out):
    """Criterion 4: pressed mid-mission. `cancelled` first, then the power-off."""
    with stack() as (robot, brain, _, marks):
        start(brain)
        wait_steps(brain, 3)
        pressed = press(robot).json()
        s = wait_end(brain)
        lad = wait_done(robot)
        before = odometry(robot)
        v0 = wheels(robot)
        time.sleep(1.0)
        after = odometry(robot)
        still = before == after and v0 == (0, 0)
        order = marks_of(marks)
        emit({"crit": 4, "how": "mid_mission", "outcome": s.get("outcome"), "steps": s.get("step"),
              "marks": order, "still": still, "phase": (lad["request"] or {}).get("phase"),
              "ok": pressed["started"] is True and s.get("outcome") == "cancelled"
              and order == ["brain", "poweroff"] and still}, out)


def brain_down(out):
    """Criterion 4: no brain at all. The step that needs it fails, on the record."""
    with stack(with_brain=False) as (robot, _, _, marks):
        r = press(robot)
        lad = wait_done(robot)
        rc = [s["rc"] for s in lad["shutdown"] if s["step"] == "brain"]
        emit({"crit": 4, "how": "brain_down", "status": r.status_code, "brain_rc": rc,
              "marks": marks_of(marks), "ok": r.status_code == 200 and len(rc) == 1 and rc[0] != 0
              and marks_of(marks) == ["poweroff"]}, out)


def phases(out):
    """Criterion 7, over HTTP: what the phone reads, then the server gone."""
    with stack(with_brain=False, brain_step=False) as (robot, _, proc, marks):
        idle = request_of(robot)["request"]
        bad = press(robot, {"x-driver": "twin-dpad", "x-app-secret": "wrong"}).status_code
        press(robot)
        lad = wait_done(robot)
        phase = lad["request"]["phase"]
        proc.send_signal(signal.SIGKILL)
        proc.wait()
        gone = False
        try:
            httpx.get(robot + "/battery", headers=AUTH, timeout=2)
        except httpx.HTTPError:
            gone = True
        emit({"crit": 7, "how": "phases", "idle": idle, "wrong_secret": bad, "phase": phase,
              "gone": gone, "marks": marks_of(marks),
              "ok": idle is None and bad == 401 and phase == "powering_off" and gone
              and marks_of(marks) == ["poweroff"]}, out)


def main():
    with open(OUT / "results.jsonl", "w") as out:
        for fn in (once, mission, brain_down, phases):
            for _ in range(N):
                fn(out)
    rows = [json.loads(line) for line in (OUT / "results.jsonl").read_text().splitlines()]
    summary = {}
    for r in rows:
        row = summary.setdefault(f"c{r['crit']}:{r['how']}", {"n": 0, "ok": 0, "outcomes": {}})
        row["n"] += 1
        row["ok"] += bool(r["ok"])
        if r.get("outcome"):
            row["outcomes"][r["outcome"]] = row["outcomes"].get(r["outcome"], 0) + 1
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary, indent=1))
    return 0 if all(v["ok"] == v["n"] for v in summary.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
