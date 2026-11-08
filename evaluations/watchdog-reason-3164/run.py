# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.164's live runs (docs/plans/ros-alignment/3.164-watchdog-keeps-stop-reason.md).

A real robot server and brain per run, `frontier` in the simulator, read only
through their HTTP APIs. One JSON line per run in results.jsonl; the summary in
summary.json. Run from the repo root, under the exclusive desktop lock:

    python tools/desktop_lock.py exclusive --why "3.164 live" -- \
        python evaluations/watchdog-reason-3164/run.py
"""
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
HOLD_S = 10.0
ABSENT = "purple unicorn"
BRAIN_APP = 'import os\nfrom control.brain_server import create_app\napp = create_app(os.environ["BRAIN_CONFIG"])\n'
PERSON = {"x-driver": "twin-dpad"}


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
def stack(tick_timeout=3.0, tick_interval=0.2):
    """A robot server and a brain of this run's own, one config for both."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        cfg = yaml.safe_load((REPO / "config" / "robot.yaml").read_text())
        cfg["brain"].update(tick_timeout_s=tick_timeout, tick_interval_s=tick_interval)
        (tmp / "robot.yaml").write_text(yaml.safe_dump(cfg))
        (tmp / "brain_app.py").write_text(BRAIN_APP)
        rp, bp = free_port(), free_port()
        env = {**os.environ, "PYTHONPATH": str(REPO), "ROBOT_CONFIG_PATH": str(tmp / "robot.yaml"),
               "BRAIN_CONFIG": str(tmp / "robot.yaml"), "ROBOT_URL": f"http://127.0.0.1:{rp}",
               "VISION_URL": "http://127.0.0.1:9", "INTERVENTION_DIR": str(tmp / "iv")}
        env.pop("APP_SHARED_SECRET", None)
        log = open(tmp / "log", "w")
        robot = _serve(["robot.server:app", "--host", "127.0.0.1", "--port", str(rp)], env, REPO, log)
        brain = None
        try:
            _up(f"http://127.0.0.1:{rp}")
            brain = _serve(["brain_app:app", "--host", "127.0.0.1", "--port", str(bp), "--app-dir", str(tmp)],
                           env, tmp, log)
            _up(f"http://127.0.0.1:{bp}")
            yield f"http://127.0.0.1:{rp}", f"http://127.0.0.1:{bp}", brain
        finally:
            for p in (brain, robot):
                if p is None:
                    continue
                with contextlib.suppress(ProcessLookupError):
                    p.send_signal(signal.SIGCONT)
                    p.kill()
                p.wait()
            log.close()


def h(robot):
    return httpx.get(robot + "/health", timeout=5).json()


def status(brain):
    return httpx.get(brain + "/mission/status", timeout=5).json()


def start(brain, target=ABSENT, fault="none", **kw):
    r = httpx.post(brain + "/mission/start", json={"target_object": target, "policy": "frontier",
                                                   "fault": fault, **kw}, timeout=30)
    r.raise_for_status()


def wait_steps(brain, n, timeout=20):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s = status(brain)
        if s.get("step", 0) >= n:
            return s
        time.sleep(0.03)
    raise RuntimeError(f"never reached step {n}: {s}")


def wait_end(brain, timeout=120):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s = status(brain)
        if not s.get("running"):
            return s
        time.sleep(0.05)
    raise RuntimeError("the mission never ended")


def wait_trip(robot, bound):
    end = time.monotonic() + bound
    while time.monotonic() < end:
        wd = h(robot)["watchdog"]
        if wd["trip"]:
            return wd
        time.sleep(0.05)
    return h(robot)["watchdog"]


def emit(rec, out):
    out.write(json.dumps(rec) + "\n")
    out.flush()
    print(json.dumps(rec), flush=True)


def hang(kind, out, tick_timeout=3.0):
    with stack(tick_timeout) as (robot, brain, proc):
        trip_after = h(robot)["watchdog"]["trip_after_s"]
        start(brain, fault="tick_hang" if kind == "tick_hang" else "none")
        if kind != "tick_hang":
            wait_steps(brain, 3)
        signalled = time.monotonic()
        if kind == "sigstop":
            proc.send_signal(signal.SIGSTOP)
        elif kind == "sigkill":
            proc.kill()
        wd = wait_trip(robot, trip_after + 2.0 + (1.0 if kind == "tick_hang" else 0.0))
        rec = {"crit": 1, "kind": kind, "tick_timeout_s": tick_timeout, "trip": wd["trip"],
               "after_signal_s": round(time.monotonic() - signalled, 2),
               "ok": bool(wd["trip"] and wd["trip"]["driver"] == "brain")}
        if kind == "tick_hang":
            s = wait_end(brain, 30)
            time.sleep(5.0)
            wd = h(robot)["watchdog"]
            rec.update(outcome=s.get("outcome"), trip_5s_after_end=wd["trip"],
                       during_after=(wd["last_fired"] or {}).get("during"),
                       ok=rec["ok"] and bool(wd["trip"]) and not wd["mission_live"])
        emit(rec, out)


def end_calm(how, out):
    with stack() as (robot, brain, _):
        if how == "found":
            start(brain, target="red backpack")
        elif how == "budget":
            start(brain, max_steps=8)
        else:
            start(brain)
            wait_steps(brain, 3)
            if how == "brain_stop":
                httpx.post(brain + "/mission/stop", timeout=30)
            else:
                httpx.post(robot + "/stop", headers=PERSON, timeout=10)
        s = wait_end(brain)
        time.sleep(3.0)
        body = h(robot)
        wd = body["watchdog"]
        lf = wd["last_fired"] or {}
        emit({"crit": 2, "how": how, "outcome": s.get("outcome"), "steps": s.get("step"),
              "trip": wd["trip"], "live": wd["mission_live"], "during": lf.get("during"),
              "last_stop_reason": (body["last_stop_reason"] or {}).get("reason"),
              "ok": wd["trip"] is None and not wd["mission_live"] and lf.get("during") is None
              and (s.get("step") or 0) > 0}, out)


def healthy_wait(out):
    """Pauses over the watchdog (1 s), under trip_after_s (2.4 s)."""
    with stack(tick_interval=1.5) as (robot, brain, _):
        start(brain, max_steps=8)
        seen = []
        end = time.monotonic() + 40
        while time.monotonic() < end and status(brain).get("running"):
            lf = h(robot)["watchdog"]["last_fired"]
            if lf:
                seen.append(lf["during"])
            time.sleep(0.2)
        s = wait_end(brain, 30)
        time.sleep(3.0)
        wd = h(robot)["watchdog"]
        emit({"crit": 2, "how": "healthy_wait", "outcome": s.get("outcome"), "steps": s.get("step"),
              "fired_mission": seen.count("mission"), "trip": wd["trip"], "live": wd["mission_live"],
              "ok": wd["trip"] is None and "mission" in seen and (s.get("step") or 0) > 0}, out)


def _hold_and_read(robot, reason, out, how):
    """Until the watchdog's record has replaced the refusal, then HOLD_S more."""
    end = time.monotonic() + 3.0
    while time.monotonic() < end and (h(robot)["last_refusal"] or {}).get("reason") != "watchdog":
        time.sleep(0.05)
    time.sleep(HOLD_S)
    body = h(robot)
    emit({"crit": 3, "how": how, "last_refusal": (body["last_refusal"] or {}).get("reason"),
          "last_stop_reason": (body["last_stop_reason"] or {}).get("reason"),
          "ago": (body["last_stop_reason"] or {}).get("seconds_ago"),
          "ok": (body["last_refusal"] or {}).get("reason") == "watchdog"
          and (body["last_stop_reason"] or {}).get("reason") == reason}, out)


def reason_person(out):
    with stack() as (robot, brain, _):
        start(brain)
        wait_steps(brain, 3)
        httpx.post(robot + "/stop", headers=PERSON, timeout=10)
        wait_end(brain)
        _hold_and_read(robot, "person_stop", out, "person_stop")


def reason_veto(out):
    with stack() as (robot, _, _b):
        for _ in range(40):
            r = httpx.post(robot + "/action", json={"action": "FORWARD"}, headers=PERSON, timeout=20).json()
            if r.get("reason") == "safety_distance":
                break
        _hold_and_read(robot, "safety_distance", out, "veto")


def reason_lapse(out):
    env = {**os.environ, "WATCHDOG_REASON_HOLD_S": str(HOLD_S)}
    rc = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                         "tests/test_watchdog_reason.py", "-k", "a_lapse_is_a_trip"],
                        cwd=REPO, env=env, capture_output=True).returncode
    emit({"crit": 3, "how": "lapse", "ok": rc == 0}, out)


def main():
    with open(OUT / "results.jsonl", "w") as out:
        for kind in ("tick_hang", "sigstop", "sigkill"):
            for _ in range(N):
                hang(kind, out)
        hang("sigstop", out, tick_timeout=30.0)
        for how in ("found", "budget", "brain_stop", "person_stop"):
            for _ in range(5):
                end_calm(how, out)
        healthy_wait(out)
        for fn in (reason_person, reason_veto, reason_lapse):
            for _ in range(N):
                fn(out)
    rows = [json.loads(line) for line in (OUT / "results.jsonl").read_text().splitlines()]
    summary = {}
    for r in rows:
        key = f"c{r['crit']}:{r.get('kind') or r.get('how')}" + (":shipped" if r.get("tick_timeout_s") == 30.0 else "")
        row = summary.setdefault(key, {"n": 0, "ok": 0, "outcomes": {}})
        row["n"] += 1
        row["ok"] += bool(r["ok"])
        if r.get("outcome"):
            row["outcomes"][r["outcome"]] = row["outcomes"].get(r["outcome"], 0) + 1
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary, indent=1))
    return 0 if all(v["ok"] == v["n"] for v in summary.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
