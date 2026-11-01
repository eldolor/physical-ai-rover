"""
3.133.C, criterion 8 live: goals and an explore mission sent from the first
second of a container start, on `vision-picar-ros:latest` rebuilt from `dev`.

    eval "$(python tools/plan_section.py env)"
    ROS_IMAGE=vision-picar-ros:latest PICAR_ROS_NETWORK=host \
        python evaluations/bridge-hold-3133/live.py [starts] [--no-mission]
    python evaluations/bridge-hold-3133/live.py --score <result.json>

Each start is `tests/demo_explore.stack()` as it is (a fresh robot server,
brain and container on an empty map store; it takes the desktop lock's
exclusive hold), run on a thread. From the second the container reads
running, the main thread posts POST /world/goal through the robot server
and again on every refusal until one is accepted, recording each reply
beside the bridge's /health `nav2` and `slam` blocks; the accepted goal is
followed to its end. The mission start does the same with the brain's
/mission/start (explore, the red backpack), retried until it is taken.

Adapted from evaluations/nav2-gate-3105/live.py (which posts only after a
reload reads ready). The goal is 3.105's point (scaled_house, 1-2 m from
the dock). Records to `evaluations/bridge-hold-3133/result.json` after
every start.
"""
import json
import os
import subprocess
import sys
import threading
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)
HERE = os.path.dirname(os.path.abspath(__file__))

import httpx  # noqa: E402

from tests import demo_explore as de  # noqa: E402

HOUSE = "scaled_house"
GOAL = (4.35, 3.15)                   # 3.105's goal (plan-3.105.json)
HOLD_S = 240.0                        # give up posting this long after the container runs
GOAL_S = 120.0
NO_SESSION = "no SLAM session yet"    # the robot server's own, before ready (3.127 Known limits)
HOLDS = ("nav2_starting", "slam_not_ready")


def started_at(after):
    """The container's start, once it is running and newer than `after`."""
    r = subprocess.run(["docker", "inspect", "-f", "{{.State.Running}} {{.State.StartedAt}}",
                        de.CONTAINER], capture_output=True, text=True)
    if r.returncode != 0:
        return None
    running, iso = r.stdout.split()
    if running != "true":
        return None
    import datetime
    ts = datetime.datetime.fromisoformat(iso[:26].rstrip("Z") + "+00:00").timestamp()
    return ts if ts > after else None


def launch():
    """stack() on a thread; returns (thread, container start time)."""
    box = {}

    def go():
        try:
            box["robot"] = de.stack(HOUSE)
        except Exception as e:  # noqa: BLE001 -- recorded by the caller
            box["error"] = f"{type(e).__name__}: {e}"

    t_launch = time.time()
    th = threading.Thread(target=go, daemon=True)
    th.start()
    while True:
        t0 = started_at(t_launch - 1)
        if t0 is not None:
            return th, t0, box
        if "error" in box:
            raise RuntimeError(box["error"])
        time.sleep(0.05)


def bridge_health(bridge):
    try:
        h = bridge.get("/health", timeout=2).json()
        s = h.get("slam") or {}
        return {"nav2": h.get("nav2"), "slam": {k: s.get(k) for k in ("state", "reload", "why")}}
    except Exception as e:  # noqa: BLE001 -- the bridge is not up yet
        return {"error": type(e).__name__}


def goal_from_start(robot, bridge, t0):
    out = {"refusals": [], "health_log": []}
    last_h = None
    while time.time() - t0 < HOLD_S:
        h = bridge_health(bridge)
        key = json.dumps(h, sort_keys=True, default=str)
        if key != last_h:
            out["health_log"].append({"t": round(time.time() - t0, 2), **h})
            last_h = key
        try:
            r = robot.post("/world/goal", json={"x_m": GOAL[0], "y_m": GOAL[1]}, timeout=10)
            body = r.json()
        except Exception as e:  # noqa: BLE001 -- the robot server is still starting
            out["refusals"].append({"t": round(time.time() - t0, 2), "error": type(e).__name__})
            time.sleep(0.25)
            continue
        if r.status_code == 200 and body.get("accepted"):
            out["accepted_s"] = round(time.time() - t0, 2)
            out["health_at_accept"] = h
            break
        out["refusals"].append({"t": round(time.time() - t0, 2), "status": r.status_code,
                                "reason": body.get("reason"), "retryable": body.get("retryable"),
                                "bridge_status": body.get("bridge_status"),
                                "detail": (body.get("detail") or "")[:200], "health": h})
        time.sleep(0.25)
    else:
        out["state"] = "never_accepted"
        return out
    t1 = time.time()
    state = "pending"
    while time.time() - t1 < GOAL_S:
        state = (robot.get("/world/goal").json().get("goal") or {}).get("state", "none")
        if state in ("succeeded", "aborted", "canceled", "rejected"):
            break
        time.sleep(0.2)
    else:
        robot.delete("/world/goal")
    out["state"], out["goal_s"] = state, round(time.time() - t1, 1)
    return out


class HealthWatch:
    """The bridge's nav2 and slam blocks at each change, during a mission."""

    def __init__(self, t0):
        self.t0, self.log, self._stop = t0, [], threading.Event()

    def __enter__(self):
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._t.join(5)

    def _run(self):
        bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=2)
        last = None
        while not self._stop.is_set():
            h = bridge_health(bridge)
            key = json.dumps(h, sort_keys=True, default=str)
            if key != last:
                self.log.append({"t": round(time.time() - self.t0, 2), **h})
                last = key
            time.sleep(0.25)


def mission_from_start(robot, t0):
    starts = []
    with HealthWatch(t0) as w:
        while time.time() - t0 < HOLD_S:
            res = de.run_mission(robot, HOUSE, policy="explore")
            if "start" in res:
                starts.append({"t": round(time.time() - t0, 2), "status": res["start"],
                               "detail": res["detail"][:200]})
                time.sleep(0.5)
                continue
            break
        else:
            return {"never_started": starts, "health_log": w.log}
    status = json.load(open(os.path.join(de.RUN_DIR, res["status_file"])))
    log = status.get("full_log") or []
    events = ((status.get("explore") or {}).get("events")) or []
    res.update(start_refusals=starts, health_log=w.log,
               started_s=round(t0 and (starts[-1]["t"] if starts else 0), 2),
               refused_lines=[ln for ln in log if "goal refused" in str(ln)],
               refused_events=[e for e in events if e.get("kind") == "goal_refused"
                               or e.get("event") == "goal_refused"])
    return res


def score(out):
    c = {}
    ss = out.get("starts", [])
    for s in ss:
        rs = [r for r in s.get("refusals", []) if "reason" in r]
        s["no_session"] = sum(r["reason"] == NO_SESSION for r in rs)
        s["held"] = {k: sum(r["reason"] == k for r in rs) for k in HOLDS}
        s["ros_unavailable"] = sum(r["reason"] == "ros_unavailable" for r in rs)
        s["other"] = sorted({r["reason"] for r in rs} - {NO_SESSION, "ros_unavailable", *HOLDS})
        s["holds_retryable"] = all(r.get("retryable") is True for r in rs if r["reason"] in HOLDS)
    c["a_no_ros_unavailable"] = all(s["ros_unavailable"] == 0 for s in ss)
    c["a_holds_retryable"] = all(s["holds_retryable"] for s in ss)
    c["a_held_seen"] = [sum(s["held"].values()) for s in ss]
    c["a_all_succeeded"] = all(s.get("state") == "succeeded" for s in ss)
    c["a_other_reasons"] = sorted({x for s in ss for x in s["other"]})
    c["a_met"] = (len(ss) == 3 and c["a_no_ros_unavailable"] and c["a_holds_retryable"]
                  and c["a_all_succeeded"] and not c["a_other_reasons"])
    # (c): a ros_unavailable before active whose nav2.why says the nodes are
    # not active is the assumption failing.
    c["c_not_active_before_active"] = sum(
        1 for s in ss for r in s.get("refusals", [])
        if r.get("reason") == "ros_unavailable"
        and "not active" in str(((r.get("health") or {}).get("nav2") or {}).get("why")))
    if "mission" in out:
        m = out["mission"]
        ex = m.get("explore") or {}
        c["b_refused_hold"] = sum(any(k in str(x) for k in HOLDS)
                                  for x in m.get("refused_lines", []) + m.get("refused_events", []))
        c["b_refused_unavailable"] = sum("ros_unavailable" in str(x)
                                         for x in m.get("refused_lines", []) + m.get("refused_events", []))
        c["b_goals_failed"] = ex.get("goals_failed")
        c["b_met"] = (c["b_refused_hold"] > 0 and c["b_refused_unavailable"] == 0
                      and m.get("outcome") == "found" and ex.get("goals_failed") == 0
                      and m.get("contact_samples") == 0)
    return c


def dump(out):
    with open(os.path.join(HERE, "result.json"), "w") as f:
        json.dump(out, f, indent=1, default=str)


def main(argv):
    if "--score" in argv:
        out = json.load(open(argv[argv.index("--score") + 1]))
        print(json.dumps(score(out), indent=1))
        return
    if os.environ.get("ROS_IMAGE") != "vision-picar-ros:latest":
        raise SystemExit("run on dev's image: ROS_IMAGE=vision-picar-ros:latest")
    n = next((int(a) for a in argv if a.isdigit()), 3)
    de.heavy_lock()
    img = subprocess.run(["docker", "image", "inspect", "-f", "{{.Id}}", de.os.environ["ROS_IMAGE"]],
                         capture_output=True, text=True).stdout.strip()
    out = {"image": os.environ["ROS_IMAGE"], "image_id": img, "house": HOUSE, "goal": GOAL,
           "starts": []}
    bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=2)
    robot = httpx.Client(base_url=f"http://127.0.0.1:{de.ROBOT}", timeout=30)
    try:
        for i in range(n):
            th, t0, box = launch()
            res = {"run_dir": de.RUN_DIR}
            try:
                res.update(goal_from_start(robot, bridge, t0))
                res["health_at_end"] = bridge_health(bridge)
            except Exception as e:  # noqa: BLE001 -- recorded; the next start goes on
                res["error"] = f"{type(e).__name__}: {e}"
            th.join(150)
            res["stack_error"] = box.get("error")
            out["starts"].append(res)
            dump(out)
            print(f"start {i + 1}: refusals {len(res.get('refusals', []))} "
                  f"accepted at {res.get('accepted_s')} s -> {res.get('state')}", flush=True)
        if "--no-mission" not in argv:
            de.MISSION_LIMIT_S = 900
            th, t0, box = launch()
            m = mission_from_start(robot, t0)
            th.join(150)
            m["run_dir"], m["stack_error"] = de.RUN_DIR, box.get("error")
            out["mission"] = m
            print(f"mission: {m.get('outcome')} contacts {m.get('contact_samples')}", flush=True)
        out["criteria"] = score(out)
        dump(out)
        print(json.dumps(out["criteria"]), flush=True)
    finally:
        de._kill_ports()


if __name__ == "__main__":
    main(sys.argv[1:])
