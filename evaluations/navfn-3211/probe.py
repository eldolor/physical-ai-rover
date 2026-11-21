# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""evaluations/navfn-3211/probe.py -- 3.211.A: NavFn's plan failures measured
offline, `use_astar: true` (dev) against `false`. How it is read is
docs/plans/ros-alignment/3.211-navfn-plan-failures.md, written before any run.

One arm: a fresh robot server (the robot on the dock), the ROS container on
a saved map (mounted at /maps, reloaded at the dock), `nav2.yaml` with
`use_astar` set by the arm (mounted over the image's), then
service/slam/probe/navfn_probe.py inside the container: `draw` once (the pair
set is kept and reused by the other arm), then `plan`. Nothing moves.

    eval "$(python tools/plan_section.py env)"
    python tools/desktop_lock.py exclusive --why "3.211.A" -- \\
        python evaluations/navfn-3211/probe.py arm astar|dijkstra MAPS_DIR OUT_DIR
    python evaluations/navfn-3211/probe.py score OUT_DIR

The map stays local (MAPS_DIR, outside the repo); OUT_DIR keeps the pairs,
the rows and the arm's ROS log.
"""

import json
import os
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
PROBE = ROOT / "service" / "slam" / "probe" / "navfn_probe.py"
NAV_YAML = ROOT / "service" / "slam" / "src" / "picar_bringup" / "config" / "nav2.yaml"
HOUSE = "complex_house"
SEED, N, MIN_M, TRIPS = 3211, 400, 3.0, 40
DOCK, STAGING = (0.0, 0.0), (-0.30, 0.0)      # the map frame: the reload puts the dock at 0, 0
ROBOT = int(os.environ.get("PICAR_ROBOT_PORT", 8100))
BRIDGE = int(os.environ.get("PICAR_BRIDGE_PORT", 8105))
CONTAINER = os.environ.get("PICAR_ROS_CONTAINER", "picar-ros-explore")
LEGAL = "when a legal potential was found"
PY = sys.executable


def _nav_yaml(arm, out):
    text = NAV_YAML.read_text()
    want = "true" if arm == "astar" else "false"
    new, n = re.subn(r"(\n\s*use_astar:\s*)(true|false)", rf"\g<1>{want}", text)
    assert n == 1, "nav2.yaml must name use_astar once"
    p = out / f"nav2-{arm}.yaml"
    p.write_text(new)
    return p


def _wait(cond, what, timeout_s):
    end = time.time() + timeout_s
    while time.time() < end:
        try:
            if cond():
                return
        except Exception:  # noqa: BLE001 -- still coming up
            pass
        time.sleep(1)
    raise SystemExit(f"{what} not ready in {timeout_s} s")


def arm(name, maps, out):
    import httpx
    assert name in ("astar", "dijkstra")
    assert os.environ.get("ROS_IMAGE", "").endswith("plan-3.211"), "run with this branch's env"
    # The defaults are explore's shared names: never `docker rm -f` another
    # session's container or take its ports (review).
    for k in ("PICAR_ROS_CONTAINER", "PICAR_ROBOT_PORT", "PICAR_BRIDGE_PORT"):
        assert os.environ.get(k), f"{k} unset: eval \"$(python tools/plan_section.py env)\" first"
    out = Path(out)
    if out.resolve() == HERE.resolve():
        raise SystemExit("OUT_DIR is the committed records' directory: run into a scratch directory")
    out.mkdir(parents=True, exist_ok=True)
    for f in (f"rows-{name}.jsonl", f"ros-{name}.log", f"meta-{name}.json"):
        if (out / f).exists():
            raise SystemExit(f"{out / f} exists: never overwritten")
    rows = out / f"rows-{name}.jsonl"
    subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
    env = {k: v for k, v in os.environ.items() if k not in ("APP_SHARED_SECRET", "LOCAL_SECRET")}
    env.update(SIM_MAP=HOUSE, ROBOT_DRIVE="ros", WORLD_MODE="ros",
               ROS_BRIDGE_URL=f"http://127.0.0.1:{BRIDGE}", PICAR_LOG_LEVEL="WARNING")
    robot_log = open(out / f"robot-{name}.log", "w")
    robot = subprocess.Popen([PY, "-m", "uvicorn", "robot.server:app", "--port", str(ROBOT),
                              "--host", "127.0.0.1", "--log-level", "warning"], cwd=ROOT, env=env,
                             stdout=robot_log, stderr=subprocess.STDOUT)
    try:
        time.sleep(4)
        nav = _nav_yaml(name, out)
        subprocess.run(["docker", "run", "-d", "--name", CONTAINER, "-v", f"{os.path.abspath(maps)}:/maps",
                        "-v", f"{nav}:/ws/src/picar_bringup/config/nav2.yaml:ro",
                        "--network", "host", "-e", f"BRIDGE_PORT={BRIDGE}", "-e", "BRIDGE_HOST=127.0.0.1",
                        "-e", f"ROBOT_URL=http://127.0.0.1:{ROBOT}",
                        "-e", f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}",
                        os.environ["ROS_IMAGE"], "ros2", "launch", "picar_bringup", "picar.launch.py"],
                       capture_output=True, check=True)
        bridge = httpx.Client(base_url=f"http://127.0.0.1:{BRIDGE}", timeout=10)
        _wait(lambda: bridge.get("/health").json()["slam"]["state"] in ("ready", "failed"), "slam", 240)
        h = bridge.get("/health").json()
        reload = h["slam"].get("reload")
        if (reload or {}).get("result") != "matched":
            raise SystemExit(f"reload not matched: {reload}")
        logs = lambda: subprocess.run(["docker", "logs", CONTAINER], capture_output=True, text=True)  # noqa: E731
        _wait(lambda: "nav2 active" in (logs().stdout + logs().stderr), "nav2", 240)
        time.sleep(5)                                    # the costmaps' first full update
        sh = lambda *a: subprocess.run(["docker", "exec", CONTAINER, "bash", "-c",  # noqa: E731
                                        "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && "
                                        + " ".join(a)], capture_output=True, text=True)
        subprocess.run(["docker", "cp", str(PROBE), f"{CONTAINER}:/tmp/navfn_probe.py"], check=True)
        pairs = out / "pairs.json"
        if pairs.exists():
            subprocess.run(["docker", "cp", str(pairs), f"{CONTAINER}:/tmp/pairs.json"], check=True)
        else:
            r = sh("python3 /tmp/navfn_probe.py draw /tmp/pairs.json", str(SEED), str(N), str(MIN_M),
                   *map(str, DOCK), *map(str, STAGING), str(TRIPS))
            print(r.stdout, r.stderr[-2000:])
            subprocess.run(["docker", "cp", f"{CONTAINER}:/tmp/pairs.json", str(pairs)], check=True)
        param = sh("ros2 param get /planner_server GridBased.use_astar").stdout.strip()
        print(name, "planner_server:", param)
        r = sh("python3 /tmp/navfn_probe.py plan /tmp/pairs.json /tmp/rows.jsonl")
        print(r.stdout, r.stderr[-2000:])
        subprocess.run(["docker", "cp", f"{CONTAINER}:/tmp/rows.jsonl", str(rows)], check=True)
        lg = logs()
        (out / f"ros-{name}.log").write_text(lg.stdout + lg.stderr)
        with open(out / f"meta-{name}.json", "w") as f:
            json.dump({"arm": name, "use_astar_param": param, "image": os.environ["ROS_IMAGE"],
                       "reload": reload, "load1": os.getloadavg()[0]}, f)
    finally:
        subprocess.run(["docker", "rm", "-f", CONTAINER], capture_output=True)
        robot.terminate()
        try:
            robot.wait(10)
        except subprocess.TimeoutExpired:
            robot.kill()
        robot_log.close()


def _legal_times(log_text):
    """Wall times (s) of NavFn's 'legal potential' errors in a ROS log."""
    out = []
    for line in log_text.splitlines():
        if LEGAL in line:
            m = re.search(r"\[(\d{9,}\.\d+)\]", line)
            if m:
                out.append(float(m.group(1)))
    return out


def load_arm(out, name):
    rows = [json.loads(x) for x in (out / f"rows-{name}.jsonl").read_text().splitlines() if x.strip()]
    log = out / f"ros-{name}.log"            # the arm's whole log (local) or its committed excerpt
    times = _legal_times((log if log.exists() else out / f"ros-{name}-planner.log").read_text())
    # Each error to the one call in flight: the last call started before it
    # (calls are serial and take milliseconds, so a window would overlap).
    # An error before the first call (costmap warm-up) is charged to none and
    # counted as `uncharged`, so the score shows it.
    starts = [r["t0"] for r in rows]
    for r in rows:
        r["legal"] = 0
    uncharged = 0
    for t in times:
        k = max((j for j, s in enumerate(starts) if s <= t), default=None)
        if k is None:
            uncharged += 1
        else:
            rows[k]["legal"] += 1
    return rows, len(times), uncharged


def _pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(q * (len(xs) - 1))))] if xs else None


def score(out):
    out = Path(out)
    a, a_legal, a_un = load_arm(out, "astar")
    d, d_legal, d_un = load_arm(out, "dijkstra")
    assert [r["i"] for r in a] == [r["i"] for r in d], "the arms must ask the same pairs"
    n = len(a)
    both_fail = [x["i"] for x, y in zip(a, d) if not x["ok"] and not y["ok"]]
    lost = [x["i"] for x, y in zip(a, d) if x["ok"] and not y["ok"]]
    gained = [x["i"] for x, y in zip(a, d) if not x["ok"] and y["ok"]]
    ratios = [y["length_m"] / x["length_m"] for x, y in zip(a, d)
              if x["ok"] and y["ok"] and x["length_m"]]
    res = {
        "pairs": n,
        "astar": {"failed": sum(not r["ok"] for r in a), "legal_rows": sum(r["legal"] > 0 for r in a),
                  "legal_lines": a_legal, "planning_ms_p50": _pct([r["planning_ms"] for r in a if r["ok"]], 0.5),
                  "planning_ms_p90": _pct([r["planning_ms"] for r in a if r["ok"]], 0.9),
                  "by_kind_failed": {k: sum(not r["ok"] for r in a if r["kind"] == k) for k in ("pair", "reverse", "staging", "dock")}},
        "dijkstra": {"failed": sum(not r["ok"] for r in d), "legal_rows": sum(r["legal"] > 0 for r in d),
                     "legal_lines": d_legal, "planning_ms_p50": _pct([r["planning_ms"] for r in d if r["ok"]], 0.5),
                     "planning_ms_p90": _pct([r["planning_ms"] for r in d if r["ok"]], 0.9),
                     "by_kind_failed": {k: sum(not r["ok"] for r in d if r["kind"] == k) for k in ("pair", "reverse", "staging", "dock")}},
        "uncharged": {"astar": a_un, "dijkstra": d_un},
        # every failed row charged with an error, and no successful one
        "failed_are_legal": ({r["i"] for r in a if not r["ok"]} == {r["i"] for r in a if r["legal"]}),
        "both_failed": len(both_fail), "lost": lost, "gained": len(gained),
        "length_ratio_median": round(statistics.median(ratios), 4) if ratios else None,
    }
    a1 = res["astar"]["legal_rows"] >= 0.01 * n
    res["A1"] = ("MET" if a1 else "FAILED (stop: the cause is not shown)")
    res["A2"] = "MET" if res["dijkstra"]["legal_lines"] == 0 and not lost else "FAILED"
    res["A3"] = "MET" if res["length_ratio_median"] is not None and res["length_ratio_median"] <= 1.02 else "FAILED"
    p90 = res["dijkstra"]["planning_ms_p90"]
    res["A4"] = "MET" if p90 is not None and p90 <= 500 else "FAILED"
    return res


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "arm":
        arm(a[1], a[2], a[3])
    elif a and a[0] == "score":
        print(json.dumps(score(a[1]), indent=1))
    else:
        sys.exit(__doc__)
