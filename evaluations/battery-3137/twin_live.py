# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.137.B's B6: the twin through a real drain, once, on the live local stack.

A real robot server (sim, `SIM_BATTERY_V` 10.62 V draining 0.01 V/s, so Low
at ~12 s, Reserve at ~42 s, Power-off at 10.0 V at ~62 s) and brain on this
section's ports, no ROS; the twin served as deployed (a static host), in a real
browser at 390 x 844 in operator mode, connected to both. The script reads
`GET /battery` itself every 0.25 s and the page's banner every 0.25 s, and
records when each level first appears in each. At Low it taps the Sim tab's
Start (B4, live: the brain's 409 `battery_low:`, worded). At Power-off it
stops the robot server and checks the banner stays (B2).

    PICAR_ROBOT_PORT=10370 PICAR_BRAIN_PORT=10371 \
        python evaluations/battery-3137/twin_live.py

Holds the desktop's exclusive lock (a timed run). Writes
`results-twin-live.json` here and one phone-size screenshot per level to
docs/evaluations/.
"""

import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import socketserver
import threading

import httpx
from playwright.sync_api import sync_playwright

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
PY = sys.executable
ROBOT_PORT = int(os.environ.get("PICAR_ROBOT_PORT", 10370))
BRAIN_PORT = int(os.environ.get("PICAR_BRAIN_PORT", 10371))
ROBOT = f"http://127.0.0.1:{ROBOT_PORT}"
BRAIN = f"http://127.0.0.1:{BRAIN_PORT}"
STATIC_PORT = ROBOT_PORT + 6
STATIC = f"http://127.0.0.1:{STATIC_PORT}"
SHOTS = REPO / "docs" / "evaluations"
DATE = "2026-11-02"
V0, DRAIN = 10.62, 0.01

BANNER = """() => { const el = document.getElementById('battery-banner');
  return {shown: !!el && el.checkVisibility(), level: el ? el.dataset.level : null,
          title: (document.getElementById('battery-title') || {}).textContent || ''}; }"""


def start(mod, port, env, run, logs):
    log = open(logs / f"{mod.split('.')[0]}.log", "w")
    proc = subprocess.Popen([PY, "-m", "uvicorn", mod, "--host", "127.0.0.1", "--port", str(port),
                             "--log-level", "warning"], cwd=run, env=env, stdout=log,
                            stderr=subprocess.STDOUT)
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise SystemExit(f"{mod} exited {proc.returncode}")
        try:
            if httpx.get(f"http://127.0.0.1:{port}/health", timeout=1).status_code == 200:
                return proc
        except httpx.HTTPError:
            pass
        time.sleep(0.2)
    raise SystemExit(f"{mod} not healthy")


def stop(proc):
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()


def main():
    sys.path.insert(0, str(REPO / "tools"))
    import desktop_lock
    desktop_lock.hold_for_life("exclusive", "3.137.B live twin check (timed drain)")
    run = Path(tempfile.mkdtemp(prefix="b3137-"))
    logs = run / "logs"
    logs.mkdir()
    env = {**os.environ, "PYTHONPATH": str(REPO), "ROBOT_URL": ROBOT,
           "VISION_URL": "http://127.0.0.1:9", "INTERVENTION_DIR": str(run / "iv"),
           "SIM_BATTERY_V": str(V0), "SIM_BATTERY_DRAIN_V_PER_S": str(DRAIN)}
    env.pop("APP_SHARED_SECRET", None)
    commit = subprocess.run(["git", "rev-parse", "--short=9", "HEAD"], cwd=REPO,
                            capture_output=True, text=True).stdout.strip()
    rec = {"commit": commit, "v0": V0, "drain_v_per_s": DRAIN, "server": {}, "page": {},
           "countdown": [], "events": []}
    # The twin served as deployed (measure.py's static host: the CloudFront
    # keys), so it reloads with the robot server gone, as on a phone.
    sys.path.insert(0, str(REPO / "evaluations" / "ux-3132"))
    import measure
    socketserver.TCPServer.allow_reuse_address = True
    httpd = socketserver.ThreadingTCPServer(("127.0.0.1", STATIC_PORT), measure._Deployed)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    robot = start("robot.server:app", ROBOT_PORT, env, run, logs)
    brain = start("control.brain_server:app", BRAIN_PORT, env, run, logs)
    t0 = time.monotonic()

    def now():
        return round(time.monotonic() - t0, 2)

    try:
        with sync_playwright() as p:
            browser = p.chromium.launch()
            ctx = browser.new_context(viewport={"width": 390, "height": 844}, color_scheme="dark")
            ctx.add_init_script("(() => { try { localStorage.setItem('vp_debug_readouts', '0');"
                                " localStorage.setItem('vp_brain_policy', 'frontier'); }"
                                " catch (e) {} })();")
            page = ctx.new_page()
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(f"{STATIC}/?serverUrl={ROBOT}&brainUrl={BRAIN}", wait_until="networkidle")
            page.wait_for_function("() => document.getElementById('connection-status')"
                                   ".classList.contains('is-ok')", timeout=20_000)
            rec["events"].append({"t_s": now(), "what": "page connected"})
            tapped = shot_at = None
            shots = {}
            deadline = time.monotonic() + 150
            while time.monotonic() < deadline:
                try:
                    b = httpx.get(f"{ROBOT}/battery", timeout=2).json()
                    lvl = b.get("level")
                    rec["server"].setdefault(lvl, {"t_s": now(), "v": b.get("v")})
                except httpx.HTTPError:
                    b = {}              # no countdown recorded from a missed read (review)
                pg = page.evaluate(BANNER)
                if pg["shown"]:
                    rec["page"].setdefault(pg["level"], {"t_s": now(), "title": pg["title"]})
                if pg["level"] == "reserve" and pg["shown"]:
                    rec["countdown"].append({"t_s": now(), "title": pg["title"],
                                             "server_poweroff_in_s":
                                                 (b.get("ladder") or {}).get("poweroff_in_s")})
                # B4 live: a person's Start at Low.
                if pg["level"] == "low" and tapped is None:
                    page.click('.tab-btn[data-tab="sim"]')
                    page.wait_for_selector("#btn-brain-mission:not([disabled])", timeout=10_000)
                    page.fill("#brain-target", "red backpack")
                    page.click("#btn-brain-mission")
                    toast = page.locator(".toast.err")
                    toast.first.wait_for(timeout=5000)
                    tapped = toast.first.inner_text()
                    rec["events"].append({"t_s": now(), "what": "start at low", "toast": tapped})
                    page.click('.tab-btn[data-tab="guide"]')
                # One screenshot a level, once the toasts have gone.
                if pg["shown"] and pg["level"] not in shots:
                    if shot_at is None or shot_at[0] != pg["level"]:
                        shot_at = (pg["level"], time.monotonic())
                    elif time.monotonic() - shot_at[1] > 4.0:
                        path = SHOTS / f"battery-3137B-{pg['level']}-live-{DATE}.png"
                        page.screenshot(path=str(path))
                        shots[pg["level"]] = str(path.relative_to(REPO))
                        rec["events"].append({"t_s": now(), "what": f"shot {pg['level']}",
                                              "title": pg["title"]})
                if "poweroff" in shots:
                    break
                page.wait_for_timeout(250)
            rec["shots"] = shots
            rec["ladder_end"] = httpx.get(f"{ROBOT}/battery", timeout=2).json().get("ladder")
            # B2 live: the robot goes quiet; the last word stays.
            stop(robot)
            rec["events"].append({"t_s": now(), "what": "robot server stopped"})
            page.wait_for_timeout(5000)
            rec["after_robot_gone"] = page.evaluate(BANNER)
            page.reload(wait_until="networkidle")
            page.wait_for_timeout(3000)
            rec["after_reload"] = page.evaluate(BANNER)
            rec["page_errors"] = errors
            browser.close()
    finally:
        stop(robot)
        stop(brain)
        httpd.shutdown()
        httpd.server_close()
    lag = {lvl: round(rec["page"][lvl]["t_s"] - rec["server"][lvl]["t_s"], 2)
           for lvl in ("low", "reserve", "poweroff") if lvl in rec["page"] and lvl in rec["server"]}
    rec["lag_s"] = lag
    (HERE / "results-twin-live.json").write_text(json.dumps(rec, indent=1))
    print(json.dumps({k: rec[k] for k in ("server", "page", "lag_s", "shots", "after_robot_gone",
                                          "after_reload", "page_errors")}, indent=1))
    print("countdown", [c["title"] for c in rec["countdown"][::8]])
    print("start at low:", tapped)


if __name__ == "__main__":
    main()
