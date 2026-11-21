#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.190.D, criteria 14-15 (amendment 4): a find through the brain's HTTP API
on the live stack, on a saved map whose rooms a person named, for a target
the shipped prior lacks -- run 1 asks the deployed `POST /room-prior`
(source `cloud`), run 2 reads the cache (`cache`) -- plus the twin's
mission-card line at 390 px in operator mode.

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:plan-3.190 \\
        python evaluations/room-prior-3190/live/live.py WORKDIR

3.158.B's stand-ins (`rehearsal.Standins`) answer VE's /identity and the
cloud's /navigate from the simulator's report ($0); only /room-prior is
forwarded to the deployed route, the app secret (`VISION_SECRET`) added
here and never logged or written. Maps, names, the cache and logs stay
under WORKDIR (outside the repo); numbers and screenshots go to this
directory.
"""
import json
import os
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

import httpx

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SITE = "https://d114x92g7i4syl.cloudfront.net"      # the stack's SiteUrl
SECRET = os.environ.get("VISION_SECRET")             # read before rehearsal pops it
WORK = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
if WORK is None or WORK == ROOT or ROOT in WORK.parents:
    sys.exit("usage: live.py WORKDIR (outside the repo)")
WORK.mkdir(parents=True, exist_ok=True)
os.environ.update(EXPLORE_LOGDIR=str(WORK / "runs"), FIND_PRIOR_SOURCE="cloud",
                  ROOM_PRIOR_CACHE=str(WORK / "room_prior_cache.json"))

sys.path.insert(0, str(ROOT / "evaluations" / "room-names-3160d"))
import d7_live as D7          # noqa: E402 -- imports rehearsal (R) and sets its env
os.environ["ROOM_NAMES_DIR"] = str(WORK / "room-names")
R = D7.R
R.VOLUME = "picar-maps-3190d"
de = R.de

from brain.request import canonical_room  # noqa: E402

HOUSE = "home_first_floor"
TARGET = "refrigerator"
PRICE_IN, PRICE_OUT = 4.40e-6, 22e-6                  # control/admin_prices.py, Opus 5.5
OUT = HERE / "live.json"
CALLS = []                                            # each forwarded /room-prior


class Forward(R.Standins):
    """The stand-ins, but /room-prior goes to the deployed route."""

    def do_POST(self):
        if self.path.rstrip("/") != "/room-prior":
            return super().do_POST()
        body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
        t0 = time.time()
        try:
            r = httpx.post(SITE + "/room-prior", content=body, timeout=30,
                           headers={"content-type": "application/json", "x-app-secret": SECRET})
        except Exception as e:  # noqa: BLE001 -- no answer: the brain sees a 502 and names it
            r, out, code = None, {"error": type(e).__name__}, 502
        if r is not None:
            code = r.status_code          # kept when the body is not JSON (review)
            try:
                out = r.json()
            except ValueError:
                out = {"error": "not JSON", "body": r.text[:200]}
        u = (out.get("usage") or {}) if isinstance(out, dict) else {}
        CALLS.append({"at": round(t0), "status": code, "s": round(time.time() - t0, 2),
                      "target": json.loads(body or b"{}").get("target_object"),
                      "stop": out.get("stop") if isinstance(out, dict) else None,
                      "prior": out.get("prior") if isinstance(out, dict) else None,
                      "model_id": out.get("model_id") if isinstance(out, dict) else None,
                      "in": u.get("input_tokens"), "out": u.get("output_tokens")})
        self._send(out if isinstance(out, dict) else {"reply": out}, code)


class Load(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.rows, self.stop_ev = [], threading.Event()

    def run(self):
        while not self.stop_ev.is_set():
            self.rows.append(os.getloadavg()[0])
            self.stop_ev.wait(10)

    def summary(self):
        s = sorted(self.rows) or [0.0]
        return {"n": len(self.rows), "max": round(s[-1], 2), "p50": round(s[len(s) // 2], 2),
                "over_4": sum(v > 4 for v in s)}


def save(rec):
    OUT.write_text(json.dumps(rec, indent=1, sort_keys=True) + "\n")


def screenshot(name, want_source):
    """Home, operator mode, 390 x 844, connected to this stack's brain."""
    from playwright.sync_api import sync_playwright
    path = HERE / f"twin-3190d-{name}.png"
    with sync_playwright() as p:
        b = p.chromium.launch()
        ctx = b.new_context(viewport={"width": 390, "height": 844}, device_scale_factor=2)
        ctx.add_init_script(
            "(() => { try { localStorage.setItem('vp_server_url', location.origin);"
            " localStorage.setItem('vp_debug_readouts', '0');"
            " localStorage.setItem('vp_active_tab', 'home'); } catch (e) {} })();")
        page = ctx.new_page()
        page.goto(f"http://127.0.0.1:{de.ROBOT}/", wait_until="load")
        page.wait_for_timeout(1500)
        page.click("#btn-settings")
        page.fill("#cfg-brain-url", f"http://127.0.0.1:{de.BRAIN}")
        page.click("#btn-brain-connect")
        page.wait_for_timeout(1500)
        page.click('.tab-btn[data-tab="home"]')
        line = page.locator("#mission-prior")
        line.wait_for(state="visible", timeout=15000)
        got = {"text": line.inner_text(), "source": line.get_attribute("data-source")}
        page.locator("#home-mission-panel").scroll_into_view_if_needed()
        page.screenshot(path=str(path))
        b.close()
    got["png"] = path.name
    got["ok"] = got["source"] == want_source
    return got


def first_goal(run_dir, f, truth):
    """The first goal_sent event in the brain's log, in the sim's frame, and
    the true room under it."""
    log = Path(run_dir) / "explore_brain.log"
    for line in log.read_text(errors="replace").splitlines():
        i = line.find("explore {")
        if i < 0:
            continue
        try:
            rec = json.loads(line[i + len("explore "):])
        except ValueError:
            continue
        if rec.get("event") != "goal_sent":
            continue
        gx, gy = rec["goal"]
        x, y = f(gx, gy)
        room = truth.room.get((int(x / R.CELL_M), int(y / R.CELL_M)))
        return {"map_xy": [gx, gy], "sim_xy": [round(x, 2), round(y, 2)], "room": room,
                "type": canonical_room(room) if room else None, "kind": rec.get("kind")}
    return None


def named_already(brain):
    """The names a person gave this saved map are kept (3.160's store): a
    second stack on it is not named again (the store caps a map at 64)."""
    t0 = time.time()
    while time.time() - t0 < D7.READY_S:
        v = brain.get("/mission/rooms").json()
        if v.get("frame_id") and v.get("rooms"):
            break
        time.sleep(3)
    person = sorted({r["name"] for r in v.get("rooms") or () if r.get("source") == "person"})
    return {"named": len(person), "kept": True, "person_rooms": sum(
        r.get("source") == "person" for r in v.get("rooms") or ()), "names": person} if person else None


def find_run(robot, brain, tag):
    f = D7.to_truth(robot.get("/world/pose").json(), robot.get("/world/truth").json())
    m = R.mission(robot, brain, HOUSE, {"job": "find", "target": TARGET})
    s = brain.get("/mission/status").json()
    rp = s.get("room_prior") or {}
    rates = rp.get("rates") or {}
    top = max(rates, key=rates.get) if rates else None
    g = first_goal(de.RUN_DIR, f, R.Truth(HOUSE, robot))
    m.pop("log_tail", None)
    return {"run": tag, **m, "room_prior": rp, "top_type": top, "first_goal": g,
            "first_goal_in_top": bool(g and top and g["type"] == top),
            "goals_sent": (s.get("explore") or {}).get("goals_sent"),
            "room_labels": s.get("room_labels")}


def main():
    if not SECRET:
        sys.exit("VISION_SECRET is not set: the paid route cannot be reached")
    rec = json.loads(OUT.read_text()) if OUT.exists() else {"house": HOUSE, "target": TARGET}
    rec["image"] = os.environ.get("ROS_IMAGE")
    port = R.free_port()
    httpd = ThreadingHTTPServer(("127.0.0.1", port), Forward)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    os.environ.update(LOCAL_VLM_URL=f"http://127.0.0.1:{port}", VISION_URL=f"http://127.0.0.1:{port}")
    load = Load()
    load.start()
    brain = None
    try:
        if "map" not in rec:
            subprocess_run(["docker", "volume", "rm", "-f", R.VOLUME])
            robot = de.stack(HOUSE)
            brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
            # Criterion 15's `none`: no saved map yet, so no layer and no call.
            r = brain.post("/mission/start", json={"target_object": TARGET, "policy": "tiered"})
            rec["none_start"] = r.status_code
            rec["none_shot"] = screenshot("none", "none")
            rec["none_status"] = brain.get("/mission/status").json().get("room_prior")
            brain.post("/mission/stop")
            for _ in range(60):
                if not brain.get("/mission/status").json().get("running"):
                    break
                time.sleep(0.5)
            print("none", json.dumps(rec["none_shot"]), flush=True)
            # The survey; the bridge saves the map at its end.
            os.environ["INVENTORY_DIR"] = str(WORK / "inventory-survey")
            de.stack(HOUSE)                  # a fresh robot on the dock after the stop
            robot = httpx.Client(base_url=f"http://127.0.0.1:{de.ROBOT}", timeout=30)
            sv = R.mission(robot, brain, HOUSE, {"job": "survey"})
            sv.pop("log_tail", None)
            bridge = httpx.Client(base_url=f"http://127.0.0.1:{de.BRIDGE}", timeout=30)
            t0 = time.time()
            while time.time() - t0 < 180:
                mh = bridge.get("/health").json()["map"]
                if mh.get("saved", 0) >= 1 and not mh.get("saving"):
                    break
                time.sleep(1)
            rec["map"] = {"survey": {k: sv.get(k) for k in ("outcome", "seconds", "contacts",
                                                             "docked", "end_reason")},
                          "bridge_map": mh}
            save(rec)
            print("map", json.dumps(rec["map"]), flush=True)
        for tag in ("run1", "run2"):
            if tag in rec:
                continue
            # A find of its own, with no memory of the survey's: an empty
            # inventory, so explore does not recall the target the survey
            # placed (run 1, 2026-11-20: its first goal was a recall).
            os.environ["INVENTORY_DIR"] = str(WORK / f"inventory-{tag}")
            robot = de.stack(HOUSE)
            brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
            named = named_already(brain) or D7.name_rooms(robot, brain, HOUSE)
            if not named.get("named"):
                rec[tag + "_names_failed"] = named
                save(rec)
                print("names failed", named, flush=True)
                return 2
            out = {"names": named, **find_run(robot, brain, tag)}
            if tag == "run1":
                out["shot"] = screenshot("cloud", "cloud")
            rec[tag] = out
            save(rec)
            print(tag, json.dumps({k: out.get(k) for k in (
                "outcome", "contacts", "seconds", "top_type", "first_goal", "first_goal_in_top")}
                ) + " source=" + str(out["room_prior"].get("source")), flush=True)
    finally:
        load.stop_ev.set()
        rec.setdefault("calls", [])
        rec["calls"] += CALLS
        rec["load"] = load.summary()
        ins = sum(c["in"] or 0 for c in rec["calls"])
        outs = sum(c["out"] or 0 for c in rec["calls"])
        rec["spend"] = {"calls": len(rec["calls"]), "in": ins, "out": outs,
                        "usd": round(ins * PRICE_IN + outs * PRICE_OUT, 5)}
        save(rec)
        de._kill_ports()
        httpd.shutdown()
    return 0


def subprocess_run(cmd):
    import subprocess
    subprocess.run(cmd, capture_output=True)


if __name__ == "__main__":
    sys.exit(main())
