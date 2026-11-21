#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.190.D, criterion 15: the `cloud` line's screenshot again, after the
wording changed (it promised an order the goal choice does not keep). On
live.py's saved map and kept names (WORKDIR as live.py's), a fresh empty
cache, so the find's start asks the deployed route once (source `cloud`);
the screenshot is taken while the find runs, then the find is stopped.

    eval "$(python tools/plan_section.py env)"
    PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:plan-3.190 \\
        python evaluations/room-prior-3190/live/shot_cloud.py WORKDIR
"""
import json
import os
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

import httpx

sys.path.insert(0, str(Path(__file__).resolve().parent))
import live as L  # noqa: E402 -- its env, stand-ins and forwarder

OUT = L.HERE / "shot_cloud.json"


def main():
    if not L.SECRET:
        sys.exit("VISION_SECRET is not set: the paid route cannot be reached")
    stamp = int(time.time())
    os.environ["ROOM_PRIOR_CACHE"] = str(L.WORK / f"room_prior_cache-shot-{stamp}.json")
    os.environ["INVENTORY_DIR"] = str(L.WORK / f"inventory-shot-{stamp}")    # empty: no recall
    port = L.R.free_port()
    httpd = ThreadingHTTPServer(("127.0.0.1", port), L.Forward)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    os.environ.update(LOCAL_VLM_URL=f"http://127.0.0.1:{port}", VISION_URL=f"http://127.0.0.1:{port}")
    rec = {}
    try:
        L.de.stack(L.HOUSE)
        brain = httpx.Client(base_url=f"http://127.0.0.1:{L.de.BRAIN}", timeout=60)
        rec["names"] = L.named_already(brain)
        r = brain.post("/mission/start", json={"target_object": L.TARGET, "policy": "tiered"})
        rec["start"] = r.status_code
        rec["room_prior"] = brain.get("/mission/status").json().get("room_prior")
        # Never shoot (and overwrite the good picture) without kept names
        # and a started find (review).
        if rec["names"] and rec["start"] == 200:
            rec["shot"] = L.screenshot("cloud", "cloud")
        brain.post("/mission/stop")
        for _ in range(60):
            if not brain.get("/mission/status").json().get("running"):
                break
            time.sleep(0.5)
    finally:
        rec["calls"] = L.CALLS
        ins = sum(c["in"] or 0 for c in L.CALLS)
        outs = sum(c["out"] or 0 for c in L.CALLS)
        rec["spend"] = {"calls": len(L.CALLS), "in": ins, "out": outs,
                        "usd": round(ins * L.PRICE_IN + outs * L.PRICE_OUT, 5)}
        # A run that did not shoot keeps the last good record (Thermos).
        ok = bool((rec.get("shot") or {}).get("ok"))
        (OUT if ok else OUT.with_name("shot_cloud-failed.json")).write_text(
            json.dumps(rec, indent=1, sort_keys=True) + "\n")
        L.de._kill_ports()
        httpd.shutdown()
    print(json.dumps({k: rec.get(k) for k in ("start", "shot", "spend")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
