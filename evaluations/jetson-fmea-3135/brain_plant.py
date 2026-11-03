# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/brain_plant.py

Criterion 4's brain server: the shipped `control.brain_server` app with ONE
guard switched off, named by `FMEA_PLANT` (comma-separated), as
`robot_plant.py` does for the robot server. A run with no plant never uses
this file: `stack.py` starts `uvicorn control.brain_server:app` itself.
Nothing in brain/ or control/ is edited; the patch is on the module, here.

| Plant | The guard switched off | Row |
|---|---|---|
| `cloud_class` | 3.47: a transport error to the cloud is a plain RuntimeError, not `CloudUnavailable` | F11 |
| `cloud_deadline` | 3.56 and B3.2's deadlines raised to 120 s (`arrival_confirm_timeout_s`, `vision_timeout_s`) | F12 |

    FMEA_PLANT=cloud_class python evaluations/jetson-fmea-3135/brain_plant.py <port>
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

PLANTS = {p for p in os.environ.get("FMEA_PLANT", "").split(",") if p}
KNOWN = {"cloud_class", "cloud_deadline"}
if not PLANTS or PLANTS - KNOWN:
    raise SystemExit(f"FMEA_PLANT must name one or more of {sorted(KNOWN)}, not {sorted(PLANTS)}")

if "cloud_deadline" in PLANTS:
    # Read by load_brain_config() under the yaml, which sets neither key.
    from control import brain_config

    brain_config.DEFAULTS["arrival_confirm_timeout_s"] = 120.0
    brain_config.DEFAULTS["vision_timeout_s"] = 120.0
if "cloud_class" in PLANTS:
    import httpx

    from brain import navigate

    _post = navigate._post

    def _post_unsorted(url, body, headers, timeout_s, client):
        try:
            return _post(url, body, headers, timeout_s, client)
        except navigate.CloudUnavailable as e:
            if isinstance(e.__cause__, httpx.TransportError):
                raise RuntimeError(f"planted: {e}") from None     # an outage read as our fault
            raise

    navigate._post = _post_unsorted       # a module global, looked up at each call

import control.brain_server as server  # noqa: E402  (builds `app` with the patches in place)
import uvicorn  # noqa: E402

uvicorn.run(server.app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
