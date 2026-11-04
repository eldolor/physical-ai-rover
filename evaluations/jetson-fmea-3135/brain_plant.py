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
| `vlm_absent` | 3.80: the local VLM gone (refused, dropped) is read as its answer "no" (score 0), so the tiers hand nothing on to the cloud | F3 |
| `write_intolerant` | 3.88 A: a goal-choice write that fails (a full disk) is logged every time and raised into the mission (`MissionRunner._record_choice` without its once-only catch) | F15 |
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
KNOWN = {"cloud_class", "cloud_deadline", "vlm_absent", "write_intolerant"}
if not PLANTS or PLANTS - KNOWN:
    raise SystemExit(f"FMEA_PLANT must name one or more of {sorted(KNOWN)}, not {sorted(PLANTS)}")

if "cloud_deadline" in PLANTS:
    # config/robot.yaml sets both keys (20 s, 8 s), so they are set on the loaded
    # config, after the yaml; brain_server takes the function by name at import.
    from control import brain_config

    _load = brain_config.load_brain_config

    def _load_long(*a, **k):
        cfg = _load(*a, **k)
        cfg["arrival_confirm_timeout_s"] = 120.0
        cfg["vision_timeout_s"] = 120.0
        return cfg

    brain_config.load_brain_config = _load_long
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

if "vlm_absent" in PLANTS:
    from brain import local_vlm

    _vpost = local_vlm.LocalVLM._post

    def _vpost_unsorted(self, route, frame, box=False):
        try:
            return _vpost(self, route, frame, box=box)
        except local_vlm.LocalVLMAbsent:
            # planted (amendment 12): VE gone read as its answer "no", so nothing is handed on
            return {"status": "answered", "score": 0.0, "parsed": True}

    local_vlm.LocalVLM._post = _vpost_unsorted

if "write_intolerant" in PLANTS:
    from control import mission_runner

    def _record_choice_intolerant(self, rec):
        full = {"mission_id": self.mission_id, "house": self.house, **rec}
        self.choices.append(full)
        if self.choice_log is None:
            return
        try:
            self.choice_log.append(full)
        except Exception as e:  # noqa: BLE001
            mission_runner.logger.warning("goal choice not recorded: %s", e)   # planted: every time
            raise                                                            # planted: into the mission

    mission_runner.MissionRunner._record_choice = _record_choice_intolerant

import control.brain_server as server  # noqa: E402  (builds `app` with the patches in place)
import uvicorn  # noqa: E402

uvicorn.run(server.app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
