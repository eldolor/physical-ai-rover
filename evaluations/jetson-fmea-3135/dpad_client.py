# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/dpad_client.py

F13's driving client: a person holding FORWARD, as a standing wheel command
(`POST /wheels`, driver `twin-dpad`, the person's rank) re-sent every 0.1 s,
which is what the watchdog guards (it zeroes a standing command on
silence). The injector kills this process with SIGKILL mid-press: the link
lost while a person drives.

    python evaluations/jetson-fmea-3135/dpad_client.py <robot_url> <secret> [m/s]
"""

import json
import sys
import time
import urllib.request

WHEEL_RADIUS_M = 0.040          # the Rover's tyre, as robot/hardware_robot.py

url, secret = sys.argv[1], sys.argv[2]
w = float(sys.argv[3] if len(sys.argv) > 3 else 0.2) / WHEEL_RADIUS_M
body = json.dumps({"left_rad_s": w, "right_rad_s": w}).encode()
while True:
    req = urllib.request.Request(url + "/wheels", data=body, method="POST",
                                 headers={"content-type": "application/json",
                                          "x-app-secret": secret, "x-driver": "twin-dpad"})
    try:
        with urllib.request.urlopen(req, timeout=1.0) as r:
            print(r.read().decode()[:200], flush=True)
    except Exception as e:                  # noqa: BLE001 -- keep pressing, as a finger would
        print(f"error {e}", flush=True)
    time.sleep(0.1)
