# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141: a $0 stand-in for VE on the desktop -- the real service's routes
(`service.local_vlm.server.create_app`) over a driver that answers every
identity question "no" and every direction question FORWARD after
`delay_s` (VE's p90 under the stack is ~1.2 s), so a tiered `explore`
search (which needs a local tier) runs where there is no Jetson.

    python evaluations/wheel-loop-3141/standin_vlm.py <port> [delay_s]
"""

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import uvicorn  # noqa: E402

from service.local_vlm.server import create_app  # noqa: E402


class StandinDriver:
    alive = ready = True

    def __init__(self, delay_s: float):
        self.delay_s, self.served = delay_s, 0
        self.lock = threading.Lock()
        self.proc = None

    def wait_ready(self, timeout_s):
        return None

    def close(self):
        self.alive = False

    def ask(self, jpeg, target, deadline_s, box=False, question="identity"):
        t0 = time.monotonic()
        with self.lock:              # one question at a time, as VE
            time.sleep(min(self.delay_s, max(0.0, deadline_s)))
            self.served += 1
        ms = (time.monotonic() - t0) * 1000
        if question == "direction":
            return {"status": "answered", "score": None, "parsed": True, "ms": ms,
                    "driver_ms": None, "direction": "forward", "p": 0.9, "mass": 0.95,
                    "probs": {"left": 0.02, "right": 0.03, "forward": 0.9}}
        return {"status": "answered", "score": 0.02, "parsed": True, "ms": ms,
                "driver_ms": None, "box": None}


def main():
    port = int(sys.argv[1])
    delay = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    app = create_app(lambda: StandinDriver(delay), info={"engines": "standin-3141"})
    uvicorn.run(app, host="127.0.0.1", port=port, log_level="warning")


if __name__ == "__main__":
    main()
