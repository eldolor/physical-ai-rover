# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/soak-find-3143/v3.py

3.143 criterion 3, bar V3 (fix V on the board): the five class V pairs from
3.135.C's run 1 (missions 6, 11, 13, 14 and 18), once each, through 3.135.C's
own live stack (`evaluations/jetson-fmea-3135/soak.py`'s `Soak`: the split
body with ground truth, the fake board and D500, the ROS container, VE as
found, YOLOE + CLIP at 4 Hz, the $0 cloud stand-in), so every mission is set
up and recorded exactly as a soak mission is. Only the schedule differs: this
list, once, in place of the soak's time-bounded rotation. Bar: >= 4 `found`,
0 contacts, VE asked at every arrival.

    python evaluations/soak-find-3143/v3.py <out>

Run on the board through `board.sh v3` (see `board.py`).
"""

from __future__ import annotations

import json
import signal
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "evaluations/jetson-fmea-3135"))

import soak  # noqa: E402
from stack import StackError  # noqa: E402

PAIRS = (("complex_house", "slippers"), ("complex_house", "toy car"),
         ("procthor:val:2", "house plant"), ("procthor:val:3", "garbage bag"),
         ("procthor:val:2", "garbage can"))


class V3(soak.Soak):
    def run(self) -> int:
        try:
            self.setup()
            (self.out / "run_start").write_text(f"{time.time()}\n")
            with open(self.out / "missions.jsonl", "a", buffering=1) as f:
                for k, (house, target) in enumerate(PAIRS):
                    dead = {n: p.returncode for n, p in self.helpers.items() if p.poll() is not None}
                    if dead:
                        raise StackError(f"a long-lived helper ended: {dead}")
                    self.say(f"m{k:03d} {house} / {target}; MemAvailable {soak.mem_available_mb()} MB")
                    rec = self.mission(k, house, target)
                    f.write(json.dumps(rec, default=str) + "\n")
                    self.say(f"m{k:03d} -> {rec.get('outcome')} {rec.get('error') or ''}")
                    if rec.get("error"):
                        time.sleep(30)     # as Soak.run: the failed stack's ports and names free first
            (self.out / "run_end").write_text(f"{time.time()}\n")
            return 0
        finally:
            self.teardown()


def main(argv) -> int:
    def on_signal(signum, _frame):
        raise SystemExit(128 + signum)     # the finally's teardown runs
    for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(s, on_signal)
    # `seconds` sizes the long-lived helpers' lifetimes (soak.Soak.setup): every
    # mission at its cap, with room.
    return V3(Path(argv[0]), len(PAIRS) * soak.MISSION_CAP_S).run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
