# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/kill-stop-3216/progress_p1.py

3.216 criterion 3 (P1): 3.22's verb sweep (`tests/verb_sweep.py`, three
houses, speeds 50 and 100, `demo_verb_sweep.py`'s default size) in process,
with `robot.safety.T_REACT_S` set to a candidate's value (unset: the code's).
Prints the straight verbs' total distance driven, by speed, and the sweep's
own bars (contacts, travel-to-contact under 18 cm).

    python evaluations/kill-stop-3216/progress_p1.py [t_react_s]
"""

import json
import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from robot import safety  # noqa: E402
from tests import verb_sweep as vs  # noqa: E402


def main() -> int:
    logging.disable(logging.WARNING)
    if len(sys.argv) > 1:
        safety.T_REACT_S = float(sys.argv[1])
    straight = vs.straight_sweep(starts_per_house=6, headings=24)
    verbs = [v for r in straight for v in r["verbs"]]
    by_speed = {}
    for r in straight:
        by_speed[r["speed"]] = by_speed.get(r["speed"], 0.0) + sum(v["moved"] for v in r["verbs"])
    print(json.dumps({"t_react_s": safety.T_REACT_S, "runs": len(straight), "verbs": len(verbs),
                      "moved_cm": round(sum(by_speed.values()), 1),
                      "moved_cm_by_speed": {k: round(v, 1) for k, v in sorted(by_speed.items())},
                      "contacts": sum(1 for r in straight if r["min_G"] < vs.G_BAR_CM),
                      "under_18cm": sum(1 for v in verbs if v["T_after"] < vs.T_BAR_CM)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
