# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/false-sighting-3213/bars_find.py -- 3.213.B's find and survey
arms: `evaluations/nogo-3192/fix/sweep_c1.py` unchanged (3.192's four find
arms, both choosers, both zone arms; the surveys), plus one field a row did
not carry, `final_d_m`: the robot's final distance from the target's nearest
true cell, for the wrong-found bar (a `found` row more than 1.0 m from it).

    SWEEP_WORKERS=4 python evaluations/false-sighting-3213/bars_find.py find|survey OUT.json [--chooser rooms]
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.realpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "evaluations", "nogo-3192", "fix"))
argv = sys.argv[1:]
if "--chooser" in argv:
    i = argv.index("--chooser")
    os.environ["SWEEP2_CHOOSER"] = argv[i + 1]
    argv = argv[:i] + argv[i + 2:]
import sweep_c1                                          # noqa: E402 -- patches sweep2/sweep
import sweep2                                            # noqa: E402

sw = sweep2.sw
sweep_c1._apply()
_live = {}
_Base = sw.MockRobot


class _Robot(_Base):
    """The world handed to the body (`place_zones` may build its own)."""

    def __init__(self, world, *a, **kw):
        _live["g"] = world
        super().__init__(world, *a, **kw)


sw.MockRobot = _Robot
_run = sw.run


def run(house, seed, arm, **kw):
    row = _run(house, seed, arm, **kw)
    g = _live.get("g")
    if g is not None and row.get("job") == "find":
        truth = [sw._centre(c) for c in sw.target_cells(g)]
        if truth:
            row["final_d_m"] = round(min(math.hypot(g.x * sw.CELL - x, g.y * sw.CELL - y)
                                         for x, y in truth), 2)
    return row


sw.run = run

if __name__ == "__main__":
    sw.main(argv)
