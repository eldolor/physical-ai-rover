# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/goal-lease-3145/robot_plant.py

3.145 criterion 3's robot server: the shipped `robot.server` app with the
goal lease switched off (`no_lease`), as 3.135's `robot_plant.py` does for its
own guards (that file is 3.135's; this one is 3.145's). Nothing in robot/ is
edited; the patch is on the app's shared `lease` dict.

    python evaluations/goal-lease-3145/robot_plant.py <port>
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def plant_no_lease(app) -> None:
    """3.145 off: a brain's nav2 goal is never leased -- the `lease` dict
    every `create_app` closure shares swapped for one whose owner never
    sets -- so a dead or frozen brain's goal drives on to its end, as
    before 3.145 (3.135.B's F6g/F7g). Also used by tests/test_goal_lease.py."""
    beat = next(r.endpoint for r in app.routes
                if getattr(r, "path", "").endswith("/autonomy/heartbeat")
                and "POST" in getattr(r, "methods", ()))
    cell = beat.__closure__[beat.__code__.co_freevars.index("lease")]

    class _NoLease(dict):
        def update(self, *a, **kw):
            kw.pop("owner", None)            # planted: no goal is ever leased
            super().update(*a, **kw)

    cell.cell_contents = _NoLease(cell.cell_contents)   # one cell, shared by every closure


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT))
    os.chdir(ROOT)
    import uvicorn

    import robot.server as server

    plant_no_lease(server.app)
    uvicorn.run(server.app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
