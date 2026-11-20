# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3192/sweep2.py -- 3.192.B's runs on 3.161.B's harness
(`evaluations/nogo-3161/sweep.py`, imported unchanged).

Two things only, added around it:
* `--chooser rooms` builds every `MissionRunner` with
  `explore_goal_policy="rooms"` (the shipped chooser, `config/robot.yaml`);
  `rule` (the default) is the harness as 3.161 ran it.
* F10: every explore `goal_ended` is recorded, and each mission's row gets
  `ends_near_zone`: per zone, the goals that ended other than `succeeded`
  with the goal point within NEAR_M of that zone.

    python evaluations/nogo-3192/sweep2.py find|survey|leave OUT.json \\
        [--chooser rooms] [--arm none]

Run from the checkout under test (the harness refuses another checkout's
packages); copy this file and `evaluations/nogo-3161/` in for a `dev` run.
"""

import functools
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.realpath(os.path.join(HERE, "..", ".."))
NEAR_M = 0.5

_spec = importlib.util.spec_from_file_location(
    "sweep", os.path.join(ROOT, "evaluations", "nogo-3161", "sweep.py"))
sw = importlib.util.module_from_spec(_spec)
sys.modules["sweep"] = sw
_spec.loader.exec_module(sw)

import brain.explore as ex                               # noqa: E402
from world import nogo                                   # noqa: E402

_ENDS: list = []
_orig_event = ex.ExploreAgent._event


def _event(self, name, pose=None, **fields):
    if name == "goal_ended":
        _ENDS.append((fields.get("state"), fields.get("goal")))
    return _orig_event(self, name, pose, **fields)


ex.ExploreAgent._event = _event
_orig_run = sw.run


def run(house, seed, arm, job="find", zones=None, max_steps=None):
    _ENDS.clear()
    kw = {} if max_steps is None else {"max_steps": max_steps}
    row = _orig_run(house, seed, arm, job=job, zones=zones, **kw)
    zs = [nogo.clean_zone(z) for z in row.get("zones") or []]
    near = {}
    for state, goal in _ENDS:
        if state == "succeeded" or not goal:
            continue
        for z in zs:
            if nogo.nearest_distance([z], goal[0], goal[1]) <= NEAR_M:
                near[z["id"]] = near.get(z["id"], 0) + 1
    row["ends_near_zone"] = near
    row["ends"] = len(_ENDS)
    row["chooser"] = os.environ.get("SWEEP2_CHOOSER", "rule")
    return row


def main(argv):
    if "--chooser" in argv:
        i = argv.index("--chooser")
        if i + 1 >= len(argv):
            raise SystemExit("--chooser needs rule or rooms")
        chooser = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
        os.environ["SWEEP2_CHOOSER"] = chooser   # worker processes read it at import
    _apply()
    sw.main(argv)


_BASE_RUNNER = sw.MissionRunner


def _apply():
    """Idempotent (review): always from the harness's own MissionRunner."""
    sw.run = run
    sw.MissionRunner = _BASE_RUNNER
    if os.environ.get("SWEEP2_CHOOSER", "rule") == "rooms":
        sw.MissionRunner = functools.partial(_BASE_RUNNER, explore_goal_policy="rooms")
    elif os.environ.get("SWEEP2_CHOOSER", "rule") != "rule":
        raise SystemExit("--chooser is rule or rooms")


_apply()   # in a pool worker (spawned or forked), the same patches

if __name__ == "__main__":
    main(sys.argv[1:])
