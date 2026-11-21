# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.210 E1, pinned in process (slow tier: one ProcTHOR find, ~800 s
simulated): `procthor:val:44`, the rooms chooser, no zones, found. On `dev`
before 3.210 it ended `exhausted` at 770.6 s, its one true place crowded out
of the look-again round by three false sightings after it."""

import functools
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _sweep():
    spec = importlib.util.spec_from_file_location(
        "nogo_sweep_3161_reseen_3210", ROOT / "evaluations" / "nogo-3161" / "sweep.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_3210_val44_rooms_no_zones_is_found(monkeypatch):
    """3.210 E1, pinned: `dev` ended it `exhausted` (770.6 s), the true place
    crowded out of the one round by three false sightings after it. The rooms
    chooser as 3.192's sweep2 sets it, nothing else patched."""
    import functools
    sw = _sweep()
    monkeypatch.setattr(sw, "MissionRunner",
                        functools.partial(sw.MissionRunner, explore_goal_policy="rooms"))
    row = sw.run("procthor:val:44", 0, "none")
    assert row["outcome"] == "found", row["outcome"]
