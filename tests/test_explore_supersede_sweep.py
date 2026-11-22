# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.213.B (C1), in process on 3.161's sweep harness: `procthor:val:102`'s
find, which ran out of budget on `dev` (3.213.A: a false sighting replaced
its good approach), is found. One ProcTHOR find (~50 s): slow tier."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _sweep():
    spec = importlib.util.spec_from_file_location(
        "nogo_sweep_3161_supersede", ROOT / "evaluations" / "nogo-3161" / "sweep.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_a_good_approach_is_kept_and_val102_is_found():
    row = _sweep().run("procthor:val:102", 0, "none")
    assert row["outcome"] == "found"
    assert row["sim_s"] < 300                 # dev: out_of_budget at 1,588 s


