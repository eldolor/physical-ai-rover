# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
3.176.B: the room hint in the simulator, pinned. The sweep's seven arms
(`evaluations/request-3176/sweep.py`, 3.116's 100 selection houses x modes R
and U) are kept in `evaluations/request-3176/sim/`; this re-scores them with
`score_sim.py` and checks the result is the recorded one
(`sim/score.json`), so a re-run that moves a number is seen. Each bar's
verdict is pinned as it came out: a failed bar stays recorded as failed.
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SIM = ROOT / "evaluations" / "request-3176" / "sim"
sys.path.insert(0, str(ROOT / "evaluations" / "request-3176"))
import score_sim                                                # noqa: E402

# The verdicts as measured (the section file, "3.176.B results").
VERDICTS = {"S1": True, "S2": False, "S3": True, "S4": True, "S5": False,
            "S6": True, "S7": False}


def test_the_recorded_score_is_what_the_arms_score(tmp_path):
    out = tmp_path / "score.json"
    score_sim.main(str(SIM), str(out))
    assert json.loads(out.read_text()) == json.loads((SIM / "score.json").read_text())


def test_each_bar_is_pinned_as_it_came_out():
    rec = json.loads((SIM / "score.json").read_text())
    assert {k: rec[k]["met"] for k in VERDICTS} == VERDICTS


def test_off_is_off_and_nothing_crashed():
    """S6 and S7 hold whatever the progress bars say: a find without a hint,
    a hint with no labels and a room no house has all run as before."""
    rec = json.loads((SIM / "score.json").read_text())
    for k in ("parsed_vs_none", "nolabels_vs_dev", "absent_vs_none"):
        assert rec["S6"][k]["differ"] == 0 and rec["S6"][k]["missions"] >= 190, k
    assert not any(rec["S7"]["exceptions"].values())
