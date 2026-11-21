# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.200.C's recorded sweeps, pinned (CLAUDE.md 7.3): the comparison over the
committed rows gives the numbers and verdicts the plan file records
(docs/plans/ros-alignment/3.200-recall-other-side.md, "3.200.C results"). R1
did not ship; its code and unit tests live at `fa85662e`."""

import contextlib
import importlib.util
import io
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
D = ROOT / "evaluations/recall-other-side-3200"
_spec = importlib.util.spec_from_file_location("wide_3200", D / "wide.py")
wide = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(wide)


def _compare():
    with contextlib.redirect_stdout(io.StringIO()):
        return wide.compare(D / "rows-C-dev.jsonl", D / "rows-C-off.jsonl", D / "rows-C-R1.jsonl")


def test_the_count_with_the_rule_off():
    with contextlib.redirect_stdout(io.StringIO()):
        s = wide.score(D / "rows-C-off.jsonl")
    assert (s["finds"], s["found"], s["face_refused"], s["face_refused_events"]) == (240, 232, 21, 21)
    assert s["face_refused_not_found"] == [1013, 2013, 3017, 4012, 4026, 5014, 5020, 5036]
    assert (s["retries"], s["wrong_found"], s["contacts"], s["capped"]) == (0, 0, 0, 0)
    assert all(v["outcome"] == "exhausted" and v["search_s"] == 202.3 for v in s["surveys"].values())


def test_bars_as_recorded():
    c = _compare()
    assert c["finds"] == {"dev": 240, "off": 240, "R1": 240} and c["refused"] == 21
    assert c["found"] == {"dev": 232, "off": 232, "R1": 236}
    assert c["C1"] == {"bar": 236, "met": True}
    assert c["won"] == [1013, 2013, 4012, 4026, 5014] and c["lost"] == [3025]
    assert c["C2"]["met"] is False                                  # the verdict: nothing ships
    assert c["refused_found"] == {"dev": 13, "R1": 17}
    assert c["C3"] == {"wrong_found": 0, "contacts": 0, "met": True}
    assert c["C4"] == {"per_place": 2, "per_mission": 2, "capped": 0, "retries": 15, "spent": 0, "met": True}
    assert c["C5"]["missions"] == 219 and c["C5"]["differ"] == [] and c["C5"]["met"]
    assert c["C5"]["max_search_s_delta"] == 0.0 and c["C5"]["surveys_identical"]
    assert c["C6"]["differ"] == [] and c["C6"]["met"] and c["C6"]["max_search_s_delta"] == 0.0


def test_dev_keeps_one_try():
    import brain.explore as ex
    assert not hasattr(ex, "RECALL_FACE_RETRY")
