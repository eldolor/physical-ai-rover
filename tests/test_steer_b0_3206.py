# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.206.B0: the open teachers' criterion-2 numbers on S1, from committed
records, pinned (amendment 2)."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("b0_3206", ROOT / "evaluations/steer-distill-3206/b0.py")
b0 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(b0)


def test_arms_pinned():
    got = {n: (s["kept"], s["kept_right"], s["meets_criterion_2"]) for n, s in b0.arms().items()}
    assert got == {
        "72B": (57, 54, False),                   # 3.214's criterion 6, reproduced
        "Florence-2-base": (58, 55, False),
        "Florence-2-large": (59, 56, False),
        "72B + Florence-2-base": (56, 53, False),
        "72B + Florence-2-large": (57, 54, False),
    }


def test_kept_rule():
    direct = {"a": "LEFT", "b": "FORWARD", "c": "RIGHT", "d": None}
    mirrored = {"a": "RIGHT", "b": "FORWARD", "c": "RIGHT", "d": "LEFT"}
    assert b0.kept(direct, mirrored) == {"a": "LEFT", "b": "FORWARD"}
    assert b0.pair({"a": "LEFT", "b": "FORWARD"}, {"a": "LEFT", "b": "RIGHT"}) == {"a": "LEFT"}


def test_score_bar_is_criterion_2():
    truth = {str(i): "LEFT" for i in range(20)}
    at_bar = {str(i): "LEFT" for i in range(14)}                  # 70% kept, all right
    assert b0.score(at_bar, truth)["meets_criterion_2"]
    short = {**{str(i): "LEFT" for i in range(13)}, "13": "RIGHT", "14": "LEFT",
             **{str(i): "LEFT" for i in range(15, 20)}}           # 19 of 20 right: 95%
    assert b0.score(short, truth)["meets_criterion_2"]
    short["15"] = "RIGHT"                                         # 18 of 20: 90%
    assert not b0.score(short, truth)["meets_criterion_2"]
