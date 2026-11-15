# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.183.B (bar B1): the rooms policy reads a goal's room type without case
(`docs/plans/ros-alignment/3.183-small-fixes-1115.md`). A person's room named
"Kitchen" (3.160.D's names) is the prior's "kitchen"; before 3.183 it read as
no type (3.176.B's repro)."""

import pytest

from brain.goal_policy import FEATURES, RoomGoalPolicy

M = {"resolution_m": 0.3, "width": 4, "height": 1, "origin_x_m": 0.0,
     "origin_y_m": 0.0, "cells": [0, 0, 0, 0]}
POSE = {"x_m": 0.15, "y_m": 0.15, "heading_deg": 90.0}
GOALS = [(0.45, 0.15), (0.75, 0.15), (1.05, 0.15)]
F = [[0.0] * len(FEATURES)] * 3


def _extend(types, target="garbage bag", hint=None):
    return RoomGoalPolicy.load().extend(M, POSE, GOALS, F, trail=[], types=types,
                                        target=target, hint=hint)


@pytest.mark.parametrize("types", [["Kitchen", "Living Room", "Bedroom"],
                                   ["KITCHEN", "living ROOM", "bedRoom"]])
def test_a_capitalised_type_reads_as_the_prior_names_it(types):
    assert _extend(types) == _extend(["kitchen", "living room", "bedroom"])


def test_the_repro_now_types_both():
    """3.176.B's repro: a capitalised kitchen was None, so its prior rate
    (garbage bag: kitchen 1.0) never reached the features."""
    assert _extend(["Kitchen", None, None]) != _extend([None, None, None])


def test_a_type_the_prior_does_not_name_still_reads_as_none():
    assert _extend(["Sunroom", "garage hall", None]) == _extend([None, None, None])


def test_a_hint_and_a_capitalised_name_agree():
    """A hint already matched without case; it still reads the same room."""
    hint = {"name": "Kitchen"}
    assert _extend(["Kitchen", "den", None], hint=hint) == \
        _extend(["kitchen", "den", None], hint=hint)


def test_edge_spaces_are_read_as_hint_matches_reads_them():
    """Review: a stray space ("Kitchen ") read as no type while a name hint
    (which strips) still matched it."""
    assert _extend([" Kitchen ", "living room ", None]) == _extend(["kitchen", "living room", None])


def test_the_sweep_scores_are_the_recorded_ones():
    """3.183.B's sweeps (evaluations/small-fixes-3183/): B2 and B3 met, and the
    fault's size on dev, re-scored from the committed arms."""
    import importlib.util
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / "evaluations/small-fixes-3183/b_score.py"
    spec = importlib.util.spec_from_file_location("b_score_3183", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    arms = {f"{t}_{a}": mod.load(f"b_{t}_{a}.json")
            for t in ("dev", "branch") for a in ("dev", "names", "names_title")}
    assert mod.differ(arms["dev_names"], arms["branch_names"]) == []
    assert mod.differ(arms["dev_dev"], arms["branch_dev"]) == []
    assert mod.differ(arms["branch_names_title"], arms["branch_names"]) == []
    assert mod.differ(arms["dev_names_title"], arms["dev_dev"]) == []
    assert len(mod.differ(arms["dev_names_title"], arms["dev_names"])) == 175
