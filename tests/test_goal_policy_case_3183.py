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
