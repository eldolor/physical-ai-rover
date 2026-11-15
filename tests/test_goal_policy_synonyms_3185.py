# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.185 (bar S1): a person's room name that is a vocabulary synonym reaches
its room type in the rooms policy (`docs/plans/ros-alignment/
3.185-room-name-synonyms.md`). Only whole vocabulary names map; a name that
merely contains a room word, or lies outside the vocabulary, reads as none."""

import pytest

from brain.goal_policy import FEATURES, RoomGoalPolicy

M = {"resolution_m": 0.3, "width": 4, "height": 1, "origin_x_m": 0.0,
     "origin_y_m": 0.0, "cells": [0, 0, 0, 0]}
POSE = {"x_m": 0.15, "y_m": 0.15, "heading_deg": 90.0}
GOALS = [(0.45, 0.15), (0.75, 0.15), (1.05, 0.15)]
F = [[0.0] * len(FEATURES)] * 3
CANON = ["living room", "bedroom", "bathroom"]


def _extend(types, target="garbage can", hint=None):
    return RoomGoalPolicy.load().extend(M, POSE, GOALS, F, trail=[], types=types,
                                        target=target, hint=hint)


@pytest.mark.parametrize("types", [["Lounge", "Master Bedroom", "Washroom"],
                                   ["den", "guest room", "restroom"],
                                   ["Family Room", "NURSERY", "powder room"]])
def test_a_synonym_reads_as_its_type(types):
    assert _extend(types) == _extend(CANON)


def test_kitchenette_is_a_kitchen():
    assert _extend(["Kitchenette", None, None], target="garbage bag") == \
        _extend(["kitchen", None, None], target="garbage bag")


@pytest.mark.parametrize("name", ["Kids Den", "Bathroom Closet", "Kitchen Pantry",
                                  "Guest Room Closet", "Ben's Room", "Sunroom",
                                  "Playroom", "Study", "Hall"])
def test_a_name_that_is_not_a_whole_vocabulary_entry_of_a_prior_type_reads_as_none(name):
    """Contains a room word but is not one; outside the vocabulary; or a
    vocabulary type the prior does not name (study -> office, hall ->
    hallway): none, as on dev."""
    assert _extend([name, None, None]) == _extend([None, None, None])


def test_a_name_hint_on_a_synonym_named_room_keeps_its_rate():
    """The hint reads the room's own name first; the synonym fallback never
    takes the hinted 1.0 away."""
    hint = {"name": "Lounge"}
    hinted = _extend(["Lounge", "bedroom", None], hint=hint)
    assert hinted != _extend(["Lounge", "bedroom", None])          # the hint acts
    assert hinted == _extend(["living room", "bedroom", None],
                             hint={"type": "living room"})        # as a type hint would
