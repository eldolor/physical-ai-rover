# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/room-prior-3190/targets.py -- 3.190.B's 40 fresh targets, fixed
in A before any call (amendment 1; pinned by
tests/test_room_prior_set_3190.py).

None is a key of the shipped prior (`brain/explore_goal_rooms.json`) or
among 3.116.C's 29 targets (the prompt was written with those in view), and
none is a near-synonym of one. EXPECTED holds the 20 whose room type was
judged unambiguous when the set was written (criterion 5); OPEN the 20
without one. Author: the author of the section (a Claude session,
2026-11-19).
"""

EXPECTED = {
    "spatula": "kitchen", "cutting board": "kitchen", "oven mitt": "kitchen",
    "dish soap": "kitchen", "kettle": "kitchen",
    "toilet paper": "bathroom", "shampoo bottle": "bathroom", "bath mat": "bathroom",
    "toothpaste": "bathroom", "razor": "bathroom",
    "alarm clock": "bedroom", "pajamas": "bedroom", "jewelry box": "bedroom",
    "duvet": "bedroom", "slippers": "bedroom",
    "video game controller": "living room", "board game": "living room",
    "sofa cushion": "living room", "dvd case": "living room",
    "fireplace poker": "living room",
}

OPEN = ("coffee mug", "scissors", "tape measure", "flashlight", "notebook", "pen",
        "sunglasses", "wristwatch", "tablet", "baseball cap", "jacket", "dog leash",
        "candle", "tissue box", "blanket", "hairbrush", "medicine bottle", "batteries",
        "stuffed animal", "spray bottle")

TARGETS = tuple(EXPECTED) + OPEN
