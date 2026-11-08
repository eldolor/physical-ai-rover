# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""evaluations/autonomy-3158/B/make_sheet.py -- writes the frozen sim house sheet (sheet.json)."""

import json
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from sim.maps import build_world  # noqa: E402
from tests.demo_explore import reachable_cells  # noqa: E402

HOUSE = "home_first_floor_stairs"
# None's name is a substring of another object's label (the sim's match).
TARGETS = ["red backpack", "side table", "toilet", "vanity", "washer",
           "refrigerator", "bookcase", "tv console", "dining table", "breakfast table"]
ABSENT = ["unicorn", "blue bicycle"]


def main():
    world = build_world(HOUSE)
    reach = reachable_cells(world)
    room = {c: n for n, cs in world.rooms.items() for c in cs}
    where = {t: sorted({room.get(c) for c, n in world.objects.items() if n == t}) for t in TARGETS}
    cells = {t: sum(1 for n in world.objects.values() if n == t) for t in TARGETS}
    rng = random.Random(3158)
    finds = [{"job": "find", "target": t, "present": True} for t in TARGETS * 2 + TARGETS[:6]]
    finds += [{"job": "find", "target": t, "present": False} for t in ABSENT * 2]
    counted = finds + [{"job": "survey"} for _ in range(5)]
    rng.shuffle(counted)
    stretch = [{"job": "find", "target": t, "present": True} for t in rng.sample(TARGETS, 8)]
    stretch += [{"job": "survey"} for _ in range(2)]
    rng.shuffle(stretch)
    sheet = {"house": HOUSE,
             "rooms": sorted(n for n, cs in world.rooms.items() if set(cs) & set(reach)),
             "dock_cell": [world.robot_x, world.robot_y],
             "dock_room": room.get((world.robot_x, world.robot_y)),
             "targets": [{"name": t, "rooms": where[t], "cells": cells[t]} for t in TARGETS],
             "absent": ABSENT,
             "schedule": [dict(m, phase="counted") for m in counted]
             + [dict(m, phase="stretch") for m in stretch]}
    Path(__file__).with_name("sheet.json").write_text(json.dumps(sheet, indent=1) + "\n")


if __name__ == "__main__":
    main()
