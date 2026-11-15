# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
3.180.B: the coverage record counts the faces a detection could see. A
target is a solid object (an occupied cell); floor seen around it does not
say it was in view. `camera_faces` gives the occupied cells camera rays reach
first; `find_view_gaps(faces=)` makes an object's unseen face a patch to look
at; without `faces` the function is as before (bar B7).
"""
from brain.frontier import camera_faces, camera_seen, find_view_gaps
from world.interface import CELL_FREE, CELL_OCCUPIED

RES = 0.1


def _room(w=40, h=20, objects=()):
    """A walled room of free floor; `objects` are occupied cells inside it."""
    cells = []
    for j in range(h):
        for i in range(w):
            wall = i in (0, w - 1) or j in (0, h - 1)
            cells.append(CELL_OCCUPIED if wall or (i, j) in objects else CELL_FREE)
    return {"usable": True, "width": w, "height": h, "resolution_m": RES,
            "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": cells}


def _centre(i, j):
    return ((i + 0.5) * RES, (j + 0.5) * RES)


def test_camera_faces_are_the_occupied_cells_rays_reach_first():
    m = _room(objects={(20, 10)})
    faces = camera_faces(m, *_centre(10, 10), 90.0)       # facing east (+x, compass 90) at it
    assert (20, 10) in faces
    assert (21, 10) not in faces                          # behind it: never reached
    assert all(m["cells"][j * m["width"] + i] == CELL_OCCUPIED for i, j in faces)


def test_camera_faces_respect_range_and_field_of_view():
    m = _room(objects={(20, 10)})
    assert (20, 10) not in camera_faces(m, *_centre(10, 10), 270.0)   # facing away (west)
    assert (20, 10) not in camera_faces(m, *_centre(10, 10), 90.0, range_m=0.5)


def test_an_object_beside_seen_floor_with_an_unseen_face_is_a_gap():
    """The 247 case: the floor round the object is all seen, its face was
    never reached by a ray. With `faces`, a gap is offered at it; without,
    none (as before)."""
    obj = (20, 10)
    m = _room(objects={obj})
    seen = {(i, j) for j in range(1, 19) for i in range(1, 39)
            if m["cells"][j * m["width"] + i] == CELL_FREE}
    here = _centre(5, 10)
    assert find_view_gaps(m, *here, seen) == []
    walls_seen = {(i, j) for j in range(20) for i in range(40)
                  if m["cells"][j * m["width"] + i] == CELL_OCCUPIED and (i, j) != obj}
    gaps = find_view_gaps(m, *here, seen, faces=walls_seen)
    assert gaps, "the object's unseen face must be a patch to look at"
    cx, cy = gaps[0].centre
    assert abs(cx - _centre(*obj)[0]) < 0.25 and abs(cy - _centre(*obj)[1]) < 0.25


def test_a_face_already_seen_makes_no_gap():
    obj = (20, 10)
    m = _room(objects={obj})
    seen = {(i, j) for j in range(1, 19) for i in range(1, 39)
            if m["cells"][j * m["width"] + i] == CELL_FREE}
    every_face = {(i, j) for j in range(20) for i in range(40)
                  if m["cells"][j * m["width"] + i] == CELL_OCCUPIED}
    assert find_view_gaps(m, *_centre(5, 10), seen, faces=every_face) == []


def test_without_faces_the_answer_is_the_old_one():
    m = _room(objects={(20, 10), (30, 5)})
    here = _centre(5, 10)
    seen = camera_seen(m, *here, 0.0)
    assert find_view_gaps(m, *here, seen) == find_view_gaps(m, *here, seen, faces=None)


# ---------- 3.180.B's sweep, pinned (failed bars stay recorded as failed) ----------

import json                                                     # noqa: E402
from pathlib import Path                                        # noqa: E402

B = Path(__file__).resolve().parent.parent / "evaluations" / "explore-view-3180" / "b"
VERDICTS = {"B1": True, "B2": False, "B3": False, "B4": False, "B5": False, "B6": False}


def test_b_bars_as_measured():
    rec = json.loads((B / "score_b.json").read_text())
    assert {k: rec[k]["met"] for k in VERDICTS} == VERDICTS
    assert len(rec["B3"]["lost"]) == 9 and rec["B6"]["exhausted_B_base"] == [9, 1]


def test_b_rows_hold_the_lost_and_the_traced():
    rows = [json.loads(line) for line in (B / "rows.jsonl").read_text().splitlines()]
    at = {(r["arm"], r["b"]["index"], r["b"]["mode"]): r for r in rows}
    assert at[("wrong", 247, "R")]["b"]["outcome"] == "found"
    assert at[("wrong", 247, "R")]["base"]["outcome"] == "exhausted"
    assert at[("right", 194, "U")]["b"]["outcome"] == "exhausted"
    lost = json.loads((B / "score_b.json").read_text())["B3"]["lost"]
    for arm, i, mode, _ in lost:
        r = at[(arm, i, mode)]
        assert (r["base"]["outcome"], r["b"]["outcome"]) == ("found", "exhausted")
