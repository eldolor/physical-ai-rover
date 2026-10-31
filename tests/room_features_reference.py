"""3.131: `brain/goal_policy.py`'s `coarse()` and `room_features()` frozen as
they were on `dev` before 3.131 (9edb50e5), with the helpers they call. The
new functions -- given this choice's `goal_context()` or not -- must give
exactly these answers (`test_room_features_ctx.py`, and the slow
`test_room_features_ctx_sweep.py`). Do not edit; `segment()` has its own
frozen copy (`room_cut_reference.py`)."""
import numpy as np

from brain.frontier import _cell_of
from brain.room_labels import segment
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN

ROOM_AREA_M2 = 20.0
ROOM_RES_M = 0.30


def coarse(m: dict, res_m: float = ROOM_RES_M) -> dict:
    """`m` at `res_m` (an integer factor of its own resolution): a coarse
    cell is occupied if any of its cells is, free if at least half are, and
    unknown otherwise. A map already that coarse is returned as it is."""
    k = int(round(res_m / m["resolution_m"]))
    if k <= 1:
        return m
    w, h = m["width"], m["height"]
    W, H = -(-w // k), -(-h // k)
    cells = np.full((H * k, W * k), CELL_UNKNOWN, dtype=np.asarray(m["cells"]).dtype)
    cells[:h, :w] = np.asarray(m["cells"]).reshape(h, w)
    blocks = cells.reshape(H, k, W, k).swapaxes(1, 2).reshape(H, W, k * k)
    out = np.full((H, W), CELL_UNKNOWN, dtype=cells.dtype)
    out[(blocks == CELL_FREE).sum(axis=2) * 2 >= k * k] = CELL_FREE
    out[(blocks == CELL_OCCUPIED).any(axis=2)] = CELL_OCCUPIED
    return {**m, "width": W, "height": H, "resolution_m": m["resolution_m"] * k,
            "cells": out.ravel().tolist()}


def _seg_at(seg, m: dict, x_m: float, y_m: float) -> int:
    """The segment id under a point, or of the nearest floor cell within one
    cell (a goal can sit on a frontier's unknown edge); -1 if none."""
    return _seg_at_cell(seg, *_cell_of(m, x_m, y_m))


def _seg_at_cell(seg, cx: int, cy: int) -> int:
    h, w = seg.shape
    for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
        i, j = cx + dx, cy + dy
        if 0 <= i < w and 0 <= j < h and seg[j, i] >= 0:
            return int(seg[j, i])
    return -1


def room_features(m: dict, pose: dict, goals: list, *, trail, types: list,
                  prior: dict) -> list:
    """Each candidate's ROOM_FEATURES. `goals`: the pool's (x_m, y_m) in
    order; `trail`: map-frame points the robot has stood on; `types`: each
    goal's room type from the labels, or None; `prior`: type -> P(target)."""
    m = coarse(m)
    seg, _ = segment(m)
    res2 = m["resolution_m"] ** 2
    ids, counts = np.unique(seg[seg >= 0], return_counts=True)
    area = dict(zip(ids.tolist(), counts.tolist()))
    here = _seg_at(seg, m, pose["x_m"], pose["y_m"])
    # The trail once per map cell it touches, not once per point.
    pts = np.asarray(list(trail), dtype=float).reshape(-1, 2)
    cells = np.unique(np.floor((pts - (m["origin_x_m"], m["origin_y_m"]))
                               / m["resolution_m"]).astype(int), axis=0)
    stood = {_seg_at_cell(seg, int(i), int(j)) for i, j in cells} | {here}
    rooms = [_seg_at(seg, m, x, y) for x, y in goals]
    out = []
    for r, t in zip(rooms, types):
        new = 1.0 if r >= 0 and r not in stood else 0.0
        p = float(prior.get(t, 0.0)) if t is not None else 0.0
        out.append([
            new,
            (area.get(r, 0) * res2 / ROOM_AREA_M2) if r >= 0 else 0.0,
            1.0 if r >= 0 and r != here else 0.0,
            (sum(x == r for x in rooms) / len(rooms)) if r >= 0 else 0.0,
            0.0 if t is None else 1.0,
            p,
            p * new,
        ])
    return out
