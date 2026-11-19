# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
world/nogo.py -- 3.161: no-go zones a person draws on the twin, as geometry.

A zone (3.161.A D1) is an axis-aligned rectangle in the saved map's frame
(the house frame `WorldInterface.get_pose()` answers in, while its
`frame_id` names a saved map), at least MIN_SIDE_M a side, at most
MAX_ZONES a map:

    {"id": "z<hex>", "x0": m, "y0": m, "x1": m, "y1": m,
     "kind": "area" | "wall", "set_at": epoch s, "by": "person"}

with x0 < x1 and y0 < y1. A wall is a rectangle WALL_WIDTH_M wide drawn as
a line (`wall_from_line`), snapped to the line's longer axis.

A zone is NOT a safety veto (CLAUDE.md rule 4): `robot/safety.py` never
reads this module. It shapes where autonomy plans -- explore's goal choice
(D3), the robot server's refusal of a goal inside one (D5) and the
bridge's keepout mask (D2, D4) -- and nothing else.

Pure Python, plus numpy where a map is masked (imported there only), so
the robot server and `control/` can import it without numpy.
"""

from __future__ import annotations

import math
import re
import time
import uuid
from typing import Iterable, Optional

MIN_SIDE_M = 0.20
MAX_ZONES = 30
WALL_WIDTH_M = 0.10
# D3: explore drops a candidate whose point lies inside a zone grown by the
# robot's half-width.
HALF_WIDTH_M = 0.12
# D4: the bridge leaves a zone the robot is in out of the mask until the
# robot is this far clear of it; the brain's way out aims a little further.
LEAVE_M = 0.30
LEAVE_GOAL_M = LEAVE_M + 0.05
MAX_COORD_M = 1000.0             # 1 km from the map's origin: anything else is damage
KINDS = ("area", "wall")
ID_RE = re.compile(r"z[0-9a-f]{1,32}")


class ZoneRefused(ValueError):
    """A zone (or a set) that cannot be kept as given."""


def _num(v, what: str) -> float:
    if isinstance(v, bool):
        raise ZoneRefused(f"{what} is not a number")
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ZoneRefused(f"{what} is not a number") from None
    if not math.isfinite(f) or abs(f) > MAX_COORD_M:
        raise ZoneRefused(f"{what} is out of range")
    return f


def new_id() -> str:
    return "z" + uuid.uuid4().hex[:12]


def clean_zone(z, now: Optional[float] = None) -> dict:
    """One zone, checked and normalised (corners ordered, rounded to the
    millimetre). Raises ZoneRefused. A zone without an id gets one; one
    without `set_at` gets `now`."""
    if not isinstance(z, dict):
        raise ZoneRefused("a zone is an object")
    xa, xb = _num(z.get("x0"), "x0"), _num(z.get("x1"), "x1")
    ya, yb = _num(z.get("y0"), "y0"), _num(z.get("y1"), "y1")
    x0, x1 = sorted((xa, xb))
    y0, y1 = sorted((ya, yb))
    kind = z.get("kind", "area")
    if kind not in KINDS:
        raise ZoneRefused(f"a zone's kind is one of {KINDS}")
    short = min(x1 - x0, y1 - y0)
    # A wall is WALL_WIDTH_M across by design; its length is what must be MIN_SIDE_M.
    if kind == "wall":
        if max(x1 - x0, y1 - y0) < MIN_SIDE_M - 1e-9 or short < WALL_WIDTH_M - 1e-6:
            raise ZoneRefused(f"a wall is at least {MIN_SIDE_M:g} m long")
    elif short < MIN_SIDE_M - 1e-9:
        raise ZoneRefused(f"a zone is at least {MIN_SIDE_M:g} m a side")
    zid = z.get("id") or new_id()
    if not isinstance(zid, str) or not ID_RE.fullmatch(zid):
        raise ZoneRefused("a zone's id is z<hex>")
    set_at = z.get("set_at")
    set_at = _set_at(set_at, now)
    return {"id": zid, "x0": round(x0, 3), "y0": round(y0, 3), "x1": round(x1, 3),
            "y1": round(y1, 3), "kind": kind, "set_at": set_at, "by": "person"}


def _set_at(v, now: Optional[float]) -> float:
    if v is None:
        return round(time.time() if now is None else now, 3)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ZoneRefused("set_at is not a time")
    return round(float(v), 3)


def clean_zones(zones) -> list:
    """A whole set, each zone checked, ids unique, at most MAX_ZONES."""
    if not isinstance(zones, list):
        raise ZoneRefused("zones is a list")
    if len(zones) > MAX_ZONES:
        raise ZoneRefused(f"at most {MAX_ZONES} zones a map")
    out = [clean_zone(z) for z in zones]
    if len({z["id"] for z in out}) != len(out):
        raise ZoneRefused("two zones share an id")
    return out


def wall_from_line(xa: float, ya: float, xb: float, yb: float) -> dict:
    """A line a person drew -> a wall zone (no id yet): WALL_WIDTH_M across,
    along the line's longer axis, centred on the line."""
    xa, ya, xb, yb = (_num(v, "a wall's end") for v in (xa, ya, xb, yb))
    half = WALL_WIDTH_M / 2
    if abs(xb - xa) >= abs(yb - ya):
        ym = (ya + yb) / 2
        return {"x0": min(xa, xb), "x1": max(xa, xb), "y0": ym - half, "y1": ym + half,
                "kind": "wall"}
    xm = (xa + xb) / 2
    return {"x0": xm - half, "x1": xm + half, "y0": min(ya, yb), "y1": max(ya, yb),
            "kind": "wall"}


# ---------- geometry ----------

def distance(z: dict, x: float, y: float) -> float:
    """Metres from (x, y) to the zone's rectangle; 0 inside it."""
    dx = max(z["x0"] - x, 0.0, x - z["x1"])
    dy = max(z["y0"] - y, 0.0, y - z["y1"])
    return math.hypot(dx, dy)


def inside(z: dict, x: float, y: float, grow: float = 0.0) -> bool:
    """(x, y) lies in the zone grown by `grow` metres (a rounded rectangle)."""
    return distance(z, x, y) <= grow if grow > 0 else (
        z["x0"] <= x <= z["x1"] and z["y0"] <= y <= z["y1"])


def zone_at(zones: Iterable[dict], x: float, y: float, grow: float = 0.0) -> Optional[dict]:
    """The first zone (x, y) lies in, grown by `grow`; None if none."""
    for z in zones or ():
        if inside(z, x, y, grow):
            return z
    return None


def nearest_distance(zones: Iterable[dict], x: float, y: float) -> float:
    return min((distance(z, x, y) for z in zones or ()), default=math.inf)


def corners(z: dict) -> list:
    """The rectangle's four corners, counter-clockwise from (x0, y0)."""
    return [(z["x0"], z["y0"]), (z["x1"], z["y0"]), (z["x1"], z["y1"]), (z["x0"], z["y1"])]


def area_m2(zones: Iterable[dict]) -> float:
    return sum((z["x1"] - z["x0"]) * (z["y1"] - z["y0"]) for z in zones or ())


# ---------- on a map (numpy here only) ----------

def cell_mask(m: dict, zones: Iterable[dict], grow: float = 0.0):
    """A (height, width) bool array: the map's cells whose CENTRE lies in a
    zone grown by `grow`."""
    import numpy as np
    w, h, res = int(m["width"]), int(m["height"]), float(m["resolution_m"])
    ox, oy = float(m["origin_x_m"]), float(m["origin_y_m"])
    out = np.zeros((h, w), dtype=bool)
    if not w or not h:
        return out
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    for z in zones or ():
        dx = np.maximum(np.maximum(z["x0"] - xs, 0.0), xs - z["x1"])
        dy = np.maximum(np.maximum(z["y0"] - ys, 0.0), ys - z["y1"])
        if grow > 0:
            out |= (dy[:, None] ** 2 + dx[None, :] ** 2) <= grow ** 2
        else:
            out |= (dy[:, None] <= 0) & (dx[None, :] <= 0)
    return out


def masked_map(m: dict, zones: Iterable[dict], cell_value: int, grow: float = 0.0) -> dict:
    """A copy of a usable map with every cell inside a zone set to
    `cell_value` (explore uses occupied: no frontier, no goal, no path
    through it). The input is never changed; `cells` keeps its type (a list
    stays a list). With no zones, the map itself."""
    zones = list(zones or ())
    if not zones or not m or not m.get("usable") or "cells" not in m:
        return m
    import numpy as np
    mask = cell_mask(m, zones, grow).ravel()
    if not mask.any():
        return m
    cells = m["cells"]
    arr = np.array(cells, dtype=np.int64 if not hasattr(cells, "dtype") else cells.dtype).ravel()
    arr[mask] = cell_value
    out = dict(m)
    out["cells"] = arr.tolist() if isinstance(cells, list) else arr
    out["nogo_cells"] = int(mask.sum())
    return out
