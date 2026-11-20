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
# A zone no longer than a house's room is wide (Thermos review: the bridge
# rasterises each in Python under its lock; 30 of these stay ~1 M cells).
MAX_SIDE_M = 15.0
# 3.161.B part 2 (E1): a zone this close to the trip home's last leg (the
# staging point to the dock) covers the dock: the chassis' circumscribed
# radius, hypot(0.253/2, 0.231/2) = 0.171 m, rounded up (pinned against
# robot/safety.py's footprint by tests/test_no_go_dock.py; not imported, so
# this module stays free of the robot server).
DOCK_CLEAR_M = 0.18
KINDS = ("area", "wall")
# Explore's own masked map only (never a WorldInterface answer): a cell a
# zone (grown by HALF_WIDTH_M) covers. Neither free (no goal, no path), nor
# unknown (no frontier beside it), nor occupied (no obstacle clearance grown
# around it: nav2's keepout is lethal to the robot's centre and is not
# inflated, so a doorway beside a zone stays a doorway).
CELL_NOGO = 2
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
    if max(x1 - x0, y1 - y0) > MAX_SIDE_M:
        raise ZoneRefused(f"a zone is at most {MAX_SIDE_M:g} m a side")
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


def _seg_point(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _crosses(a, b, c, d) -> bool:
    """Segments ab and cd intersect (touching counts)."""
    def orient(p, q, r):
        v = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        return (v > 1e-12) - (v < -1e-12)

    def on(p, q, r):
        return (min(p[0], r[0]) - 1e-12 <= q[0] <= max(p[0], r[0]) + 1e-12
                and min(p[1], r[1]) - 1e-12 <= q[1] <= max(p[1], r[1]) + 1e-12)
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    if o1 != o2 and o3 != o4:
        return True
    return ((o1 == 0 and on(a, c, b)) or (o2 == 0 and on(a, d, b))
            or (o3 == 0 and on(c, a, d)) or (o4 == 0 and on(c, b, d)))


def segment_distance(z: dict, a, b) -> float:
    """Metres from the segment a-b to the zone's rectangle; 0 if they meet."""
    (ax, ay), (bx, by) = a, b
    if inside(z, ax, ay) or inside(z, bx, by):
        return 0.0
    cs = corners(z)
    edges = list(zip(cs, cs[1:] + cs[:1]))
    if any(_crosses((ax, ay), (bx, by), c, d) for c, d in edges):
        return 0.0
    return min([distance(z, ax, ay), distance(z, bx, by)]
               + [_seg_point(cx, cy, ax, ay, bx, by) for cx, cy in cs])


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


PINCH_PASSES = 4                 # 3.196: rounds of `close_pinches` (each settles the cells it touches)


def close_pinches(m: dict, cell_value: int, free: int = 0) -> dict:
    """3.196 D1: a masked map with no diagonal step past a zone's corner.

    An 8-connected walk (`brain/frontier.py`'s `reach`) steps diagonally
    between two free cells whose two shared neighbours are both blocked: a
    gap of no width. Beside a wall it is old and rare (the simulator's walls
    are cell-aligned); beside a zone it is how explore reached frontiers
    behind a zone that filled a door (`procthor:val:15`), which no navigator
    can. Every 2 x 2 block whose diagonal pair is `free` while the other two
    are not, at least one of them `cell_value`, gets its free pair set to
    `cell_value` too, for up to `PINCH_PASSES` rounds (a closed pair can
    make a new pinch beside it). Returns `m` itself when nothing is closed;
    otherwise a copy, `cells` of the same type, `pinch_cells` counted."""
    if not m or not m.get("usable") or "cells" not in m:
        return m
    import numpy as np
    w, h = m["width"], m["height"]
    cells = m["cells"]
    a = np.array(cells).reshape(h, w)          # a copy: the input is never changed
    closed = 0
    for _ in range(PINCH_PASSES):
        ok = a == free
        zone = a == cell_value
        shut = ~ok
        mark = np.zeros_like(ok)
        tl, tr = (slice(0, -1), slice(0, -1)), (slice(0, -1), slice(1, None))
        bl, br = (slice(1, None), slice(0, -1)), (slice(1, None), slice(1, None))
        for p, q, s, t in ((tl, br, tr, bl), (tr, bl, tl, br)):
            pinch = ok[p] & ok[q] & shut[s] & shut[t] & (zone[s] | zone[t])
            mark[p] |= pinch
            mark[q] |= pinch
        if not mark.any():
            break
        closed += int(mark.sum())
        a[mark] = cell_value
    if not closed:
        return m
    out = dict(m)
    out["cells"] = a.ravel().tolist() if isinstance(cells, list) else a.ravel()
    out["pinch_cells"] = closed
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
