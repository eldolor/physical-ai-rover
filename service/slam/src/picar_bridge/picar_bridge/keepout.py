# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""keepout -- the mask behind POST /nav/keepout and POST /nav/zones, as plain
Python (3.68; 3.161's zones).

The robot server decides where nav2 must not plan (robot/wedge_watch.py);
this only draws those discs into an OccupancyGrid for nav2's stock
KeepoutFilter: 100 inside a disc (lethal to the robot's centre), -1
(unknown) elsewhere, which the filter skips -- it writes any other value
over an unknown master cell, so a 0 made unmapped space free (3.76 K5).
Kept out of bridge.py, which imports rclpy, so it is tested on a
laptop (tests/test_wedge_watch.py).
"""

import math

RES_M = 0.05
MARGIN_M = 0.10
UNKNOWN = -1      # nav_msgs/OccupancyGrid's unknown: the filter leaves the cell as it was

# 3.161: a person's no-go zones, drawn into the same mask as the spots (their
# union). Each arrives as a polygon in ROS's map frame (world/ros_world.py
# rotates the house-frame rectangle through the anchor). No expiry.
MAX_ZONES = 30
MAX_ZONE_M = 15.5         # world/nogo.py's 15 m side (rotation keeps a side's length), and rounding
# D4: a zone the robot is in is left out of the mask until the robot is
# LEAVE_M clear of it. It starts when the robot's centre is more than
# SUSPEND_DEPTH_M inside, or when the robot is BOXED by the zone's drawn cells
# (its own cell and its four neighbours all drawn: NavFn clears only the
# robot's own cell, so it could not plan out), or when the zone arrives with
# the robot within LEAVE_M of it. A path hugging the drawn edge never
# suspends a zone (an undrawn neighbour). 3.193: boxed is tested on the cells
# themselves, since a zone now marks every cell it touches.
LEAVE_M = 0.30
SUSPEND_DEPTH_M = 0.05
MIN_ZONE_WIDTH_M = 0.001  # area / longest side; world/nogo.py's thinnest, a wall, is 0.10 m


def parse(spots, now: float) -> list:
    """The route's body -> [{x_m, y_m, r_m, until}], in the map frame.
    Raises ValueError on a malformed spot rather than drawing a guess."""
    out = []
    for s in spots:
        x, y, r, ttl = (float(s[k]) for k in ("x_m", "y_m", "r_m", "ttl_s"))
        if not all(math.isfinite(v) for v in (x, y, r, ttl)) or r <= 0 or r > 1.0 or ttl <= 0:
            raise ValueError(f"bad keepout spot {s!r}")
        out.append({"x_m": x, "y_m": y, "r_m": r, "until": now + ttl})
    return out


def live(spots: list, now: float) -> list:
    return [s for s in spots if s["until"] > now]


def parse_zones(zones) -> list:
    """POST /nav/zones' body -> [{id, poly: [(x, y), ...]}], ROS's map frame.
    Raises ValueError on anything malformed rather than drawing a guess."""
    if not isinstance(zones, list) or len(zones) > MAX_ZONES:
        raise ValueError(f"zones is a list of at most {MAX_ZONES}")
    out = []
    for z in zones:
        zid, poly = z["id"], z["poly"]
        if not isinstance(zid, str) or not 0 < len(zid) <= 40:
            raise ValueError(f"bad zone id {zid!r}")
        if not isinstance(poly, list) or len(poly) != 4:
            raise ValueError(f"zone {zid}: a polygon of four corners")
        pts = [(float(p[0]), float(p[1])) for p in poly]
        if not all(math.isfinite(v) for p in pts for v in p):
            raise ValueError(f"zone {zid}: a corner is not finite")
        for a, b in zip(pts, pts[1:] + pts[:1]):
            if math.hypot(b[0] - a[0], b[1] - a[1]) > MAX_ZONE_M:
                raise ValueError(f"zone {zid}: a side longer than {MAX_ZONE_M:g} m")
        area = abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1]))) / 2
        longest = max(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:] + pts[:1]))
        if longest == 0 or area / longest < MIN_ZONE_WIDTH_M:
            # 3.193 (reviews): its edges would still mark a line of cells
            raise ValueError(f"zone {zid}: a polygon with no width")
        out.append({"id": zid, "poly": pts})
    if len({z["id"] for z in out}) != len(out):
        raise ValueError("two zones share an id")
    return out


def _inside_poly(poly, x: float, y: float) -> bool:
    """Even-odd rule (a convex quadrilateral here)."""
    inside = False
    n = len(poly)
    for i in range(n):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def _seg_dist(ax, ay, bx, by, x, y) -> float:
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def zone_distance(z: dict, x: float, y: float) -> float:
    """Signed metres from (x, y) to the zone's edge: positive outside,
    negative inside (the depth)."""
    p = z["poly"]
    d = min(_seg_dist(*p[i], *p[(i + 1) % len(p)], x, y) for i in range(len(p)))
    return -d if _inside_poly(p, x, y) else d


def suspend(zones: list, suspended: set, pose, fresh=(), phase=(0.0, 0.0)) -> set:
    """D4: the ids left out of the mask after this pose ((x, y) in the map
    frame, or None when unknown: nothing changes). `fresh`: ids that just
    arrived, which the robot within LEAVE_M of also suspends. `phase`: the
    mask's lattice (mask()'s)."""
    if pose is None:
        return {i for i in suspended if any(z["id"] == i for z in zones)}
    x, y = pose
    out, near = set(), []
    for z in zones:
        d = zone_distance(z, x, y)
        if z["id"] in suspended:
            if d < LEAVE_M:
                out.add(z["id"])                 # not yet clear of it
        elif d < -SUSPEND_DEPTH_M or (z["id"] in fresh and d < LEAVE_M):
            out.add(z["id"])
        elif d < 2.5 * RES_M:                    # may draw the robot's cell or a neighbour
            near.append(z)
    # Boxed by the zones still drawn, together (review: two zones a gap
    # apart each draw an edge cell; neither alone boxes the robot): every
    # zone that draws one of the five cells is left out.
    drawn = [z for z in near if z["id"] not in out]
    if drawn:
        px, py = phase
        i, j = math.floor((x - px) / RES_M), math.floor((y - py) / RES_M)
        five = [(px + (i + di) * RES_M, py + (j + dj) * RES_M)
                for di, dj in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))]
        by = [[z["id"] for z in drawn if _touches(z["poly"], cx, cy, RES_M)] for cx, cy in five]
        if all(by):
            out.update(i for ids in by for i in ids)
    return out


# 3.193: an edge closer than this to a cell's side is ON that side: it marks
# the cell only when the zone is on the cell's side of it (a zone whose edge
# lies on a cell boundary marks no extra row of cells).
EDGE_EPS_M = 1e-6


def _edge_cells(a, b, x0: float, y0: float, res: float, w: int, h: int):
    """The (i, j) cells whose square the segment a-b crosses with positive
    length inside (EDGE_EPS_M in from each side), column by column, so the
    work is the edge's cells, not its bounding box's."""
    (ax, ay), (bx, by) = a, b
    if bx < ax:
        (ax, ay), (bx, by) = (bx, by), (ax, ay)
    lo = max(int(math.floor((ax - x0) / res)), 0)
    hi = min(int(math.floor((bx - x0) / res)), w - 1)
    for i in range(lo, hi + 1):
        left = max(x0 + i * res + EDGE_EPS_M, ax)
        right = min(x0 + (i + 1) * res - EDGE_EPS_M, bx)
        if left > right:
            continue                     # the edge only grazes this column's side
        if bx == ax:
            ylo, yhi = min(ay, by), max(ay, by)
        else:
            t = (by - ay) / (bx - ax)
            ya, yb = ay + (left - ax) * t, ay + (right - ax) * t
            ylo, yhi = min(ya, yb), max(ya, yb)
        j_lo = max(int(math.floor((ylo - y0 + EDGE_EPS_M) / res)), 0)
        j_hi = min(int(math.floor((yhi - y0 - EDGE_EPS_M) / res)), h - 1)
        for j in range(j_lo, j_hi + 1):
            yield i, j


def _lattice(v: float, res: float, ph: float = 0.0) -> float:
    """3.193: the mask's cells sit on one fixed lattice, `ph + k * res` in the
    map frame, whatever else is drawn -- the bridge passes the SLAM map's own
    phase, so a mask cell IS a costmap cell (KeepoutFilter samples the mask at
    each costmap cell's centre) -- and a zone's cells do not move as other
    zones or spots come and go."""
    return ph + math.floor((v - ph) / res) * res


def phase_of(origin_x: float, origin_y: float, res: float = RES_M) -> tuple:
    """The lattice phase of a grid whose origin is (origin_x, origin_y), in
    [0, res): a phase within EDGE_EPS_M of res is 0 (review: floor() of
    -19.95 / 0.05 gives 0.04999...)."""
    def one(v):
        ph = v % res
        return 0.0 if ph > res - EDGE_EPS_M or ph < EDGE_EPS_M else ph
    return one(origin_x), one(origin_y)


def phase_moved(a, b, res: float = RES_M) -> bool:
    """Whether two phases differ, modulo res."""
    return any(min(abs(x - y), res - abs(x - y)) > EDGE_EPS_M for x, y in zip(a, b))


def _touches(poly, cx0: float, cy0: float, res: float) -> bool:
    """Whether the zone marks the cell [cx0, cx0+res] x [cy0, cy0+res]: its
    centre inside, or an edge crossing it (mask()'s rule, one cell at a time)."""
    if _inside_poly(poly, cx0 + res / 2, cy0 + res / 2):
        return True
    lo_x, hi_x = cx0 + EDGE_EPS_M, cx0 + res - EDGE_EPS_M
    lo_y, hi_y = cy0 + EDGE_EPS_M, cy0 + res - EDGE_EPS_M
    n = len(poly)
    for k in range(n):
        (ax, ay), (bx, by) = poly[k], poly[(k + 1) % n]
        t0, t1, dx, dy = 0.0, 1.0, bx - ax, by - ay
        for p, q in ((-dx, ax - lo_x), (dx, hi_x - ax), (-dy, ay - lo_y), (dy, hi_y - ay)):
            if p == 0:
                if q < 0:
                    t0, t1 = 1.0, 0.0
                    break
            else:
                t = q / p
                if p < 0:
                    t0 = max(t0, t)
                else:
                    t1 = min(t1, t)
        if t0 < t1:                      # a positive length inside
            return True
    return False


def boxed(z: dict, x: float, y: float, res: float = RES_M, phase=(0.0, 0.0)) -> bool:
    """Whether the zone alone draws (x, y)'s cell AND its four neighbours on
    the lattice: NavFn, which frees only the robot's own cell, could not leave.
    (suspend() tests the drawn zones together.)"""
    px, py = phase
    i, j = math.floor((x - px) / res), math.floor((y - py) / res)
    return all(_touches(z["poly"], px + (i + di) * res, py + (j + dj) * res, res)
               for di, dj in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)))


def mask(spots: list, res: float = RES_M, zones=(), phase=(0.0, 0.0)) -> dict:
    """One grid that covers every disc and zone polygon (a single unknown
    cell when there are none, so a cleared set still replaces the last
    mask). Row-major from the origin, +x then +y, as nav_msgs/OccupancyGrid.
    A zone marks every cell it overlaps (3.193: before, only cells whose
    centre was inside, which left up to half a cell of its edge passable); a
    spot marks the cells whose centre is inside its disc."""
    zones = list(zones or ())
    if not spots and not zones:
        return {"origin_x_m": 0.0, "origin_y_m": 0.0, "resolution_m": res,
                "width": 1, "height": 1, "data": [UNKNOWN]}
    xs = ([s["x_m"] - s["r_m"] for s in spots] + [s["x_m"] + s["r_m"] for s in spots]
          + [p[0] for z in zones for p in z["poly"]])
    ys = ([s["y_m"] - s["r_m"] for s in spots] + [s["y_m"] + s["r_m"] for s in spots]
          + [p[1] for z in zones for p in z["poly"]])
    x0, x1 = _lattice(min(xs) - MARGIN_M, res, phase[0]), max(xs) + MARGIN_M
    y0, y1 = _lattice(min(ys) - MARGIN_M, res, phase[1]), max(ys) + MARGIN_M
    w, h = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    data = [UNKNOWN] * (w * h)
    for z in zones:
        # 3.193: every cell the zone touches. The cells whose centre is
        # inside, then every cell an edge passes through -- a cell that holds
        # part of the zone with its centre outside holds part of its edge.
        px, py = [p[0] for p in z["poly"]], [p[1] for p in z["poly"]]
        i0, i1 = max(int((min(px) - x0) / res), 0), min(int((max(px) - x0) / res), w - 1)
        j0, j1 = max(int((min(py) - y0) / res), 0), min(int((max(py) - y0) / res), h - 1)
        for j in range(j0, j1 + 1):
            cy = y0 + (j + 0.5) * res
            for i in range(i0, i1 + 1):
                if _inside_poly(z["poly"], x0 + (i + 0.5) * res, cy):
                    data[j * w + i] = 100
        n = len(z["poly"])
        for k in range(n):
            for i, j in _edge_cells(z["poly"][k], z["poly"][(k + 1) % n], x0, y0, res, w, h):
                data[j * w + i] = 100
    for s in spots:
        r2 = s["r_m"] ** 2
        i0, i1 = int((s["x_m"] - s["r_m"] - x0) / res), int((s["x_m"] + s["r_m"] - x0) / res)
        j0, j1 = int((s["y_m"] - s["r_m"] - y0) / res), int((s["y_m"] + s["r_m"] - y0) / res)
        for j in range(max(j0, 0), min(j1 + 1, h)):
            cy = y0 + (j + 0.5) * res
            for i in range(max(i0, 0), min(i1 + 1, w)):
                cx = x0 + (i + 0.5) * res
                if (cx - s["x_m"]) ** 2 + (cy - s["y_m"]) ** 2 <= r2:
                    data[j * w + i] = 100
    return {"origin_x_m": x0, "origin_y_m": y0, "resolution_m": res,
            "width": w, "height": h, "data": data}
