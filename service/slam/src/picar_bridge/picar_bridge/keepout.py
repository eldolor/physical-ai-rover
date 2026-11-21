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


# 3.195 (D1): the bridge draws each zone grown by one cell. nav2's controller
# tracks the plan on its own and the keepout mask is on the global costmap
# only, so a plan hugging the drawn edge was followed a few mm inside it at a
# corner (3.193's R1). The zone itself, as a person drew it, still decides
# D4's depth and leave rules; the boxed rule reads what is drawn.
GROW_M = 0.05
ARC_STEPS = 8             # segments per corner arc of the margin (a right angle: 11.25 degrees each)


def _hull(pts) -> list:
    """The convex hull, counter-clockwise, duplicates and collinear points dropped."""
    pts = sorted(set((float(p[0]), float(p[1])) for p in pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def grown(poly, g: float = GROW_M) -> list:
    """Every point within g of the polygon, as a polygon: its convex hull
    offset outward by g, each edge moved out along its normal and each corner
    rounded by an arc of radius g drawn as ARC_STEPS segments that lie just
    outside the arc (so the arc is covered; at most g / cos(step / 2) out,
    0.24 mm past g for a right angle). Rounded, not mitred (review): a mitre
    reaches g * sqrt(2) past a corner, so a drawn cell came 0.14 m from a
    zone, past explore's 0.12 m (world/nogo.py HALF_WIDTH_M); rounded, a
    drawn cell reaches at most g plus a cell's diagonal (0.121 m). The hull covers any polygon
    (world/nogo.py only sends rectangles, rotated through the anchor)."""
    hull = _hull(poly)
    if g <= 0 or len(hull) < 3:
        return [tuple(p) for p in poly] if g <= 0 else _box(poly, g)
    n = len(hull)
    normals = []
    for a, b in zip(hull, hull[1:] + hull[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        normals.append((dy / L, -dx / L))        # counter-clockwise: outward is the right
    out = []
    for i in range(n):
        a0 = math.atan2(normals[i - 1][1], normals[i - 1][0])
        turn = (math.atan2(normals[i][1], normals[i][0]) - a0) % (2 * math.pi)
        if turn > math.pi:
            turn = 0.0                           # review: a near-straight corner rounded to -0, not a full circle
        step = turn / ARC_STEPS
        r = g / math.cos(step / 2)
        px, py = hull[i]
        out.append((px + g * math.cos(a0), py + g * math.sin(a0)))
        for k in range(ARC_STEPS):
            ang = a0 + (k + 0.5) * step
            out.append((px + r * math.cos(ang), py + r * math.sin(ang)))
        out.append((px + g * math.cos(a0 + turn), py + g * math.sin(a0 + turn)))
    return out


def _box(poly, g: float) -> list:
    """A degenerate polygon's bounding box grown by g (parse_zones refuses
    one with no width before it gets here)."""
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    return [(min(xs) - g, min(ys) - g), (max(xs) + g, min(ys) - g),
            (max(xs) + g, max(ys) + g), (min(xs) - g, max(ys) + g)]


def with_margin(zones: list, g: float = GROW_M) -> list:
    """3.195: each parsed zone with `drawn`, its polygon grown by g -- what
    mask() draws and suspend()'s boxed rule tests. `poly` stays the zone."""
    return [{**z, "drawn": grown(z["poly"], g)} for z in zones]


def _drawn(z: dict) -> list:
    return z.get("drawn") or z["poly"]


# 3.195 (review, both reviewers): a robot in a zone's MARGIN, outside the zone
# itself, can be boxed by the margin's cells alone (the controller rides
# ~25 mm into the drawn cells). Leaving the whole zone out for it (D4) would
# let nav2 plan through the zone, the fault the margin is there to stop. So
# the margin alone comes off: the zone is drawn bare, as 3.193 drew it, until
# the robot is MARGIN_BACK_M clear of the zone, then grown again.
MARGIN_BACK_M = GROW_M + 2 * RES_M
HYST_M = 0.01             # a margin comes back only when it would box the robot nowhere this near


def bare(zones: list, off) -> list:
    """The zones as drawn: those in `off` without their margin."""
    return [{k: v for k, v in z.items() if k != "drawn"} if z["id"] in off else z for z in zones]


def _five(x: float, y: float, phase) -> list:
    px, py = phase
    i, j = math.floor((x - px) / RES_M), math.floor((y - py) / RES_M)
    return [(px + (i + di) * RES_M, py + (j + dj) * RES_M)
            for di, dj in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))]


def _boxing(view: list, five: list) -> list:
    """Per cell of the five, the zones of `view` (as drawn) that draw it."""
    return [[z for z in view if _touches(_drawn(z), cx, cy, RES_M)] for cx, cy in five]


def margins_off(zones: list, off: set, suspended: set, pose, phase=(0.0, 0.0), fresh=()) -> set:
    """The ids drawn without their margin after this pose ((x, y), map frame;
    None changes nothing). Among the zones D4 keeps drawn this step (not
    suspended, not deeper than SUSPEND_DEPTH_M, not fresh within LEAVE_M:
    review, a zone D4 leaves out anyway never costs a neighbour its margin):
    * a margin that is off comes back once drawing it would box the robot
      (its cell and four neighbours all drawn) at no point within HYST_M of
      the pose, or once the robot is
      MARGIN_BACK_M clear of the zone (review: not only by distance, so a
      robot following the edge gets the margin back at the next corner);
    * when the zones as drawn box the robot, the margins of the zones drawing
      its five cells come off, nearest zone first, until it is not boxed; if
      even the bare zones box it, none come off (suspend() decides).
    suspend() then runs on bare()."""
    ids = {z["id"] for z in zones}
    if pose is None:
        return {i for i in off if i in ids}
    x, y = pose
    near = []
    for z in zones:
        d = zone_distance(z, x, y)
        if (z["id"] in suspended or d < -SUSPEND_DEPTH_M or (z["id"] in fresh and d < LEAVE_M)
                or zone_distance({"poly": _drawn(z)}, x, y) >= 2.5 * RES_M + MARGIN_BACK_M):
            continue
        near.append((z, d))
    keep = {z["id"] for z, d in near if z["id"] in off and d < MARGIN_BACK_M}
    view = [z for z, _ in near]
    five = _five(x, y, phase)
    probes = [_five(x + sx * HYST_M, y + sy * HYST_M, phase) for sx in (-1, 1) for sy in (-1, 1)]
    for zid in sorted(keep):
        # back once it would box the robot nowhere within HYST_M of the pose
        # (review round 4: on a cell boundary, a mm of SLAM jitter flipped it
        # every tick and republished the mask at 5 Hz)
        if not any(all(_boxing(bare(view, keep - {zid}), f)) for f in [five] + probes):
            keep.discard(zid)
    by = _boxing(bare(view, keep), five)
    if all(by):
        # as few margins as unbox the robot, the zone it is nearest (or
        # deepest in) first; none when even the bare zones box it -- D4's
        # suspend() then leaves those out whole, and no neighbour loses its
        # margin for it (review)
        d_of = {z["id"]: d for z, d in near}
        cand = sorted({z["id"] for zs in by for z in zs if "drawn" in z}, key=lambda i: (d_of[i], i))
        trial = set(keep)
        for zid in cand:
            trial.add(zid)
            if not all(_boxing(bare(view, trial), five)):
                for back in reversed(cand[:cand.index(zid)]):   # review round 4: none needlessly
                    if back in trial and not all(_boxing(bare(view, trial - {back}), five)):
                        trial.discard(back)
                return trial
    return keep


def _inside_poly(poly, x: float, y: float) -> bool:
    """Even-odd rule (a zone's quadrilateral, or its grown convex polygon)."""
    inside = False
    n = len(poly)
    for i in range(n):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def _row_spans(poly, y: float) -> list:
    """The x-spans [a, b) where the line at y is inside the polygon, by
    _inside_poly's rule (a point is inside when an odd number of edge
    crossings lie to its right)."""
    xs = []
    n = len(poly)
    for i in range(n):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
        if (ay > y) != (by > y):
            xs.append(ax + (y - ay) * (bx - ax) / (by - ay))
    xs.sort()
    return list(zip(xs[0::2], xs[1::2]))


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


def suspend(zones: list, suspended: set, pose, fresh=(), phase=(0.0, 0.0), boxed: bool = True) -> set:
    """D4: the ids left out of the mask after this pose ((x, y) in the map
    frame, or None when unknown: nothing changes). `fresh`: ids that just
    arrived, which the robot within LEAVE_M of also suspends. `phase`: the
    mask's lattice (mask()'s). 3.195: depth and leave measure from the zone
    (`poly`); boxed tests the cells drawn (`drawn`, the grown polygon).
    3.197: `boxed=False` skips the boxed rule (the bridge's step() vents
    instead); the default keeps 3.193's rule for every other caller."""
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
        elif zone_distance({"poly": _drawn(z)}, x, y) < 2.5 * RES_M:
            near.append(z)                       # may draw the robot's cell or a neighbour
    # Boxed by the zones still drawn, together (review: two zones a gap
    # apart each draw an edge cell; neither alone boxes the robot): every
    # zone that draws one of the five cells is left out.
    drawn = [z for z in near if z["id"] not in out]
    if drawn and boxed:
        px, py = phase
        i, j = math.floor((x - px) / RES_M), math.floor((y - py) / RES_M)
        five = [(px + (i + di) * RES_M, py + (j + dj) * RES_M)
                for di, dj in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))]
        by = [[z["id"] for z in drawn if _touches(_drawn(z), cx, cy, RES_M)] for cx, cy in five]
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
    return all(_touches(_drawn(z), px + (i + di) * res, py + (j + dj) * res, res)
               for di, dj in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)))


def _draw(polys, x0: float, y0: float, w: int, h: int, res: float, data: list) -> None:
    """Mark 100 in `data` (w x h, row-major from (x0, y0)) every cell a
    polygon overlaps, clipped to the grid: mask()'s zone rule, shared with
    the vent's window (3.197)."""
    for poly in polys:
        # 3.193: every cell the zone touches. The cells whose centre is
        # inside, then every cell an edge passes through -- a cell that holds
        # part of the zone with its centre outside holds part of its edge.
        px, py = [p[0] for p in poly], [p[1] for p in poly]
        i0, i1 = max(int(math.floor((min(px) - x0) / res)), 0), min(int(math.floor((max(px) - x0) / res)), w - 1)
        j0, j1 = max(int(math.floor((min(py) - y0) / res)), 0), min(int(math.floor((max(py) - y0) / res)), h - 1)
        for j in range(j0, j1 + 1):
            # 3.195: the cells whose centre is inside, a row at a time (the
            # even-odd crossings of the row's centre line, as _inside_poly
            # counts them), not a point test per cell: a grown zone has ~40
            # corners, and the bridge redraws under its lock.
            for a_, b_ in _row_spans(poly, y0 + (j + 0.5) * res):
                lo = max(int(math.ceil((a_ - x0) / res - 0.5)), i0)
                hi = min(int(math.ceil((b_ - x0) / res - 0.5)) - 1, i1)
                for i in range(lo, hi + 1):
                    data[j * w + i] = 100
        n = len(poly)
        for k in range(n):
            for i, j in _edge_cells(poly[k], poly[(k + 1) % n], x0, y0, res, w, h):
                data[j * w + i] = 100


def mask(spots: list, res: float = RES_M, zones=(), phase=(0.0, 0.0), holes=()) -> dict:
    """One grid that covers every disc and zone polygon (a single unknown
    cell when there are none, so a cleared set still replaces the last
    mask). Row-major from the origin, +x then +y, as nav_msgs/OccupancyGrid.
    A zone marks every cell it overlaps (3.193: before, only cells whose
    centre was inside, which left up to half a cell of its edge passable); a
    spot marks the cells whose centre is inside its disc. `holes` (3.197):
    lattice cells (i, j) -- the cell [phase + i * res, phase + (i + 1) * res)
    each way -- left unknown after the zones are drawn and before the spots
    (the vent: a spot is never vented)."""
    zones = list(zones or ())
    if not spots and not zones:
        return {"origin_x_m": 0.0, "origin_y_m": 0.0, "resolution_m": res,
                "width": 1, "height": 1, "data": [UNKNOWN]}
    polys = [_drawn(z) for z in zones]          # 3.195: the grown polygon, when the bridge gave one
    xs = ([s["x_m"] - s["r_m"] for s in spots] + [s["x_m"] + s["r_m"] for s in spots]
          + [p[0] for poly in polys for p in poly])
    ys = ([s["y_m"] - s["r_m"] for s in spots] + [s["y_m"] + s["r_m"] for s in spots]
          + [p[1] for poly in polys for p in poly])
    x0, x1 = _lattice(min(xs) - MARGIN_M, res, phase[0]), max(xs) + MARGIN_M
    y0, y1 = _lattice(min(ys) - MARGIN_M, res, phase[1]), max(ys) + MARGIN_M
    w, h = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    data = [UNKNOWN] * (w * h)
    _draw(polys, x0, y0, w, h, res, data)
    if holes:
        oi, oj = round((x0 - phase[0]) / res), round((y0 - phase[1]) / res)
        for i, j in holes:
            li, lj = i - oi, j - oj
            if 0 <= li < w and 0 <= lj < h:
                data[lj * w + li] = UNKNOWN
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


# ---------- 3.197: the vent ----------
# D4's boxed rule left a zone out WHOLE (until the robot was LEAVE_M clear)
# whenever the drawn cells boxed the robot, which happens all along a slanted
# edge and at corners (the Rover's zones are rotated by its start heading);
# and a slanted gap between two zones could leave the robot a free
# 4-neighbour that leads nowhere (NavFn's A* is 4-connected), which the
# boxed rule never saw. Instead, the bridge's step() undraws the fewest
# cells that give the robot a way out -- a VENT -- and draws everything else.

VENT_REACH_CELLS = 10     # a 4-connected walk this far (Chebyshev, cells) from the robot's cell is out
VENT_MAX_CELLS = 20       # a vent larger than this: D4's whole-zone rule, as before (amendment 1: 10 left lanes out)
_COST_MARGIN, _COST_EDGE, _COST_BODY = 1, 4, 16   # freeing a cell drawn by a margin / a zone's edge / its body


def _cell_of(x: float, y: float, phase) -> tuple:
    return math.floor((x - phase[0]) / RES_M), math.floor((y - phase[1]) / RES_M)


def _window(view: list, phase, ci: int, cj: int, r: int) -> dict:
    """{(i, j): cost} for each cell within r (Chebyshev) of (ci, cj) that a
    zone of `view` draws; an undrawn cell is absent. The cost of freeing it:
    _COST_MARGIN when only a margin draws it, _COST_EDGE when a zone as the
    person drew it touches it, _COST_BODY when its centre is inside one."""
    n = 2 * r + 1
    x0, y0 = phase[0] + (ci - r) * RES_M, phase[1] + (cj - r) * RES_M
    drawn, body = [UNKNOWN] * (n * n), [UNKNOWN] * (n * n)
    _draw([_drawn(z) for z in view], x0, y0, n, n, RES_M, drawn)
    _draw([z["poly"] for z in view], x0, y0, n, n, RES_M, body)
    out = {}
    for lj in range(n):
        for li in range(n):
            k = lj * n + li
            if drawn[k] != 100 and body[k] != 100:
                continue
            cost = _COST_MARGIN
            if body[k] == 100:
                cx, cy = x0 + (li + 0.5) * RES_M, y0 + (lj + 0.5) * RES_M
                cost = _COST_BODY if any(_inside_poly(z["poly"], cx, cy) for z in view) else _COST_EDGE
            out[(ci - r + li, cj - r + lj)] = cost
    return out


def _escapes(grid: dict, start: tuple, holes=(), reach: int = VENT_REACH_CELLS) -> bool:
    """A 4-connected walk from `start` (free: NavFn frees the robot's own
    cell) through cells not drawn, or vented, to one `reach` cells away."""
    si, sj = start
    seen, todo = {start}, [start]
    while todo:
        i, j = todo.pop()
        if max(abs(i - si), abs(j - sj)) >= reach:
            return True
        for c in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if c not in seen and (c not in grid or c in holes):
                seen.add(c)
                todo.append(c)
    return False


def _cheapest_vent(grid: dict, start: tuple, reach: int = VENT_REACH_CELLS):
    """The drawn cells on the cheapest 4-connected path from `start` to a
    cell `reach` away (each drawn cell costs its grid cost, an undrawn one
    nothing; fewer steps break ties), plus `start` itself when drawn
    (_vent applies VENT_MAX_CELLS). None only when no cell `reach` away is
    in the grid's window (never, for a window of radius `reach` or more)."""
    import heapq
    si, sj = start
    best = {start: (0, 0)}
    prev = {}
    heap = [(0, 0, start)]
    goal = None
    while heap:
        cost, steps, c = heapq.heappop(heap)
        if best.get(c, (math.inf, 0)) < (cost, steps):
            continue
        if max(abs(c[0] - si), abs(c[1] - sj)) >= reach:
            goal = c
            break
        i, j = c
        for nb in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if max(abs(nb[0] - si), abs(nb[1] - sj)) > reach:
                continue
            key = (cost + grid.get(nb, 0), steps + 1)
            if key < best.get(nb, (math.inf, 0)):
                best[nb] = key
                prev[nb] = c
                heapq.heappush(heap, (key[0], key[1], nb))
    if goal is None:
        return None
    cells, c = set(), goal
    while c != start:
        if c in grid:
            cells.add(c)
        c = prev[c]
    if start in grid:
        cells.add(start)
    return frozenset(cells)


def _vent(drawn: list, pose, phase, vent: frozenset):
    """(the vent after this pose, the ids D4 must leave out whole because no
    vent of VENT_MAX_CELLS opens a way). `drawn`: the zones as drawn this
    step (margins and D4's depth and fresh rules already applied)."""
    x, y = pose
    reach_m = (VENT_REACH_CELLS + 2) * RES_M * math.sqrt(2)
    near = [z for z in drawn if zone_distance({"poly": _drawn(z)}, x, y) < reach_m]
    if not near:
        return frozenset(), set()
    start = _cell_of(x, y, phase)
    r = VENT_REACH_CELLS + 1                       # one more: a HYST_M probe may sit in a neighbour cell
    grid = _window(near, phase, start[0], start[1], r)
    if vent and _escapes(grid, start, vent):
        # kept until the robot could leave with no vent at its pose and
        # HYST_M round it (3.195's hysteresis: a mm of jitter on a cell
        # boundary must not open and close it every tick)
        probes = {_cell_of(x + sx * HYST_M, y + sy * HYST_M, phase) for sx in (-1, 1) for sy in (-1, 1)}
        if all(_escapes(grid, c) for c in probes | {start}):
            return frozenset(), set()
        return vent, set()
    if not vent and _escapes(grid, start):
        return frozenset(), set()
    new = _cheapest_vent(grid, start)
    if new is not None and len(new) <= VENT_MAX_CELLS:
        return new, set()
    # no small vent: D4's whole-zone rule, as before, for the zones the
    # cheapest way out runs through (review: not "any of the five cells",
    # which could name a zone that blocks nothing, or none at all)
    cells = new or frozenset()
    return frozenset(), {z["id"] for z in near
                         if any(_touches(_drawn(z), phase[0] + i * RES_M, phase[1] + j * RES_M, RES_M)
                                for i, j in cells)}


def _relattice(cells, old, new) -> frozenset:
    """Cells of lattice `old` as the cells of lattice `new` they overlap (a
    phase move: review, keeping the way out rather than dropping it until the
    next pose)."""
    out = set()
    for i, j in cells:
        x0, y0 = old[0] + i * RES_M + EDGE_EPS_M, old[1] + j * RES_M + EDGE_EPS_M
        x1, y1 = x0 + RES_M - 2 * EDGE_EPS_M, y0 + RES_M - 2 * EDGE_EPS_M
        for a in range(math.floor((x0 - new[0]) / RES_M), math.floor((x1 - new[0]) / RES_M) + 1):
            for b in range(math.floor((y0 - new[1]) / RES_M), math.floor((y1 - new[1]) / RES_M) + 1):
                out.add((a, b))
    return frozenset(out)


def step(zones: list, state: dict, pose, phase=(0.0, 0.0), fresh=()) -> dict:
    """The bridge's whole zone step for this pose ((x, y), map frame, or None
    when unknown): 3.195's margins first, then D4's depth and fresh rules,
    then the vent (3.197) where 3.193's boxed rule left zones out whole.
    `state`: {"suspended", "bare", "vent", "vent_phase"} as the last step
    returned it (a missing key is empty). Returns the same keys: `bare` is
    3.195's zones_bare (never also suspended); `vent` the lattice cells
    mask(holes=) leaves undrawn."""
    susp0, bare0 = set(state.get("suspended") or ()), set(state.get("bare") or ())
    vent0 = frozenset(state.get("vent") or ())
    old = state.get("vent_phase") or (0.0, 0.0)
    if vent0 and phase_moved(old, phase):
        vent0 = _relattice(vent0, old, phase)      # a new lattice: the cells it overlaps
    off = margins_off(zones, bare0, susp0, pose, phase, fresh=fresh)
    view = bare(zones, off)
    after = suspend(view, susp0, pose, fresh, phase=phase, boxed=False)
    vent = vent0
    if pose is not None:
        vent, whole = _vent([z for z in view if z["id"] not in after], pose, phase, vent0)
        after |= whole
    return {"suspended": after, "bare": off - after, "vent": vent, "vent_phase": tuple(phase)}
