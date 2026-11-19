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
MAX_ZONE_M = 50.0         # no side of a house is longer
# D4: a zone the robot is in is left out of the mask until the robot is
# LEAVE_M clear of it. It starts when the robot's centre is more than
# SUSPEND_DEPTH_M inside (NavFn clears the robot's own cell, so one cell in it
# can still plan out, and a path hugging the edge never suspends a zone), or
# when the zone arrives with the robot within LEAVE_M of it.
LEAVE_M = 0.30
SUSPEND_DEPTH_M = 0.05


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


def suspend(zones: list, suspended: set, pose, fresh=()) -> set:
    """D4: the ids left out of the mask after this pose ((x, y) in the map
    frame, or None when unknown: nothing changes). `fresh`: ids that just
    arrived, which the robot within LEAVE_M of also suspends."""
    if pose is None:
        return {i for i in suspended if any(z["id"] == i for z in zones)}
    x, y = pose
    out = set()
    for z in zones:
        d = zone_distance(z, x, y)
        if z["id"] in suspended:
            if d < LEAVE_M:
                out.add(z["id"])                 # not yet clear of it
        elif d < -SUSPEND_DEPTH_M or (z["id"] in fresh and d < LEAVE_M):
            out.add(z["id"])
    return out


def mask(spots: list, res: float = RES_M, zones=()) -> dict:
    """One grid that covers every disc and zone polygon (a single unknown
    cell when there are none, so a cleared set still replaces the last
    mask). Row-major from the origin, +x then +y, as nav_msgs/OccupancyGrid."""
    zones = list(zones or ())
    if not spots and not zones:
        return {"origin_x_m": 0.0, "origin_y_m": 0.0, "resolution_m": res,
                "width": 1, "height": 1, "data": [UNKNOWN]}
    xs = ([s["x_m"] - s["r_m"] for s in spots] + [s["x_m"] + s["r_m"] for s in spots]
          + [p[0] for z in zones for p in z["poly"]])
    ys = ([s["y_m"] - s["r_m"] for s in spots] + [s["y_m"] + s["r_m"] for s in spots]
          + [p[1] for z in zones for p in z["poly"]])
    x0, x1 = min(xs) - MARGIN_M, max(xs) + MARGIN_M
    y0, y1 = min(ys) - MARGIN_M, max(ys) + MARGIN_M
    w, h = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    data = [UNKNOWN] * (w * h)
    for z in zones:
        px, py = [p[0] for p in z["poly"]], [p[1] for p in z["poly"]]
        i0, i1 = max(int((min(px) - x0) / res), 0), min(int((max(px) - x0) / res), w - 1)
        j0, j1 = max(int((min(py) - y0) / res), 0), min(int((max(py) - y0) / res), h - 1)
        for j in range(j0, j1 + 1):
            cy = y0 + (j + 0.5) * res
            for i in range(i0, i1 + 1):
                if _inside_poly(z["poly"], x0 + (i + 0.5) * res, cy):
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
