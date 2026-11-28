# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
brain/hazards.py -- things on the floor under the lidar, in the house frame,
for a while (3.231.C1, 3.237).

Two sources, both off by default:

* **the camera** (3.231.C1, 3.237 arm 3): detections whose label is a hazard
  class, placed by their bearing (the camera's pan included) and range. The
  simulator's range is its own; on the Rover it is the floor-contact range or
  the OAK-D's depth (3.74 C), which 3.231.C2 brings.
* **the depth camera's floor band** (3.237 arm 2): each zone with a range
  whose point the lidar sees PAST -- along the bearing from the lidar to the
  point, the scan's range is longer than the point's by more than the
  tolerance -- so something under the lidar's plane. A lidar return at or
  short of the point (a wall, a tall thing, or something in front) explains
  it. A zone `unusable` or `no_target` marks nothing (M3).

What is found **steers and never vetoes** (3.74, rule 4): explore takes each
hazard as a small no-go zone beside a person's (`ExploreAgent.hazards`), so
its goal choice and reach treat it as a wall, and a frontier behind a pile is
unreachable rather than open. The veto stays `robot/safety.py`'s. Hazards
expire after `ttl_s` (socks move); one seen again is refreshed.
"""

import math
from typing import Iterable, Optional

from robot.safety import LIDAR_X_M      # the lidar ahead of the centre (the xacro's laser_x)

# The sim's low labels (`sim/maps/complex_house.py` HAZARDS_FT and
# `sim/maps/__init__.py` SCATTER_LABELS): 3.74 C's five classes and the
# scatter's three more. The Rover's list is 3.231.B's to decide.
HAZARD_CLASSES = ("cable", "sock", "shoe", "toy", "pet bowl", "slippers", "towel", "toy car")
TTL_S = 300.0              # a hazard not seen again for this long is dropped
HALF_M = 0.15              # a hazard's zone: a square this far each way (one sim cell)
MERGE_M = 0.25             # a point this near a live hazard refreshes it
CAP = 64                   # live hazards at most; the oldest goes first
# A floor-band point is marked only when a second tick's reading lands within
# MERGE_M of it within CONFIRM_S: one reading off a pose, a scan and a grid
# read at different instants can put a door jamb "under" the lidar while the
# robot turns (bug pass). The camera's detections need no second look.
CONFIRM_S = 3.0
# A band point is low only if the lidar's range along its bearing passes it
# by more than this: 5 cm, plus 5 % of the range (stereo depth's error grows
# with range; the review's grazing wall read a point 15 cm from any beam).
EXPLAIN_BASE_M, EXPLAIN_PER_M = 0.05, 0.05
SCAN_HINT_M = 2.0          # explore asks for a scan this far: the band reaches 1.5 m
ID_PREFIX = "hazard-"


def _point(pose: dict, bearing_deg: float, r_m: float, ahead_m: float = 0.0) -> tuple:
    """A point `r_m` out along compass `heading + bearing` from a spot
    `ahead_m` ahead of the robot's centre (the house frame, as
    `ExploreAgent._sighting_now`)."""
    h = math.radians(pose["heading_deg"])
    ox, oy = pose["x_m"] + ahead_m * math.sin(h), pose["y_m"] - ahead_m * math.cos(h)
    a = math.radians(pose["heading_deg"] + bearing_deg)
    return ox + r_m * math.sin(a), oy - r_m * math.cos(a)


def detection_points(frame: dict, pose: dict, pan_deg: float,
                     classes: Iterable[str] = HAZARD_CLASSES) -> list:
    """(label, x, y) for each detection of a hazard class with a bearing and
    a range: bearing off the camera axis plus the pan, range from the centre
    (the simulator's report)."""
    classes = set(classes)
    out = []
    for d in frame.get("detections") or ():
        if (d.get("label") in classes and d.get("bearing_deg") is not None
                and d.get("distance_m") is not None):
            x, y = _point(pose, float(pan_deg) + float(d["bearing_deg"]), float(d["distance_m"]))
            out.append((d["label"], x, y))
    return out


def _scan_range_at(scan: dict, bearing_deg: float) -> float:
    """The shortest of the two beams either side of `bearing_deg` (clockwise,
    zero ahead) and the beam on it: the lidar's range along that bearing,
    erring short -- which only ever explains a point, never marks one."""
    ranges = scan.get("ranges_m") or []
    if not ranges:
        return 0.0
    a0, da = float(scan.get("angle_min_deg", -180.0)), float(scan.get("angle_increment_deg", 1.0))
    k = (bearing_deg - a0) / da
    out = math.inf
    for i in (math.floor(k), math.floor(k) + 1, round(k)):
        r = ranges[int(i) % len(ranges)]
        # No return (None, NaN, 0) is unknown -- a dropout on glass reads so --
        # and unknown explains a point, never marks one (bug pass); a range
        # past the scan's reach is an explicit inf.
        if r is None or (isinstance(r, float) and math.isnan(r)) or float(r) <= 0.0:
            return 0.0
        out = min(out, float(r))
    return out


def band_points(band: Optional[dict], scan: Optional[dict], pose: dict) -> list:
    """(x, y) for each floor-band zone with a range that the lidar sees past:
    under the lidar's plane. No band, or no usable scan to tell low from
    tall, marks nothing."""
    if not isinstance(band, dict) or not isinstance(scan, dict) or not scan.get("usable"):
        return []
    ahead = float(band.get("origin_x_cm") or 0.0) / 100.0
    fov, cols = float(band.get("fov_deg") or 0.0), int(band.get("cols") or 0)
    pan = float(band.get("pan_deg") or 0.0)     # where the band points off the body (bug pass)
    zones = band.get("zones") or []
    if cols and len(zones) != cols:
        return []                         # a block that disagrees with itself marks nothing
    h = math.radians(pose["heading_deg"])
    lx, ly = pose["x_m"] + LIDAR_X_M * math.sin(h), pose["y_m"] - LIDAR_X_M * math.cos(h)
    out = []
    for i, z in enumerate(zones):
        if z.get("status") != "range" or z.get("distance_cm") is None:
            continue                      # unusable or nothing in reach: never a mark (M3)
        b = z.get("bearing_deg")
        if b is None:
            if not cols:
                continue
            b = -fov / 2.0 + fov * (i + 0.5) / cols
        x, y = _point(pose, pan + float(b), float(z["distance_cm"]) / 100.0, ahead)
        # The point from the lidar: its range, and its bearing off the body.
        rho = math.hypot(x - lx, y - ly)
        beta = (math.degrees(math.atan2(x - lx, -(y - ly))) - pose["heading_deg"] + 180.0) % 360.0 - 180.0
        if _scan_range_at(scan, beta) > rho + EXPLAIN_BASE_M + EXPLAIN_PER_M * rho:
            out.append((x, y))
    return out


class HazardList:
    """Live hazards in the house frame: added, refreshed when seen again,
    dropped after `ttl_s`, at most `cap`. `zones(now)` gives them as no-go
    zones (`world/nogo.py`'s shape), with a revision that changes only when
    the set does."""

    def __init__(self, ttl_s: float = TTL_S, half_m: float = HALF_M,
                 merge_m: float = MERGE_M, cap: int = CAP):
        self.ttl_s, self.half_m, self.merge_m, self.cap = ttl_s, half_m, merge_m, cap
        self._items: list = []            # [id, x, y, last_seen, source, label]
        self._pending: list = []          # [x, y, first_seen] awaiting a second look
        self._next = 0
        self.rev = 0
        self.added = {"camera": 0, "floor_band": 0}

    def _expire(self, now: float) -> None:
        keep = [h for h in self._items if now - h[3] <= self.ttl_s]
        if len(keep) != len(self._items):
            self._items = keep
            self.rev += 1

    def add(self, x: float, y: float, now: float, source: str, label: str = "",
            confirm: bool = False) -> bool:
        """Mark (x, y); True if it is a new hazard, False if it refreshed one
        or (`confirm`) is waiting for a second look within CONFIRM_S."""
        self._expire(now)
        for h in self._items:
            if math.hypot(h[1] - x, h[2] - y) <= self.merge_m:
                h[3] = now
                return False
        if confirm:
            self._pending = [p for p in self._pending if now - p[2] <= CONFIRM_S]
            seen = [p for p in self._pending
                    if math.hypot(p[0] - x, p[1] - y) <= self.merge_m and p[2] < now]
            if not seen:
                if not any(math.hypot(p[0] - x, p[1] - y) <= self.merge_m for p in self._pending):
                    self._pending.append([x, y, now])
                return False
            self._pending = [p for p in self._pending if p not in seen]
        self._items.append([f"{ID_PREFIX}{self._next}", x, y, now, source, label])
        self._next += 1
        if len(self._items) > self.cap:
            self._items.sort(key=lambda h: h[3])
            del self._items[0]
        self.added[source] = self.added.get(source, 0) + 1
        self.rev += 1
        return True

    def zones(self, now: float) -> list:
        self._expire(now)
        r = self.half_m
        return [{"id": h[0], "x0": h[1] - r, "x1": h[1] + r, "y0": h[2] - r, "y1": h[2] + r,
                 "source": h[4], "label": h[5]} for h in self._items]

    def __len__(self) -> int:
        return len(self._items)
