# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
robot/mover_tracker.py

`PLAN-ros-alignment.md` 3.234.B -- a short-horizon tracker of things that
move (people, pets) on the lidar's scan, in the robot server, outside ROS
(rule 3: a sensor that feeds a veto stays out).

**In B nothing in the veto reads it.** It is measured offline
(`tests/mover_track_sweep.py`); 3.234.C may add its predictions as extra
points to the swept-corridor check, behind a key, where a prediction can
only shrink clearance.

    scan (get_scan() dict: 360 nearest-return bins, body frame, clockwise
    positive) + the odometry pose at the scan's capture time
        -> points in the odometry frame
        -> segments split at a range jump
        -> clusters the size of a person or pet, neither edge hidden by
           something nearer (a hidden edge slides as the robot moves)
        -> nearest-neighbour association against each track's prediction
        -> velocity: a least-squares slope over the last `WINDOW` scans
        -> `moving` once a track has `CONFIRM` observations and speed over
           `MOVING_M_S`

Frames: the odometry frame is right-handed, metres and radians (x forward
at the start, y left, yaw counter-clockwise). The body frame is x forward,
y left, the rotation centre at the origin; the lidar sits `lidar_x_m`
ahead of it. Pure Python: about 360 points a scan.
"""

import math
from typing import Callable, List, Optional, Sequence, Union

from robot.lidar_ld19 import LIDAR_YAW_DEG

LIDAR_X_M = 0.04            # robot.safety.LIDAR_X_M (not imported: safety imports nothing of this)
MAX_RANGE_M = 6.0           # beyond this a mover is no hazard inside the horizon
JUMP_M = 0.12               # a range jump this large between adjacent bins splits a segment
JUMP_PER_M = 0.06           # ... plus this per metre of range (bins spread out with range)
MAX_EXTENT_M = 0.8          # wider than a person's shoulders: a wall or furniture
MIN_POINTS = 2
OCCLUDED_M = 0.10           # a neighbour bin this much nearer hides the cluster's edge
GATE_M = 0.5                # association gate around a track's prediction
CONFIRM = 3                 # observations before a track may be `moving`
WINDOW = 4                  # observations in the velocity fit
MOVING_M_S = 0.3            # B1's threshold for a moving track
MAX_SPEED_M_S = 4.0         # faster than this is a mis-association, not a pet
REVOLUTION_S = 0.1          # the D500 at 10 Hz (sim.fake_lidar.RATE_HZ)
COAST_S = 0.25              # a track unmatched this long is dropped
RESET_M = 0.03              # an observation this far off the prediction ...
RESET_FRAC = 0.15            # ... plus this share of the predicted step restarts the fit


class Track:
    __slots__ = ("id", "obs", "x", "y", "vx", "vy", "t", "born", "hits", "seen")

    def __init__(self, tid: int, t: float, x: float, y: float):
        self.id = tid
        self.obs = [(t, x, y)]
        self.x, self.y, self.vx, self.vy, self.t, self.born = x, y, 0.0, 0.0, t, t
        self.hits = 1
        self.seen = 0               # the tracker's scan count when last observed

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def moving(self) -> bool:
        return self.hits >= CONFIRM and MOVING_M_S < self.speed <= MAX_SPEED_M_S

    def predict(self, t: float) -> tuple:
        dt = t - self.t
        return self.x + self.vx * dt, self.y + self.vy * dt

    def add(self, t: float, x: float, y: float) -> None:
        # A mover that stops or turns leaves the straight line the window
        # fits: an observation off the prediction by more than noise plus a
        # share of the step restarts the fit FROM THIS observation. The
        # velocity is then unknown (0: not moving) until the next scan gives
        # a slope, rather than a slope drawn across the stop or the turn.
        qx, qy = self.predict(t)
        step = self.speed * max(0.0, t - self.t)
        self.hits += 1
        if len(self.obs) >= 2 and math.hypot(x - qx, y - qy) > RESET_M + RESET_FRAC * step:
            self.obs = [(t, x, y)]
            self.x, self.y, self.t, self.vx, self.vy = x, y, t, 0.0, 0.0
            return
        self.obs.append((t, x, y))
        del self.obs[:-WINDOW]
        self.x, self.y, self.t = x, y, t
        n = len(self.obs)
        if n < 2:
            return
        mt = sum(o[0] for o in self.obs) / n
        mx = sum(o[1] for o in self.obs) / n
        my = sum(o[2] for o in self.obs) / n
        stt = sum((o[0] - mt) ** 2 for o in self.obs)
        if stt <= 1e-12:
            return
        self.vx = sum((o[0] - mt) * (o[1] - mx) for o in self.obs) / stt
        self.vy = sum((o[0] - mt) * (o[2] - my) for o in self.obs) / stt

    def as_dict(self) -> dict:
        return {"id": self.id, "x": round(self.x, 4), "y": round(self.y, 4),
                "vx": round(self.vx, 4), "vy": round(self.vy, 4),
                "speed": round(self.speed, 4), "moving": self.moving,
                "observations": self.hits, "t": self.t}


def scan_points(ranges: Sequence[Optional[float]], angle_min_deg: float = -180.0,
                increment_deg: float = 1.0, lidar_x_m: float = LIDAR_X_M,
                max_range_m: float = MAX_RANGE_M) -> List[Optional[tuple]]:
    """Per bin, (range, x, y) in the body frame, or None for no return or
    one past `max_range_m`. Bearings are clockwise positive."""
    out = []
    for i, r in enumerate(ranges):
        if r is None or r > max_range_m:
            out.append(None)
            continue
        b = math.radians(angle_min_deg + i * increment_deg)
        out.append((r, lidar_x_m + r * math.cos(b), -r * math.sin(b)))
    return out


def clusters(pts: List[Optional[tuple]], angle_min_deg: float = -180.0,
             increment_deg: float = 1.0, lidar_x_m: float = LIDAR_X_M) -> List[tuple]:
    """(x, y, n, extent, bin) in the body frame for each segment of person
    or pet size whose edges are not hidden by a nearer neighbour; `bin` is
    its middle bin index (fractional).

    (x, y) is the object's estimated CENTRE: along the middle bearing, at
    the nearest return plus half the width. The centroid of the returns
    sits on the near face, and how far in front of the centre depends on
    the range (a near disc shows a narrower arc, densely sampled), so a
    pet that stops in front of the robot kept 'moving' as the bias grew."""
    n = len(pts)
    if n == 0:
        return []
    joined = [False] * n            # joined[i]: bin i continues bin i - 1's segment
    for i in range(n):
        a, b = pts[i - 1], pts[i]
        if a is not None and b is not None:
            gap = math.hypot(a[1] - b[1], a[2] - b[2])
            joined[i] = gap <= JUMP_M + JUMP_PER_M * min(a[0], b[0])
    if all(joined):
        return []                   # one closed ring: a room, no object
    start = next(i for i in range(n) if not joined[i])
    segs, cur = [], []
    for k in range(n):
        i = (start + k) % n
        if not joined[i] and cur:
            segs.append(cur)
            cur = []
        if pts[i] is not None:
            cur.append(i)
    if cur:
        segs.append(cur)
    out = []
    for seg in segs:
        if len(seg) < MIN_POINTS:
            continue
        first, last = pts[seg[0]], pts[seg[-1]]
        extent = math.hypot(first[1] - last[1], first[2] - last[2])
        if extent > MAX_EXTENT_M:
            continue
        before, after = pts[(seg[0] - 1) % n], pts[(seg[-1] + 1) % n]
        near = min(pts[i][0] for i in seg)
        if ((before is not None and before[0] < near - OCCLUDED_M)
                or (after is not None and after[0] < near - OCCLUDED_M)):
            continue
        mid = (seg[0] + ((seg[-1] - seg[0]) % n) / 2.0) % n
        b = math.radians(angle_min_deg + mid * increment_deg)
        r = near + extent / 2.0
        out.append((lidar_x_m + r * math.cos(b), -r * math.sin(b), len(seg), extent, mid))
    return out


class MoverTracker:
    """Feed it one scan at a time with the odometry pose; read `tracks`
    (all) or `movers()` (the moving ones seen in the latest scan).

    **Each cluster is stamped when the beam crossed it**, not at the
    revolution's midpoint: a revolution runs from lidar angle 0 to 360 in
    `REVOLUTION_S`, so a cluster at body bearing b was taken at
    `capture + (((b - yaw) mod 360) / 360 - 0.5) * REVOLUTION_S`. Stamped at
    the midpoint, a pet at 2.5 m/s whose bearing swings across the
    revolution reads up to half a revolution's travel (12 cm) off."""

    def __init__(self, yaw_deg: float = LIDAR_YAW_DEG, revolution_s: float = REVOLUTION_S):
        self.yaw_deg, self.revolution_s = yaw_deg, revolution_s
        self.tracks: List[Track] = []
        self._next_id = 1
        self.scans = 0
        self.t = None

    def update(self, scan: dict, pose: Union[tuple, Callable[[float], tuple]],
               t: float) -> List[Track]:
        """`t` is the scan's capture time (its revolution's midpoint, the
        driver's `capture_unix`); `pose` is (x, y, yaw) in the odometry
        frame, or a function of time giving it (each cluster is placed from
        the pose when the beam crossed it). Returns the moving tracks seen
        in this scan. An unusable scan ages the tracks and adds nothing."""
        self.scans += 1
        self.t = t
        dets = []
        if scan.get("usable"):
            a0 = scan.get("angle_min_deg", -180.0)
            inc = scan.get("angle_increment_deg", 1.0)
            pts = scan_points(scan["ranges_m"], a0, inc)
            for bx, by, _n, _e, mid in clusters(pts, a0, inc):
                frac = ((a0 + mid * inc - self.yaw_deg) % 360.0) / 360.0
                tk = t + (frac - 0.5) * self.revolution_s
                px, py, yaw = pose(tk) if callable(pose) else pose
                c, s = math.cos(yaw), math.sin(yaw)
                dets.append((tk, px + bx * c - by * s, py + bx * s + by * c))
        self._associate(dets, t)
        return self.movers()

    def _associate(self, dets: List[tuple], t: float) -> None:
        pairs = []
        for ti, tr in enumerate(self.tracks):
            for di, (tk, dx, dy) in enumerate(dets):
                qx, qy = tr.predict(tk)
                d = math.hypot(dx - qx, dy - qy)
                if d <= GATE_M:
                    pairs.append((d, ti, di))
        pairs.sort()
        used_t, used_d = set(), set()
        for _d, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            self.tracks[ti].add(*dets[di])
            self.tracks[ti].seen = self.scans
        self.tracks = [tr for tr in self.tracks if t - tr.t <= COAST_S]
        for di, (tk, dx, dy) in enumerate(dets):
            if di not in used_d:
                tr = Track(self._next_id, tk, dx, dy)
                tr.seen = self.scans
                self.tracks.append(tr)
                self._next_id += 1

    def movers(self) -> List[Track]:
        """The moving tracks seen in the latest scan. A track that missed
        it (hidden, or a dropped scan) is kept for `COAST_S` to be found
        again, but is not reported: its velocity is a scan old."""
        return [tr for tr in self.tracks if tr.moving and tr.seen == self.scans]

    def coasting(self) -> List[Track]:
        """Moving tracks that missed the latest scan (inside `COAST_S`)."""
        return [tr for tr in self.tracks if tr.moving and tr.seen != self.scans]
