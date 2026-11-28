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
MAX_PATH_M = 1.2            # longer along its returns than this: a wall that folds back
MIN_POINTS = 2
OCCLUDED_M = 0.10           # a neighbour bin this much nearer hides the cluster's edge
GATE_M = 0.5                # association gate around a track's prediction
CONFIRM = 3                 # observations before a track may be `moving`
WINDOW = 4                  # observations in the velocity fit
MOVING_M_S = 0.3            # B1's threshold for a moving track
MAX_SPEED_M_S = 4.0         # faster than this is a mis-association, not a pet
REVOLUTION_S = 0.1          # the D500 at 10 Hz (sim.fake_lidar.RATE_HZ)
FREE_M = 0.15               # free space: an earlier beam reached this far past the point
FREE_BACK_S = (0.5, 1.0)    # ... in the scan nearest either of these ages
TURN_MAX_DEG = 3.0          # a revolution in which the body turned more is not tracked
HISTORY_S = max(FREE_BACK_S) + 0.2   # how far back scans are kept for it
CLOCK_STEP_S = 0.5          # a capture time this far behind the last: the clock stepped back
COAST_S = 0.25              # a track unmatched this long is dropped
RESET_M = 0.03              # an observation this far off the prediction ...
RESET_FRAC = 0.15           # ... plus this share of the predicted step restarts the fit


class Track:
    __slots__ = ("id", "obs", "x", "y", "vx", "vy", "t", "born", "hits", "seen", "free")

    def __init__(self, tid: int, t: float, x: float, y: float):
        self.id = tid
        self.obs = [(t, x, y)]
        self.x, self.y, self.vx, self.vy, self.t, self.born = x, y, 0.0, 0.0, t, t
        self.hits = 1
        self.seen = 0               # the tracker's scan count when last observed
        self.free = False           # an observation stood where an earlier beam passed clear

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def moving(self) -> bool:
        # CONFIRM observations in the CURRENT fit, not over the track's
        # life: after a restart a two-point slope over one scan turned 4 cm
        # of centre noise into 0.4 m/s (most of the development starts'
        # false tracks).
        return (self.free and len(self.obs) >= CONFIRM
                and MOVING_M_S < self.speed <= MAX_SPEED_M_S)

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
             increment_deg: float = 1.0, lidar_x_m: float = LIDAR_X_M,
             seam: Optional[int] = None) -> List[tuple]:
    """(x, y, n, extent, bin, near_x, near_y) in the body frame for each
    segment of person or pet size whose edges are not hidden by a nearer
    neighbour; `bin` is its middle bin index (fractional), (near_x, near_y)
    its nearest return.

    (x, y) is the object's estimated CENTRE: along the middle bearing, at
    the nearest return plus half the width. The centroid of the returns
    sits on the near face, and how far in front of the centre depends on
    the range (a near disc shows a narrower arc, densely sampled), so a
    pet that stops in front of the robot kept 'moving' as the bias grew.

    A segment never joins across bin `seam` (the revolution's first bin):
    the bins either side were taken a whole revolution apart. A segment
    longer along its returns than `MAX_PATH_M` is refused even if its ends
    are close: a room's wall broken only at a door is door-wide end to end."""
    n = len(pts)
    if n == 0:
        return []
    joined = [False] * n            # joined[i]: bin i continues bin i - 1's segment
    for i in range(n):
        a, b = pts[i - 1], pts[i]
        if a is not None and b is not None:
            gap = math.hypot(a[1] - b[1], a[2] - b[2])
            joined[i] = gap <= JUMP_M + JUMP_PER_M * min(a[0], b[0])
    if seam is not None:
        joined[seam % n] = False
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
        path = sum(math.hypot(pts[i][1] - pts[j][1], pts[i][2] - pts[j][2])
                   for i, j in zip(seg, seg[1:]))
        if path > MAX_PATH_M:
            continue
        before, after = pts[(seg[0] - 1) % n], pts[(seg[-1] + 1) % n]
        k_near = min(seg, key=lambda i: pts[i][0])
        near = pts[k_near][0]
        if ((before is not None and before[0] < near - OCCLUDED_M)
                or (after is not None and after[0] < near - OCCLUDED_M)):
            continue
        mid = (seg[0] + ((seg[-1] - seg[0]) % n) / 2.0) % n
        b = math.radians(angle_min_deg + mid * increment_deg)
        r = near + extent / 2.0
        out.append((lidar_x_m + r * math.cos(b), -r * math.sin(b), len(seg), extent, mid,
                    pts[k_near][1], pts[k_near][2]))
    return out


class MoverTracker:
    """Feed it one scan at a time with the odometry pose; read `tracks`
    (all) or `movers()` (the moving ones seen in the latest scan).

    **Each cluster is stamped when the beam crossed it**, not at the
    revolution's midpoint: a revolution runs from lidar angle 0 to 360 in
    `REVOLUTION_S`, so a cluster at body bearing b was taken at
    `capture + (((b - yaw) mod 360) / 360 - 0.5) * REVOLUTION_S`. Stamped at
    the midpoint, a pet at 2.5 m/s whose bearing swings across the
    revolution reads up to half a revolution's travel (12 cm) off.

    **A track is moving only once it stood in free space**: one of its
    observations' nearest return lies where the scan about 0.5 s or 1 s
    earlier had its beam pass at least `FREE_M` beyond. A static surface
    seen from a moving robot was there before (the old beam stopped at it),
    and so was anything an occlusion has just revealed (the old beam
    stopped short); a person or pet walks into space the lidar saw empty.
    The centre estimate of a static object wobbles as the view changes;
    without this test that wobble made 49 moving tracks per 10 minutes of
    driving on the development starts."""

    def __init__(self, yaw_deg: float = LIDAR_YAW_DEG, revolution_s: float = REVOLUTION_S):
        self.yaw_deg, self.revolution_s = yaw_deg, revolution_s
        self.tracks: List[Track] = []
        self._next_id = 1
        self.scans = 0
        self.t = None
        self._history: List[tuple] = []     # (t, pose, ranges, angle_min_deg, increment_deg)

    def update(self, scan: dict, pose: Union[tuple, Callable[[float], tuple]],
               t: float, revolution_s: Optional[float] = None) -> List[Track]:
        """`t` is the scan's capture time (its revolution's midpoint, the
        driver's `capture_unix`); `pose` is (x, y, yaw) in the odometry
        frame, or a function of time giving it (each cluster is placed from
        the pose when the beam crossed it). Returns the moving tracks seen
        in this scan. An unusable scan ages the tracks and adds nothing.
        `pose` must answer up to `t + revolution / 2` (the revolution's last
        point): the turn check and the late clusters ask for it, so a caller
        whose odometry ends at `t` must wait for it or extrapolate. A
        revolution read twice (the same `t`) is ignored; a clock that steps
        back more than `CLOCK_STEP_S` restarts the tracker.
        `revolution_s` overrides the nominal period for this scan (the
        driver knows the lidar's real speed; C's wiring passes it)."""
        rev_s = self.revolution_s if revolution_s is None else revolution_s
        if self.t is not None and t <= self.t:
            if t >= self.t - CLOCK_STEP_S:
                return self.movers()        # the same revolution read again: no new data
            self.__init__(self.yaw_deg, self.revolution_s)   # the clock stepped back: start over
        self.scans += 1
        self.t = t
        dets = []
        if scan.get("usable") and scan.get("ranges_m") and not self._turning(pose, t, rev_s):
            a0 = scan.get("angle_min_deg", -180.0)
            inc = scan.get("angle_increment_deg", 1.0)
            pts = scan_points(scan["ranges_m"], a0, inc)
            olds = self._olds(t)
            seam = int(round(((self.yaw_deg - a0) % 360.0) / inc)) % len(pts)
            for bx, by, _n, _e, mid, nx, ny in clusters(pts, a0, inc, seam=seam):
                frac = ((a0 + mid * inc - self.yaw_deg) % 360.0) / 360.0
                tk = t + (frac - 0.5) * rev_s
                px, py, yaw = pose(tk) if callable(pose) else pose
                c, s = math.cos(yaw), math.sin(yaw)
                free = any(self._was_free(old, px + nx * c - ny * s, py + nx * s + ny * c)
                           for old in olds)
                dets.append((tk, px + bx * c - by * s, py + bx * s + by * c, free))
            p0 = pose(t) if callable(pose) else pose
            self._history.append((t, p0, scan["ranges_m"], a0, inc))
            self._history = [h for h in self._history if t - h[0] <= HISTORY_S]
        self._associate(dets, t)
        return self.movers()

    def _turning(self, pose, t: float, revolution_s: float) -> bool:
        """The body turned more than `TURN_MAX_DEG` across this revolution.
        The driver bins each point by the lidar's angle alone, so a turning
        body's scan is smeared (40 degrees a revolution at the simulator's
        pivot rate): such a scan neither feeds a track nor becomes history
        for the free-space test, and the tracks coast across it."""
        if not callable(pose):
            return False
        h = revolution_s / 2.0
        d = pose(t + h)[2] - pose(t - h)[2]
        return abs(math.degrees((d + math.pi) % (2 * math.pi) - math.pi)) > TURN_MAX_DEG

    def _olds(self, t: float) -> List[tuple]:
        """The kept scans nearest each age in `FREE_BACK_S` (within 0.15 s:
        a dropped scan or two), none while the history is short."""
        out = []
        for back in FREE_BACK_S:
            best = min(self._history, key=lambda h: abs(t - back - h[0]), default=None)
            if best is not None and abs(t - back - best[0]) <= 0.15 and best not in out:
                out.append(best)
        return out

    def _was_free(self, old: tuple, x: float, y: float) -> bool:
        """The odometry point (x, y) lay at least `FREE_M` short of where
        `old`'s beams through it (its bin and both neighbours) stopped. A
        bin with no return is NOT clear: on a real lidar it may be a dark
        or shiny surface, and once a track is free it stays free."""
        _t, (px, py, yaw), ranges, a0, inc = old
        lx, ly = px + LIDAR_X_M * math.cos(yaw), py + LIDAR_X_M * math.sin(yaw)
        d = math.hypot(x - lx, y - ly)
        bearing = math.degrees(yaw - math.atan2(y - ly, x - lx))     # clockwise positive
        i = int(round(((bearing - a0 + 180.0) % 360.0 - 180.0) / inc)) % len(ranges)
        for j in (i - 1, i, i + 1):
            r = ranges[j % len(ranges)]
            if r is None or r < d + FREE_M:
                return False
        return True

    def _associate(self, dets: List[tuple], t: float) -> None:
        pairs = []
        for ti, tr in enumerate(self.tracks):
            for di, (tk, dx, dy, _f) in enumerate(dets):
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
            tk, dx, dy, free = dets[di]
            self.tracks[ti].add(tk, dx, dy)
            self.tracks[ti].seen = self.scans
            self.tracks[ti].free |= free
        self.tracks = [tr for tr in self.tracks if t - tr.t <= COAST_S]
        for di, (tk, dx, dy, free) in enumerate(dets):
            if di not in used_d:
                tr = Track(self._next_id, tk, dx, dy)
                tr.seen = self.scans
                tr.free = free
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
