# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
retrace.py -- 3.224: reverse along the body's own track
(`docs/plans/ros-alignment/3.224-reverse-retrace.md`).

With the lidar's view astern hidden by the mast (3.218), the only ground
known to be clear behind the body is ground the body itself occupied a
moment ago. `RetraceTrack` keeps that track -- a pose dead-reckoned from the
signed wheel travel and the body's heading -- and answers how far straight
back the body may go while staying on it.

Pure and thread-safe: `robot/safety.py` feeds it from whichever thread asks
for a clearance (the wheel loop, a verb, a request), and it never touches
the robot itself.
"""
import math
import threading
from collections import deque
from typing import Optional

# The user's choice and its bounds (3.224's design; criteria 1-3).
RETRACE_MAX_CM = 50.0        # never offer more than this behind the body
RETRACE_WINDOW_S = 120.0     # older track is forgotten: something may have moved in
RETRACE_LATERAL_CM = 3.0     # off the reverse line by more: not the same ground
RETRACE_HEADING_DEG = 10.0   # turned by more: the swept footprint differs
RETRACE_V_M_S = 0.10         # the ROS path's reverse speed cap while retracing
# A jump in the wheel travel between two samples larger than this is not
# motion (a board reboot, an odometer reset): the track is dropped.
JUMP_M = 0.5
# Samples closer than this along the path add nothing (the wheel loop asks
# 20 times a second); a heading change of a degree still records one.
MIN_STEP_M = 0.005
MAX_SAMPLES = 2000
# Motion seen across a gap longer than this was not watched (review: the
# fallback controller's track across a ROS spell, the brain's sparse asks,
# an idle reboot): its shape is unknown, so the track starts afresh there.
GAP_S = 0.5


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class RetraceTrack:
    """The body's recent poses, newest last: `(t, x_m, y_m, heading_deg)`."""

    def __init__(self):
        self._lock = threading.Lock()
        self._poses: deque = deque(maxlen=MAX_SAMPLES)
        self._s: Optional[float] = None       # the last signed wheel travel, m
        self._t: Optional[float] = None       # when the last sample came, kept or not

    def reset(self) -> None:
        with self._lock:
            self._poses.clear()
            self._s = None
            self._t = None

    def update(self, t: float, travel_m: Optional[float], heading_deg: Optional[float]) -> None:
        """One sample: `travel_m` the signed mean wheel travel since start
        (forward positive), `heading_deg` the body's heading. Either None
        (unusable wheels or odometry) drops the track: an unknown motion
        cannot be retraced."""
        if (travel_m is None or heading_deg is None
                or not math.isfinite(travel_m) or not math.isfinite(heading_deg)):
            self.reset()
            return
        with self._lock:
            last_t, self._t = self._t, t
            fresh = self._s is None or not self._poses or abs(travel_m - self._s) > JUMP_M
            if not fresh:
                ds = travel_m - self._s
                _, x, y, h0 = self._poses[-1]
                turn = _wrap_deg(heading_deg - h0)
                moved = abs(ds) >= MIN_STEP_M or abs(turn) >= 1.0
                if not moved:
                    return
                fresh = last_t is None or t - last_t > GAP_S
            if fresh:
                self._poses.clear()
                self._poses.append((t, 0.0, 0.0, heading_deg))
                self._s = travel_m
                return
            mid = math.radians(h0 + turn / 2.0)
            self._poses.append((t, x + ds * math.cos(mid), y + ds * math.sin(mid), heading_deg))
            self._s = travel_m

    def room_cm(self, now: float) -> float:
        """How far straight back the body may go on its own track, in cm:
        walking back from the newest pose, the farthest distance behind it
        (along its heading) of a contiguous run of poses within
        `RETRACE_LATERAL_CM` of the reverse line and `RETRACE_HEADING_DEG`
        of the heading, none older than `RETRACE_WINDOW_S`; capped at
        `RETRACE_MAX_CM` less how far the body has already backed up from
        the run's most forward pose, so one back-out never covers more
        than `RETRACE_MAX_CM` of track, however long the track. 0.0 with no
        such run."""
        with self._lock:
            if not self._poses:
                return 0.0
            _, px, py, ph = self._poses[-1]
            ux, uy = math.cos(math.radians(ph)), math.sin(math.radians(ph))
            best, ahead = 0.0, 0.0
            for t, x, y, h in reversed(self._poses):
                if now - t > RETRACE_WINDOW_S or abs(_wrap_deg(h - ph)) > RETRACE_HEADING_DEG:
                    break
                dx, dy = px - x, py - y
                along = dx * ux + dy * uy             # positive: that pose is behind now
                if abs(dx * uy - dy * ux) * 100.0 > RETRACE_LATERAL_CM:
                    break
                best = max(best, along * 100.0)
                ahead = max(ahead, -along * 100.0)    # backed up from there already
            return round(max(0.0, min(best, RETRACE_MAX_CM - ahead)), 1)
