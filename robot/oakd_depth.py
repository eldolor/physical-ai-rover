# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
robot/oakd_depth.py -- 3.222: the OAK-D Lite's stereo depth as the floor
band (3.74) and the drop block (3.159) on the real Rover.

The D500 scans at its own height and passes over a sock, a cable or a
stair edge. The simulator has published `get_depth_grid()["floor"]` and
`["drop"]` since 3.74 and 3.159; this driver publishes the same two blocks,
in the same shape, from a real depth frame, so `robot/safety.py` and
`robot/drop_guard.py` read the car exactly as they read the simulator. It
lives in the robot server, outside ROS (rule 3: a sensor that feeds a veto).

**Geometry** (`DepthGeometry`, pure numpy). Each depth pixel is put in the
body frame by the measured mounting (`Mount`: the lens's height above the
floor, its pitch, how far ahead of the chassis centre it sits, any sideways
offset). A point's height says what it is: floor within a tolerance of the
floor plane, a thing above it (up to the robot's own height), a drop below
it. Per column (96 over the camera's 73 degrees, as the simulator's):

* **floor band**: the nearest thing at a horizontal range from the lens in
  [`min_range_cm`, `max_range_cm`], at the bearing it lay at. It takes the
  `min_points`-th nearest point of the column, so one speckle is not a
  sock. A thing nearer than `min_range_cm`, or a column whose floor region
  returned too little (a dropout, or a thing inside the stereo's MinZ,
  where it returns nothing) is UNUSABLE; nothing found over enough seen
  floor is NO_TARGET.
* **drop block**: the column's pixel rows walked outward along the floor,
  from the first row that sees floor. Floor runs on while rows read floor;
  it ends at two rows in a row that do not: a thing (`kind: thing`), or
  points below the floor or no return where floor should be (`kind:
  edge`, 3.159's two past-edge modes). Floor seen to `far_cm` is
  NO_TARGET; a column whose first floor row is unseen is UNUSABLE.

**Freshness.** A frame older than `STALE_S` gives both blocks with every
zone UNUSABLE, never no block: a missing `drop` block switches the drop
guard off (`no_drop_sensor`), an unusable one keeps it on, and its memory
-- floor never seen is never clear -- stops a forward move at the end of
remembered floor (3.222 design, K5).

depthai is imported only by `OakdSource`: the suite never needs it.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np

from robot.interface import ZONE_NO_TARGET, ZONE_RANGE, ZONE_UNUSABLE

logger = logging.getLogger(__name__)

# The simulator's band (sim/mock_robot.py FLOOR_BAND_*, DROP_*): the same
# columns and reach, so the safety layer's numbers mean the same thing.
COLS = 96
FOV_DEG = 73.0              # the OV7251 pair's horizontal field
MIN_RANGE_CM = 20.0         # MinZ at extended disparity, rounded up
MAX_RANGE_CM = 150.0
DROP_FAR_CM = 100.0

STALE_S = 0.5               # K5: unusable within 0.5 s
REOPEN_S = 2.0
STALL_S = 2.0               # nothing from an open device this long: reopen it
BAD_FRAMES = 10             # unusable frames in a row: reopen

_FLOOR, _THING, _BELOW, _NONE = 0, 1, 2, 3


@dataclass(frozen=True)
class Mount:
    """Where the lens is on the body (3.222: measured by the user with a
    tape, 2026-11-26). `pitch_deg` is DOWN-positive; `lateral_cm` is to the
    RIGHT of the centreline (the body's clockwise-positive side)."""
    height_cm: float = 14.0
    pitch_deg: float = 0.0
    x_cm: float = 9.5
    lateral_cm: float = 0.0

    def __post_init__(self):
        # Refused where it is built, so a bad setting fails start-up rather
        # than the reader retrying it for ever (review).
        if not (all(math.isfinite(v) for v in (self.height_cm, self.pitch_deg,
                                               self.x_cm, self.lateral_cm))
                and self.height_cm > 0 and abs(self.pitch_deg) < 80):
            raise ValueError(f"unusable depth mount: {self!r}")


@dataclass(frozen=True)
class Intrinsics:
    """The depth frame's pinhole model, in pixels, at its own resolution."""
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class Tuning:
    floor_tol_cm: float = 1.5       # floor within this of the plane ...
    tol_per_m_cm: float = 1.0       # ... widened by this per metre of range
    drop_cm: float = 4.0            # below the plane by this: a drop
    max_height_cm: float = 40.0     # above this the robot passes under
    min_points: int = 4             # a thing needs this many points
    min_seen: float = 0.5           # share of a column's floor pixels returned
    stride: int = 2                 # every n-th pixel, both ways


def unusable_blocks(mount: Mount, cols: int = COLS, fov_deg: float = FOV_DEG,
                    near_cm: float = MIN_RANGE_CM, max_range_cm: float = MAX_RANGE_CM,
                    far_cm: float = DROP_FAR_CM) -> dict:
    """Both blocks, every zone unusable: what a stale camera publishes."""
    zones = lambda: [{"status": ZONE_UNUSABLE, "distance_cm": None} for _ in range(cols)]  # noqa: E731
    return {
        "floor": {"cols": cols, "fov_deg": fov_deg, "pan_deg": 0.0,
                  "origin_x_cm": mount.x_cm, "min_range_cm": MIN_RANGE_CM,
                  "max_range_cm": max_range_cm, "zones": zones()},
        "drop": {"cols": cols, "fov_deg": fov_deg, "pan_deg": 0.0,
                 "origin_x_cm": mount.x_cm, "near_cm": near_cm, "far_cm": far_cm,
                 "zones": zones()},
    }


class DepthGeometry:
    """Depth frame (millimetres, 0 = no return) -> the two blocks. Built once
    per intrinsics: every pixel's ray, and where it meets the floor, are
    fixed by the mounting."""

    def __init__(self, intr: Intrinsics, mount: Mount = Mount(),
                 tuning: Tuning = Tuning(), cols: int = COLS, fov_deg: float = FOV_DEG):
        self.intr, self.mount, self.tuning = intr, mount, tuning
        self.cols, self.fov_deg = cols, fov_deg
        s = max(1, int(tuning.stride))
        u = np.arange(0, intr.width, s, dtype=np.float32)
        v = np.arange(0, intr.height, s, dtype=np.float32)
        self._s = s
        self._rows = v.astype(np.int32)
        uu, vv = np.meshgrid(u, v)
        self._xn = (uu - intr.cx) / intr.fx               # right, per unit depth
        self._yn = (vv - intr.cy) / intr.fy               # down, per unit depth
        th = math.radians(mount.pitch_deg)
        self._c, self._sn = math.cos(th), math.sin(th)
        # Per unit depth: forward and up in the body frame.
        self._fwd = self._c - self._yn * self._sn
        self._up = -self._sn - self._yn * self._c
        # Where each ray meets the floor (rays at or above the horizon never do).
        down = self._up < -1e-6
        t = np.where(down, mount.height_cm / np.where(down, -self._up, 1.0), np.inf)
        ff = t * self._fwd
        fr = t * self._xn + mount.lateral_cm
        self._floor_range = np.where(down, np.hypot(ff, fr), np.inf)
        self._floor_col = np.where(down, self._col_of(np.degrees(np.arctan2(fr, ff))), -1)
        down = down & (self._floor_col >= 0)        # inside the band's field
        # The floor seen nearest: the drop block's `near_cm` and where the
        # band's floor region starts.
        # The worst column's (review): a column at the field's edge first sees
        # floor further out than the centre's (range f / cos b), and the
        # drop memory marks floor from `near_cm` along every column.
        firsts = np.full(cols, np.inf)
        np.minimum.at(firsts, self._floor_col[down], self._floor_range[down])
        firsts = firsts[np.isfinite(firsts)]
        self.near_cm = float(max(MIN_RANGE_CM, firsts.max())) if firsts.size else MIN_RANGE_CM
        # And the farthest (review): a camera pitched down sees no floor past
        # its top ray, so neither block may claim floor, or clear, past it.
        # The worst column's again: the column that sees least far.
        lasts = np.full(cols, -np.inf)
        np.maximum.at(lasts, self._floor_col[down], self._floor_range[down])
        lasts = lasts[np.isfinite(lasts)]
        reach = float(lasts.min()) if lasts.size else 0.0
        self.max_range_cm = min(MAX_RANGE_CM, reach)
        self.far_cm = min(DROP_FAR_CM, reach)
        region = down & (self._floor_range >= self.near_cm) & (self._floor_range <= self.max_range_cm)
        self._region = region
        self._region_count = np.bincount(self._floor_col[region].ravel(), minlength=cols)[:cols]
        # The drop walk: per column, its floor rows outward (bottom up),
        # within [near, far], as one padded matrix of row indices so a frame
        # is walked in a few array operations (3.222 C: a loop per column
        # cost 23 ms a frame on the Jetson).
        walk = down & (self._floor_range >= self.near_cm - 1e-6) & (self._floor_range <= self.far_cm)
        self._walk = walk
        self._nrows = n = len(self._rows)
        key = self._floor_col * n + np.arange(n)[:, None]
        self._walk_keys = key[walk]
        size = cols * n
        total = np.bincount(self._walk_keys, minlength=size)[:size].reshape(cols, n)
        rng = np.bincount(self._walk_keys, weights=self._floor_range[walk],
                          minlength=size)[:size].reshape(cols, n)
        with np.errstate(invalid="ignore", divide="ignore"):
            walk_range = rng / total
        outward = [np.nonzero(total[k] > 0)[0][::-1] for k in range(cols)]
        width = max(1, max(len(o) for o in outward))
        # Padding points at row `n`, a row that always reads floor, so it
        # never ends a walk: a lone last row does not end the floor (review).
        self._order = np.full((cols, width), n, dtype=np.intp)
        for k, o in enumerate(outward):
            self._order[k, :len(o)] = o
        self._has_rows = np.array([len(o) > 0 for o in outward])
        self._walk_total = total
        self._order_range = np.take_along_axis(
            np.concatenate([walk_range, np.full((cols, 1), np.nan)], axis=1), self._order, axis=1)

    def _col_of(self, bearing_deg):
        col = np.floor((bearing_deg + self.fov_deg / 2.0) / self.fov_deg * self.cols).astype(np.int32)
        return np.where((col >= 0) & (col < self.cols), col, -1)

    def blocks(self, depth_mm: np.ndarray) -> dict:
        """One frame's `floor` and `drop` blocks."""
        tn = self.tuning
        s = self._s
        z = depth_mm[::s, ::s].astype(np.float32) * np.float32(0.1)   # cm along the axis
        if z.shape != self._xn.shape:
            raise ValueError(f"depth frame {depth_mm.shape} is not {self.intr.height}x{self.intr.width}")
        valid = z > 0
        f = z * self._fwd
        h = self.mount.height_cm + z * self._up
        # The tolerance grows with the forward distance, not the range: at
        # most the range, so never looser than designed.
        tol = tn.floor_tol_cm + tn.tol_per_m_cm * 0.01 * f
        floor = valid & (np.abs(h) <= tol)
        thing = valid & (h > tol) & (h <= tn.max_height_cm) & (f > 0)
        below = valid & (h < -tn.drop_cm)
        # Range and bearing only where a thing stands (most pixels are floor).
        tf = f[thing]
        tr = z[thing] * self._xn[thing] + self.mount.lateral_cm
        t_rng = np.hypot(tf, tr)
        t_bear = np.degrees(np.arctan2(tr, tf))
        return {"floor": self._band(valid, thing, t_rng, t_bear),
                "drop": self._drop(valid, floor, thing, below, t_rng)}

    # -- the floor band -------------------------------------------------------

    def _band(self, valid, thing, t_rng, t_bear) -> dict:
        tn, cols = self.tuning, self.cols
        col = self._col_of(t_bear)
        inside = col >= 0
        near_thing = inside & (t_rng < MIN_RANGE_CM)
        in_band = inside & (t_rng >= MIN_RANGE_CM) & (t_rng <= self.max_range_cm)
        too_near = np.bincount(col[near_thing], minlength=cols)[:cols]
        seen = np.bincount(self._floor_col[self._region & valid], minlength=cols)[:cols]
        c_in, r_in, b_in = col[in_band], t_rng[in_band], t_bear[in_band]
        order = np.lexsort((r_in, c_in))
        c_in, r_in, b_in = c_in[order], r_in[order], b_in[order]
        starts = np.searchsorted(c_in, np.arange(cols))
        counts = np.bincount(c_in, minlength=cols)[:cols]
        k = max(1, int(tn.min_points))
        enough = seen >= tn.min_seen * self._region_count
        zones = []
        for i in range(cols):
            if too_near[i] >= k:
                zones.append({"status": ZONE_UNUSABLE, "distance_cm": None})
            elif counts[i] >= k:
                j = starts[i] + k - 1
                zones.append({"status": ZONE_RANGE, "distance_cm": round(float(r_in[j]), 1),
                              "bearing_deg": round(float(b_in[j]), 3) + 0.0})
            elif self._region_count[i] and enough[i]:
                zones.append({"status": ZONE_NO_TARGET, "distance_cm": None})
            else:
                zones.append({"status": ZONE_UNUSABLE, "distance_cm": None})
        return {"cols": cols, "fov_deg": self.fov_deg, "pan_deg": 0.0,
                "origin_x_cm": self.mount.x_cm, "min_range_cm": MIN_RANGE_CM,
                "max_range_cm": round(self.max_range_cm, 1), "zones": zones}

    # -- the drop block -------------------------------------------------------

    def _drop(self, valid, floor, thing, below, t_rng) -> dict:
        cols, n = self.cols, self._nrows
        size = cols * n
        walk, keys = self._walk, self._walk_keys

        def count(mask, weights=None):
            sel = mask[walk]
            return np.bincount(keys[sel], weights=None if weights is None else weights,
                               minlength=size)[:size].reshape(cols, n)

        total = self._walk_total
        n_valid, n_floor = count(valid), count(floor)
        n_thing, n_below = count(thing), count(below)
        # The things' ranges, summed per row (t_rng is over every thing;
        # take the ones on walk rows).
        thing_rng = count(thing, t_rng[walk[thing]])
        # A row's label: no return over half of it -> none; else the most
        # common of floor, thing, below; a returned row that is none of the
        # three (between the tolerance and the drop threshold) is floor.
        stack = np.stack([n_floor, n_thing, n_below])
        label = np.argmax(stack, axis=0).astype(np.int8)
        label[stack.max(axis=0) == 0] = _FLOOR
        label[n_valid * 2 < total] = _NONE
        label = np.concatenate([label, np.full((cols, 1), _FLOOR, np.int8)], axis=1)
        lab = np.take_along_axis(label, self._order, axis=1)          # outward
        nonfloor = lab != _FLOOR
        ends = nonfloor[:, :-1] & nonfloor[:, 1:]                     # two in a row
        found = ends.any(axis=1)
        e = np.argmax(ends, axis=1)
        rows = self._order[np.arange(cols), e]
        kind = lab[np.arange(cols), e]
        zones = []
        for k in range(cols):
            if not self._has_rows[k]:
                zones.append({"status": ZONE_UNUSABLE, "distance_cm": None})
            elif not found[k]:
                zones.append({"status": ZONE_NO_TARGET, "distance_cm": None})
            elif e[k] == 0 and kind[k] == _NONE:
                # The first floor row unseen: it says nothing (M3).
                zones.append({"status": ZONE_UNUSABLE, "distance_cm": None})
            elif kind[k] == _THING:
                d = float(thing_rng[k, rows[k]] / max(1, n_thing[k, rows[k]]))
                zones.append({"status": ZONE_RANGE, "distance_cm": round(d, 1), "kind": "thing"})
            else:
                d = float(self._order_range[k, e[k]])
                zones.append({"status": ZONE_RANGE, "distance_cm": round(d, 1), "kind": "edge"})
        return {"cols": cols, "fov_deg": self.fov_deg, "pan_deg": 0.0,
                "origin_x_cm": self.mount.x_cm, "near_cm": round(self.near_cm, 1),
                "far_cm": round(self.far_cm, 1), "zones": zones}


class DepthCamera:
    """The newest frame's blocks, computed on a reader thread of its own.

    `open_source()` returns an object with `intrinsics()` -> `Intrinsics`,
    `read()` -> `(depth_mm, t_monotonic)` or None for nothing this time, and
    `close()`. Any of them may raise; the reader closes the source and opens
    it again after `reopen_s`. So does a source that delivers nothing for
    `stall_s`, or `BAD_FRAMES` frames in a row it cannot use (review: a hung
    stream on a device that stays open never recovered).

    Freshness is judged on the frame's ARRIVAL on our clock; its age (for
    `_aged()`) on its device timestamp, which can only make it older
    (review: a device clock behind ours read stale for good, one ahead read
    fresh frames that were late)."""

    def __init__(self, open_source: Callable[[], object], mount: Mount = Mount(),
                 tuning: Tuning = Tuning(), stale_s: float = STALE_S,
                 reopen_s: float = REOPEN_S, stall_s: Optional[float] = None,
                 monotonic: Callable[[], float] = time.monotonic, name: str = "depth"):
        self._open_source = open_source
        self.mount, self.tuning = mount, tuning
        self.stale_s, self.reopen_s = stale_s, reopen_s
        self.stall_s = STALL_S if stall_s is None else stall_s
        self._monotonic = monotonic
        self.name = name
        self._lock = threading.Lock()
        self._blocks: Optional[dict] = None
        self._taken_at: Optional[float] = None       # the device's word, never after arrival
        self._arrived_at: Optional[float] = None     # ours, for staleness
        self._reach = {"near_cm": MIN_RANGE_CM, "max_range_cm": MAX_RANGE_CM,
                       "far_cm": DROP_FAR_CM}
        self._frames = 0
        self._opens = 0
        self._failures = 0
        self._last_error: Optional[str] = None
        self._compute_s: Optional[float] = None
        self._running = True
        self._source = None
        self._thread = threading.Thread(target=self._run, name=f"{name}-reader", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        geometry, last_good, bad, phase = None, 0.0, 0, "open"
        while self._running:
            try:
                if self._source is None:
                    phase = "open"
                    self._source = self._open_source()
                    geometry = DepthGeometry(self._source.intrinsics(), self.mount, self.tuning)
                    with self._lock:
                        self._opens += 1
                        self._reach = {"near_cm": geometry.near_cm,
                                       "max_range_cm": geometry.max_range_cm,
                                       "far_cm": geometry.far_cm}
                    last_good, bad, phase = self._monotonic(), 0, "read"
                got = self._source.read()
                if got is None:
                    if self._monotonic() - last_good > self.stall_s:
                        raise RuntimeError(f"no frame for {self.stall_s:g} s")
                    self._sleep(0.01)
                    continue
                depth, taken_at = got
                t0 = time.perf_counter()
                try:
                    blocks = geometry.blocks(depth)
                except Exception as e:  # noqa: BLE001 -- a bad frame says nothing
                    bad += 1
                    if bad >= BAD_FRAMES:
                        raise RuntimeError(f"{bad} frames in a row unusable: {e}") from e
                    self._note_failure(f"frame: {e}")
                    continue
                spent = time.perf_counter() - t0
                now = self._monotonic()
                taken = float(taken_at) if taken_at is not None else now
                if not math.isfinite(taken):
                    taken = now
                with self._lock:
                    recovered, self._last_error = self._last_error is not None, None
                    self._blocks = blocks
                    self._arrived_at, self._taken_at = now, min(taken, now)
                    self._frames += 1
                    self._compute_s = spent
                last_good, bad = now, 0
                if recovered:
                    logger.warning("%s: delivering again", self.name)
            except Exception as e:  # noqa: BLE001 -- a USB device may fail any way
                self._note_failure(f"{phase}: {e}")
                self._close_source()
                self._sleep(self.reopen_s)
        self._close_source()

    def _note_failure(self, why: str) -> None:
        with self._lock:
            self._failures += 1
            repeat = why == self._last_error
            self._last_error = why
        if not repeat:
            logger.warning("%s: %s", self.name, why)

    def _close_source(self) -> None:
        source, self._source = self._source, None
        if source is not None:
            try:
                source.close()
            except Exception:  # noqa: BLE001 -- already failing; closing is best-effort
                pass

    def _sleep(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while self._running and time.monotonic() < end:
            time.sleep(min(0.05, seconds))

    def _fresh(self, now: float) -> bool:
        return self._arrived_at is not None and now - self._arrived_at <= self.stale_s

    def blocks(self) -> dict:
        """The `floor` and `drop` blocks of `get_depth_grid()`: the newest
        frame's, or every zone unusable when it is stale (never omitted)."""
        now = self._monotonic()
        with self._lock:
            if self._fresh(now):
                return self._blocks
            reach = dict(self._reach)
        return unusable_blocks(self.mount, **reach)

    def sensor_age_s(self) -> Optional[float]:
        """The frame's age, while it is fresh; None when stale -- its blocks
        are then unusable and never compared, so their age must not shrink
        the other sensors' clearances (`SafetyController._aged`)."""
        now = self._monotonic()
        with self._lock:
            if not self._fresh(now):
                return None
            # At most `stale_s` old (+ since arrival): a device clock behind
            # ours must not age a frame that came in on time by seconds.
            return min(max(0.0, now - self._taken_at), now - self._arrived_at + self.stale_s)

    def status(self) -> dict:
        """For `/health` -- description, never a verdict (control/health.py)."""
        now = self._monotonic()
        with self._lock:
            age = None if self._taken_at is None else round(now - self._taken_at, 3)
            return {"frames": self._frames, "age_s": age, "fresh": self._fresh(now),
                    "opens": self._opens, "failures": self._failures,
                    "last_error": self._last_error,
                    "compute_ms": (None if self._compute_s is None
                                   else round(self._compute_s * 1000.0, 1)),
                    "mount": {"height_cm": self.mount.height_cm, "pitch_deg": self.mount.pitch_deg,
                              "x_cm": self.mount.x_cm, "lateral_cm": self.mount.lateral_cm},
                    "reader_tid": self._thread.native_id}

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=3.0)
        if self._thread.is_alive():
            # A reader inside a device open keeps it until the open returns;
            # it closes the source itself then.
            logger.warning("%s: reader still busy at close; the device may be held "
                           "until its open returns", self.name)


class OakdSource:
    """The OAK-D Lite through depthai 2.x: the mono pair at `resolution`,
    extended disparity (3.74 B4), left-right check, a 7x7 median, depth
    aligned to the rectified right camera, `fps` frames a second (10: at 15
    the driver cost 29% of a core on the Jetson, past K6, 3.222 C). The
    frame's timestamp is the device's, synced by depthai to the host's
    steady clock (`time.monotonic()` on Linux)."""

    def __init__(self, resolution: str = "THE_480_P", fps: float = 10.0,
                 device_id: Optional[str] = None):
        import depthai as dai

        self._dai = dai
        res = getattr(dai.MonoCameraProperties.SensorResolution, resolution)
        p = dai.Pipeline()
        left, right = p.create(dai.node.MonoCamera), p.create(dai.node.MonoCamera)
        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        for cam in (left, right):
            cam.setResolution(res)
            cam.setFps(fps)
        stereo = p.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setExtendedDisparity(True)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        out = p.create(dai.node.XLinkOut)
        out.setStreamName("depth")
        stereo.depth.link(out.input)
        info = dai.DeviceInfo(device_id) if device_id else None
        self._device = dai.Device(p, info) if info else dai.Device(p)
        try:
            w, h = right.getResolutionWidth(), right.getResolutionHeight()
            calib = self._device.readCalibration()
            m = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, w, h)
            self._intr = Intrinsics(width=w, height=h, fx=m[0][0], fy=m[1][1],
                                    cx=m[0][2], cy=m[1][2])
            self._queue = self._device.getOutputQueue("depth", maxSize=1, blocking=False)
        except Exception:
            self._device.close()
            raise

    def intrinsics(self) -> Intrinsics:
        return self._intr

    def read(self) -> Optional[Tuple[np.ndarray, float]]:
        msg = self._queue.tryGet()
        if msg is None:
            if self._device.isClosed():
                raise RuntimeError("the OAK-D closed")
            return None
        return msg.getFrame(), msg.getTimestamp().total_seconds()

    def close(self) -> None:
        self._device.close()
