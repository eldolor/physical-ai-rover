# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
robot/drop_guard.py -- 3.159: no fall at a stair.

The lidar scans at 12 cm and sees a stairwell as open space; the cone, the
swept corridor and 3.74's floor band look for things. This looks for
missing floor. The depth camera's `drop` block (a field of
`get_depth_grid()`, beside `floor`) says, per column, how far along it the
floor is seen continuously and what ended it: an `edge` (the floor drops
away, or no return where floor should be), a `thing` (a wall or an object:
the other sensors' business) or nothing within reach (`NO_TARGET`).

**Unseen is never clear, and never a distance** (M3, both ways). The
camera is blind under the bumper (MinZ, ~14 cm past it), nothing looks
astern, and a column drops out. So the rule is not judged on one frame but
on a **floor memory**: 5 cm cells in the body's own dead-reckoned frame
(wheel travel and odometry heading -- egocentric, rule 2), marked `floor`
along each column out to where it ends, `edge` at an edge, `thing` at a
thing. A move may go only as far as remembered floor reaches:
* forward and astern, along the swept corridor (`_corridor_room`'s width);
* a turn, only if the chassis' swept outline stays on remembered floor or
  is already off it (a turn that opens the gap is never refused, 3.19).

At start the robot's own disc (`SEED_RADIUS_CM`) is floor: it stands there,
on the dock. Cells are forgotten `FORGET_M` of travel after they were seen
(dead reckoning drifts with distance, not with time: a robot that waits a
minute must not lose the strip under its bumper). A backend with no `drop`
block reads `no_drop_sensor` and the guard is off: every real body today.
"""

from __future__ import annotations

import math
import threading
import weakref
from typing import Optional, Tuple

from robot.interface import ZONE_NO_TARGET, ZONE_RANGE

CELL_CM = 5.0
SEED_RADIUS_CM = 30.0
FORGET_M = 3.0
# How far past the leading edge the corridor is followed. Past the largest
# required clearance (48 cm at 0.6 m/s), so beyond it floor is clear.
RUN_CM = 60.0
STEP_CM = 2.5
LATERAL_STEP_CM = 3.0
# A turn is looked at in steps of this, out to the larger of one step and
# the angle it covers in PIVOT_HORIZON_S (two wheel-loop periods): a fast
# turn must not carry a corner past an edge between two looks.
PIVOT_STEP_DEG = 5.0
PIVOT_HORIZON_S = 0.1
# A turn is refused only when it shrinks the outline's gap to a remembered
# lip and ends under this: a corner swings at most 5.6 cm past the outline
# (the half-diagonal 17.13 cm less the half-width 11.55), and the lip
# samples lie up to 2.4 cm apart (amendment 3).
PIVOT_GAP_CM = 8.0
# 3.159.B2's mark: discs nav2 plans around, one per MARK_STEP_M of lip.
MARK_R_M = 0.30
MARK_STEP_M = 0.15
# A lip nearer the centre than this is not marked: its disc would cover the
# robot's own centre, and nav2 cannot plan from inside a keepout.
MARK_CLEAR_M = MARK_R_M + 0.05
MARK_TTL_S = 3600.0
MAX_MARKS = 64

FLOOR, EDGE, THING = 1, 2, 3
# Memory is pruned of cells seen more than FORGET_M of travel ago once it
# holds this many (10 m^2 of 5 cm cells), and again each time it doubles.
PRUNE_AT = 4000

_GUARDS: "weakref.WeakKeyDictionary" = weakref.WeakKeyDictionary()
_GUARDS_LOCK = threading.Lock()


def guard_for(robot, half_len_cm: float, half_wid_cm: float, side_margin_cm: float
              ) -> "DropGuard":
    """The one floor memory of `robot`, shared by every `SafetyController`
    over it (the wheel loop's, the verbs', the drills'). Two memories dead-
    reckoning apart would disagree about where remembered floor lies, and
    the one called least would place it worst."""
    with _GUARDS_LOCK:
        try:
            guard = _GUARDS.get(robot)
        except TypeError:                     # unhashable or no weakref: its own
            return DropGuard(half_len_cm, half_wid_cm, side_margin_cm)
        if guard is None:
            guard = DropGuard(half_len_cm, half_wid_cm, side_margin_cm)
            try:
                _GUARDS[robot] = guard
            except TypeError:
                pass
        return guard


class DropGuard:
    """The floor memory and the three questions asked of it. One per robot
    (`guard_for`), shared by its `SafetyController`s; the wheel loop and a
    verb read it from two threads, so every read and write holds `_lock`."""

    def __init__(self, half_len_cm: float, half_wid_cm: float, side_margin_cm: float):
        self.half_len = half_len_cm
        self.half_wid = half_wid_cm
        self.side_margin = side_margin_cm
        # Taken off the room before an edge or unseen floor, so that the
        # bar a straight move keeps from a drop is never below the full
        # one: a creep (3.44's 3 cm) may nose up to a sock, not to a stair.
        self._lock = threading.Lock()
        self._cells: dict = {}          # (i, j) -> (state, path_m when seen)
        self._pose = None               # (x_cm, y_cm, theta_rad, path_m)
        self._wheels = None             # last (left_rad, right_rad)
        self._prune_at = PRUNE_AT
        self.lens_x = 0.0               # the block's origin_x_cm, as last read
        self.active = False
        self.probed = False             # a grid has been asked for once
        self.ring_seen = False          # the last frame saw floor at MinZ
        self.folds = 0                  # updates that advanced the pose

    # -- the body's own frame -------------------------------------------------

    def _advance(self, robot) -> Optional[tuple]:
        """Dead-reckon from the wheels since the last call: the distance
        from both wheels' travel, the heading from odometry."""
        try:
            w = robot.get_wheel_state()
            odo = robot.get_odometry()
        except Exception:  # noqa: BLE001 -- no motion to integrate
            return self._pose
        if not (w.get("usable") and odo.get("usable")):
            return self._pose
        l, r = w["left"]["position_rad"], w["right"]["position_rad"]
        theta = math.radians(-float(odo["heading_deg"]))     # cw+ compass -> ccw+
        if self._pose is None:
            self._pose = (0.0, 0.0, theta, 0.0)
            self._wheels = (l, r)
            self._seed()
            return self._pose
        dl, dr = l - self._wheels[0], r - self._wheels[1]
        self._wheels = (l, r)
        ds = (dl + dr) / 2.0 * w["wheel_radius_m"] * 100.0
        x, y, th0, path = self._pose
        mid = th0 + _wrap(theta - th0) / 2.0
        self._pose = (x + ds * math.cos(mid), y + ds * math.sin(mid), theta,
                      path + abs(ds) / 100.0)
        return self._pose

    def _seed(self) -> None:
        x, y, _, path = self._pose
        n = int(SEED_RADIUS_CM // CELL_CM) + 1
        ci, cj = _cell(x, y)
        for i in range(ci - n, ci + n + 1):
            for j in range(cj - n, cj + n + 1):
                if math.hypot((i - ci) * CELL_CM, (j - cj) * CELL_CM) <= SEED_RADIUS_CM:
                    self._cells[(i, j)] = (FLOOR, path)

    def _to_world(self, along: float, lateral: float) -> Tuple[float, float]:
        """Body (along ahead, lateral to the LEFT) -> the memory's frame."""
        x, y, th, _ = self._pose
        c, s = math.cos(th), math.sin(th)
        return x + along * c - lateral * s, y + along * s + lateral * c

    def _state(self, along: float, lateral: float) -> Optional[int]:
        got = self._cells.get(_cell(*self._to_world(along, lateral)))
        return got[0] if got else None

    # -- what the camera saw --------------------------------------------------

    def update(self, robot, grid: Optional[dict]) -> str:
        """Fold this frame's `drop` block into memory. Returns a reason when
        there is no usable block (`no_drop_sensor`), else ''."""
        block = (grid or {}).get("drop") if isinstance(grid, dict) else None
        with self._lock:
            if not isinstance(block, dict) or not block.get("zones"):
                if not self.active:
                    return "no_drop_sensor"
                block = None              # a frame without one: memory only
            self.active = True
            if self._advance(robot) is None:
                return "drop_unusable"
            self.folds += 1
            path = self._pose[3]
            # The body stands on what is under it.
            for along in _steps(-self.half_len, self.half_len, CELL_CM):
                for lat in _steps(-self.half_wid, self.half_wid, CELL_CM):
                    self._mark(along, lat, FLOOR, path)
            if block is not None:
                self._fold(block, path)
            self._forget(path)
            return ""

    def _fold(self, block: dict, path: float) -> None:
        try:
            fov = float(block["fov_deg"])
            cols = len(block["zones"])
            origin = float(block.get("origin_x_cm") or 0.0)
            pan = float(block.get("pan_deg") or 0.0)
            near = float(block["near_cm"])
            far = float(block["far_cm"])
        except (TypeError, ValueError, KeyError):
            return                                   # unreadable: memory only (M3)
        if not (cols and fov > 0 and math.isfinite(origin) and math.isfinite(pan)):
            return
        self.lens_x = origin
        reach = self.half_wid + self.side_margin
        ring_seen = True          # every corridor column sees floor from its MinZ ring
        ends = []                 # (point, kind) per column, None where it said nothing
        for k, zone in enumerate(block["zones"]):
            centre = -fov / 2.0 + fov * (k + 0.5) / cols        # cw+ like the band
            b = math.radians(centre + pan)
            in_corridor = abs(near * math.sin(b)) <= reach
            end, kind = None, None
            status = zone.get("status") if isinstance(zone, dict) else None
            if status == ZONE_NO_TARGET:
                end = far
            elif status == ZONE_RANGE:
                try:
                    end = float(zone["distance_cm"])
                except (TypeError, ValueError, KeyError):
                    end = None
                if end is not None and not math.isfinite(end):
                    end = None
                kind = EDGE if zone.get("kind") == "edge" else THING
            # Unusable, or unreadable: says nothing (M3).
            if in_corridor and (end is None or (kind is not None and end < near + (
                    0.0 if kind == THING else CELL_CM))):
                ring_seen = False
            if end is None:
                ends.append(None)
                continue
            # Floor from the camera's nearest reach to just short of the end.
            d = near
            while d < end - CELL_CM / 2.0:
                self._mark(origin + d * math.cos(b), -d * math.sin(b), FLOOR, path)
                d += STEP_CM
            ends.append(((origin + end * math.cos(b), -end * math.sin(b)), kind)
                        if kind is not None else None)
        self.ring_seen = ring_seen
        if ring_seen:
            # The band from the bumper to the MinZ ring, across the
            # corridor, is blind to the camera. An edge (straight, on this
            # scale) cannot cross it without crossing the ring inside the
            # corridor, which every corridor column just saw as floor -- so
            # the band is floor. Without this a turn in a tight place leaves
            # it unseen for good and the robot never drives on (3.159.B
            # amendment 2). An edge already remembered stays. Half a cell
            # wider all round: the corridor search samples on its own
            # lattice, and a cell it clips at a corner must be here.
            pad = CELL_CM / 2.0
            for along in _steps(self.half_len - pad, origin + near + pad, pad):
                for lat in _steps(-reach - pad, reach + pad, pad):
                    self._mark(along, lat, FLOOR, path)
        # The ends last, so no column's floor paints over another's end.
        prev = None
        for end in ends:
            if end is None:
                prev = None
                continue
            at, kind = end
            self._mark(*at, kind, path, lip=kind == EDGE)
            # A wall is a line, not the dots its columns end on: join two
            # neighbours' ends, so a sightline cannot slip between them.
            if kind == THING and prev is not None and math.dist(prev, at) <= 2 * CELL_CM:
                for f in _steps(0.0, 1.0, 0.25):
                    self._mark(prev[0] + (at[0] - prev[0]) * f, prev[1] + (at[1] - prev[1]) * f,
                               THING, path)
            prev = at if kind == THING else None

    def _mark(self, along, lateral, state, path, lip=False) -> None:
        """`lip`: an edge's own ray end, kept with its cell (where in the
        cell the floor ended) for the turn check's gap and the mark."""
        at = self._to_world(along, lateral)
        key = _cell(*at)
        old = self._cells.get(key)
        # An edge is sticky: nothing but an edge rewrites one (a noisy range
        # past a lip must not paint floor over a stairwell, nor a wall's end
        # or its join a thing that would hide the lip from `clearance`).
        if old is not None and old[0] == EDGE and state != EDGE:
            return
        self._cells[key] = (state, path, at) if lip else (state, path)

    def _forget(self, path: float) -> None:
        if len(self._cells) < self._prune_at:
            return
        self._cells = {k: v for k, v in self._cells.items() if path - v[1] <= FORGET_M}
        self._prune_at = max(PRUNE_AT, 2 * len(self._cells))

    # -- the questions ---------------------------------------------------------

    def clearance(self, direction: int, margin_cm: float = 0.0
                  ) -> Tuple[Optional[float], str]:
        """Room along the swept corridor before floor that is not
        remembered (`drop_unseen`) or an edge (`drop_edge`), less
        `margin_cm` (the caller's: the memory is shared); None when the
        corridor is remembered floor out to `RUN_CM` (or a thing ends it:
        the cone's and the corridor's). On, with no pose to read memory
        from (no usable wheels or odometry yet): 0, `drop_unusable` (M3)."""
        margin = margin_cm
        with self._lock:
            if not self.active:
                return None, "no_drop_sensor"
            if self._pose is None:
                return 0.0, "drop_unusable"
            reach = self.half_wid + self.side_margin
            lats = list(_steps(-reach, reach, LATERAL_STEP_CM))
            ended = [False] * len(lats)
            # Lines from the centre outward, so a thing met on one excuses
            # unseen floor on every line beyond it on that side too: behind
            # a wall is floor the camera cannot see and the body cannot
            # reach (the cone, the corridor and the floor band stop it at
            # the wall). A thing never hides a remembered edge: an edge on
            # any line is the answer, however many things lie before it
            # (a remembered thing may have walked away).
            order = sorted(range(len(lats)), key=lambda n: abs(lats[n]))
            for room in _steps(0.0, RUN_CM, STEP_CM):
                along = direction * (self.half_len + room)
                for n in order:
                    lat = lats[n]
                    st = self._state(along, lat)
                    if st == EDGE:
                        return self._room(room, margin), "drop_edge"
                    if ended[n]:
                        continue
                    if st == THING:
                        for m, other in enumerate(lats):
                            if abs(other) >= abs(lat) and (other * lat > 0 or lat == 0):
                                ended[m] = True
                    elif st is None:
                        if direction > 0 and self._shadowed(along, lat):
                            ended[n] = True          # behind a thing: as the thing
                            continue
                        return self._room(room, margin), ("drop_unseen" if direction > 0
                                                          else "drop_unseen_astern")
            return None, "drop_floor_seen"

    def _shadowed(self, along: float, lat: float) -> bool:
        """Whether the sightline from the lens to this point crosses a
        remembered thing: floor the camera cannot see behind a wall."""
        x0 = self.lens_x
        n = max(1, int(math.hypot(along - x0, lat) / STEP_CM))
        for k in range(1, n):
            f = k / n
            if self._state(x0 + (along - x0) * f, lat * f) == THING:
                return True
        return False

    @staticmethod
    def _room(room: float, margin: float) -> float:
        return round(max(0.0, room - margin), 1)

    def pivot_blocked(self, omega_rad_s: float) -> Optional[str]:
        """A turn that would shrink the outline's gap to a remembered lip
        and end under `PIVOT_GAP_CM` (amendment 3: measured on the lip
        samples, not the 5 cm cells, which refused turns that opened the
        gap, B's D4). A turn that opens the gap, or keeps it at 8 cm or
        more, is never refused. Unseen floor does not refuse a turn
        (amendment 2): a corner swings at most 5.6 cm past the outline, so
        the worst it can do is overhang, never fall -- and the turn sweep
        judges that on truth (D2)."""
        if omega_rad_s == 0:
            return None
        with self._lock:
            if not self.active:
                return None
            if self._pose is None:
                return "drop_unusable"
            reach = max(PIVOT_STEP_DEG, math.degrees(abs(omega_rad_s)) * PIVOT_HORIZON_S)
            looks = int(math.ceil(reach / PIVOT_STEP_DEG - 1e-9))
            lips = self._lips_near(math.hypot(self.half_len, self.half_wid) + PIVOT_GAP_CM)
            if not lips:
                return None
            # Edge by edge (a run of touching edge cells is one lip): the gap
            # to each is its nearest sample's, as the truth's is to its
            # nearest drop. A lip the turn opens, or one already inside the
            # outline, must not excuse another the turn closes; within one
            # lip a corner sliding along it keeps its gap.
            groups = _groups(lips)
            now = [min(self._gap(a, l) for a, l, _ in g) for g in groups]
            for k in range(1, looks + 1):
                # The body turned by `step` is the lips turned by -step.
                step = math.radians(math.copysign(reach * k / looks, omega_rad_s))
                c, s = math.cos(step), math.sin(step)
                for g, g0 in zip(groups, now):
                    after = min(self._gap(a * c + l * s, -a * s + l * c) for a, l, _ in g)
                    if after < PIVOT_GAP_CM and after < g0 - 1e-6:
                        return "drop_pivot_edge"
            return None

    def _gap(self, along: float, lat: float) -> float:
        """A body-frame point's distance from the chassis outline (0 on or
        inside it)."""
        return math.hypot(max(abs(along) - self.half_len, 0.0),
                          max(abs(lat) - self.half_wid, 0.0))

    def _lips_near(self, radius_cm: float) -> list:
        """The remembered lip samples within `radius_cm` of the centre, in
        the body frame (an edge cell with none: its centre)."""
        x, y, th, _ = self._pose
        c, s = math.cos(th), math.sin(th)
        n = int(radius_cm // CELL_CM) + 1
        ci, cj = _cell(x, y)
        out = []
        for i in range(ci - n, ci + n + 1):
            for j in range(cj - n, cj + n + 1):
                got = self._cells.get((i, j))
                if not got or got[0] != EDGE:
                    continue
                px, py = got[2] if len(got) > 2 else ((i + 0.5) * CELL_CM, (j + 0.5) * CELL_CM)
                dx, dy = px - x, py - y
                if math.hypot(dx, dy) <= radius_cm:
                    out.append((dx * c + dy * s, -dx * s + dy * c, (i, j)))
        return out

    def lips(self, radius_cm: float = RUN_CM + 40.0) -> list:
        """The remembered lip samples near the robot, body frame (along
        ahead, lateral left, cm): what the mark is made of."""
        with self._lock:
            if not self.active or self._pose is None:
                return []
            return [(a, l) for a, l, _ in self._lips_near(radius_cm)]


def _groups(lips: list) -> list:
    """Lip samples (along, lateral, cell) grouped by touching edge cells
    (8-neighbours): one group per lip."""
    by_cell = {}
    for lip in lips:
        by_cell.setdefault(lip[2], []).append(lip)
    seen, out = set(), []
    for start in by_cell:
        if start in seen:
            continue
        seen.add(start)
        stack, group = [start], []
        while stack:
            i, j = stack.pop()
            group.extend(by_cell[(i, j)])
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    n = (i + di, j + dj)
                    if n in by_cell and n not in seen:
                        seen.add(n)
                        stack.append(n)
        out.append(group)
    return out


def to_house(x_m: float, y_m: float, heading_deg: float, along_cm: float, lat_cm: float) -> tuple:
    """A body-frame point (along ahead, lateral left, cm) at a house pose
    (x east, y south, compass heading: 0 = -y, 90 = +x) -> house metres."""
    h = math.radians(heading_deg)
    a, l = along_cm / 100.0, lat_cm / 100.0
    return x_m + a * math.sin(h) - l * math.cos(h), y_m - a * math.cos(h) - l * math.sin(h)


class DropMarks:
    """3.159.B2: the drop discs nav2 must plan around, in the house frame.
    One per `MARK_STEP_M` of lip (keyed on that grid), at most `MAX_MARKS`
    (the oldest dropped), living `MARK_TTL_S` and resent with every keepout
    post. A new SLAM session (`session`) empties the book: its discs were
    placed on another map. Not thread safe: the caller holds its lock."""

    def __init__(self, r_m: float = MARK_R_M, step_m: float = MARK_STEP_M,
                 ttl_s: float = MARK_TTL_S, max_marks: int = MAX_MARKS):
        self.r_m, self.step_m, self.ttl_s, self.max_marks = r_m, step_m, ttl_s, max_marks
        self.marks = {}                  # key -> (x_m, y_m, until)
        self.session = None

    def add(self, now: float, points: list, session=None, at=None) -> int:
        """House-frame lip points; returns how many discs are new. `at`: the
        robot's centre (x, y); a disc that would cover it is not made. A
        disc seen again lives on from now."""
        if session != self.session:
            self.marks, self.session = {}, session
        self.marks = {k: v for k, v in self.marks.items() if v[2] > now}
        new = 0
        for x, y in points:
            key = (round(x / self.step_m), round(y / self.step_m))
            cx, cy = key[0] * self.step_m, key[1] * self.step_m
            if at is not None and math.hypot(cx - at[0], cy - at[1]) < MARK_CLEAR_M:
                continue
            if key in self.marks:
                self.marks[key] = (cx, cy, now + self.ttl_s)
                continue
            self.marks[key] = (cx, cy, now + self.ttl_s)
            new += 1
        if len(self.marks) > self.max_marks:
            keep = sorted(self.marks.items(), key=lambda kv: kv[1][2])[-self.max_marks:]
            self.marks = dict(keep)
        return new

    def spots(self, now: float) -> list:
        """The live discs as the bridge takes them (`set_keepout`)."""
        return [{"x_m": round(x, 3), "y_m": round(y, 3), "r_m": self.r_m,
                 "ttl_s": max(round(until - now, 1), 0.1)}
                for x, y, until in self.marks.values() if until > now]


def _outline(half_len, half_wid, step=3.0):
    for a in _steps(-half_len, half_len, step):
        yield a, half_wid
        yield a, -half_wid
    for l in _steps(-half_wid, half_wid, step):
        yield half_len, l
        yield -half_len, l


def _steps(lo, hi, step):
    n = max(1, int(math.ceil((hi - lo) / step)))
    for k in range(n + 1):
        yield lo + (hi - lo) * k / n


def _cell(x, y):
    return (math.floor(x / CELL_CM), math.floor(y / CELL_CM))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
