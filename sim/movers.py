# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
movers.py -- people and pets in the simulator (PLAN-ros-alignment.md 3.30).

A `Mover` is a named solid object that hops cell to cell around a closed
path, one hop per `hop_s` of SIM time (`GridWorld.sim_time`). It lives in
`GridWorld.objects` like any other object, so everything that senses or
moves -- `solid_cells`, the scan, collision, `MockWorld`'s map, the camera --
sees it with no code of its own. `GridWorld.advance_time()` is what moves it.

**The keep-out is the one rule worth reading twice.** A mover never hops
to a cell CLOSER to the robot than `MOVER_KEEPOUT_M` beyond the chassis'
turning circle -- or, when the robot has come nearer than that by itself,
closer than the mover already is. It may step past at the same distance, or
away; otherwise it waits and retries at its next hop. That keeps
responsibility clean -- a mover never closes the gap, so the robot can only
come closer than the stop line by its OWN motion, which is the thing 3.18's
ground-truth metric measures. (The first version waited whenever the next
cell was inside the keep-out at all, and a person beside a doorway froze
for good with the robot waiting on them.) It is also a fidelity limit, said
here rather than discovered: a mover never approaches the robot, so this
cannot test a pet darting at it.

Whole 30 cm hops, deterministic: the same path on the same clock is in the
same place, which is what lets a test pin it.
"""

import math
from dataclasses import dataclass

# 3.30: no hop into a cell this close to the chassis' turning circle -- the
# same 20 cm as `min_distance_cm`, so a mover can never appear inside the
# stop line.
MOVER_KEEPOUT_M = 0.20


@dataclass
class Mover:
    """A person or pet: `name` (what the camera's detector would call it),
    a closed `path` of 4-adjacent floor cells it walks round, starting at
    `path[0]`, and the sim seconds per hop."""

    name: str
    path: tuple
    hop_s: float = 1.0
    # 3.31: stand still this long before the first hop (someone in a
    # doorway), and walk the path once and stop at its end (they leave).
    start_s: float = 0.0
    loop: bool = True
    index: int = 0
    next_hop_at: float = 0.0
    hops: int = 0
    waits: int = 0

    def __post_init__(self):
        self.path = tuple(tuple(c) for c in self.path)
        if len(self.path) < 2:
            raise ValueError(f"mover {self.name!r}: a path needs at least two cells")
        if self.hop_s <= 0:
            raise ValueError(f"mover {self.name!r}: hop_s must be positive")
        pairs = self.path[1:] + (self.path[:1] if self.loop else ())
        for i, (a, b) in enumerate(zip(self.path, pairs)):
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
                raise ValueError(
                    f"mover {self.name!r}: path cells {a} and {b} (step {i}) are not "
                    "4-adjacent -- a mover walks, it does not jump, and a looping "
                    "path closes from its last cell back to its first")

    @property
    def cell(self) -> tuple:
        return self.path[self.index]

    @property
    def done(self) -> bool:
        """A one-way walk that has reached its end."""
        return not self.loop and self.index == len(self.path) - 1

    @property
    def next_cell(self) -> tuple:
        return self.path[(self.index + 1) % len(self.path)]


def point_to_cell_cells(x: float, y: float, cell: tuple) -> float:
    """Distance, in cells, from a point to the nearest point of a cell's
    square (0 inside it)."""
    cx, cy = cell
    return math.hypot(max(cx - x, 0.0, x - (cx + 1)), max(cy - y, 0.0, y - (cy + 1)))


# ---------------------------------------------------------------------------
# 3.234.A: movers that APPROACH. Everything above is 3.30's cell-hopping
# mover, unchanged to the bit; everything below is opt-in and separate
# (`GridWorld.disc_movers`, never `GridWorld.movers` or `objects`).
# ---------------------------------------------------------------------------

# What the opt-in mode is called: `SIM_MOVERS=approach:<m/s>` in any house
# (`sim.maps.build_movers`). One disc mover at that speed.
APPROACH_SCENARIO = "approach"
# The three speeds 3.234 A1 names: a slow walk, a walk, a pet's dash.
APPROACH_SPEEDS_M_S = (0.5, 1.2, 2.5)
# A disc in the lidar's plane: a pet's body, a person's two legs as one.
DISC_RADIUS_M = 0.15
# Where a leg starts: this far from the robot's centre, with clear floor
# all the way to where it aims.
LEG_START_M = (1.0, 3.0)
# A crossing leg aims this far ahead of the robot, along its heading: the
# route it is on, rather than where it stands.
CROSS_AHEAD_M = 0.5
CROSS_MAX_COS = math.cos(math.radians(45))
# Standing still between legs, and how finely a leg is integrated.
DWELL_S = 1.0
SUBSTEP_S = 0.02
MAX_SPEED_M_S = 5.0
# A disc never comes closer than this to the chassis: it stops, as a
# person would rather than walk through a robot, and its leg ends there.
# Contact is the robot's doing or nobody's.
CONTACT_M = 0.01
CELL_M = 0.30


def disc_rect_gap_cells(px, py, x, y, theta, half_length, half_width):
    """Distance, in cells, from a point to a rectangle centred at (x, y)
    with its long axis along `theta` (0 inside it). The disc mover's own
    contact rule; the 3.234 instrument has its own copy, by design."""
    ux, uy = math.cos(theta), math.sin(theta)
    dx, dy = px - x, py - y
    a, b = dx * ux + dy * uy, -dx * uy + dy * ux
    return math.hypot(max(abs(a) - half_length, 0.0), max(abs(b) - half_width, 0.0))


class DiscMover:
    """A person or pet as a continuous disc that walks at `speed_m_s` in
    straight legs, each aimed at the robot (`approach`) or across its route
    `CROSS_AHEAD_M` ahead of it (`cross`), in turn, and carrying on past the
    aim point as far again as it started from. A leg starts where it can:
    a seeded free spot `LEG_START_M` from the robot with clear floor to the
    aim -- someone coming round a corner. Between legs it stands
    `DWELL_S`.

    It never comes within `CONTACT_M` of the chassis (it stops and the leg
    ends) and never enters a wall or object cell. Everything else is the
    robot's to avoid: it does not slow for the robot, as 3.30's movers
    did, which is the point of 3.234.

    Deterministic for a `seed` and the robot's own path."""

    def __init__(self, name: str, speed_m_s: float, seed: int = 0,
                 radius_m: float = DISC_RADIUS_M, start_s: float = 0.0):
        import random
        # Bounded so a sub-step (20 ms) never carries the disc further than
        # its own radius: the contact and wall rules are checked at each
        # sub-step's end, and a faster disc could step through the chassis.
        if not 0 < speed_m_s <= MAX_SPEED_M_S:
            raise ValueError(f"disc mover {name!r}: speed must be in (0, {MAX_SPEED_M_S}] m/s")
        self.name, self.speed_m_s, self.radius_m = name, float(speed_m_s), float(radius_m)
        self.rng = random.Random(f"disc-mover-{name}-{seed}")
        self.x = self.y = None          # cells; None until the first leg
        self.leg = None                 # (x0, y0, ux, uy, length_cells, kind)
        self.along = 0.0
        self.rest_until = start_s
        self.legs = {"approach": 0, "cross": 0}
        self.stops = 0                  # legs ended by the contact rule
        self.clock = 0.0

    @property
    def radius_cells(self) -> float:
        return self.radius_m / CELL_M

    @property
    def placed(self) -> bool:
        return self.x is not None

    def disc(self):
        """(x, y, r) in cells, or None before the first leg."""
        return None if self.x is None else (self.x, self.y, self.radius_cells)

    # -- geometry against the house --

    def _clear_at(self, world, x, y) -> bool:
        """The disc at (x, y) touches no wall and no object cell."""
        r = self.radius_cells
        for cy in range(math.floor(y - r), math.floor(y + r) + 1):
            for cx in range(math.floor(x - r), math.floor(x + r) + 1):
                if world._is_passable(cx, cy) and (cx, cy) not in world.objects:
                    continue
                if point_to_cell_cells(x, y, (cx, cy)) < r:
                    return False
        return True

    def _chassis_gap(self, world, x, y) -> float:
        from sim.grid_world import FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH
        return disc_rect_gap_cells(x, y, world.x, world.y, world.theta,
                                   FOOTPRINT_HALF_LENGTH, FOOTPRINT_HALF_WIDTH) - self.radius_cells

    def _line_clear(self, world, x0, y0, x1, y1) -> bool:
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.1))
        return all(self._clear_at(world, x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
                   for k in range(n + 1))

    def _new_leg(self, world) -> bool:
        first = "approach" if (self.legs["approach"] + self.legs["cross"]) % 2 == 0 else "cross"
        # A crossing that does not fit here (a hallway, side on) becomes an
        # approach this time. If neither fits, the disc stands where its last
        # leg ended -- a doorway, perhaps -- and tries again each `DWELL_S`.
        return any(self._try_leg(world, kind)
                   for kind in ((first,) if first == "approach" else ("cross", "approach")))

    def _try_leg(self, world, kind) -> bool:
        ahead = CROSS_AHEAD_M / CELL_M if kind == "cross" else 0.0
        ax, ay = world.x + math.cos(world.theta) * ahead, world.y + math.sin(world.theta) * ahead
        lo, hi = LEG_START_M[0] / CELL_M, LEG_START_M[1] / CELL_M
        contact = CONTACT_M / CELL_M
        for _ in range(40):
            d = lo + (hi - lo) * self.rng.random()
            a = self.rng.random() * 2 * math.pi
            x0, y0 = world.x + math.cos(a) * d, world.y + math.sin(a) * d
            if not self._clear_at(world, x0, y0) or self._chassis_gap(world, x0, y0) < contact:
                continue
            dist = math.hypot(ax - x0, ay - y0)
            if dist < 1e-6:
                continue
            ux, uy = (ax - x0) / dist, (ay - y0) / dist
            # A crossing comes from the side: within 45 degrees of square
            # to the heading, never head-on along the route.
            if kind == "cross" and abs(ux * math.cos(world.theta)
                                       + uy * math.sin(world.theta)) > CROSS_MAX_COS:
                continue
            # Clear floor to the aim point -- or, for an approach, to the
            # chassis, where the contact rule will stop it.
            stop = dist if kind == "cross" else max(0.0, dist - 0.6)
            if not self._line_clear(world, x0, y0, x0 + ux * stop, y0 + uy * stop):
                continue
            self.x, self.y = x0, y0
            self.leg, self.along = (x0, y0, ux, uy, 2.0 * dist, kind), 0.0
            self.legs[kind] += 1
            return True
        return False

    def advance(self, world, dt: float) -> None:
        """Let `dt` seconds pass, in `SUBSTEP_S` steps."""
        end = self.clock + dt
        while self.clock < end - 1e-12:
            h = min(SUBSTEP_S, end - self.clock)
            self.clock += h
            if self.leg is None:
                if self.clock >= self.rest_until and not self._new_leg(world):
                    self.rest_until = self.clock + DWELL_S
                continue
            x0, y0, ux, uy, length, _kind = self.leg
            along = min(length, self.along + self.speed_m_s * h / CELL_M)
            nx, ny = x0 + ux * along, y0 + uy * along
            if (not self._clear_at(world, nx, ny)
                    or self._chassis_gap(world, nx, ny) < CONTACT_M / CELL_M):
                if self._chassis_gap(world, nx, ny) < CONTACT_M / CELL_M:
                    self.stops += 1
                self.leg, self.rest_until = None, self.clock + DWELL_S
                continue
            self.x, self.y, self.along = nx, ny, along
            if along >= length:
                self.leg, self.rest_until = None, self.clock + DWELL_S
