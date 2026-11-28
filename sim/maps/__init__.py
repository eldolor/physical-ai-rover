# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""The houses the simulator can load, by name -- robot/factory.py reads
SIM_MAP and builds one of these; the demos that score against ground truth
read the same variable, so the robot and its judge are in the same house."""

import math
import os


# 3.159: a house with a stairwell -- `v` cells (`sim/grid_world.py`
# CELL_DROP) stamped over its floor, as cell ranges (x0, x1, y0, y1),
# inclusive. Only the depth camera's `drop` block sees them.
STAIRS = {
    # 0.9 m square, 60 cm inside the bedroom's door: entering, the robot
    # must turn along its top row.
    "scaled_house": [(6, 8, 10, 12)],
    # 1.2 m along the living room's south wall: the basement's stairs.
    "home_first_floor": [(4, 7, 39, 40)],
}


def _stamp_stairs(world, house: str):
    layout = [list(row) for row in world.layout]
    for x0, x1, y0, y1 in STAIRS[house]:
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                if layout[y][x] != "." or (x, y) in world.objects:
                    raise ValueError(f"{house}: stair cell {(x, y)} is not empty floor")
                layout[y][x] = "v"
    world.layout = ["".join(row) for row in layout]
    return world


# 3.237.A: SIM_LOW_SCATTER=<n>:<seed> lays n low things (3.74's labels and
# heights) on any house's free floor, the same cells for the same seed.
SCATTER_LABELS = ("cable", "sock", "shoe", "toy", "slippers", "towel", "pet bowl", "toy car")
SCATTER_KEEP_M = 1.0     # never within this of the start


def build_world(name: str = "starter_house"):
    """The named house. It carries its own name (`map_name`), so whoever
    holds the world -- robot/server.py's /health -- reports the house that
    was BUILT, not what an environment variable says now. With
    SIM_LOW_SCATTER set, low things scattered on its floor (3.237.A)."""
    world = _build_world(name)
    spec = os.environ.get("SIM_LOW_SCATTER")
    if spec:
        try:
            n, seed = (int(p) for p in spec.split(":"))
            if n < 0:
                raise ValueError
        except ValueError:
            raise ValueError(f"SIM_LOW_SCATTER={spec!r}: <count>:<seed>") from None
        scatter_low(world, n, f"{name}:{seed}")
    return world


def _reachable(world, start, blocked) -> set:
    from collections import deque
    h, w = len(world.layout), len(world.layout[0])
    seen, q = {start}, deque([start])
    while q:
        x, y = q.popleft()
        for n in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if (n not in seen and 0 <= n[0] < w and 0 <= n[1] < h
                    and world.layout[n[1]][n[0]] in ".D" and n not in blocked):
                seen.add(n)
                q.append(n)
    return seen


def scatter_low(world, n: int, seed: str) -> list:
    """Lay `n` low things on free floor cells reachable from the start:
    never within SCATTER_KEEP_M of it, never on an object, and never where
    one would cut off a cell it does not itself cover (a doorway one cell
    wide), so no house is made impossible. A label the house already uses
    for a tall thing is skipped, so nothing standing becomes low. Returns
    the cells, in the order laid."""
    import random
    from sim.maps.complex_house import LOW_HEIGHTS_M
    # A label the house already uses, low or tall, is skipped (review): a
    # tall one would be lowered, and a copy of a low one could stand in for
    # a find's target.
    labels = [lb for lb in SCATTER_LABELS if lb not in set(world.objects.values())]
    if not labels:
        raise ValueError("SIM_LOW_SCATTER: this house already uses every scatter label")
    start = (world.robot_x, world.robot_y)
    objects = dict(world.objects)
    floor = _reachable(world, start, set(objects))
    rng = random.Random(f"3237-scatter-{seed}")
    candidates = sorted(floor)
    rng.shuffle(candidates)
    laid = []
    for cell in candidates:
        if len(laid) == n:
            break
        if math.dist(cell, start) * 0.30 < SCATTER_KEEP_M:
            continue
        # Open floor only: the cell and its eight neighbours plain floor
        # with nothing on them, so no thing narrows a doorway or a one-cell
        # passage the chassis must swing through (review: a cell flood fill
        # alone passes a shoe beside a door).
        if any(not (0 <= cell[1] + j < len(world.layout) and 0 <= cell[0] + i < len(world.layout[0]))
               or world.layout[cell[1] + j][cell[0] + i] != "." or (cell[0] + i, cell[1] + j) in objects
               for i in (-1, 0, 1) for j in (-1, 0, 1)):
            continue
        if _reachable(world, start, set(objects) | {cell}) != floor - {cell}:
            continue
        objects[cell] = labels[len(laid) % len(labels)]
        floor = floor - {cell}
        laid.append(cell)
    if len(laid) < n:
        raise ValueError(f"SIM_LOW_SCATTER: only {len(laid)} of {n} cells qualify")
    heights = dict(world.object_heights or {})
    heights.update({lb: LOW_HEIGHTS_M[lb] for lb in labels})
    world.objects = objects          # replaced, never mutated (3.30)
    world.object_heights = heights
    return laid


def _build_world(name: str):
    if name.endswith("_stairs") and name[:-len("_stairs")] in STAIRS:
        world = _stamp_stairs(_build_world(name[:-len("_stairs")]), name[:-len("_stairs")])
        world.map_name = name
        return world
    if name == "starter_house":
        from sim.maps.starter_house import build_starter_world as build
    elif name == "scaled_house":
        from sim.maps.scaled_house import build_scaled_world as build
    elif name == "home_first_floor":
        from sim.maps.home_first_floor import build_home_world as build
    elif name == "complex_house":
        from sim.maps.complex_house import build_complex_world as build
    elif name == "complex_house_hazards":
        from sim.maps.complex_house import build_complex_hazards_world as build
    else:
        from sim.maps.procthor import build_procthor_world, houses, parse_split_name
        parsed = parse_split_name(name)
        split, index = parsed or (None, None)
        build = None if parsed is None or index >= len(houses(split)) \
            else (lambda: build_procthor_world(index, split))
    if build is not None:
        world = build()
        world.map_name = name
        _start_turn(world)
        return world
    raise ValueError(f"unknown SIM_MAP {name!r} (starter_house | scaled_house | home_first_floor | complex_house | complex_house_hazards | scaled_house_stairs | home_first_floor_stairs | procthor:val:<0-999> | procthor:test:<0-999> | procthor:train:<0-9999>)")


def _start_turn(world) -> None:
    """3.197: SIM_START_TURN_DEG turns the robot's start heading clockwise
    (a compass turn) by that many degrees, so SLAM's map frame -- the robot's
    start -- lies at a slant to the house, as the Rover's start heading will
    put it, and a no-go zone drawn in the house reaches the bridge rotated
    against the costmap's lattice. Unset (the default), nothing changes."""
    turn = os.environ.get("SIM_START_TURN_DEG")
    if turn:
        from sim import renderer          # review: no PIL import for a hook that is off
        try:
            deg = float(turn)
        except ValueError:
            raise ValueError(f"SIM_START_TURN_DEG={turn!r}: a number of degrees") from None
        world.theta = renderer.normalize_angle(world.theta + math.radians(deg))


def build_movers(house: str, scenario: str):
    """The movers of one named scenario in one house (PLAN-ros-alignment.md
    3.30) -- a fresh list each call, since a `Mover` carries its own state.
    `SIM_MOVERS` names the scenario; each house lists its own in `MOVERS`."""
    import importlib
    from sim.maps.procthor import parse_split_name
    from sim.movers import APPROACH_SCENARIO, MAX_SPEED_M_S, DiscMover
    # 3.234.A: `approach:<m/s>[:<seed>]` -- one disc mover that walks at the
    # robot and across its route, in any house (`sim.movers.DiscMover`).
    if scenario.split(":")[0] == APPROACH_SCENARIO:
        parts = scenario.split(":")
        try:
            speed = float(parts[1])
            seed = int(parts[2]) if len(parts) > 2 else 0
            if len(parts) > 3 or not 0 < speed <= MAX_SPEED_M_S:
                raise ValueError
        except (IndexError, ValueError):
            raise ValueError(f"SIM_MOVERS={scenario!r}: {APPROACH_SCENARIO}:<m/s>[:<seed>], "
                             f"a speed in (0, {MAX_SPEED_M_S}]") from None
        return [DiscMover("person", speed, seed=seed)]
    # Every ProcTHOR house is one module, and none has movers.
    # A variant (3.74's `complex_house_hazards`) shares its house's movers.
    base = house[:-len("_hazards")] if house.endswith("_hazards") else house
    base = base[:-len("_stairs")] if base.endswith("_stairs") else base
    module = importlib.import_module("sim.maps." + ("procthor" if parse_split_name(house) is not None else base))
    scenarios = getattr(module, "MOVERS", {})
    if scenario not in scenarios:
        raise ValueError(f"unknown SIM_MOVERS {scenario!r} for {house} "
                         f"({' | '.join(sorted(scenarios)) or 'this house has none'})")
    return scenarios[scenario]()
