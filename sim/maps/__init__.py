# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""The houses the simulator can load, by name -- robot/factory.py reads
SIM_MAP and builds one of these; the demos that score against ground truth
read the same variable, so the robot and its judge are in the same house."""


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


def build_world(name: str = "starter_house"):
    """The named house. It carries its own name (`map_name`), so whoever
    holds the world -- robot/server.py's /health -- reports the house that
    was BUILT, not what an environment variable says now."""
    if name.endswith("_stairs") and name[:-len("_stairs")] in STAIRS:
        world = _stamp_stairs(build_world(name[:-len("_stairs")]), name[:-len("_stairs")])
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
        return world
    raise ValueError(f"unknown SIM_MAP {name!r} (starter_house | scaled_house | home_first_floor | complex_house | complex_house_hazards | scaled_house_stairs | home_first_floor_stairs | procthor:val:<0-999> | procthor:test:<0-999> | procthor:train:<0-999>)")


def build_movers(house: str, scenario: str):
    """The movers of one named scenario in one house (PLAN-ros-alignment.md
    3.30) -- a fresh list each call, since a `Mover` carries its own state.
    `SIM_MOVERS` names the scenario; each house lists its own in `MOVERS`."""
    import importlib
    from sim.maps.procthor import parse_split_name
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
