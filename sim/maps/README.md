# sim/maps: the houses the simulator can load

Each house is a `GridWorld` of 0.30 m cells. `robot/factory.py` picks one by
name: the `SIM_MAP` environment variable, else `sim_map:` in
`config/robot.yaml`, else `starter_house`. `sim/maps/__init__.py`'s
`build_world(name)` is the one place a name becomes a world; an unknown name
raises `ValueError` listing every valid one. The demos that score against
ground truth read the same variable, so the robot and its judge are in the
same house.

```bash
SIM_MAP=scaled_house uvicorn robot.server:app --port 8000
```

| `SIM_MAP` | File | What it is, and what it is judged for |
|---|---|---|
| `starter_house` (the default) | `starter_house.py` | 13 x 10 cells, 30 cm (one-cell) doors; living room, hallway, kitchen; a red backpack in the kitchen. Most tests were measured here. Its doors are too narrow for nav2's footprint, so nav2 is not judged here |
| `scaled_house` | `scaled_house.py` | 27 x 14 cells at real proportions, 90 cm doors. **nav2 is judged here**, and the R1 sweeps. The only house with a mover scenario (`SIM_MOVERS=hallway_crossing`, 3.30) |
| `home_first_floor` | `home_first_floor.py` | A realistic furnished test house, **not a replica of anyone's home**: furniture as solid cells, tables as four legs, a kitchen leading to a garage. The furnished-house runs |
| `complex_house` | `complex_house.py` | 3.46's test house, 60 x 44 ft: a loop (living room, kitchen, hall) that gives SLAM loop closure, an L-shaped hall, three dead ends (study, closet, ensuite), an L-shaped bedroom, and 16 one-cell small things on the floor. 3.46's object-inventory sweep |
| `complex_house_hazards` | `complex_house.py` | The same house with heights, and low hazards (cable, sock, shoe, toy) under the lidar's 12 cm plane (3.74). Floor-hazard tests |
| `scaled_house_stairs`, `home_first_floor_stairs` | `__init__.py` (`STAIRS`) | The base house with a stairwell stamped over empty floor (`v` cells), seen only by the depth camera's `drop` block (3.159) |
| `procthor:val:<0-999>`, `procthor:test:<0-999>`, `procthor:train:<0-999>` | `procthor.py`, `procthor_data/` | 3.66: floor plans imported from ProcTHOR-10K (AI2, Apache-2.0; `procthor_data/LICENSE`). Only the floor plan crosses over, rasterised to the same grid; the target is a `red backpack` placed by a seed from the house index. `val` is the sweep set; `test` is kept out of every tuning run (3.65 amendment 3); `train` takes its first 1,000 as fresh test houses (3.88). `usable_<split>.json` pins the houses a sweep should take (570 in `val`, 613 in `test`). No procthor house has movers |

Other variables that act on a house: `SIM_MOVERS` (a mover scenario, off
when unset) and `SIM_START_TURN_DEG` (3.197: turns the start heading; read
in `sim/maps/__init__.py`).

## Where the rest is written

* `docs/engineering/simulator/ENGINEERING.md`: the file table, the `SIM_MAP`
  row and how to run a house.
* `docs/evaluations/PROCTHOR-HOUSES-2026-10-11.md`: why the simulator is
  our own and what the ProcTHOR import showed.
* Plan sections: `docs/plans/ros-alignment/3.46-object-inventory.md`,
  `docs/plans/ros-alignment/3.66-procthor-houses.md`,
  `docs/plans/ros-alignment/3.74-floor-hazards.md`,
  `docs/plans/ros-alignment/3.159-drop-detection.md`.
