# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.237 (`docs/plans/ros-alignment/3.237-explore-three-sensors.md`): the
instrument for explore with lidar, depth and the pan-tilt camera, and B0's
recorded baseline. No mission runs here; the harness's parts are checked
on their own, and the recorded sweep against the plan."""

import functools
import importlib.util
import json
import math
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SWEEP = ROOT / "evaluations/explore3-3237"


@functools.lru_cache(maxsize=None)
def _sweep():
    """The sweep module under a name of its own (many evaluations have a
    `sweep.py`), with sys.path untouched."""
    spec = importlib.util.spec_from_file_location("explore3_3237_sweep", SWEEP / "sweep.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_scatter_is_seeded_and_closes_no_doorway():
    """A4's rule, held by the harness until A: the same cells for the same
    house; every free cell reachable before is reachable after, less the
    placed ones; none within 1 m of the house's start."""
    sw = _sweep()
    from sim.maps import build_world
    house = "procthor:val:0"
    plain = build_world(house)
    start = (plain.robot_x, plain.robot_y)
    before = sw.reachable(plain, start, set(plain.objects))
    a, b = sw.house_world(house), sw.house_world(house)
    assert a.objects == b.objects
    placed = set(a.objects) - set(plain.objects)
    assert len(placed) == sw.SCATTER
    assert sw.reachable(a, start, set(a.objects)) == before - placed
    assert all(math.dist(c, start) * 0.3 >= 1.0 for c in placed)
    # Every placed thing is low to the judge, and to the sim's scan rule.
    assert placed <= set(sw.low_of(a)) and placed <= set(a.low_cells)


def test_starts_keep_a_metre_from_low_things_and_the_target():
    sw = _sweep()
    for house in ("complex_house_hazards", "procthor:val:2"):
        world = sw.house_world(house)
        keep = sw.low_of(world) + [c for c, n in world.objects.items() if n == sw.TARGET]
        starts = sw.starts(house)
        assert len(set(starts)) == sw.STARTS
        for x, y, _ in starts:
            assert min(math.dist((x, y), (c[0] + 0.5, c[1] + 0.5)) for c in keep) * 0.3 >= 1.0


def test_the_planner_is_blind_to_low_things_only():
    """LidarNav plans through a low thing and around a tall one, as nav2 on
    a lidar costmap; WalkingNav (FakeNav's planner) avoids both, which is
    why it cannot judge this section (refinement 5)."""
    sw = _sweep()
    from sim.mock_robot import MockRobot
    from tests.test_explore import WalkingNav
    world = sw.house_world("complex_house_hazards")
    robot = MockRobot(world, render=False)
    nav, walking = sw.lidar_nav(robot), WalkingNav(robot)
    low = sorted(world.low_cells)
    tall = sorted(set(world.objects) - set(world.low_cells))
    on = lambda path, cells: any(c in cells for c in path or [])  # noqa: E731
    # A path between the free neighbours either side of a low cell.
    for c in low:
        ends = [(c[0] - 1, c[1]), (c[0] + 1, c[1])]
        if all(sw._free(world, e) and e not in world.objects for e in ends):
            assert nav._path(*ends) == [ends[0], c, ends[1]]
            assert not on(walking._path(*ends), {c})
            break
    else:
        raise AssertionError("no low thing between two free cells")
    for c in tall:
        ends = [(c[0] - 1, c[1]), (c[0] + 1, c[1])]
        if all(sw._free(world, e) and e not in world.objects for e in ends):
            assert not on(nav._path(*ends), {c})
            break


def test_the_judge_counts_a_touch_once_per_approach():
    """Ground truth: the chassis within CONTACT_CM of a low square is one
    event; it counts again only after leaving by LEAVE_CM."""
    sw = _sweep()
    from tests.footprint_sweep import CELL_CM, HALF_LENGTH
    world = sw.house_world("complex_house_hazards")
    cell = sw.low_of(world)[0]
    judge = sw.Judge(world, {cell}, set())
    world.theta = 0.0                                   # facing +x, toward the cell
    y = cell[1] + 0.5
    for gap_cm, events in ((10.0, 0), (0.5, 1), (2.0, 1), (0.2, 1), (5.0, 1), (0.0, 2)):
        world.x, world.y = cell[0] - HALF_LENGTH - gap_cm / CELL_CM, y
        judge.check()
        assert judge.low_events == events, gap_cm


def test_an_unknown_arm_is_refused():
    sw = _sweep()
    try:
        sw.main("l")
    except SystemExit as e:
        assert "unknown arm" in str(e)
    else:
        raise AssertionError("arm 'l' ran")


def test_the_recorded_baseline_is_b0s():
    """B0 as recorded in results/ (the plan's B0 table): the gate passed
    (arm 1's median >= 0.5 low contacts a survey) and the veto alone short
    of B1's 80 % fall."""
    load = lambda a: json.loads((SWEEP / "results" / f"{a}.json").read_text())  # noqa: E731
    lidar, veto = load("L"), load("V")
    for arm in (lidar, veto):
        assert arm["missions"] == 110 and arm["left_out_error_or_capped"] == 0
        assert arm["other_contacts"] == 0
    assert lidar["low_contacts_survey_median"] >= 0.5                    # the gate
    assert (lidar["low_contacts"], veto["low_contacts"]) == (319, 148)
    assert 0.2 < veto["low_contacts"] / lidar["low_contacts"] < 0.5      # -54 %: short of B1
    assert (lidar["refusals_floor_band"], veto["refusals_floor_band"] > 0) == (0, True)
    assert (lidar["find_rate"], veto["find_rate"]) == (0.4545, 0.4)
    assert (lidar["coverage_median"], veto["coverage_median"]) == (0.5953, 0.5621)


# ---------- 3.237.A ----------

def hazard_fingerprint(steps: int = 300, seed: int = 3237) -> str:
    """A1 on the hazard house: the scan, the level row, the floor block and
    the map over a seeded drive (detections left out: A gates them)."""
    import hashlib
    import random
    from sim.fake_lidar import world_caster
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld
    robot = MockRobot(build_world("complex_house_hazards"), render=False)
    world = MockWorld(robot.world)
    lidar = world_caster(robot.world)
    rng = random.Random(seed)
    digest = hashlib.sha256()
    verbs = (lambda: robot.drive_forward(duration=0.5), lambda: robot.turn_left(30),
             lambda: robot.turn_right(45), lambda: robot.look_left(), lambda: robot.look_center())
    for i in range(steps):
        rng.choice(verbs)()
        reading = {"grid": robot.get_depth_grid(), "distance": robot.get_distance(),
                   "near": robot.get_scan(max_range_m=0.6)["ranges_m"]}
        if i % 50 == 0:
            reading["scan"] = robot.get_scan()["ranges_m"]
            reading["lidar"] = [lidar(a, 0.0) for a in range(0, 360, 2)]
            world.observe()
            reading["map"] = world.get_map()
        digest.update(json.dumps(reading, sort_keys=True).encode())
    return digest.hexdigest()


HAZARD_PINNED = "d4be857d0a0f68edcc485b7d8996b1d1be1b12cb2c0f9069b3923f938dd51967"   # dev, before A
SCATTERED = [f"procthor:val:{i}" for i in (0, 2, 3, 4, 5)]
# A2's recorded run is 500 poses a house (3,000 in all, the plan's table);
# the fast tier runs 100 a house (~15 s). PICAR_A2_POSES=500 repeats the
# recorded run. The bar (100 % agreement) is the same at either count.
POSES = int(os.environ.get("PICAR_A2_POSES", "100"))
LENS_M, MOUNT_DOWN_DEG = 0.210, 15.0                       # the xacro, read here
PAN_X_M, LENS_X_M = -0.009, 0.048                          # pan axis, lens ahead of it


def _lens_forward_m(x, y, theta, view, c):
    """The test's own pinhole geometry: the floor point's distance from the
    lens along the camera's horizontal axis, in metres."""
    lx = x + (PAN_X_M * math.cos(theta) + LENS_X_M * math.cos(view)) / 0.30
    ly = y + (PAN_X_M * math.sin(theta) + LENS_X_M * math.sin(view)) / 0.30
    return ((c[0] - lx) * math.cos(view) + (c[1] - ly) * math.sin(view)) * 0.30
VFOV_DEG = 2 * math.degrees(math.atan(math.tan(math.radians(30.0)) * 0.75))


def test_a1_the_hazard_house_senses_what_it_did(monkeypatch):
    """A1: camera level, no scatter, band on: the scan, level row, floor
    block and map equal `dev`'s before any A code (the four plain houses are
    `tests/test_sim_fingerprint.py`'s, unchanged)."""
    for var in ("SIM_LOW_SCATTER", "SIM_FLOOR_BAND"):
        monkeypatch.delenv(var, raising=False)
    assert hazard_fingerprint() == HAZARD_PINNED


def _poses(world, n, seed):
    import random
    rng = random.Random(seed)
    free = [(x, y) for y, row in enumerate(world.layout) for x, c in enumerate(row)
            if c in ".D" and (x, y) not in world.objects]
    out = []
    while len(out) < n:
        cx, cy = rng.choice(free)
        x, y, th = cx + rng.random(), cy + rng.random(), rng.uniform(-math.pi, math.pi)
        world.theta = th
        if world.footprint_overlaps(x, y):
            continue
        out.append((x, y, th, rng.choice((-30.0, -15.0, 0.0, 15.0, 30.0)), rng.choice((-1, 0, 1))))
    return out


def _march(world, solid, x, y, angle, max_cells):
    """The test's own ray cast: an exact cell-by-cell traversal (Amanatides
    and Woo) to the first wall or solid cell, in cells. A fixed-step march
    misses a ray clipping a corner for less than its step."""
    dx, dy = math.cos(angle), math.sin(angle)
    cx, cy = math.floor(x), math.floor(y)
    sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    tdx = abs(1.0 / dx) if dx else math.inf
    tdy = abs(1.0 / dy) if dy else math.inf
    tx = ((cx + 1 - x) if dx > 0 else (x - cx)) * tdx if dx else math.inf
    ty = ((cy + 1 - y) if dy > 0 else (y - cy)) * tdy if dy else math.inf
    t = 0.0

    def hit(i, j):
        return (not (0 <= j < len(world.layout) and 0 <= i < len(world.layout[0]))
                or world.layout[j][i] == "#" or (i, j) in solid)
    while t <= max_cells:
        if hit(cx, cy):
            return t
        if abs(tx - ty) < 1e-9 and (hit(cx + sx, cy) or hit(cx, cy + sy)):
            return tx                  # through a corner: touching either side is a hit
        if tx < ty:
            t, tx, cx = tx, tx + tdx, cx + sx
        else:
            t, ty, cy = ty, ty + tdy, cy + sy
    return math.inf


def _a2_house(house, monkeypatch):
    from sim import renderer
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    monkeypatch.delenv("SIM_HAZARD_RECALL", raising=False)
    monkeypatch.delenv("SIM_HAZARD_FP", raising=False)
    if house.startswith("procthor"):
        monkeypatch.setenv("SIM_LOW_SCATTER", "8:0")
    else:
        monkeypatch.delenv("SIM_LOW_SCATTER", raising=False)
    world, bare = build_world(house), build_world(house)
    low = set(world.low_cells)
    assert low, house
    bare.objects = {c: n for c, n in world.objects.items() if c not in low}
    robot, bare_robot = MockRobot(world, render=False), MockRobot(bare, render=False)
    heights = world.object_heights
    low_labels = {n for n in set(world.objects.values()) if heights.get(n, math.inf) < 0.12}
    counts = {"scan": 0, "band_zones": 0, "camera_seen": 0, "camera_hidden": 0}
    for x, y, th, tilt, pan in _poses(world, POSES, f"3237-a2-{house}"):
        for w in (world, bare):
            w.x, w.y, w.theta, w.pan = x, y, th, pan
        robot.tilt_deg = tilt
        # The lidar: the low things change no beam.
        assert robot.get_scan()["ranges_m"] == bare_robot.get_scan()["ranges_m"], (house, x, y)
        counts["scan"] += 1
        # The floor band: each zone against the test's own march, one cell.
        band = robot.get_depth_grid()["floor"]
        off = band["origin_x_cm"] / 30.0
        lx, ly = x + off * math.cos(th), y + off * math.sin(th)
        fov = math.radians(band["fov_deg"])
        for i, z in enumerate(band["zones"]):
            # One ray a zone: its published bearing is the zone's centre,
            # rounded to 0.001 deg; the unrounded centre, so a ray through a
            # cell's corner is cast where the band cast it.
            rel = -fov / 2 + fov * (i + 0.5) / band["cols"]
            if z.get("bearing_deg") is not None:
                assert abs(math.degrees(rel) - z["bearing_deg"]) < 0.001, (house, i, z)
            t = _march(world, set(world.objects), lx, ly, th + rel, 150 / 30.0 + 1.0) * 30.0
            if z["status"] == "range":
                assert abs(t - z["distance_cm"]) <= 30.0, (house, i, t, z)
            elif z["status"] == "no_target":
                assert t >= 150.0 - 30.0, (house, i, t, z)
            else:
                assert t <= 20.0 + 30.0, (house, i, t, z)
            counts["band_zones"] += 1
        # The camera: low detections are exactly those the vertical rule and
        # 3.32's occlusion pass.
        got = sorted((d["label"], d["distance_m"]) for d in world.frame_description()["detections"]
                     if d["label"] in low_labels)
        view = world.view_angle()
        pitch = MOUNT_DOWN_DEG - tilt
        want = []
        for c in low:
            d = math.hypot(c[0] + 0.5 - x, c[1] + 0.5 - y)
            if not 0.5 < d <= renderer.FPV_MAX_DIST:
                continue
            others = {o for o in world.objects if o != c and o not in low}
            if renderer.visible_bearing(world.layout, x, y, view, c, solid=others) is None:
                continue
            f = _lens_forward_m(x, y, th, view, (c[0] + 0.5, c[1] + 0.5))
            if f > 0 and abs(math.degrees(math.atan(LENS_M / f)) - pitch) <= VFOV_DEG / 2:
                want.append((world.objects[c], round(d * 0.30, 3)))
                counts["camera_seen"] += 1
            else:
                counts["camera_hidden"] += 1
        assert got == sorted(want), (house, x, y, tilt, got, want)
    return counts


def test_a3_a_level_camera_misses_the_floor_at_its_feet(monkeypatch):
    """A3, red first on `dev`: with the optical axis level, a low thing
    nearer than h / tan(vfov / 2) is not detected; tilted down, it is."""
    from sim.maps import build_world
    monkeypatch.delenv("SIM_HAZARD_RECALL", raising=False)
    monkeypatch.delenv("SIM_HAZARD_FP", raising=False)
    world = build_world("complex_house_hazards")
    world.camera_mount_pitch_deg = 0.0
    near = LENS_M / math.tan(math.radians(VFOV_DEG / 2)) / 0.30        # cells, from the lens
    cell = next(c for c, n in world.objects.items() if n == "sock")
    # The lens 0.75 x `near` behind the sock, facing it, nothing in between.
    world.theta, world.pan = 0.0, 0
    world.x = cell[0] + 0.5 - 0.75 * near - (PAN_X_M + LENS_X_M) / 0.30
    world.y = cell[1] + 0.5
    assert not world.footprint_overlaps(world.x, world.y)
    frame = world.frame_description
    labels = lambda: [d["label"] for d in frame()["detections"]]  # noqa: E731
    assert "sock" not in labels() and "sock" not in frame()["objects_visible"]
    world.camera_tilt_deg = -30.0                                      # look down
    assert "sock" in labels() and "sock" in frame()["objects_visible"]


def test_a4_the_scatter_is_seeded_and_closes_no_doorway(monkeypatch):
    """A4: the same cells for the same seed, different for another; none
    within 1 m of the start or on an object; the floor reachable from the
    start, less the cells laid, is unchanged."""
    from sim.maps import _build_world, _reachable, build_world
    for house in ["scaled_house", "home_first_floor"] + SCATTERED[:3]:
        plain = _build_world(house)
        start = (plain.robot_x, plain.robot_y)
        floor = _reachable(plain, start, set(plain.objects))
        monkeypatch.setenv("SIM_LOW_SCATTER", "8:0")
        a, b = build_world(house), build_world(house)
        monkeypatch.setenv("SIM_LOW_SCATTER", "8:1")
        other = build_world(house)
        laid = set(a.objects) - set(plain.objects)
        assert a.objects == b.objects and other.objects != a.objects
        assert len(laid) == 8 and laid <= set(a.low_cells)
        assert set(plain.objects) <= set(a.objects)
        assert all(a.objects[c] == plain.objects[c] for c in plain.objects)
        assert all(math.dist(c, start) * 0.30 >= 1.0 for c in laid)
        # Open floor: the cell and its eight neighbours plain and empty.
        assert all(plain.layout[c[1] + j][c[0] + i] == "." and (c[0] + i, c[1] + j) not in plain.objects
                   for c in laid for i in (-1, 0, 1) for j in (-1, 0, 1))
        assert _reachable(a, start, set(a.objects)) == floor - laid
        # Nothing that stood is lowered.
        assert set(a.low_cells) - laid == set(plain.low_cells)


def test_a_sim_floor_band_0_withholds_the_band(monkeypatch):
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    robot = MockRobot(build_world("complex_house_hazards"), render=False)
    assert "floor" in robot.get_depth_grid()
    monkeypatch.setenv("SIM_FLOOR_BAND", "0")
    assert "floor" not in robot.get_depth_grid()
    from robot.safety import SafetyController
    assert SafetyController(robot, 20.0).floor_clearance() == (None, "no_floor_band")


def test_a_the_tilt_lives_on_the_world():
    """`MockRobot.tilt_deg` reads and writes the world's; a look levels it."""
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    robot = MockRobot(build_world("complex_house_hazards"), render=False)
    robot.tilt_step(-20)
    assert robot.world.camera_tilt_deg == robot.tilt_deg == -20.0
    robot.look_center()
    assert robot.world.camera_tilt_deg == 0.0


def test_a_the_scatter_reuses_no_label_and_keeps_the_tilt_across_houses(monkeypatch):
    """Review: no scattered copy of a label the house holds (a find's target
    could be one); the hazard house, which holds them all, refuses; a house
    change keeps the tilt."""
    import pytest
    from sim.maps import _build_world, build_world
    from sim.mock_robot import MockRobot
    monkeypatch.setenv("SIM_LOW_SCATTER", "8:0")
    for house in SCATTERED[:2]:
        plain, a = _build_world(house), build_world(house)
        laid = set(a.objects) - set(plain.objects)
        assert not {a.objects[c] for c in laid} & set(plain.objects.values())
    with pytest.raises(ValueError, match="every scatter label"):
        build_world("complex_house_hazards")
    monkeypatch.delenv("SIM_LOW_SCATTER")
    robot = MockRobot(build_world("complex_house_hazards"), render=False)
    robot.tilt_step(-20)
    robot.load_house(build_world("scaled_house"))
    assert robot.tilt_deg == -20.0
