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
