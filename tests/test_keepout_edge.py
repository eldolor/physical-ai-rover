# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.193 E1: the bridge's keepout mask marks every 0.05 m cell a no-go zone
touches, so no point inside a zone lies in a cell nav2 may plan through.

Before 3.193 a cell was marked only when its CENTRE lay inside the zone, so up
to half a cell (0.025 m) along every edge stayed passable; 3.192's live run
(F9) saw the robot's true centre 2 mm inside zone `za15d23feaa38`'s south edge.
"""

import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "service/slam/src/picar_bridge"))
from picar_bridge import keepout  # noqa: E402


def _rect(x0, y0, x1, y1, zid="z"):
    return {"id": zid, "poly": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]}


def _rotated(cx, cy, w, h, deg, zid="r"):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    pts = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
    return {"id": zid, "poly": [[cx + c * x - s * y, cy + s * x + c * y] for x, y in pts]}


def _cell_value(m, x, y, outside=None):
    i = math.floor((x - m["origin_x_m"]) / m["resolution_m"])
    j = math.floor((y - m["origin_y_m"]) / m["resolution_m"])
    if not (0 <= i < m["width"] and 0 <= j < m["height"]):
        assert outside is not None, (x, y)
        return outside
    return m["data"][j * m["width"] + i]


def _clip_area(poly, cx0, cy0, res):
    """The exact area of polygon (convex) within the cell [cx0, cx0+res] x
    [cy0, cy0+res], by Sutherland-Hodgman (a test oracle, not the code's rule)."""
    pts = [tuple(p) for p in poly]
    for axis, bound, keep_ge in ((0, cx0, True), (0, cx0 + res, False),
                                 (1, cy0, True), (1, cy0 + res, False)):
        out = []
        for k in range(len(pts)):
            p, q = pts[k], pts[(k + 1) % len(pts)]
            pin = p[axis] >= bound if keep_ge else p[axis] <= bound
            qin = q[axis] >= bound if keep_ge else q[axis] <= bound
            if pin:
                out.append(p)
            if pin != qin:
                t = (bound - p[axis]) / (q[axis] - p[axis])
                out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
        pts = out
        if not pts:
            return 0.0
    return abs(sum(pts[k][0] * pts[(k + 1) % len(pts)][1] - pts[(k + 1) % len(pts)][0] * pts[k][1]
                   for k in range(len(pts)))) / 2


# 3.192's live zone (house frame) and its F9 truth sample, 2 mm inside y0.
F9_ZONE = _rect(4.344, 1.386, 4.956, 2.514, "za15d23feaa38")
F9_POINT = (4.897969838375288, 1.3883674706130045)


@pytest.mark.parametrize("shift", [0.0, 0.01, 0.02, 0.03, 0.04])
def test_the_f9_point_2mm_inside_a_zone_edge_is_lethal(shift):
    """Live, the grid's rows fell where the anchor's rotation put them; here a
    second zone shifts the grid's origin under the F9 zone by `shift`."""
    other = _rect(0.432, 0.886 - shift, 0.968, 1.2, "zo")          # sets the origin_y
    z = keepout.parse_zones([F9_ZONE, other])
    assert keepout.zone_distance(z[0], *F9_POINT) < 0                # inside
    assert _cell_value(keepout.mask([], zones=z), *F9_POINT) == 100


def test_an_edge_between_a_cells_centre_and_its_far_side_marks_the_cell():
    """An anchor zone pins the grid's origin at (-0.1, -0.1), so row 22 covers
    y in [1.00, 1.05) with its centre at 1.025. A zone whose south edge is at
    1.04 holds the strip [1.04, 1.05) of that row: the row is marked though its
    centre is outside the zone; the row below is not."""
    anchor = _rect(0.0, 0.0, 0.2, 0.2, "anchor")
    zone = _rect(3.0, 1.04, 4.0, 2.0, "z")
    m = keepout.mask([], zones=keepout.parse_zones([anchor, zone]))
    assert m["origin_y_m"] == pytest.approx(-0.1)
    assert _cell_value(m, 3.5, 1.045) == 100
    assert _cell_value(m, 3.5, 1.01) == 100                          # the same cell
    assert _cell_value(m, 3.5, 0.99) == -1


@pytest.mark.parametrize("zone", [
    F9_ZONE,
    _rect(0.532, 3.455, 0.968, 4.045, "z5100dacad3c8"),
    _rect(1.615, 2.822, 2.285, 3.478, "zb6ae70cafb14"),
    _rotated(2.0, 2.0, 1.0, 0.6, 30.0),
    _rotated(-1.3, 0.7, 0.4, 2.2, -63.0),
    _rect(0.0, 0.0, 1.0, 0.1, "wall"),                  # a wall, 0.10 m wide
], ids=lambda z: z["id"])
def test_every_point_just_inside_an_edge_is_lethal(zone):
    z = keepout.parse_zones([zone])[0]
    m = keepout.mask([], zones=[z])
    poly = z["poly"]
    n = len(poly)
    gx, gy = sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n
    checked = 0
    for k in range(n):
        (ax, ay), (bx, by) = poly[k], poly[(k + 1) % n]
        L = math.hypot(bx - ax, by - ay)
        nx, ny = -(by - ay) / L, (bx - ax) / L                       # a normal
        if (gx - ax) * nx + (gy - ay) * ny < 0:
            nx, ny = -nx, -ny                                        # made inward
        for f in [i / 40 for i in range(1, 40)]:
            for depth_mm in (1, 2, 5, 12, 24):
                d = depth_mm / 1000
                x, y = ax + f * (bx - ax) + nx * d, ay + f * (by - ay) + ny * d
                if keepout.zone_distance(z, x, y) < 0:
                    checked += 1
                    assert _cell_value(m, x, y) == 100, (zone["id"], x, y)
    assert checked > 500


@pytest.mark.parametrize("zone", [
    F9_ZONE,
    _rotated(2.0, 2.0, 1.0, 0.6, 30.0),
    _rotated(-1.3, 0.7, 0.4, 2.2, -63.0),
    _rect(0.0, 0.0, 1.0, 0.5, "aligned"),
], ids=lambda z: z["id"])
def test_every_lethal_cell_overlaps_the_zone(zone):
    """Exactly the cells the zone overlaps: a marked cell holds some of the
    zone (no over-drawing), and a cell holding any of it (more than a 1e-6 m
    sliver along a side, EDGE_EPS_M's tolerance) is marked."""
    z = keepout.parse_zones([zone])[0]
    m = keepout.mask([], zones=[z])
    res, w = m["resolution_m"], m["width"]
    for j in range(m["height"]):
        for i in range(w):
            area = _clip_area(z["poly"], m["origin_x_m"] + i * res, m["origin_y_m"] + j * res, res)
            if m["data"][j * w + i] == 100:
                assert area > 1e-12, (i, j)
            elif area > res * 1e-6:
                raise AssertionError(f"cell {(i, j)} holds {area:.2e} m2 of the zone, unmarked")


def test_random_points_inside_random_zones_are_lethal():
    rng = random.Random(3193)
    for _ in range(40):
        z = keepout.parse_zones([_rotated(rng.uniform(-5, 5), rng.uniform(-5, 5),
                                          rng.uniform(0.2, 2.0), rng.uniform(0.2, 2.0),
                                          rng.uniform(-180, 180))])[0]
        m = keepout.mask([], zones=[z])
        xs, ys = [p[0] for p in z["poly"]], [p[1] for p in z["poly"]]
        for _ in range(300):
            x, y = rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys))
            if keepout._inside_poly(z["poly"], x, y):
                assert _cell_value(m, x, y) == 100


def test_an_edge_on_a_cell_boundary_marks_only_the_zones_side():
    """1.0 x 0.5 m with its edges on cell boundaries: 20 x 10 cells, as before."""
    m = keepout.mask([], zones=keepout.parse_zones([_rect(0.0, 0.0, 1.0, 0.5)]))
    assert m["data"].count(100) == 200


def test_a_largest_zone_draws_quickly():
    """A 15 m rotated zone (world/nogo.py's cap) is drawn under the bridge's
    lock: the edge walk keeps it near the centre rule's cost."""
    import time
    z = keepout.parse_zones([_rotated(0.0, 0.0, 15.0, 15.0, 37.0)])
    t = time.perf_counter()
    m = keepout.mask([], zones=z)
    assert time.perf_counter() - t < 5.0
    assert abs(m["data"].count(100) * 0.05 ** 2 - 225.0) < 4.0


def test_spots_keep_their_centre_rule():
    """D1: the wedge discs are drawn by their own loop, unchanged."""
    spots = keepout.parse([{"x_m": 1.0, "y_m": 1.0, "r_m": 0.15, "ttl_s": 5}], now=0.0)
    m = keepout.mask(spots)
    res, w = m["resolution_m"], m["width"]
    for j in range(m["height"]):
        for i in range(w):
            cx = m["origin_x_m"] + (i + 0.5) * res
            cy = m["origin_y_m"] + (j + 0.5) * res
            inside = (cx - 1.0) ** 2 + (cy - 1.0) ** 2 <= 0.15 ** 2
            assert (m["data"][j * w + i] == 100) == inside


# ---------- D4: boxed by the drawn cells (3.193 reviews) ----------

def test_a_robot_boxed_by_the_drawn_edge_cell_suspends_the_zone():
    """The review's case: the lattice puts rows at 1.00 / 1.05; zone y0 = 1.03;
    the robot at y = 1.06 is only 0.03 m inside the zone, but its own cell
    [1.05, 1.10), the edge cell [1.00, 1.05) below it and its other three
    neighbours are all drawn, so NavFn (which frees only the robot's own cell)
    could not plan out: suspended (D4's 0.05 m depth alone would not)."""
    z = keepout.parse_zones([_rect(3.0, 1.03, 4.0, 2.0, "z")])
    m = keepout.mask([], zones=z)
    assert _cell_value(m, 3.5, 1.06) == 100 and _cell_value(m, 3.5, 1.02) == 100
    assert keepout.boxed(z[0], 3.5, 1.06)
    assert keepout.suspend(z, set(), (3.5, 1.06)) == {"z"}


def test_a_robot_hugging_the_drawn_edge_does_not_suspend_the_zone():
    """0.01 m outside the zone, in its drawn edge cell: the cell below is free."""
    z = keepout.parse_zones([_rect(3.0, 1.03, 4.0, 2.0, "z")])
    assert not keepout.boxed(z[0], 3.5, 1.02)
    assert keepout.suspend(z, set(), (3.5, 1.02)) == set()
    assert keepout.suspend(z, set(), (3.5, 1.0)) == set()


def test_a_robot_more_than_005_inside_is_suspended_as_before():
    z = keepout.parse_zones([_rotated(2.0, 2.0, 1.0, 0.6, 30.0, "r")])
    rng = random.Random(7)
    for _ in range(300):
        x, y = rng.uniform(1.4, 2.6), rng.uniform(1.4, 2.6)
        if keepout.zone_distance(z[0], x, y) < -keepout.SUSPEND_DEPTH_M:
            assert keepout.suspend(z, set(), (x, y)) == {"r"}


@pytest.mark.parametrize("phase", [(0.0, 0.0), (0.013, 0.037), (0.049, 0.001)])
def test_a_robot_the_zones_keep_drawn_always_has_a_free_4_neighbour(phase):
    """D4's promise on the published mask: while a zone stays drawn under the
    robot, the robot's cell or one of its 4 neighbours (NavFn's A*) is not
    lethal, on the mask's lattice (the SLAM map's, so the costmap's)."""
    rng = random.Random(3193)
    for _ in range(30):
        z = keepout.parse_zones([_rotated(rng.uniform(-3, 3), rng.uniform(-3, 3),
                                          rng.uniform(0.3, 1.5), rng.uniform(0.3, 1.5),
                                          rng.uniform(-180, 180), "r")])
        m = keepout.mask([], zones=z, phase=phase)
        xs, ys = [p[0] for p in z[0]["poly"]], [p[1] for p in z[0]["poly"]]
        for _ in range(200):
            x, y = rng.uniform(min(xs) - 0.08, max(xs) + 0.08), rng.uniform(min(ys) - 0.08, max(ys) + 0.08)
            if keepout.suspend(z, set(), (x, y), phase=phase):
                continue
            free = [(dx, dy) for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))
                    if _cell_value(m, x + dx * 0.05, y + dy * 0.05, outside=-1) != 100]
            assert free, (x, y)


def test_the_mask_sits_on_the_slam_maps_lattice():
    """KeepoutFilter samples the mask at each costmap cell's centre; with the
    map's phase a mask cell is a costmap cell."""
    ph = keepout.phase_of(-3.4127, 2.0071)
    assert ph == pytest.approx((0.0373, 0.0071), abs=1e-9)
    m = keepout.mask([], zones=keepout.parse_zones([_rect(1.0, 1.0, 2.0, 2.0)]), phase=ph)
    for v, p in ((m["origin_x_m"], ph[0]), (m["origin_y_m"], ph[1])):
        k = (v - p) / 0.05
        assert abs(k - round(k)) < 1e-9
    # every point inside is still lethal on a shifted lattice
    z = keepout.parse_zones([F9_ZONE])
    m = keepout.mask([], zones=z, phase=ph)
    assert _cell_value(m, *F9_POINT) == 100


def test_a_zones_cells_do_not_move_when_another_zone_comes_or_goes():
    """The fixed lattice: zone a's drawn cells, in map coordinates, are the
    same alone and beside another zone that moves the grid's corner."""
    a = _rect(1.013, 1.027, 1.6, 1.9, "a")
    b = _rect(-0.333, -0.777, 0.2, 0.2, "b")

    def cells(m):
        w, res = m["width"], m["resolution_m"]
        return {(round(m["origin_x_m"] / res) + i, round(m["origin_y_m"] / res) + j)
                for j in range(m["height"]) for i in range(w) if m["data"][j * w + i] == 100}
    alone = cells(keepout.mask([], zones=keepout.parse_zones([a])))
    both = cells(keepout.mask([], zones=keepout.parse_zones([a, b])))
    only_b = cells(keepout.mask([], zones=keepout.parse_zones([b])))
    assert alone and alone == both - only_b


def test_a_zone_with_no_area_is_refused():
    with pytest.raises(ValueError):
        keepout.parse_zones([{"id": "flat", "poly": [[0.013, 0.013], [1.013, 0.513],
                                                     [1.013, 0.513], [0.013, 0.013]]}])
    with pytest.raises(ValueError):                    # a 15 m sliver 10 um wide
        keepout.parse_zones([{"id": "sliver", "poly": [[0, 0], [15, 0], [15, 1e-5], [0, 1e-5]]}])
    keepout.parse_zones([{"id": "wall", "poly": [[0, 0], [15, 0], [15, 0.1], [0, 0.1]]}])


def test_the_bridge_draws_and_suspends_on_the_maps_lattice():
    src = (Path(__file__).resolve().parent.parent
           / "service/slam/src/picar_bridge/picar_bridge/bridge.py").read_text()
    assert "phase=self.mask_phase" in src and src.count("phase=self.mask_phase") == 2
    assert "keepout.phase_of(info.origin.position.x, info.origin.position.y)" in src


def test_two_zones_a_gap_apart_that_box_the_robot_together_are_suspended():
    """Review round 3: A and B 0.02 m apart; the robot in the gap is outside
    both, but its cell and its four neighbours are all drawn by one or the
    other: both are left out."""
    zs = keepout.parse_zones([_rect(0.0, 0.0, 1.0, 1.0, "a"), _rect(0.0, 1.02, 1.0, 2.0, "b")])
    m = keepout.mask([], zones=zs)
    assert all(_cell_value(m, 0.5 + dx * 0.05, 1.01 + dy * 0.05) == 100
               for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)))
    assert keepout.suspend(zs, set(), (0.5, 1.01)) == {"a", "b"}
    # a zone far away is not touched by it
    zs2 = zs + keepout.parse_zones([_rect(3.0, 3.0, 4.0, 4.0, "c")])
    assert keepout.suspend(zs2, set(), (0.5, 1.01)) == {"a", "b"}


@pytest.mark.parametrize("n", range(-400, 400, 7))
def test_a_phase_on_the_lattice_is_zero(n):
    ph = keepout.phase_of(n * 0.05, -n * 0.05)
    assert ph == (0.0, 0.0)
    assert not keepout.phase_moved(ph, (0.05 - 1e-9, 0.0))
    assert keepout.phase_moved(keepout.phase_of(n * 0.05 + 0.013, 0.0), (0.0, 0.0))


def test_several_zones_never_box_a_robot_they_keep_drawn():
    """E6 over the union: three zones placed close together (gaps of a few
    cm), random robots; whatever stays drawn leaves a free 4-neighbour."""
    rng = random.Random(31930)
    for _ in range(60):
        cx, cy = rng.uniform(-2, 2), rng.uniform(-2, 2)
        zs = keepout.parse_zones([
            _rotated(cx + rng.uniform(-0.6, 0.6), cy + rng.uniform(-0.6, 0.6),
                     rng.uniform(0.2, 0.9), rng.uniform(0.2, 0.9), rng.uniform(-90, 90), f"z{k}")
            for k in range(3)])
        ph = (rng.uniform(0, 0.05), rng.uniform(0, 0.05))
        for _ in range(150):
            x, y = cx + rng.uniform(-1.2, 1.2), cy + rng.uniform(-1.2, 1.2)
            out = keepout.suspend(zs, set(), (x, y), phase=ph)
            m = keepout.mask([], zones=[z for z in zs if z["id"] not in out], phase=ph)
            if m["data"] == [-1]:
                continue
            free = [(dx, dy) for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))
                    if _cell_value(m, x + dx * 0.05, y + dy * 0.05, outside=-1) != 100]
            assert free, (x, y, out)


def test_health_lists_the_suspended_ids_and_counts_apart():
    """3.193: /health's nogo block kept the counter under `suspended`, over
    the ids list (`**self.zone_events` last)."""
    src = (Path(__file__).resolve().parent.parent
           / "service/slam/src/picar_bridge/picar_bridge/bridge.py").read_text()
    view = src[src.index("def _nogo_view"):src.index("def _publish_keepout")]
    assert '"suspended": sorted(self.zones_suspended)' in view
    assert "**self.zone_events" not in view and '"suspended_count"' in view
