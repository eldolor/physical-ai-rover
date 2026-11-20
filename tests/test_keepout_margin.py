# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.195 C6-C7: the bridge draws each no-go zone grown by one cell
(`keepout.GROW_M`), so a plan hugging the drawn edge keeps 0.05 m more from
the zone; D4's leave-a-zone rule still holds on the grown mask.

3.193's R1 saw the robot's true centre 3.6 and 2.9 mm inside a zone's corner
with every cell the zone touches drawn: nav2's controller tracks the plan on
its own, and the keepout mask is on the global costmap only.
"""

import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "service/slam/src/picar_bridge"))
from picar_bridge import keepout  # noqa: E402
from tests.test_review_376h1 import bridge_mod  # noqa: E402,F401  (the fixture: bridge.py on stubs)

G = keepout.GROW_M
RES = keepout.RES_M


def _rect(x0, y0, x1, y1, zid="z"):
    return {"id": zid, "poly": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]}


def _rotated(cx, cy, w, h, deg, zid="r"):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    pts = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
    return {"id": zid, "poly": [[cx + c * x - s * y, cy + s * x + c * y] for x, y in pts]}


def _cell(m, x, y, outside=-1):
    i = math.floor((x - m["origin_x_m"]) / m["resolution_m"])
    j = math.floor((y - m["origin_y_m"]) / m["resolution_m"])
    if not (0 <= i < m["width"] and 0 <= j < m["height"]):
        return outside
    return m["data"][j * m["width"] + i]


def _zones(*zs):
    """As the bridge holds them: parsed, then given the margin."""
    return keepout.with_margin(keepout.parse_zones(list(zs)))


R1_ZONE = _rect(4.344, 1.386, 4.956, 2.514, "z89cc245b57e8")     # 3.193 R1's corner
R1_POINTS = [(4.952445, 1.397572), (4.899890, 1.388866)]          # its two entries


# ---------- grown() ----------

ARC = G / math.cos(math.pi / 2 / keepout.ARC_STEPS / 2) - G     # the arc's polygon stands this far proud


def test_a_rectangle_grows_one_cell_each_way_with_rounded_corners():
    for poly in (R1_ZONE["poly"], list(reversed(R1_ZONE["poly"]))):   # either winding
        g = keepout.grown(poly)
        xs, ys = sorted(p[0] for p in g), sorted(p[1] for p in g)
        assert xs[0] == pytest.approx(4.344 - G) and xs[-1] == pytest.approx(4.956 + G)
        assert ys[0] == pytest.approx(1.386 - G) and ys[-1] == pytest.approx(2.514 + G)
        # the corner is an arc, not a mitre: nothing reaches past G (and the arc's allowance)
        assert max(-keepout.zone_distance({"poly": [tuple(p) for p in poly]}, *q) for q in g) < 0
        assert max(keepout.zone_distance({"poly": [tuple(p) for p in poly]}, *q) for q in g) <= G + ARC + 1e-12


def test_a_rotated_rectangle_grows_by_g_along_every_edge():
    z = _rotated(1.0, 2.0, 0.8, 0.5, 37.0)
    g = keepout.grown(z["poly"])
    for (ax, ay), (bx, by) in zip(z["poly"], z["poly"][1:] + z["poly"][:1]):
        mx, my = (ax + bx) / 2, (ay + by) / 2
        assert -keepout.zone_distance({"poly": g}, mx, my) == pytest.approx(G, abs=1e-9)


@pytest.mark.parametrize("poly", [
    [(0, 0), (2, 1), (0, 2), (0.5, 1)],                   # a dart: a reflex corner
    [(0, 0.1), (3, 0), (3.05, 0.1), (3, 0.2)],            # a kite, one corner about 4 degrees
    [(0, 0), (1, 0), (1, 0), (0, 1)],                     # a repeated corner
])
def test_any_polygon_grows_to_cover_every_point_within_g(poly):
    """The hull, rounded: every point within G of the polygon is inside, and
    nothing is farther than G (plus the arc's allowance) from the hull."""
    gp = keepout.grown(poly)
    hull = keepout._hull(poly)
    rng = random.Random(5)
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    for _ in range(3000):
        x, y = rng.uniform(min(xs) - 0.1, max(xs) + 0.1), rng.uniform(min(ys) - 0.1, max(ys) + 0.1)
        if keepout.zone_distance({"poly": [tuple(p) for p in poly]}, x, y) < G - 1e-9:
            assert keepout.zone_distance({"poly": gp}, x, y) < 0, (x, y)
    assert max(keepout.zone_distance({"poly": hull}, *q) for q in gp) <= G / math.cos(math.pi / keepout.ARC_STEPS / 2) + 1e-12


def test_the_parsed_zone_is_kept_as_drawn_by_the_person():
    z = _zones(R1_ZONE)[0]
    assert z["poly"] == [tuple(p) for p in R1_ZONE["poly"]]
    assert z["drawn"] == keepout.grown(z["poly"])
    assert keepout.with_margin(keepout.parse_zones([R1_ZONE]), 0.0)[0]["drawn"] == z["poly"]


# ---------- C6: the grown mask ----------

def test_r1s_entries_and_their_margin_are_lethal():
    """R1's two entries, and the same points pushed 0.049 m out of the zone."""
    zs = _zones(R1_ZONE)
    for phase in ((0.0, 0.0), (0.0362, 0.035), (0.0191, 0.0084)):     # 3.193 E7's phases
        m = keepout.mask([], zones=zs, phase=phase)
        for x, y in R1_POINTS + [(4.956 + 0.049, 1.40), (4.90, 1.386 - 0.049),
                                 (4.956 + 0.034, 1.386 - 0.034)]:     # beyond the corner, inside the mitre
            assert _cell(m, x, y) == 100, (phase, x, y)


@pytest.mark.parametrize("phase", [(0.0, 0.0), (0.013, 0.037), (0.049, 0.001)])
def test_every_point_inside_or_within_g_of_a_zone_is_lethal(phase):
    rng = random.Random(3195)
    for k in range(40):
        z = (_rotated(rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(0.2, 1.5),
                      rng.uniform(0.2, 1.5), rng.uniform(-180, 180)) if k % 2 else
             _rect(*(lambda x, y: (x, y, x + rng.uniform(0.2, 1.5), y + rng.uniform(0.2, 1.5)))(
                 rng.uniform(-3, 3), rng.uniform(-3, 3))))
        zs = _zones(z)
        m = keepout.mask([], zones=zs, phase=phase)
        xs, ys = [p[0] for p in z["poly"]], [p[1] for p in z["poly"]]
        hit = 0
        for _ in range(400):
            x, y = rng.uniform(min(xs) - 0.07, max(xs) + 0.07), rng.uniform(min(ys) - 0.07, max(ys) + 0.07)
            if keepout.zone_distance(zs[0], x, y) < G - 1e-6:
                hit += 1
                assert _cell(m, x, y) == 100, (z, x, y)
        assert hit


@pytest.mark.parametrize("phase", [(0.0, 0.0), (0.021, 0.044)])
def test_every_lethal_cell_lies_within_g_and_a_cell_diagonal_of_the_zone(phase):
    rng = random.Random(1951)
    worst = [0.0]
    for _ in range(30):
        z = _rotated(rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(0.2, 1.5),
                     rng.uniform(0.2, 1.5), rng.uniform(-180, 180))
        zs = _zones(z)
        m = keepout.mask([], zones=zs, phase=phase)
        w = m["width"]
        for j in range(m["height"]):
            for i in range(w):
                if m["data"][j * w + i] == 100:
                    cx = m["origin_x_m"] + (i + 0.5) * RES
                    cy = m["origin_y_m"] + (j + 0.5) * RES
                    # C6: the whole cell within G and one cell's diagonal (plus the arc's allowance)
                    far = max(keepout.zone_distance(zs[0], cx + sx * RES / 2, cy + sy * RES / 2)
                              for sx in (-1, 1) for sy in (-1, 1))
                    assert far <= G + RES * math.sqrt(2) + ARC + 1e-9
                    worst[0] = max(worst[0], far)
    assert worst[0] > G + RES                    # the bound is approached, not vacuous


def test_spots_are_not_grown():
    m = keepout.mask([{"x_m": 1.0, "y_m": 1.0, "r_m": 0.15, "until": 1e12}], zones=_zones())
    assert _cell(m, 1.0, 1.0 + 0.16) != 100


# ---------- C7: D4 with the margin ----------

@pytest.mark.parametrize("phase", [(0.0, 0.0), (0.013, 0.037), (0.049, 0.001)])
def test_a_robot_the_grown_zone_keeps_drawn_always_has_a_free_4_neighbour(phase):
    """3.193 E6 on the grown mask: robots inside a zone (to beyond the 0.05 m
    depth) and in its margin; wherever the zone stays drawn, the robot's cell
    or one of its four neighbours is not lethal."""
    rng = random.Random(31957)
    checked = 0
    for _ in range(30):
        zs = _zones(_rotated(rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(0.3, 1.5),
                             rng.uniform(0.3, 1.5), rng.uniform(-180, 180), "r"))
        m = keepout.mask([], zones=zs, phase=phase)
        for _ in range(300):
            # near the zone: between 0.08 m inside and G + 0.04 m outside
            px, py = rng.choice(zs[0]["poly"])
            x, y = px + rng.uniform(-0.6, 0.6), py + rng.uniform(-0.6, 0.6)
            d = keepout.zone_distance(zs[0], x, y)
            if not -0.08 < d < G + 0.04:
                continue
            if keepout.suspend(zs, set(), (x, y), phase=phase):
                continue
            checked += 1
            free = [(dx, dy) for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))
                    if _cell(m, x + dx * RES, y + dy * RES) != 100]
            assert free, (x, y, d)
    assert checked > 500


@pytest.mark.parametrize("phase", [(0.0, 0.0), (0.0, 0.031), (0.017, 0.049), (0.033, 0.008)])
def test_a_robot_outside_the_grown_edge_does_not_suspend_the_zone(phase):
    """A plan keeps to the cells outside the drawn polygon: a robot outside it
    along a straight edge of an axis-aligned zone (its nearest zone point at
    least 0.10 m from a corner) leaves the zone drawn, on any lattice phase.

    Not on a slanted edge: a line at an angle crosses the robot's cell and
    both outward neighbours for some positions, so D4 boxes the robot. That
    is 3.193's touched-cell rule, there before the margin: robots 0-40 mm
    outside the drawn edge of rotated zones, 641 of 12,000 suspended without
    the margin, 558 of 12,000 with it (0 of 12,000 either way when aligned).

    Not at a corner either: there a robot just outside the drawn polygon can have
    its own cell and both outward neighbours touched by slivers of it, so
    D4 boxes it and leaves the zone out until it is LEAVE_M clear. Measured
    (3.195.B review), robots 0-30 mm outside near the corners: 21 of 7,426
    positions on axis-aligned zones, 148 of 7,270 on rotated ones (a mitred
    margin: 0 and 257). The live runs count suspensions during each find."""
    rng = random.Random(4)
    checked = 0
    for _ in range(20):
        zs = _zones(_rotated(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0.4, 1.2),
                             rng.uniform(0.4, 1.2), rng.choice([0.0, 90.0, 180.0, -90.0]), "z"))
        poly = zs[0]["poly"]
        for _ in range(300):
            a, b = rng.choice(list(zip(poly, poly[1:] + poly[:1])))
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            t = rng.uniform(0.10, L - 0.10) / L
            nx, ny = (b[1] - a[1]) / L, (a[0] - b[0]) / L         # outward: counter-clockwise winding
            off = G + rng.uniform(1e-4, 0.04)
            x, y = a[0] + t * (b[0] - a[0]) + off * nx, a[1] + t * (b[1] - a[1]) + off * ny
            assert keepout.zone_distance({"poly": zs[0]["drawn"]}, x, y) > 0
            checked += 1
            assert keepout.suspend(zs, set(), (x, y), phase=phase) == set(), (x, y, phase)
    assert checked == 6000


def _step(zs, off, susp, pose, phase=(0.0, 0.0)):
    """The bridge's _zones_step: margins first, then D4 on the zones as drawn."""
    off = keepout.margins_off(zs, off, susp, pose, phase)
    susp = keepout.suspend(keepout.bare(zs, off), susp, pose, phase=phase)
    drawn = [z for z in keepout.bare(zs, off) if z["id"] not in susp]
    return off - susp, susp, keepout.mask([], zones=drawn, phase=phase)     # zones_bare = off - after


PHASES = [(0.0, 0.0), (0.0, 0.031), (0.049, 0.049), (0.04, 0.04), (0.017, 0.008)]


@pytest.mark.parametrize("phase", PHASES)
def test_a_robot_in_the_margin_never_switches_the_zone_off(phase):
    """Both reviewers' case: a robot 0-50 mm outside a straight edge, inside
    the margin, was boxed by the margin's cells and D4 left the WHOLE zone out
    until it was 0.30 m clear. Now only the margin comes off: the zone itself
    stays drawn, every point inside it lethal."""
    zs = _zones(_rect(3.0, 1.03, 4.0, 2.0, "z"))
    for k in range(51):
        y = 1.03 - k * 0.001
        off, susp, m = _step(zs, set(), set(), (3.5, y), phase)
        assert susp == set(), (y, phase)
        assert all(_cell(m, 3.0005 + 0.00999 * i, 1.0301 + 0.009 * jj) == 100
                   for i in range(101) for jj in range(108)), (y, phase)


def test_a_robot_just_inside_the_zone_keeps_the_zone_drawn_and_plans_out():
    """D4: 0.01-0.04 m inside the zone: the margin comes off and the bare
    zone (3.193's drawing) leaves the robot a free 4-neighbour; deeper than
    SUSPEND_DEPTH_M the whole zone is left out, as before."""
    zs = _zones(_rect(3.0, 1.03, 4.0, 2.0, "z"))
    for phase in PHASES:
        for y in (1.035, 1.04, 1.06, 1.075):
            off, susp, m = _step(zs, set(), set(), (3.5, y), phase)
            if susp:
                continue                          # boxed by the bare zone too: left out (3.193)
            assert any(_cell(m, 3.5 + dx * RES, y + dy * RES) != 100
                       for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))), (y, phase)
    assert _step(zs, set(), set(), (3.5, 1.5))[1] == {"z"}           # deep inside, as before
    assert keepout.suspend(zs, {"z"}, (3.5, 1.03 - 0.29)) == {"z"}   # leave rule from the zone
    assert keepout.suspend(zs, {"z"}, (3.5, 1.03 - 0.31)) == set()


def test_the_margin_comes_back_as_soon_as_it_would_not_box_the_robot():
    """Review round 3: not only 0.15 m out. A robot walking out of the margin
    gets it back at the first pose where the grown zone leaves it a free
    4-neighbour; while it is off, putting it back would box the robot."""
    zs = _zones(_rect(3.0, 1.03, 4.0, 2.0, "z"))
    phase = (0.049, 0.049)
    off, susp, _ = _step(zs, set(), set(), (3.5, 1.03 - 0.02), phase)
    assert off == {"z"} and susp == set()
    back = None
    for k in range(1, 30):
        y = 1.03 - 0.02 - 0.005 * k
        off, susp, m = _step(zs, off, susp, (3.5, y), phase)
        if off:                                   # it would box the robot within HYST_M
            assert any(all(keepout._boxing(zs, keepout._five(3.5 + sx * keepout.HYST_M, y + sy * keepout.HYST_M, phase)))
                       for sx in (-1, 0, 1) for sy in (-1, 0, 1))
        elif back is None:
            back = 1.03 - y
            assert _cell(m, 3.5, 1.03 - 0.045) == 100                 # grown again
    assert back is not None and back < keepout.MARGIN_BACK_M


def test_a_neighbours_margin_is_not_taken_for_a_boxing_d4_resolves():
    """Review round 3's probe: the robot 0.05 m inside zone a, 0.08 m from
    zone b. Zone b keeps its margin."""
    zs = _zones(_rect(3.0, 1.0, 4.0, 2.0, "a"), _rect(3.0, 2.03, 4.0, 3.0, "b"))
    off, susp, m = _step(zs, set(), set(), (3.5, 1.95), (0.049, 0.049))
    assert "b" not in off
    assert _cell(m, 3.5, 2.03 - 0.045) == 100


def test_a_fresh_zone_left_out_is_not_also_bare():
    zs = _zones(_rect(3.0, 1.03, 4.0, 2.0, "z"))
    off = keepout.margins_off(zs, set(), set(), (3.5, 1.01), (0.049, 0.049), fresh={"z"})
    assert off == set()


@pytest.mark.parametrize("phase", PHASES[:3])
def test_d4s_promise_holds_on_the_drawn_mask(phase):
    """3.193 E6 through the bridge's whole step: robots inside and around
    zones (two zones a few cm apart among them); wherever a zone stays drawn
    over the robot, its cell or a 4-neighbour is free."""
    rng = random.Random(1953)
    checked = 0
    for n in range(40):
        z1 = _rotated(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0.3, 1.2),
                      rng.uniform(0.3, 1.2), rng.choice([0.0, rng.uniform(-180, 180)]), "a")
        zl = [z1]
        if n % 2:
            xs = [p[0] for p in z1["poly"]]
            zl.append(_rect(max(xs) + rng.uniform(0.0, 0.12), -3, max(xs) + 0.6, 3, "b"))
        zs = _zones(*zl)
        px, py = rng.choice(zs[0]["drawn"])
        x, y = px + rng.uniform(-0.1, 0.1), py + rng.uniform(-0.1, 0.1)
        off, susp = set(), set()
        for _ in range(250):                    # a walk, the state carried (review round 5)
            x, y = x + rng.uniform(-0.01, 0.01), y + rng.uniform(-0.01, 0.01)
            off, susp, m = _step(zs, off, susp, (x, y), phase)
            checked += 1
            assert any(_cell(m, x + dx * RES, y + dy * RES) != 100
                       for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))), (x, y, off, susp)
    assert checked == 10000


def test_the_bridge_draws_every_posted_zone_with_its_margin(bridge_mod, monkeypatch):
    """Through Bridge.set_zones (rclpy stubbed): the zone is kept as posted,
    and the polygon handed to the mask is the grown one."""
    b = bridge_mod.Bridge()
    b.slam_state = "ready"
    monkeypatch.setattr(b, "_pose_in", lambda frame: None)
    drawn = []
    real = bridge_mod.keepout.mask
    monkeypatch.setattr(bridge_mod.keepout, "mask",
                        lambda spots, **kw: drawn.append(kw.get("zones")) or real(spots, **kw))
    code, body = b.set_zones({"zones": [R1_ZONE], "session": b.session})
    assert code == 200 and body["zones"] == 1
    assert b.zones[0]["poly"] == [tuple(p) for p in R1_ZONE["poly"]]
    assert drawn and drawn[-1][0]["drawn"] == keepout.grown(R1_ZONE["poly"])
    m = real([], zones=drawn[-1])
    assert _cell(m, 4.956 + 0.045, 2.0) == 100 and _cell(m, *R1_POINTS[1]) == 100


def test_the_bridge_takes_off_only_the_margin_for_a_robot_beside_a_zone(bridge_mod, monkeypatch):
    b = bridge_mod.Bridge()
    b.slam_state = "ready"
    b.mask_phase = (0.049, 0.049)
    monkeypatch.setattr(b, "_pose_in", lambda frame: {"x_m": 3.5, "y_m": 1.03 - 0.02, "age_s": 0.1})
    b.zones_fresh = set()
    code, body = b.set_zones({"zones": [_rect(3.0, 1.03, 4.0, 2.0, "z")], "session": b.session})
    # posted with the robot 0.02 m away: D4's "arrives within LEAVE_M" suspends it, as before
    assert code == 200 and body["suspended"] == ["z"]
    monkeypatch.setattr(b, "_pose_in", lambda frame: {"x_m": 3.5, "y_m": 1.03 - 0.40, "age_s": 0.1})
    b._watch_zones()                                   # clear: drawn again, grown
    assert b.zones_suspended == set() and b.zones_bare == set()
    monkeypatch.setattr(b, "_pose_in", lambda frame: {"x_m": 3.5, "y_m": 1.03 - 0.02, "age_s": 0.1})
    b._watch_zones()                                   # into the margin: only the margin comes off
    assert b.zones_suspended == set() and b.zones_bare == {"z"}
    assert b._nogo_view()["margin_off"] == ["z"] and b._nogo_view()["suspended"] == []


@pytest.mark.parametrize("seed", range(3))
def test_a_mm_of_jitter_on_a_cell_boundary_never_flaps_the_margin(seed):
    """Review round 4: a robot standing still, its pose jittering 1 mm
    either side of a cell boundary (or of a corner, both axes), settles within
    four ticks and then never changes its margins (round 4 saw a flip every
    tick; settling can take two steps: a second zone's margin, or a zone that
    D4 then leaves out whole)."""
    rng = random.Random(seed)
    flaps = 0
    for _ in range(300):
        zs = _zones(*[_rotated(rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), rng.uniform(0.2, 0.8),
                               rng.uniform(0.2, 0.8), rng.choice([0.0, rng.uniform(-180, 180)]), f"z{k}")
                      for k in range(rng.randint(1, 3))])
        phase = (rng.uniform(0, RES), rng.uniform(0, RES))
        bx = phase[0] + RES * round((rng.uniform(-0.8, 0.8) - phase[0]) / RES)   # on a cell boundary
        by_ = phase[1] + RES * round((rng.uniform(-0.8, 0.8) - phase[1]) / RES)
        corner = rng.random() < 0.5             # review round 5: both axes on a boundary too
        y0 = by_ if corner else rng.uniform(-0.8, 0.8)
        off, susp, changes, last = set(), set(), 0, None
        for t in range(20):
            x = bx + (0.001 if t % 2 else -0.001)
            y = y0 + ((0.001 if t % 4 < 2 else -0.001) if corner else 0.0)
            off, susp, _ = _step(zs, off, susp, (x, y), phase)
            if t >= 4 and off != last:
                changes += 1
            last = set(off)
        flaps += changes > 0
    assert flaps == 0
