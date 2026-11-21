# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.197 V8: a robot the drawn no-go zones box (or leave only a diagonal way
out of) gets a VENT -- the fewest cells undrawn that open a 4-connected way
out -- instead of D4 leaving a whole zone out until it is 0.30 m clear.

3.193's boxed rule switched a zone off whole all along a slanted edge and at
corners; the Rover's zones are rotated by its start heading, so every edge is
slanted against the costmap's lattice.
"""

import importlib.util
import json
import math
import random
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "service/slam/src/picar_bridge"))
from picar_bridge import keepout  # noqa: E402

# review: by path, under its own name (four other evaluations have a sweep.py)
_spec = importlib.util.spec_from_file_location("sweep3197", ROOT / "evaluations/nogo-3197/sweep.py")
S = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(S)
from tests.test_review_376h1 import bridge_mod  # noqa: E402,F401  (the fixture: bridge.py on stubs)

RES = keepout.RES_M


def _dev_step(zs, pose, phase):
    """dev's (3.195's) step: margins, then suspend() with its boxed rule."""
    off = keepout.margins_off(zs, set(), set(), pose, phase)
    return keepout.suspend(keepout.bare(zs, off), set(), pose, phase=phase)


def _boxed_cases(heading=45.0, phase=(0.013, 0.037), want=20, seed=7):
    """Positions where dev's step leaves the zone out whole: robots from
    0.05 m inside a slanted zone to just outside its drawn edge."""
    rng = random.Random(seed)
    zs = S.zones_of(S.rotated(0.3, -0.2, 0.9, 0.7, heading, "z"))
    out = []
    while len(out) < want:
        x, y = rng.uniform(-0.6, 1.2), rng.uniform(-1.1, 0.7)
        d = keepout.zone_distance(zs[0], x, y)
        if -keepout.SUSPEND_DEPTH_M <= d <= keepout.GROW_M + 0.04 and _dev_step(zs, (x, y), phase):
            out.append((x, y))
    return zs, out


# ---------- mask(holes=) ----------

def test_holes_are_left_unknown_and_spots_still_drawn_over_them():
    zs = S.zones_of(S.rotated(0.0, 0.0, 1.0, 1.0, 0.0, "z"))
    phase = (0.013, 0.037)
    full = keepout.mask([], zones=zs, phase=phase)
    hole = keepout._cell_of(0.0, 0.0, phase)
    m = keepout.mask([], zones=zs, phase=phase, holes={hole})
    S.PH = phase
    assert S.lethal(full, *hole) and not S.lethal(m, *hole)
    assert sum(v == 100 for v in full["data"]) - sum(v == 100 for v in m["data"]) == 1
    spot = [{"x_m": 0.0, "y_m": 0.0, "r_m": 0.1, "until": 1e12}]
    assert S.lethal(keepout.mask(spot, zones=zs, phase=phase, holes={hole}), *hole)
    assert keepout.mask([], zones=zs, phase=phase, holes=()) == full


# ---------- the step ----------

@pytest.mark.parametrize("heading", [15.0, 30.0, 45.0, 60.0])
def test_a_robot_dev_switched_a_zone_off_for_gets_a_vent_instead(heading):
    """Red on dev (no keepout.step): every position where dev's step leaves
    the zone out whole now keeps it drawn, the robot plans out, the vent is
    small and near, and the zone's body away from the robot stays lethal."""
    phase = (0.013, 0.037)
    zs, cases = _boxed_cases(heading, phase)
    for x, y in cases:
        st = keepout.step(zs, {}, (x, y), phase)
        assert st["suspended"] == set(), (x, y)
        assert st["vent"] and len(st["vent"]) <= 10
        m = S.drawn_mask(zs, st, phase)
        assert S.plans_out(m, x, y, phase), (x, y)
        n, far = S.vent_far(zs, st, (x, y), phase)
        assert far <= S.FAR_M
        bad, tested = S.zone_points_lethal(zs, st, phase, (x, y), random.Random(1))
        assert bad == 0 and tested > 0


def test_a_diagonal_way_out_between_two_zones_is_vented():
    """Dev's hole: two zones 9 cm apart at 30 degrees leave the robot a free
    4-neighbour (3.193 E6 holds) but only a diagonal line of free cells, which
    a 4-connected planner cannot use. The vent opens a 4-connected way."""
    h, phase = 30.0, (0.049, 0.001)
    c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
    w1, gap = 0.6, 0.092
    off = w1 / 2 + gap + 0.3
    zs = S.zones_of(S.rotated(0, 0, w1, 0.8, h, "a"), S.rotated(c * off, s * off, 0.6, 0.8, h, "b"))
    u, v = w1 / 2 + gap / 2, -0.13
    x, y = c * u - s * v, s * u + c * v
    off_ = keepout.margins_off(zs, set(), set(), (x, y), phase)
    dev_susp = keepout.suspend(keepout.bare(zs, off_), set(), (x, y), phase=phase)
    dev = {"suspended": dev_susp, "bare": off_ - dev_susp, "vent": None}
    assert not S.plans_out(S.drawn_mask(zs, dev, phase), x, y, phase)       # dev: stuck
    st = keepout.step(zs, {}, (x, y), phase)
    assert st["suspended"] == set() and st["vent"]
    assert S.plans_out(S.drawn_mask(zs, st, phase), x, y, phase)


def test_the_vent_closes_once_the_robot_is_out_and_never_flaps():
    phase = (0.013, 0.037)
    zs, cases = _boxed_cases(45.0, phase, want=1, seed=3)
    x, y = cases[0]
    st = keepout.step(zs, {}, (x, y), phase)
    assert st["vent"]
    # walk straight away from the zone's centre: the vent closes on the way
    cx = sum(p[0] for p in zs[0]["poly"]) / 4
    cy = sum(p[1] for p in zs[0]["poly"]) / 4
    ux, uy = x - cx, y - cy
    L = math.hypot(ux, uy)
    closed_at = None
    for k in range(1, 40):
        p = (x + ux / L * 0.01 * k, y + uy / L * 0.01 * k)
        st = keepout.step(zs, st, p, phase)
        assert st["suspended"] == set()
        assert S.plans_out(S.drawn_mask(zs, st, phase), *p, phase)
        if not st["vent"] and closed_at is None:
            closed_at = k
    assert closed_at is not None and closed_at < 30
    # a mm of jitter in place never opens and closes it tick after tick
    st, changes, last = keepout.step(zs, {}, (x, y), phase), 0, None
    for t in range(20):
        st = keepout.step(zs, st, (x + (0.001 if t % 2 else -0.001), y), phase)
        if t >= 4 and st["vent"] != last:
            changes += 1
        last = st["vent"]
    assert changes == 0


def test_a_new_lattice_drops_the_old_vent():
    phase = (0.013, 0.037)
    zs, cases = _boxed_cases(30.0, phase, want=1, seed=5)
    st = keepout.step(zs, {}, cases[0], phase)
    assert st["vent"] and st["vent_phase"] == phase
    st2 = keepout.step(zs, st, cases[0], (0.031, 0.022))
    assert st2["vent_phase"] == (0.031, 0.022)
    assert S.plans_out(S.drawn_mask(zs, st2, (0.031, 0.022)), *cases[0], (0.031, 0.022))


def test_a_vent_survives_a_phase_move_with_no_pose():
    """Review: the bridge redraws on a new lattice even with no pose; the
    vent's cells move onto it (every new cell they overlap) and the robot can
    still plan out of the mask published then."""
    phase, new = (0.013, 0.037), (0.031, 0.022)
    zs, cases = _boxed_cases(45.0, phase, want=5, seed=9)
    for x, y in cases:
        st = keepout.step(zs, {}, (x, y), phase)
        st2 = keepout.step(zs, st, None, new)
        assert st2["vent"] and st2["vent_phase"] == new and st2["suspended"] == set()
        assert S.plans_out(S.drawn_mask(zs, st2, new), x, y, new), (x, y)
        st3 = keepout.step(zs, st2, (x, y), new)        # the next pose bounds it again
        assert len(st3["vent"]) <= keepout.VENT_MAX_CELLS
        assert keepout._far(st3["vent"], x, y, new) <= keepout.VENT_FAR_M


def test_a_vent_left_behind_along_an_edge_is_re_derived():
    """Review: a robot sliding along a slanted edge never keeps a vent that
    reaches farther than VENT_FAR_M from it."""
    phase = (0.013, 0.037)
    zs = S.zones_of(S.rotated(0.0, 0.0, 2.0, 0.8, 30.0, "z"))
    c, s_ = math.cos(math.radians(30)), math.sin(math.radians(30))
    st = {}
    for k in range(120):
        u, v = -0.9 + 0.015 * k, -0.4 + 0.03            # 3 cm inside the long edge
        x, y = c * u - s_ * v, s_ * u + c * v
        st = keepout.step(zs, st, (x, y), phase)
        assert st["suspended"] == set()
        assert keepout._far(st["vent"], x, y, phase) <= keepout.VENT_FAR_M + 1e-9
        assert S.plans_out(S.drawn_mask(zs, st, phase), x, y, phase)


def test_a_re_post_of_the_same_zones_keeps_the_vent(bridge_mod, monkeypatch):
    phase = (0.013, 0.037)
    zs, cases = _boxed_cases(45.0, phase, want=1)
    b = bridge_mod.Bridge()
    b.slam_state = "ready"
    b.mask_phase = phase
    zone = {"id": "z", "poly": [list(p) for p in zs[0]["poly"]]}
    monkeypatch.setattr(b, "_pose_in", lambda frame: {"x_m": 3.0, "y_m": 3.0, "age_s": 0.1})
    b.set_zones({"zones": [zone], "session": b.session})
    monkeypatch.setattr(b, "_pose_in", lambda frame: {"x_m": cases[0][0], "y_m": cases[0][1], "age_s": 0.1})
    b._watch_zones()
    vent = b.zones_vent
    assert vent and b.zone_events["vents"] == 1
    # a re-post with the pose unknown (with a fresh pose, 3.161's "arrives
    # within LEAVE_M" rule leaves the zone out whole, as before)
    monkeypatch.setattr(b, "_pose_in", lambda frame: None)
    b.set_zones({"zones": [zone], "session": b.session})
    assert b.zones_vent == vent and b.zone_events["vents"] == 1
    b.set_zones({"zones": [{**zone, "id": "y"}], "session": b.session})    # a changed set
    assert b.zones_vent == frozenset()


def test_no_small_vent_falls_back_to_d4s_whole_zone_rule(monkeypatch):
    phase = (0.013, 0.037)
    zs, cases = _boxed_cases(45.0, phase, want=3)
    monkeypatch.setattr(keepout, "VENT_MAX_CELLS", 0)
    for x, y in cases:
        st = keepout.step(zs, {}, (x, y), phase)
        assert st["suspended"] == {"z"} and not st["vent"]


def test_depth_fresh_and_leave_rules_are_unchanged():
    zs = S.zones_of(S.rotated(0.0, 0.0, 1.0, 1.0, 30.0, "z"))
    assert keepout.step(zs, {}, (0.0, 0.0))["suspended"] == {"z"}               # deep inside
    st = keepout.step(zs, {}, (0.0, 0.0))
    assert keepout.step(zs, st, (0.0, 0.70))["suspended"] == {"z"}             # not 0.30 m clear
    assert keepout.step(zs, st, (0.0, 1.10))["suspended"] == set()             # clear
    assert keepout.step(zs, {}, (0.0, 0.75), fresh={"z"})["suspended"] == {"z"}  # arrives near
    assert keepout.step(zs, {}, None) == {"suspended": set(), "bare": set(), "vent": frozenset(),
                                          "vent_phase": (0.0, 0.0)}


def test_suspend_keeps_its_boxed_rule_by_default():
    phase = (0.013, 0.037)
    zs, cases = _boxed_cases(45.0, phase, want=3)
    for x, y in cases:
        off = keepout.margins_off(zs, set(), set(), (x, y), phase)
        assert keepout.suspend(keepout.bare(zs, off), set(), (x, y), phase=phase) == {"z"}
        assert keepout.suspend(keepout.bare(zs, off), set(), (x, y), phase=phase, boxed=False) == set()


@pytest.mark.parametrize("heading", [0.0, 30.0, 45.0])
def test_a_small_sweep_never_switches_a_zone_off_and_always_plans_out(heading):
    """S1 in small: 400 robots round one zone, 0.05 m inside to 0.04 m outside
    its drawn edge, on two phases."""
    rng = random.Random(int(heading) + 11)
    for phase in ((0.0, 0.0), (0.031, 0.022)):
        zs = S.zones_of(S.rotated(0.2, 0.1, 0.8, 0.6, heading + rng.uniform(-2, 2), "z"))
        got = 0
        while got < 200:
            x, y = rng.uniform(-0.4, 0.8), rng.uniform(-0.4, 0.6)
            d = keepout.zone_distance(zs[0], x, y)
            if not -keepout.SUSPEND_DEPTH_M <= d <= keepout.GROW_M + 0.04:
                continue
            got += 1
            st = keepout.step(zs, {}, (x, y), phase)
            assert st["suspended"] == set(), (x, y, phase)
            assert S.plans_out(S.drawn_mask(zs, st, phase), x, y, phase), (x, y, phase)


# ---------- the bridge ----------

def test_the_bridge_vents_instead_of_switching_the_zone_off(bridge_mod, monkeypatch):
    phase = (0.013, 0.037)
    zs, cases = _boxed_cases(45.0, phase, want=1)
    x, y = cases[0]
    b = bridge_mod.Bridge()
    b.slam_state = "ready"
    b.mask_phase = phase
    holes = []
    real = bridge_mod.keepout.mask
    monkeypatch.setattr(bridge_mod.keepout, "mask",
                        lambda spots, **kw: holes.append(kw.get("holes")) or real(spots, **kw))
    monkeypatch.setattr(b, "_pose_in", lambda frame: {"x_m": 3.0, "y_m": 3.0, "age_s": 0.1})
    zone = {"id": "z", "poly": [list(p) for p in zs[0]["poly"]]}
    assert b.set_zones({"zones": [zone], "session": b.session})[0] == 200   # posted far away
    monkeypatch.setattr(b, "_pose_in", lambda frame: {"x_m": x, "y_m": y, "age_s": 0.1})
    b._watch_zones()
    assert b.zones_suspended == set() and b.zones_vent
    assert holes[-1] == b.zones_vent
    v = b._nogo_view()
    assert v["suspended"] == [] and v["vent_cells"] == len(b.zones_vent) and v["vents"] == 1
    # a new zone set starts unvented; a phase move drops the cells too
    b.set_zones({"zones": [zone], "session": b.session})
    assert b.zone_events["vents"] >= 1
    monkeypatch.setattr(b, "_pose_in", lambda frame: {"x_m": 3.0, "y_m": 3.0, "age_s": 0.1})
    b._watch_zones()
    assert b.zones_vent == frozenset() and b._nogo_view()["vent_cells"] == 0


# ---------- the sweeps (3.197.B), pinned ----------

EVAL = ROOT / "evaluations/nogo-3197"


def test_the_sweeps_met_v1_to_v7():
    dev = json.loads((EVAL / "sweep-dev.json").read_text())
    new = json.loads((EVAL / "sweep-3197.json").read_text())
    assert dev["code"] == "dev" and new["code"] == "3.197"
    assert dev["S1"]["boxing_suspensions"] == 2913 and dev["S5_two_zones"]["cannot_plan_out"] == 1122
    assert new["S1"]["positions"] == 12000 and new["S1"]["boxing_suspensions"] == 0          # V1
    assert new["S1b_3195_sampling"]["boxing_suspensions"] == 0                                # V2
    for k in ("S1", "S1b_3195_sampling", "S5_two_zones", "S6_walks"):                         # V3
        assert new[k]["cannot_plan_out"] == 0, k
    assert new["S5_two_zones"]["boxing_suspensions"] <= 510                                   # V4
    assert new["S6_walks"]["steps"] == 10000 and new["S6_walks"]["boxing_suspensions"] == 0   # V5
    assert new["S1"]["vent_cells_max"] <= 10 and new["S1"]["vent_far_m_max"] <= 0.60          # V6
    assert new["S1"]["zone_points_unlethal"] == 0 and new["S1"]["zone_points_tested"] > 10000
    assert new["S7_jitter"]["flaps"] == 0                                                     # V7


def test_the_start_turn_hook_refuses_a_value_that_is_not_a_number(monkeypatch):
    from sim.maps import build_world
    monkeypatch.setenv("SIM_START_TURN_DEG", "30deg")
    with pytest.raises(ValueError, match="SIM_START_TURN_DEG"):
        build_world("scaled_house")


def test_the_start_turn_hook_turns_the_simulated_robot(monkeypatch):
    from sim.maps import build_world
    base = build_world("scaled_house").heading_deg
    monkeypatch.setenv("SIM_START_TURN_DEG", "30")
    assert abs((build_world("scaled_house").heading_deg - base) % 360 - 30) < 1e-9
    monkeypatch.delenv("SIM_START_TURN_DEG")
    assert build_world("scaled_house").heading_deg == base


def test_the_live_runs_met_l1_to_l5():
    s = json.loads((EVAL / "summary.json").read_text())
    c = s["counted"]
    assert c["runs"] == 10 and c["invalid_runs"] == [] and c["peak_load"] < 4.0
    assert s["by_prefix"]["a"]["runs"] == 5 and s["by_prefix"]["r"]["runs"] == 5
    assert all(r["start_turn_deg"] == "30" for r in s["runs"]["r"])
    assert c["L1_entries"] == 0
    assert c["L2_min_m"] >= 0.010
    assert c["L3_found"] == 10 and c["L3_median_s"] <= 79.5 and c["L3_goals_in_zone"] == 0
    assert c["L3_docked"] == 10
    assert c["L4_both_events"] == 10 and min(c["L4_restore_clear_m"]) >= 0.30
    assert max(c["L4_left_after_s"]) <= 30 and set(c["L4_final"]) == {"cancelled"}
    assert c["L5_whole_suspensions"] == [0] * 10
