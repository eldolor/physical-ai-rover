# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.192 (F7): a person's no-go zone is a wall for reach only. What explore
understands of the map -- frontiers' cells, the room cut, the goal features --
reads the map without the zones; every goal still keeps out of a zone grown
by the robot's half-width (3.161 D3, amended in 3.192).

The map: 4.0 x 1.4 m at 0.05 m, walls round it, floor west of x = 2.0 and
unknown east of it, so one frontier (1.3 m) runs along x = 1.95-2.0.
"""

import pytest

import brain.explore as explore_mod
from brain.explore import ExploreAgent
from brain.goal_policy import goal_policy_for
from brain.memory import MissionMemory
from sim.maps import build_world
from sim.mock_robot import MockRobot
from tests.test_explore import FakeNav
from world import nogo
from world.interface import CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN

RES, W, H = 0.05, 80, 28
FRAME = "map-floor"


def _map():
    cells = []
    for j in range(H):
        for i in range(W):
            if j in (0, H - 1) or i in (0, W - 1):
                cells.append(CELL_OCCUPIED)
            elif i < 40:
                cells.append(CELL_FREE)
            else:
                cells.append(CELL_UNKNOWN)
    return {"usable": True, "width": W, "height": H, "resolution_m": RES,
            "origin_x_m": 0.0, "origin_y_m": 0.0, "cells": cells,
            "map_id": "m1", "map_version": 1}


class _World:
    def __init__(self):
        self.m = _map()

    def get_map(self):
        return self.m

    def get_pose(self):
        return {"usable": True, "x_m": 0.5, "y_m": 0.7, "heading_deg": 0.0,
                "frame_id": FRAME}


def _agent(zones=(), policy=None):
    robot = MockRobot(build_world("scaled_house"), render=False)
    agent = ExploreAgent(robot, MissionMemory(mission="m", target_object="purple elephant"),
                         navigator=FakeNav(robot), clock=lambda: 0.0, world=_World(),
                         goal_policy=policy)
    agent.set_zones([nogo.clean_zone(z) for z in zones])
    return agent


def _frontiers_of(agent, monkeypatch):
    """The frontiers explore's choice works with (whatever map it reads them
    off), and the goal it sends."""
    got = []
    real = explore_mod.find_frontiers

    def spy(*a, **k):
        out = real(*a, **k)
        got.append(out)
        return out
    monkeypatch.setattr(explore_mod, "find_frontiers", spy)
    agent._choose_next(0.0)
    monkeypatch.setattr(explore_mod, "find_frontiers", real)
    assert got, "explore made no frontier search"
    goal = agent._goal and (agent._goal["x_m"], agent._goal["y_m"])
    # The caller's own filter (a frontier wholly inside a zone) runs after.
    return got[-1], goal


def _in_grown(zones, x, y):
    return nogo.zone_at([nogo.clean_zone(z) for z in zones], x, y, nogo.HALF_WIDTH_M) is not None


def test_a_zone_cutting_a_frontier_keeps_its_zone_free_middle_and_size(monkeypatch):
    """F7 (b): a zone over the frontier's south end. On `dev` the frontier was
    read off the masked map: cut short, its middle and goal moved north. Now
    it is the zone-free frontier, its goal outside the grown zone."""
    free, _ = _frontiers_of(_agent(), monkeypatch)
    zone = {"x0": 1.85, "y0": 0.05, "x1": 2.30, "y1": 0.45}
    agent = _agent([zone])
    cut, goal = _frontiers_of(agent, monkeypatch)
    assert len(free) == 1 and len(cut) == 1
    assert cut[0].size_m == pytest.approx(free[0].size_m)
    assert sorted(cut[0].cells) == sorted(free[0].cells)
    assert cut[0].goal == pytest.approx(free[0].goal)
    assert not _in_grown([zone], *cut[0].goal)
    assert goal is not None and not _in_grown([zone], *goal)


def test_a_frontiers_goal_never_lies_in_a_grown_zone(monkeypatch):
    """D3's core, unchanged: the zone-free goal (the reachable cell nearest the
    middle) would lie in this zone; the goal moves to a stop outside it."""
    free, _ = _frontiers_of(_agent(), monkeypatch)
    gx, gy = free[0].goal
    zone = {"x0": gx - 0.25, "y0": gy - 0.25, "x1": gx + 0.25, "y1": gy + 0.25}
    agent = _agent([zone])
    got, goal = _frontiers_of(agent, monkeypatch)
    assert got and all(not _in_grown([zone], *f.goal) for f in got)
    assert goal is None or not _in_grown([zone], *goal)


def test_a_frontier_wholly_inside_a_grown_zone_is_dropped():
    """F7 (c): every frontier cell inside the grown zone -- no goal for it.
    (`dev` drops it too, by masking its cells; this pins the drop.)"""
    zone = {"x0": 1.85, "y0": 0.05, "x1": 2.30, "y1": 1.35}
    agent = _agent([zone])
    agent._choose_next(0.0)
    sent = [e for e in agent.events if e["event"] == "goal_sent"]
    assert not [e for e in sent if e.get("kind") == "frontier"]


def test_the_features_read_the_map_without_the_zones(monkeypatch):
    """F7 (a): a zone over unknown floor 0.5 m past the frontier holds no
    candidate and cuts no frontier. Every pooled candidate's features (goal
    and room features, the shipped `rooms` chooser) equal those with no zone.
    On `dev` the masked cells read as neither unknown nor free (and as walls
    to the room cut), so `unknown_near` and the room features changed."""
    policy = goal_policy_for("rooms")

    def feats(zones):
        agent = _agent(zones, policy=policy)
        got = []
        real = agent._pool

        def spy(*a, **k):
            pool, f = real(*a, **k)
            got.append(sorted((tuple(round(v, 6) for v in c.goal), tuple(round(v, 9) for v in x))
                              for (_, c, _), x in zip(pool, f)))
            return pool, f
        agent._pool = spy
        agent._choose_next(0.0)
        assert got, "no pool was built"
        return got[-1]

    zone = {"x0": 2.55, "y0": 0.40, "x1": 3.00, "y1": 1.00}
    without, with_zone = feats([]), feats([zone])
    assert without, "no candidate"
    assert with_zone == without


def test_with_no_zones_the_choice_reads_one_map():
    """No zones: the floor map is the map itself -- nothing changes."""
    agent = _agent()
    agent._choose_next(0.0)
    assert agent._floor_of is None
    m = agent.world.get_map()
    assert agent._floor(m) is m



# ---------- 3.192.B's sweeps, as recorded (evaluations/nogo-3192/summary.json) ----------

def _summary():
    import json
    from pathlib import Path
    p = Path(__file__).resolve().parents[1] / "evaluations" / "nogo-3192" / "summary.json"
    return json.loads(p.read_text())


def test_f1_rule_chooser_as_recorded_failed_by_one():
    """F1 (Z5 as written, chooser rule): 86 of 96 against 87 -- FAILED by one
    (dev at this base: 82 against 87); median time met."""
    s = _summary()
    f, d = s["find_rule"], s["dev_find_rule"]
    assert (f["found_zones"], f["found_none"], f["n"]) == (86, 87, 96)
    assert f["lost"] == ["procthor:val:26 s0", "procthor:val:39 s0", "procthor:val:44 s1"]
    assert f["time_within_5pct"] and not f["rate_equal"]
    assert (d["found_zones"], d["found_none"]) == (82, 87)


def test_f1_prime_rooms_chooser_as_recorded():
    """F1' (the shipped chooser): find rate equal, 81 of 96 each arm; median
    time +5.08% -- FAILED by 0.08 points (dev: 80 against 81, +18.53%)."""
    s = _summary()
    f, d = s["find_rooms"], s["dev_find_rooms"]
    assert (f["found_zones"], f["found_none"]) == (81, 81) and f["rate_equal"]
    assert f["median_ratio"] == 1.0508 and not f["time_within_5pct"]
    assert (d["found_zones"], d["found_none"], d["median_ratio"]) == (80, 81, 1.1853)


def test_f2_f4_no_goal_in_a_zone_and_no_entry():
    s = _summary()
    for ch in ("rule", "rooms"):
        assert s[f"safety_{ch}"]["in_zone"] == 0
        assert s[f"safety_{ch}"]["in_grown_012"] == 0
        assert s[f"safety_{ch}"]["entries"] == 0
        assert s[f"safety_{ch}"]["goals"] > 2000
    assert s["leave"]["entries_after_leaving"] == 0


def test_f3_survey_coverage_within_a_point_both_choosers():
    s = _summary()
    for ch in ("rule", "rooms"):
        assert s[f"survey_{ch}"]["houses"] == 12 and s[f"survey_{ch}"]["met"]


def test_f5_every_start_in_a_zone_leaves():
    s = _summary()["leave"]
    assert (s["n"], s["left"], s["restored_within_030_of_clearing"]) == (10, 10, 10)


def test_f6_no_zone_arm_is_devs_on_both_choosers():
    s = _summary()
    for ch in ("rule", "rooms"):
        assert s[f"identical_{ch}"] == {"compared": 96, "identical": 96, "differ": []}


def test_f10_goal_loops_as_recorded():
    """F10: rule 0; rooms 5 aborted goals by one zone in one mission -- FAILED
    (bar 3; dev 4 at the same zone, procthor:val:15 s0, which still ends found)."""
    s = _summary()
    assert s["loops_rule"]["worst"] == 0
    assert (s["loops_rooms"]["worst"], s["loops_rooms"]["worst_at"]) == (5, "procthor:val:15 s0 z20")
    assert s["dev_loops_rooms"]["worst"] == 4


def test_f9_live_as_recorded():
    """F9: found, docked, 0 goals in a zone; 1 entry, 2 mm deep along a zone's
    edge (the bridge's cell-centre mask) -- FAILED, recorded."""
    import json
    from pathlib import Path
    d = json.loads((Path(__file__).resolve().parents[1] / "evaluations" / "nogo-3192"
                    / "live.json").read_text())
    m = d["mission"]
    assert m["outcome"] == "found" and d["goals_in_zone"] == [] and d["z6_refused"] == 10
    assert len(m["entries"]) == 1 and m["entries"][0]["zones"] == ["za15d23feaa38"]
    z = next(z for z in d["zones"] if z["id"] == "za15d23feaa38")
    assert 0 < m["entries"][0]["at"][1] - z["y0"] < 0.005


# ---------- 3.192.B amendment 2: the re-measure on dev 974ae9bb (2026-11-21) ----------

def _remeasure(name):
    import json
    from pathlib import Path
    return json.loads((Path(__file__).resolve().parents[1] / "evaluations" / "nogo-3192"
                       / "remeasure" / name).read_text())


def test_remeasure_f1_rule_still_fails_by_one():
    """After 3.193/3.195/3.196 the rule chooser's find rate is unchanged: 86 of
    96 with zones against 87 -- F1 FAILED, the same three losses."""
    f = _remeasure("summary.json")["find_rule"]
    assert (f["found_zones"], f["found_none"], f["n"]) == (86, 87, 96)
    assert f["lost"] == ["procthor:val:26 s0", "procthor:val:39 s0", "procthor:val:44 s1"]
    assert f["median_ratio"] == 1.0 and not f["rate_equal"]


def test_remeasure_f1_prime_rooms_time_still_over_by_008_points():
    """Rooms: 81 against 81 (rate met), median +5.08% -- F1' FAILED on time."""
    f = _remeasure("summary.json")["find_rooms"]
    assert (f["found_zones"], f["found_none"]) == (81, 81) and f["rate_equal"]
    assert f["median_ratio"] == 1.0508 and not f["time_within_5pct"]


def test_remeasure_f10_now_met_and_the_rest_as_before():
    """F10: 0 aborted goals within 0.5 m of a zone in every mission, both
    choosers (3.192: rooms 5). F2/F4/F5/F6 as before."""
    s = _remeasure("summary.json")
    assert s["loops_rule"]["worst"] == 0 and s["loops_rooms"]["worst"] == 0
    for ch in ("rule", "rooms"):
        assert s[f"safety_{ch}"]["in_zone"] == 0 and s[f"safety_{ch}"]["entries"] == 0
        assert s[f"identical_{ch}"]["identical"] == 96
        assert s[f"survey_{ch}"]["houses"] == 12
    assert (s["leave"]["left"], s["leave"]["entries_after_leaving"]) == (10, 0)


def test_remeasure_f9_live_met():
    """F9 live on the bridge with 3.193 + 3.195: found, 0 entries, 0 goals in a zone."""
    d = _remeasure("live-1.json")
    m = d["mission"]
    assert m["outcome"] == "found" and m["entries"] == [] and m["truth_samples"] >= 100
    assert d["goals_in_zone"] == [] and d["z6_refused"] == 10 and d["peak_load"] <= 4
