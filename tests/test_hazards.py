# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.231.C1 / 3.237 (amendment 2): `brain/hazards.py`, and explore taking
hazards as short-lived zones beside a person's. Off by default."""

import math

import pytest

from brain import hazards as hz


POSE = {"usable": True, "x_m": 2.0, "y_m": 3.0, "heading_deg": 90.0}   # facing +x (east)


def test_a_detection_lands_where_the_camera_saw_it():
    frame = {"detections": [{"label": "sock", "bearing_deg": 0.0, "distance_m": 1.0},
                            {"label": "sofa", "bearing_deg": 0.0, "distance_m": 2.0},
                            {"label": "shoe", "bearing_deg": 10.0, "distance_m": None}]}
    pts = hz.detection_points(frame, POSE, pan_deg=0.0)
    assert len(pts) == 1 and pts[0][0] == "sock"
    assert pts[0][1] == pytest.approx(3.0) and pts[0][2] == pytest.approx(3.0)
    # Panned 90 deg right (clockwise): south, +y in the house frame.
    (_, x, y), = hz.detection_points(frame, POSE, pan_deg=90.0)
    assert (x, y) == (pytest.approx(2.0), pytest.approx(4.0))


def _scan(ranges):
    return {"usable": True, "angle_min_deg": -180.0, "angle_increment_deg": 1.0,
            "ranges_m": ranges}


def _band(zones):
    return {"cols": len(zones), "fov_deg": 73.0, "origin_x_cm": 6.5, "zones": zones}


def test_a_band_point_the_lidar_explains_is_not_low():
    """A band return with a lidar return beside it is a wall or a tall
    thing; one with none is under the lidar's plane."""
    zone = {"status": "range", "distance_cm": 50.0, "bearing_deg": 0.0}
    wall = [math.inf] * 360
    wall[180] = 0.065 + 0.50 - hz.LIDAR_X_M          # the same point, from the lidar
    assert hz.band_points(_band([zone]), _scan(wall), POSE) == []
    (x, y), = hz.band_points(_band([zone]), _scan([math.inf] * 360), POSE)
    assert (x, y) == (pytest.approx(2.565), pytest.approx(3.0))


def test_an_unusable_or_empty_zone_marks_nothing():
    """M3: a zone the band could not read, or read as nothing in reach, is
    never a mark -- and no scan (cannot tell low from tall) marks nothing."""
    zones = [{"status": "unusable", "distance_cm": None},
             {"status": "no_target", "distance_cm": None},
             {"status": "range", "distance_cm": None}]
    assert hz.band_points(_band(zones), _scan([math.inf] * 360), POSE) == []
    zone = [{"status": "range", "distance_cm": 50.0, "bearing_deg": 0.0}]
    assert hz.band_points(_band(zone), {"usable": False}, POSE) == []
    assert hz.band_points(_band(zone), None, POSE) == []
    assert hz.band_points(None, _scan([math.inf] * 360), POSE) == []


def test_the_list_merges_refreshes_expires_and_caps():
    h = hz.HazardList(ttl_s=10.0, cap=3)
    assert h.add(0.0, 0.0, 0.0, "camera", "sock") is True
    rev = h.rev
    assert h.add(0.1, 0.1, 5.0, "camera", "sock") is False and h.rev == rev   # refreshed
    assert len(h.zones(14.0)) == 1                   # seen at 5: alive at 14
    assert h.zones(15.5) == [] and h.rev == rev + 1  # expired: the set changed
    for i in range(4):
        h.add(float(i), 0.0, 20.0 + i, "floor_band")
    assert len(h) == 3 and {z["x0"] for z in h.zones(23.0)} == {1 - hz.HALF_M, 2 - hz.HALF_M,
                                                                 3 - hz.HALF_M}
    z = h.zones(23.0)[0]
    assert z["x1"] - z["x0"] == pytest.approx(2 * hz.HALF_M) and z["id"].startswith("hazard-")
    assert h.added == {"camera": 1, "floor_band": 4}


def _explore(house="complex_house_hazards"):
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld
    from tests.test_explore import FakeNav
    world = build_world(house)
    robot = MockRobot(world, render=False)
    runner = MissionRunner(robot, policy="explore", job="survey", navigator=FakeNav(robot),
                           world=MockWorld(world), clock=lambda: world.sim_time,
                           idle=robot.pass_time, mission_time_limit_s=120)
    runner.start()
    return runner, world


def test_hazards_off_change_nothing(monkeypatch):
    """Off by default: the list is None and is never fed."""
    from brain.explore import ExploreAgent
    called = []
    monkeypatch.setattr(ExploreAgent, "_update_hazards", lambda *a: called.append(1))
    runner, _ = _explore()
    for _ in range(20):
        runner.tick()
    assert runner.agent.hazards is None and called == [] and runner.agent.zones == []


def test_hazards_on_become_zones_beside_a_persons():
    """On: a camera sighting of a hazard class becomes a zone; a person's
    zone stays; when the hazard expires, only the person's is left."""
    runner, world = _explore()
    agent = runner.agent
    person = {"id": "z1", "x0": 0.0, "x1": 0.3, "y0": 0.0, "y1": 0.3}
    agent.set_zones([person])
    agent.hazards = hz.HazardList(ttl_s=30.0)
    agent.hazard_sources = ("camera", "floor_band")
    for _ in range(40):
        runner.tick()
    ids = [z["id"] for z in agent.zones]
    assert "z1" in ids and any(i.startswith("hazard-") for i in ids), ids
    assert agent.hazards.added["camera"] + agent.hazards.added["floor_band"] > 0
    agent.hazard_sources = ()                   # nothing seen again
    world.sim_time = agent.clock() + 31.0
    agent._update_hazards({"detections": []}, agent.clock())
    assert [z["id"] for z in agent.zones if not z["id"].startswith("hazard-")] == ["z1"]


def test_a_grazing_wall_and_a_point_behind_a_return_are_not_low():
    """Review: a band ray meeting a wall at a shallow angle lands between
    1-degree beams; the lidar's range ALONG the point's bearing still stops
    at the wall, so it is explained. So is a point behind a nearer return."""
    zone = {"status": "range", "distance_cm": 140.0, "bearing_deg": 30.0}
    # A wall the band's point sits on, 1.2 m+ out at bearing ~29 deg from the lidar.
    lx = 0.04
    px, py = 0.065 + 1.40 * math.cos(math.radians(30)), 1.40 * math.sin(math.radians(30))
    rho = math.hypot(px - lx, py)
    beta = math.degrees(math.atan2(py, px - lx))
    ranges = [math.inf] * 360
    for k in (math.floor(beta), math.floor(beta) + 1):
        ranges[180 + k] = rho + 0.03          # the wall, read a little long (overshoot)
    assert hz.band_points(_band([zone]), _scan(ranges), POSE) == []
    near = [math.inf] * 360
    for k in (math.floor(beta), math.floor(beta) + 1):
        near[180 + k] = 0.6                   # something tall in front of the point
    assert hz.band_points(_band([zone]), _scan(near), POSE) == []


def test_the_target_is_never_its_own_hazard():
    """Review: a find for a hazard class must not wall off its target."""
    runner, _ = _explore()
    agent = runner.agent
    agent.memory.target_object = "sock"
    agent.hazards = hz.HazardList()
    agent.hazard_sources = ("camera",)
    frame = {"detections": [{"label": "sock", "bearing_deg": 0.0, "distance_m": 1.0},
                            {"label": "shoe", "bearing_deg": 5.0, "distance_m": 1.2}]}
    agent._update_hazards(frame, agent.clock())
    assert [z["label"] for z in agent.zones] == ["shoe"]


def test_a_reading_it_cannot_use_marks_nothing_and_ends_nothing():
    """Review: steering only -- a malformed detection is logged once and
    marks nothing; the mission goes on."""
    runner, _ = _explore()
    agent = runner.agent
    agent.hazards = hz.HazardList()
    agent.hazard_sources = ("camera",)
    bad = {"detections": [{"label": "sock", "bearing_deg": "left", "distance_m": 1.0}]}
    agent._update_hazards(bad, agent.clock())
    agent._update_hazards(bad, agent.clock())
    assert agent.zones == [] and agent._hazard_failed
    assert sum(e.get("event") == "hazard_error" for e in agent.events) == 1


def test_standing_beside_a_hazard_sends_no_leave_goal():
    """Review: the band marks from 20 cm, so the robot can stand in a
    hazard's grown zone; that zone is left off the mask, never a
    leave_zone goal (a person's zones alone send it out)."""
    runner, world = _explore()
    agent = runner.agent
    pose = agent._pose()
    agent.hazards = hz.HazardList()
    agent.hazards.add(pose["x_m"] + 0.2, pose["y_m"], agent.clock(), "floor_band")
    agent._hazard_zones = agent.hazards.zones(agent.clock())
    agent._hazard_rev = agent.hazards.rev
    agent.set_zones([])
    for _ in range(15):
        runner.tick()
    assert agent.zone_leaves == 0


def test_a_band_point_needs_a_second_look():
    """Bug pass: one band reading off a skewed pose can put a door jamb
    under the lidar; a band point is marked only when a later tick's lands
    within MERGE_M inside CONFIRM_S. The camera's needs no second look."""
    h = hz.HazardList()
    assert h.add(1.0, 1.0, 0.0, "floor_band", confirm=True) is False and len(h) == 0
    assert h.add(1.05, 1.0, 0.0, "floor_band", confirm=True) is False and len(h) == 0  # same tick
    assert h.add(1.05, 1.0, 0.5, "floor_band", confirm=True) is True and len(h) == 1
    assert h.add(3.0, 3.0, 1.0, "floor_band", confirm=True) is False
    assert h.add(3.0, 3.0, 1.0 + hz.CONFIRM_S + 0.1, "floor_band", confirm=True) is False
    assert len(h) == 1                                          # too late: a fresh first look
    assert h.add(5.0, 5.0, 9.0, "camera", "sock") is True


def test_the_band_pan_and_an_unread_beam():
    """Bug pass: the band's own `pan_deg` turns its points; a beam with no
    return (None) is unknown and explains a point, never marks it; a block
    whose `cols` disagrees with its zones marks nothing."""
    zone = {"status": "range", "distance_cm": 50.0, "bearing_deg": 0.0}
    band = dict(_band([zone]), pan_deg=90.0)
    (x, y), = hz.band_points(band, _scan([math.inf] * 360), POSE)
    assert (x, y) == (pytest.approx(2.0 + 0.065), pytest.approx(3.5))
    assert hz.band_points(_band([zone]), _scan([None] * 360), POSE) == []
    assert hz.band_points(dict(_band([zone]), cols=2), _scan([math.inf] * 360), POSE) == []
