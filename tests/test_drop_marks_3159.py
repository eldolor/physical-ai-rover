# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.159.B2 (`docs/plans/ros-alignment/3.159-drop-detection.md`, amendment
3): a drop edge that refuses a nav2 twist is marked into nav2's keepout as
discs at the remembered lip, sent with the wedge spots, and kept past a
mission's end. The real robot server, RosWorld behind it and a fake
bridge, as tests/test_wedge_watch.py's."""

import pytest
from fastapi.testclient import TestClient

import robot.server as server
from robot import drop_guard
from robot import wedge_watch as ww
from robot.safety import SafetyController
from tests.test_ros_goals import world_with
from tests.test_ros_world import FakeTruth
from tests.test_wedge_watch import CLAMP, KeepoutBridge, wait_for

DROP = "forward clamped: 20.0cm <= 23cm (drop_edge)"


class _Lips:
    """A guard that remembers one lip 50 cm straight ahead."""

    def lips(self, radius_cm=0.0):
        return [(50.0, 0.0)]


@pytest.fixture
def drop_server(monkeypatch):
    monkeypatch.delenv("APP_SHARED_SECRET", raising=False)
    truth = FakeTruth(1.0, 1.0, 90.0)
    bridge = KeepoutBridge(start_truth={"x_m": 1.0, "y_m": 1.0, "heading_deg": 90.0})
    world, _ = world_with(truth=truth, bridge=bridge)
    monkeypatch.setattr(server, "get_world", lambda *a, **kw: world)
    monkeypatch.setattr(server, "WedgeWatch", lambda: ww.WedgeWatch(hold_s=0.2))
    why = {"reason": DROP}
    monkeypatch.setattr(SafetyController, "vet_wheel_velocity",
                        lambda self, l, r: (0.0, 0.0, why["reason"]) if l + r > 0 else (l, r, None))
    monkeypatch.setattr(SafetyController, "_drop_guard", lambda self: _Lips())
    return TestClient(server.create_app()), bridge, why


def _post(client, n=3):
    for _ in range(n):
        client.post("/wheels", json={"left_rad_s": 2.0, "right_rad_s": 2.0},
                    headers={"x-driver": "ros"})


def test_a_drop_refusal_during_a_goal_marks_the_lip(drop_server):
    client, bridge, _ = drop_server
    _post(client)
    assert not wait_for(lambda: bridge.keepout, timeout=0.3), "no goal: nothing marked"
    import time
    time.sleep(1.05)                          # past the once-a-second mark limit
    assert client.post("/world/goal", json={"x_m": 3.0, "y_m": 1.0}).json()["accepted"]
    _post(client)
    assert wait_for(lambda: bridge.keepout)
    # At the session's start facing ROS +x: the lip 0.5 m straight ahead,
    # on the 0.15 m grid.
    (s,) = bridge.keepout
    assert (s["x_m"], s["y_m"]) == pytest.approx((0.45, 0.0), abs=0.08)
    assert s["r_m"] == drop_guard.MARK_R_M and s["ttl_s"] > 3000


def test_a_drop_already_marked_is_not_sent_again(drop_server):
    client, bridge, _ = drop_server
    assert client.post("/world/goal", json={"x_m": 3.0, "y_m": 1.0}).json()["accepted"]
    _post(client)
    assert wait_for(lambda: bridge.keepout_posts == 1)
    _post(client, 10)
    assert not wait_for(lambda: bridge.keepout_posts > 1, timeout=0.5)


def test_a_wedge_mark_carries_the_drop_and_a_missions_end_keeps_it(drop_server):
    client, bridge, why = drop_server
    assert client.post("/world/goal", json={"x_m": 3.0, "y_m": 1.0}).json()["accepted"]
    _post(client)
    assert wait_for(lambda: bridge.keepout_posts == 1)
    why["reason"] = CLAMP                     # now a plain wedge
    import time
    end = time.monotonic() + 0.6
    while time.monotonic() < end:
        _post(client, 1)
        time.sleep(0.02)
    assert wait_for(lambda: bridge.keepout and len(bridge.keepout) == 2)
    assert sorted(s["r_m"] for s in bridge.keepout) == [ww.SPOT_R_M, drop_guard.MARK_R_M]
    reply = client.delete("/world/keepout").json()
    assert reply["cleared"] == 1 and reply["sent"]
    assert [s["r_m"] for s in bridge.keepout] == [drop_guard.MARK_R_M], (
        "a stairwell stays where it is: its discs outlive the mission")


def test_a_turn_refused_at_an_edge_marks_it_too(drop_server):
    """B2 review: a `drop_pivot_edge` refusal never marked the edge."""
    client, bridge, why = drop_server
    why["reason"] = "pivot clamped (drop_pivot_edge)"
    assert client.post("/world/goal", json={"x_m": 3.0, "y_m": 1.0}).json()["accepted"]
    _post(client)
    assert wait_for(lambda: bridge.keepout)


def test_a_refusal_every_tick_reads_the_pose_once_a_second(drop_server, monkeypatch):
    """B2 review: every 20 Hz refusal started a mark and its pose read."""
    client, bridge, _ = drop_server
    assert client.post("/world/goal", json={"x_m": 3.0, "y_m": 1.0}).json()["accepted"]
    starts = []
    real = server.threading.Thread

    def counted(*a, **k):
        if k.get("name") == "keepout-drop":
            starts.append(1)
        return real(*a, **k)
    monkeypatch.setattr(server.threading, "Thread", counted)
    import time
    t0 = time.monotonic()
    _post(client, 20)
    # One a second at most, however long 20 posts took on a loaded machine.
    assert 1 <= len(starts) <= 1 + int(time.monotonic() - t0)
