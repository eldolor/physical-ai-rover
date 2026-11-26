# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.222 K8: the OAK-D driver's geometry on synthetic depth frames whose truth
is known (a ray cast of a floor, boxes and a drop through the same pinhole
model), its freshness rule, and its wiring into HardwareRobot and the
factory. No depthai: the device is a fake source."""

import math
import time

import numpy as np
import pytest

from robot.interface import ZONE_NO_TARGET, ZONE_RANGE, ZONE_UNUSABLE
from robot.oakd_depth import (COLS, DepthCamera, DepthGeometry, Intrinsics, Mount,
                              unusable_blocks)

INTR = Intrinsics(width=640, height=480, fx=432.0, fy=432.0, cx=319.5, cy=239.5)
MOUNT = Mount(height_cm=14.0, pitch_deg=0.0, x_cm=9.5, lateral_cm=0.0)
MIN_Z_CM = 18.0


def render(boxes=(), drop_at_cm=None, drop_depth_cm=20.0, mount=MOUNT, intr=INTR,
           max_z_cm=1000.0):
    """Depth (mm) of a scene in the LENS's frame: x ahead of the lens,
    y to the right, z up from the floor. `boxes`: (x0, x1, y0, y1, height).
    `drop_at_cm`: the floor ends at that x and a lower floor lies
    `drop_depth_cm` below (a stair). Inside MinZ or beyond `max_z_cm` the
    stereo returns nothing (0)."""
    u, v = np.meshgrid(np.arange(intr.width, dtype=np.float64),
                       np.arange(intr.height, dtype=np.float64))
    xn = (u - intr.cx) / intr.fx
    yn = (v - intr.cy) / intr.fy
    th = math.radians(mount.pitch_deg)
    fwd = math.cos(th) - yn * math.sin(th)          # per unit depth
    up = -math.sin(th) - yn * math.cos(th)
    right = xn
    h0, y0 = mount.height_cm, mount.lateral_cm
    best = np.full(xn.shape, np.inf)

    def take(t, ok):
        np.minimum(best, np.where(ok & (t > 0), t, np.inf), out=best)

    with np.errstate(divide="ignore", invalid="ignore"):
        t = -h0 / up                                      # the floor, z = 0
        ok = up < 0
        if drop_at_cm is not None:
            ok &= t * fwd < drop_at_cm
            t2 = -(h0 + drop_depth_cm) / up                # the lower floor
            take(t2, (up < 0) & (t2 * fwd >= drop_at_cm))
            tw = drop_at_cm / fwd                          # the stair's riser
            zw = h0 + tw * up
            take(tw, (fwd > 0) & (zw < 0) & (zw >= -drop_depth_cm))
        take(t, ok)
        for x0, x1, ya, yb, hb in boxes:
            for tx, face_ok in ((x0 / fwd, None), ):
                yy = y0 + tx * right
                zz = h0 + tx * up
                take(tx, (fwd > 0) & (yy >= ya) & (yy <= yb) & (zz >= 0) & (zz <= hb))
            tz = (hb - h0) / up                            # the top
            xx, yy = tz * fwd, y0 + tz * right
            take(tz, (xx >= x0) & (xx <= x1) & (yy >= ya) & (yy <= yb))
            for ys in (ya, yb):                            # the sides
                ts = (ys - y0) / right
                xx, zz = ts * fwd, h0 + ts * up
                take(ts, (xx >= x0) & (xx <= x1) & (zz >= 0) & (zz <= hb))
    z = np.where(np.isfinite(best) & (best >= MIN_Z_CM) & (best <= max_z_cm), best, 0.0)
    return np.round(z * 10.0).astype(np.uint16)


GEOM = DepthGeometry(INTR, MOUNT)


def centre_zones(block, half=4):
    mid = len(block["zones"]) // 2
    return block["zones"][mid - half:mid + half]


def test_bare_floor_reads_no_target_in_both_blocks():
    b = GEOM.blocks(render())
    assert all(z["status"] == ZONE_NO_TARGET for z in b["floor"]["zones"])
    assert all(z["status"] == ZONE_NO_TARGET for z in b["drop"]["zones"])


@pytest.mark.parametrize("d", [30.0, 50.0, 80.0, 110.0, 140.0])
def test_a_5cm_box_reads_its_range_and_bearing(d):
    """K8: within +/-3 cm of the box's face, at its bearing."""
    b = GEOM.blocks(render(boxes=[(d, d + 10.0, -5.0, 5.0, 5.0)]))
    ranged = [z for z in b["floor"]["zones"] if z["status"] == ZONE_RANGE]
    assert ranged, "the box was not seen"
    nearest = min(ranged, key=lambda z: z["distance_cm"])
    assert abs(nearest["distance_cm"] - d) <= 3.0
    assert abs(nearest["bearing_deg"]) <= math.degrees(math.atan(5.0 / d)) + 0.5
    # Nothing ranged outside the box's bearings.
    for i, z in enumerate(b["floor"]["zones"]):
        centre = -36.5 + 73.0 * (i + 0.5) / COLS
        if abs(centre) > math.degrees(math.atan(6.0 / d)) + 1.0:
            assert z["status"] != ZONE_RANGE, (i, z)


def test_a_sock_3cm_reads_at_one_metre():
    b = GEOM.blocks(render(boxes=[(100.0, 112.0, -6.0, 6.0, 3.0)]))
    nearest = min((z for z in centre_zones(b["floor"]) if z["status"] == ZONE_RANGE),
                  key=lambda z: z["distance_cm"])
    assert abs(nearest["distance_cm"] - 100.0) <= 3.0


def test_a_flat_cable_is_not_seen():
    """1 cm is inside the floor tolerance: 3.74 B3 expects depth to miss it."""
    b = GEOM.blocks(render(boxes=[(60.0, 61.5, -30.0, 30.0, 1.0)]))
    assert all(z["status"] != ZONE_RANGE for z in b["floor"]["zones"])


def test_no_return_is_unusable_never_clear():
    """K5/M3: a dropout (every pixel 0) reads unusable in both blocks."""
    b = GEOM.blocks(np.zeros((480, 640), dtype=np.uint16))
    assert all(z["status"] == ZONE_UNUSABLE for z in b["floor"]["zones"])
    assert all(z["status"] == ZONE_UNUSABLE for z in b["drop"]["zones"])


def test_a_thing_inside_minz_is_unusable_not_clear():
    """A box 12 cm from the lens: the stereo returns nothing on it, so the
    columns it fills read unusable, never no-target."""
    b = GEOM.blocks(render(boxes=[(12.0, 30.0, -8.0, 8.0, 20.0)]))
    for z in centre_zones(b["floor"], half=2):
        assert z["status"] == ZONE_UNUSABLE


def test_a_half_dropped_column_is_unusable():
    depth = render()
    depth[:, 300:340] = 0
    b = GEOM.blocks(depth)
    assert all(z["status"] == ZONE_UNUSABLE for z in centre_zones(b["floor"], half=2))


@pytest.mark.parametrize("edge", [40.0, 60.0, 90.0])
def test_a_stair_edge_reads_edge_within_5cm(edge):
    b = GEOM.blocks(render(drop_at_cm=edge))
    for z in centre_zones(b["drop"]):
        assert z["status"] == ZONE_RANGE and z["kind"] == "edge", z
        assert abs(z["distance_cm"] - edge) <= 5.0, z
    # The band does not call a hole a thing.
    assert all(z["status"] != ZONE_RANGE for z in centre_zones(b["floor"]))


def test_a_wall_ends_the_floor_as_a_thing():
    b = GEOM.blocks(render(boxes=[(70.0, 90.0, -200.0, 200.0, 40.0)]))
    for z in centre_zones(b["drop"]):
        assert z["status"] == ZONE_RANGE and z["kind"] == "thing"
        assert abs(z["distance_cm"] - 70.0) <= 5.0


def test_near_cm_is_where_the_floor_is_first_seen_in_the_worst_column():
    """A level lens 14 cm up whose half field down is atan(239.5/432) first
    sees floor 25.25 cm ahead; a column at the field's edge (inner edge
    ~35.7 deg) only at 25.25 / cos 35.7 deg, and memory marks from near_cm
    along every column (review)."""
    f = 14.0 * 432.0 / 239.5
    assert GEOM.near_cm == pytest.approx(f / math.cos(math.radians(36.5 - 73.0 / 96)), abs=0.6)
    assert GEOM.blocks(render())["drop"]["near_cm"] == pytest.approx(GEOM.near_cm, abs=0.1)


def test_a_lone_unseen_last_row_is_not_an_edge():
    """Review: the grazing far end often returns nothing; one row there
    must not paint a sticky edge into the drop memory."""
    depth = render()
    walk = GEOM._order[COLS // 2]
    row = GEOM._rows[walk[walk < GEOM._nrows][-1]]          # the centre's farthest row
    depth[row:row + GEOM._s, :] = 0
    b = GEOM.blocks(depth)
    assert all(z["status"] == ZONE_NO_TARGET for z in centre_zones(b["drop"]))


def test_a_steep_mount_claims_no_floor_past_its_top_ray():
    """Review: pitched 40 deg down, the top ray meets the floor well short of
    100 cm; neither block may claim floor or clear past it."""
    m = Mount(height_cm=14.0, pitch_deg=40.0, x_cm=9.5)
    g = DepthGeometry(INTR, m)
    # The centre's top ray: 40 - atan(239.5/432) = 11.0 deg below level.
    top = 14.0 / math.tan(math.radians(40.0 - math.degrees(math.atan(239.5 / 432.0))))
    # The edge columns lose the top rows to the field, so see less far still.
    assert g.near_cm < g.far_cm <= top + 0.5 and g.max_range_cm == g.far_cm
    b = g.blocks(render(mount=m))
    assert b["drop"]["far_cm"] == pytest.approx(g.far_cm, abs=0.1)
    assert b["floor"]["max_range_cm"] == pytest.approx(g.max_range_cm, abs=0.1)


def test_pitched_mount_still_reads_true_ranges():
    m = Mount(height_cm=14.0, pitch_deg=10.0, x_cm=9.5)
    g = DepthGeometry(INTR, m)
    b = g.blocks(render(boxes=[(60.0, 70.0, -5.0, 5.0, 5.0)], mount=m))
    nearest = min((z for z in b["floor"]["zones"] if z["status"] == ZONE_RANGE),
                  key=lambda z: z["distance_cm"])
    assert abs(nearest["distance_cm"] - 60.0) <= 3.0
    assert all(z["status"] == ZONE_NO_TARGET
               for z in g.blocks(render(mount=m))["drop"]["zones"])


def test_blocks_have_the_simulators_shape():
    from sim import mock_robot

    b = GEOM.blocks(render())
    assert b["floor"]["cols"] == mock_robot.FLOOR_BAND_COLS
    assert b["floor"]["fov_deg"] == mock_robot.FLOOR_BAND_FOV_DEG
    assert b["floor"]["min_range_cm"] == mock_robot.FLOOR_BAND_MIN_CM
    assert b["floor"]["max_range_cm"] == mock_robot.FLOOR_BAND_MAX_CM
    assert b["floor"]["pan_deg"] == 0.0 and b["drop"]["pan_deg"] == 0.0
    assert b["floor"]["origin_x_cm"] == MOUNT.x_cm
    assert b["drop"]["far_cm"] == mock_robot.DROP_FAR_CM
    assert set(unusable_blocks(MOUNT)) == {"floor", "drop"}


def test_the_safety_layer_stops_on_the_band():
    """End to end through floor_clearance(): a 5 cm box 60 cm ahead of the
    lens leaves 60 + 9.5 - 12.65 cm of room before the bumper."""
    from robot.safety import SafetyController

    grid = {"rows": 1, "cols": 1, "zones": [{"status": ZONE_UNUSABLE, "distance_cm": None}],
            **GEOM.blocks(render(boxes=[(60.0, 70.0, -5.0, 5.0, 5.0)]))}
    room, why = SafetyController(robot=object()).floor_clearance(grid)
    assert why == "floor_band"
    assert room == pytest.approx(60.0 + 9.5 - 25.3 / 2.0, abs=3.0)


# -- the device side ----------------------------------------------------------

class FakeSource:
    def __init__(self, frames, intr=INTR, fail_read_after=None):
        self.frames, self.intr = list(frames), intr
        self.reads = 0
        self.closed = False
        self.fail_read_after = fail_read_after

    def intrinsics(self):
        return self.intr

    def read(self):
        self.reads += 1
        if self.fail_read_after is not None and self.reads > self.fail_read_after:
            raise RuntimeError("unplugged")
        if not self.frames:
            return None
        return self.frames.pop(0), time.monotonic()

    def close(self):
        self.closed = True


def _wait(pred, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_fresh_frames_are_published_and_aged():
    cam = DepthCamera(lambda: FakeSource([render()]), mount=MOUNT, stale_s=0.5)
    try:
        assert _wait(lambda: cam.status()["frames"] == 1)
        b = cam.blocks()
        assert b["floor"]["zones"][COLS // 2]["status"] == ZONE_NO_TARGET
        assert 0.0 <= cam.sensor_age_s() < 0.5
    finally:
        cam.close()


def test_a_stale_camera_publishes_unusable_blocks_and_no_age():
    """K5: never omitted (the drop guard stays on), never clear, and its age
    does not shrink the other sensors' clearances."""
    cam = DepthCamera(lambda: FakeSource([render()]), mount=MOUNT, stale_s=0.2)
    try:
        assert _wait(lambda: cam.status()["frames"] == 1)
        assert _wait(lambda: cam.sensor_age_s() is None, timeout=1.0)
        b = cam.blocks()
        assert set(b) == {"floor", "drop"}
        assert all(z["status"] == ZONE_UNUSABLE for blk in b.values() for z in blk["zones"])
        assert b["drop"]["near_cm"] == pytest.approx(GEOM.near_cm, abs=0.1)
    finally:
        cam.close()


def test_never_opened_is_unusable():
    def broken():
        raise RuntimeError("no device")

    cam = DepthCamera(broken, mount=MOUNT, reopen_s=0.05)
    try:
        assert _wait(lambda: cam.status()["failures"] >= 2)
        b = cam.blocks()
        assert all(z["status"] == ZONE_UNUSABLE for blk in b.values() for z in blk["zones"])
        assert cam.status()["last_error"].startswith("open:")
    finally:
        cam.close()


def test_an_unplugged_camera_reconnects():
    sources = []

    def opener():
        src = FakeSource([render()] * 3, fail_read_after=2 if not sources else None)
        sources.append(src)
        return src

    cam = DepthCamera(opener, mount=MOUNT, reopen_s=0.05)
    try:
        assert _wait(lambda: cam.status()["opens"] >= 2)
        assert sources[0].closed
        assert _wait(lambda: cam.status()["frames"] >= 3)
    finally:
        cam.close()


def test_a_bad_frame_is_skipped_not_fatal():
    cam = DepthCamera(lambda: FakeSource([np.zeros((10, 10), np.uint16), render()]),
                      mount=MOUNT)
    try:
        assert _wait(lambda: cam.status()["frames"] == 1)
        assert cam.status()["failures"] == 1
    finally:
        cam.close()


def test_a_device_clock_ahead_does_not_make_a_frame_younger():
    class Ahead(FakeSource):
        def read(self):
            got = super().read()
            return None if got is None else (got[0], got[1] + 5.0)

    cam = DepthCamera(lambda: Ahead([render()]), mount=MOUNT)
    try:
        assert _wait(lambda: cam.status()["frames"] == 1)
        assert cam.sensor_age_s() >= 0.0
    finally:
        cam.close()


def test_a_bad_mount_is_refused_where_it_is_built():
    for bad in ({"height_cm": 0.0}, {"pitch_deg": float("nan")}, {"x_cm": float("inf")},
                {"pitch_deg": 85.0}):
        with pytest.raises(ValueError):
            Mount(**bad)


def test_a_stalled_stream_is_reopened():
    """Review: a device that stays open and sends nothing recovers."""
    cam = DepthCamera(lambda: FakeSource([]), mount=MOUNT, reopen_s=0.05, stall_s=0.1)
    try:
        assert _wait(lambda: cam.status()["opens"] >= 2)
        assert "no frame" in cam.status()["last_error"]
    finally:
        cam.close()


def test_frames_it_cannot_use_in_a_row_reopen_the_source():
    bad = np.zeros((10, 10), np.uint16)
    cam = DepthCamera(lambda: FakeSource([bad] * 20), mount=MOUNT, reopen_s=0.05)
    try:
        assert _wait(lambda: cam.status()["opens"] >= 2)
    finally:
        cam.close()


def test_freshness_is_arrival_and_age_is_capped():
    """Review: a device clock 10 s behind ours neither reads stale for good
    nor ages an on-time frame by seconds."""
    class Behind(FakeSource):
        def read(self):
            got = super().read()
            return None if got is None else (got[0], got[1] - 10.0)

    cam = DepthCamera(lambda: Behind([render()]), mount=MOUNT, stale_s=0.5)
    try:
        assert _wait(lambda: cam.status()["frames"] == 1)
        age = cam.sensor_age_s()
        assert age is not None and age <= 1.0
        assert cam.blocks()["floor"]["zones"][COLS // 2]["status"] == ZONE_NO_TARGET
    finally:
        cam.close()


# -- the wiring ---------------------------------------------------------------

class StubDepth:
    def __init__(self, age=0.07):
        self.age, self.closed = age, False

    def blocks(self):
        return unusable_blocks(MOUNT)

    def sensor_age_s(self):
        return self.age

    def status(self):
        return {"frames": 0}

    def close(self):
        self.closed = True


def _car(**kw):
    from robot.hardware_robot import HardwareRobot
    from sim.fake_esp32 import FakeEsp32
    from sim.maps.scaled_house import build_scaled_world
    from sim.mock_robot import MockRobot

    board = FakeEsp32(MockRobot(build_scaled_world(), render=False))
    return HardwareRobot(board.path, **kw), board


def test_hardware_robot_adds_the_blocks_and_their_age():
    depth = StubDepth()
    robot, board = _car(depth=depth)
    try:
        grid = robot.get_depth_grid()
        assert set(grid) >= {"zones", "floor", "drop"}
        assert robot.sensor_age_s() == pytest.approx(0.07)
        assert robot.depth_status() == {"frames": 0}
    finally:
        robot.close()
        board.close()
    assert depth.closed


def test_hardware_robot_without_depth_is_unchanged():
    from robot.interface import unusable_grid

    robot, board = _car()
    try:
        assert robot.get_depth_grid() == unusable_grid()
        assert robot.depth_status() is None
    finally:
        robot.close()
        board.close()


def test_factory_has_no_depth_unless_asked(monkeypatch):
    from robot import factory

    monkeypatch.delenv("ROBOT_DEPTH", raising=False)
    assert factory._depth({"hardware": {}}) is None
    monkeypatch.setenv("ROBOT_DEPTH", "kinect")
    with pytest.raises(ValueError):
        factory._depth({"hardware": {}})


def test_factory_reads_the_mount(monkeypatch):
    from robot import factory, oakd_depth

    made = {}

    class Cam:
        def __init__(self, source, mount):
            made["mount"] = mount

    monkeypatch.setattr(oakd_depth, "DepthCamera", Cam)
    monkeypatch.setenv("ROBOT_DEPTH", "oakd")
    monkeypatch.setenv("DEPTH_X_CM", "11")
    monkeypatch.delenv("DEPTH_HEIGHT_CM", raising=False)
    factory._depth({"hardware": {"depth_mount": {"height_cm": 14.0, "pitch_deg": 0.0}}})
    assert made["mount"] == Mount(height_cm=14.0, pitch_deg=0.0, x_cm=11.0, lateral_cm=0.0)


def test_a_bad_depth_setting_closes_the_lidar_and_camera(monkeypatch):
    """Review: _depth() raising must not leave the other readers holding
    their devices."""
    from robot import factory

    closed = []

    class Dev:
        def __init__(self, name):
            self.name = name

        def close(self):
            closed.append(self.name)

    monkeypatch.setattr(factory, "_lidar", lambda c: Dev("lidar"))
    monkeypatch.setattr(factory, "_camera", lambda c: Dev("camera"))
    monkeypatch.setenv("ROBOT_SERIAL", "/dev/null-board")
    monkeypatch.setenv("ROBOT_DEPTH", "kinect")
    monkeypatch.delenv("SIM_MOTOR_BOARD", raising=False)
    monkeypatch.delenv("ROBOT_COMMAND_URL", raising=False)
    with pytest.raises(ValueError):
        factory._backend({"mode": "hardware", "hardware": {}})
    assert sorted(closed) == ["camera", "lidar"]


def test_an_empty_yaml_mount_key_is_unset(monkeypatch):
    from robot import factory, oakd_depth

    made = {}
    monkeypatch.setattr(oakd_depth, "DepthCamera", lambda source, mount: made.setdefault("m", mount))
    monkeypatch.setenv("ROBOT_DEPTH", "oakd")
    for env in ("DEPTH_HEIGHT_CM", "DEPTH_PITCH_DEG", "DEPTH_X_CM", "DEPTH_LATERAL_CM"):
        monkeypatch.delenv(env, raising=False)
    factory._depth({"hardware": {"depth_mount": {"height_cm": None}}})
    assert made["m"] == Mount()


def test_the_yaml_mount_is_the_measured_one():
    from robot.factory import load_config

    assert load_config()["hardware"]["depth_mount"] == {
        "height_cm": 14.0, "pitch_deg": 0.0, "x_cm": 9.5, "lateral_cm": 0.0}
