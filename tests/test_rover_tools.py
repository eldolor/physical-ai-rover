# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_rover_tools.py

`PLAN-ros-alignment.md` 3.168: the bring-up tools in `tools/rover/`, dry-run
against the fakes on ptys. The first block pins the section's one hard rule:
no tool can reach the board's input.
"""

import json
import os
import re
import select
import stat
import subprocess
import threading
import time
from pathlib import Path

import pytest

from robot.camera_stream import CameraStream
from robot.lidar_ld19 import LIDAR_YAW_DEG
from sim.fake_camera import FakeCameraSource
from sim.fake_esp32 import FakeEsp32
from sim.fake_lidar import Emitter, open_pty
from tools.rover import board_compare as bc
from tools.rover import camera_check as cc
from tools.rover import dump_firmware as df
from tools.rover import lidar_check as lc
from tools.rover import serial_ro
from tools.rover import serial_route as sr
from tools.rover.serial_ro import BOARD_BAUD, open_readonly

TOOLS = Path(__file__).resolve().parents[1] / "tools" / "rover"
SOURCES = sorted(p for p in TOOLS.iterdir() if p.suffix in (".py", ".sh"))


@pytest.fixture
def board():
    fake = FakeEsp32(bc._StillBody(), firmware="stock", battery_v=12.0, drain_v_per_s=0.0)
    yield fake
    fake.close()


# ---------- criterion 1: nothing reaches the board's input ----------

def test_a_write_on_the_port_fails_in_the_kernel(board):
    fd = open_readonly(board.path, BOARD_BAUD)
    try:
        with pytest.raises(OSError):
            os.write(fd, b'{"T":0}\n')
    finally:
        os.close(fd)
    assert board.lines_in == 0


@pytest.mark.parametrize("path", SOURCES, ids=lambda p: p.name)
def test_no_source_holds_a_command_a_serial_library_or_a_flash_verb(path):
    text = path.read_text()
    assert not re.search(r"T:\s*(?!1001\b)\d", text), "a T: command other than the 1001 frame"
    assert not re.search(r"""["']T["']\s*:""", text), "a command dict"
    assert not re.search(r"^\s*(import serial\b|from serial\b)", text, re.M)
    assert not re.search(r"write[_-]flash|erase[_-](flash|region)|merge[_-]bin|write[_-]mem", text)


def test_every_serial_tool_leaves_the_boards_input_untouched(board, monkeypatch, tmp_path):
    monkeypatch.setattr(sr, "PATTERNS", ())
    sr.scan(0.3, [board.path])
    bc.capture(board.path, 0.3)
    lc.capture(board.path, str(tmp_path / "cap.bin"), 0.3)
    assert board.lines_in == 0


# ---------- criterion 2: L0.3 ----------

class _Lidar:
    """A fake D500 on a pty, in the scene criterion 4 names."""

    def __init__(self, box_deg=0.0, wall_m=1.0):
        self.master, self.slave, self.path = open_pty(None)
        self.emitter = _emitter(box_deg, wall_m)
        self._running = True
        self._t0 = time.monotonic()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._running:
            for pkt in self.emitter.due(time.monotonic() - self._t0):
                while pkt and self._running:      # never drop: a reader is here
                    if select.select([], [self.master], [], 0.1)[1]:
                        try:
                            pkt = pkt[os.write(self.master, pkt):]
                        except (BlockingIOError, OSError):
                            time.sleep(0.001)
            time.sleep(0.005)

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        os.close(self.master)
        os.close(self.slave)


def _emitter(box_deg=0.0, wall_m=1.0):
    def cast_mm(lidar_deg, _t):
        body = (lidar_deg + LIDAR_YAW_DEG + 180.0) % 360.0 - 180.0
        if abs(body - box_deg) <= 4.0:
            return 600
        if abs(body - 90.0) <= 20.0:
            return int(wall_m * 1000)
        return 3000
    return Emitter(cast_mm)


@pytest.fixture
def lidar():
    fake = _Lidar()
    yield fake
    fake.close()


def test_the_route_names_each_port_by_listening(board, lidar, monkeypatch):
    monkeypatch.setattr(sr, "PATTERNS", ())
    report = sr.scan(0.6, [board.path, lidar.path])
    roles = {p["path"]: p["role"] for p in report["ports"]}
    assert roles == {board.path: "board", lidar.path: "lidar"}
    assert report["problems"] == []


def test_the_route_says_what_it_did_not_hear(monkeypatch):
    monkeypatch.setattr(sr, "PATTERNS", ())
    assert sr.scan(0.1)["problems"] == ["no board heard on any port", "no lidar heard on any port"]


def test_a_held_port_is_never_listened_to(board, monkeypatch):
    monkeypatch.setattr(serial_ro, "holders", lambda _p: ["812 python3"])
    with pytest.raises(serial_ro.PortHeld):
        open_readonly(board.path, BOARD_BAUD)
    assert sr.listen(board.path, 0.1)["role"] == "held"


def test_a_port_that_is_not_a_tty_is_unreadable_and_the_scan_goes_on(tmp_path, monkeypatch):
    monkeypatch.setattr(sr, "PATTERNS", ())
    plain = tmp_path / "plain"
    plain.write_text("x")
    report = sr.scan(0.1, [str(plain)])
    assert report["ports"][0]["role"] == "unreadable"


def test_a_port_that_goes_away_is_reported_not_read_as_a_short_capture():
    master, slave, path = open_pty(None)
    fd = open_readonly(path, BOARD_BAUD)
    os.close(slave)
    os.close(master)
    try:
        with pytest.raises(serial_ro.PortLost):
            list(serial_ro.chunks(fd, 2.0))
    finally:
        os.close(fd)


def test_the_route_names_a_port_already_held():
    ports = [{"path": "/dev/ttyTHS1", "role": "board", "held_by": ["812 python3"]},
             {"path": "/dev/ttyUSB0", "role": "lidar", "held_by": []}]
    assert sr.problems(ports) == ["/dev/ttyTHS1 is held by ['812 python3'] (L0.2: stop it first)"]


def _port(path, role, usb=None):
    return {"path": path, "role": role, "usb": usb, "held_by": []}


def test_the_header_uart_gets_a_kernel_name_rule():
    assert sr.rule_for(_port("/dev/ttyTHS1", "board")) == (
        'SUBSYSTEM=="tty", KERNEL=="ttyTHS1", SYMLINK+="picar-board", GROUP="dialout", MODE="0660"')


def test_a_usb_lidar_is_matched_by_serial_or_by_its_port():
    usb = {"vendor": "10c4", "product": "ea60", "serial": "0001", "usb_path": "1-2.3"}
    assert 'ATTRS{serial}=="0001"' in sr.rule_for(_port("/dev/ttyUSB0", "lidar", usb))
    rule = sr.rule_for(_port("/dev/ttyUSB0", "lidar", {**usb, "serial": ""}))
    assert 'KERNELS=="1-2.3"' in rule and 'SYMLINK+="picar-lidar"' in rule


def test_two_bridges_with_one_serial_are_told_apart_by_usb_port():
    a = _port("/dev/ttyUSB0", "board", {"vendor": "10c4", "product": "ea60", "serial": "0001", "usb_path": "1-2.1"})
    b = _port("/dev/ttyUSB1", "lidar", {"vendor": "10c4", "product": "ea60", "serial": "0001", "usb_path": "1-2.3"})
    text = sr.rules_text({"ports": [a, b]})
    assert 'KERNELS=="1-2.1"' in text and 'KERNELS=="1-2.3"' in text and "ATTRS{serial}" not in text


def test_two_channels_of_one_bridge_are_told_apart_by_interface():
    usb = {"vendor": "10c4", "product": "ea70", "serial": "0001", "usb_path": "1-2"}
    a, b = _port("/dev/ttyUSB0", "board", {**usb, "interface": "00"}), _port("/dev/ttyUSB1", "lidar", {**usb, "interface": "01"})
    assert 'ATTRS{bInterfaceNumber}=="01"' in sr.rule_for(b, [a, b])


def test_a_role_heard_on_several_ports_gets_no_rule():
    report = {"ports": [_port("/dev/ttyTHS1", "board"), _port("/dev/ttyTHS2", "board")]}
    assert "SYMLINK" not in sr.rules_text(report)


def test_a_real_port_needs_root(monkeypatch):
    monkeypatch.setattr(serial_ro, "can_see_all", lambda: False)
    monkeypatch.setattr(serial_ro, "holders", lambda _p: [])     # /dev/null has many
    assert serial_ro.holder_may_hide("/dev/null")
    with pytest.raises(serial_ro.PortHeld, match="sudo"):
        open_readonly("/dev/null", BOARD_BAUD)


def test_a_pty_needs_no_root(board, monkeypatch):
    monkeypatch.setattr(serial_ro, "can_see_all", lambda: False)
    assert not serial_ro.holder_may_hide(board.path)


def test_a_pty_gets_no_rule():
    report = {"ports": [_port("/dev/pts/7", "board")]}
    assert "SYMLINK" not in sr.rules_text(report)


def _install(tmp_path, body, *args):
    rules = tmp_path / "99.rules"
    rules.write_text(body)
    sudo = tmp_path / "bin" / "sudo"
    sudo.parent.mkdir()
    marker = tmp_path / "sudo-ran"
    sudo.write_text(f"#!/bin/sh\ntouch {marker}\n")
    sudo.chmod(sudo.stat().st_mode | stat.S_IEXEC)
    env = dict(os.environ, PATH=f"{sudo.parent}:{os.environ['PATH']}")
    r = subprocess.run(["bash", str(TOOLS / "install_udev.sh"), str(rules), *args],
                       capture_output=True, text=True, env=env, timeout=30)
    return r, marker


GOOD_RULES = 'SUBSYSTEM=="tty", KERNEL=="ttyTHS1", SYMLINK+="picar-board"\n'


def test_the_install_script_does_nothing_without_yes(tmp_path):
    r, marker = _install(tmp_path, GOOD_RULES)
    assert r.returncode == 0 and "dry run" in r.stdout
    assert not marker.exists()


def test_the_install_script_refuses_a_rule_that_runs_something(tmp_path):
    r, marker = _install(tmp_path, GOOD_RULES + 'KERNEL=="x", RUN+="/bin/sh"\n', "--yes")
    assert r.returncode == 3 and not marker.exists()


# ---------- criterion 3: L0.5 ----------

@pytest.mark.parametrize("firmware", ("stock", "fork"))
def test_the_fake_compared_with_itself_has_no_differences(firmware):
    ref = bc.reference(firmware, 1.0)
    fake = FakeEsp32(bc._StillBody(), firmware=firmware, battery_v=12.0, drain_v_per_s=0.0)
    try:
        report = bc.compare(bc.capture(fake.path, 1.5), ref)
    finally:
        fake.close()
    assert report["missing_fields"] == [] and report["extra_fields"] == [], report
    assert report["units"] == [] and report["rate_ok"], report
    assert report["v_range"] == [1200, 1200]
    assert len(report["not_exercised"]) == 2 and not report["met"]     # still wheels prove no units
    assert fake.lines_in == 0


REF = {"fields": ["L", "R", "T", "odl", "odr", "v"], "period_p50_ms": 50.0}


def _frame(**over):
    return {"T": 1001, "L": 0, "R": 0, "odl": 0, "odr": 0, "v": 1200, **over}


def _lines(n=40, period=0.05, **over):
    return [(i * period, json.dumps(_frame(**over)).encode()) for i in range(n)]


MOVING = {"L": 0.2, "R": 0.2, "odl": 3, "odr": 3}


def test_a_clean_synthetic_board_is_met():
    assert bc.compare(_lines(**MOVING), REF)["met"]


def test_a_failed_voltage_read_is_counted_apart():
    report = bc.compare(_lines(**MOVING) + _lines(5, v=0, **MOVING), REF)
    assert report["v_zero_frames"] == 5 and report["v_range"] == [1200, 1200] and report["units"] == []


def test_a_missing_field_is_named():
    lines = [(t, json.dumps({k: v for k, v in json.loads(raw).items() if k != "odr"}).encode())
             for t, raw in _lines()]
    report = bc.compare(lines, REF)
    assert report["missing_fields"] == ["odr"] and not report["met"]


def test_v_in_volts_is_named():
    report = bc.compare(_lines(v=12.0), REF)
    assert report["units"] == ["v is not an integer (centivolts)"] and not report["met"]
    report = bc.compare(_lines(v=12), REF)
    assert "outside" in report["units"][0] and not report["met"]


def test_a_third_of_the_rate_is_named():
    report = bc.compare(_lines(period=0.15, **MOVING), REF)
    assert not report["rate_ok"] and not report["met"]


def test_an_extra_field_and_boot_text_are_reported():
    lines = [(0.0, b"rst:0x1 (POWERON_RESET)")] + _lines(extra=1)
    report = bc.compare(lines, REF)
    assert report["extra_fields"] == ["extra"]
    assert report["text_lines"] == ["rst:0x1 (POWERON_RESET)"]


def test_a_reset_is_read_as_its_silence_and_text():
    before = _lines(10)
    after = [(t + 3.0, raw) for t, raw in _lines(10)]
    report = bc.compare(before + [(1.0, b"ets Jul 29 2019")] + after, REF)
    assert report["resets"] == [{"output_silent_s": 2.55, "boot_text": ["ets Jul 29 2019"],
                                 "boot_text_to_first_frame_s": 2.0}]


def test_the_forks_odometers_are_read_at_100_per_cm():
    assert bc.units([_frame(odl=12, odr=0, odlt=1234, odrt=0, ms=5)]) == []
    assert "odlt does not track odl at 100 per cm" in bc.units([_frame(odl=12, odr=0, odlt=12000, odrt=0, ms=5)])


def test_v_change_intervals_are_read():
    frames = [(t, _frame(v=1200 + int(t // 10))) for t in range(45)]
    assert bc.v_changes_s(frames) == [10.0, 10.0, 10.0]


# ---------- criterion 4: L0.6 ----------

GEOMETRY = {"wall_bearing": 90.0, "wall_m": 1.0, "box_m": 0.6}


def _bytes(seconds=31.0, **scene):
    return b"".join(_emitter(**scene).due(seconds))


def test_a_clean_capture_meets_every_bar():
    report = lc.score(_bytes(), **GEOMETRY)
    assert report["met"], report
    assert report["crc_failed"] == 0 and abs(report["scan_rate_hz"] - 10.0) < 0.2
    assert abs(report["box_bearing_deg"]) <= 2.0 and abs(report["wall_error_m"]) <= 0.02


def test_capture_then_score_over_a_real_pty(lidar, tmp_path):
    out = tmp_path / "cap.bin"
    meta = lc.capture(lidar.path, str(out), 2.5)
    assert meta["bytes"] > 0 and (tmp_path / "cap.bin.json").exists()
    report = lc.score(out.read_bytes(), **GEOMETRY)
    assert report["failed"] == ["duration"], report       # all else met, in 2.5 s


def test_a_short_capture_is_not_met():
    assert lc.score(_bytes(seconds=5.0), **GEOMETRY)["failed"] == ["duration"]


def test_a_wall_behind_the_box_does_not_move_its_bearing():
    def cast(lidar_deg, _t):
        body = (lidar_deg + LIDAR_YAW_DEG + 180.0) % 360.0 - 180.0
        return 600 if abs(body - 10.0) <= 4.0 else 700
    data = b"".join(Emitter(cast).due(31.0))
    report = lc.score(data, box_m=0.6)
    assert abs(report["box_bearing_deg"] - 10.0) < 1.0 and "box" in report["failed"]


def test_corruption_over_the_bar_fails_crc():
    data = bytearray(_bytes())
    for i in range(10, len(data), 47 * 100):      # a byte in every 100th packet: 1%
        data[i] ^= 0xFF
    report = lc.score(bytes(data), **GEOMETRY)
    assert report["crc_share"] > 0.001 and report["failed"] == ["crc"]


def test_a_box_off_bearing_fails():
    report = lc.score(_bytes(box_deg=5.0), **GEOMETRY)
    assert report["failed"] == ["box"] and abs(report["box_bearing_deg"] - 5.0) < 1.0


def test_a_wrong_yaw_reads_as_a_box_off_bearing():
    report = lc.score(_bytes(), **GEOMETRY, yaw_deg=LIDAR_YAW_DEG + 180.0)
    assert "box" in report["failed"]


def test_a_wall_off_by_five_cm_fails():
    report = lc.score(_bytes(wall_m=1.05), **GEOMETRY)
    assert report["failed"] == ["wall"] and abs(report["wall_error_m"] - 0.05) < 0.005


def test_a_slow_lidar_fails_the_rate():
    def cast(_a, _t):
        return 3000
    data = b"".join(Emitter(cast, rate_hz=8.0).due(31.0))
    assert "rate" in lc.score(data, **GEOMETRY)["failed"]


def test_missing_geometry_is_not_met():
    report = lc.score(_bytes())
    assert report["failed"] == [] and len(report["not_measured"]) == 2 and not report["met"]


# ---------- criterion 5: L0.4 ----------

@pytest.fixture
def esptool(tmp_path):
    """A stand-in: writes its last argument, `FAKE_ESPTOOL` picks how."""
    tool = tmp_path / "fake-esptool"
    count = tmp_path / "count"
    tool.write_text(f"""#!/usr/bin/env python3
import os, sys
out, size = sys.argv[-1], int(sys.argv[-2], 16)
mode = os.environ.get("FAKE_ESPTOOL", "same")
open({str(count)!r}, "a").write(" ".join(sys.argv[1:]) + "\\n")
n = len(open({str(count)!r}).readlines())
if mode == "fail":
    sys.exit(2)
body = bytes([n % 256]) * size if mode == "differ" else b"\\xa5" * size
open(out, "wb").write(body[:size // 2] if mode == "short" else body)
""")
    tool.chmod(tool.stat().st_mode | stat.S_IEXEC)
    return [str(tool)], count


def _dump(esptool, tmp_path, monkeypatch, mode="same", **kw):
    monkeypatch.setenv("FAKE_ESPTOOL", mode)
    return df.dump(str(tmp_path / "port"), tmp_path / "kept", size=1024, tool=esptool[0],
                   name="stock", **kw)


def test_two_equal_reads_are_kept_with_a_checksum(esptool, tmp_path, monkeypatch):
    m = _dump(esptool, tmp_path, monkeypatch)
    kept = tmp_path / "kept"
    assert (kept / "stock.bin").stat().st_size == 1024
    assert m["sha256"] in (kept / "stock.bin.sha256").read_text()
    assert sorted(p.name for p in kept.iterdir()) == ["stock.bin", "stock.bin.json", "stock.bin.sha256"]
    calls = esptool[1].read_text().splitlines()
    assert len(calls) == 2 and all("read_flash 0 0x400" in c for c in calls)


def test_on_a_uart_a_person_restarts_the_bootloader_between_reads(esptool, tmp_path, monkeypatch):
    asked = []
    _dump(esptool, tmp_path, monkeypatch, before="no_reset", prompt=asked.append)
    assert len(asked) == 1 and "BOOT" in asked[0]
    assert all("--before no_reset --after no_reset" in c for c in esptool[1].read_text().splitlines())


def test_two_reads_that_differ_keep_nothing(esptool, tmp_path, monkeypatch):
    with pytest.raises(df.DumpError, match="differ"):
        _dump(esptool, tmp_path, monkeypatch, "differ")
    assert list((tmp_path / "kept").iterdir()) == []


def test_a_short_read_is_refused(esptool, tmp_path, monkeypatch):
    with pytest.raises(df.DumpError, match="short read"):
        _dump(esptool, tmp_path, monkeypatch, "short")


def test_esptool_failing_is_refused(esptool, tmp_path, monkeypatch):
    with pytest.raises(df.DumpError, match="exited 2"):
        _dump(esptool, tmp_path, monkeypatch, "fail")


def test_an_existing_dump_is_never_overwritten(esptool, tmp_path, monkeypatch):
    _dump(esptool, tmp_path, monkeypatch)
    before = (tmp_path / "kept" / "stock.bin").read_bytes()
    with pytest.raises(df.DumpError, match="already exists"):
        _dump(esptool, tmp_path, monkeypatch, "differ")
    assert (tmp_path / "kept" / "stock.bin").read_bytes() == before


def test_a_dump_inside_the_repository_is_refused(esptool):
    with pytest.raises(df.DumpError, match="inside the repository"):
        df.dump("/dev/null", df.ROOT / "firmware", size=16, tool=esptool[0])


def test_a_held_port_is_refused(esptool, tmp_path, monkeypatch):
    monkeypatch.setattr(df, "holders", lambda _p: ["812 python3"])
    with pytest.raises(df.DumpError, match="held by"):
        _dump(esptool, tmp_path, monkeypatch)
    assert not esptool[1].exists()


# ---------- criterion 6: L0.8 ----------

def _stream(source, **kw):
    return CameraStream(source, name="camera", **kw)


def _wait_for_a_frame(stream):
    deadline = time.monotonic() + 10.0
    while stream.status()["frames"] == 0 and time.monotonic() < deadline:
        time.sleep(0.02)
    assert stream.status()["frames"] > 0


def test_the_camera_check_reads_the_fake_camera_through_the_stream():
    stream = _stream(lambda: FakeCameraSource(30.0))
    try:
        _wait_for_a_frame(stream)
        report = cc.measure(stream, 1.0, period=0.05, min_seconds=0.5)
    finally:
        stream.close()
    assert report["stale_served"] == 0 and report["distinct_frames"] > 5, report
    assert report["age_p90_ms"] is not None and report["measured_s"] >= 0.5


class _AgedStream:
    """The stream's contract with a fixed frame age: the bar, without a
    loaded host's scheduling in it."""

    stale_s = 0.5

    def __init__(self, age_s):
        self.age_s, self.seq = age_s, 0

    def get_camera_frame(self):
        self.seq += 1
        return {"captured_at": time.time() - self.age_s, "metadata": {"seq": self.seq}}

    def status(self):
        return {}


def test_the_age_bar_is_read_at_100_ms():
    assert cc.measure(_AgedStream(0.03), 0.2, period=0.01, min_seconds=0.1)["met"]
    assert not cc.measure(_AgedStream(0.2), 0.2, period=0.01, min_seconds=0.1)["met"]


def test_a_camera_check_shorter_than_the_bar_is_not_met():
    stream = _stream(lambda: FakeCameraSource(30.0))
    try:
        _wait_for_a_frame(stream)
        assert not cc.measure(stream, 0.3, period=0.05)["met"]
    finally:
        stream.close()


class _Stalls:
    """Three frames, then nothing: a camera that stops delivering."""

    def __init__(self):
        self.n = 0

    def read(self):
        self.n += 1
        if self.n > 3:
            time.sleep(0.05)
            return None
        return b"\xff\xd8jpeg"

    def close(self):
        pass


def test_a_stalled_camera_is_refused_and_never_served():
    source = _Stalls()
    stream = _stream(lambda: source, stale_s=0.15)
    try:
        time.sleep(0.05)
        report = cc.measure(stream, 0.6, period=0.02)
    finally:
        stream.close()
    assert report["refused_stale"] > 0 and report["stale_served"] == 0, report
    assert not report["met"]


def test_a_camera_that_never_delivers_is_not_met():
    source = _Stalls()
    source.n = 10
    stream = _stream(lambda: source, stale_s=0.15)
    try:
        report = cc.measure(stream, 0.2, period=0.02)
    finally:
        stream.close()
    assert report["served"] == 0 and not report["met"]
