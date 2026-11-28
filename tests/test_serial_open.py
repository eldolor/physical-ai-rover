"""3.238: the robot server in hardware mode hung at start-up after a fresh
boot, in `tty.setraw()` on `/dev/ttyTHS1` (the kernel in `set_termios`).

`setraw` applies its settings with `TCSAFLUSH`, which waits for the port's
output to drain; a port left at 9600 baud with echo on, under a board that
streams at 115200, echoes what it hears and never drains. A pty cannot show
that wait -- Linux's pty driver reports no pending output, so its
`set_termios` never blocks -- so `never_drains` models the serial driver:
any `tcsetattr` that is not `TCSANOW`, and any `tcdrain`, waits until the
test ends. The first test pins that the model reproduces the hang on the old
path; the rest hold the open to a bound under it, and check what it leaves.
"""

import fcntl
import os
import termios
import threading
import time
import tty

import pytest

import robot.hardware_robot as hw
from robot.hardware_robot import HardwareRobot, open_serial
from sim.fake_esp32 import FakeEsp32
from sim.maps.starter_house import build_starter_world
from sim.mock_robot import MockRobot

BOUND_S = 1.0       # criterion 1: the open configures within this, under the model


@pytest.fixture
def never_drains(monkeypatch):
    """A line whose output never drains: every draining call waits until
    teardown. Returns the list of calls that waited."""
    release = threading.Event()
    waited = []
    real_set, real_drain = termios.tcsetattr, termios.tcdrain

    def tcsetattr(fd, when, attrs):
        if when != termios.TCSANOW:
            waited.append(("tcsetattr", when))
            release.wait()
        return real_set(fd, when, attrs)

    def tcdrain(fd):
        waited.append(("tcdrain",))
        release.wait()
        return real_drain(fd)

    for mod in (termios, tty):      # tty does `from termios import *`
        monkeypatch.setattr(mod, "tcsetattr", tcsetattr, raising=False)
        monkeypatch.setattr(mod, "tcdrain", tcdrain, raising=False)
    yield waited
    release.set()


def _as_after_boot(path):
    """Leave a tty as `/dev/ttyTHS1` is after a boot: 9600, cooked, echo on."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    try:
        a = termios.tcgetattr(fd)
        a[0] |= termios.ICRNL | termios.IXON
        a[1] |= termios.OPOST
        a[3] |= termios.ECHO | termios.ICANON | termios.ISIG | termios.IEXTEN
        a[4] = a[5] = termios.B9600
        termios.tcsetattr(fd, termios.TCSANOW, a)
    finally:
        os.close(fd)


def _assert_raw_115200(fd):
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
    assert ispeed == ospeed == termios.B115200
    assert not lflag & (termios.ECHO | termios.ICANON | termios.ISIG | termios.IEXTEN)
    assert not oflag & termios.OPOST
    assert not iflag & (termios.ICRNL | termios.IXON | termios.IXOFF)
    assert cflag & termios.CSIZE == termios.CS8
    assert cflag & (termios.CREAD | termios.CLOCAL) == termios.CREAD | termios.CLOCAL
    assert not cflag & (termios.PARENB | termios.CSTOPB)
    assert cc[termios.VMIN] == 1 and cc[termios.VTIME] == 0


@pytest.fixture
def board():
    body = MockRobot(build_starter_world(), render=False)
    b = FakeEsp32(body)
    _as_after_boot(b.path)
    yield b
    b.close()


def _in_thread(fn):
    out = {}

    def run():
        try:
            out["value"] = fn()
        except BaseException as e:     # noqa: BLE001 -- reported by the test
            out["error"] = e
    t = threading.Thread(target=run, daemon=True)
    t0 = time.monotonic()
    t.start()
    t.join(BOUND_S)
    return t, out, time.monotonic() - t0


def test_the_model_reproduces_the_hang_on_the_old_path(board, never_drains):
    """What dev did before 3.238: `tty.setraw()` waits for good."""
    fd = os.open(board.path, os.O_RDWR | os.O_NOCTTY)
    try:
        t, out, _ = _in_thread(lambda: tty.setraw(fd))
        assert t.is_alive(), "setraw returned: the model no longer models the drain"
        assert ("tcsetattr", termios.TCSAFLUSH) in never_drains
    finally:
        never_drains.clear()


def test_a_port_that_never_drains_configures_within_the_bound(board, never_drains):
    t, out, took = _in_thread(lambda: HardwareRobot(board.path))
    robot = out.get("value")
    try:
        assert not t.is_alive(), f"HardwareRobot still opening after {BOUND_S} s"
        assert "error" not in out, out.get("error")
        assert took < BOUND_S
        assert never_drains == []           # nothing on the open path waits
        _assert_raw_115200(robot._fd)
        # The link works after it: frames arrive and are parsed.
        end = time.monotonic() + 2.0
        while robot.frames == 0 and time.monotonic() < end:
            time.sleep(0.02)
        assert robot.frames > 0
        assert robot.feedback_status()["fresh"]
    finally:
        if robot is not None:
            robot.close()


def test_open_serial_leaves_a_blocking_raw_port(board):
    """Without the model, on a real pty left as after a boot."""
    t0 = time.monotonic()
    fd = open_serial(board.path, 115200)
    try:
        assert time.monotonic() - t0 < BOUND_S
        _assert_raw_115200(fd)
        assert not fcntl.fcntl(fd, fcntl.F_GETFL) & os.O_NONBLOCK
    finally:
        os.close(fd)


@pytest.mark.skipif(not os.path.isdir("/proc/self/fd"), reason="needs /proc")
def test_a_failed_set_up_closes_the_fd(board, monkeypatch):
    def refuse(fd, when, attrs):
        raise termios.error(5, "Input/output error")
    monkeypatch.setattr(hw.termios, "tcsetattr", refuse)
    before = set(os.listdir("/proc/self/fd"))
    with pytest.raises(termios.error):
        open_serial(board.path)
    assert set(os.listdir("/proc/self/fd")) <= before
