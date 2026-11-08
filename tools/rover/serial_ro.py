# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tools/rover/serial_ro.py

3.168: a serial port that can only be read. The bring-up tools listen to the
Rover's motor board and lidar and must never reach their input, so the port
is opened `O_RDONLY`: a write on it fails in the kernel, whatever a later
edit of a tool does. No serial library, for the same reason.

A port another process holds is refused: setting the baud here would change
it under that process, and its next command would reach the board as garbage.
On a real device that needs root, or a holder could be hidden from the check.

The limit of "read-only": a freshly booted tty has the kernel's echo on until
the raw settings below land, so bytes arriving in that instant could be
echoed to TX. Too brief and too garbled to be a command, but not nothing.

Opening a tty can still raise DTR. On a USB bridge wired to the ESP32's
auto-reset that resets the board (no motion: `setup()` stops the wheels); on
the 40-pin UART there is no such line.
"""

import glob
import os
import select
import stat
import termios
import time
from pathlib import Path
from typing import Iterator, List, Tuple

BOARD_BAUD = 115200       # sim/fake_esp32.py, robot/hardware_robot.py
LIDAR_BAUD = 230400       # robot/lidar_ld19.BAUD
HANGUP_READS = 3          # "readable" with nothing to read, this many running


class PortHeld(OSError):
    pass


class PortLost(OSError):
    pass


def holders(path: str, proc: str = "/proc") -> List[str]:
    """`pid comm` of each OTHER process with this port open, as far as this
    user may see (`can_see_all()` says whether that is everyone)."""
    real = os.path.realpath(path)
    out = []
    for fd_dir in glob.glob(os.path.join(proc, "[0-9]*", "fd")):
        pid = os.path.basename(os.path.dirname(fd_dir))
        if pid == str(os.getpid()):
            continue
        try:
            for fd in os.listdir(fd_dir):
                if os.path.realpath(os.path.join(fd_dir, fd)) == real:
                    out.append(f"{pid} {Path(proc, pid, 'comm').read_text().strip()}")
                    break
        except OSError:
            continue
    return out


def can_see_all() -> bool:
    return os.geteuid() == 0


def holder_may_hide(path: str) -> bool:
    """A real serial device, and a user who cannot see every process."""
    if can_see_all():
        return False
    try:
        device = stat.S_ISCHR(os.stat(path).st_mode)
    except OSError:
        return False
    return device and not os.path.realpath(path).startswith("/dev/pts/")


def open_readonly(path: str, baud: int) -> int:
    """`path` raw at `baud`, read-only and never the controlling terminal."""
    held = holders(path)
    if held:
        raise PortHeld(f"{path} is held by {held}")
    if holder_may_hide(path):
        raise PortHeld(f"{path}: not root, so a holder could be hidden: run under sudo")
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        speed = getattr(termios, f"B{baud}")
        _iflag, _oflag, cflag, _lflag, _ispeed, _ospeed, cc = termios.tcgetattr(fd)
        cflag = (cflag & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.HUPCL)) \
            | termios.CS8 | termios.CREAD | termios.CLOCAL
        cc = list(cc)
        cc[termios.VMIN], cc[termios.VTIME] = 0, 0
        termios.tcsetattr(fd, termios.TCSANOW, [0, 0, cflag, 0, speed, speed, cc])
        termios.tcflush(fd, termios.TCIFLUSH)      # what waited unread is not this capture's
    except termios.error as e:
        os.close(fd)
        raise OSError(f"{path}: not a usable serial port: {e}") from e
    except Exception:
        os.close(fd)
        raise
    return fd


def chunks(fd: int, seconds: float, clock=time.monotonic) -> Iterator[Tuple[float, bytes]]:
    """(arrival time, bytes) for `seconds`. A port that goes away raises
    `PortLost`: a capture cut short must not read as a complete one."""
    end = clock() + seconds
    empty = 0
    while True:
        left = end - clock()
        if left <= 0:
            return
        ready, _, _ = select.select([fd], [], [], min(left, 0.1))
        if not ready:
            continue
        try:
            data = os.read(fd, 65536)
        except BlockingIOError:
            continue
        except OSError as e:
            raise PortLost(f"the port went away: {e}") from e
        if not data:
            empty += 1
            if empty >= HANGUP_READS:
                raise PortLost("the port hung up")
            time.sleep(0.01)
            continue
        empty = 0
        yield clock(), data


def lines(fd: int, seconds: float, clock=time.monotonic) -> Iterator[Tuple[float, bytes]]:
    """Whole newline-ended lines with the time each was completed."""
    buf = b""
    for t, data in chunks(fd, seconds, clock):
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            yield t, line.strip()
