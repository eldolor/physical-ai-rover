# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.155 criterion 1: `robot/locked_serve.py` locks the robot server's
touched pages (`MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT`) only under an
unlimited memlock limit -- waiting up to `PICAR_MEMLOCK_WAIT_S` for one --
says what happened in one line, and then serves `robot.server:app` with the
arguments it was given."""

import ctypes
import errno
import json
import resource
import sys

import robot.locked_serve as ls

INF = resource.RLIM_INFINITY


class Libc:
    def __init__(self, rc=0, err=0):
        self.calls, self.rc, self.err = [], rc, err

    def mlockall(self, flags):
        self.calls.append(flags)
        if self.rc:
            ctypes.set_errno(self.err)
        return self.rc


def test_locks_touched_pages_under_an_unlimited_limit(monkeypatch):
    libc = Libc()
    monkeypatch.setattr(ls, "_libc", lambda: libc)
    monkeypatch.setattr(ls.resource, "getrlimit", lambda r: (INF, INF))
    got = ls.lock()
    assert libc.calls == [ls.MCL_CURRENT | ls.MCL_FUTURE | ls.MCL_ONFAULT]
    assert got["locked"] is True and got["limit_soft_kb"] == "unlimited"


def test_never_locks_under_a_finite_limit(monkeypatch):
    libc = Libc()
    monkeypatch.setattr(ls, "_libc", lambda: libc)
    monkeypatch.setattr(ls.resource, "getrlimit", lambda r: (64 << 10, 64 << 10))
    got = ls.lock()
    assert libc.calls == [] and got["locked"] is False and got["error"] == "limit_not_raised"
    assert got["limit_soft_kb"] == 64


def test_a_refusal_is_reported(monkeypatch):
    monkeypatch.setattr(ls, "_libc", lambda: Libc(rc=-1, err=errno.EPERM))
    monkeypatch.setattr(ls.resource, "getrlimit", lambda r: (INF, INF))
    got = ls.lock()
    assert got["locked"] is False and got["error"] == "EPERM"


def test_waits_for_the_limit_to_be_raised(monkeypatch):
    seq = [(8 << 20, 8 << 20)] * 3 + [(INF, INF)]
    t = [0.0]
    monkeypatch.setattr(ls.resource, "getrlimit", lambda r: seq.pop(0) if len(seq) > 1 else seq[0])
    libc = Libc()
    monkeypatch.setattr(ls, "_libc", lambda: libc)
    got = ls.lock(wait_s=10, sleep=lambda s: t.__setitem__(0, t[0] + s), now=lambda: t[0])
    assert got["locked"] is True and 0.2 <= t[0] < 1.0
    seq[:] = [(8 << 20, 8 << 20)]
    t[0] = 0.0
    got = ls.lock(wait_s=2, sleep=lambda s: t.__setitem__(0, t[0] + s), now=lambda: t[0])
    assert got["error"] == "limit_not_raised" and t[0] >= 2


def test_main_says_what_happened_then_serves_the_robot_server(monkeypatch, capsys):
    served = []
    monkeypatch.setattr(ls, "lock", lambda wait_s=0.0: {"locked": False, "error": "limit_not_raised",
                                                         "wait_s": wait_s})
    monkeypatch.setattr(ls, "_serve", lambda argv: served.append(argv))
    monkeypatch.setenv("PICAR_MEMLOCK_WAIT_S", "10")
    ls.main(["uvicorn", "robot.server:app", "--port", "8000", "--host", "127.0.0.1"])
    line = [x for x in capsys.readouterr().err.splitlines() if x.startswith("robot server memory lock: ")]
    assert json.loads(line[0].split(": ", 1)[1])["wait_s"] == 10.0
    assert served == [["robot.server:app", "--port", "8000", "--host", "127.0.0.1"]]
    monkeypatch.setenv("PICAR_MEMLOCK", "0")                    # the plant: no lock at all
    monkeypatch.setattr(ls, "lock", lambda wait_s=0.0: (_ for _ in ()).throw(AssertionError("locked")))
    ls.main(["uvicorn", "robot.server:app"])
    assert "PICAR_MEMLOCK=0" in capsys.readouterr().err
    import pytest
    with pytest.raises(SystemExit):
        ls.main(["--port", "8000"])                              # uvicorn's own command line only


def test_serve_hands_uvicorn_its_own_command_line(monkeypatch):
    seen = []
    import uvicorn.main
    monkeypatch.setattr(uvicorn.main, "main", lambda: seen.append(list(sys.argv)))
    ls._serve(["robot.server:app", "--port", "8000"])
    assert seen == [["uvicorn", "robot.server:app", "--port", "8000"]]


def test_a_bad_wait_never_stops_the_robot_server(monkeypatch, capsys):
    served = []
    monkeypatch.setattr(ls, "lock", lambda wait_s=0.0: {"locked": False, "wait_s": wait_s})
    monkeypatch.setattr(ls, "_serve", lambda argv: served.append(argv))
    monkeypatch.setenv("PICAR_MEMLOCK_WAIT_S", "10s")
    ls.main(["uvicorn", "robot.server:app"])
    got = json.loads(capsys.readouterr().err.split("robot server memory lock: ")[1].splitlines()[0])
    assert got["wait_s"] == 0.0 and got["wait_s_ignored"] == "10s" and served
