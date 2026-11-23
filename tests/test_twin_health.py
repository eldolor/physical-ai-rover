# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.215.B: the twin's health line and `control/health.py`, without a browser.

The page re-implements the command's verdict rules (the servers publish
fields, not a verdict), and nothing pinned the two together until they had
drifted. `tests/test_ui.py` puts one table of `/health` bodies through both;
this module pins the two numbers the rules share, and checks that table
stands on both sides of every threshold.
"""

import re

from control import health
from tests.health_cases import BRAIN_CASES, ROBOT_CASES
from tests.twin_sources import twin_source


def _constant(name):
    found = re.findall(r"^\s*const %s = ([0-9.]+);" % name, twin_source(), re.M)
    assert len(found) == 1, (name, found)
    return float(found[0])


def test_the_page_and_the_command_share_their_two_numbers():
    """Criterion B3: change one side and this fails."""
    assert _constant("WATCHDOG_POLL_SLACK") == health.WATCHDOG_POLL_SLACK
    assert _constant("TICK_SLACK_S") == health.TICK_SLACK_S


def _status(monkeypatch, check, body):
    monkeypatch.setattr(health, "_get", lambda *a, **k: body)
    return check("http://stub")


def test_the_table_stands_on_both_sides_of_every_threshold(monkeypatch):
    """Criterion B1's table: at least 12 robot bodies and 8 brain bodies,
    and each of the command's problems is provoked by at least one."""
    assert len(ROBOT_CASES) >= 12 and len(BRAIN_CASES) >= 8
    assert len(dict(ROBOT_CASES)) == len(ROBOT_CASES)
    assert len(dict(BRAIN_CASES)) == len(BRAIN_CASES)
    said = {}
    for check, cases in ((health.check_robot, ROBOT_CASES), (health.check_brain, BRAIN_CASES)):
        for name, body in cases:
            part = _status(monkeypatch, check, body)
            said[name] = (part["status"], " | ".join(part["problems"]))
    ok = [n for n, (status, _) in said.items() if status == health.OK]
    for needle in ("watchdog loop has not polled", "the bridge is not answering",
                   "posts have stopped", "the mission loop is not turning"):
        assert any(needle in problems for _, problems in said.values()), needle
    # The boundaries themselves are not faults: `>` on both.
    for name in ("the watchdog at exactly 10 polls",
                 "a tick past the deadline, inside the slack",
                 "a tick at exactly the deadline plus the slack",
                 "drive: direct, ros_up false", "drive: ros, ros_up not published"):
        assert name in ok, (name, said[name])
    for name in ("the watchdog just past 10 polls", "a tick just past the slack",
                 "drive: ros, the bridge down"):
        assert name not in ok, (name, said[name])
