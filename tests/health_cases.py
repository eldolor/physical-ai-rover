# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.215.B: one table of `/health` bodies, judged twice.

`control/health.py` is the definition of "unhealthy" (CLAUDE.md section 6,
M5). The twin's health line re-implements its rules in the browser, and by
2026-11-22 the two had drifted (`3.207-docs-gaps.md`, Batch 5). These bodies
sit on both sides of every verdict input's threshold; `tests/test_ui.py`
puts each through the command and through the page and counts the
disagreements (the bar is 0), and `tests/test_twin_health.py` checks the
table itself covers what it says.

Not a test module: a table two modules read.
"""

_ID = {"git_revision": "abc1234"}


def _robot(**fields):
    return {"status": "ok", "mode": "sim", "seconds_since_last_command": 0.2,
            "watchdog_timeout_s": 1.0, "seconds_since_watchdog_poll": 0.05,
            "watchdog_poll_interval_s": 0.1, "identity": _ID, **fields}


def _brain(**fields):
    return {"status": "ok", "mission_running": True, "tick_timeout_s": 30.0,
            "seconds_since_last_tick": 0.2, "identity": _ID, **fields}


HEALTHY_ROBOT = _robot()
HEALTHY_BRAIN = _brain(mission_running=False)

# (name, body). What each should read is the command's to say, not this
# table's; the names say which side of which threshold a body sits on.
ROBOT_CASES = [
    ("quiet and well", HEALTHY_ROBOT),
    ("a server older than M5: no watchdog fields",
     {"status": "ok", "mode": "sim", "identity": _ID}),
    ("the poll interval absent", _robot(watchdog_poll_interval_s=None,
                                        seconds_since_watchdog_poll=99.0)),
    ("the poll age absent", _robot(seconds_since_watchdog_poll=None)),
    ("the watchdog at exactly 10 polls", _robot(seconds_since_watchdog_poll=1.0)),
    ("the watchdog just past 10 polls", _robot(seconds_since_watchdog_poll=1.01)),
    ("the watchdog long gone", _robot(seconds_since_watchdog_poll=5.0)),
    ("drive: ros, ROS up",
     _robot(drive={"mode": "ros", "ros_up": True, "bridge_up": True, "ros_post_age_s": 0.04})),
    ("drive: ros, the bridge down",
     _robot(drive={"mode": "ros", "ros_up": False, "bridge_up": False, "ros_post_age_s": None})),
    ("drive: ros, the bridge up and the posts stopped",
     _robot(drive={"mode": "ros", "ros_up": False, "bridge_up": True, "ros_post_age_s": 12.5})),
    ("drive: ros, ros_up false and nothing said of the bridge",
     _robot(drive={"mode": "ros", "ros_up": False})),
    ("drive: ros, ros_up not published",
     _robot(drive={"mode": "ros"})),
    ("drive: ros, ros_up null",
     _robot(drive={"mode": "ros", "ros_up": None, "bridge_up": False})),
    ("drive: direct, ros_up false",
     _robot(drive={"mode": "direct", "ros_up": False, "bridge_up": False})),
    ("drive null", _robot(drive=None)),
    ("ROS down and the watchdog gone, both",
     _robot(seconds_since_watchdog_poll=5.0,
            drive={"mode": "ros", "ros_up": False, "bridge_up": False})),
]

BRAIN_CASES = [
    ("no mission", HEALTHY_BRAIN),
    ("no mission, an old tick", _brain(mission_running=False, seconds_since_last_tick=900.0)),
    ("a mission ticking", _brain()),
    ("a tick at exactly the deadline", _brain(seconds_since_last_tick=30.0)),
    ("a tick past the deadline, inside the slack", _brain(seconds_since_last_tick=30.5)),
    ("a tick at exactly the deadline plus the slack", _brain(seconds_since_last_tick=31.0)),
    ("a tick just past the slack", _brain(seconds_since_last_tick=31.01)),
    ("a loop long gone", _brain(seconds_since_last_tick=120.0)),
    ("a mission with no tick yet", _brain(seconds_since_last_tick=None)),
    ("a mission with no deadline published",
     _brain(tick_timeout_s=None, seconds_since_last_tick=120.0)),
]
