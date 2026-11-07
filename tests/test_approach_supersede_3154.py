# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.154 criterion 6 (`docs/plans/ros-alignment/3.154-approach-abort-on-cancel.md`):
a new nav2 goal supersedes the live one with no cancel sent first. A cancel
right before the send halted nav2's tree for the old goal, and nav2 ended the
NEW goal ABORTED -- explore's approach, 6-10 ms after it began executing
(3.136.B). The bridge still holds the old goal live until nav2's result
(3.76 Q3), with its first cancel deferred past the switch; a Stop meanwhile
cancels it at once. Explore no longer sends a DELETE before its approach."""

import types

from tests.test_explore import WalkingNav
from tests.test_goal_ids_3136 import _run
from tests.test_review_376h1 import _Handle, _accepted_goal, _bridge, _later, bridge_mod  # noqa: F401


def test_a_new_goal_sends_no_cancel_for_the_old_one_at_the_switch(bridge_mod):
    b = _bridge(bridge_mod)
    old = _accepted_goal(b)
    b.send_goal({"x_m": 2.0, "y_m": 0.0})
    assert old.cancels == 0, "a cancel at the switch is what aborted the new goal"
    new = _Handle()
    b.nav.futures[-1].complete(new)
    b._resend_cancels()
    assert old.cancels == 0, "still inside the grace"
    # The old goal is still tracked: it reads live until nav2 ends it (3.76 Q3).
    assert b.goal_state()[1]["cancelling"] == 1
    _later(b, bridge_mod.SUPERSEDE_GRACE_S + 0.5)
    b._resend_cancels()
    assert old.cancels == 1, "no result after the grace: the timer cancels it"
    old.result_future.complete(types.SimpleNamespace(status=6))    # ABORTED, preempted
    assert b.goal_state()[1]["cancelling"] == 0
    assert b.goal_state()[1]["goal"]["state"] == "active", "the old result is not the new goal's"
    assert new.cancels == 0


def test_a_stop_inside_the_grace_cancels_the_superseded_goal_at_once(bridge_mod):
    b = _bridge(bridge_mod)
    old = _accepted_goal(b)
    b.send_goal({"x_m": 2.0, "y_m": 0.0})
    new = _Handle()
    b.nav.futures[-1].complete(new)
    assert b.cancel_goal() is True
    assert old.cancels == 1 and new.cancels == 1


class _CallNav(WalkingNav):
    """A goal stays live while the robot walks it, so an approach can find
    one to supersede (FakeNav's goals end at the next poll)."""


def test_explore_sends_no_cancel_right_before_an_approach():
    runner, nav, grid = _run(_CallNav)
    kinds = [e["kind"] for e in runner.agent.events if e["event"] == "goal_sent"]
    assert "approach" in kinds, "the mission never sent an approach: the test proves nothing"
    sets = [i for i, c in enumerate(nav.calls) if c[0] == "set"]
    for n, i in enumerate(sets):
        if n < len(kinds) and kinds[n] == "approach":
            assert i == 0 or nav.calls[i - 1][0] != "cancel", (
                f"a cancel right before approach send {n}: {nav.calls[max(0, i - 3):i + 1]}")
