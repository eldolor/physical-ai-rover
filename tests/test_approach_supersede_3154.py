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
    """Records which `set` calls are approaches (explore's own kind, read
    from the call stack), so a refused send can never shift the pairing."""

    def set_goal(self, x_m, y_m):
        import inspect
        kind = next((f.frame.f_locals.get("kind") for f in inspect.stack()[1:4]
                     if f.function == "_send"), None)
        self.calls.append(("kind", kind))
        return super().set_goal(x_m, y_m)


def test_explore_sends_no_cancel_right_before_an_approach():
    runner, nav, grid = _run(_CallNav)
    calls = nav.calls
    approaches = [i for i, c in enumerate(calls) if c == ("kind", "approach")]
    assert approaches, "the mission never sent an approach: the test proves nothing"
    for i in approaches:
        before = [c for c in calls[:i] if c[0] in ("cancel", "set", "kind")]
        assert not before or before[-1][0] != "cancel", (
            f"a cancel right before an approach send: {calls[max(0, i - 4):i + 1]}")


class _RefuseApproach(WalkingNav):
    def set_goal(self, x_m, y_m):
        import inspect
        if any(f.function == "_send" and f.frame.f_locals.get("kind") == "approach"
               for f in inspect.stack()[1:4]) and self.goal and self.goal["state"] == "active":
            self.calls.append(("set", round(x_m, 2), round(y_m, 2)))
            self.calls.append(("refused",))
            self.refused_approach = True
            return {"accepted": False, "reason": "preempted"}
        return super().set_goal(x_m, y_m)


def test_a_refused_approach_still_ends_the_goal_it_would_have_superseded():
    """Review: with no goal delivered, nothing supersedes the old one, so it
    is cancelled -- never left driving untracked."""
    runner, nav, grid = _run(_RefuseApproach)
    assert getattr(nav, "refused_approach", False), "no approach was refused: the test proves nothing"
    for i, c in enumerate(nav.calls):
        if c == ("refused",):
            after = [x[0] for x in nav.calls[i + 1:] if x[0] in ("cancel", "set")]
            assert after and after[0] == "cancel", (
                f"the goal the refused approach would have superseded was not ended: "
                f"{nav.calls[i - 2:i + 4]}")


def test_a_goal_superseded_while_pending_gets_the_grace_too(bridge_mod):
    """Review: an old goal still pending when superseded was cancelled the
    moment nav2 accepted it -- the same mid-switch cancel."""
    b = _bridge(bridge_mod)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})          # A, pending
    b.send_goal({"x_m": 2.0, "y_m": 0.0})          # B supersedes it
    a = _Handle()
    b.nav.futures[0].complete(a)                   # nav2 accepts A late
    assert a.cancels == 0
    _later(b, bridge_mod.SUPERSEDE_GRACE_S + 0.5)
    b._resend_cancels()
    assert a.cancels == 1


def test_a_stop_before_a_late_accept_cancels_the_superseded_goal_at_once(bridge_mod):
    """Review 2: A pending, B supersedes it, a Stop, then nav2 accepts A: no
    grace for A -- the Stop came after its supersede."""
    b = _bridge(bridge_mod)
    b.send_goal({"x_m": 1.0, "y_m": 0.0})          # A, pending
    b.send_goal({"x_m": 2.0, "y_m": 0.0})          # B
    b.cancel_goal()                                # a Stop
    a = _Handle()
    b.nav.futures[0].complete(a)
    assert a.cancels == 1
