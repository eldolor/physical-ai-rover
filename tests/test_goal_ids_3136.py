# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.136.G criteria 1-2: nav2's goal id travels with the goal. The bridge
makes the goal's UUID and returns it in `POST /goal`'s reply; explore keeps
each accepted send's id with its kind, published as `status.explore.sent`."""

import uuid

from tests.test_explore import FakeNav
from tests.test_review_376h1 import _bridge, bridge_mod  # noqa: F401  (the fixture)


def test_the_bridge_returns_the_id_it_gives_nav2(bridge_mod):
    b = _bridge(bridge_mod)
    code, body = b.send_goal({"x_m": 1.0, "y_m": 0.0})
    assert code == 200 and body["state"] == "pending"
    sent = b.nav.uuids[-1]
    assert body["goal_id"] == bytes(sent.uuid).hex() and len(body["goal_id"]) == 32
    assert b.goal["goal_id"] == body["goal_id"]
    code, body2 = b.send_goal({"x_m": 2.0, "y_m": 0.0})
    assert code == 200 and body2["goal_id"] != body["goal_id"]


class IdNav(FakeNav):
    """FakeNav whose accepted goals carry a bridge id, as the 3.136.G bridge's do."""

    def __init__(self, robot, **kw):
        super().__init__(robot, **kw)
        self.ids = []

    def set_goal(self, x_m, y_m):
        reply = super().set_goal(x_m, y_m)
        if reply.get("accepted"):
            reply["goal_id"] = uuid.uuid4().hex
            self.ids.append(reply["goal_id"])
        return reply


def _run(nav_cls, refuse_first=0, max_steps=80):
    """tests.test_explore's `_mission`, with the navigator class given."""
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld
    grid = build_world("scaled_house")
    robot = MockRobot(grid, render=False)
    nav = nav_cls(robot, refuse_first=refuse_first)
    runner = MissionRunner(robot, target_object="red backpack", policy="explore", navigator=nav,
                           world=MockWorld(grid), max_steps=max_steps,
                           clock=lambda: grid.sim_time, idle=robot.pass_time)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < 5000:
        ticks += 1
    return runner, nav, grid


def test_explore_publishes_each_accepted_send_with_its_kind_and_id():
    runner, nav, grid = _run(IdNav, refuse_first=2)
    s = runner.status()["explore"]
    sent = s["sent"]
    assert [i for _, i, _ in sent] == nav.ids[-64:]        # in order; refusals record nothing
    assert len(sent) == min(64, s["goals_sent"])
    assert [n for n, _, _ in sent] == list(range(s["goals_sent"] - len(sent) + 1, s["goals_sent"] + 1))
    kinds = [e["kind"] for e in runner.agent.events if e["event"] == "goal_sent"]
    assert [k for _, _, k in sent] == kinds[-64:]


def test_a_bridge_with_no_id_records_null():
    runner, nav, grid = _run(FakeNav)
    sent = runner.status()["explore"]["sent"]
    assert sent and all(i is None for _, i, _ in sent)


def test_the_record_is_bounded():
    from brain.explore import SENT_IDS_KEPT
    assert SENT_IDS_KEPT == 64


# ---- 3.136.B amendment 6: the brain records each cancel it sends, by goal id ----

class CancelNav(IdNav):
    """IdNav that logs, at each cancel, which goal the bridge held then.
    Every third goal never arrives (stays active), so explore's goal timeout
    cancels it."""

    def __init__(self, robot, **kw):
        super().__init__(robot, **kw)
        self.cancelled_ids = []

    def get_goal(self):
        if self.goal and self.goal["state"] == "active" and len(self.ids) % 3 == 0:
            return {"goal": dict(self.goal), "plan": []}
        return super().get_goal()

    def cancel_goal(self):
        self.cancelled_ids.append(self.ids[-1] if self.ids else None)
        return super().cancel_goal()


def test_each_cancel_the_brain_sends_is_recorded_with_its_goal_id_first():
    runner, nav, grid = _run(CancelNav, max_steps=120)
    ex = runner.status()["explore"]
    cancels = ex["cancels"]
    assert ex["cancels_in_flight"] == 0
    assert nav.cancelled_ids, "the mission sent no cancel: the test proves nothing"
    assert [c[1] for c in cancels] == nav.cancelled_ids[-64:]
    assert [c[0] for c in cancels] == list(range(len(nav.cancelled_ids) - len(cancels) + 1,
                                                 len(nav.cancelled_ids) + 1))
    assert all(isinstance(c[2], float) and len(c) == 4 for c in cancels)
    assert all(c[3] is False for c in cancels)     # FakeNav answers no bridge `cancelled`


def test_a_cancel_is_recorded_with_the_time_before_it_went_out_once_it_returned():
    import time as _time
    from control.mission_runner import _GatedNavigator
    sent_at, fail = [], [False]

    class Nav:
        def set_goal(self, x, y):
            return {"accepted": True, "goal_id": "f" * 32}

        def cancel_goal(self):
            sent_at.append(_time.time())
            if fail[0]:
                raise OSError("the bridge is down")
            return {"cancelled": True}

        def get_goal(self):
            return {"goal": {"state": "active", "x_m": 1.0, "y_m": 2.0, "goal_id": "f" * 32}}

    g = _GatedNavigator(Nav(), lambda: True)
    g.set_goal(1.0, 2.0)
    g.cancel_goal()
    (n, gid, t, taken), = g.cancels_snapshot()["cancels"]
    assert (n, gid, taken) == (1, "f" * 32, True) and t <= sent_at[0] + 1e-6
    g.cancel_own()                            # the mission's end: the same record
    assert g.cancels_snapshot()["cancels"][-1][:2] == [2, "f" * 32]
    fail[0] = True                            # a cancel that never reached the bridge
    try:
        g.cancel_goal()
    except OSError:
        pass
    g.cancel_own()                            # swallowed there, and still not recorded
    assert len(g.cancels_snapshot()["cancels"]) == 2 and g.cancels_snapshot()["cancels_in_flight"] == 0


def test_a_goal_whose_answer_was_lost_is_not_named_by_a_later_cancel():
    from control.mission_runner import _GatedNavigator
    replies = [{"accepted": True, "goal_id": "a" * 32}, OSError("lost")]

    class Nav:
        def set_goal(self, x, y):
            r = replies.pop(0)
            if isinstance(r, Exception):
                raise r
            return r

        def cancel_goal(self):
            return {"canceled": True}

    g = _GatedNavigator(Nav(), lambda: True)
    g.set_goal(1.0, 0.0)
    try:
        g.set_goal(2.0, 0.0)
    except OSError:
        pass
    g.cancel_goal()
    assert g.cancels[-1][1] is None



def test_nothing_is_read_before_a_cancel_goes_out():
    from control.mission_runner import _GatedNavigator
    calls = []

    class Nav:
        def set_goal(self, x, y):
            return {"accepted": True, "goal_id": "f" * 32}

        def cancel_goal(self):
            calls.append("cancel")
            return {"cancelled": True}

        def get_goal(self):
            calls.append("get")
            return {"goal": None}

    g = _GatedNavigator(Nav(), lambda: True)
    g.set_goal(1.0, 2.0)
    g.cancel_goal()
    assert calls == ["cancel"]



def test_the_end_reports_when_its_own_goal_cancel_is_done():
    from control.mission_runner import _GatedNavigator
    seen = []

    class Nav:
        def set_goal(self, x, y):
            return {"accepted": True, "goal_id": "f" * 32}

        def get_goal(self):
            seen.append(g.cancels_snapshot()["goal_end_done"])
            return {"goal": {"state": "active", "x_m": 1.0, "y_m": 2.0}}

        def cancel_goal(self):
            return {"cancelled": True}

    g = _GatedNavigator(Nav(), lambda: True)
    g.set_goal(1.0, 2.0)
    assert g.cancels_snapshot()["goal_end_done"] is False
    g.end()
    assert seen == [False] and g.cancels_snapshot()["goal_end_done"] is True
    assert g.cancels_snapshot()["cancels"][-1][3] is True


def test_a_goal_answered_after_the_end_keeps_the_end_open_until_it_cancelled_itself():
    """`end()` ran while a goal was on its way: not done until that goal's
    late answer has cancelled it (review)."""
    import threading
    from control.mission_runner import _GatedNavigator
    running, release, reading = [True], threading.Event(), []

    class Nav:
        def set_goal(self, x, y):
            release.wait(5)
            return {"accepted": True, "goal_id": "f" * 32}

        def get_goal(self):
            reading.append(g.cancels_snapshot()["goal_end_done"])
            return {"goal": {"state": "active", "x_m": 1.0, "y_m": 2.0}}

        def cancel_goal(self):
            return {"cancelled": True}

    g = _GatedNavigator(Nav(), lambda: running[0])
    t = threading.Thread(target=g.set_goal, args=(1.0, 2.0))
    t.start()
    import time as _time
    while g._in_flight == 0:
        _time.sleep(0.01)
    running[0] = False
    g.end()                                      # nothing accepted yet: nothing to cancel
    assert g.cancels_snapshot()["goal_end_done"] is False
    release.set()
    t.join(5)
    assert reading == [False]                    # the late cancel read the goal while still open
    snap = g.cancels_snapshot()
    assert snap["goal_end_done"] is True and snap["cancels"][-1][1] == "f" * 32
