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
    cancels = runner.status()["explore"]["cancels"]
    assert nav.cancelled_ids, "the mission sent no cancel: the test proves nothing"
    assert [i for _, i, _ in cancels] == nav.cancelled_ids[-64:]
    assert [n for n, _, _ in cancels] == list(range(len(nav.cancelled_ids) - len(cancels) + 1,
                                                    len(nav.cancelled_ids) + 1))
    assert all(isinstance(t, float) for _, _, t in cancels)


def test_a_cancel_is_recorded_before_it_is_sent(monkeypatch):
    import time as _time
    from control.mission_runner import _GatedNavigator
    seen = []

    class Nav:
        def set_goal(self, x, y):
            return {"accepted": True, "goal_id": "f" * 32}

        def cancel_goal(self):
            seen.append(list(g.cancels))      # what the record held as the cancel went out
            return {"canceled": True}

        def get_goal(self):
            return {"goal": {"state": "active", "x_m": 1.0, "y_m": 2.0}}

    g = _GatedNavigator(Nav(), lambda: True)
    g.set_goal(1.0, 2.0)
    before = _time.time()
    g.cancel_goal()
    assert seen[0] and seen[0][-1][1] == "f" * 32 and seen[0][-1][2] >= round(before, 3) - 0.001
    g.cancel_own()                            # the mission's end: the same record
    assert len(seen) == 2 and seen[1][-1][0] == 2 and seen[1][-1][1] == "f" * 32


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
