# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.136.G criteria 1-2: nav2's goal id travels with the goal. The bridge
makes the goal's UUID and returns it in `POST /goal`'s reply; explore keeps
each accepted send's id with its kind, published as `status.explore.sent`."""

import uuid

from tests.test_explore import FakeNav, _mission
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


def _run(nav_cls, **kw):
    import tests.test_explore as te
    old = te.FakeNav
    te.FakeNav = nav_cls
    try:
        return _mission(**kw)
    finally:
        te.FakeNav = old


def test_explore_publishes_each_accepted_send_with_its_kind_and_id():
    runner, nav, grid = _run(IdNav, refuse_first=2)
    s = runner.status()["explore"]
    sent = s["sent"]
    assert [i for i, _ in sent] == nav.ids[-64:]           # in order; refusals record nothing
    assert len(sent) == min(64, s["goals_sent"])
    kinds = [e["kind"] for e in runner.agent.events if e["event"] == "goal_sent"]
    assert [k for _, k in sent] == kinds[-64:]


def test_a_bridge_with_no_id_records_null():
    runner, nav, grid = _mission()
    sent = runner.status()["explore"]["sent"]
    assert sent and all(i is None for i, _ in sent)


def test_the_record_is_bounded():
    from brain.explore import SENT_IDS_KEPT
    assert SENT_IDS_KEPT == 64
