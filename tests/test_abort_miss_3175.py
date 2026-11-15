# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.175.F: a recall goal that ends without arriving counts as the miss it
never got -- the carried landmark at that place is held to `STALE_BELIEF`, so
the next mission does not recall it
(docs/plans/ros-alignment/3.175-memory-when-things-move.md)."""

from brain.inventory import REPORT_BELIEF, STALE_BELIEF, Inventory
from tests.test_explore import FakeNav
from tests.test_recall_3172 import _far_floor, _report, _row, _runner


class AbortingNav(FakeNav):
    """FakeNav whose first `abort` accepted goals abort where the robot stands."""

    def __init__(self, robot, abort=1):
        super().__init__(robot)
        self.abort = abort

    def get_goal(self):
        if self.goal and self.goal["state"] == "active" and self.abort:
            self.calls.append(("get",))
            self.abort -= 1
            self.robot.pass_time(2.0)
            self.goal["state"] = "aborted"
            return {"goal": self.goal, "plan": []}
        return super().get_goal()


def _lm(inv, label, x=None):
    return next(lm for lm in inv.landmarks if lm.label == label and (x is None or lm.x == x))


def test_recall_failed_holds_carried_unseen_landmarks_at_the_place_stale():
    inv = Inventory()
    inv.seed("f1", _report([_row("washer", 1.0, 1.0), _row("washer", 1.5, 1.0),
                            _row("dryer", 1.0, 1.0)]))
    names = lambda label: label == "washer"            # noqa: E731
    assert inv.recall_failed(1.01, 1.0, names) == 1
    assert _lm(inv, "washer", 1.0).belief <= STALE_BELIEF + 1e-9 < REPORT_BELIEF
    assert _lm(inv, "washer", 1.5).belief >= REPORT_BELIEF        # another place
    assert _lm(inv, "dryer").belief >= REPORT_BELIEF              # not the target
    assert inv.recall_failed(1.0, 1.0, names) == 0                # already stale: not counted again
    nxt = Inventory()
    nxt.seed("f1", dict(inv.report()))
    assert _lm(nxt, "washer", 1.0).carried and _lm(nxt, "washer", 1.0).belief < REPORT_BELIEF


def test_a_landmark_seen_this_mission_keeps_its_belief():
    inv = Inventory()
    inv.seed("f1", _report([_row("washer", 1.0, 1.0)]))
    lm = _lm(inv, "washer")
    lm.seen_seq = inv.carry_seq                          # seen by this mission
    before = lm.score
    assert inv.recall_failed(1.0, 1.0) == 0 and lm.score == before


def _recall_mission(abort):
    runner, grid, nav = _runner("washer")
    nav = AbortingNav(runner.robot, abort=abort)
    runner.agent.navigator = nav
    place = _far_floor(grid, nav)
    runner.inventory.seed("f1", _report([_row("washer", *place)]))
    assert runner._recall_carried() == 1
    for _ in range(300):
        runner.tick()
        if any(k == "frontier" for _, _, k in runner.agent.sent):
            break
    return runner, place


def test_an_aborted_recall_holds_its_landmark_stale():
    runner, place = _recall_mission(abort=1)
    ended = [e for e in runner.agent.events if e["event"] == "goal_ended" and e["kind"] == "recall"]
    assert [e["state"] for e in ended] == ["aborted"]
    lm = _lm(runner.inventory, "washer")
    assert lm.belief <= STALE_BELIEF + 1e-9 and runner._recall_abort_misses == 1
    assert any("held stale" in line for line in runner._log)


def test_a_recall_that_arrives_counts_no_abort_miss():
    runner, place = _recall_mission(abort=0)
    ended = [e for e in runner.agent.events if e["event"] == "goal_ended" and e["kind"] == "recall"]
    assert [e["state"] for e in ended] == ["succeeded"]
    assert getattr(runner, "_recall_abort_misses", 0) == 0
    assert not any("held stale" in line for line in runner._log)


def test_each_event_is_read_once():
    runner, place = _recall_mission(abort=1)
    lm = _lm(runner.inventory, "washer")
    lm.score = 5.0                                       # as if seen again since
    assert runner._recall_misses() == 0 and lm.score == 5.0
