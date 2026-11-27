# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.229 (3.228 F1): the brain's safety check gets the body's self box.

`MissionRunner` hands the agent `_HaltGate(robot)`; before 3.229 the gate
had no `lidar_self_box_cm()`, so the agent's controller (and `ReturnTrip`,
built on it) read the Rover's mast as an obstacle at 0.0 cm and refused
the escape and the back-off the robot server would have allowed. Built
through the runner, as the brain builds it (3.218's test built a bare
`RemoteRobot`). Criteria: docs/plans/ros-alignment/3.229-brain-self-box.md.
"""

import pytest

import brain.dock as docking
from control.mission_runner import MissionRunner, _HaltGate
from robot.safety import SafetyController, SafetyViolation
from sim.maps import build_world
from sim.mock_robot import MockRobot
from sim.mock_world import MockWorld
from tests.test_explore import FakeNav
from tests.test_lidar_self_box import ROVER_BOX, _recorded

MAST = _recorded("median_cm")       # 360 bins from -180 deg, None off the mast


class _MastBody(MockRobot):
    """The simulator with the Rover's mast in every scan and its box reported."""

    def get_scan(self, max_range_m=None):
        scan = super().get_scan(max_range_m=max_range_m)
        assert scan["angle_min_deg"] == -180.0 and scan["angle_increment_deg"] == 1.0
        ranges = list(scan["ranges_m"])
        for i, mast in enumerate(MAST):
            if mast is not None and (ranges[i] is None or mast < ranges[i]):
                ranges[i] = mast
        return {**scan, "ranges_m": ranges}

    def lidar_self_box_cm(self):
        return list(ROVER_BOX)


def _runner(body_class):
    grid = build_world("scaled_house")
    body = body_class(grid, render=False)
    runner = MissionRunner(body, target_object="x", policy="explore",
                           navigator=FakeNav(body), world=MockWorld(grid),
                           mission_time_limit_s=1200, clock=lambda: grid.sim_time,
                           idle=body.pass_time)
    runner.start()
    return runner, body, grid


def _spy(safety):
    """Record every move the brain vets: (action, refusal or None)."""
    seen, real = [], safety.check_and_execute

    def vet(action, **kw):
        try:
            out = real(action, **kw)
        except SafetyViolation as e:
            seen.append((action, str(e)))
            raise
        seen.append((action, None))
        return out

    safety.check_and_execute = vet
    return seen


# ---- criterion 1: the brain's clearances are the server's -------------------

def test_the_brain_reads_the_mast_as_the_server_does():
    runner, body, _ = _runner(_MastBody)
    brain = runner.agent.safety
    server = SafetyController(body, self_box_cm=ROVER_BOX)
    assert brain.self_box_cm == ROVER_BOX
    assert brain.forward_clearance() == server.forward_clearance()
    assert brain.reverse_clearance() == server.reverse_clearance() == (0.0, "astern_occluded")
    f, source = brain.forward_clearance()
    assert f > brain.min_distance_cm, (f, source)        # not 0.0 scan_footprint


# ---- criterion 2: the escape and the back-off are sent ----------------------

def test_the_escape_sends_its_forward_and_refuses_reverse_only_as_the_server_does():
    runner, _, _ = _runner(_MastBody)
    agent = runner.agent
    seen = _spy(agent.safety)
    agent._queue_escape("wedged")
    for _ in range(50):
        if not agent._pending:
            break
        agent._do_pending()
    reverses = [why for a, why in seen if a == "REVERSE"]
    assert reverses and all(why and "astern_occluded" in why for why in reverses), seen
    assert ("FORWARD", None) in seen, seen
    assert not [s for s in seen if s[1] and ("scan_rear" in s[1] or "scan_footprint" in s[1])], seen


def test_the_return_back_off_is_refused_only_as_occluded_and_its_nudge_is_sent():
    runner, _, _ = _runner(_MastBody)
    safety = runner.agent.safety
    seen = _spy(safety)
    trip = docking.ReturnTrip({"x_m": 0.0, "y_m": 0.0, "theta": 0.0, "source": "test"},
                              runner._navigator,
                              safety, runner.world.get_pose, runner.clock)
    trip._back_off()
    assert seen[0][0] == "REVERSE" and "astern_occluded" in seen[0][1], seen
    assert docking.NUDGE_S > 0 and trip._moves == ["nudge"]
    trip.step()                                         # the queued nudge
    assert seen[1:] == [("FORWARD", None)], seen


# ---- criterion 3: a body with no box is as before ---------------------------

def test_a_body_without_a_box_is_unchanged():
    runner, body, _ = _runner(MockRobot)
    assert runner._gated.lidar_self_box_cm() is None
    brain = runner.agent.safety
    plain = SafetyController(body)
    assert brain.self_box_cm is None
    assert brain.forward_clearance() == plain.forward_clearance()
    assert brain.reverse_clearance() == plain.reverse_clearance()


@pytest.mark.parametrize("answer", [None, OSError("down"), "nonsense"])
def test_the_gate_passes_the_body_answer_and_the_controller_still_checks_it(answer):
    class Body(MockRobot):
        def lidar_self_box_cm(self):
            if isinstance(answer, Exception):
                raise answer
            return answer

    body = Body(build_world("scaled_house"), render=False)
    gate = _HaltGate(body, lambda: True)
    assert SafetyController(gate).self_box_cm is None   # an unreadable box is no box
