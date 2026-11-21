# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/test_inventory_edge_3204.py -- 3.204.B: the inventory's depth-edge
gate (`brain/inventory.py` `straddles_edge`, `Inventory(edge_gap_m=...)`),
behind `brain.inventory_edge_gate`, OFF by default.

The key-off rule (the nearest return in the gate) stays pinned where it was,
`tests/test_inventory.py::test_the_nearest_return_in_the_gate_places_it`;
this file pins the key-on rule beside it, the default, and the wiring. The
house-scale numbers are pinned at the bottom from the recorded sweeps
(`evaluations/phantom-landmark-3204/B/`, the bars in
`docs/plans/ros-alignment/3.204-phantom-landmark.md`).
"""
import json
from pathlib import Path

import pytest

from brain import inventory as inv
from brain.inventory import Inventory, scan_returns, straddles_edge
from tests.test_inventory import _pose, _scan

JAMB = {0: 2.0, 1: 0.6, -1: 2.0}        # test_inventory's jamb in front of a mug
MUG = [{"label": "mug", "bearing_deg": 0.0}]


def test_off_is_the_default_everywhere():
    """The user's ranging rule (2026-11-17) stays unless the user says so."""
    from control.brain_config import load_brain_config
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from tests.conftest import mock_world_for
    assert Inventory().edge_gap_m is None
    assert load_brain_config()["inventory_edge_gate"] is False
    robot = MockRobot(build_world("complex_house"), render=False)
    r = MissionRunner(robot, target_object="unicorn", max_steps=2, policy="frontier",
                      world=mock_world_for(robot))
    assert r.inventory.edge_gap_m is None
    r = MissionRunner(robot, target_object="unicorn", max_steps=2, policy="frontier",
                      world=mock_world_for(robot), inventory_edge_gate=True)
    assert r.inventory.edge_gap_m == inv.EDGE_GAP_M


def test_the_brain_config_refuses_a_non_bool(tmp_path):
    from control.brain_config import load_brain_config
    p = tmp_path / "robot.yaml"
    for bad in ("1", "yes please", "0.3"):
        p.write_text(f"brain:\n  inventory_edge_gate: {bad}\n")
        with pytest.raises(ValueError, match="inventory_edge_gate"):
            load_brain_config(p)
    p.write_text("brain:\n  inventory_edge_gate: true\n")
    assert load_brain_config(p)["inventory_edge_gate"] is True


def test_the_brain_server_hands_the_key_to_the_runner(tmp_path, monkeypatch):
    from fastapi.testclient import TestClient
    import control.brain_server as bs
    from tests.conftest import RecordingRobot, fresh_mock_robot
    seen = []
    real = bs.drills.apply

    def spy(fault, kwargs, config):        # where the server's kwargs meet the runner class
        seen.append(kwargs.get("inventory_edge_gate"))
        return real(fault, kwargs, config)
    monkeypatch.setattr(bs.drills, "apply", spy)
    for value in ("false", "true"):
        cfg = tmp_path / f"robot-{value}.yaml"
        cfg.write_text(f"brain:\n  inventory_edge_gate: {value}\n")
        app = bs.create_app(config_path=str(cfg),
                            robot_factory=lambda: RecordingRobot(fresh_mock_robot()))
        with TestClient(app) as client:
            resp = client.post("/mission/start", json={"target_object": "x",
                                                       "policy": "frontier"})
            client.post("/mission/stop")
        assert resp.status_code == 200, resp.text
    assert seen == [False, True]


def test_a_gate_over_a_jamb_straddles_an_edge():
    rets = scan_returns(_scan(JAMB))
    assert straddles_edge(rets, 0.0, 0.3)
    assert not straddles_edge(rets, 0.0, 1.5)          # wider than the step
    flat = scan_returns(_scan({b: 2.0 for b in range(-3, 4)}))
    assert not straddles_edge(flat, 0.0, 0.3)
    assert not straddles_edge(scan_returns(_scan()), 0.0, 0.3)   # no returns


def test_key_on_leaves_the_jamb_case_unplaced():
    """The pinned key-off case, key on: the jamb is not taken for the mug."""
    i = Inventory(edge_gap_m=0.3)
    i.observe(MUG, _pose(), _scan(JAMB))
    assert i.landmarks == [] and i.unplaced == 1 and i.observations == 1


def test_key_on_still_places_a_clean_gate():
    i = Inventory(edge_gap_m=0.3)
    i.observe(MUG, _pose(), _scan({b: 2.0 for b in range(-3, 4)}))
    (lm,) = i.landmarks
    assert (lm.x, lm.y) == pytest.approx((0.0, -2.0), abs=0.01)


def test_key_on_an_edge_sighting_spares_a_miss():
    """Held like a bearing with no return: the mug seen beside a jamb is
    not missed there. Key off, the same frame ranges the jamb, makes a
    second landmark, and the mug takes a (pending) miss."""
    scan = _scan({-3: 0.6, -2: 2.0, -1: 2.0, 0: 2.0, 1: 2.0, 2: 2.0})
    seen = [{"label": "mug", "bearing_deg": -1.5}]
    for gap, spared in ((0.3, True), (None, False)):
        i = Inventory(edge_gap_m=gap)
        i.observe(MUG, _pose(0, 0, 90), _scan({0: 2.5}))
        i.observe(MUG, _pose(0.6, 0, 90), _scan({0: 1.9}))
        mug = i.landmarks[0]
        i.observe(seen, _pose(0.5, 0, 90), scan)
        assert (mug._pending_miss is None) is spared
        assert len(i.landmarks) == (1 if spared else 2)


def test_key_on_a_far_return_is_still_far():
    """An edge gate never turns an out-of-range return into a spare: a
    detection whose nearest return is past MAX_RANGE_M was never placed."""
    far = {b: inv.MAX_RANGE_M + 1 + (b % 2) for b in range(-3, 4)}     # an edge, past range
    rets = scan_returns(_scan(far))
    assert straddles_edge(rets, 0.0, 0.3)
    for gap in (0.3, None):
        i = Inventory(edge_gap_m=gap)
        captured = []
        i._misses = lambda pose, returns, pan, fov, joined, unplaced=(): captured.append(unplaced)
        i.observe(MUG, _pose(), _scan(far))
        assert i.landmarks == [] and i.unplaced == 1
        assert captured == [[]]            # spares nothing, key on or off


# ---------- the recorded sweeps (evaluations/phantom-landmark-3204/B/) ----------

EVAL = Path(__file__).resolve().parents[1] / "evaluations" / "phantom-landmark-3204"


def _score(name):
    import importlib.util
    spec = importlib.util.spec_from_file_location("phantom_sweep_3204", EVAL / "sweep.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.score(EVAL / name)


def test_e5_key_off_is_a_row_for_row():
    """E5: the key-off rows on seed 3462 are 3.204.A's dev rows, byte for byte."""
    for arm in ("clean", "noisy"):
        assert (EVAL / "B" / f"off-{arm}-3462.jsonl").read_bytes() == \
            (EVAL / f"dev-{arm}.jsonl").read_bytes()


# (gap, phantoms, phantoms reported, true carried, true reported, recall found of 894)
TUNING = [(0.2, 4, 0, 1575, 1161, 850), (0.3, 4, 0, 1585, 1172, 850),
          (0.5, 4, 0, 1595, 1176, 860), (0.8, 5, 0, 1593, 1189, 868)]


@pytest.mark.parametrize("gap, ph, ph_rep, true, true_rep, found", TUNING)
def test_tuning_seed_3462_as_recorded(gap, ph, ph_rep, true, true_rep, found):
    """The tuning arms (clean, seed 3462): phantoms fall from 50 to 4-5 at
    every gap, but recall (found of 894) stays under E2's 0.979 at every one:
    tuning FAILED as the bars wrote it."""
    s = _score(f"B/tune-clean-edge{gap}.jsonl")
    assert (s["phantoms"], s["phantoms_reported"], s["true_carried"], s["true_reported"],
            s["found"], s["seen"]) == (ph, ph_rep, true, true_rep, found, 894)
    off = _score("dev-clean.jsonl")
    assert off["recall_mean"] - s["recall_mean"] > 0.01          # E2 fails
