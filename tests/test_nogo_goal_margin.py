# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.199: a person's goal beside a no-go zone is refused within
`world/nogo.GOAL_CLEAR_M`, where the bridge's grown mask (3.195) may have
drawn its cell; clear of that, taken exactly as before.

3.195 grew each drawn zone by 0.05 m, so a drawn cell reaches 0.121 m from
the zone, while D5 (3.161) still refused only the zone itself: a tap in the
margin landed in a lethal cell and NavFn's 0.10 m tolerance had to find a
free one, at the very end of its float loop.
"""

import importlib.util
import json
import math
from pathlib import Path

import pytest

from world import nogo
from tests.test_no_go import FRAME, ZONE, zoned_server  # noqa: F401  (the fixture)

ROOT = Path(__file__).resolve().parent.parent
EVAL = ROOT / "evaluations/nogo-3199"


def _goals():
    spec = importlib.util.spec_from_file_location("goals_3199", EVAL / "goals.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _tap(client, x, y, driver="twin-dpad"):
    return client.post("/world/goal", json={"x_m": x, "y_m": y}, headers={"x-driver": driver}).json()


# ZONE is x 2..3, y 0.5..1.5 (house frame); taps beside its east edge and off
# its north-east corner.
BESIDE = [0.001, 0.02, 0.05, 0.08, 0.10, 0.12]


def test_a_tap_beside_a_zone_is_refused_as_inside_one(zoned_server):
    """G1 (red on dev: these taps were taken): along an edge and off a corner,
    1-120 mm out, refused `no_go_zone` and never sent to nav2."""
    client, bridge, _ = zoned_server
    z = nogo.clean_zone(ZONE)
    client.put("/world/nogo", json={"frame_id": FRAME, "zones": [z]})
    taps = [(3.0 + d, 1.0) for d in BESIDE] + [(3.0 + d / math.sqrt(2), 1.5 + d / math.sqrt(2))
                                               for d in BESIDE]
    for x, y in taps:
        r = _tap(client, x, y)
        assert r["accepted"] is False and r["reason"] == "no_go_zone", (x, y, r)
        assert r["retryable"] is True and r["zone"] == z["id"]
    assert bridge.goal is None


def test_a_tap_clear_of_the_margin_is_taken_as_before(zoned_server):
    """G3: 0.13 m out (past GOAL_CLEAR_M) along the edge and off the corner,
    the tap reaches the bridge, as on dev."""
    client, bridge, _ = zoned_server
    client.put("/world/nogo", json={"frame_id": FRAME, "zones": [nogo.clean_zone(ZONE)]})
    for x, y in ((3.13, 1.0), (3.0 + 0.13 / math.sqrt(2), 1.5 + 0.13 / math.sqrt(2))):
        bridge.goal = None
        r = _tap(client, x, y)
        assert r["accepted"] is True, r
        assert bridge.goal is not None


def test_the_band_edge_is_goal_clear_m():
    z = nogo.clean_zone(ZONE)
    g = nogo.GOAL_CLEAR_M
    assert nogo.goal_zone([z], 3.0 + g - 1e-9, 1.0) is z
    assert nogo.goal_zone([z], 3.0 + g + 1e-9, 1.0) is None
    assert nogo.goal_zone([z], 2.5, 1.0) is z                  # inside, as 3.161
    assert nogo.goal_zone([], 3.0, 1.0) is None


def test_a_brain_goal_in_the_margin_is_not_refused_for_a_zone(zoned_server):
    """G5: autonomy is never refused here (3.161's Thermos finding: the trip
    home over a zone by the dock)."""
    client, _, _ = zoned_server
    client.put("/world/nogo", json={"frame_id": FRAME, "zones": [nogo.clean_zone(ZONE)]})
    assert _tap(client, 3.05, 1.0, driver="brain").get("reason") != "no_go_zone"


def test_a_dpad_press_in_the_margin_drives(zoned_server):
    """G5: the robot (at (1, 1)) 0.05 m from a zone, in its margin: the D-pad
    is never refused for a zone (rule 4)."""
    client, _, _ = zoned_server
    client.put("/world/nogo", json={"frame_id": FRAME, "zones": [
        nogo.clean_zone({"x0": 1.05, "y0": 0.5, "x1": 2.0, "y1": 1.5})]})
    for verb in ("FORWARD", "LEFT", "REVERSE"):
        body = client.post("/action", json={"action": verb}, headers={"x-driver": "twin-dpad"}).json()
        assert "no_go" not in json.dumps(body), body


def test_the_simulation_holds_and_dev_fails_it():
    """G2-G3 on a fresh seed (800 goals each way): 3.199's rule never lets a
    goal land in a drawn cell; dev's rule does; past GOAL_CLEAR_M the rules
    agree."""
    g = _goals()
    near = g.run(800, seed=99, d_lo=0.0, d_hi=0.15)
    assert near["3.199"]["in_drawn"] == 0 and near["3.199"]["fails"] == 0
    assert near["dev"]["in_drawn"] > 0 and near["dev"]["refused"] == 0
    far = g.run(800, seed=98, d_lo=nogo.GOAL_CLEAR_M, d_hi=1.0)
    assert far["same_answer"] == 800 and far["3.199"]["refused"] == 0


def test_navfn_float_loop_misses_its_far_edge_on_some_goals():
    """The model's NavFn: with a free point only at +tolerance, the float
    loop visits it for some goals and steps past it for others (rounding),
    as the planner does -- every dev failure in summary.json is one of these
    (the 5 x 5 points on exact steps would all have found a free cell)."""
    g = _goals()
    found = []
    for k in range(400):
        gx = 0.37 + 0.123 * k
        free_far = lambda x, y, gx=gx: x >= gx + 0.1 - 1e-9          # noqa: E731
        found.append(g.navfn_plans(lambda x, y, f=free_far: not f(x, y), gx, 0.5))
    assert True in found and False in found
    assert not g.navfn_plans(lambda x, y: True, 1.0, 0.5)
    assert g.navfn_plans(lambda x, y: x < 0.95, 1.0, 0.5)             # -tolerance is always visited


def test_the_recorded_summary_is_pinned():
    """G2-G3 as recorded (summary.json, seed 3199 / 3200, 7,200 goals each)."""
    s = json.loads((EVAL / "summary.json").read_text())
    assert s["goal_clear_m"] == nogo.GOAL_CLEAR_M
    near, far = s["near"], s["far"]
    assert near["n"] == 7200 and far["n"] == 7200
    assert near["3.199"]["in_drawn"] == 0 and near["3.199"]["fails"] == 0
    assert near["dev"]["in_drawn"] == 3905 and near["dev"]["fails"] == 28
    assert far["same_answer"] == 7200 and far["dev"]["refused"] == 0 == far["3.199"]["refused"]


def test_the_recorded_summary_is_what_the_code_gives_today():
    """The pin is not a stale copy: the near set rerun gives the same counts."""
    g = _goals()
    s = json.loads((EVAL / "summary.json").read_text())
    near = g.run(s["near"]["n"], seed=s["near"]["seed"], d_lo=0.0, d_hi=0.15)
    assert near == s["near"]


@pytest.mark.parametrize("d", [0.0, 0.06, 0.12, 0.124])
def test_refused_up_to_the_reach_on_any_phase_and_anchor(d):
    """C6's reach (0.121 m) is inside the band: a goal anywhere within it
    is refused before it could meet a drawn cell."""
    z = nogo.clean_zone(ZONE)
    assert nogo.goal_zone([z], 3.0 + d, 1.0) is z
