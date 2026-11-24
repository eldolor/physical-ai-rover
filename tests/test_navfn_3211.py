# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.211: NavFn's plan failures, `use_astar` true against false, pinned from
the offline probe's records (`evaluations/navfn-3211/`), and the shipped
`nav2.yaml` on the arm that passed. How the bars were written:
docs/plans/ros-alignment/3.211-navfn-plan-failures.md."""

import importlib.util
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "evaluations" / "navfn-3211"
NAV_YAML = ROOT / "service" / "slam" / "src" / "picar_bringup" / "config" / "nav2.yaml"


def _probe():
    spec = importlib.util.spec_from_file_location("navfn_probe_3211", HERE / "probe.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_records_meet_a1_to_a4():
    s = _probe().score(HERE)
    assert s["pairs"] == 840
    # A1: the defect, at 44 of 840 under A*, every failure NavFn's own error.
    assert s["astar"]["failed"] == 44 and s["astar"]["legal_rows"] == 44 and s["failed_are_legal"]
    assert s["uncharged"] == {"astar": 0, "dijkstra": 0}
    # A2-A4 under Dijkstra.
    assert s["dijkstra"]["failed"] == 0 and s["dijkstra"]["legal_lines"] == 0 and s["lost"] == []
    assert s["length_ratio_median"] <= 1.02
    assert s["dijkstra"]["planning_ms_p90"] <= 500
    assert (s["A1"], s["A2"], s["A3"], s["A4"]) == ("MET",) * 4


def test_each_error_is_charged_to_one_call():
    p = _probe()
    rows = [{"t0": 1795276000.0, "t1": 1795276000.01}, {"t0": 1795276000.02, "t1": 1795276000.03}]
    log = "x [1795276000.005] when a legal potential was found\n"
    import tempfile, json
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        (d / "rows-a.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
        (d / "ros-a.log").write_text(log)
        got, n, un = p.load_arm(d, "a")
    assert n == 1 and un == 0 and [r["legal"] for r in got] == [1, 0]


def test_nav2_ships_dijkstra():
    found = re.findall(r"\n\s*use_astar:\s*(true|false)", NAV_YAML.read_text())
    assert found == ["false"]


def test_the_live_run_meets_b2_b3():
    import json
    text = (HERE / "live-b-ros-excerpt.log").read_text()
    assert "nav2 active" in text and "legal potential" not in text          # B2
    find = next(json.loads(x) for x in (HERE / "live-b.jsonl").read_text().splitlines()
                if json.loads(x).get("row") == "find")
    assert find["contacts"] == 0 and find["outcome"] != "failed"           # B3
    assert find["ret"]["state"] == "docked"


def _path_shape():
    spec = importlib.util.spec_from_file_location("navfn_path_shape_3211", HERE / "path_shape.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_the_path_shape_records_meet_c1_to_c6():
    """3.211.C: scaled_house, three runs a side, T (A*) against F (Dijkstra)."""
    s = _path_shape().score(HERE / "path-shape")
    assert s["runs"] == s["valid"] == {"T": 3, "F": 3} and s["load_max"] <= 4.0
    assert s["params"] == {"T": ["Boolean value is: True"], "F": ["Boolean value is: False"]}
    assert s["succeeded"]["F"] == [6, 6, 6]
    assert min(s["min_clearance"]["F"]) >= 0.1155
    assert max(s["reversals_per_m"]["F"]) <= 1.0
    assert s["plans"] == {"T": 18, "F": 18}
    assert s["plan_len_ratio_median"] <= 1.02 and s["metres_ratio"] <= 1.05
    assert s["seconds_ratio"] <= 1.10
    assert [s[f"C{i}"] for i in range(1, 7)] == ["MET"] * 6


def test_the_t_arm_differs_from_the_shipped_yaml_in_use_astar_alone(tmp_path):
    t = _path_shape().arm_yaml("T", tmp_path).read_text().splitlines()
    f = NAV_YAML.read_text().splitlines()
    diff = [(a, b) for a, b in zip(f, t) if a != b]
    assert len(f) == len(t) and len(diff) == 1 and "use_astar: true" in diff[0][1]
    assert _path_shape().arm_yaml("F", tmp_path) is None


def test_a_plan_is_scored_on_truth():
    ps = _path_shape().plan_stats([[0.0, 0.0], [3.0, 4.0], [3.0, 5.0]], lambda x, y: x)
    assert ps == {"plan_len_m": 6.0, "plan_min_clear_m": 0.0, "plan_median_clear_m": 3.0,
                  "plan_points": 3}
    assert _path_shape().plan_stats([[1.0, 1.0]], lambda x, y: x) is None
