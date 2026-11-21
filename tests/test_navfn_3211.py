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
