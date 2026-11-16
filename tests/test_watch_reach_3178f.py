# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.178.F: why passes never reach a report -- the diagnosis (tuning seeds
only) and the pass's view count, pinned. The sweeps themselves are not run
here (each is minutes); `evaluations/watch-list-3178/diagnose_f.py` and
`sweep.py` regenerate the files, and the section
(`docs/plans/ros-alignment/3.178-watch-list.md`, 3.178.F) gives the numbers.
"""
from __future__ import annotations

import importlib.util
import json
import sys
from collections import Counter
from pathlib import Path

import pytest

from brain import inventory as inv

HERE = Path(__file__).resolve().parents[1] / "evaluations" / "watch-list-3178"


def _load(name: str, file: str):
    """An evaluation script under a unique module name (other evaluations
    have a `sweep.py` too); nothing goes on sys.path."""
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, HERE / file)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        try:
            spec.loader.exec_module(mod)
        except BaseException:
            del sys.modules[name]          # never cache a half-run module
            raise
    return sys.modules[name]


sw = _load("watch_sweep_3178", "sweep.py")

ONE_VIEW = "one viewpoint"
PLACEMENT = "placement"
FUSION = "fusion"


def _lost(seed: int) -> list:
    d = json.loads((HERE / f"diagnose-f-{seed}.json").read_text())
    assert d["steps"] == 150 and d["seed"] == seed
    return [(key, m["mission"], e) for key in ("search", "present") for m in d[key]
            for e in m["lost"]]


def test_the_diagnosis_reproduces_es_tuning_table():
    """E's sized-first table: B's seeds 40 passes / 37 reached, D's 42 / 38."""
    for seed, passes, reached in ((3178, 40, 37), (4178, 42, 38)):
        d = json.loads((HERE / f"diagnose-f-{seed}.json").read_text())
        rows = d["search"] + d["present"]
        assert len(rows) == 40
        assert sum(m["passes"] for m in rows) == passes
        assert sum(m["reached"] for m in rows) == reached
        assert sum(len(m["lost"]) for m in rows) == passes - reached


def test_the_seven_lost_passes_and_their_causes():
    lost = {(seed, key, m, e["item"]): e for seed in (3178, 4178) for key, m, e in _lost(seed)}
    causes = {k: e["cause"].split(":")[0] for k, e in lost.items()}
    assert causes == {
        (3178, "search", 8, "wallet"): PLACEMENT,
        (3178, "search", 17, "glasses"): PLACEMENT,      # then fused into a dresser
        (3178, "present", 3, "phone"): ONE_VIEW,
        (4178, "search", 20, "wallet"): ONE_VIEW,
        (4178, "present", 3, "phone"): FUSION,
        (4178, "present", 8, "wallet"): PLACEMENT,
        (4178, "present", 17, "glasses"): ONE_VIEW,
    }
    assert Counter(causes.values()) == {ONE_VIEW: 3, PLACEMENT: 3, FUSION: 1}
    # A one-view loss had one view of every true sighting out to MAX_RANGE_M.
    for k, e in lost.items():
        assert (e["true_views"] < 2) == (causes[k] == ONE_VIEW)
    # The ranging losses: a view placed on a NEARER surface, > 0.5 m off.
    for k in ((3178, "search", 8, "wallet"), (3178, "search", 17, "glasses"),
              (4178, "present", 8, "wallet")):
        far = [r for f in lost[k]["frames"] for r in f["rows"]
               if r["to_inst"] is not None and r["to_inst"] > sw.PIN_M]
        assert far and all(r["range_m"] < r["inst_range"] for r in far)
    # The fusions: the item's own looks went to another label's landmark.
    phone = lost[(4178, "present", 3, "phone")]["landmarks"]
    assert [(lm["label"], lm["votes"]) for lm in phone] == [
        ("headphones", {"headphones": 2, "mug": 1, "phone": 1})]
    glasses = lost[(3178, "search", 17, "glasses")]["landmarks"]
    assert [(lm["label"], lm["votes"]) for lm in glasses] == [
        ("dresser", {"dresser": 1, "glasses": 1})]
    # No loss was the erring detector's alone: none took a pass the camera saw
    # from two views below two.
    assert not [k for k, e in lost.items() if e["true_views"] >= 2 > e["kept_views"]]


def test_views_is_the_inventorys_new_view_rule():
    p = {"x_m": 1.0, "y_m": 1.0, "heading_deg": 0.0}
    assert sw.views([]) == 0
    assert sw.views([p, dict(p)]) == 1                                   # one pose, one view
    assert sw.views([p, {**p, "x_m": 1.0 + inv.NEW_VIEW_M}]) == 2         # moved
    assert sw.views([p, {**p, "heading_deg": inv.NEW_VIEW_DEG}]) == 2     # turned
    assert sw.views([p, {**p, "x_m": 1.0 + inv.NEW_VIEW_M * 0.9}]) == 1
    assert sw.MIN_VIEWS == 2
    # Two hits are what reach a report, and one is not (MIN_VIEWS's reason).
    assert inv._sigmoid(inv.L_HIT) < inv.REPORT_BELIEF <= inv._sigmoid(2 * inv.L_HIT)


def test_score_mission_counts_two_view_passes():
    insts = sw.instances()
    keys = sw.item_instances(insts)["keys"][0]
    x0, y0 = [(c[0] + 0.5) * 0.3 for c in insts[keys][1]][0], [(c[1] + 0.5) * 0.3 for c in insts[keys][1]][0]
    pin = {"x_m": x0, "y_m": y0, "belief": 0.85}
    r = {"nearest": {keys: 1.0}, "views": {keys: 1},
         "watch": {"label": {"pins": {"keys": [pin]}}},
         "reported": [{"label": "keys", "x_m": x0, "y_m": y0, "belief": 0.85}]}
    one = sw.score_mission(r, "label", insts)
    assert one["passes"] == [keys] and one["reached"] == [keys]
    assert one["passes_2v"] == one["reached_2v"] == one["found_2v"] == []
    two = sw.score_mission({**r, "views": {keys: 2}}, "label", insts)
    assert two["passes_2v"] == two["reached_2v"] == two["found_2v"] == [keys]
    # A row with no `views` (D's and E's files) counts no two-view pass.
    assert sw.score_mission({k: v for k, v in r.items() if k != "views"},
                            "label", insts)["passes_2v"] == []


@pytest.mark.parametrize("seed,passes,reached,two,reached_two,wrong_label", [
    (3178, 40, 37, 39, 37, 9),
    (4178, 42, 38, 40, 38, 9),
])
def test_tuning_numbers_are_pinned(seed, passes, reached, two, reached_two, wrong_label):
    r = json.loads((HERE / f"results-f-tune-{seed}.json").read_text())
    s, p = r["search"], r["present"]
    assert (s["seed"], p["seed"], s["max_steps"]) == (seed, seed + 1, 150)
    assert s["passes"] + p["passes"] == passes
    assert s["reached"] + p["reached"] == reached
    assert s["passes_2v"] + p["passes_2v"] == two
    assert s["reached_2v"] + p["reached_2v"] == reached_two
    assert s["rules"]["label"]["wrong_pins"] + p["rules"]["label"]["wrong_pins"] == wrong_label
    assert s["same_as_off"] + p["same_as_off"] == 40
    # A two-view pass is a pass; every row's views name its passes.
    for row in s["rows"] + p["rows"]:
        assert len(row["views"]) == len(row["passes"])
        assert len(row["passes_2v"]) == sum(v >= sw.MIN_VIEWS for v in row["views"].values())


def test_diagnosis_never_runs_on_fresh_seeds():
    diagnose_f = _load("watch_diagnose_3178f", "diagnose_f.py")
    range_rule_f = _load("watch_range_rule_3178f", "range_rule_f.py")
    assert diagnose_f.sw is sw and range_rule_f.load_sweep() is sw
    # Every fresh seed, and the tuning seed below one (its present sweep is seed + 1).
    for seed in (5177, 5178, 5179, 6177, 6178, 6179):
        with pytest.raises(SystemExit, match="tuning seeds only"):
            diagnose_f.main(["--seed", str(seed), "--out", "/dev/null"])
        with pytest.raises(SystemExit, match="tuning seeds only"):
            range_rule_f.main(["closest", "--seed", str(seed), "--out", "/tmp/x.json"])
        with pytest.raises(SystemExit, match="tuning seeds only"):     # mode last
            range_rule_f.main([f"--seed={seed}", "--out", "/tmp/x.json", "closest"])
    # Neither writes over a pinned file.
    with pytest.raises(SystemExit, match="outside"):
        range_rule_f.main(["median", "--seed", "3178", "--out", str(HERE / "results-d.json")])
    with pytest.raises(SystemExit, match="pinned"):
        diagnose_f.main(["--seed", "3178", "--out", "diagnose-f-3178.json", "--workers", "1",
                         "--missions", "0"])
    assert range_rule_f.inv.range_at is range_rule_f.orig


def test_fs_fresh_run_is_pinned():
    """3.178.F's fresh run (6178/6179, once): F1-F3 met; D3 as information."""
    r = json.loads((HERE / "results-f.json").read_text())
    s, p = r["search"], r["present"]
    assert (s["seed"], p["seed"]) == (6178, 6179)
    assert (s["missions"], p["missions"], s["max_steps"]) == (20, 20, 150)
    assert r["list_false_rate"] == sw.LIST_FALSE_RATE and len(r["placed"]) == 9
    two = s["passes_2v"] + p["passes_2v"]
    reached_two = s["reached_2v"] + p["reached_2v"]
    assert (s["passes_2v"], p["passes_2v"]) == (23, 24)
    assert two >= 30                                              # the sample bar
    assert (s["reached_2v"], p["reached_2v"]) == (23, 23)
    assert reached_two / two >= 0.90                              # F1: 46 of 47
    # E's definition, information: 46 of 53.
    assert (s["passes"], p["passes"], s["reached"], p["reached"]) == (28, 25, 23, 23)
    label = (s["rules"]["label"], p["rules"]["label"])
    votes = (s["rules"]["votes"], p["rules"]["votes"])
    assert [x["wrong_pins"] for x in label] == [3, 4]
    assert sum(x["wrong_pins"] for x in label) <= 14              # F2
    assert s["same_as_off"] + p["same_as_off"] == 40              # F3
    # D3, information only.
    assert [(x["found"], x["found_2v"]) for x in label] == [(23, 23), (23, 23)]
    assert [(x["found"], x["found_2v"]) for x in votes] == [(23, 23), (20, 20)]
    assert [x["wrong_pins"] for x in votes] == [1, 2]
    assert s["carried"] + p["carried"] == 0
    # Every pass E's way that did not reach a report but one was a glimpse.
    lost_two = 0
    for row in s["rows"] + p["rows"]:
        lost_two += len(row["passes_2v"]) - len(row["reached_2v"])
    assert lost_two == 1
