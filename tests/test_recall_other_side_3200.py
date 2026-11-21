# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.200.B's recorded sweeps, pinned (CLAUDE.md 7.3): the scorer over the
committed rows gives the numbers the plan file records
(docs/plans/ros-alignment/3.200-recall-other-side.md). The rule itself did not
ship; its unit tests live with its code at `d82bcb5f`."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
D = ROOT / "evaluations/recall-other-side-3200"
_spec = importlib.util.spec_from_file_location(
    "find_last_seen_370", ROOT / "evaluations/find-last-seen-370/sweep.py")
sweep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sweep)


def _score(name):
    with contextlib.redirect_stdout(io.StringIO()):
        return sweep.score(D / f"rows-{name}.jsonl")


def _rows(name):
    with open(D / f"rows-{name}.jsonl") as f:
        rows = [json.loads(line) for line in f if line.strip()]
    return rows[0], {(r["arm"], r["k"]): r for r in rows[1:]}


def _ok(r):
    return r["outcome"] == "found" and not r["wrong_found"]


def test_found_recall_first_and_stability_as_recorded():
    want = {"dev-inflated": (7, 6, 17), "R1-inflated": (7, 6, 17), "R2-inflated": (6, 6, 17),
            "dev-fake": (19, 20, 20), "R1-fake": (20, 20, 20), "R2-fake": (20, 20, 20)}
    for name, (sv, un, first) in want.items():
        s = _score(name)
        assert (s["found"]["surveyed"], s["found"]["unsurveyed"]) == (sv, un), name
        assert s["first_goal"]["recall_first"] == first, name
        st = s["stability"]
        assert (st["wrong_found"], st["contacts"], st["capped"], st["met"]) == (0, 0, 0, True), name


def test_a1_failed_and_paired_losses():
    """A1 (>= 9) failed for both; A3: R1 lost none, R2 lost k 11; FakeNav k 11 won."""
    for nav, rule, lost, won in (("inflated", "R1", [], []), ("inflated", "R2", [11], []),
                                 ("fake", "R1", [], [11]), ("fake", "R2", [], [11])):
        _, dev = _rows(f"dev-{nav}")
        _, r = _rows(f"{rule}-{nav}")
        s = [k for (a, k) in r if a == "surveyed"]
        assert sorted(k for k in s if _ok(dev[("surveyed", k)]) and not _ok(r[("surveyed", k)])) == lost
        assert sorted(k for k in s if not _ok(dev[("surveyed", k)]) and _ok(r[("surveyed", k)])) == won
    assert _score("R1-inflated")["found"]["surveyed"] < 9 and _score("R2-inflated")["found"]["surveyed"] < 9


def test_no_face_refusal_under_inflated_nav_and_the_cap():
    for nav in ("inflated", "fake"):
        for rule in ("R1", "R2"):
            _, r = _rows(f"{rule}-{nav}")
            assert max(x["recall_most_per_place"] for x in r.values()) <= 2
            assert max(x["recall_goals"] for x in r.values()) <= 6
    _, r1 = _rows("R1-inflated")
    assert sum(x["recall_retries"] for x in r1.values()) == 0          # nothing for R1 to act on
    assert sum(x["recall_unlooked"] for x in r1.values()) == 15        # all navigator failures
    _, r2 = _rows("R2-inflated")
    assert sum(x["recall_retries"] for x in r2.values()) == 15


def test_a6_surveys_identical_and_unsurveyed_differ_only_in_search_s():
    for nav in ("inflated", "fake"):
        dsv, dev = _rows(f"dev-{nav}")
        for rule in ("R1", "R2"):
            sv, r = _rows(f"{rule}-{nav}")
            assert {k: v for k, v in sv.items() if k != "rule"} == {k: v for k, v in dsv.items() if k != "rule"}
            for key, x in r.items():
                if key[0] != "unsurveyed":
                    continue
                d = dev[key]
                assert all(x[f] == d[f] for f in ("outcome", "goals", "steps", "distance_m"))
                assert abs(x["search_s"] - d["search_s"]) <= 0.2 + 1e-9
