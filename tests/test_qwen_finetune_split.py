# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.188.A: the leave-one-object-out folds and the stock engine's numbers
they are judged against, pinned. Committed records only; no frames."""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DIR = ROOT / "evaluations" / "qwen-finetune-3188"


def _split():
    spec = importlib.util.spec_from_file_location("split_3188", DIR / "split.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_committed_split_is_what_the_script_computes(tmp_path, monkeypatch):
    m = _split()
    monkeypatch.setattr(m, "HERE", tmp_path)
    fresh = json.loads(json.dumps(m.main()))
    assert fresh == json.loads((DIR / "split.json").read_text())


def test_folds_hold_out_whole_objects_and_cover_the_set():
    s = json.loads((DIR / "split.json").read_text())
    assert s["frames"] == 2261 and s["set_hash"] == "163dec93a76a"
    assert len(s["objects"]) == 6
    held = sum(f["stock"]["frames"] for f in s["folds"].values())
    assert held == s["frames"]
    for f in s["folds"].values():
        assert f["train_frames"] + f["stock"]["frames"] == s["frames"]
    walks = [w for f in s["folds"].values() for w in f["held_out_walks"]]
    assert len(walks) == len(set(walks)) == 21      # no walk in two folds


def test_stock_numbers_match_3_92():
    s = json.loads((DIR / "split.json").read_text())
    p = s["stock_pooled"]
    assert (p["yes"], p["false_yes"], p["precision"], p["recall"]) == (508, 26, 0.9488, 0.9009)
    worst = s["folds"]["grey backpack"]["stock"]
    assert (worst["false_yes"], worst["false_no"], worst["precision"]) == (12, 15, 0.8681)
