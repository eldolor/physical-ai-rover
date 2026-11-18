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


# --- 3.188.A2: the folds resolved to files (evaluations/.../resolve.py, from S3)

def _j(name):
    return json.loads((DIR / name).read_text())


def test_folds_resolve_to_files_and_agree_with_the_split():
    f = _j("folds.json")
    assert f["walks"] == 21 and f["frames"] == 2261
    assert f["keys_sha256_12"] == "163dec93a76a"      # the same set A pinned
    assert f["missing_on_s3"] == [] and f["label_vs_3.92"] == []
    assert all(x["agrees_with_split_json"] for x in f["folds"].values())
    for x in f["folds"].values():                      # a walk never straddles
        assert not set(x["held_out_walks"]) & set(x["train_walks"])


def test_the_299_arrival_frames_are_all_inside_the_2261():
    o = _j("overlap-299.json")
    assert (o["arrival_frames"], o["in_2261"], o["not_in_2261"], o["in_view"]) == (299, 299, 0, 74)
    folds = _j("folds.json")["folds"]
    for w, v in o["per_walk"].items():
        assert w in folds[v["fold"]]["held_out_walks"]


def test_lookalike_lists():
    l = _j("lookalikes.json")
    assert l["teal_bin"]["frames"] == 34 and l["teal_bin"]["in_2261"] == 34
    assert l["taupe_bag"]["frames"] == 7 and l["taupe_bag"]["in_2261"] == 0
    assert l["taupe_bag"]["on_s3"] == 7
    teal = {k.split("/")[0] for k in l["teal_bin"]["keys"]}
    assert teal == {"blue-bottle-20260907-185007"} and len(set(l["teal_bin"]["keys"])) == 34
    assert {k.split("/")[0] for k in l["taupe_bag"]["keys"]} == {"red-backpack-gpt-6-astra-20261021-205737"}
    frames = _j("frames.json")["walks"]
    for k in l["teal_bin"]["keys"]:                   # every one is a labelled "not visible"
        w, f = k.split("/")
        assert frames[w]["frames"][f]["visible"] is False
