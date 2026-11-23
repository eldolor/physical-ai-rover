# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.206.B1 (amendment 3): the Bedrock runner's reading of a reply, its
cost, its torn-line tolerance and the pilot gate -- no AWS call is made."""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("b1_3206", ROOT / "evaluations/steer-distill-3206/b1_bedrock.py")
b1 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(b1)

T = {"id": "w/f.jpg", "key": "w/f.jpg", "set": "S1", "target": "x", "mirror": False}


def got(text, stop="end_turn", blocks=None):
    content = blocks if blocks is not None else [{"text": text}]
    return {"output": {"message": {"content": content}}, "stopReason": stop,
            "usage": {"inputTokens": 1000, "outputTokens": 40}}


def test_tasks():
    ts = b1.tasks()
    assert len(ts) == 200 and sum(t["mirror"] for t in ts) == 100
    assert [t["set"] for t in ts].count("S1") == 120
    assert b1.tasks(pilot=True) == ts[:2] and ts[1]["id"] == ts[0]["id"] + "#mirror"


def test_reply_row_reads_the_box_in_0_1000():
    r = b1.reply_row(T, got('```json\n[\n\t{"bbox_2d": [872, 442, 999, 516], "label": "x"}\n]\n```'))
    assert r["status"] == "answered" and abs(r["frac_x"] - 0.9355) < 1e-9 and r["coords_ok"]
    assert b1.reply_row(T, got("[]"))["status"] == "answered"          # no box: an answer of none
    assert b1.reply_row(T, got("[]"))["frac_x"] is None


def test_reply_row_refuses_what_is_not_an_answer():
    cut = got('```json\n[\n\t{"bbox_2d": [872, 442, 9', stop="max_tokens")
    assert b1.reply_row(T, cut)["status"] == "truncated"
    assert b1.reply_row(T, got("", blocks=[{"reasoningContent": {}}]))["status"] == "no_text"
    pix = b1.reply_row(T, got('[{"bbox_2d": [1100, 300, 1250, 420]}]'))
    assert pix["status"] == "out_of_range" and pix["frac_x"] is None
    two = got("", blocks=[{"reasoningContent": {}}, {"text": '[{"bbox_2d": [100, 1, 200, 2]}]'}])
    assert b1.reply_row(T, two)["status"] == "answered"                # every text block is read


def test_cost_counts_failed_calls_that_reached_bedrock():
    ok = {"in_tok": 1000, "out_tok": 40}
    assert abs(b1.cost([ok]) - (1000 * 0.53e-6 + 40 * 2.66e-6)) < 1e-12
    assert b1.cost([{"status": "error", "billed_unknown": True}]) > b1.cost([ok])
    assert b1.cost([{"status": "error", "billed_unknown": False}]) == 0


def test_load_skips_only_a_torn_last_line(tmp_path):
    p = tmp_path / "rows.jsonl"
    p.write_text(json.dumps({"id": "a", "status": "answered"}) + "\n" + '{"id": "b", "sta')
    assert [r["id"] for r in b1._load(p)] == ["a"]


def test_scores_refuse_a_partial_run():
    out = b1.scores([{**T, "id": b1.tasks()[0]["id"], "status": "answered", "frac_x": 0.9}])
    assert not out["coverage"]["whole"] and out["coverage"]["missing"] == 199


def test_b1_scores_pinned():
    """The committed run (200 calls, 2026-11-23), judged whole."""
    out = b1.scores(b1._load(b1.ROWS))
    assert out["coverage"]["whole"]
    c = out["criterion_2"]
    assert (c["kept"], c["kept_right"], c["meets_criterion_2"]) == (59, 55, False)
    assert out["s2_boxes"] == 0 and out["s1_right_without_mirror_rule"] == 56
    assert len(out["border_wrong_kept"]) == 4 and out["nonborder_wrong_kept"] == []
    assert {k: (v["kept"], v["kept_right"]) for k, v in out["pairs"].items()} == {
        "Qwen3-VL + 72B": (57, 54), "Qwen3-VL + Florence-2-large": (58, 55)}
