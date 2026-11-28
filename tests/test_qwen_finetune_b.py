# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.188.B (amendment 2): the fine-tune's training rows, the job's
question and adapter placement, the launcher's $10 stop and the scorer,
pinned on committed records. No frame, no model, no network."""

import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DIR = ROOT / "evaluations" / "qwen-finetune-3188"


def _load(name):
    spec = importlib.util.spec_from_file_location(f"{name}_3188_test", DIR / f"{name}.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert str(DIR) not in sys.path
    return m


@pytest.fixture(scope="module")
def bm():
    return _load("b_manifest")


@pytest.fixture(scope="module")
def built(bm):
    return bm.build()


@pytest.fixture(scope="module")
def job():
    return _load("b_job")


@pytest.fixture(scope="module")
def launch():
    return _load("b_launch")


@pytest.fixture(scope="module")
def bs():
    return _load("b_score")


def test_the_training_rows_are_amendment_twos(bm, built):
    c = bm.counts(built)
    assert c["pool"] == 2040
    assert c["left_out"] == {"test draw": 200, "in view, no kept label": 21}
    assert c["stock_asked"] == 2468 and c["test"] == 200
    trains = {f: c[f]["train"] for f in bm.FOLDS}
    assert trains == {"1": 3804, "2": 4012, "3": 3622, "4": 3732, "5": 2756, "6": 2474, "all": 4080}
    assert {f: c[f]["asked"] for f in bm.FOLDS} == {
        "1": 212, "2": 87, "3": 255, "4": 243, "5": 701, "6": 970, "all": 441}
    assert {f: c[f]["held_out"] for f in bm.FOLDS[:-1]} == dict(zip(bm.FOLDS, json.loads(
        (DIR / "split.json").read_text())["objects"]))


def test_nothing_held_out_is_trained_on(bm, built):
    test = set(built["test"])
    labels = {r["key"]: r for r in json.loads(bm.cl.LABELS.read_text())["rows"]}
    frames = bm.mf.build()["frames"]
    for fold, rec in built["folds"].items():
        keys = {t["key"] for t in rec["train"]}
        assert not keys & test, fold
        assert not any("taupe" in frames[k]["sets"] for k in keys)
        if rec["held_out"]:
            assert all(bm.s3214.object_of(k) != rec["held_out"] for k in keys), fold
        for t in rec["train"]:
            if frames[t["key"]]["visible"]:
                assert labels[t["key"]]["kept"] and t["answer"] != "[]"
            else:
                assert t["answer"] == "[]"
        # Every frame twice: as it is and mirrored.
        assert sum(t["mirror"] for t in rec["train"]) * 2 == len(rec["train"])
    # Each fold model is asked about every frame of its object, never one it saw.
    for f in bm.FOLDS[:-1]:
        rec = built["folds"][f]
        assert not {t["key"] for t in rec["asked"]} & {t["key"] for t in rec["train"]}


def test_a_label_and_its_mirror_in_the_students_pixels(bm, built):
    # 1280 x 720 is seen as 840 x 448 (VE's pixel bounds).
    assert bm.model_size(1280, 720) == (840, 448)
    assert bm.mirrored_box([578, 196, 674, 223], (1280, 720)) == [166, 196, 262, 223]
    assert bm.answer(None) == "[]"
    assert bm.answer([578.0, 196, 674, 223]) == '[{"bbox_2d": [578, 196, 674, 223]}]'
    rows = [t for t in built["folds"]["all"]["train"]
            if t["key"] == "a-white-phone-charger-ca-20260921-203250/frame-0014.jpg"]
    assert [t["answer"] for t in rows] == ['[{"bbox_2d": [578, 196, 674, 223]}]',
                                           '[{"bbox_2d": [166, 196, 262, 223]}]']


def test_the_job_asks_the_students_own_question_and_imports_nothing(job):
    from tools.jetson.bench_identity_llama import PROMPT
    assert job.PROMPT == PROMPT
    assert (job.MIN_PIXELS, job.MAX_PIXELS) == (128 * 28 * 28, 512 * 28 * 28)
    src = (DIR / "b_job.py").read_text()
    assert not re.search(r"^\s*(from|import)\s+(brain|control|robot|sim|world|service|tools)\b", src, re.M)
    assert job.SEED == 3188 and job.LR == 2e-4 and (job.LORA_R, job.LORA_ALPHA) == (16, 32)


def test_the_adapter_lands_on_the_language_model_only(job):
    pat = re.compile(job.LORA_TARGETS)
    for name in ("model.layers.0.self_attn.q_proj", "model.layers.35.mlp.down_proj",
                 "base_model.model.model.layers.3.self_attn.o_proj",
                 "model.language_model.layers.2.mlp.gate_proj"):
        assert pat.fullmatch(name), name
    for name in ("visual.blocks.0.mlp.gate_proj", "visual.blocks.0.attn.qkv", "visual.merger.mlp.0",
                 "base_model.model.visual.blocks.1.mlp.up_proj", "lm_head", "model.embed_tokens",
                 "model.layers.0.self_attn.q_proj.lora_A"):
        assert not pat.fullmatch(name), name


def test_only_the_reply_is_learned(job):
    assert job.reply_labels([1, 2, 3, 9, 8], [9, 8]) == [-100, -100, -100, 9, 8]
    with pytest.raises(ValueError):
        job.reply_labels([1, 2, 3, 9, 8], [3, 8])
    with pytest.raises(ValueError):
        job.reply_labels([1, 2], [])
    assert job.plan_steps(3804) == (238, 12)
    e = job.lp_entry(0, "[", -0.1, [("[", -0.1), ("[]", -2.5)])
    assert e["top_logprobs"][1] == {"token": "[]", "logprob": -2.5}
    assert job.lp_entry(job.TOP_FOR, "x", -0.1, [("x", -0.1)])["top_logprobs"] == []


def test_links_travel_as_secrets_and_the_stop_is_ten_dollars(launch):
    links = {"stamp": "s", "JOB_URL": "J", "MANIFEST_URL": "M",
             "runs": {"stock": "S", "pilot": "P", "rest": "R"}}
    spec = launch.job_spec("pilot", links, "a100-large", 50)
    assert spec["secrets"] == {"JOB_URL": "J", "MANIFEST_URL": "M", "RESULT_PUT_URL": "P"}
    assert spec["environment"] == {"FOLDS": "1"}
    assert not any(v in json.dumps(spec["environment"]) + json.dumps(spec["command"])
                   for v in ("J", "M", "P") if len(v) > 1)
    assert launch.job_spec("rest", links, "a100-large", 9)["environment"] == {"FOLDS": "2,3,4,5,6,all"}
    assert launch.job_spec("stock", links, "a100-large", 9)["environment"] == {}
    assert launch.STOP_USD == 10.0
    assert launch.bill_of(61, 0.041667) == round(2 * 0.041667, 4)
    jobs = [{"usd": 1.0, "worst_usd": 2.0}, {"usd": None, "worst_usd": 3.0}]
    assert launch.spent(jobs) == 4.0
    launch.check_budget(jobs, 6.0)
    with pytest.raises(SystemExit):
        launch.check_budget(jobs, 6.01)


def _ve_answers(bs):
    """VE's committed answers standing in for every model: the scorer must
    reproduce 3.188.A's and 3.206.A's numbers and fail the progress bars."""
    ve = {r["key"]: r for r in bs.mf._jsonl(ROOT / "evaluations/tier-calibration-392/qwen-VE.jsonl")}
    sizes = bs.bm.s3214_sizes()
    b = bs.bm.build()

    def one(t, model):
        r, size = ve.get(t["key"]), sizes.get(t["key"], (1280, 720))
        yes = bool(r and r["status"] == "answered" and (r["score"] or 0) >= 0.5)
        box = r["box"] if r else None
        if box and t["mirror"]:
            box = bs.bm.mirrored_box(box, size)
        return {"id": t["id"], "model": model, "status": "answered", "score": 0.0,
                "yes": yes, "box": box, "size": list(size)}
    out = [one(t, "stock") for t in b["stock"]]
    for f, rec in b["folds"].items():
        out += [one(t, f"fold-{f}") for t in rec["asked"]]
    return out


def test_the_scorer_reproduces_the_recorded_baselines(bs):
    out = bs.scores(_ve_answers(bs))
    assert all(c["whole"] for c in out["coverage"].values())
    assert out["stock"]["pooled"] == out["stock"]["ve_pooled"]
    assert (out["stock"]["pooled"]["false_yes"], out["stock"]["pooled"]["missed"]) == (26, 53)
    local = out["fold_steering"]["local"]
    assert (local["right"], local["silent"], local["wrong_side"]) == (139, 19, 5)   # 3.206.A
    assert out["criteria_3188"]["1 no object worse"]
    assert not out["criteria_3188"]["2 mistakes <= 0.75 x stock's"]
    assert not out["criteria_3188"]["5 teal 0 of 34, taupe 0 of 7"]          # VE: teal 1
    assert not out["criteria_3206"]["5 local order right >= 90% and above A by its half-width"]
    assert out["criteria_3206"]["8 no object's local order right count below A's"]
    assert out["all_objects_reported"]["taupe_yes"] == 0


def test_a_partial_run_judges_nothing(bs):
    answers = _ve_answers(bs)
    out = bs.scores([x for x in answers if x["model"] != "fold-4"])
    assert "criteria_3188" not in out and not out["coverage"]["fold-4"]["whole"]
    err = [dict(x, status="error") if x["model"] == "stock" and x["id"].endswith("0000.jpg") else x
           for x in answers]
    assert "stock" not in bs.scores(err)


def test_a_reply_is_read_as_ves_is(bs):
    box_lp = [{"token": "[{\"", "logprob": -0.05,
               "top_logprobs": [{"token": "[{\"", "logprob": -0.05}, {"token": "[]", "logprob": -3.0}]}]
    row = {"id": "k", "key": "k", "model": "fold-1", "size": [1280, 720], "mirror": False,
           "text": '[{"bbox_2d": [700, 10, 800, 90]}]', "lp": box_lp}
    got = bs.answer(row)
    assert got["yes"] and got["box"] == [700.0, 10.0, 800.0, 90.0]
    assert bs.box_third(got) == "RIGHT"
    assert bs.box_third({**got, "yes": False}) is None
    assert bs.answer({**row, "error": "boom"})["status"] == "error"


def test_a_result_link_takes_one_job_and_a_rerun_gets_its_own(launch, bm):
    jobs = [{"stamp": "s1", "result": "rest", "usd": 1.0, "worst_usd": 2.0}]
    with pytest.raises(SystemExit):
        launch.check_result_link(jobs, "s1", "rest")
    launch.check_result_link(jobs, "s1", "rerun-1")
    launch.check_result_link(jobs, "s2", "rest")
    assert {"stock", "pilot", "rest", "rerun-1"} <= set(bm.RESULT_LINKS)
    links = {"stamp": "s", "JOB_URL": "J", "MANIFEST_URL": "M", "runs": {"rest": "R", "rerun-1": "X"}}
    spec = launch.job_spec("rest", links, "a100-large", 9, "4,5", "rerun-1")
    assert spec["secrets"]["RESULT_PUT_URL"] == "X" and spec["environment"] == {"FOLDS": "4,5"}


def test_a_label_past_the_image_is_clipped_before_it_is_mirrored(bm):
    box = bm.clipped([578.7, -3, 841.2, 223], (1280, 720))
    assert box == [578.7, 0, 840, 223]
    assert bm.answer(bm.mirrored_box(box, (1280, 720))) == '[{"bbox_2d": [0, 0, 261, 223]}]'


def test_a_later_run_adds_to_the_answers_never_over_them(bs):
    a1 = {"model": "stock", "id": "k", "status": "answered", "yes": True}
    err = {**a1, "status": "error"}
    assert bs.merged([err], [a1]) == [a1]
    assert bs.merged([a1], [{**a1, "id": "j"}])[1] == a1
    with pytest.raises(ValueError):
        bs.merged([a1], [{**a1, "yes": False}])


def test_the_job_checks_the_size_the_labels_assume(job):
    from brain.local_vlm import model_size
    for size in ((1280, 720), (640, 480), (1920, 1080), (300, 200), (720, 1280)):
        assert job.seen_size(*size) == model_size(*size)
    job.check_grid([[1, 32, 60]], [(1280, 720)])          # 840 x 448
    with pytest.raises(RuntimeError):
        job.check_grid([[1, 32, 58]], [(1280, 720)])


def test_a_start_refuses_a_stale_job_and_a_bad_timeout(launch, bm):
    with pytest.raises(SystemExit):
        launch.check_job_hash({"job_sha256": "0" * 64})
    launch.check_job_hash({"job_sha256": bm.job_sha256()})
