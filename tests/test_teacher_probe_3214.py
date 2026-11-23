# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.214: the teacher probe's frame list, question and scorer, pinned on
committed records. No frame, no model, no network."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DIR = ROOT / "evaluations" / "teacher-probe-3214"


def _load(name):
    spec = importlib.util.spec_from_file_location(f"{name}_3214", DIR / f"{name}.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert str(DIR) not in sys.path and "manifest" not in sys.modules
    return m


@pytest.fixture(scope="module")
def mf():
    return _load("manifest")


@pytest.fixture(scope="module")
def score():
    return _load("score")


@pytest.fixture(scope="module")
def job():
    return _load("job")


def test_the_sets_are_the_pinned_ones(mf):
    man = mf.build()
    assert man["counts"] == {"I2261": 2261, "I299": 299, "teal": 34, "taupe": 7, "S1": 60, "S2": 40}
    pinned = json.loads((mf.FT / "frames.json").read_text())
    keys = {f"{w}/{f}" for w, rec in pinned["walks"].items() for f in rec["frames"]}
    assert {k for k, v in man["frames"].items() if "I2261" in v["sets"]} == keys
    # The lookalikes are frames WITHOUT the target; the taupe ones lie outside I-2261.
    assert not any(v["visible"] for v in man["frames"].values() if set(v["sets"]) & {"teal", "taupe"})
    assert all(v["sets"] == ["taupe"] for v in man["frames"].values() if "taupe" in v["sets"])
    # 3.80.I asked its steering frames with the walk's own target text.
    assert all(s["target"] == man["frames"][k]["target"] for k, s in man["steer"].items())
    assert sum(v["visible"] for v in man["frames"].values() if "I299" in v["sets"]) == 74


def test_each_model_is_asked_what_the_section_says(mf):
    man = mf.build()
    qwen, sup = mf.tasks(man, "qwen"), mf.tasks(man, "super")
    assert len(qwen) == 2261 + 7 + 100            # every frame once; S1 and S2 again mirrored
    assert sum(t["mirror"] for t in qwen) == sum(t["mirror"] for t in sup) == 100
    asked = {t["key"] for t in sup}
    assert all(set(man["frames"][k]["sets"]) & {"I299", "teal", "taupe", "S1", "S2"} for k in asked)
    assert len({t["id"] for t in qwen}) == len(qwen)          # ids are unique
    assert [t["id"] for t in qwen] == [t["id"] for t in mf.tasks(man, "qwen")]   # a fixed order


def test_staging_refuses_to_write_links_into_the_repo(mf):
    with pytest.raises(SystemExit, match="outside the repo"):
        mf.main(["--stage", "1", "--links", str(DIR / "links.json")])
    with pytest.raises(SystemExit, match="outside the repo"):
        mf.main(["--stage", "1"])
    with pytest.raises(SystemExit, match="outside the repo"):       # the main checkout too
        mf.main(["--stage", "1", "--links", str(ROOT / "links.json")])


def test_the_job_asks_the_students_own_question(job):
    from tools.jetson import bench_identity_llama as bench

    assert (job.PROMPT, job.MAX_TOKENS, job.TOP_LOGPROBS) == (
        bench.PROMPT, bench.MAX_TOKENS, bench.TOP_LOGPROBS)
    body = job.qwen_body(b"jpeg", "blue bottle")
    kinds = [c["type"] for c in body["messages"][0]["content"]]
    assert kinds == ["image_url", "text"]                      # the image, then the question
    assert body["messages"][0]["content"][1]["text"] == bench.PROMPT.format(target="blue bottle")
    assert body["temperature"] == 0 and body["logprobs"] and body["top_logprobs"] == 20
    assert (job.MIN_PIXELS, job.MAX_PIXELS) == (128 * 784, 512 * 784)


def test_the_scorer_reproduces_ves_recorded_numbers(score, mf):
    ve = score.ve_answers()
    pooled = score.identity(ve)
    assert (pooled["yes"], pooled["false_yes"], pooled["missed"], pooled["mistakes"]) == (508, 26, 53, 79)
    assert (pooled["precision"], pooled["recall"]) == (0.9488, 0.9009)
    table = {o: (r["frames"], r["in_view"], r["false_yes"], r["missed"])
             for o, r in score.per_object(ve).items()}
    assert table == {                                           # 3.188.A's table
        "a white phone charger cable": (176, 118, 12, 2),
        "blue and yellow running shoes": (63, 38, 1, 6),
        "blue bottle": (242, 28, 1, 0),
        "grey backpack": (211, 94, 12, 15),
        "red backpack": (678, 33, 0, 3),
        "woven laundry basket": (891, 224, 0, 27),
    }
    man = mf.build()
    i299 = score.identity([a for a in ve if "I299" in man["frames"][a["key"]]["sets"]])
    assert (i299["mistakes"], i299["precision"], i299["recall"]) == (6, 0.9722, 0.9459)


def _reply(*tokens):
    """A chat-completions `logprobs.content`: each token certain."""
    return [{"token": t, "logprob": 0.0, "top_logprobs": [{"token": t, "logprob": 0.0}]}
            for t in tokens]


def test_a_72b_reply_is_read_as_ves_is(score):
    yes = score.qwen_answer({"text": '[{"bbox_2d": [84, 100, 252, 300]}]', "mirror": False,
                             "size": [640, 480], "lp": _reply("[{", '"bbox_2d"')})
    assert yes["yes"] and yes["status"] == "answered" and yes["score"] == 1.0
    from brain.local_vlm import model_size
    assert yes["frac_x"] == pytest.approx(168 / model_size(640, 480)[0])
    no = score.qwen_answer({"text": "[]", "mirror": False, "size": [640, 480], "lp": _reply("[]")})
    assert not no["yes"] and no["score"] == 0.0 and no["frac_x"] is None
    prose = score.qwen_answer({"text": "I cannot see it.", "mirror": False, "size": [640, 480],
                               "lp": _reply("I", " cannot")})
    assert prose["status"] == "unparsed" and not prose["yes"]
    assert score.qwen_answer({"error": "HTTPError"})["status"] == "error"
    # vLLM's raw pieces (the pilot's reply): a fenced list opened by "[Ċ".
    fenced = score.qwen_answer({
        "text": '```json\n[\n    {"bbox_2d": [556, 193, 648, 217]}\n]\n```', "mirror": False,
        "size": [1280, 720], "lp": _reply("```", "json", "Ċ", "[Ċ", "ĠĠĠ", "Ġ{Ċ", 'Ġ"', "bbox")})
    assert fenced["yes"] and fenced["score"] == 1.0


def test_supers_replies_are_read_by_the_amendments(score):
    row = {"identity": {"text": "Yes, on the floor."},
           "grounding": {"text": '```json\n[{"bbox_2d": [100, 200, 300, 400], "label": "x"}]\n```'}}
    got = score.super_answer(row)
    assert got["yes"] and got["parsed_identity"] and got["parsed_grounding"]
    assert got["frac_x"] == pytest.approx(0.2)                 # 0-1000 relative (amendment 2)
    none = score.super_answer({"identity": {"text": "No."}, "grounding": {"text": "Not found."}})
    assert not none["yes"] and none["parsed_identity"] and not none["parsed_grounding"]
    vague = score.super_answer({"identity": {"text": "The scene shows a hallway."},
                                "grounding": {"text": ""}})
    assert not vague["yes"] and not vague["parsed_identity"]
    assert not score.super_answer({"identity": {"text": "Yesterday's bag."},
                                   "grounding": {"text": ""}})["yes"]
    assert score.super_box_frac('{"bbox_2d": [1.2.3, 0, 5, 6]}') is None       # not a number
    assert score.super_box_frac('{"bbox_2d": [300, 0, 100, 10]}') is None      # not a box


def test_identity_counts_and_the_disagreement_rule(score):
    rows = [{"key": "a", "visible": True, "yes": True}, {"key": "b", "visible": True, "yes": False},
            {"key": "c", "visible": False, "yes": True}, {"key": "d", "visible": False, "yes": False}]
    got = score.identity(rows)
    assert (got["false_yes"], got["missed"], got["mistakes"], got["precision"], got["recall"]) == (
        1, 1, 2, 0.5, 0.5)
    student = [{"key": "a", "visible": True, "yes": False}, {"key": "b", "visible": True, "yes": False},
               {"key": "c", "visible": False, "yes": False}, {"key": "d", "visible": False, "yes": False}]
    dis = score.disagreement(rows, student)
    assert dis == {"frames": 2, "teacher_right": 1, "share": 0.5}   # right on a, wrong on c


def test_a_steering_label_is_kept_only_when_the_mirror_swaps(score):
    steer = {"a": {"set": "S1", "truth": "LEFT"}, "b": {"set": "S1", "truth": "RIGHT"},
             "c": {"set": "S1", "truth": "FORWARD"}, "d": {"set": "S1", "truth": "LEFT"},
             "e": {"set": "S2", "truth": None}, "f": {"set": "S2", "truth": None}}
    direct = {"a": "LEFT", "b": "LEFT", "c": "FORWARD", "d": None, "e": "LEFT", "f": None}
    mirrored = {"a": "RIGHT", "b": "LEFT", "c": "FORWARD", "d": None}
    got = score.steering(direct, mirrored, steer)
    # a: swapped, kept and right. b: the same word both ways, dropped.
    # c: FORWARD both ways, kept and right. d: no answer.
    assert (got["kept"], got["kept_right"], got["answered"], got["right_all"]) == (2, 2, 3, 2)
    assert (got["kept_share"], got["kept_right_share"], got["s2_boxes"]) == (0.5, 1.0, 1)


def test_a_teacher_that_answers_as_ve_does_fails_as_ve_would(score, mf):
    """The criteria end to end: VE's own answers fed in as the teacher's
    fail criteria 1-3 and 5 (79 mistakes, 94.9%, a teal yes, nothing to
    teach) and hold criterion 4 (no object worse than itself)."""
    man = mf.build()
    ans = {a["key"]: {"yes": a["yes"], "frac_x": None, "status": "answered"}
           for a in score.ve_answers()}
    for k in man["frames"]:
        ans.setdefault(k, {"yes": False, "frac_x": None, "status": "answered"})
    for i in [t["id"] for t in mf.tasks(man, "qwen") if t["mirror"]]:
        ans[i] = {"yes": False, "frac_x": None, "status": "answered"}
    whole = score.verdicts_qwen(ans, man)
    assert whole["coverage"]["whole"]
    assert [v for _, v in sorted(whole["criteria"].items())] == [False, False, False, True, False, False]


def test_a_partial_or_failed_run_passes_nothing(score, mf):
    man = mf.build()
    perfect = {t["id"]: {"yes": man["frames"][t["key"]]["visible"], "frac_x": None,
                         "status": "answered"} for t in mf.tasks(man, "qwen")}
    assert score.verdicts_qwen(perfect, man)["criteria"]["1 mistakes <= 59"]
    partial = dict(list(perfect.items())[:1500])
    got = score.verdicts_qwen(partial, man)
    assert got["coverage"]["missing"] == 868 and not any(got["criteria"].values())
    failed = {**perfect, "x": None}
    failed.pop("x")
    first = next(iter(failed))
    failed[first] = {"yes": False, "frac_x": None, "status": "error"}
    got = score.verdicts_qwen(failed, man)
    assert got["coverage"]["errors"] == 1 and not any(got["criteria"].values())


def test_supers_grounding_is_expected_only_where_the_target_is_in_view(score, mf):
    man = mf.build()
    ans = {}
    for t in mf.tasks(man, "super"):
        vis = man["frames"][t["key"]]["visible"]
        ans[t["id"]] = {"status": "answered", "yes": vis, "frac_x": 0.5 if vis else None,
                        "parsed_identity": True, "parsed_grounding": vis}
    got = score.verdicts_super(ans, man)
    assert got["parsed"]["grounding_in_view"] == 1.0          # no box out of view is right
    assert got["criteria"]["7 parseable on >= 95% in each mode"]


def test_the_pilot_shows_a_box_an_empty_answer_and_a_mirror(mf):
    man = mf.build()
    for model in ("qwen", "super"):
        pilot = [t for t in mf.tasks(man, model) if t["pilot"]]
        assert len(pilot) == 5 and sum(t["mirror"] for t in pilot) == 1
        seen = [man["frames"][t["key"]]["visible"] for t in pilot]
        assert seen.count(True) == 3 and seen.count(False) == 2


def test_a_jobs_links_travel_as_secrets_never_as_environment():
    launch = _load("launch")
    links = {"JOB_URL": "https://j.example/sig", "qwen": {
        "MANIFEST_URL": "https://m.example/sig", "RESULT_PUT_URL": "https://r.example/sig",
        "PILOT_PUT_URL": "https://p.example/sig"}}
    pilot = launch.job_spec("qwen", links, "a100-large", 40, 5)
    assert pilot["secrets"] == {"JOB_URL": "https://j.example/sig",
                                "MANIFEST_URL": "https://m.example/sig",
                                "RESULT_PUT_URL": "https://p.example/sig"}
    assert pilot["environment"] == {"LIMIT": "5"} and pilot["timeoutSeconds"] == 2400
    full = launch.job_spec("qwen", links, "a100-large", 60, 0)
    assert full["secrets"]["RESULT_PUT_URL"] == "https://r.example/sig" and full["environment"] == {}
    arm = launch.job_spec("super800", {**links, "super800": links["qwen"]}, "a100-large", 60, 0)
    assert arm["environment"] == {"SUPER_PIXELS": "786432,819200"}     # amendment 4, nothing else
    assert arm["command"] == launch.job_spec("super", {**links, "super": links["qwen"]},
                                             "a100-large", 60, 0)["command"]
    shown = json.dumps({k: full[k] for k in full if k != "secrets"})
    assert ".example/sig" not in shown                  # no link outside the secrets
