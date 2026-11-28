# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/qwen-finetune-3188/b_manifest.py

3.188.B (amendments 1-2): what each Hugging Face Job trains on and is
asked, from committed records only. No frame is read here.

* **Training, per fold:** every frame of I-2261 whose object is one of the
  fold's five (all six for the all-objects model), less 3.206's 200-frame
  test draw (`c_labels.test_keys()`) and less every in-view frame without
  a kept 3.206.C label. A frame people labelled not in view answers `[]`;
  an in-view frame answers its kept label (the 72B's box in the student's
  pixels). Each frame is trained twice: as it is and mirrored, the box
  mirrored with it in the student's pixels.
* **Asked, per fold:** the held-out object's frames of I-2261 (the 3.188
  criteria), the mirror of that object's test-draw frames (3.206's
  criterion 7), and for the red backpack fold the 7 taupe-bag frames. The
  stock model is asked the union (every fold's tasks), the all-objects
  model the test draw both ways and the 41 lookalike frames (reported).

    .venv/bin/python evaluations/qwen-finetune-3188/b_manifest.py              # counts
    .venv/bin/python evaluations/qwen-finetune-3188/b_manifest.py --stage 24 \\
        --links /outside/the/repo/links.json

`--stage` writes nothing to the repo: the staged manifest (presigned GET
links to the frames, PUT links for the adapters) goes to the deploy
bucket's private `gpu-eval/qwen-finetune-3188/`, and the links a job
needs are written, mode 600, to a file outside every work tree.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parents[1]


def _by_path(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


s3214 = _by_path("score_3214_b", ROOT / "evaluations/teacher-probe-3214/score.py")
cl = _by_path("c_labels_3206_b", ROOT / "evaluations/steer-distill-3206/c_labels.py")
mf = s3214.mf
model_size = s3214.model_size

STAGE_PREFIX = "gpu-eval/qwen-finetune-3188"
SPLIT = HERE / "split.json"
# Fold numbers follow split.json's object order; "all" is the all-objects model.
FOLDS = ("1", "2", "3", "4", "5", "6", "all")
TAUPE_FOLD_OBJECT = "red backpack"
# One result link per job: a re-run never writes over an earlier run's rows.
RESULT_LINKS = ("stock", "pilot", "rest", "rerun-1", "rerun-2", "rerun-3")


def objects() -> list:
    return json.loads(SPLIT.read_text())["objects"]


def fold_object(fold: str):
    """The object a fold holds out; None for the all-objects model."""
    return None if fold == "all" else objects()[int(fold) - 1]


def answer(box) -> str:
    """The reply the student is trained to give: VE's own format, whole pixels."""
    if box is None:
        return "[]"
    return json.dumps([{"bbox_2d": [int(round(v)) for v in box]}])


def clipped(box: list, size: tuple) -> list:
    """A box in the student's pixels, each edge clipped to the image it sees."""
    w, h = model_size(*size)
    return [min(w, max(0, box[0])), min(h, max(0, box[1])), min(w, max(0, box[2])), min(h, max(0, box[3]))]


def mirrored_box(box: list, size: tuple) -> list:
    """A box in the student's pixels, left and right swapped with the image."""
    w = model_size(*size)[0]
    return [w - box[2], box[1], w - box[0], box[3]]


def build() -> dict:
    man = mf.build()
    frames = man["frames"]
    sizes = s3214_sizes()
    test = cl.test_keys()
    labels = {r["key"]: r for r in json.loads(cl.LABELS.read_text())["rows"]}
    in_2261 = sorted(k for k, v in frames.items() if "I2261" in v["sets"])
    taupe = sorted(k for k, v in frames.items() if "taupe" in v["sets"])
    look = json.loads((mf.FT / "lookalikes.json").read_text())
    teal = sorted(look["teal_bin"]["keys"])

    pool, left_out = [], {"test draw": 0, "in view, no kept label": 0}
    for key in in_2261:
        row = frames[key]
        if key in test:
            left_out["test draw"] += 1
            continue
        if row["visible"]:
            lab = labels.get(key)
            if not lab or not lab["kept"]:
                left_out["in view, no kept label"] += 1
                continue
            box = clipped(lab["label"], sizes[key])
        else:
            box = None
        pool.append({"key": key, "target": row["target"], "object": s3214.object_of(key),
                     "box": box, "size": sizes[key]})

    folds = {}
    for fold in FOLDS:
        held = fold_object(fold)
        train = []
        for p in pool:
            if p["object"] == held:
                continue
            train.append({"key": p["key"], "target": p["target"], "mirror": False,
                          "answer": answer(p["box"])})
            train.append({"key": p["key"], "target": p["target"], "mirror": True,
                          "answer": answer(mirrored_box(p["box"], p["size"]) if p["box"] else None)})
        if held is None:
            asked = [t for k in sorted(test) for t in _both(k, frames)]
            asked += [_task(k, frames, False) for k in teal + taupe if k not in test]
        else:
            mine = [k for k in in_2261 if s3214.object_of(k) == held]
            asked = [_task(k, frames, False) for k in mine]
            asked += [_task(k, frames, True) for k in mine if k in test]
            if held == TAUPE_FOLD_OBJECT:
                asked += [_task(k, frames, False) for k in taupe]
        folds[fold] = {"held_out": held, "train": train, "asked": asked}
    stock = sorted({t["id"]: t for f in FOLDS[:-1] for t in folds[f]["asked"]}.values(),
                   key=lambda t: t["id"])
    return {"folds": folds, "stock": stock, "left_out": left_out,
            "pool": len(pool), "test": sorted(test)}


def _task(key: str, frames: dict, mirror: bool) -> dict:
    return {"id": key + ("#mirror" if mirror else ""), "key": key,
            "target": frames[key]["target"], "mirror": mirror}


def _both(key: str, frames: dict) -> list:
    return [_task(key, frames, False), _task(key, frames, True)]


def s3214_sizes() -> dict:
    """key -> (w, h) of the frame, from 3.92's committed perception rows."""
    return {r["key"]: (r["w"], r["h"])
            for r in mf._jsonl(ROOT / "evaluations/tier-calibration-392/perception.jsonl")}


def job_sha256() -> str:
    """What the launcher compares before a start: the staged job is the committed one."""
    return hashlib.sha256((HERE / "b_job.py").read_bytes()).hexdigest()


def digest(obj) -> str:
    """sha256 (12 hex) of a JSON value, keys sorted: pins a run's inputs."""
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()[:12]


def counts(b: dict) -> dict:
    out = {"pool": b["pool"], "left_out": b["left_out"], "stock_asked": len(b["stock"]),
           "test": len(b["test"])}
    for fold, f in b["folds"].items():
        out[fold] = {"held_out": f["held_out"], "train": len(f["train"]),
                     "train_boxes": sum(t["answer"] != "[]" for t in f["train"]),
                     "asked": len(f["asked"]), "digest": digest(f)}
    return out


def stage(hours: float, b: dict) -> dict:  # pragma: no cover -- needs AWS
    """The staged manifest on S3 (frames' GET links, adapters' PUT links,
    every fold's tasks) and the links a job needs; nothing in the repo."""
    import botocore.session
    from botocore.config import Config

    sess = botocore.session.get_session()
    if sess.get_credentials().token:
        # A link signed with temporary credentials dies with them.
        sys.exit("refused: these AWS credentials are temporary; presign with a long-lived key")
    s3 = sess.create_client("s3", region_name=mf.REGION, config=Config(signature_version="s3v4"))
    secs = int(hours * 3600)

    def get(bucket, key):
        return s3.generate_presigned_url("get_object", Params={"Bucket": bucket, "Key": key},
                                         ExpiresIn=secs)

    def put(key):
        return s3.generate_presigned_url("put_object", Params={"Bucket": mf.DEPLOY_BUCKET, "Key": key},
                                         ExpiresIn=secs)

    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    base = f"{STAGE_PREFIX}/{stamp}"
    keys = sorted({t["key"] for t in b["stock"]}
                  | {t["key"] for f in b["folds"].values() for t in f["train"] + f["asked"]})
    staged = {
        "stamp": stamp,
        "urls": {k: get(mf.RECORDINGS_BUCKET, f"{mf.RECORDINGS_PREFIX}/{k}") for k in keys},
        "stock": b["stock"],
        "folds": b["folds"],
        "adapter_put": {f: put(f"{base}/adapter-{f}.tar") for f in FOLDS},
    }
    s3.put_object(Bucket=mf.DEPLOY_BUCKET, Key=f"{base}/staged.json",
                  Body=json.dumps(staged).encode())
    s3.put_object(Bucket=mf.DEPLOY_BUCKET, Key=f"{STAGE_PREFIX}/{stamp}/b_job.py",
                  Body=(HERE / "b_job.py").read_bytes())
    links = {"stamp": stamp, "job_sha256": job_sha256(), "JOB_URL": get(mf.DEPLOY_BUCKET, f"{base}/b_job.py"),
             "MANIFEST_URL": get(mf.DEPLOY_BUCKET, f"{base}/staged.json"), "runs": {}}
    for run in RESULT_LINKS:
        links["runs"][run] = put(f"{base}/rows-{run}.jsonl")
    return links


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", type=float, metavar="HOURS")
    ap.add_argument("--links", type=Path, help="with --stage: write the links here (outside the repo)")
    args = ap.parse_args(argv)
    b = build()
    print(json.dumps(counts(b), indent=1))
    if args.stage:
        if args.links is None or mf._in_a_work_tree(args.links):
            sys.exit("--links must name a file outside the repo: presigned links are never committed")
        links = stage(args.stage, b)
        fd = os.open(args.links, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as f:
            f.write(json.dumps(links))
        print(f"staged {links['stamp']} for {args.stage} h; links in {args.links}")


if __name__ == "__main__":
    main()
