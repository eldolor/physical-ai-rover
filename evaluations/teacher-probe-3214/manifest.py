# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/teacher-probe-3214/manifest.py

3.214: the frames both candidate teachers are asked about, from committed
records only (no frame is read, nothing is drawn anew):

* I-2261: every row of 3.92's `qwen-VE.jsonl` -- its key, the target text
  VE was asked with, and the people's label.
* I-299: the rows whose walk is one of 3.48's four arrival walks
  (`overlap-299.json`).
* teal / taupe: `lookalikes.json`'s 34 and 7 keys (the taupe frames are
  not in I-2261; their target is the walk's, "red backpack").
* S1 / S2: 3.80.I's 60 in-view and 40 out-of-view steering frames, each
  asked a second time mirrored.

    .venv/bin/python evaluations/teacher-probe-3214/manifest.py           # prints the counts
    .venv/bin/python evaluations/teacher-probe-3214/manifest.py --stage H # + presigned links, H hours

`--stage` writes NOTHING to the repo: the staged manifest (with presigned
GET links to the frames) goes to the deploy bucket's private
`gpu-eval/teacher-probe-3214/`, and the links a job needs are printed as a
JSON object for `launch.py` to pass as job secrets. A presigned link holds
a signature: never commit or log one.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parents[1]
VE_2261 = ROOT / "evaluations/tier-calibration-392/qwen-VE.jsonl"
FT = ROOT / "evaluations/qwen-finetune-3188"
SETS_380 = ROOT / "evaluations/tiered-380/I"

RECORDINGS_BUCKET = "vision-picar-recordings-303351622021-us-east-2"
RECORDINGS_PREFIX = "recordings"
DEPLOY_BUCKET = "vision-picar-deploy-303351622021-us-east-2"
STAGE_PREFIX = "gpu-eval/teacher-probe-3214"
REGION = "us-east-2"
TAUPE_TARGET = "red backpack"


def _jsonl(path: Path) -> list:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def build() -> dict:
    """`{"frames": {key: {"target", "visible", "sets": [...]}},
    "steer": {key: {"set", "target", "truth"}}, "counts": {...}}`."""
    frames = {}
    for r in _jsonl(VE_2261):
        frames[r["key"]] = {"target": r["target"], "visible": bool(r["visible"]), "sets": ["I2261"]}
    arrival = set(json.loads((FT / "overlap-299.json").read_text())["arrival_walks"])
    for key, row in frames.items():
        if key.split("/")[0] in arrival:
            row["sets"].append("I299")
    look = json.loads((FT / "lookalikes.json").read_text())
    for key in look["teal_bin"]["keys"]:
        frames[key]["sets"].append("teal")
    for key in look["taupe_bag"]["keys"]:
        if key in frames:
            raise ValueError(f"taupe frame {key} is inside I-2261: lookalikes.json says none is")
        # A taupe bag where the target (a red backpack) is not: label false.
        frames[key] = {"target": TAUPE_TARGET, "visible": False, "sets": ["taupe"]}

    steer = {}
    s1 = json.loads((SETS_380 / "labels-s1.json").read_text())["rows"]
    s2 = json.loads((SETS_380 / "sets.json").read_text())["S2"]
    for name, rows in (("S1", s1), ("S2", s2)):
        for r in rows:
            key = f"{r['walk']}/{r['frame']}"
            if key not in frames:
                raise ValueError(f"{name} frame {key} is not in I-2261")
            steer[key] = {"set": name, "target": r["target"], "truth": r.get("truth")}
            frames[key]["sets"].append(name)

    def n(tag):
        return sum(tag in row["sets"] for row in frames.values())

    return {"frames": dict(sorted(frames.items())), "steer": dict(sorted(steer.items())),
            "counts": {tag: n(tag) for tag in ("I2261", "I299", "teal", "taupe", "S1", "S2")}}


def tasks(manifest: dict, model: str) -> list:
    """The questions one model is asked, in a fixed order: `{"id", "key",
    "target", "mirror"}`. The 72B: every frame once, and S1/S2 again
    mirrored. Super: I-299, S1 and S2 (and the lookalikes, all inside
    them or seven more), S1/S2 again mirrored."""
    out = []
    pilot = pilot_ids(manifest)
    for key, row in manifest["frames"].items():
        sets = set(row["sets"])
        wanted = bool(sets) if model == "qwen" else bool(sets & {"I299", "teal", "taupe", "S1", "S2"})
        if not wanted:
            continue
        # The steering sets were asked with their own target text (3.80.I);
        # it is the walk's target, the same string, checked in the test.
        out.append({"id": key, "key": key, "target": row["target"], "mirror": False,
                    "pilot": key in pilot})
        if sets & {"S1", "S2"}:
            out.append({"id": key + "#mirror", "key": key, "target": row["target"], "mirror": True,
                        "pilot": key + "#mirror" in pilot})
    return out


def pilot_ids(manifest: dict) -> set:
    """The five-task pilot: the first two S1 frames (in view), one of them
    again mirrored, and the first two S2 frames (out of view) -- so a pilot
    shows a box, an empty answer and a mirrored frame, for either model."""
    s1 = [k for k, v in manifest["steer"].items() if v["set"] == "S1"][:2]
    s2 = [k for k, v in manifest["steer"].items() if v["set"] == "S2"][:2]
    return {*s1, s1[0] + "#mirror", *s2}


def stage(hours: float) -> dict:  # pragma: no cover -- needs AWS
    """Presigned GET links for every frame into a staged manifest on S3;
    returns the links a job needs (staged manifests, result uploads)."""
    import botocore.session
    from botocore.config import Config

    s3 = botocore.session.get_session().create_client(
        "s3", region_name=REGION, config=Config(signature_version="s3v4"))
    if botocore.session.get_session().get_credentials().token:
        # A link signed with temporary credentials dies with them, whatever
        # ExpiresIn says: a job would load a model and then fetch nothing.
        sys.exit("refused: these AWS credentials are temporary; presign with a long-lived key")
    secs = int(hours * 3600)
    manifest = build()

    def get(bucket, key):
        return s3.generate_presigned_url("get_object", Params={"Bucket": bucket, "Key": key},
                                         ExpiresIn=secs)

    def put(key):
        return s3.generate_presigned_url("put_object", Params={"Bucket": DEPLOY_BUCKET, "Key": key},
                                         ExpiresIn=secs)

    # Each staging has its own keys: a re-run never replaces an earlier run's rows.
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    links = {"stamp": stamp}
    for model in ("qwen", "super", "super800"):          # super800: the same tasks, its own rows
        staged = [{**t, "url": get(RECORDINGS_BUCKET, f"{RECORDINGS_PREFIX}/{t['key']}")}
                  for t in tasks(manifest, model)]
        key = f"{STAGE_PREFIX}/{stamp}/staged-{model}.json"
        s3.put_object(Bucket=DEPLOY_BUCKET, Key=key, Body=json.dumps(staged).encode())
        links[model] = {"MANIFEST_URL": get(DEPLOY_BUCKET, key),
                        "RESULT_PUT_URL": put(f"{STAGE_PREFIX}/{stamp}/rows-{model}.jsonl"),
                        "PILOT_PUT_URL": put(f"{STAGE_PREFIX}/{stamp}/pilot-{model}.jsonl")}
    job = (HERE / "job.py").read_bytes()
    s3.put_object(Bucket=DEPLOY_BUCKET, Key=f"{STAGE_PREFIX}/job.py", Body=job)
    links["JOB_URL"] = get(DEPLOY_BUCKET, f"{STAGE_PREFIX}/job.py")
    return links


def _in_a_work_tree(path: Path) -> bool:
    """Inside ANY git work tree (this worktree, the main checkout, another
    worktree): somewhere one `git add -A` could pick the links up."""
    import subprocess
    parent = path.resolve().parent
    if not parent.is_dir():
        return True                      # nowhere to write: refuse as well
    got = subprocess.run(["git", "-C", str(parent), "rev-parse", "--is-inside-work-tree"],
                         capture_output=True, text=True)
    return got.returncode == 0 and got.stdout.strip() == "true"


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", type=float, metavar="HOURS")
    ap.add_argument("--links", type=Path, help="with --stage: write the links here (outside the repo)")
    args = ap.parse_args(argv)
    manifest = build()
    print(manifest["counts"], {m: len(tasks(manifest, m)) for m in ("qwen", "super")})
    if args.stage:
        if args.links is None or _in_a_work_tree(args.links):
            sys.exit("--links must name a file outside the repo: presigned links are never committed")
        fd = os.open(args.links, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as f:
            f.write(json.dumps(stage(args.stage)))
        print(f"staged for {args.stage} h; links in {args.links}")


if __name__ == "__main__":
    main()
