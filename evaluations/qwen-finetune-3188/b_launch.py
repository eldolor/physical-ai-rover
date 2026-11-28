# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/qwen-finetune-3188/b_launch.py

3.188.B: start, read, stop and bill the Hugging Face Jobs that run
`b_job.py` (3.214's launcher pattern). Every start is a paid call: it
prints the flavour, its price and the timeout's worst case, and refuses a
start whose worst case would carry the section past its $10 stop.

    python b_manifest.py --stage 24 --links /outside/links.json
    python b_launch.py start stock --links .../links.json --timeout-min 40
    python b_launch.py start pilot --links .../links.json --timeout-min 50   # fold 1
    python b_launch.py start rest  --links .../links.json --timeout-min 180  # folds 2-6, all
    python b_launch.py status|logs|cancel <job id>
    python b_launch.py bill <job id>        # once it has ended: its running time into the ledger
    python b_launch.py spent

The ledger (`b-spend.json`, committed) holds every job started: until a
job is billed its whole worst case counts as spent. Billing is by the
jobs API's own `runningSecs`, rounded up to whole minutes, at the
flavour's per-minute price read when the job was started.

The token is never read here: in a Claude Code cloud session the proxy
puts it on requests to huggingface.co. The presigned links travel as job
SECRETS (never the environment, which the job page shows) and are never
printed.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parents[1]
_spec = importlib.util.spec_from_file_location(
    "launch_3214_b", ROOT / "evaluations/teacher-probe-3214/launch.py")
L = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(L)

LEDGER = HERE / "b-spend.json"
STOP_USD = 10.0          # criterion 6: the whole of B stops at $10 (the user, 2026-11-17)
IMAGE = L.IMAGE
FETCH = ("python -c \"import os,urllib.request;"
         "open('/tmp/b_job.py','wb').write(urllib.request.urlopen(os.environ['JOB_URL']).read())\"")
RUNS = {"stock": ("stock", ""), "pilot": ("train", "1"), "rest": ("train", "2,3,4,5,6,all")}
ENDED = {"COMPLETED", "ERROR", "CANCELED", "CANCELLED", "DELETED", "TIMEOUT"}


def ledger() -> list:
    return json.loads(LEDGER.read_text())["jobs"] if LEDGER.exists() else []


def write_ledger(jobs: list) -> None:
    LEDGER.write_text(json.dumps({"stop_usd": STOP_USD, "jobs": jobs,
                                  "spent_usd": round(spent(jobs), 2)}, indent=1) + "\n")


def spent(jobs: list) -> float:
    """Billed jobs at their cost; a job not yet billed at its worst case."""
    return sum(j["usd"] if j.get("usd") is not None else j["worst_usd"] for j in jobs)


def bill_of(running_secs: float, per_min: float) -> float:
    return round(math.ceil(running_secs / 60) * per_min, 4)


def job_spec(run: str, links: dict, flavor: str, timeout_min: int, folds: str = "",
             result: str = "") -> dict:
    mode, default = RUNS[run]
    secrets = {"JOB_URL": links["JOB_URL"], "MANIFEST_URL": links["MANIFEST_URL"],
               "RESULT_PUT_URL": links["runs"][result or run]}
    env = {"FOLDS": folds or default} if mode == "train" else {}
    return {"command": ["bash", "-c", f"set -e; {FETCH}; python /tmp/b_job.py {mode}"],
            "arguments": [], "environment": env, "secrets": secrets, "flavor": flavor,
            "timeoutSeconds": int(timeout_min * 60), "dockerImage": IMAGE}


def check_result_link(jobs: list, stamp: str, result: str) -> None:
    """Each staged result link takes one job: the job's preflight empties
    it, so a second job on it would erase rows already paid for."""
    if any(j.get("stamp") == stamp and j.get("result") == result for j in jobs):
        sys.exit(f"refused: staging {stamp}'s result link {result!r} is already used; "
                 "pass --result rerun-N (or stage again)")


def check_budget(jobs: list, worst: float) -> None:
    if spent(jobs) + worst > STOP_USD:
        sys.exit(f"refused: ${spent(jobs):.2f} spent or held + ${worst:.2f} worst case "
                 f"is over the ${STOP_USD:.0f} stop")


def start(args) -> None:  # pragma: no cover -- a paid call
    links = json.loads(Path(args.links).read_text())
    per_min = L.price_per_min(args.flavor)
    worst = round(per_min * args.timeout_min, 4)
    jobs = ledger()
    print(f"{args.run} on {args.flavor}: ${per_min * 60:.2f}/h, timeout {args.timeout_min} min "
          f"=> at most ${worst:.2f}; spent or held so far ${spent(jobs):.2f} of ${STOP_USD:.0f}")
    result = args.result or args.run
    check_result_link(jobs, links["stamp"], result)
    check_budget(jobs, worst)
    spec = job_spec(args.run, links, args.flavor, args.timeout_min, args.folds, result)
    # Held BEFORE the post: a job created whose reply is lost still counts.
    row = {"id": None, "run": args.run, "folds": spec["environment"].get("FOLDS"),
           "stamp": links["stamp"], "result": result, "flavor": args.flavor, "per_min": per_min,
           "timeout_min": args.timeout_min, "worst_usd": worst,
           "started": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
           "running_secs": None, "usd": None, "stage": "posting"}
    jobs.append(row)
    write_ledger(jobs)
    with L.call("POST", f"/jobs/{L.namespace()}", spec) as r:
        info = json.loads(r.read())
    row.update(id=info.get("id"), stage=(info.get("status") or {}).get("stage"))
    write_ledger(jobs)
    print("job", row["id"], row["stage"])


def bill(args) -> None:  # pragma: no cover
    info = L.get(f"/jobs/{L.namespace()}/{args.job}")
    stage = (info.get("status") or {}).get("stage")
    if stage not in ENDED:
        sys.exit(f"{args.job} is {stage}: bill it once it has ended")
    jobs = ledger()
    row = next((j for j in jobs if j["id"] == args.job), None)
    if row is None:
        sys.exit(f"{args.job} is not in the ledger")
    secs = (info.get("durations") or {}).get("runningSecs")
    if not isinstance(secs, (int, float)):
        sys.exit(f"{args.job}: the jobs API gave no runningSecs; the worst case stays held")
    row.update(running_secs=secs, usd=bill_of(secs, row["per_min"]), stage=stage,
               message=(info.get("status") or {}).get("message"))
    write_ledger(jobs)
    print(f"{args.job}: {stage}, {secs} s running => ${row['usd']:.2f}; total ${spent(jobs):.2f}")


def show_spent(args) -> None:
    jobs = ledger()
    for j in jobs:
        print(j["id"], j["run"], j["stage"], j["running_secs"], j["usd"], "worst", j["worst_usd"])
    print(f"spent or held: ${spent(jobs):.2f} of ${STOP_USD:.0f}")


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("start")
    s.add_argument("run", choices=sorted(RUNS))
    s.add_argument("--links", required=True)
    s.add_argument("--flavor", default="a100-large")
    s.add_argument("--timeout-min", type=int, required=True)
    s.add_argument("--folds", default="", help="train runs: override the run's folds (a re-run)")
    s.add_argument("--result", default="", help="the staged result link (default: the run's name)")
    s.set_defaults(fn=start)
    for name, fn in (("status", L.status), ("logs", L.logs), ("cancel", L.cancel), ("bill", bill)):
        p = sub.add_parser(name)
        p.add_argument("job")
        if name == "logs":
            p.add_argument("--seconds", type=float, default=20)
        p.set_defaults(fn=fn)
    sub.add_parser("spent").set_defaults(fn=show_spent)
    args = ap.parse_args(argv)
    args.fn(args)


if __name__ == "__main__":
    main()
