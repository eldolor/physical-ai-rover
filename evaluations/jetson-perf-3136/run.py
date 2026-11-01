"""
evaluations/jetson-perf-3136/run.py

3.136's entry point on the desktop.

    python evaluations/jetson-perf-3136/run.py plant            # criterion 3, desktop, exclusive lock
    python evaluations/jetson-perf-3136/run.py smoke [--sha REV] # 3.136.A's board smoke run (no sudo)
    python evaluations/jetson-perf-3136/run.py found             # the board's as-found state, read only

`board_run()` is the shape every board test takes (3.136.B-D, and 3.135.A
can import it): under both locks, snapshot the board, put the commit under
test at ~/bench-3136, optionally stop the twin's units (sudo), sample
telemetry, run one board command with PICAR_BOARD_TOKEN, fetch its output,
put the board back and check it against the snapshot (criterion 4). The
record lands in `<out>/record.json`.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import boardrun as br  # noqa: E402
import scorers  # noqa: E402
import validity  # noqa: E402

PY = sys.executable


def board_run(label: str, board_cmd: str, out: Path, *, sha: str = "HEAD",
              allow_sudo: bool = False, stop_twin: bool = False, run=br.ssh,
              push=br.push_checkout, fetch=br.fetch, sampler_cls=br.Sampler) -> dict:
    """One board test, inside `boardrun.locked` (refuses without the token).
    `board_cmd` runs in ~/bench-3136 with $OUT set to its own output
    directory there and PICAR_BOARD_TOKEN exported."""
    token = br.require_board_lock()
    out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    board_out = f"{br.BENCH_DIR}/out-{label}-{stamp}"
    rec: dict = {"label": label, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                 "allow_sudo": allow_sudo, "stop_twin": stop_twin, "board_out": board_out}
    before = br.snapshot(run)
    rec["as_found"] = before
    (out / "as_found.json").write_text(json.dumps(before, indent=1))
    sampler = None
    try:
        rec["sha"] = push(sha, run)
        if stop_twin:
            br.stop_twin(run, allow_sudo=allow_sudo)
        sampler = sampler_cls(f"~/{board_out}/telemetry", run)
        sampler.start()
        t0 = time.time()
        r = run(f"cd ~/{br.BENCH_DIR} && export OUT=~/{board_out} PICAR_BOARD_TOKEN={shlex.quote(token)} "
                f"&& mkdir -p $OUT && {{ {board_cmd} ; }} > $OUT/cmd.log 2>&1; echo $?",
                check=False, timeout=6 * 3600)
        rec["cmd_exit"] = int(r.strip().splitlines()[-1]) if r.strip() else None
        rec["cmd_s"] = round(time.time() - t0, 1)
    finally:
        if sampler is not None:
            sampler.stop()
            rec["desktop_load"] = sampler.desktop_load
        try:
            fetch(f"~/{board_out}", out / "board")
        except Exception as e:  # noqa: BLE001 -- the restore below must still run
            rec["fetch_error"] = str(e)
        rec["restore_actions"] = br.restore(before, run, allow_sudo=allow_sudo)
        after, diffs = br.settle_and_check(before, run, wait_s=120 if rec["restore_actions"] else 5)
        rec["after"] = after
        rec["as_found_differences"] = diffs
        rec["left_as_found"] = not diffs
        (out / "record.json").write_text(json.dumps(rec, indent=1))
    return rec


# ------------------------------------------------------------- the smoke run

SMOKE_WALKS = ("blue-shoes-20260907-152528", "red-backpack-20260907-144856")
SMOKE_CMD = (
    "set -a; source ~/.vision-picar-local-secrets; set +a; export APP_SHARED_SECRET=\"$LOCAL_SECRET\"; "
    "nvpmodel -q > $OUT/nvpmodel.txt 2>&1; "
    "curl -s -H \"x-app-secret: $LOCAL_SECRET\" localhost:8000/health > $OUT/health0.json; "
    "date +%s.%N > $OUT/bench_start; "
    ".venv/bin/python -m tools.jetson.bench_local_vlm frames --recordings recordings "
    + " ".join(f"--walk {w}" for w in SMOKE_WALKS)
    + " --out $OUT/vlm.json > $OUT/vlm.log 2>&1; echo $? > $OUT/bench_exit; "
    "date +%s.%N > $OUT/bench_end; "
    "curl -s -H \"x-app-secret: $LOCAL_SECRET\" localhost:8000/health > $OUT/health1.json")


def smoke_inner(a) -> int:
    out = Path(a.out)
    rec = board_run("smoke", SMOKE_CMD, out, sha=a.sha, allow_sudo=False, stop_twin=False)
    b = out / "board"
    tele = br.read_sysfs(b / "telemetry/sysfs.jsonl") if (b / "telemetry/sysfs.jsonl").exists() else []
    b0 = float((b / "bench_start").read_text()) if (b / "bench_start").exists() else None
    b1 = float((b / "bench_end").read_text()) if (b / "bench_end").exists() else None
    tc = validity.telemetry_counts(tele, b0, b1)
    h0 = json.loads((b / "health0.json").read_text() or "{}") if (b / "health0.json").exists() else {}
    h1 = json.loads((b / "health1.json").read_text() or "{}") if (b / "health1.json").exists() else {}
    wheel = scorers.wheel_delta(h0, h1) if "wheel_loop" in h0 and "wheel_loop" in h1 else None
    vlm = scorers.vlm_latency(json.loads((b / "vlm.json").read_text())) if (b / "vlm.json").exists() else None
    run_doc = {"kind": "smoke", "window": None, "bench": [b0, b1], "wheel": wheel,
               "oom_kills": tc["oom_kills"], "nav": None,
               "power_mode": (b / "nvpmodel.txt").read_text().splitlines()[0]
               if (b / "nvpmodel.txt").exists() else None,
               "desktop_load": rec.get("desktop_load"), "temps": tc["temps"], "clocks": tc["clocks"]}
    j = validity.judge(run_doc)
    summary = {"sha": rec.get("sha"), "cmd_exit": rec.get("cmd_exit"), "cmd_s": rec.get("cmd_s"),
               "bench_exit": (b / "bench_exit").read_text().strip() if (b / "bench_exit").exists() else None,
               "vlm_latency_ms": vlm, "telemetry": tc, "wheel": wheel, "validity": j,
               "scored_value": validity.score(run_doc, vlm and vlm.get("p90")),
               "tegrastats_lines": len((b / "telemetry/tegrastats.log").read_text().splitlines())
               if (b / "telemetry/tegrastats.log").exists() else 0,
               "desktop_load_samples": len(rec.get("desktop_load") or []),
               "restore_actions": rec["restore_actions"],
               "as_found_differences": rec["as_found_differences"],
               "left_as_found": rec["left_as_found"]}
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))
    ok = (rec.get("cmd_exit") == 0 and summary["bench_exit"] == "0" and rec["left_as_found"]
          and tc["temps"] and tc["clocks"] and summary["tegrastats_lines"] and not j["valid"])
    print("smoke:", "OK" if ok else "NOT OK")
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("smoke")
    s.add_argument("--sha", default="HEAD")
    s.add_argument("--out", default=str(HERE / "board" / "smoke"))
    s.add_argument("--wait", action="store_true", help="wait for the desktop lock")
    i = sub.add_parser("_smoke")
    i.add_argument("--sha", default="HEAD")
    i.add_argument("--out", required=True)
    p = sub.add_parser("plant")
    p.add_argument("--wait", action="store_true", help="wait for the desktop lock")
    sub.add_parser("found")
    a = ap.parse_args(argv)
    if a.cmd == "smoke":
        sha = subprocess.run(["git", "rev-parse", a.sha], cwd=ROOT, capture_output=True, text=True,
                             check=True).stdout.strip()
        # Not timed: the shared desktop lock.
        return br.locked([PY, str(Path(__file__).resolve()), "_smoke", "--sha", sha, "--out", a.out],
                         "3.136.A board smoke", desktop_mode="shared", desktop_wait=a.wait)
    if a.cmd == "_smoke":
        return smoke_inner(a)
    if a.cmd == "plant":
        return subprocess.call([PY, str(ROOT / "tools/desktop_lock.py"), "exclusive",
                                *([] if a.wait else ["--no-wait"]), "--why", "3.136.A planted delay",
                                "--", PY, str(HERE / "plant.py")], cwd=ROOT)
    if a.cmd == "found":
        print(json.dumps(br.snapshot(), indent=1))
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
