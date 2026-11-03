# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
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
              push=br.push_checkout, fetch=br.fetch, sampler_cls=br.Sampler,
              bench_dir: str = br.BENCH_DIR, heartbeat_cls=br.Heartbeat,
              sleep=time.sleep) -> dict:
    """One board test, inside `boardrun.locked` (refuses without the token).
    `board_cmd` runs in ~/<bench_dir> (3.136's `bench-3136` by default;
    3.135 passes `bench-3135`) with $OUT set to its own output directory
    there and PICAR_BOARD_TOKEN exported.

    3.136.F: a lease runs beside it on the board, so a desktop that goes
    away mid-run (killed, or its network lost) still leaves the board as
    found (`boardrun.lease`). If no exit code comes back from the bench's
    ssh, the bench may still run there: this side then does nothing on the
    board and returns (its own board hold ends); the lease ends the bench,
    takes the board and restores, and its `lease.json` is the record.
    """ 
    token = br.require_board_lock()
    out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    board_out = f"{bench_dir}/out-{label}-{stamp}"
    rec: dict = {"label": label, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                 "allow_sudo": allow_sudo, "stop_twin": stop_twin, "board_out": board_out}
    before = br.snapshot(run)
    rec["as_found"] = before
    (out / "as_found.json").write_text(json.dumps(before, indent=1))
    sampler = heartbeat = None
    lease_started, lease_pid, lease_start, bench_started = False, "", "", False
    try:
        rec["sha"] = push(sha, run, bench_dir=bench_dir)
        # The harness's own boardrun.py, never the commit under test's (which may
        # predate the lease), and the snapshot to restore from.
        run(f"mkdir -p ~/{board_out} && cat > ~/{board_out}/lease_boardrun.py",
            input=Path(br.__file__).read_text())
        got = run(f"cat > ~/{board_out}/as_found.json && touch ~/{board_out}/heartbeat && "
                  f"{{ systemctl is-active graphical.target >/dev/null && echo graphical || echo other; }}"
                  f" > ~/{board_out}/target_at_start && {{ setsid nohup python3 ~/{board_out}/lease_boardrun.py"
                  f" lease ~/{board_out} --silence {br.LEASE_SILENCE_S:g}{' --sudo' if allow_sudo else ''}"
                  f" > ~/{board_out}/lease.log 2>&1 < /dev/null & }} && L=$! && echo $L && "
                  + (f"{{ echo {br.LEASE_OOM_ADJ} | sudo -n tee /proc/$L/oom_score_adj >/dev/null; }}; "
                     if allow_sudo else "")
                  + f"sleep 2; kill -0 $L 2>/dev/null && echo alive $({br.START_TIME_SH % '$L'})",
                  input=json.dumps(before), check=False) or ""
        lines = got.split()
        if len(lines) < 3 or not lines[0].isdigit() or lines[1] != "alive":
            # A lease that did start but answered wrong must not act later: let it go.
            run(f"touch ~/{board_out}/lease.stop", check=False)
            raise br.BoardError(f"the lease did not start ({got.strip()[:80]!r}); see ~/{board_out}/lease.log")
        lease_pid, lease_start, lease_started = lines[0], lines[2], True
        heartbeat = heartbeat_cls(f"~/{board_out}")
        heartbeat.start()
        if stop_twin:
            br.stop_twin(run, allow_sudo=allow_sudo)
        sampler = sampler_cls(f"~/{board_out}/telemetry", run)
        sampler.start()
        t0 = time.time()
        bench_started = True
        # 3.93's check in every bench: its keeper holds the board while the bench lives,
        # so a desktop gone mid-run never hands the board over beside its bench.
        r = run(f"cd ~/{bench_dir} && export OUT=~/{board_out} PICAR_BOARD_TOKEN={shlex.quote(token)} "
                f"&& mkdir -p $OUT && echo \"$$ $({br.START_TIME_SH % '$$'})\" > $OUT/bench.pid "
                f"&& {{ source tools/jetson/board_lock_check.sh || exit $?; {board_cmd} ; }} "
                f"> $OUT/cmd.log 2>&1; echo $?",
                check=False, timeout=6 * 3600)
        last = r.strip().splitlines()[-1] if r.strip() else ""
        rec["cmd_exit"] = int(last) if last.lstrip("-").isdigit() else None
        rec["cmd_s"] = round(time.time() - t0, 1)
    finally:
        # Each step on its own: a failure in one (an ssh drop) never skips
        # the restore or the record, and never masks the run's own error.
        # No exit code back (an ssh that raised, or returned empty: a drop read as 255):
        # the bench may still run there. This side then does NOTHING on the board -- no
        # restore, no release, no sampler kill -- and the run ends here, which ends its board
        # hold. The bench's keeper keeps the board while it lives; the lease (no more
        # beats) ends it, takes the board and restores, and its `lease.json` says so.
        if lease_started and bench_started and rec.get("cmd_exit") is None:
            if heartbeat is not None:
                heartbeat.stop()
                rec["heartbeat_losses"] = getattr(heartbeat, "losses", None)
            rec["lease"] = "left to the lease: no exit code from the bench's ssh"
            rec["restore_actions"] = [f"left to the lease: see ~/{board_out}/lease.json"]
            rec["left_as_found"] = None
            if sampler is not None:
                rec["desktop_load"] = sampler.desktop_load
        else:
            if sampler is not None:
                try:
                    sampler.stop()
                except Exception as e:  # noqa: BLE001
                    rec["sampler_error"] = str(e)
                rec["desktop_load"] = sampler.desktop_load
            try:
                fetch(f"~/{board_out}", out / "board")
            except Exception as e:  # noqa: BLE001
                rec["fetch_error"] = str(e)
            rec["left_as_found"] = False
            try:
                # Put back with the lease still beating behind: a desktop lost now is still covered.
                rec["restore_actions"], after, diffs = br.restore_and_check(
                    before, run, allow_sudo=allow_sudo, board_out=f"~/{board_out}")
                rec["after"] = after
                rec["as_found_differences"] = diffs
                rec["left_as_found"] = not diffs
            except Exception as e:  # noqa: BLE001
                rec["restore_error"] = f"{type(e).__name__}: {e}"
            # Only now, the board put back, is the lease let go (STOPPED: the file landed).
            if lease_started:
                if heartbeat is not None:
                    heartbeat.stop()
                    rec["heartbeat_losses"] = getattr(heartbeat, "losses", None)
                st = ""
                for _ in range(3):
                    try:
                        st = (run(f"touch ~/{board_out}/lease.stop && echo STOPPED; sleep 0.5; "
                                  f"cat {br.state_path(board_out)} 2>/dev/null; "
                                  f"[ \"$({br.START_TIME_SH % lease_pid})\" = {lease_start} ] && echo LEASE-ALIVE",
                                  check=False) or "")
                    except Exception as e:  # noqa: BLE001
                        st = f"error {e}"
                    if "STOPPED" in st:
                        break
                words = st.split()
                if "STOPPED" not in words:
                    rec["lease"] = f"not released ({st.strip()[:80]!r}): it may act on its own"
                elif "LEASE-ALIVE" not in words and not {"released", "done"} & set(words):
                    rec["lease"] = "the lease had died during the run: this run was unprotected"
                elif {"acting", "done"} & set(words):
                    rec["lease"] = "the lease acted during the run: see lease.json"
                else:
                    rec["lease"] = "released"
        (out / "record.json").write_text(json.dumps(rec, indent=1))
    return rec


# ------------------------------------------------------------- the smoke run

SMOKE_WALKS = ("blue-shoes-20260907-152528", "red-backpack-20260907-144856")
SMOKE_CMD = (
    "set -a; source ~/.vision-picar-local-secrets; set +a; export APP_SHARED_SECRET=\"$LOCAL_SECRET\"; "
    "nvpmodel -q 2>&1 | grep -m1 'NV Power Mode' > $OUT/nvpmodel.txt; "
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
               "restore_actions": rec.get("restore_actions"), "restore_error": rec.get("restore_error"),
               "as_found_differences": rec.get("as_found_differences"),
               "left_as_found": rec["left_as_found"]}
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))
    ok = (rec.get("cmd_exit") == 0 and summary["bench_exit"] == "0" and rec["left_as_found"]
          and tc["temps"] and tc["clocks"] and summary["tegrastats_lines"] and not j["valid"])
    print("smoke:", "OK" if ok else "NOT OK")
    return 0 if ok else 1


# ------------------------------------------------- 3.136.B: the steady window

P8_REFERENCE = ROOT / "evaluations/identity-378/V/TVE-299.json"
BARS = {"P1": 250.0, "P2_ms": 250.0, "P2_mem_mb": 100.0, "P8": 1500.0, "P11_ratio": 0.90,
        "P12_pause": 0.10}
CONTROLLER_HZ = 20.0       # nav2.yaml controller_frequency


def _f(p: Path):
    return float(p.read_text().strip()) if p.exists() and p.read_text().strip() else None


def score_steady(out: Path, rec: dict) -> dict:
    """3.136.B's rows from one steady.sh window (amendment 2)."""
    b = out / "board"
    st = {k: _f(b / k) for k in ("window_start", "window_end", "p1_start", "p1_end",
                                 "p8_start", "p8_end", "p2_start", "p2_end")}
    ws, we = st["window_start"], st["window_end"]
    tele = br.read_sysfs(b / "telemetry/sysfs.jsonl") if (b / "telemetry/sysfs.jsonl").exists() else []
    day = time.strftime("%Y-%m-%d", time.localtime(ws)) if ws else None
    h0 = json.loads((b / "health_w0.json").read_text() or "{}") if (b / "health_w0.json").exists() else {}
    h1 = json.loads((b / "health_w1.json").read_text() or "{}") if (b / "health_w1.json").exists() else {}
    wheel = scorers.wheel_delta(h0, h1) if "wheel_loop" in h0 and "wheel_loop" in h1 else None
    pace = scorers.jsonl(b / "pace.jsonl")
    shim = scorers.jsonl(b / "shim.jsonl")
    nav = scorers.nav_goals(pace, ws, we)
    tc = validity.telemetry_counts(tele, ws, we)
    o0, o1 = _f(b / "oom_start"), _f(b / "oom_end")
    # Clipped: a ring buffer that wrapped never cancels a kill the counter saw.
    dmesg_ooms = max(0, int(o1 - o0)) if o0 is not None and o1 is not None else None
    pm = (b / "nvpmodel.txt").read_text().splitlines()[0] if (b / "nvpmodel.txt").exists() else None

    def judge(bench):
        doc = {"kind": "full", "window": [ws, we], "bench": bench, "wheel": wheel,
               # Either count of a kill makes the run invalid.
               "oom_kills": None if tc["oom_kills"] is None else max(tc["oom_kills"], dmesg_ooms or 0),
               # Amendment 2: explore's goals; a goal its sender cancelled is
               # neither (n counts succeeded + aborted).
               "nav": {"ok": nav["succeeded"], "n": nav["succeeded"] + nav["aborted"]},
               "power_mode": pm, "desktop_load": rec.get("desktop_load"),
               "temps": tc["temps"], "clocks": tc["clocks"]}
        return doc, validity.judge(doc)

    rows = {}
    # P1, P2: perception p90 in their own stretches; P2's added memory.
    for row, log, a, z in (("P1", "perception-p1.log", "p1_start", "p1_end"),
                           ("P2", "perception-p2.log", "p2_start", "p2_end")):
        ms = (scorers.perception_ms((b / log).read_text(), st[a], st[z], day)
              if (b / log).exists() and st[a] and st[z] else [])
        doc, j = judge([st[a], st[z]])
        mem = (validity.telemetry_counts(tele, st[a], st[z])["mem_available_min_mb"]
               if st[a] and st[z] else None)
        rows[row] = {"n": len(ms), "p50_ms": scorers.percentile(ms, 50),
                     "p90_ms": scorers.percentile(ms, 90), "mem_available_min_mb": mem,
                     "valid": j["valid"], "reasons": j["reasons"]}
    p1, p2 = rows["P1"], rows["P2"]
    haz = [r.get("hazard_boxes") for r in scorers.jsonl(b / "perc-p2.jsonl")
           if st["p2_start"] and st["p2_end"] and st["p2_start"] <= r["t0"] <= st["p2_end"]]
    p2["hazard_boxes_per_frame"] = round(sum(h or 0 for h in haz) / len(haz), 3) if haz else None
    p2["added_mem_mb"] = (p1["mem_available_min_mb"] - p2["mem_available_min_mb"]
                          if p1["mem_available_min_mb"] is not None
                          and p2["mem_available_min_mb"] is not None else None)
    p1["met"] = p1["n"] >= 200 and p1["p90_ms"] is not None and p1["p90_ms"] <= BARS["P1"]
    p2["met"] = (p2["n"] >= 200 and p2["p90_ms"] is not None and p2["p90_ms"] <= BARS["P2_ms"]
                 and p2["added_mem_mb"] is not None and p2["added_mem_mb"] <= BARS["P2_mem_mb"])
    # P8: VE over HTTP, and its answers against 3.78's.
    vdoc = json.loads((b / "vlm.json").read_text()) if (b / "vlm.json").exists() else None
    doc, j = judge([st["p8_start"], st["p8_end"]])
    if vdoc:
        sys.path.insert(0, str(ROOT))
        from tools.jetson.bench_local_vlm import compare
        cmp = compare(vdoc["records"], json.loads(P8_REFERENCE.read_text())["records"])
        lat = scorers.vlm_latency(vdoc)
        rows["P8"] = {**lat, "answers_identical": cmp["met"], "flips": cmp["flips"],
                      "off": len(cmp["off"]), "missing": len(cmp["missing"]),
                      "valid": j["valid"], "reasons": j["reasons"],
                      "met": lat["n"] >= 200 and lat["p90"] <= BARS["P8"] and cmp["met"]}
    else:
        rows["P8"] = {"valid": False, "reasons": j["reasons"] + ["no vlm.json"], "met": False}
    # P4, P11, P12 over the whole window.
    doc, j = judge([ws, we])
    perc = [[r["t0"], r["t1"]] for name in ("perc-p1.jsonl", "perc-p2.jsonl")
            for r in scorers.jsonl(b / name) if ws and we and ws <= r["t0"] <= we]
    ov = scorers.overlaps({"map_fetch": scorers.shim_spans(shim, "map", ws, we),
                           "ve_question": scorers.shim_spans(shim, "vlm", ws, we),
                           "perception": perc,
                           "room_cut": scorers.shim_spans(shim, "segment", ws, we, "room_labels.py")},
                          scorers.map_updates(pace, ws, we))
    rows["P4"] = {"late_ticks": wheel and wheel["late"], "moving_ticks": wheel and wheel["moving"],
                  "max_dt_s_since_start": wheel and wheel["max_dt_s"],
                  "window_s": round(we - ws, 1) if ws and we else None, "overlap": ov,
                  "valid": j["valid"], "reasons": j["reasons"],
                  "met": bool(wheel) and bool(ws and we) and wheel["late"] == 0 and (we - ws) >= 1800}
    cr, sl = scorers.controller_rate(pace, ws, we), scorers.scan_lag(pace, ws, we)
    rows["P11"] = {"controller": cr, "ratio": round(cr["rate_hz"] / CONTROLLER_HZ, 3) if cr["rate_hz"] else None,
                   "scan_lag": sl, "valid": j["valid"], "reasons": j["reasons"],
                   "met": cr["rate_hz"] is not None and cr["intervals"] >= 200
                   and cr["rate_hz"] >= BARS["P11_ratio"] * CONTROLLER_HZ}
    rw = scorers.room_work(shim, ws, we)
    mp = [r for r in scorers.jsonl(b / "mappoll.jsonl") if ws and we and ws <= r["t"] <= we]
    fetch = scorers.durations_ms(scorers.shim_spans(shim, "map", ws, we))
    rows["P12"] = {**rw, "map_reply": scorers.map_summary(mp),
                   "explore_map_fetch": {"n": len(fetch), "p50_ms": scorers.percentile(fetch, 50),
                                         "p90_ms": scorers.percentile(fetch, 90)},
                   "valid": j["valid"], "reasons": j["reasons"],
                   "met": rw["pause_share"] is not None and rw["segment"]["n"] > 0
                   and rw["pause_share"] <= BARS["P12_pause"]}
    missions = [m for m in scorers.jsonl(b / "missions.jsonl") if "outcome" in m]
    refused = sum("outcome" not in m for m in scorers.jsonl(b / "missions.jsonl"))
    return {"sha": rec.get("sha"), "stamps": st, "window_s": rows["P4"]["window_s"],
            "uptime_at_start_s": (rec.get("as_found") or {}).get("uptime_s"),
            "wheel": wheel, "nav2_goals": nav, "telemetry": tc, "dmesg_oom_kills": dmesg_ooms,
            "power_mode": pm, "validity": j,
            "desktop_load_max": max((x[1] for x in rec.get("desktop_load") or []), default=None),
            "missions": {"n": len(missions), "refused_starts": refused,
                         "outcomes": {o: sum(m.get("outcome") == o for m in missions)
                                      for o in {m.get("outcome") for m in missions}},
                         "label_calls": sum((m.get("room_labels") or {}).get("label_calls") or 0
                                            for m in missions)},
            "cloud_calls": len(scorers.jsonl(b / "cloud_calls.jsonl")),
            "rows": rows, "cmd_exit": rec.get("cmd_exit"),
            "restore_actions": rec.get("restore_actions"),
            "as_found_differences": rec.get("as_found_differences"),
            "left_as_found": rec.get("left_as_found")}


def steady_inner(a) -> int:
    out = Path(a.out)
    env = " ".join(f"{k}={shlex.quote(v)}" for k, v in (x.split("=", 1) for x in a.env or []))
    rec = board_run(a.label, f"{env} bash evaluations/jetson-perf-3136/steady.sh $OUT", out,
                    sha=a.sha, allow_sudo=True, stop_twin=True)
    summary = score_steady(out, rec)
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))
    return 0 if rec.get("cmd_exit") == 0 and rec.get("left_as_found") else 1


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
    for name in ("steady", "_steady"):
        t = sub.add_parser(name)
        t.add_argument("--sha", default="HEAD")
        t.add_argument("--label", default="steady")
        t.add_argument("--out", default=None)
        t.add_argument("--env", action="append", help="KEY=VALUE for steady.sh (a smoke's short window)")
        t.add_argument("--wait", action="store_true", help="wait for the desktop lock")
    sc = sub.add_parser("score")
    sc.add_argument("out")
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
    if a.cmd == "steady":
        sha = subprocess.run(["git", "rev-parse", a.sha], cwd=ROOT, capture_output=True, text=True,
                             check=True).stdout.strip()
        out = a.out or str(HERE / "board" / f"{a.label}-{time.strftime('%Y%m%d-%H%M')}")
        # Timed: the exclusive desktop lock, then the board lock.
        return br.locked([PY, str(Path(__file__).resolve()), "_steady", "--sha", sha,
                          "--label", a.label, "--out", out,
                          *[x for e in (a.env or []) for x in ("--env", e)]],
                         f"3.136.B {a.label}", desktop_mode="exclusive", desktop_wait=a.wait)
    if a.cmd == "_steady":
        return steady_inner(a)
    if a.cmd == "score":
        out = Path(a.out)
        summary = score_steady(out, json.loads((out / "record.json").read_text()))
        (out / "summary.json").write_text(json.dumps(summary, indent=1))
        print(json.dumps(summary, indent=1))
        return 0
    if a.cmd == "found":
        print(json.dumps(br.snapshot(), indent=1))
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
