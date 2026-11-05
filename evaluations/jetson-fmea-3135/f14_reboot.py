# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/f14_reboot.py

3.135.B F14 (amendment 18), from the DESKTOP: the board rebooted mid-mission,
on the twin's production units as found (as F8-unit; nothing changed), 3 runs.
A board-side script would die with the reboot, so this one drives from here
and sends its board-side steps over ssh (this file, on the board's python3's
stdin; stdlib only on that side).

Each run:
1. Under the board lock (`board_lock.py run`): the boot id, the units and
   the stack's files read; a frontier mission started through the board's
   brain and seen driving; then `sudo -n systemctl reboot`. The lock dies with
   the board; the orchestrating session keeps the board clear meanwhile.
2. No lock: the board polled over ssh, each service timed from the reboot
   command -- the boot id changed (else the run is invalid), the robot's and
   the brain's `/health` 200 (the bar: <= 90 s), `picar-ngrok` and
   `picar-local-vlm` active and VE's `/health` ready (reported).
3. Under the board lock again: every JSON file loads and every JSONL line
   parses in the stack's inventory, intervention and explore-choice
   directories (0 unparseable is the bar); the next mission starts (200, and
   running), then is stopped; the units as found.

    python evaluations/jetson-fmea-3135/f14_reboot.py <out> [--runs 3]
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

BOARD_DIR = "~/vision-picar-service"          # picar-stack's worktree (3.71)
DATA_DIRS = ("recordings/inventory", "recordings/interventions", "recordings/explore-choices")
UNITS = ("picar-stack", "picar-ngrok", "picar-local-vlm")
ROBOT, BRAIN, VE = "http://127.0.0.1:8000", "http://127.0.0.1:8001", "http://127.0.0.1:8005"
BACK_BAR_S = 90.0
BACK_WAIT_S = 300.0
DRIVE_WAIT_S = 120.0
REBOOT_AFTER_S = 20.0          # a mission this long under way before the reboot
SETTLE_S = 60.0                # between runs: VE loaded, the board quiet


# ------------------------------------------------------------------ both sides

def check_tree(root: Path, dirs=DATA_DIRS) -> dict:
    """Every JSON file loaded and every JSONL line parsed under `dirs`
    (pure, the planted check feeds it torn copies). Other files are counted,
    not read (images)."""
    res = {"files": 0, "json": 0, "jsonl_lines": 0, "other": 0, "bad": [], "index": {}}
    for d in dirs:
        base = Path(root, d)
        if not base.is_dir():
            continue
        for p in sorted(base.rglob("*")):
            if not p.is_file():
                continue
            res["files"] += 1
            rel = str(p.relative_to(root))
            res["index"][rel] = p.stat().st_size
            if p.suffix == ".json":
                res["json"] += 1
                try:
                    json.loads(p.read_text())
                except (ValueError, UnicodeDecodeError) as e:
                    res["bad"].append({"file": rel, "why": f"{type(e).__name__}: {str(e)[:80]}"})
            elif p.suffix == ".jsonl":
                try:
                    lines = p.read_text().splitlines()
                except UnicodeDecodeError as e:
                    res["bad"].append({"file": rel, "why": f"UnicodeDecodeError: {e}"})
                    continue
                res["index"][rel] = sum(1 for line in lines if line.strip())      # a JSONL's lines
                for n, line in enumerate(lines, 1):
                    if not line.strip():
                        continue
                    res["jsonl_lines"] += 1
                    try:
                        json.loads(line)
                    except ValueError as e:
                        res["bad"].append({"file": rel, "line": n, "why": f"{type(e).__name__}: {str(e)[:80]}"})
            else:
                res["other"] += 1
    return res


def lost_or_shrunk(before: dict, after: dict) -> list:
    """Files the reboot lost, or JSONL files it left with fewer lines (pure)."""
    b, a = (before or {}).get("index") or {}, (after or {}).get("index") or {}
    out = [{"file": f, "why": "gone"} for f in b if f not in a]
    out += [{"file": f, "why": f"{b[f]} -> {a[f]} lines"} for f in b
            if f in a and f.endswith(".jsonl") and a[f] < b[f]]
    return out


def judge(ms: dict) -> dict:
    r = []
    up = all(v == "active/enabled" for v in (ms.get("units_before") or {"x": ""}).values())
    valid = ms.get("boot_changed") is True and ms.get("driving_at_reboot") is True and up
    if not up:
        r.append(f"invalid: the units were not all up before the run ({ms.get('units_before')})")
    if ms.get("boot_changed") is not True:
        r.append("invalid: the boot id did not change (no reboot)")
    if ms.get("driving_at_reboot") is not True:
        r.append("invalid: the mission was not driving at the reboot")
    b = ms.get("stack_back_s")
    if b is None or b > BACK_BAR_S:
        r.append(f"the stack back {b} s after the reboot (bar {BACK_BAR_S:.0f})")
    files = ms.get("files_after") or {}
    if not files or files.get("bad") is None:
        r.append("the files were not read after the reboot")
    elif files["bad"]:
        r.append(f"{len(files['bad'])} unparseable: {files['bad'][:3]}")
    gone = lost_or_shrunk(ms.get("files_before"), files)
    if files and gone:
        r.append(f"{len(gone)} files lost or shortened by the reboot: {gone[:3]}")
    if ms.get("next_started") is not True:
        r.append(f"the next mission did not start ({ms.get('next_start')})")
    if ms.get("units_after") != ms.get("units_before"):
        r.append(f"the units not as found: {ms.get('units_after')} vs {ms.get('units_before')}")
    passed = not [x for x in r if not x.startswith("invalid")] and valid
    return {"pass": passed, "valid": valid, "reasons": r}


# ------------------------------------------------------------ the board side

def _http(method: str, url: str, body=None, secret: str = "", timeout: float = 5.0):
    import urllib.error
    import urllib.request
    req = urllib.request.Request(url, method=method, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json",
                                          **({"x-app-secret": secret} if secret else {})})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b"null")
    except urllib.error.HTTPError as e:
        return e.code, (e.read() or b"")[:300].decode(errors="replace")
    except (OSError, ValueError) as e:
        return None, f"{type(e).__name__}: {e}"


def _secret() -> str:
    for line in Path("~/.vision-picar-local-secrets").expanduser().read_text().splitlines():
        if line.strip().startswith("APP_SHARED_SECRET="):
            return line.split("=", 1)[1].strip().strip("'\"")
    return ""


def _units() -> dict:
    out = {}
    for u in UNITS:
        a = subprocess.run(["systemctl", "is-active", u], capture_output=True, text=True).stdout.strip()
        e = subprocess.run(["systemctl", "is-enabled", u], capture_output=True, text=True).stdout.strip()
        out[u] = f"{a}/{e}"
    return out


def board_state() -> dict:
    code_r, _ = _http("GET", ROBOT + "/health", timeout=2)
    code_b, _ = _http("GET", BRAIN + "/health", timeout=2)
    code_v, body_v = _http("GET", VE + "/health", timeout=2)
    return {"boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
            "uptime_s": float(Path("/proc/uptime").read_text().split()[0]),
            "robot": code_r, "brain": code_b,
            "ve_ready": code_v == 200 and isinstance(body_v, dict) and body_v.get("ready") is True,
            "units": _units()}


def board_files() -> dict:
    return check_tree(Path(BOARD_DIR).expanduser())


def board_start(drive_wait_s: float = DRIVE_WAIT_S) -> dict:
    """A frontier mission started and seen driving (a step under way)."""
    secret = _secret()
    code, body = _http("POST", BRAIN + "/mission/start", {"target_object": "red backpack", "policy": "frontier"},
                       secret=secret, timeout=60)
    res = {"start": code, "start_body": body if code != 200 else None}
    if code != 200:
        return res
    t0, end = time.monotonic(), time.monotonic() + drive_wait_s
    while time.monotonic() < end:
        c, st = _http("GET", BRAIN + "/mission/status", secret=secret, timeout=3)
        if c == 200 and isinstance(st, dict) and st.get("running") and (st.get("step") or 0) >= 1:
            res["driving"] = True
            res["driving_s"] = round(time.monotonic() - t0, 1)
            res["step"] = st.get("step")
            return res
        time.sleep(0.5)
    res["driving"] = False
    return res


def board_mission() -> dict:
    c, st = _http("GET", BRAIN + "/mission/status", secret=_secret(), timeout=3)
    st = st if isinstance(st, dict) else {}
    return {"code": c, "running": st.get("running"), "step": st.get("step"), "outcome": st.get("outcome")}


def board_stop() -> dict:
    code, body = _http("POST", BRAIN + "/mission/stop", {}, secret=_secret(), timeout=30)
    return {"stop": code}


def board_main(cmd: str) -> None:
    fn = {"state": board_state, "files": board_files, "start": board_start, "stop": board_stop,
          "mission": board_mission}[cmd]
    print(json.dumps(fn()))


# ---------------------------------------------------------- the desktop side

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "picar-jetson"]


def on_board(cmd: str, timeout: float = 180.0) -> dict:
    r = subprocess.run([*SSH, "python3", "-", "board", cmd], input=Path(__file__).read_text(),
                       capture_output=True, text=True, timeout=timeout)
    lines = r.stdout.strip().splitlines()
    if r.returncode != 0 or not lines:                 # a board going down can answer 0 with nothing
        raise RuntimeError(f"board {cmd}: {r.returncode} {r.stderr.strip()[:300]}")
    return json.loads(lines[-1])


def locked(argv: list, why: str) -> int:
    """`argv` (this file's subcommand) under the board lock, waiting for it."""
    py = sys.executable
    return subprocess.call([py, str(ROOT / "tools/jetson/board_lock.py"), "run", "--wait", "3600",
                            "--why", why, "--", py, str(Path(__file__).resolve()), *argv], cwd=ROOT)


def pre(out: Path) -> None:
    """Step 1 (under the lock): state, files, a mission driving, the reboot."""
    ms: dict = {}
    ms["before"] = on_board("state")
    ms["units_before"] = ms["before"]["units"]
    ms["files_before"] = on_board("files")
    s = on_board("start", timeout=DRIVE_WAIT_S + 90)
    ms["start"] = s
    ms["driving_at_reboot"] = s.get("driving") is True
    if ms["driving_at_reboot"]:
        time.sleep(REBOOT_AFTER_S)
        ms["before_reboot"] = on_board("state")
        ms["mission_at_reboot"] = on_board("mission")
        # Mid-mission means still running when the reboot goes out.
        ms["driving_at_reboot"] = ms["mission_at_reboot"].get("running") is True
    if ms["driving_at_reboot"]:
        # CLOCK_MONOTONIC is one clock for every process on this host (the steps are two).
        ms["reboot_mono"] = time.monotonic()
        r = subprocess.run([*SSH, "sudo", "-n", "systemctl", "reboot"], capture_output=True, text=True, timeout=30)
        ms["reboot_rc"] = r.returncode
        if r.returncode not in (0, 255):           # refused (255: the board dropped the session)
            ms["driving_at_reboot"] = False
            ms["reboot_refused"] = r.stderr.strip()[:300]
            on_board("stop")
    else:
        on_board("stop")
    (out / "pre.json").write_text(json.dumps(ms, indent=1))


def back(out: Path, ms: dict) -> None:
    """Step 2 (no lock: the board is rebooting): each service timed from the
    reboot command, on the desktop's CLOCK_MONOTONIC."""
    t0 = ms["reboot_mono"]
    seen: dict = {}
    end = t0 + BACK_WAIT_S
    old = ms["before"]["boot_id"]
    while time.monotonic() < end:
        try:
            st = on_board("state", timeout=15)
        except Exception:                  # noqa: BLE001 -- a board going down answers anything
            time.sleep(1.0)
            continue
        dt = round(time.monotonic() - t0, 1)
        if st["boot_id"] != old:
            seen.setdefault("ssh_s", dt)
            if st["robot"] == 200:
                seen.setdefault("robot_s", dt)
            if st["brain"] == 200:
                seen.setdefault("brain_s", dt)
            if st["units"]["picar-ngrok"].startswith("active/"):
                seen.setdefault("ngrok_s", dt)
            if st["ve_ready"]:
                seen.setdefault("ve_s", dt)
            if {"robot_s", "brain_s", "ngrok_s", "ve_s"} <= set(seen):
                break
        time.sleep(1.0)
    ms["back"] = seen
    ms["boot_changed"] = "ssh_s" in seen
    ms["stack_back_s"] = max(seen["robot_s"], seen["brain_s"]) if {"robot_s", "brain_s"} <= set(seen) else None


def post(out: Path) -> None:
    """Step 3 (under the lock): the files, the next mission, the units."""
    ms = json.loads((out / "pre.json").read_text())
    ms["files_after"] = on_board("files")
    nxt = on_board("start", timeout=DRIVE_WAIT_S + 90)
    ms["next_start"] = nxt
    ms["next_started"] = nxt.get("start") == 200 and nxt.get("driving") is True
    ms["next_stop"] = on_board("stop")
    ms["after"] = on_board("state")
    ms["units_after"] = ms["after"]["units"]
    (out / "post.json").write_text(json.dumps(ms, indent=1))


def run_one(out: Path, i: int) -> dict:
    d = out / f"F14-{i}"
    d.mkdir(parents=True, exist_ok=True)
    rec = {"row": "F14", "i": i, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    try:
        # The lock's holder dies with the board, so its exit code says nothing:
        # the step's own record does.
        locked(["_pre", str(d)], f"3.135.B F14 run {i}: a mission, then sudo reboot")
        if not (d / "pre.json").exists():
            raise RuntimeError("the pre-reboot step wrote no record (failed, or the board was busy)")
        ms = json.loads((d / "pre.json").read_text())
        if ms.get("driving_at_reboot"):
            back(d, ms)
            (d / "pre.json").write_text(json.dumps(ms, indent=1))
            locked(["_post", str(d)], f"3.135.B F14 run {i}: after the reboot")
            if not (d / "post.json").exists():
                raise RuntimeError("the post-reboot step wrote no record")
            ms = json.loads((d / "post.json").read_text())
        rec["measures"] = ms
    except Exception as e:                 # noqa: BLE001 -- recorded, the row goes on
        rec["error"] = f"{type(e).__name__}: {e}"
        rec["measures"] = rec.get("measures") or {}
    rec["verdict"] = judge(rec["measures"])
    rec["ended"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    (d / "result.json").write_text(json.dumps(rec, indent=1))
    return rec


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == "board":
        board_main(argv[1])
        return 0
    if argv and argv[0] in ("_pre", "_post"):
        (pre if argv[0] == "_pre" else post)(Path(argv[1]))
        return 0
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--runs", type=int, default=3)
    a = ap.parse_args(argv)
    sys.path.insert(0, str(HERE))
    from rows import verdict
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    recs = []
    for i in range(1, a.runs + 4):
        if sum(1 for r in recs if r.get("error") is None and r["verdict"]["valid"]) >= a.runs:
            break
        if recs:
            time.sleep(SETTLE_S)
        rec = run_one(out, i)
        recs.append(rec)
        print(json.dumps({"i": i, "verdict": rec["verdict"], "error": rec.get("error")}), flush=True)
    v = {"row": "F14", **verdict(recs, a.runs)}
    (out / "row.json").write_text(json.dumps(v, indent=1))
    print(json.dumps(v))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
