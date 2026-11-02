"""
evaluations/jetson-perf-3136/boardrun.py

3.136.A: the desktop's runner for board tests (reusable by 3.135.A). It
drives the existing board tools over ssh; it rewrites none of them.

What it gives a test step:

* `locked(argv, why, desktop_mode)` -- re-runs the calling script under both
  locks, outermost first: the desktop lock (`tools/desktop_lock.py`,
  exclusive for timed runs, shared otherwise, `--no-wait`) and the board lock
  (`tools/jetson/board_lock.py run --why ...`, no wait: another session's
  hold refuses the run with exit 75). `board_lock.py run` releases the board
  however the inner process ends (its own `finally`), so no trap here can be
  skipped.
* `snapshot()` / `differences()` -- the board's as-found state: the twin's
  and the VLM's units (active, enabled), `/run/picar-vlm-hold`, the VLM's env
  file (sha256 and mode, never its contents), running containers, the
  service ports, the power mode. Criterion 4 compares the snapshot taken
  after a run with the one before.
* `restore()` -- puts back what differs: units started or stopped, a hold
  flag this run's bench left. It needs `sudo -n` on the board and is refused
  unless the run passed `allow_sudo` (3.136.A ran without sudo, the user's
  decision for that step); a no-sudo run changes no unit, so it has nothing
  to put back, and any difference is reported, never hidden.
* `push_checkout(sha)` -- the commit under test as a git checkout in
  `~/bench-3136` on the board (3.131's method: `restart.sh` checks each
  server's revision against a git checkout), with the service checkout's
  venv, weights and the rsynced `recordings/` linked in.
* `Sampler` -- the board's telemetry for a run, without sudo: `tegrastats`
  (it runs as the user on JetPack 6) and, once a second, epoch,
  `MemAvailable`, `/proc/vmstat`'s `oom_kill` counter (dmesg is restricted
  to root), every readable thermal zone, every CPU's and the GPU's current
  and maximum clock from sysfs (JetPack 6's tegrastats prints no
  throttling, 3.76 P1). The desktop's load average is sampled beside it.

The pieces are plain functions over an `ssh` callable, so tests drive them
with a fake board.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Optional, Sequence

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
PY = sys.executable
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
       "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=4", "picar-jetson"]
BOARD_REPO = "vision-picar"          # ~/vision-picar on the board: pushes land here
BENCH_DIR = "bench-3136"             # ~/bench-3136: the commit under test
SERVICE_DIR = "vision-picar-service"
UNITS = ("picar-stack", "picar-ngrok", "picar-local-vlm")
HOLD_FLAG = "/run/picar-vlm-hold"
VLM_ENV = "~/.vision-picar-local-vlm.env"
PORTS = (8000, 8001, 8002, 8003, 8004, 8005, 8080, 8090)
# Keys whose change means the board was not left as found (criterion 4).
# MemAvailable, load and uptime move by themselves and are recorded only.
COMPARED = ("units", "hold_flag", "vlm_env", "containers", "listening", "power_mode")


class BoardError(RuntimeError):
    pass


def ssh(cmd: str, *, check: bool = True, timeout: float = 120.0, input: Optional[str] = None) -> str:
    """One command on the board through bash -lc-free `bash -c`; stdout."""
    r = subprocess.run(SSH + [f"bash -c {shlex.quote(cmd)}"], capture_output=True, text=True,
                       timeout=timeout, input=input)
    if check and r.returncode != 0:
        raise BoardError(f"board command failed ({r.returncode}): {cmd[:120]}\n{r.stderr.strip()[-800:]}")
    return r.stdout


# --------------------------------------------------------------- the locks

def locked(argv: Sequence[str], why: str, desktop_mode: str = "exclusive",
           desktop_wait: bool = False) -> int:
    """Run `argv` (a python command line) under the desktop lock, then the
    board lock. Returns its exit code; 75 if either lock is busy. The board
    lock never waits (another session's hold refuses the run); the desktop
    lock waits only with `desktop_wait`."""
    if desktop_mode not in ("exclusive", "shared"):
        raise ValueError(desktop_mode)
    cmd = [PY, str(ROOT / "tools/desktop_lock.py"), desktop_mode,
           *([] if desktop_wait else ["--no-wait"]), "--why", why, "--",
           PY, str(ROOT / "tools/jetson/board_lock.py"), "run", "--why", why, "--", *argv]
    return subprocess.call(cmd, cwd=ROOT)


def require_board_lock() -> str:
    """Inside `locked`: the board token, or refuse (never run unlocked)."""
    tok = os.environ.get("PICAR_BOARD_TOKEN", "")
    if not tok:
        raise BoardError("no PICAR_BOARD_TOKEN: board actions run only under board_lock.py")
    return tok


# ------------------------------------------------------- as found, and back

SNAPSHOT_SH = r"""
set -u
units='%(units)s'
echo "{"
echo "\"units\": {"
first=1
for u in $units; do
  [ $first = 1 ] || echo ","; first=0
  printf '"%%s": {"active": "%%s", "enabled": "%%s"}' "$u" \
    "$(systemctl is-active $u 2>/dev/null)" "$(systemctl is-enabled $u 2>/dev/null)"
done
echo "},"
if [ -e %(flag)s ]; then printf '"hold_flag": "%%s",\n' "$(tr -d '\n"\\' < %(flag)s)"; else echo '"hold_flag": null,'; fi
f=%(env)s
if [ -e "$f" ]; then
  printf '"vlm_env": {"sha256": "%%s", "mode": "%%s"},\n' "$(sha256sum "$f" | cut -d' ' -f1)" "$(stat -c %%a "$f")"
else echo '"vlm_env": null,'; fi
printf '"containers": "%%s",\n' "$(docker ps --format '{{.Names}}' 2>/dev/null | sort | tr '\n' ' ')"
printf '"listening": "%%s",\n' "$(for p in %(ports)s; do ss -ltnH "sport = :$p" 2>/dev/null | grep -q . && printf '%%s ' $p; done)"
printf '"power_mode": "%%s",\n' "$(nvpmodel -q 2>/dev/null | grep -m1 'NV Power Mode')"
printf '"mem_available_mb": %%s,\n' "$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)"
printf '"oom_kill": %%s,\n' "$(awk '/^oom_kill /{print $2}' /proc/vmstat)"
printf '"uptime_s": %%s,\n' "$(cut -d' ' -f1 /proc/uptime)"
printf '"epoch": %%s\n' "$(date +%%s.%%N)"
echo "}"
"""


def snapshot(run: Callable[..., str] = ssh) -> dict:
    sh = SNAPSHOT_SH % {"units": " ".join(UNITS), "flag": HOLD_FLAG, "env": VLM_ENV,
                        "ports": " ".join(map(str, PORTS))}
    return json.loads(run(sh))


def differences(before: dict, after: dict) -> list:
    """Every compared key that is not as found, as human-readable lines."""
    out = []
    for k in COMPARED:
        b, a = before.get(k), after.get(k)
        if k == "units":
            for u in sorted(set(b or {}) | set(a or {})):
                if (b or {}).get(u) != (a or {}).get(u):
                    out.append(f"unit {u}: {(b or {}).get(u)} -> {(a or {}).get(u)}")
        elif b != a:
            out.append(f"{k}: {b!r} -> {a!r}")
    return out


def stop_twin(run: Callable[..., str] = ssh, *, allow_sudo: bool) -> None:
    """README sections 4-5: the twin's units off before a bench (the bench's
    own `vlm_hold` holds the VLM)."""
    if not allow_sudo:
        raise BoardError("stopping picar-stack/picar-ngrok needs sudo; this run has allow_sudo off")
    run("sudo -n systemctl stop picar-stack picar-ngrok")


def restore(before: dict, run: Callable[..., str] = ssh, *, allow_sudo: bool) -> list:
    """Put back what differs from `before`; returns the actions taken. Never
    touches the env file (nothing here writes it): a changed one is only
    reported by the next `differences()`."""
    now = snapshot(run)
    diffs = differences(before, now)
    if not diffs:
        return []
    if not allow_sudo:
        return [f"NOT RESTORED (allow_sudo off): {d}" for d in diffs]
    acts = []
    # The hold flag first: while it exists the VLM's unit skips every start.
    if before.get("hold_flag") is None and now.get("hold_flag") is not None:
        owner = str(now["hold_flag"]).split()
        alive = False
        if len(owner) == 2 and owner[0].isdigit():
            # vlm_hold.sh's stamp: "<pid> <start time>"; alive only if both match.
            alive = run(f"cut -d')' -f2 /proc/{owner[0]}/stat 2>/dev/null | awk '{{print $20}}'",
                        check=False).strip() == owner[1]
        if alive:
            acts.append(f"hold flag kept: its owner {now['hold_flag']} is alive")
        else:
            run(f"sudo -n rm -f {HOLD_FLAG}")
            acts.append(f"removed a stale hold flag ({now['hold_flag']})")
    for u in UNITS:
        want_en, have_en = before["units"][u]["enabled"], now["units"][u]["enabled"]
        if want_en != have_en and want_en in ("enabled", "disabled"):
            run(f"sudo -n systemctl {want_en[:-1]} {u}", check=False)
            acts.append(f"{want_en[:-1]}d {u} (was {have_en})")
        want = before["units"][u]["active"]
        have = now["units"][u]["active"]
        if want == have:
            continue
        if want in ("active", "activating") and have not in ("active", "activating"):
            run(f"sudo -n systemctl reset-failed {u} 2>/dev/null; sudo -n systemctl start {u}", check=False)
            acts.append(f"started {u} (was {have})")
        elif want not in ("active", "activating") and have in ("active", "activating"):
            run(f"sudo -n systemctl stop {u}", check=False)
            acts.append(f"stopped {u} (was {want} before the run)")
    return acts


def settle_and_check(before: dict, run: Callable[..., str] = ssh, *, wait_s: float = 90.0,
                     poll_s: float = 3.0, sleep=time.sleep) -> tuple:
    """Units take a while to come up (the VLM loads ~40 s, the stack ~7 s):
    poll until the compared state equals `before` or `wait_s` passes.
    Returns (after_snapshot, differences)."""
    end = time.monotonic() + wait_s
    while True:
        after = snapshot(run)
        diffs = differences(before, after)
        if not diffs or time.monotonic() >= end:
            return after, diffs
        sleep(poll_s)


# ------------------------------------------------------- the commit, there

def push_checkout(sha: str, run: Callable[..., str] = ssh, bench_dir: str = BENCH_DIR) -> str:
    """The commit under test at ~/<bench_dir> on the board (3.136's
    `bench-3136` by default; 3.135 passes `bench-3135`), a git checkout, on
    a board branch of the same name. Returns the board's `git rev-parse
    HEAD` there (must equal `sha`)."""
    # A plain name starting `bench-`: never the board's service worktree or
    # its repo, which a forced checkout here would detach under the twin.
    # Pinned to a plan section's own name (bench-3NNN): `bench-378` and the
    # like hold the board's shipped engines.
    if not re.fullmatch(r"bench-3\d{3}", bench_dir):
        raise BoardError(f"bench_dir {bench_dir!r}: only a section's own bench-3NNN")
    full = subprocess.run(["git", "rev-parse", sha], cwd=ROOT, capture_output=True, text=True,
                          check=True).stdout.strip()
    # Our own ref on the board's repo; `+` because the bench ref moves with each commit tested.
    subprocess.run(["git", "push", "-q", f"picar-jetson:{BOARD_REPO}",
                    f"+{full}:refs/heads/{bench_dir}"], cwd=ROOT, check=True,
                   capture_output=True, text=True, timeout=300)
    got = run(f"""set -e
if [ ! -d ~/{bench_dir}/.git ]; then git clone -q --no-checkout ~/{BOARD_REPO} ~/{bench_dir}; fi
cd ~/{bench_dir}
git fetch -q ~/{BOARD_REPO} {bench_dir}
git checkout -q -f --detach {full}
[ -e .venv ] || ln -s ~/{SERVICE_DIR}/.venv .venv
for f in yoloe-11s-seg.pt mobileclip_blt.ts; do [ -e "$f" ] || ln -s ~/{SERVICE_DIR}/"$f" "$f"; done
[ -e recordings ] || ln -s ~/{BOARD_REPO}/recordings recordings
git rev-parse HEAD""", timeout=300).strip()
    if got != full:
        raise BoardError(f"the board's checkout is {got}, not {full}")
    return full


def fetch(board_dir: str, local_dir: Path) -> None:
    """Copy a run's directory back from the board."""
    local_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(["rsync", "-a", "-e", " ".join(SSH[:-1]),
                    f"picar-jetson:{board_dir}/", f"{local_dir}/"], check=True, timeout=600)


# --------------------------------------------------------------- telemetry

SAMPLER_SH = r"""
out=%(out)s; mkdir -p "$out"
num() { case "$1" in ''|*[!0-9]*) return 1;; esac; }
n() { if num "$1"; then printf '%%s' "$1"; else printf null; fi; }
tegrastats --interval 1000 --logfile "$out/tegrastats.log" & echo $! > "$out/sampler.pids"
(
  while true; do
    # Every value through n(): an unreadable file is null, never a broken line.
    printf '{"t": %%s, "mem_mb": %%s, "oom_kill": %%s, "temps": {' "$(date +%%s.%%N)" \
      "$(n "$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)")" "$(n "$(awk '/^oom_kill /{print $2}' /proc/vmstat)")"
    sep=
    for z in /sys/class/thermal/thermal_zone*; do
      v=$(cat $z/temp 2>/dev/null) || continue; num "$v" || continue
      printf '%%s"%%s": %%s' "$sep" "$(tr -dc 'A-Za-z0-9_-' < $z/type)" "$v"; sep=", "
    done
    printf '}, "cpu_khz": ['; sep=
    for c in /sys/devices/system/cpu/cpu[0-9]*/cpufreq; do
      printf '%%s[%%s, %%s]' "$sep" "$(n "$(cat $c/scaling_cur_freq 2>/dev/null)")" "$(n "$(cat $c/scaling_max_freq 2>/dev/null)")"; sep=", "
    done
    g=$(ls -d /sys/class/devfreq/*.gpu /sys/class/devfreq/*.ga10b 2>/dev/null | head -1)
    printf '], "gpu_hz": [%%s, %%s]}\n' "$(n "$(cat $g/cur_freq 2>/dev/null)")" "$(n "$(cat $g/max_freq 2>/dev/null)")"
    sleep 1
  done
) > "$out/sysfs.jsonl" 2>/dev/null &
echo $! >> "$out/sampler.pids"
"""


class Sampler:
    """Board telemetry (no sudo) plus the desktop's load, for one run."""

    def __init__(self, board_out: str, run: Callable[..., str] = ssh, period_s: float = 1.0):
        self.board_out, self.run, self.period_s = board_out, run, period_s
        self.desktop_load: list = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        # nohup + setsid: the samplers outlive this ssh session until stop().
        self.run(f"setsid nohup bash -c {shlex.quote(SAMPLER_SH % {'out': self.board_out})} "
                 f">/dev/null 2>&1 < /dev/null & sleep 1.5; cat {self.board_out}/sampler.pids")
        self._thread = threading.Thread(target=self._desktop, daemon=True)
        self._thread.start()

    def _desktop(self) -> None:
        while not self._stop.is_set():
            self.desktop_load.append([round(time.time(), 1), *[round(x, 2) for x in os.getloadavg()]])
            self._stop.wait(self.period_s)

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        # The subshell loop and tegrastats, by the pids recorded at start.
        self.run(f"for p in $(cat {self.board_out}/sampler.pids 2>/dev/null); do "
                 f"pkill -P $p 2>/dev/null; kill $p 2>/dev/null; done; true", check=False)


def read_sysfs(path: Path) -> list:
    rows = []
    for line in Path(path).read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            continue          # a line cut by the kill
    return rows


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
