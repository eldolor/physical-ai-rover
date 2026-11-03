# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
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

* `lease()` / `Heartbeat` (3.136.F) -- a run whose desktop goes away still
  ends on the board. `board_run` sends this file to `<out>` and starts it
  there as a lease beside the bench (`python3 <out>/lease_boardrun.py lease
  <out>`, detached, stdlib only -- the harness's own copy, never the commit
  under test's), and beats down one held ssh session every `HEARTBEAT_S`.
  With no beat for `LEASE_SILENCE_S` the lease ends the bench's process tree
  (`<out>/bench.pid`, pid and start time) and the samplers, then -- only if
  it can take the board lock -- removes the containers not running as found,
  puts the systemd target back and `restore()`s the units from the as-found
  snapshot it was handed; `<out>/lease.json` says what it did. `board_run`
  lets it go (`<out>/lease.stop`) once its bench has ended, and if the bench
  ssh failed it leaves the lease to act and waits for its record. *Why:*
  on 2026-11-03 a desktop runner stopped mid-row left its bench running,
  the twin's units stopped and the samplers on, beside the next session's
  bench (3.135.B, amendment 10); since 2026-11-02 a soak's samplers outlived
  their desktop, too.

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


# 3.136.F: the lease. 90 s of silence is a desktop gone (the board lock's own
# silence limit); a bench gets two minutes to tear itself down before SIGKILL
# (a soak's teardown isolates the systemd target back).
HEARTBEAT_S = 15.0
LEASE_SILENCE_S = 90.0
LEASE_POLL_S = 5.0
LEASE_TERM_WAIT_S = 120.0


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


# ---------------------------------------------------- 3.136.F: the lease

def restore_and_check(before: dict, run: Callable[..., str] = ssh, *, allow_sudo: bool) -> tuple:
    """`restore()`, then `settle_and_check()` for as long as a restore that
    did something needs (units come up slowly): (actions, after, diffs).
    Shared by the desktop's end of a run and the lease's."""
    acts = restore(before, run, allow_sudo=allow_sudo)
    acted = any(not a.startswith(("NOT RESTORED", "hold flag kept")) for a in acts)
    after, diffs = settle_and_check(before, run, wait_s=120 if acted else 5)
    return acts, after, diffs


class Heartbeat:
    """The desktop's half: one held ssh session, a byte down its stdin every
    `period_s`, the board touching `<path>/heartbeat` per byte. Held, as
    board_lock.py's keeper is, so a fault row that stops NEW ssh logins
    (memory, a full disk) cannot fake a desktop gone. A session that ends
    while the desktop lives (the board side killed, a hiccup) is opened again
    at the next beat and counted (`losses`), so only a desktop that stays
    away for `LEASE_SILENCE_S` reads as gone. `spawn(cmd)` starts the board
    side with a writable stdin (tests: a local bash)."""

    def __init__(self, path: str, period_s: float = HEARTBEAT_S, spawn=None):
        self.path, self.period_s = path, period_s
        self.spawn = spawn or (lambda cmd: subprocess.Popen([*SSH, cmd], stdin=subprocess.PIPE,
                                                            stdout=subprocess.DEVNULL,
                                                            stderr=subprocess.DEVNULL))
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.proc = None
        self.losses = 0

    def _open(self) -> None:
        self.proc = self.spawn(f"while read -r _; do touch {self.path}/heartbeat; done")

    def start(self) -> None:
        self._open()
        self._thread = threading.Thread(target=self._beat, daemon=True, name="board-heartbeat")
        self._thread.start()

    def _beat(self) -> None:
        while not self._stop.is_set():
            try:
                if self.proc is None or self.proc.poll() is not None:
                    self.losses += 1
                    self._open()
                self.proc.stdin.write(b"\n")
                self.proc.stdin.flush()
            except (OSError, ValueError):
                self.proc = None            # opened again at the next beat
            self._stop.wait(self.period_s)

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        if self.proc is not None:
            try:
                self.proc.stdin.close()
            except OSError:
                pass
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()


def local_run(cmd: str, check: bool = True, timeout: float = 120.0, input: Optional[str] = None) -> str:
    """The board's own shell, for the lease (it runs there): `ssh`'s shape."""
    r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=timeout, input=input)
    if check and r.returncode != 0:
        raise BoardError(f"{cmd[:80]}: exit {r.returncode}: {r.stderr.strip()[:200]}")
    return r.stdout


def _stat(pid: int) -> Optional[list]:
    """/proc/<pid>/stat's fields after the command name (state is [0], ppid
    [1], start time [19]), or None."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rsplit(")", 1)[1].split()
    except (OSError, IndexError):
        return None


def _alive(pid: int) -> bool:
    st = _stat(pid)
    return st is not None and st[0] != "Z"


def start_time(pid: int) -> Optional[str]:
    st = _stat(pid)
    return st[19] if st else None


def _descendants(roots) -> list:
    """`roots` and every process under them, by parent pid (a child in its
    own session, as `stack.py` starts each role, is still a child)."""
    kids: dict = {}
    for d in os.listdir("/proc"):
        if d.isdigit():
            st = _stat(int(d))
            if st:
                kids.setdefault(int(st[1]), []).append(int(d))
    out, todo = [], list(roots)
    while todo:
        p = todo.pop()
        if p in out:
            continue
        out.append(p)
        todo.extend(kids.get(p, []))
    return out


def end_tree(pid: int, term_wait_s: float, *, allow_sudo: bool = False) -> dict:
    """SIGTERM `pid`'s whole tree at once (each process its own teardown:
    soak.sh's EXIT trap, soak.py's handlers), wait up to `term_wait_s`, then
    SIGKILL the tree as it is THEN (walked again from every member, so what
    a teardown forked meanwhile goes too). Every pid is kept with its start
    time and signalled only while it still has it, so a pid reused meanwhile
    is never touched. A process this user may not signal (a `sudo` step's)
    is killed with `sudo -n kill -9` when the run has sudo. Returns the pids
    signalled and any still alive after.

    Known limit: a process the bench daemonised (re-parented to init) is not
    in the tree."""
    import signal
    born = {p: start_time(p) for p in _descendants([pid]) if _alive(p)}

    def same(p):
        return _alive(p) and start_time(p) == born.get(p)

    for p in list(born):
        try:
            os.kill(p, signal.SIGTERM)
        except OSError:
            pass
    end = time.monotonic() + term_wait_s
    while time.monotonic() < end and any(same(p) for p in born):
        time.sleep(0.2)
    for _ in range(3):
        for p in _descendants([q for q in born if same(q)]):
            if p not in born and _alive(p):
                born[p] = start_time(p)     # forked by a teardown since: its own start time
        left = [p for p in born if same(p)]
        if not left:
            break
        denied = []
        for p in left:
            try:
                os.kill(p, signal.SIGKILL)
            except PermissionError:
                denied.append(p)
            except OSError:
                pass
        if denied and allow_sudo:
            subprocess.run(["sudo", "-n", "kill", "-9", *map(str, denied)], capture_output=True, timeout=10)
        time.sleep(0.5)
    return {"signalled": sorted(born), "survivors": [p for p in born if same(p)]}


def _take_board(lock_dir: Path, who: str):
    """The board lock (3.93's file), without waiting: an open file holding it,
    or None if another session has it now."""
    import fcntl
    try:
        f = open(lock_dir / ".picar-board.lock", "a")
    except OSError:
        return None
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.close()
        return None
    try:
        tmp = lock_dir / ".picar-board.owner.tmp"
        tmp.write_text(who + "\n")
        os.replace(tmp, lock_dir / ".picar-board.owner")
    except OSError:
        pass
    return f


LOCK_RETRY_S = 150.0   # longer than a dying desktop holder's silence limit plus a keeper's poll
FALLBACK_DIR = Path("/dev/shm")


def _state(out: Path, state: str) -> None:
    tmp = out / "lease.state.tmp"
    tmp.write_text(state + "\n")
    os.replace(tmp, out / "lease.state")


def fallback_record(out: Path) -> Path:
    """Where a lease's record goes when `<out>` cannot be written (a full
    disk): RAM, named after the run's directory."""
    return FALLBACK_DIR / f"picar-lease-{Path(out).name}.json"


def lease(out, *, silence_s: float = LEASE_SILENCE_S, period_s: float = LEASE_POLL_S,
          run: Callable[..., str] = local_run, allow_sudo: bool = False,
          term_wait_s: float = LEASE_TERM_WAIT_S, lock_dir=None, lock_retry_s: float = LOCK_RETRY_S) -> str:
    """The board's half: wait for `<out>/lease.stop` (the desktop let go:
    "released") or `silence_s` without a heartbeat (the desktop gone), then:

    0. `lease.state` goes `acting` BEFORE `lease.stop` is read a last time, so
       a desktop that lets go reads `acting` after its stop if the lease got
       there first (and waits for its record), and never both act unseen;
    1. end this run's own bench (`bench.pid`: "<pid> <start time>", checked
       so a reused pid is never killed) and its samplers (only a pid still
       running a sampler of this run), always -- they are this run's, and on
       a board someone else now holds they are in the way;
    2. only with the board lock taken here (retried for `lock_retry_s`: the
       dead desktop's holder and the bench's own keeper let go a little
       later): remove the containers that were not running as found, put the
       systemd target back if the bench moved it, and restore the units. If
       another session holds the board, its state is its own: left, said so.

    Silence is counted on the monotonic clock from the last heartbeat SEEN
    (its mtime changing), never wall clock minus mtime: a clock step on the
    board (3.135's F16) is no silence. Writes `<out>/lease.json` (or, on a
    full disk, `fallback_record(out)`): "acted"."""
    out = Path(out).expanduser()
    lock_dir = Path(lock_dir or os.environ.get("PICAR_BOARD_LOCK_DIR") or Path.home())
    (out / "lease.pid").write_text(f"{os.getpid()}\n")
    _state(out, "watching")

    def mtime():
        try:
            return (out / "heartbeat").stat().st_mtime_ns
        except OSError:
            return None

    last, seen_at = mtime(), time.monotonic()
    while True:
        if (out / "lease.stop").exists():
            _state(out, "released")
            return "released"
        m = mtime()
        if m != last:
            last, seen_at = m, time.monotonic()
        silence = time.monotonic() - seen_at
        if silence > silence_s:
            _state(out, "acting")
            if (out / "lease.stop").exists():          # let go just now: the desktop restores
                _state(out, "released")
                return "released"
            break
        time.sleep(period_s)
    rec: dict = {"silence_s": round(silence, 1), "acted_at": time.time()}
    survivors = []
    try:
        pid_s, start_s = ((out / "bench.pid").read_text().split() + [""])[:2]
        pid = int(pid_s)
        if start_time(pid) is None:
            rec["bench_ended"] = "the bench had already ended"
        elif start_s and start_time(pid) != start_s:
            rec["bench_ended"] = f"pid {pid} is no longer the bench (start time {start_time(pid)}, not {start_s})"
        else:
            rec["bench_ended"] = end_tree(pid, term_wait_s, allow_sudo=allow_sudo)
            survivors += rec["bench_ended"]["survivors"]
    except (OSError, ValueError) as e:
        rec["bench_ended"] = f"no bench pid: {e}"
    samplers, skipped = [], []
    for f in (out / "sampler.pids", out / "telemetry" / "sampler.pids"):
        try:
            pids = [int(p) for p in f.read_text().split()]
        except (OSError, ValueError):
            continue
        for p in pids:
            try:
                cmd = Path(f"/proc/{p}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            except OSError:
                continue                    # gone already
            if "tegrastats" not in cmd and str(out.name) not in cmd:
                skipped.append(p)           # not a sampler of this run any more: a reused pid
                continue
            r = end_tree(p, 5.0, allow_sudo=allow_sudo)
            samplers += r["signalled"]
            survivors += r["survivors"]
    rec["samplers_ended"], rec["sampler_pids_skipped"] = samplers, skipped
    rec["survivors"] = survivors
    who = f"3.136.F lease of {out} (pid {os.getpid()}), since {time.strftime('%Y-%m-%dT%H:%M:%S%z')}"
    held, end = _take_board(lock_dir, who), time.monotonic() + lock_retry_s
    while held is None and time.monotonic() < end:
        time.sleep(2.0)
        held = _take_board(lock_dir, who)
    rec["board_lock"] = "taken" if held else "held by another session: its containers and units left"
    try:
        if held and survivors:
            rec["error"] = f"processes of the bench survived ({survivors}): units not restored beside them"
        elif held:
            before = json.loads((out / "as_found.json").read_text())
            found = set((before.get("containers") or "").split())
            now = snapshot(run)
            extra = sorted(set((now.get("containers") or "").split()) - found)
            for c in extra:                 # docker needs no sudo for the bench's user
                run(f"docker rm -f {shlex.quote(c)}", check=False, timeout=60)
            rec["containers_removed"] = extra
            tgt = (out / "target_at_start").read_text().strip() if (out / "target_at_start").exists() else ""
            if tgt == "graphical" and allow_sudo and \
                    run("systemctl is-active graphical.target", check=False).strip() != "active":
                run("sudo -n systemctl isolate graphical.target", check=False, timeout=120)
                rec["target_restored"] = "graphical.target"
            rec["restore_actions"], _after, rec["after_differences"] = restore_and_check(
                before, run, allow_sudo=allow_sudo)
    except Exception as e:  # noqa: BLE001 -- what was done is still written
        rec["error"] = f"{type(e).__name__}: {e}"
    finally:
        if held:
            held.close()
    text = json.dumps(rec, indent=1, default=str)
    try:
        (out / "lease.json").write_text(text)
    except OSError:
        fallback_record(out).write_text(text)       # a full disk: the record in RAM
    try:
        _state(out, "done")
    except OSError:
        pass
    return "acted"


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


def main(argv=None) -> int:
    """`python3 boardrun.py lease <out> [--silence S] [--sudo]`: the lease, on the board."""
    import argparse
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    le = sub.add_parser("lease")
    le.add_argument("out")
    le.add_argument("--silence", type=float, default=LEASE_SILENCE_S)
    le.add_argument("--sudo", action="store_true")
    a = ap.parse_args(argv)
    print(lease(a.out, silence_s=a.silence, allow_sudo=a.sudo), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
