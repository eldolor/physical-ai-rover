# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.136.A's harness against a fake board (no ssh, no locks):

    .venv/bin/python -m pytest evaluations/jetson-perf-3136/test_harness.py -q
"""

import copy
import json
import sys
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import boardrun as br  # noqa: E402
import run as runner  # noqa: E402
import scorers  # noqa: E402
import validity  # noqa: E402

GOOD = {"kind": "full", "window": [100.0, 400.0], "bench": [110.0, 390.0],
        "wheel": {"late": 0, "moving": 6000}, "oom_kills": 0, "nav": {"ok": 18, "n": 18},
        "power_mode": "NV Power Mode: 15W", "desktop_load": [[1, 0.5, 0.5, 0.5]],
        "temps": 300, "clocks": 300}


def test_a_good_run_is_valid_and_scored():
    assert validity.judge(GOOD) == {"valid": True, "reasons": []}
    assert validity.score(GOOD, 183) == 183


@pytest.mark.parametrize("change, reason", [
    ({"kind": "quiet"}, "not a full-stack run"),
    ({"window": None}, "no full-stack window"),
    ({"bench": [90.0, 390.0]}, "before the window"),
    ({"bench": [110.0, 401.0]}, "after the window"),
    ({"wheel": {"late": 1, "moving": 6000}}, "1 late wheel ticks"),
    ({"wheel": {"late": 0, "moving": 0}}, "no moving wheel ticks"),
    ({"oom_kills": 1}, "1 OOM kills"),
    ({"oom_kills": None}, "OOM kills not counted"),
    ({"nav": {"ok": 17, "n": 18}}, "17/18"),
    ({"nav": {"ok": 0, "n": 0}}, "no nav2 goals"),
    ({"power_mode": "NV Power Mode: MAXN"}, "not 15W"),
    ({"desktop_load": []}, "desktop load"),
    ({"temps": 0}, "temperature"),
    ({"clocks": 0}, "clocks"),
])
def test_each_broken_condition_flags_the_run_and_it_is_not_scored(change, reason):
    run = {**GOOD, **change}
    j = validity.judge(run)
    assert not j["valid"]
    assert any(reason in r for r in j["reasons"]), j["reasons"]
    assert validity.score(run, 183) is None


def test_telemetry_counts_inside_the_window():
    rows = [{"t": t, "mem_mb": 900 - t, "oom_kill": 3 + (t >= 5), "temps": {"cpu": 45000, "tj": 51000},
             "cpu_khz": [[729600, 1497600]], "gpu_hz": [306000000, 612000000]} for t in range(10)]
    c = validity.telemetry_counts(rows, 2, 7)
    assert c["samples"] == 6 and c["temps"] == 6 and c["clocks"] == 6
    assert c["max_temp_c"] == 51.0 and c["oom_kills"] == 1 and c["mem_available_min_mb"] == 893
    assert c["cpu_below_max_samples"] == 6 and c["gpu_below_max_samples"] == 6
    rows[3]["temps"]["PMIC-Die"] = 100000
    rows[4]["gpu_hz"] = [None, None]
    c = validity.telemetry_counts(rows, 2, 7)
    assert c["max_temp_c"] == 51.0 and c["clocks"] == 5


def test_perception_scorer_reads_frame_lines_inside_the_window_only():
    t0 = time.mktime(time.strptime("2026-11-01 10:00:00", "%Y-%m-%d %H:%M:%S"))
    log = "loaded 1 pipeline\n09:59:59 900 absent\n10:00:00 120 detected\n10:00:01 130 absent\n" \
          "10:00:05 999 absent\ndone: 4 frames\n"
    assert scorers.perception_ms(log) == [900, 120, 130, 999]
    assert scorers.perception_ms(log, t0, t0 + 2) == [120, 130]


def test_percentile_is_nearest_rank():
    assert scorers.percentile(list(range(1, 11)), 90) == 9
    assert scorers.percentile([5], 90) == 5
    assert scorers.percentile([], 90) is None


def test_nav_counts_and_headroom_window_across_midnight():
    assert scorers.nav_counts("a: succeeded\ngoals 17/18 succeeded\n") == {"ok": 17, "n": 18}
    assert scorers.nav_counts("nothing") is None
    t0, t1 = scorers.headroom_window("== window start 23:59:50; x\n== window end 00:00:10 bench", "2026-11-01")
    assert t1 - t0 == 20


def test_wheel_delta():
    d = scorers.wheel_delta({"wheel_loop": {"moving_ticks": 10, "late_ticks": 1, "max_dt_s": 0.06}},
                            {"wheel_loop": {"moving_ticks": 210, "late_ticks": 2, "max_dt_s": 0.152}})
    assert d == {"moving": 200, "late": 1, "max_dt_s": 0.152, "max_dt_s_before": 0.06}


# ------------------------------------------------------------ a fake board

FOUND = {"units": {u: {"active": "active", "enabled": "enabled"} for u in br.UNITS},
         "hold_flag": None, "vlm_env": {"sha256": "ab", "mode": "600"}, "containers": "",
         "listening": "8000 8001 8005 8080 ", "power_mode": "NV Power Mode: 15W",
         "mem_available_mb": 2500, "oom_kill": 0, "uptime_s": 1.0, "epoch": 1.0}


class FakeBoard:
    def __init__(self, state=None):
        self.state = copy.deepcopy(state or FOUND)
        self.cmds = []
        self.lease_state = "watching"
        self.lease_dead = False
        self.bench_after = "BENCH-ALIVE"

    def __call__(self, cmd, check=True, timeout=0, input=None):
        self.cmds.append(cmd)
        if "systemctl is-active" in cmd and "units=" in cmd:       # the snapshot script
            return json.dumps(self.state)
        if "sudo -n systemctl stop picar-stack picar-ngrok" in cmd:
            for u in ("picar-stack", "picar-ngrok"):
                self.state["units"][u]["active"] = "inactive"
            return ""
        if "sudo -n systemctl" in cmd and "start" in cmd:
            u = cmd.split()[-1]
            self.state["units"][u]["active"] = "active"
            return ""
        if "sudo -n rm -f" in cmd:
            self.state["hold_flag"] = None
            return ""
        # The bench, the lease's start and its release read /proc too: they answer first.
        if "echo $?" in cmd:
            return "0\n"
        if "lease_boardrun.py lease" in cmd:
            return "4242\nOOM-NOT-SET\nalive 99\n"                 # the lease's pid, alive 2 s on, its start
        if "BENCH-ALIVE" in cmd:
            return self.bench_after + "\n"
        if "lease.stop" in cmd:
            return "STOPPED\n" + self.lease_state + "\n" + ("" if self.lease_dead else "LEASE-ALIVE\n")
        if "lease.json" in cmd:
            return ""
        if "/proc/" in cmd and "stat" in cmd:
            return ""                                             # the flag's owner is gone
        return ""


@pytest.fixture(autouse=True)
def _ram(tmp_path, monkeypatch):
    """The lease's RAM files (its state, a full disk's record) under the test's own directory."""
    (tmp_path / "shm").mkdir(exist_ok=True)
    monkeypatch.setattr(br, "FALLBACK_DIR", tmp_path / "shm")


class FakeHeartbeat:
    def __init__(self, path):
        self.path = path

    def start(self):
        pass

    def stop(self):
        pass


def test_differences_name_what_moved_and_ignore_what_moves_by_itself():
    after = copy.deepcopy(FOUND)
    after["mem_available_mb"] = 100
    after["uptime_s"] = 99
    assert br.differences(FOUND, after) == []
    after["units"]["picar-ngrok"]["active"] = "inactive"
    after["hold_flag"] = "123 456"
    d = br.differences(FOUND, after)
    assert len(d) == 2 and "picar-ngrok" in d[0] and "hold_flag" in d[1]


def test_without_sudo_nothing_is_stopped_and_a_difference_is_reported_not_hidden():
    b = FakeBoard()
    with pytest.raises(br.BoardError):
        br.stop_twin(b, allow_sudo=False)
    b.state["units"]["picar-ngrok"]["active"] = "failed"
    acts = br.restore(FOUND, b, allow_sudo=False)
    assert acts and acts[0].startswith("NOT RESTORED")
    assert not any("sudo" in c for c in b.cmds)


def test_with_sudo_restore_puts_back_units_and_a_stale_hold_flag():
    b = FakeBoard()
    br.stop_twin(b, allow_sudo=True)
    b.state["units"]["picar-local-vlm"]["active"] = "inactive"
    b.state["hold_flag"] = "4242 99"
    acts = br.restore(FOUND, b, allow_sudo=True)
    assert any("stale hold flag" in a for a in acts)
    assert sum(a.startswith("started") for a in acts) == 3
    after, diffs = br.settle_and_check(FOUND, b, wait_s=0)
    assert diffs == [] and after["hold_flag"] is None
    # The flag goes before any start: while it exists the VLM's starts are skipped.
    rm = next(i for i, c in enumerate(b.cmds) if "rm -f" in c)
    first_start = next(i for i, c in enumerate(b.cmds) if "systemctl start" in c)
    assert rm < first_start


def test_board_run_refuses_without_the_board_lock(monkeypatch, tmp_path):
    monkeypatch.delenv("PICAR_BOARD_TOKEN", raising=False)
    with pytest.raises(br.BoardError):
        runner.board_run("x", "true", tmp_path, run=FakeBoard())


class FakeSampler:
    def __init__(self, out, run):
        self.desktop_load = [[1, 0.1, 0.1, 0.1]]

    def start(self):
        pass

    def stop(self):
        pass


def test_a_bench_whose_ssh_breaks_is_left_to_the_lease_and_nothing_is_done_beside_it(monkeypatch, tmp_path):
    """3.136.F: was "board_run restores even when the command breaks the board".
    With no exit code back the bench may still run there: this side touches
    nothing (no restore, no release), records that, and returns -- its board
    hold ends, and the lease ends the bench and restores."""
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "t0k")
    b = FakeBoard()

    def push(sha, run, bench_dir=br.BENCH_DIR):
        return "f" * 40

    def bad_cmd_board(cmd, check=True, timeout=0, input=None):
        if "echo $?" in cmd:          # the bench: stops the VLM, then its ssh dies
            b.state["units"]["picar-local-vlm"]["active"] = "inactive"
            raise RuntimeError("ssh dropped")
        return b(cmd, check, timeout, input)

    with pytest.raises(RuntimeError):
        runner.board_run("x", "true", tmp_path, allow_sudo=True, stop_twin=True, run=bad_cmd_board,
                         push=push, fetch=lambda *a: None, sampler_cls=FakeSampler,
                         heartbeat_cls=FakeHeartbeat, sleep=lambda s: None)
    rec = json.loads((tmp_path / "record.json").read_text())
    assert rec["lease"].startswith("left to the lease") and rec["left_as_found"] is None
    assert not any(("systemctl start" in c or "lease.stop" in c or "docker rm" in c) for c in b.cmds)


def test_board_run_passes_the_token_to_the_board_command(monkeypatch, tmp_path):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "t0k")
    b = FakeBoard()
    rec = runner.board_run("x", "echo hi", tmp_path, run=b, push=lambda s, r, bench_dir=None: "f" * 40,
                           fetch=lambda *a: None, sampler_cls=FakeSampler, heartbeat_cls=FakeHeartbeat)
    cmd = next(c for c in b.cmds if "echo hi" in c)
    assert "PICAR_BOARD_TOKEN=t0k" in cmd and rec["cmd_exit"] == 0 and rec["left_as_found"]
    assert not any("sudo" in c for c in b.cmds)


def test_plant_verdict_tolerance():
    import plant
    assert plant.verdict(30, 131)["met"] and plant.verdict(30, 139.9)["met"]
    assert not plant.verdict(30, 141)["met"] and not plant.verdict(30, 119)["met"]


# ------------------------------------------------- 3.136.B's scorers

def test_controller_rate_reads_only_executing_spans():
    rows = [{"k": "fp", "t": 10.0, "goals": [["a", 2]]}, {"k": "fp", "t": 11.0, "goals": [["a", 4]]}]
    rows += [{"k": "cmd", "t": 10.0 + i * 0.05} for i in range(21)]        # 20 Hz while executing
    rows += [{"k": "cmd", "t": 12.0 + i * 0.1} for i in range(10)]         # a behaviour, after
    r = scorers.controller_rate(rows)
    assert r["intervals"] == 20 and abs(r["rate_hz"] - 20.0) < 0.01


def test_scan_lag_and_nav_goals():
    rows = [{"k": "scan", "t": 5.0 + i, "stamp": 5.0 + i - 0.007} for i in range(10)]
    assert abs(scorers.scan_lag(rows)["p90_ms"] - 7.0) < 0.01
    nav = [{"k": "nav", "t": 1.0, "goals": [["a", 2]]}, {"k": "nav", "t": 2.0, "goals": [["a", 4], ["b", 2]]},
           {"k": "nav", "t": 3.0, "goals": [["a", 4], ["b", 5]]}, {"k": "nav", "t": 9.0, "goals": [["c", 6]]}]
    assert scorers.nav_goals(nav, 0, 5) == {"n": 2, "succeeded": 1, "canceled": 1, "aborted": 0}
    assert scorers.nav_goals(nav)["aborted"] == 1


def test_room_work_pause_share():
    rows = [{"k": "segment", "t0": 100.0 + 5 * i, "t1": 100.2 + 5 * i} for i in range(10)]
    rows += [{"k": "walk", "t0": 100.2 + 5 * i, "t1": 100.25 + 5 * i} for i in range(10)]
    r = scorers.room_work(rows, 100.0, 150.0)
    assert r["segment"]["n"] == 10 and abs(r["pause_share"] - 0.05) < 1e-6


def test_overlaps_needs_every_group_and_counts_map_updates():
    g = {"a": [[0, 2], [5, 6]], "b": [[1, 3]], "c": [[1.5, 1.8], [5.2, 5.5]]}
    o = scorers.overlaps(g, [1.6, 5.3])
    assert o["all_spans"] == 1 and abs(o["all_s"] - 0.3) < 1e-9 and o["all_with_map_update"] == 1


# ---------------------------------------- 3.136.F: the lease (a lost desktop)

import os  # noqa: E402
import signal  # noqa: E402
import subprocess  # noqa: E402


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    with open(f"/proc/{pid}/stat") as f:
        return f.read().rsplit(")", 1)[1].split()[0] != "Z"


def _reap(*procs):
    for p in procs:
        try:
            p.wait(timeout=0.1)
        except subprocess.TimeoutExpired:
            pass


class LeaseBoard(FakeBoard):
    """The board as the lease sees it: the twin stopped by the bench, a
    container the bench started still running."""

    def __init__(self):
        super().__init__()
        for u in ("picar-stack", "picar-ngrok"):
            self.state["units"][u]["active"] = "inactive"
        self.state["containers"] = "picar-ros-3135 "

    def __call__(self, cmd, check=True, timeout=0, input=None):
        if cmd.startswith("docker rm -f"):
            self.cmds.append(cmd)
            self.state["containers"] = ""
            return ""
        return super().__call__(cmd, check, timeout, input)


def test_a_lease_whose_desktop_lets_go_does_nothing(tmp_path):
    (tmp_path / "heartbeat").touch()
    (tmp_path / "as_found.json").write_text(json.dumps(FOUND))
    (tmp_path / "lease.stop").touch()
    b = LeaseBoard()
    assert br.lease(tmp_path, silence_s=60, period_s=0.01, run=b, allow_sudo=True,
                    lock_dir=tmp_path) == "released"
    assert b.cmds == [] and not (tmp_path / "lease.json").exists()


def test_a_lost_desktop_ends_the_bench_its_samplers_and_containers_and_restores_the_units(tmp_path):
    # The bench: a shell with a child, and a grandchild in its own session (as stack.py starts
    # every role), all to be ended; the samplers, by their recorded pids.
    bench = subprocess.Popen(["bash", "-c", "sleep 300 & setsid sleep 300 & wait"])
    # samplers of this run: their command lines name its directory (the sysfs loop's does)
    samplers = [subprocess.Popen(["bash", "-c", f"sleep 300; : {tmp_path.name}"]) for _ in range(2)]
    time.sleep(0.3)
    kids = [int(p) for p in subprocess.run(["pgrep", "-P", str(bench.pid)], capture_output=True,
                                           text=True).stdout.split()]
    assert len(kids) == 2
    (tmp_path / "bench.pid").write_text(f"{bench.pid} {br.start_time(bench.pid)}\n")
    (tmp_path / "telemetry").mkdir()                         # where board_run's Sampler writes them
    (tmp_path / "telemetry" / "sampler.pids").write_text("".join(f"{p.pid}\n" for p in samplers))
    (tmp_path / "as_found.json").write_text(json.dumps(FOUND))
    hb = tmp_path / "heartbeat"
    hb.touch()
    os.utime(hb, (time.time() - 120, time.time() - 120))       # the desktop went quiet 2 min ago
    b = LeaseBoard()
    try:
        assert br.lease(tmp_path, silence_s=0.3, period_s=0.01, lock_retry_s=0.1, run=b, allow_sudo=True,
                        term_wait_s=2.0, lock_dir=tmp_path) == "acted"
        _reap(bench, *samplers)
        assert not any(_alive(p) for p in [bench.pid, *kids, *(s.pid for s in samplers)])
        assert br.state_path(tmp_path).read_text().strip() == "done"
        assert "docker rm -f picar-ros-3135" in b.cmds
        rec = json.loads((tmp_path / "lease.json").read_text())
        assert sum(a.startswith("started") for a in rec["restore_actions"]) == 2
        assert rec["after_differences"] == [] and rec["silence_s"] >= 0.3 and rec["board_lock"] == "taken"
    finally:
        for p in [bench, *samplers]:
            if p.poll() is None:
                p.kill()
        for k in kids:
            try:
                os.kill(k, signal.SIGKILL)
            except ProcessLookupError:
                pass


def test_a_lost_desktop_without_sudo_ends_the_bench_and_reports_the_units(tmp_path):
    bench = subprocess.Popen(["sleep", "300"])
    (tmp_path / "bench.pid").write_text(f"{bench.pid} {br.start_time(bench.pid)}\n")
    (tmp_path / "as_found.json").write_text(json.dumps(FOUND))
    hb = tmp_path / "heartbeat"
    hb.touch()
    os.utime(hb, (time.time() - 120, time.time() - 120))
    b = LeaseBoard()
    try:
        assert br.lease(tmp_path, silence_s=0.3, period_s=0.01, lock_retry_s=0.1, run=b, allow_sudo=False,
                        term_wait_s=2.0, lock_dir=tmp_path) == "acted"
        _reap(bench)
        assert not _alive(bench.pid)
        rec = json.loads((tmp_path / "lease.json").read_text())
        acts = rec["restore_actions"]
        assert "removed container picar-ros-3135" in acts           # docker needs no sudo
        assert all(a.startswith("NOT RESTORED") for a in acts if "picar-" in a and "container" not in a)
        assert not any("sudo" in c for c in b.cmds)
    finally:
        if bench.poll() is None:
            bench.kill()


def test_board_run_holds_a_lease_heartbeats_it_and_lets_it_go_before_its_restore(monkeypatch, tmp_path):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "t0k")
    b = FakeBoard()
    rec = runner.board_run("x", "echo hi", tmp_path, run=b, push=lambda s, r, bench_dir=None: "f" * 40,
                           fetch=lambda *a: None, sampler_cls=FakeSampler, heartbeat_cls=FakeHeartbeat)
    send = next(i for i, c in enumerate(b.cmds) if "cat > " in c and "lease_boardrun.py" in c and "lease ~/" not in c)
    lease = next(i for i, c in enumerate(b.cmds) if "lease_boardrun.py lease" in c)
    bench = next(i for i, c in enumerate(b.cmds) if "echo hi" in c)
    stop = next(i for i, c in enumerate(b.cmds) if "lease.stop" in c)
    assert send < lease < bench < stop and rec["lease"] == "released"
    assert "/proc/$$/stat" in b.cmds[bench] and "bench.pid" in b.cmds[bench]
    assert "as_found.json" in b.cmds[lease] and "setsid nohup" in b.cmds[lease]
    # the commit under test's boardrun.py is never the lease: the harness's own copy is
    assert "evaluations/jetson-perf-3136/boardrun.py" not in b.cmds[lease]
    # the lease is let go only once the board is put back (a desktop lost mid-restore is still covered)
    snaps = [i for i, c in enumerate(b.cmds) if "systemctl is-active" in c and "units=" in c]
    assert snaps[-1] < stop and rec["left_as_found"]


def test_board_run_refuses_to_run_a_bench_with_no_lease(monkeypatch, tmp_path):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "t0k")
    b = FakeBoard()

    def no_lease(cmd, check=True, timeout=0, input=None):          # started, then gone
        return "4242\n" if "lease_boardrun.py lease" in cmd else b(cmd, check, timeout, input)

    with pytest.raises(br.BoardError, match="lease did not start"):
        runner.board_run("x", "echo hi", tmp_path, run=no_lease, push=lambda s, r, bench_dir=None: "f" * 40,
                         fetch=lambda *a: None, sampler_cls=FakeSampler, heartbeat_cls=FakeHeartbeat)
    assert not any("echo hi" in c for c in b.cmds)


def test_the_heartbeat_is_one_held_session_touching_the_file_per_beat(tmp_path):
    hb = br.Heartbeat(str(tmp_path), period_s=0.02,
                      spawn=lambda cmd: subprocess.Popen(["bash", "-c", cmd], stdin=subprocess.PIPE))
    hb.start()
    time.sleep(0.2)
    first = (tmp_path / "heartbeat").stat().st_mtime
    time.sleep(0.2)
    assert (tmp_path / "heartbeat").stat().st_mtime > first
    hb.stop()
    assert hb.proc.poll() is not None                       # the session ends with the run


def test_a_reused_bench_pid_is_never_killed(tmp_path):
    other = subprocess.Popen(["sleep", "300"])
    try:
        (tmp_path / "bench.pid").write_text(f"{other.pid} 1\n")     # the bench's start time was 1
        (tmp_path / "as_found.json").write_text(json.dumps(FOUND))
        hb = tmp_path / "heartbeat"
        hb.touch()
        os.utime(hb, (time.time() - 120, time.time() - 120))
        br.lease(tmp_path, silence_s=0.3, period_s=0.01, lock_retry_s=0.1, run=LeaseBoard(), allow_sudo=True,
                 term_wait_s=1.0, lock_dir=tmp_path)
        assert _alive(other.pid)
        assert "no longer the bench" in json.loads((tmp_path / "lease.json").read_text())["bench_ended"]
    finally:
        other.kill()


def test_a_lease_whose_board_another_session_holds_ends_its_bench_and_leaves_the_rest(tmp_path):
    import fcntl
    bench = subprocess.Popen(["sleep", "300"])
    lock = open(tmp_path / ".picar-board.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX)                         # another session's hold
    try:
        (tmp_path / "bench.pid").write_text(f"{bench.pid} {br.start_time(bench.pid)}\n")
        (tmp_path / "as_found.json").write_text(json.dumps(FOUND))
        hb = tmp_path / "heartbeat"
        hb.touch()
        os.utime(hb, (time.time() - 120, time.time() - 120))
        b = LeaseBoard()
        br.lease(tmp_path, silence_s=0.3, period_s=0.01, lock_retry_s=0.1, run=b, allow_sudo=True, term_wait_s=2.0,
                 lock_dir=tmp_path)
        _reap(bench)
        assert not _alive(bench.pid) and b.cmds == []           # no container, no unit touched
        assert "held by another session" in json.loads((tmp_path / "lease.json").read_text())["board_lock"]
    finally:
        lock.close()
        if bench.poll() is None:
            bench.kill()


def test_a_beating_heartbeat_keeps_the_lease_quiet_even_across_a_clock_step(tmp_path):
    """Silence is the monotonic time since the mtime last CHANGED: a beat whose
    stamp is hours old (the board's clock stepped back) is still a beat."""
    import threading
    (tmp_path / "as_found.json").write_text(json.dumps(FOUND))
    hb = tmp_path / "heartbeat"
    hb.touch()
    stop = threading.Event()

    def beat():
        k = 0
        while not stop.is_set():
            k += 1
            os.utime(hb, (time.time() - 7200 + k, time.time() - 7200 + k))   # 2 h in the past, moving
            stop.wait(0.05)
            if k == 15:
                (tmp_path / "lease.stop").touch()

    th = threading.Thread(target=beat)
    th.start()
    try:
        got = br.lease(tmp_path, silence_s=0.3, period_s=0.01, run=LeaseBoard(), lock_dir=tmp_path)
    finally:
        stop.set()
        th.join()
    assert got == "released" and not (tmp_path / "lease.json").exists()


def test_a_sampler_pid_now_running_something_else_is_not_killed(tmp_path):
    other = subprocess.Popen(["sleep", "300"])                    # not a sampler of this run
    try:
        (tmp_path / "sampler.pids").write_text(f"{other.pid}\n")
        (tmp_path / "as_found.json").write_text(json.dumps(FOUND))
        br.lease(tmp_path, silence_s=0.2, period_s=0.01, run=LeaseBoard(), lock_dir=tmp_path,
                 lock_retry_s=0.1)
        assert _alive(other.pid)
        assert json.loads((tmp_path / "lease.json").read_text())["sampler_pids_skipped"] == [other.pid]
    finally:
        other.kill()


def test_the_lease_retries_the_board_lock_while_a_dying_holder_lets_go(tmp_path):
    import fcntl
    import threading
    lock = open(tmp_path / ".picar-board.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX)
    (tmp_path / "as_found.json").write_text(json.dumps(FOUND))
    threading.Timer(0.5, lock.close).start()                    # the dead desktop's holder, 0.5 s on
    b = LeaseBoard()
    br.lease(tmp_path, silence_s=0.2, period_s=0.01, run=b, lock_dir=tmp_path, lock_retry_s=5.0,
             allow_sudo=True)
    rec = json.loads((tmp_path / "lease.json").read_text())
    assert rec["board_lock"] == "taken" and "docker rm -f picar-ros-3135" in b.cmds


def test_a_full_disk_keeps_the_lease_acting_and_its_record_in_ram(tmp_path, monkeypatch):
    (tmp_path / "as_found.json").write_text(json.dumps(FOUND))
    real = Path.write_text

    def full(self, *a, **k):                       # the run's directory full; RAM is not
        if self.parent == tmp_path and self.name not in ("as_found.json", "lease.pid"):   # written before it filled
            raise OSError(28, "No space left on device")
        return real(self, *a, **k)

    monkeypatch.setattr(Path, "write_text", full)
    monkeypatch.setattr(br, "_take_board", lambda d, who: open(os.devnull))       # the lock file is on the full disk too
    b = LeaseBoard()
    assert br.lease(tmp_path, silence_s=0.2, period_s=0.01, run=b, lock_dir=tmp_path,
                    lock_retry_s=0.1, allow_sudo=True) == "acted"
    assert "docker rm -f picar-ros-3135" in b.cmds                  # it acted, though nothing could be written
    assert json.loads(br.fallback_record(tmp_path).read_text())["board_lock"] == "taken"
    assert not (tmp_path / "lease.json").exists()


def _run_with(monkeypatch, tmp_path, board, **kw):
    monkeypatch.setenv("PICAR_BOARD_TOKEN", "t0k")
    return runner.board_run("x", "true", tmp_path, allow_sudo=True, stop_twin=True, run=board,
                            push=lambda s, r, bench_dir=None: "f" * 40, fetch=lambda *a: None,
                            sampler_cls=FakeSampler, heartbeat_cls=FakeHeartbeat, sleep=lambda s: None, **kw)


def test_an_ssh_that_returns_no_exit_code_is_not_a_bench_that_ended(monkeypatch, tmp_path):
    b = FakeBoard()
    def dropped(cmd, check=True, timeout=0, input=None):
        if "echo $?" in cmd:
            b.cmds.append(cmd)
            return ""                                              # ssh 255, nothing back
        return b(cmd, check, timeout, input)

    rec = _run_with(monkeypatch, tmp_path, dropped)
    assert rec["cmd_exit"] is None and rec["lease"].startswith("left to the lease") and rec["left_to_lease"]
    assert not any("lease.stop" in c or "systemctl start" in c for c in b.cmds)


def test_the_release_says_when_the_lease_acted_or_died_during_the_run(monkeypatch, tmp_path):
    b = FakeBoard()
    b.lease_state = "done"
    rec = _run_with(monkeypatch, tmp_path, b)
    assert rec["left_as_found"] and rec["lease"] == "the lease acted during the run: see lease.json"
    b = FakeBoard()
    b.lease_dead = True
    rec = _run_with(monkeypatch, tmp_path / "2", b) if (tmp_path / "2").mkdir() is None else None
    assert rec["lease"].startswith("the lease had died during the run")


def test_a_release_that_never_landed_is_not_called_released(monkeypatch, tmp_path):
    b = FakeBoard()

    def lost(cmd, check=True, timeout=0, input=None):
        return "" if "lease.stop" in cmd else b(cmd, check, timeout, input)

    rec = _run_with(monkeypatch, tmp_path, lost)
    assert rec["lease"].startswith("not released") and sum("lease.stop" in c for c in b.cmds) == 0


def test_every_bench_keeps_the_board_while_it_lives(monkeypatch, tmp_path):
    b = FakeBoard()
    _run_with(monkeypatch, tmp_path, b)
    bench = next(c for c in b.cmds if "echo $?" in c)
    assert "source tools/jetson/board_lock_check.sh" in bench


def test_a_bench_that_never_started_is_not_waited_for(monkeypatch, tmp_path):
    b = FakeBoard()

    def no_sudo(cmd, check=True, timeout=0, input=None):
        if "systemctl stop picar-stack" in cmd:
            raise br.BoardError("sudo -n refused")
        if "lease.json" in cmd:
            raise AssertionError("waited for a lease although no bench started")
        return b(cmd, check, timeout, input)

    with pytest.raises(br.BoardError):
        _run_with(monkeypatch, tmp_path, no_sudo)
    rec = json.loads((tmp_path / "record.json").read_text())
    assert rec["lease"] == "released" and rec["left_as_found"]




def test_a_bench_found_gone_after_its_ssh_failed_is_a_normal_end(monkeypatch, tmp_path):
    b = FakeBoard()
    b.bench_after = "BENCH-GONE"

    def dropped(cmd, check=True, timeout=0, input=None):
        if "echo $?" in cmd:
            b.cmds.append(cmd)
            return ""
        return b(cmd, check, timeout, input)

    rec = _run_with(monkeypatch, tmp_path, dropped)
    assert rec["bench_after_ssh"] == "gone" and rec["lease"] == "released" and rec["left_as_found"]


def test_a_refused_lock_check_comes_back_as_the_benchs_exit_code():
    import inspect
    src = inspect.getsource(runner.board_run)
    assert "( source tools/jetson/board_lock_check.sh ) && ( {board_cmd} )" in src


def test_an_exit_in_the_board_command_still_echoes_its_code(tmp_path):
    """The shape board_run sends, run for real: `exit 75` inside the command."""
    out = subprocess.run(["bash", "-c", "true && ( exit 75 ) > /dev/null 2>&1; echo $?"],
                         capture_output=True, text=True).stdout.strip()
    assert out == "75"


def test_the_lease_waits_out_its_own_runs_hold_but_not_anothers(tmp_path):
    import fcntl
    import threading
    (tmp_path / "as_found.json").write_text(json.dumps(FOUND))
    lock = open(tmp_path / ".picar-board.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX)
    (tmp_path / ".picar-board.owner").write_text("plan/x pid 1 (desk): 3.136 run, token t0k\n")
    threading.Timer(1.0, lock.close).start()          # this run's desktop finishes scoring at 1 s
    b = LeaseBoard()
    br.lease(tmp_path, silence_s=0.2, period_s=0.01, run=b, lock_dir=tmp_path, lock_retry_s=0.1,
             allow_sudo=True, token="t0k")
    assert json.loads((tmp_path / "lease.json").read_text())["board_lock"] == "taken"
    assert (tmp_path / ".picar-board.owner").read_text() == ""      # left as a released holder leaves it
