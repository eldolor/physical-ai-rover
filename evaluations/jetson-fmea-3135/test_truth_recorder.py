# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.151: the truth recorder under memory pressure. Criterion 2 -- a sample
never waits on the file (a write that blocks 1 s leaves no gap over 0.2 s
between samples, and every sample is written once it unblocks, in order);
criterion 1 -- the memory lock answers what happened, a refusal included,
and the stack locks the body by default, `FMEA_TRUTH_LOCK=0` skipping it."""

import ctypes
import errno
import json
import sys
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import stack  # noqa: E402
import truth_recorder as tr  # noqa: E402


class BlockingFile:
    """A file whose first write blocks `block_s`, as a write stalled in
    reclaim does; the lines it got, in order."""

    def __init__(self, block_s):
        self.block_s, self.lines, self.writes = block_s, [], 0

    def write(self, text):
        self.writes += 1
        if self.writes == 1:
            time.sleep(self.block_s)
        self.lines.extend(text.splitlines())

    def flush(self):
        pass


def _sampler():
    n = [0]

    def sample():
        n[0] += 1
        return {"t": round(time.monotonic(), 4), "i": n[0]}
    return sample


def test_a_blocked_write_leaves_no_hole_in_the_samples():
    f = BlockingFile(1.0)
    rec = tr.Recorder(_sampler(), f, period_s=0.02)
    rec.start()
    time.sleep(1.6)
    rec.stop()
    rows = [json.loads(x) for x in f.lines]
    ts = [r["t"] for r in rows]
    assert len(rows) > 60                                      # ~80 samples in 1.6 s
    assert max(b - a for a, b in zip(ts, ts[1:])) <= 0.2       # the old loop: a 1 s gap
    assert [r["i"] for r in rows] == list(range(1, len(rows) + 1))   # all, in order, once
    assert rec.latest == rows[-1]


def test_stop_drains_what_the_writer_holds():
    f = BlockingFile(0.5)
    rec = tr.Recorder(_sampler(), f, period_s=0.01)
    rec.start()
    time.sleep(0.3)                                            # the writer still blocked
    rec.stop()
    rows = [json.loads(x) for x in f.lines]
    assert rows and [r["i"] for r in rows] == list(range(1, len(rows) + 1))
    assert rows[-1]["i"] == rec.latest["i"]


def test_a_failed_write_is_reported_and_recording_goes_on(capsys):
    class Failing(BlockingFile):
        def write(self, text):
            self.writes += 1
            if self.writes == 1:
                raise OSError(errno.ENOSPC, "no space")
            self.lines.extend(text.splitlines())
    f = Failing(0)
    rec = tr.Recorder(_sampler(), f, period_s=0.01)
    rec.start()
    time.sleep(0.3)
    rec.stop()
    assert "truth write failed" in capsys.readouterr().err
    assert len(f.lines) > 5


def test_the_lock_reports_success_and_refusal(monkeypatch):
    calls = []

    class Libc:
        def mlockall(self, flags):
            calls.append(flags)
            return 0
    monkeypatch.setattr(tr, "_libc", lambda: Libc())
    import resource
    monkeypatch.setattr(tr.resource, "getrlimit", lambda r: (8 << 20, 8 << 20))
    got = tr.lock_memory()                          # a finite limit: MCL_FUTURE would break mmaps
    assert calls == [] and got["locked"] is False and got["error"] == "limit_not_raised"
    inf = resource.RLIM_INFINITY
    monkeypatch.setattr(tr.resource, "getrlimit", lambda r: (inf, inf))
    got = tr.lock_memory()
    assert calls == [tr.MCL_CURRENT | tr.MCL_FUTURE | tr.MCL_ONFAULT] and got["locked"] is True
    assert got["limit_soft_kb"] == "unlimited"
    assert {"errno", "error", "limit_soft_kb", "limit_hard_kb", "vm_lck_kb", "vm_rss_kb"} <= set(got)

    class Refusing:
        def mlockall(self, flags):
            ctypes.set_errno(errno.ENOMEM)
            return -1
    monkeypatch.setattr(tr, "_libc", lambda: Refusing())
    got = tr.lock_memory()
    assert got["locked"] is False and got["error"] == "ENOMEM"


def test_the_lock_on_this_machine_answers_without_raising():
    got = tr.lock_memory(dry=True)          # never locks the test process
    assert got["locked"] is None and got["limit_soft_kb"] is not None


def test_the_body_serves_the_lock_and_records_through_the_recorder():
    text = (HERE / "truth_body.py").read_text()
    assert '@app.post("/fmea/mlock")' in text and "truth_recorder" in text


class FakeStack(stack.Stack):
    def __init__(self, out, sudo_ok, extra_env=None):
        super().__init__(out, extra_env=extra_env)
        self.sudo_ok, self.ran, self.posted = sudo_ok, [], []

    def _sudo(self, argv):
        self.ran.append(argv)
        return self.sudo_ok

    def _post_lock(self):
        self.posted.append(True)
        return {"locked": self.sudo_ok, "error": None if self.sudo_ok else "ENOMEM"}

    def pid(self, role):
        return 4242


@pytest.mark.parametrize("sudo_ok", [True, False])
def test_the_stack_locks_the_body_by_default(tmp_path, sudo_ok):
    s = FakeStack(tmp_path, sudo_ok)
    got = s.lock_body()
    assert s.ran and "prlimit" in s.ran[0] and "--pid" in s.ran[0] and "4242" in s.ran[0]
    assert s.posted == [True] and got["limit_raised"] is sudo_ok
    assert json.loads((tmp_path / "body_lock.json").read_text())["locked"] is sudo_ok


def test_fmea_truth_lock_0_skips_the_lock(tmp_path):
    s = FakeStack(tmp_path, True, extra_env={"FMEA_TRUTH_LOCK": "0"})
    got = s.lock_body()
    assert s.ran == [] and s.posted == [] and got == {"locked": False, "skipped": "FMEA_TRUTH_LOCK=0"}
    assert json.loads((tmp_path / "body_lock.json").read_text())["skipped"]


def test_a_truth_row_keeps_its_fields_order_and_rounding(tmp_path):
    """The real body's `sample()` (in a child: it builds the simulator): the
    same keys in the same order as the loop before 3.151, its rounding, and
    the line `Recorder` writes is that row's `json.dumps` plus a newline."""
    import os
    import subprocess
    code = ("import json, sys; sys.path.insert(0, %r); import truth_body as tb; "
            "r = tb.sample(); print(json.dumps(r))" % str(HERE))
    env = dict(os.environ, FMEA_DIR=str(tmp_path), SIM_BODY_SHM=f"picar_3151_test_{os.getpid()}")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env,
                         timeout=120, cwd=str(HERE.parents[1]))
    assert out.returncode == 0, out.stderr[-500:]
    row = json.loads(out.stdout.strip().splitlines()[-1])
    assert list(row) == ["t", "x", "y", "th", "cam_th", "sp", "gap_cm"]
    assert row["t"] == round(row["t"], 4) and row["x"] == round(row["x"], 5)
    assert isinstance(row["sp"], list) and len(row["sp"]) == 2
    f = BlockingFile(0)
    rec = tr.Recorder(lambda: row, f, period_s=0.01)
    rec.start()
    time.sleep(0.05)
    rec.stop()
    assert f.lines[0] == json.dumps(row)


def test_a_writer_blocked_past_stop_is_the_only_writer():
    """stop() never writes beside a writer still blocked: its rows come
    after the blocked batch, in order (review)."""
    f = BlockingFile(1.5)
    rec = tr.Recorder(_sampler(), f, period_s=0.01)
    rec.start()
    time.sleep(0.2)
    assert rec.stop(timeout_s=0.3) is False          # still blocked
    rec._writer.join(3)
    rows = [json.loads(x) for x in f.lines]
    assert [r["i"] for r in rows] == list(range(1, len(rows) + 1)) and rows[-1]["i"] == rec.latest["i"]


def test_a_sample_that_raises_is_a_hole_not_a_dead_sampler(capsys):
    n = [0]

    def sample():
        n[0] += 1
        if n[0] == 3:
            raise RuntimeError("geometry")
        return {"t": time.monotonic(), "i": n[0]}
    f = BlockingFile(0)
    rec = tr.Recorder(sample, f, period_s=0.01)
    rec.start()
    time.sleep(0.2)
    rec.stop()
    got = [json.loads(x)["i"] for x in f.lines]
    assert 3 not in got and got[:3] == [1, 2, 4] and len(got) > 8
    assert "truth sample failed" in capsys.readouterr().err


def test_the_backlog_is_capped_and_counted(monkeypatch):
    monkeypatch.setattr(tr, "BACKLOG_MAX", 5)
    f = BlockingFile(0.5)
    rec = tr.Recorder(_sampler(), f, period_s=0.005)
    rec.start()
    time.sleep(0.3)
    assert len(rec._lines) <= 5 and rec.dropped > 0
    rec.stop()


def test_a_lock_post_that_raises_is_reported(tmp_path, monkeypatch):
    s = stack.Stack(tmp_path)
    monkeypatch.setattr(stack, "http", lambda *a, **k: (_ for _ in ()).throw(TimeoutError("slow")))
    got = s._post_lock()
    assert got["locked"] is False and "TimeoutError" in got["error"]


def test_sigterm_drains_the_recorder(tmp_path):
    """The stack's teardown (SIGTERM): the recorder stops in the app's
    shutdown, before uvicorn re-raises the signal (review)."""
    import os
    import signal
    import socket
    import subprocess
    with socket.socket() as so:
        so.bind(("127.0.0.1", 0))
        port = so.getsockname()[1]
    env = dict(os.environ, FMEA_DIR=str(tmp_path), SIM_BODY_SHM=f"picar_3151_term_{os.getpid()}",
               PYTHONUNBUFFERED="1")
    p = subprocess.Popen([sys.executable, str(HERE / "truth_body.py"), str(port)], env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                         cwd=str(HERE.parents[1]))
    try:
        deadline = time.monotonic() + 60
        while not (tmp_path / "truth.jsonl").exists() or (tmp_path / "truth.jsonl").stat().st_size < 500:
            assert time.monotonic() < deadline and p.poll() is None
            time.sleep(0.2)
        p.send_signal(signal.SIGTERM)
        out, _ = p.communicate(timeout=20)
    finally:
        if p.poll() is None:
            p.kill()
    assert "truth recorder stopped: drained=True" in out
